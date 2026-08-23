#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

// =============================================================================
// StrongestPeakEngine - RELEASE-SAFE revision
//
// Main safety properties:
// - No dynamic allocation in processSample()/FFT path.
// - All user parameters are finite-checked and hard-clamped.
// - Non-finite or absurdly large audio samples are sanitized before they can
//   poison FFT/filter state.
// - Analysis is always PRE-EQ (no detector self-locking).
// - Dynamic EQ uses a topology-preserving-transform (TPT) state-variable
//   peaking structure instead of abruptly retuning a Direct-Form biquad.
//   This is substantially safer for fast/high-Q frequency tracking.
// - Internal filter parameters are smoothed sample-by-sample.
// - Invalid/empty analysis ranges fail safely to dry/bypass.
// - Detector and filter share the same <= 0.45 * sample-rate ceiling.
// =============================================================================
class StrongestPeakEngine
{
public:
    static constexpr int fftOrder = 12;
    static constexpr int fftSize  = 1 << fftOrder; // 4096
    static constexpr int hopSize  = 512;

    void prepare(float sampleRate)
    {
        if (!isFiniteFloat(sampleRate) || sampleRate < 1000.0f || sampleRate > 768000.0f)
            sampleRate = 44100.0f;

        sampleRate_ = sampleRate;

        ring_.fill(0.0f);
        window_.fill(0.0f);
        real_.fill(0.0f);
        imag_.fill(0.0f);

        for (int n = 0; n < fftSize; ++n)
        {
            window_[static_cast<std::size_t>(n)] =
                0.5f - 0.5f * std::cos(
                    twoPi * static_cast<float>(n)
                    / static_cast<float>(fftSize - 1));
        }

        writePosition_ = 0;
        samplesInRing_ = 0;
        samplesSinceFft_ = 0;

        detectedHz_ = 0.0f;
        trackedHz_ = 0.0f;

        peak_.prepare(sampleRate_);
        updateFilterTarget();
    }

    void setParameters(float gainDb,
                       float q,
                       float minFrequency,
                       float maxFrequency,
                       float trackingMs)
    {
        // std::clamp does not repair NaN. Sanitize first, then clamp.
        gainDb       = finiteOr(gainDb,       gainDb_);
        q            = finiteOr(q,            q_);
        minFrequency = finiteOr(minFrequency, minFrequency_);
        maxFrequency = finiteOr(maxFrequency, maxFrequency_);
        trackingMs   = finiteOr(trackingMs,   trackingMs_);

        gainDb_ = std::clamp(gainDb, -24.0f, 24.0f);
        q_ = std::clamp(q, 0.30f, 12.0f);

        minFrequency_ = std::clamp(minFrequency, 20.0f, 18000.0f);
        maxFrequency_ = std::clamp(maxFrequency, 20.0f, 18000.0f);

        if (minFrequency_ > maxFrequency_)
            std::swap(minFrequency_, maxFrequency_);

        trackingMs_ = std::clamp(trackingMs, 5.0f, 500.0f);

        updateFilterTarget();
    }

    bool processSample(float input, float& output) noexcept
    {
        // Protect the module from upstream NaN/Inf and pathological values.
        // +/- 1e6 is roughly +/-120 dBFS amplitude, far beyond meaningful DAW
        // audio, while keeping FFT arithmetic many orders below float overflow.
        const float safeInput = sanitizeAudio(input);

        // Analysis sees input BEFORE EQ.
        const bool newAnalysis = pushAnalysisSample(safeInput);

        output = peak_.process(safeInput);

        // Last-resort containment. A single invalid upstream sample must never
        // permanently poison this module or the rest of the graph.
        if (!isFiniteFloat(output))
        {
            peak_.resetState();
            output = 0.0f;
        }
        else
        {
            // This is NOT an audible limiter. The ceiling is intentionally
            // absurdly high (~+160 dBFS amplitude) and only exists to prevent
            // a catastrophic finite value from being passed downstream.
            output = std::clamp(output, -maxSafeOutput, maxSafeOutput);
        }

        return newAnalysis;
    }

    float getDetectedHz() const noexcept { return detectedHz_; }

    // Target/smoothed detector frequency.
    float getTrackedHz() const noexcept { return trackedHz_; }

    // Actual centre frequency currently used by the sample-smoothed TPT EQ.
    float getEqHz() const noexcept
    {
        return trackedHz_ > 0.0f ? peak_.getCurrentFrequencyHz() : 0.0f;
    }

private:
    static constexpr float pi =
        3.14159265358979323846264338327950288f;
    static constexpr float twoPi = 2.0f * pi;
    static constexpr float maxSafeAudio  = 1000000.0f;
    static constexpr float maxSafeOutput = 100000000.0f;

    static bool isFiniteFloat(float value) noexcept
    {
        // Bit-level IEEE-754 finite test. This deliberately does not rely on
        // compiler floating-point assumptions, so it remains trustworthy even
        // if a host/example project was configured with aggressive FP flags.
        std::uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        return (bits & 0x7f800000u) != 0x7f800000u;
    }

    static bool isSubnormalFloat(float value) noexcept
    {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));

        const auto exponent = bits & 0x7f800000u;
        const auto mantissa = bits & 0x007fffffu;
        return exponent == 0u && mantissa != 0u;
    }

    static float finiteOr(float value, float fallback) noexcept
    {
        return isFiniteFloat(value) ? value : fallback;
    }

    static float sanitizeAudio(float x) noexcept
    {
        if (!isFiniteFloat(x))
            return 0.0f;

        // Subnormal input is many hundreds of dB below any useful audio level
        // and can cause needless CPU penalties on some processors.
        if (isSubnormalFloat(x))
            return 0.0f;

        return std::clamp(x, -maxSafeAudio, maxSafeAudio);
    }

    // -------------------------------------------------------------------------
    // DynamicTptPeak
    //
    // TPT state-variable filter. The normalized band-pass component has unity
    // gain at Fc. Therefore:
    //
    //     y = dry + (linearGain - 1) * normalizedBand
    //
    // gives unity far from Fc and the requested +/- dB gain at Fc.
    // Unlike an abruptly retuned Direct-Form biquad, the TPT states remain
    // well-behaved under rapid cutoff modulation.
    // -------------------------------------------------------------------------
    class DynamicTptPeak
    {
    public:
        void prepare(float sampleRate) noexcept
        {
            sampleRate_ = sampleRate;
            resetState();

            currentG_ = targetG_ = frequencyToG(1000.0f);
            currentK_ = targetK_ = 1.0f / 3.0f;
            currentGain_ = targetGain_ = 1.0f;

            // Small extra smoothing is deliberately separate from detector
            // Tracking. It prevents coefficient/control discontinuities.
            const float timeSeconds = 0.003f; // 3 ms
            smoothingStep_ = 1.0f - std::exp(-1.0f / (timeSeconds * sampleRate_));
            smoothingStep_ = std::clamp(smoothingStep_, 0.0f, 1.0f);
        }

        void resetState() noexcept
        {
            state1_ = 0.0f;
            state2_ = 0.0f;
        }

        void setTarget(float frequency, float q, float gainDb) noexcept
        {
            if (!isFiniteFloat(frequency) || !isFiniteFloat(q) || !isFiniteFloat(gainDb))
            {
                setBypassTarget();
                return;
            }

            const float maxF = (std::max)(10.0f, sampleRate_ * 0.45f);
            frequency = std::clamp(frequency, 10.0f, maxF);
            q = std::clamp(q, 0.30f, 12.0f);
            gainDb = std::clamp(gainDb, -24.0f, 24.0f);

            targetG_ = frequencyToG(frequency);
            targetK_ = 1.0f / q;
            targetGain_ = std::pow(10.0f, gainDb / 20.0f);

            if (!isFiniteFloat(targetG_) || !isFiniteFloat(targetK_) || !isFiniteFloat(targetGain_))
            {
                setBypassTarget();
            }
        }

        void setBypassTarget() noexcept
        {
            targetGain_ = 1.0f;
        }

        float process(float x) noexcept
        {
            smoothParameters();

            // Stable TPT SVF equations.
            const float denom = 1.0f + currentG_ * (currentG_ + currentK_);

            if (!(denom > 0.0f) || !isFiniteFloat(denom))
            {
                resetState();
                return x;
            }

            const float a1 = 1.0f / denom;
            const float a2 = currentG_ * a1;
            const float a3 = currentG_ * a2;

            const float v3 = x - state2_;
            const float v1 = a1 * state1_ + a2 * v3;
            const float v2 = state2_ + a2 * state1_ + a3 * v3;

            state1_ = 2.0f * v1 - state1_;
            state2_ = 2.0f * v2 - state2_;

            // Prevent denormal CPU stalls on silence/tails.
            if (std::abs(state1_) < 1.0e-20f) state1_ = 0.0f;
            if (std::abs(state2_) < 1.0e-20f) state2_ = 0.0f;

            if (!isFiniteFloat(state1_) || !isFiniteFloat(state2_))
            {
                resetState();
                return 0.0f;
            }

            // Raw v1 peaks at Q, so K*v1 is unity at centre frequency.
            const float normalizedBand = currentK_ * v1;
            const float y = x + (currentGain_ - 1.0f) * normalizedBand;

            return isFiniteFloat(y) ? y : 0.0f;
        }

        float getCurrentFrequencyHz() const noexcept
        {
            if (!(currentG_ >= 0.0f) || !isFiniteFloat(currentG_))
                return 0.0f;

            return std::atan(currentG_) * sampleRate_ / pi;
        }

    private:
        float frequencyToG(float frequency) const noexcept
        {
            const float maxF = (std::max)(10.0f, sampleRate_ * 0.45f);
            frequency = std::clamp(frequency, 10.0f, maxF);
            return std::tan(pi * frequency / sampleRate_);
        }

        void smoothParameters() noexcept
        {
            currentG_    += smoothingStep_ * (targetG_    - currentG_);
            currentK_    += smoothingStep_ * (targetK_    - currentK_);
            currentGain_ += smoothingStep_ * (targetGain_ - currentGain_);

            // Numerical guardrails; these ranges are much wider than normal.
            currentG_ = std::clamp(currentG_, 0.0f, 10.0f);
            currentK_ = std::clamp(currentK_, 1.0f / 12.0f, 1.0f / 0.30f);
            currentGain_ = std::clamp(currentGain_,
                                      0.05f,
                                      20.0f);
        }

        float sampleRate_ = 44100.0f;
        float smoothingStep_ = 1.0f;

        float targetG_ = 0.0f;
        float targetK_ = 1.0f / 3.0f;
        float targetGain_ = 1.0f;

        float currentG_ = 0.0f;
        float currentK_ = 1.0f / 3.0f;
        float currentGain_ = 1.0f;

        float state1_ = 0.0f;
        float state2_ = 0.0f;
    };

    bool pushAnalysisSample(float sample) noexcept
    {
        ring_[static_cast<std::size_t>(writePosition_)] = sample;
        writePosition_ = (writePosition_ + 1) & (fftSize - 1);

        samplesInRing_ = (std::min)(samplesInRing_ + 1, fftSize);
        ++samplesSinceFft_;

        if (samplesInRing_ < fftSize || samplesSinceFft_ < hopSize)
            return false;

        samplesSinceFft_ = 0;
        analyseFrame();
        return true;
    }

    void setNoDetection() noexcept
    {
        detectedHz_ = 0.0f;
        trackedHz_ = 0.0f;
        peak_.setBypassTarget();
    }

    void analyseFrame() noexcept
    {
        double sum = 0.0;
        double sumSquares = 0.0;

        // First reconstruct the chronological frame. Keep it unwindowed long
        // enough to estimate and remove DC. Without this, a large DC offset can
        // leak through the Hann window and masquerade as a low-frequency peak.
        for (int i = 0; i < fftSize; ++i)
        {
            const int ringIndex = (writePosition_ + i) & (fftSize - 1);
            const float dry = ring_[static_cast<std::size_t>(ringIndex)];

            real_[static_cast<std::size_t>(i)] = dry;
            imag_[static_cast<std::size_t>(i)] = 0.0f;

            sum += static_cast<double>(dry);
            sumSquares += static_cast<double>(dry) * static_cast<double>(dry);
        }

        const double invN = 1.0 / static_cast<double>(fftSize);
        const double mean = sum * invN;

        // AC variance = E[x^2] - E[x]^2. Roundoff can make an exact-DC frame
        // microscopically negative, so clamp only that tiny numerical residue.
        const double meanSquare = sumSquares * invN;
        const double variance = (std::max)(0.0, meanSquare - mean * mean);
        const float acRms = static_cast<float>(std::sqrt(variance));

        if (!isFiniteFloat(acRms) || acRms < 1.0e-7f)
        {
            setNoDetection();
            return;
        }

        const float meanF = static_cast<float>(mean);

        for (int i = 0; i < fftSize; ++i)
        {
            real_[static_cast<std::size_t>(i)] =
                (real_[static_cast<std::size_t>(i)] - meanF)
                * window_[static_cast<std::size_t>(i)];
        }

        fftInPlace(real_, imag_);

        const float binHz = sampleRate_ / static_cast<float>(fftSize);

        const float safeMin = (std::max)(minFrequency_, binHz);
        const float safeMax = (std::min)(
            (std::min)(maxFrequency_, sampleRate_ * 0.45f),
            sampleRate_ * 0.5f - 2.0f * binHz);

        // No legal FFT search region at this sample rate/settings.
        if (!isFiniteFloat(safeMin) || !isFiniteFloat(safeMax)
            || safeMax <= safeMin)
        {
            setNoDetection();
            return;
        }

        int minBin = static_cast<int>(std::ceil(safeMin / binHz));
        int maxBin = static_cast<int>(std::floor(safeMax / binHz));

        minBin = std::clamp(minBin, 1, fftSize / 2 - 2);
        maxBin = std::clamp(maxBin, minBin, fftSize / 2 - 2);

        int strongestBin = minBin;
        float strongestPower = 0.0f;
        bool foundLocalPeak = false;

        for (int bin = minBin; bin <= maxBin; ++bin)
        {
            const float pL = powerAt(bin - 1);
            const float pC = powerAt(bin);
            const float pR = powerAt(bin + 1);

            if (pC >= pL && pC >= pR && pC > strongestPower)
            {
                strongestPower = pC;
                strongestBin = bin;
                foundLocalPeak = true;
            }
        }

        if (!foundLocalPeak)
        {
            for (int bin = minBin; bin <= maxBin; ++bin)
            {
                const float p = powerAt(bin);

                if (p > strongestPower)
                {
                    strongestPower = p;
                    strongestBin = bin;
                }
            }
        }

        if (!(strongestPower > 0.0f) || !isFiniteFloat(strongestPower))
        {
            setNoDetection();
            return;
        }

        const float m1 = std::sqrt((std::max)(0.0f, powerAt(strongestBin - 1)));
        const float m2 = std::sqrt((std::max)(0.0f, powerAt(strongestBin)));
        const float m3 = std::sqrt((std::max)(0.0f, powerAt(strongestBin + 1)));

        const float denominator = m1 - 2.0f * m2 + m3;

        float delta = 0.0f;
        if (isFiniteFloat(denominator) && std::abs(denominator) > 1.0e-20f)
            delta = 0.5f * (m1 - m3) / denominator;

        if (!isFiniteFloat(delta))
            delta = 0.0f;

        delta = std::clamp(delta, -0.5f, 0.5f);

        const float detected =
            (static_cast<float>(strongestBin) + delta) * binHz;

        if (!(detected > 0.0f) || !isFiniteFloat(detected))
        {
            setNoDetection();
            return;
        }

        detectedHz_ = detected;
        updateTrackedFrequency(detectedHz_);
        updateFilterTarget();
    }

    float powerAt(int bin) const noexcept
    {
        const float re = real_[static_cast<std::size_t>(bin)];
        const float im = imag_[static_cast<std::size_t>(bin)];
        return re * re + im * im;
    }

    void updateTrackedFrequency(float detected) noexcept
    {
        if (!(detected > 0.0f) || !isFiniteFloat(detected))
            return;

        if (!(trackedHz_ > 0.0f) || !isFiniteFloat(trackedHz_))
        {
            trackedHz_ = detected;
            return;
        }

        const float tauSeconds =
            (std::max)(0.001f, trackingMs_ * 0.001f);

        const float hopSeconds =
            static_cast<float>(hopSize) / sampleRate_;

        const float alpha = std::exp(-hopSeconds / tauSeconds);

        const float oldLog = std::log((std::max)(1.0f, trackedHz_));
        const float newLog = std::log((std::max)(1.0f, detected));

        const float result =
            std::exp(alpha * oldLog + (1.0f - alpha) * newLog);

        if (isFiniteFloat(result) && result > 0.0f)
            trackedHz_ = result;
    }

    void updateFilterTarget() noexcept
    {
        if (!(trackedHz_ > 0.0f) || !isFiniteFloat(trackedHz_))
        {
            peak_.setBypassTarget();
            return;
        }

        // Even at 0 dB, keep frequency/Q tracking internally. Output remains
        // mathematically dry because linearGain == 1, but re-enabling boost/cut
        // no longer starts from a stale filter frequency/state.
        peak_.setTarget(trackedHz_, q_, gainDb_);
    }

    static void fftInPlace(std::array<float, fftSize>& real,
                           std::array<float, fftSize>& imag) noexcept
    {
        for (int i = 1, j = 0; i < fftSize; ++i)
        {
            int bit = fftSize >> 1;

            for (; j & bit; bit >>= 1)
                j ^= bit;

            j ^= bit;

            if (i < j)
            {
                std::swap(real[static_cast<std::size_t>(i)],
                          real[static_cast<std::size_t>(j)]);
                std::swap(imag[static_cast<std::size_t>(i)],
                          imag[static_cast<std::size_t>(j)]);
            }
        }

        for (int length = 2; length <= fftSize; length <<= 1)
        {
            const float angle = -twoPi / static_cast<float>(length);
            const float stepReal = std::cos(angle);
            const float stepImag = std::sin(angle);
            const int half = length >> 1;

            for (int base = 0; base < fftSize; base += length)
            {
                float wReal = 1.0f;
                float wImag = 0.0f;

                for (int j = 0; j < half; ++j)
                {
                    const int even = base + j;
                    const int odd  = even + half;

                    const float oddReal =
                        real[static_cast<std::size_t>(odd)] * wReal
                      - imag[static_cast<std::size_t>(odd)] * wImag;

                    const float oddImag =
                        real[static_cast<std::size_t>(odd)] * wImag
                      + imag[static_cast<std::size_t>(odd)] * wReal;

                    const float evenReal = real[static_cast<std::size_t>(even)];
                    const float evenImag = imag[static_cast<std::size_t>(even)];

                    real[static_cast<std::size_t>(even)] = evenReal + oddReal;
                    imag[static_cast<std::size_t>(even)] = evenImag + oddImag;
                    real[static_cast<std::size_t>(odd)]  = evenReal - oddReal;
                    imag[static_cast<std::size_t>(odd)]  = evenImag - oddImag;

                    const float nextReal =
                        wReal * stepReal - wImag * stepImag;
                    const float nextImag =
                        wReal * stepImag + wImag * stepReal;

                    wReal = nextReal;
                    wImag = nextImag;
                }
            }
        }
    }

    float sampleRate_ = 44100.0f;

    float gainDb_ = 6.0f;
    float q_ = 3.0f;
    float minFrequency_ = 40.0f;
    float maxFrequency_ = 8000.0f;
    float trackingMs_ = 70.0f;

    std::array<float, fftSize> ring_{};
    std::array<float, fftSize> window_{};
    std::array<float, fftSize> real_{};
    std::array<float, fftSize> imag_{};

    int writePosition_ = 0;
    int samplesInRing_ = 0;
    int samplesSinceFft_ = 0;

    float detectedHz_ = 0.0f;
    float trackedHz_ = 0.0f;

    DynamicTptPeak peak_;
};
