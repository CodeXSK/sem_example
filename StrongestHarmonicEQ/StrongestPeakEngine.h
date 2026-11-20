#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

class StrongestPeakEngine
{
public:
    static constexpr int fftOrder = 12;
    static constexpr int fftSize  = 1 << fftOrder; // 4096
    static constexpr int hopSize  = 512;

    void prepare(float sampleRate)
    {
        // Keep normal host behaviour identical. Only replace an invalid or
        // nonsensical host sample-rate so it cannot poison filter maths.
        sampleRate_ = (isFiniteFloat(sampleRate) && sampleRate >= 1000.0f && sampleRate <= 768000.0f)
                        ? sampleRate
                        : 44100.0f;

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

        biquad_.reset();
        updateFilterCoefficients();
    }

    void setParameters(float boostDb,
                       float q,
                       float minFrequency,
                       float maxFrequency,
                       float trackingMs)
    {
        // std::clamp() does not sanitise NaN. Ignore invalid control values
        // and keep the last valid value instead. For all normal finite values
        // these expressions are exactly the same clamps as the original.
        if (isFiniteFloat(boostDb))
            boostDb_ = std::clamp(boostDb, -24.0f, 24.0f);

        if (isFiniteFloat(q))
            q_ = std::clamp(q, 0.30f, 12.0f);

        if (isFiniteFloat(minFrequency))
            minFrequency_ = std::clamp(minFrequency, 20.0f, 18000.0f);

        if (isFiniteFloat(maxFrequency))
            maxFrequency_ = std::clamp(maxFrequency, 20.0f, 18000.0f);

        if (minFrequency_ > maxFrequency_)
            std::swap(minFrequency_, maxFrequency_);

        if (isFiniteFloat(trackingMs))
            trackingMs_ = std::clamp(trackingMs, 5.0f, 500.0f);

        updateFilterCoefficients();
    }

    bool processSample(float input, float& output)
    {
        // IMPORTANT:
        // Analysis sees INPUT BEFORE EQ, preventing the boost from reinforcing
        // its own detector decision.
        // Real audio is left completely untouched. Only non-finite or
        // astronomically large values are contained so one corrupt upstream
        // sample cannot permanently poison the FFT/filter state.
        const float safeInput = sanitiseAudioSample(input);

        const bool newAnalysis = pushAnalysisSample(safeInput);

        output = biquad_.process(safeInput);

        if (!isFiniteFloat(output))
        {
            biquad_.clearState();
            output = 0.0f;
        }

        return newAnalysis;
    }

    float getDetectedHz() const noexcept { return detectedHz_; }
    float getTrackedHz() const noexcept  { return trackedHz_;  }

private:
    static constexpr float pi =
        3.14159265358979323846264338327950288f;
    static constexpr float twoPi = 2.0f * pi;

    // Bit-level finite test. This remains reliable even in builds that use
    // aggressive floating-point optimisation flags.
    static bool isFiniteFloat(float value) noexcept
    {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        return (bits & 0x7f800000u) != 0x7f800000u;
    }

    static float sanitiseAudioSample(float value) noexcept
    {
        if (!isFiniteFloat(value))
            return 0.0f;

        // Normal plugin audio is many orders of magnitude below this. This is
        // purely a catastrophe guard for corrupt/hostile upstream data.
        constexpr float absurdInput = 1.0e20f;
        if (value > absurdInput || value < -absurdInput)
            return 0.0f;

        return value;
    }

    class Biquad
    {
    public:
        void reset() noexcept
        {
            clearState();
            setBypass();
        }

        void clearState() noexcept
        {
            z1_ = 0.0f;
            z2_ = 0.0f;
        }

        void setBypass() noexcept
        {
            b0_ = 1.0f;
            b1_ = 0.0f;
            b2_ = 0.0f;
            a1_ = 0.0f;
            a2_ = 0.0f;
        }

        void setPeaking(float sampleRate,
                        float frequency,
                        float q,
                        float gainDb) noexcept
        {
            if (!StrongestPeakEngine::isFiniteFloat(sampleRate)
                || !StrongestPeakEngine::isFiniteFloat(frequency)
                || !StrongestPeakEngine::isFiniteFloat(q)
                || !StrongestPeakEngine::isFiniteFloat(gainDb)
                || sampleRate < 1000.0f)
            {
                setBypass();
                return;
            }

            const float maxFrequency =
                (std::max)(20.0f, sampleRate * 0.45f);

            frequency = std::clamp(frequency, 10.0f, maxFrequency);
            q = (std::max)(0.1f, q);
            gainDb = std::clamp(gainDb, -24.0f, 24.0f);

            // RBJ Audio EQ Cookbook peaking EQ.
            const float A = std::pow(10.0f, gainDb / 40.0f);
            const float w0 = twoPi * frequency / sampleRate;
            const float c = std::cos(w0);
            const float s = std::sin(w0);
            const float alpha = s / (2.0f * q);

            const float nb0 = 1.0f + alpha * A;
            const float nb1 = -2.0f * c;
            const float nb2 = 1.0f - alpha * A;
            const float na0 = 1.0f + alpha / A;
            const float na1 = -2.0f * c;
            const float na2 = 1.0f - alpha / A;

            const float invA0 = 1.0f / na0;

            const float newB0 = nb0 * invA0;
            const float newB1 = nb1 * invA0;
            const float newB2 = nb2 * invA0;
            const float newA1 = na1 * invA0;
            const float newA2 = na2 * invA0;

            if (!StrongestPeakEngine::isFiniteFloat(newB0)
                || !StrongestPeakEngine::isFiniteFloat(newB1)
                || !StrongestPeakEngine::isFiniteFloat(newB2)
                || !StrongestPeakEngine::isFiniteFloat(newA1)
                || !StrongestPeakEngine::isFiniteFloat(newA2))
            {
                setBypass();
                return;
            }

            b0_ = newB0;
            b1_ = newB1;
            b2_ = newB2;
            a1_ = newA1;
            a2_ = newA2;
        }

        float process(float x) noexcept
        {
            if (!StrongestPeakEngine::isFiniteFloat(x)
                || !StrongestPeakEngine::isFiniteFloat(z1_)
                || !StrongestPeakEngine::isFiniteFloat(z2_))
            {
                clearState();
                return 0.0f;
            }

            const float y = b0_ * x + z1_;
            const float newZ1 = b1_ * x - a1_ * y + z2_;
            const float newZ2 = b2_ * x - a2_ * y;

            if (!StrongestPeakEngine::isFiniteFloat(y)
                || !StrongestPeakEngine::isFiniteFloat(newZ1)
                || !StrongestPeakEngine::isFiniteFloat(newZ2))
            {
                clearState();
                return 0.0f;
            }

            z1_ = newZ1;
            z2_ = newZ2;

            if (std::abs(z1_) < 1.0e-20f) z1_ = 0.0f;
            if (std::abs(z2_) < 1.0e-20f) z2_ = 0.0f;

            return y;
        }

    private:
        float b0_ = 1.0f;
        float b1_ = 0.0f;
        float b2_ = 0.0f;
        float a1_ = 0.0f;
        float a2_ = 0.0f;
        float z1_ = 0.0f;
        float z2_ = 0.0f;
    };

    bool pushAnalysisSample(float sample)
    {
        ring_[static_cast<std::size_t>(writePosition_)] = sample;

        // fftSize is a power of two, so this is equivalent to modulo.
        writePosition_ = (writePosition_ + 1) & (fftSize - 1);

        samplesInRing_ = (std::min)(samplesInRing_ + 1, fftSize);
        ++samplesSinceFft_;

        if (samplesInRing_ < fftSize || samplesSinceFft_ < hopSize)
            return false;

        samplesSinceFft_ = 0;
        analyseFrame();
        return true;
    }

    void analyseFrame()
    {
        double sumSquares = 0.0;

        // Once the ring is full, writePosition_ points to the oldest sample.
        for (int i = 0; i < fftSize; ++i)
        {
            const int ringIndex =
                (writePosition_ + i) & (fftSize - 1);

            const float dry =
                ring_[static_cast<std::size_t>(ringIndex)];

            sumSquares +=
                static_cast<double>(dry) * static_cast<double>(dry);

            real_[static_cast<std::size_t>(i)] =
                dry * window_[static_cast<std::size_t>(i)];

            imag_[static_cast<std::size_t>(i)] = 0.0f;
        }

        const float rms =
            std::sqrt(static_cast<float>(
                sumSquares / static_cast<double>(fftSize)));

        // Very low threshold. This only avoids selecting numerical noise
        // during near-digital-silence.
        if (rms < 1.0e-7f)
        {
            detectedHz_ = 0.0f;
            return;
        }

        fftInPlace(real_, imag_);

        const float binHz =
            sampleRate_ / static_cast<float>(fftSize);

        const float safeMin =
            (std::max)(minFrequency_, binHz);

        const float safeMax =
            (std::min)(
                maxFrequency_,
                sampleRate_ * 0.5f - 2.0f * binHz);

        if (!isFiniteFloat(binHz) || binHz <= 0.0f
            || !isFiniteFloat(safeMin) || !isFiniteFloat(safeMax)
            || safeMax < safeMin)
        {
            detectedHz_ = 0.0f;
            return;
        }

        int minBin =
            static_cast<int>(std::ceil(safeMin / binHz));

        int maxBin =
            static_cast<int>(std::floor(safeMax / binHz));

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

        // Use magnitude (sqrt power) for parabolic interpolation.
        const float m1 =
            std::sqrt((std::max)(0.0f, powerAt(strongestBin - 1)));

        const float m2 =
            std::sqrt((std::max)(0.0f, powerAt(strongestBin)));

        const float m3 =
            std::sqrt((std::max)(0.0f, powerAt(strongestBin + 1)));

        const float denominator = m1 - 2.0f * m2 + m3;

        float delta = 0.0f;

        if (std::abs(denominator) > 1.0e-20f)
            delta = 0.5f * (m1 - m3) / denominator;

        delta = std::clamp(delta, -0.5f, 0.5f);

        detectedHz_ =
            (static_cast<float>(strongestBin) + delta) * binHz;

        if (!isFiniteFloat(detectedHz_) || detectedHz_ <= 0.0f)
        {
            detectedHz_ = 0.0f;
            return;
        }

        updateTrackedFrequency(detectedHz_);
        updateFilterCoefficients();
    }

    float powerAt(int bin) const noexcept
    {
        const float re = real_[static_cast<std::size_t>(bin)];
        const float im = imag_[static_cast<std::size_t>(bin)];
        return re * re + im * im;
    }

    void updateTrackedFrequency(float detected)
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

        const float alpha =
            std::exp(-hopSeconds / tauSeconds);

        // Smooth in log-frequency space, which is more natural for pitch.
        const float oldLog =
            std::log((std::max)(1.0f, trackedHz_));

        const float newLog =
            std::log((std::max)(1.0f, detected));

        trackedHz_ =
            std::exp(alpha * oldLog + (1.0f - alpha) * newLog);
    }

    void updateFilterCoefficients()
    {
        if (!(trackedHz_ > 0.0f) || std::abs(boostDb_) < 0.001f)
        {
            biquad_.setBypass();
            return;
        }

        biquad_.setPeaking(
            sampleRate_,
            trackedHz_,
            q_,
            boostDb_);
    }

    static void fftInPlace(std::array<float, fftSize>& real,
                           std::array<float, fftSize>& imag)
    {
        // Bit reversal.
        for (int i = 1, j = 0; i < fftSize; ++i)
        {
            int bit = fftSize >> 1;

            for (; j & bit; bit >>= 1)
                j ^= bit;

            j ^= bit;

            if (i < j)
            {
                std::swap(
                    real[static_cast<std::size_t>(i)],
                    real[static_cast<std::size_t>(j)]);

                std::swap(
                    imag[static_cast<std::size_t>(i)],
                    imag[static_cast<std::size_t>(j)]);
            }
        }

        // Iterative radix-2 Cooley-Tukey FFT.
        for (int length = 2; length <= fftSize; length <<= 1)
        {
            const float angle =
                -twoPi / static_cast<float>(length);

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

                    const float evenReal =
                        real[static_cast<std::size_t>(even)];

                    const float evenImag =
                        imag[static_cast<std::size_t>(even)];

                    real[static_cast<std::size_t>(even)] =
                        evenReal + oddReal;

                    imag[static_cast<std::size_t>(even)] =
                        evenImag + oddImag;

                    real[static_cast<std::size_t>(odd)] =
                        evenReal - oddReal;

                    imag[static_cast<std::size_t>(odd)] =
                        evenImag - oddImag;

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

    float boostDb_ = 6.0f;
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

    Biquad biquad_;
};
