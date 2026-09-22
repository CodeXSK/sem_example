#include "mp_sdk_audio.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <vector>

using namespace gmpi;

namespace
{
    constexpr int   kLines = 8; // maximum. User-selectable active count: 4, 6, or 8.
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kInvSqrt4 = 0.5f;
    constexpr float kInvSqrt6 = 0.40824829046386301636f;
    constexpr float kInvSqrt8 = 0.35355339059327376220f;
    constexpr float kInternalToDisplayedUnits = 10.0f;

    constexpr float kMaxSupportedSampleRate = 384000.0f;
    constexpr float kMaxDelayMs = 1000.0f;
    constexpr float kMinSize = 0.25f;
    constexpr float kMaxSize = 4.0f;
    constexpr float kMinRt60 = 0.10f;
    constexpr float kMaxRt60 = 120.0f;
    constexpr float kMaxRandomOffsetMs = 10.0f;
    constexpr float kMaxModDepthMs = 2.0f;
    constexpr float kMinModRateHz = 0.001f;
    constexpr float kMaxModRateHz = 2.0f;
    constexpr float kMinDecayRatio = 0.10f;
    constexpr float kMaxDecayRatio = 3.0f;
    constexpr float kMinCrossoverHz = 20.0f;
    constexpr float kMaxLowCrossoverHz = 2000.0f;
    constexpr float kMinHighCrossoverHz = 1000.0f;
    constexpr float kMaxHighCrossoverHz = 30000.0f;
    constexpr float kMaxStereoWidth = 2.0f;

    // Optional scattering all-pass placed inside each active FDN line.
    // Small delays deliberately avoid turning the tank into a second long echo network.
    constexpr float kMaxInternalAllPassDelayMs = 50.0f;
    constexpr float kMaxInternalAllPassG = 0.55f;
    constexpr float kMinInternalAllPassSize = 0.25f;
    constexpr float kMaxInternalAllPassSize = 4.0f;

    constexpr float kParameterSmoothingMs = 25.0f;
    constexpr float kDelaySmoothingMs = 80.0f;
    constexpr float kFreezeSmoothingMs = 60.0f;
    constexpr float kResetFadeMs = 20.0f;

    constexpr int kDelayBufferSamples =
        static_cast<int>(kMaxSupportedSampleRate * (kMaxDelayMs * 0.001f)) + 8;

    constexpr int kAllPassDelayBufferSamples =
        static_cast<int>(kMaxSupportedSampleRate * (kMaxInternalAllPassDelayMs * 0.001f)) + 8;

    inline float clampf(float x, float lo, float hi)
    {
        return (std::max)(lo, (std::min)(hi, x));
    }

    inline float finiteOrZero(float x)
    {
        return std::isfinite(x) ? x : 0.0f;
    }

    inline float zapDenormal(float x)
    {
        return std::abs(x) < 1.0e-30f ? 0.0f : x;
    }

    inline float safetyLimit(float x)
    {
        x = finiteOrZero(x);
        // This is intentionally enormous compared with normal SynthEdit levels.
        // It only prevents NaN/Inf cascades after accidental unstable automation.
        return clampf(x, -1000.0f, 1000.0f);
    }

    class SmoothedValue
    {
    public:
        void reset(float value)
        {
            current_ = target_ = value;
        }

        void setTime(float timeMs, float sampleRate)
        {
            const float seconds = (std::max)(0.0001f, timeMs * 0.001f);
            coefficient_ = std::exp(-1.0f / (seconds * sampleRate));
        }

        void setTarget(float target)
        {
            target_ = target;
        }

        float next()
        {
            current_ = target_ + coefficient_ * (current_ - target_);
            current_ = zapDenormal(current_);
            return current_;
        }

        float current() const { return current_; }

    private:
        float current_ = 0.0f;
        float target_ = 0.0f;
        float coefficient_ = 0.0f;
    };

    class FractionalDelay
    {
    public:
        FractionalDelay()
            : buffer_(static_cast<std::size_t>(kDelayBufferSamples), 0.0f)
        {
        }

        void clear()
        {
            std::fill(buffer_.begin(), buffer_.end(), 0.0f);
            writeIndex_ = 0;
            fractionalInputPrev_ = 0.0f;
            fractionalOutputPrev_ = 0.0f;
        }

        float read(float delaySamples)
        {
            // v2: first-order Thiran all-pass fractional-delay interpolation,
            // matching the approach used by Multi AllPass Diffuser v9.
            // For a static fractional delay its magnitude is unity, unlike
            // linear interpolation which introduces a small HF loss in every pass.
            // This matters especially inside an FDN because the interpolated signal
            // circulates through the feedback matrix many times.
            const float maxDelay = static_cast<float>(kDelayBufferSamples - 4);
            delaySamples = clampf(finiteOrZero(delaySamples), 1.0f, maxDelay);

            int integerDelay = static_cast<int>(std::floor(delaySamples));
            float fraction = delaySamples - static_cast<float>(integerDelay);
            integerDelay = (std::max)(1, (std::min)(integerDelay, kDelayBufferSamples - 4));
            fraction = clampf(fraction, 0.0f, 0.999999f);

            int readIndex = writeIndex_ - integerDelay;
            while (readIndex < 0)
                readIndex += kDelayBufferSamples;
            if (readIndex >= kDelayBufferSamples)
                readIndex %= kDelayBufferSamples;

            const float integerDelayed =
                finiteOrZero(buffer_[static_cast<std::size_t>(readIndex)]);

            float delayed = integerDelayed;

            // Hfrac(z) = (a + z^-1) / (1 + a z^-1)
            // a = (1-d)/(1+d), d in [0,1).
            if (fraction > 1.0e-5f)
            {
                const float a = (1.0f - fraction) / (1.0f + fraction);
                delayed =
                    a * integerDelayed
                    + fractionalInputPrev_
                    - a * fractionalOutputPrev_;

                delayed = zapDenormal(finiteOrZero(delayed));
                fractionalInputPrev_ = integerDelayed;
                fractionalOutputPrev_ = delayed;
            }
            else
            {
                // Exact integer tap: bypass the fractional all-pass and keep its
                // history coherent. This is also the path used by full Freeze.
                fractionalInputPrev_ = integerDelayed;
                fractionalOutputPrev_ = integerDelayed;
            }

            return zapDenormal(delayed);
        }

        void write(float input)
        {
            buffer_[static_cast<std::size_t>(writeIndex_)] = zapDenormal(safetyLimit(input));
            ++writeIndex_;
            if (writeIndex_ >= kDelayBufferSamples)
                writeIndex_ = 0;
        }

    private:
        std::vector<float> buffer_;
        int writeIndex_ = 0;
        float fractionalInputPrev_ = 0.0f;
        float fractionalOutputPrev_ = 0.0f;
    };


    class SmallFractionalDelay
    {
    public:
        SmallFractionalDelay()
            : buffer_(static_cast<std::size_t>(kAllPassDelayBufferSamples), 0.0f)
        {
        }

        void clear()
        {
            std::fill(buffer_.begin(), buffer_.end(), 0.0f);
            writeIndex_ = 0;
            fractionalInputPrev_ = 0.0f;
            fractionalOutputPrev_ = 0.0f;
        }

        float read(float delaySamples)
        {
            const float maxDelay = static_cast<float>(kAllPassDelayBufferSamples - 4);
            delaySamples = clampf(finiteOrZero(delaySamples), 1.0f, maxDelay);

            int integerDelay = static_cast<int>(std::floor(delaySamples));
            float fraction = delaySamples - static_cast<float>(integerDelay);
            integerDelay = (std::max)(1, (std::min)(integerDelay, kAllPassDelayBufferSamples - 4));
            fraction = clampf(fraction, 0.0f, 0.999999f);

            int readIndex = writeIndex_ - integerDelay;
            while (readIndex < 0)
                readIndex += kAllPassDelayBufferSamples;
            if (readIndex >= kAllPassDelayBufferSamples)
                readIndex %= kAllPassDelayBufferSamples;

            const float integerDelayed =
                finiteOrZero(buffer_[static_cast<std::size_t>(readIndex)]);

            float delayed = integerDelayed;
            if (fraction > 1.0e-5f)
            {
                const float a = (1.0f - fraction) / (1.0f + fraction);
                delayed =
                    a * integerDelayed
                    + fractionalInputPrev_
                    - a * fractionalOutputPrev_;

                delayed = zapDenormal(finiteOrZero(delayed));
                fractionalInputPrev_ = integerDelayed;
                fractionalOutputPrev_ = delayed;
            }
            else
            {
                fractionalInputPrev_ = integerDelayed;
                fractionalOutputPrev_ = integerDelayed;
            }

            return zapDenormal(delayed);
        }

        void write(float input)
        {
            buffer_[static_cast<std::size_t>(writeIndex_)] =
                zapDenormal(safetyLimit(input));

            ++writeIndex_;
            if (writeIndex_ >= kAllPassDelayBufferSamples)
                writeIndex_ = 0;
        }

    private:
        std::vector<float> buffer_;
        int writeIndex_ = 0;
        float fractionalInputPrev_ = 0.0f;
        float fractionalOutputPrev_ = 0.0f;
    };

    class ScatteringAllPass
    {
    public:
        void clear()
        {
            delay_.clear();
        }

        float process(float input, float delaySamples, float g)
        {
            input = finiteOrZero(input);
            g = clampf(finiteOrZero(g), -kMaxInternalAllPassG, kMaxInternalAllPassG);

            // Canonical Schroeder all-pass:
            //   y = delayed - g*x
            //   delay.write(x + g*y)
            // H(z) = (z^-D - g) / (1 - g*z^-D)
            const float delayed = delay_.read(delaySamples);
            const float output = zapDenormal(safetyLimit(delayed - g * input));
            delay_.write(input + g * output);
            return output;
        }

    private:
        SmallFractionalDelay delay_;
    };

    class ThreeBandDecayState
    {
    public:
        void clear()
        {
            lowState_ = 0.0f;
            highState_ = 0.0f;
        }

        float process(
            float input,
            float lowCoeff,
            float highCoeff,
            float lowGain,
            float midGain,
            float highGain)
        {
            input = finiteOrZero(input);

            lowState_ += lowCoeff * (input - lowState_);
            highState_ += highCoeff * (input - highState_);
            lowState_ = zapDenormal(finiteOrZero(lowState_));
            highState_ = zapDenormal(finiteOrZero(highState_));

            const float low = lowState_;
            const float mid = highState_ - lowState_;
            const float high = input - highState_;

            return zapDenormal(safetyLimit(
                low * lowGain + mid * midGain + high * highGain));
        }

    private:
        float lowState_ = 0.0f;
        float highState_ = 0.0f;
    };

    inline float onePoleCoeff(float cutoffHz, float sampleRate)
    {
        const float nyquistSafe = 0.45f * sampleRate;
        const float fc = clampf(cutoffHz, 5.0f, nyquistSafe);
        return 1.0f - std::exp(-2.0f * kPi * fc / sampleRate);
    }

    inline float rt60Gain(float delaySeconds, float rt60Seconds)
    {
        rt60Seconds = clampf(rt60Seconds, kMinRt60, kMaxRt60 * kMaxDecayRatio);
        delaySeconds = (std::max)(1.0e-6f, delaySeconds);
        const float g = std::pow(10.0f, -3.0f * delaySeconds / rt60Seconds);
        return clampf(finiteOrZero(g), 0.0f, 0.9999995f);
    }

    inline std::uint32_t xorshift32(std::uint32_t& state)
    {
        if (state == 0)
            state = 0x6d2b79f5u;
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    inline float randomUnit(std::uint32_t& state)
    {
        const std::uint32_t v = xorshift32(state);
        return static_cast<float>(v & 0x00ffffffu) / 16777215.0f;
    }

    inline float randomBipolar(std::uint32_t& state)
    {
        return randomUnit(state) * 2.0f - 1.0f;
    }

    inline float inverseSqrtForLines(int lines)
    {
        if (lines <= 4)
            return kInvSqrt4;
        if (lines <= 6)
            return kInvSqrt6;
        return kInvSqrt8;
    }

    inline void hadamardPowerOfTwo(
        const std::array<float, kLines>& in,
        std::array<float, kLines>& out,
        int activeLines)
    {
        std::array<float, kLines> x{};
        for (int i = 0; i < activeLines; ++i)
            x[static_cast<std::size_t>(i)] = in[static_cast<std::size_t>(i)];

        for (int step = 1; step < activeLines; step <<= 1)
        {
            for (int i = 0; i < activeLines; i += (step << 1))
            {
                for (int j = 0; j < step; ++j)
                {
                    const float a = x[static_cast<std::size_t>(i + j)];
                    const float b = x[static_cast<std::size_t>(i + j + step)];
                    x[static_cast<std::size_t>(i + j)] = a + b;
                    x[static_cast<std::size_t>(i + j + step)] = a - b;
                }
            }
        }

        out.fill(0.0f);
        const float norm = inverseSqrtForLines(activeLines);
        for (int i = 0; i < activeLines; ++i)
            out[static_cast<std::size_t>(i)] =
                x[static_cast<std::size_t>(i)] * norm;
    }

    inline void orthogonalMix(
        const std::array<float, kLines>& in,
        std::array<float, kLines>& out,
        int activeLines)
    {
        // 4 and 8 lines use normalized Hadamard.
        if (activeLines == 4 || activeLines == 8)
        {
            hadamardPowerOfTwo(in, out, activeLines);
            return;
        }

        // No real Hadamard matrix exists at order 6. For six lines use an
        // orthonormal DCT-II matrix instead. It is dense, energy-preserving,
        // and therefore suitable as the FDN feedback mixing matrix.
        out.fill(0.0f);
        constexpr int n = 6;
        for (int k = 0; k < n; ++k)
        {
            float sum = 0.0f;
            for (int i = 0; i < n; ++i)
            {
                const float angle =
                    (kPi / static_cast<float>(n)) *
                    (static_cast<float>(i) + 0.5f) *
                    static_cast<float>(k);
                sum += in[static_cast<std::size_t>(i)] * std::cos(angle);
            }

            const float norm = (k == 0)
                ? std::sqrt(1.0f / static_cast<float>(n))
                : std::sqrt(2.0f / static_cast<float>(n));

            out[static_cast<std::size_t>(k)] = sum * norm;
        }
    }
}

// -----------------------------------------------------------------------------
// Pandocrator FDN Reverb Tank v3
// -----------------------------------------------------------------------------
// Four, six, or eight active delay lines form a Feedback Delay Network.
// 4/8-line modes use normalized Hadamard feedback mixing; 6-line mode uses
// an orthonormal 6x6 DCT matrix because a Hadamard matrix does not exist at
// order 6. The user can optionally place one short Schroeder all-pass inside
// every active FDN line for extra echo-density/scattering before the damping
// and orthogonal feedback matrix.
//
// This module is intentionally a TANK, not an input diffuser. For a complete
// reverb, feed it from a separate early/input diffuser (for example the user's
// Multi AllPass Diffuser v7/v8 with its global Feedback set to 0).
//
// Decay is specified as RT60 seconds rather than a raw feedback coefficient:
//      g_i = 10 ^ (-3 * delay_i_seconds / RT60)
// This keeps differently sized delay lines on approximately the same decay law.
//
// Freeze convention:
//   - new input injection fades to zero
//   - damping/decay gains fade to unity
//   - modulation fades to zero
//   - at full freeze delay reads are rounded to integer samples
// Thus the full-freeze loop becomes approximately lossless: integer delays plus
// an orthogonal Hadamard matrix. This is much safer for very long/infinite holds
// than simply pushing an ordinary feedback knob above unity.
class FDNReverbTank final : public MpBase2
{
public:
    FDNReverbTank()
    {
        // Pin order MUST match FDNReverbTank.xml.
        initializePin(pinInputL);
        initializePin(pinInputR);
        initializePin(pinRt60Seconds);
        initializePin(pinMasterSize);
        initializePin(pinLfDecayRatio);
        initializePin(pinHfDecayRatio);
        initializePin(pinLowCrossoverHz);
        initializePin(pinHighCrossoverHz);
        initializePin(pinModDepthMs);
        initializePin(pinModRateHz);
        initializePin(pinStereoWidth);
        initializePin(pinFreeze);
        initializePin(pinRandomOffsetMs);
        initializePin(pinRandomSeed);
        initializePin(pinMatrixShuffle);
        initializePin(pinFdnLines);
        initializePin(pinInternalAllPassEnable);
        initializePin(pinInternalDiffusion);
        initializePin(pinInternalAllPassSize);

        for (auto& pin : pinDelayMs)
            initializePin(pin);

        initializePin(pinWetL);
        initializePin(pinWetR);

        const std::array<float, kLines> defaults = {
            37.13f, 41.11f, 47.29f, 53.17f,
            61.31f, 71.23f, 79.43f, 89.17f
        };

        for (int i = 0; i < kLines; ++i)
        {
            delayMsSmoothers_[static_cast<std::size_t>(i)].reset(defaults[static_cast<std::size_t>(i)]);
            randomOffsetSmoothers_[static_cast<std::size_t>(i)].reset(0.0f);
            lowGainSmoothers_[static_cast<std::size_t>(i)].reset(0.85f);
            midGainSmoothers_[static_cast<std::size_t>(i)].reset(0.85f);
            highGainSmoothers_[static_cast<std::size_t>(i)].reset(0.75f);
        }

        rt60Smoother_.reset(4.0f);
        sizeSmoother_.reset(1.0f);
        lfRatioSmoother_.reset(1.15f);
        hfRatioSmoother_.reset(0.55f);
        lowCrossoverSmoother_.reset(250.0f);
        highCrossoverSmoother_.reset(6000.0f);
        modDepthSmoother_.reset(0.10f);
        modRateSmoother_.reset(0.10f);
        widthSmoother_.reset(1.0f);
        freezeSmoother_.reset(0.0f);
        matrixShuffleSmoother_.reset(1.0f);
        internalAllPassEnableSmoother_.reset(0.0f);
        internalDiffusionSmoother_.reset(0.50f);
        internalAllPassSizeSmoother_.reset(1.0f);

        const auto now = static_cast<std::uint64_t>(
            std::chrono::high_resolution_clock::now().time_since_epoch().count());
        const auto self = static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(this));
        autoSeed_ = static_cast<std::uint32_t>((now ^ (self + 0x9e3779b97f4a7c15ULL)) & 0xffffffffu);
        if (autoSeed_ == 0)
            autoSeed_ = 0x41c64e6du;

        regenerateSeedPattern(101u);
        regenerateMatrixPattern(activeSeed_, activeLines_);
    }

    void subProcess(int sampleFrames)
    {
        auto inputL = getBuffer(pinInputL);
        auto inputR = getBuffer(pinInputR);
        auto rt60Pin = getBuffer(pinRt60Seconds);
        auto sizePin = getBuffer(pinMasterSize);
        auto lfRatioPin = getBuffer(pinLfDecayRatio);
        auto hfRatioPin = getBuffer(pinHfDecayRatio);
        auto lowCrossPin = getBuffer(pinLowCrossoverHz);
        auto highCrossPin = getBuffer(pinHighCrossoverHz);
        auto modDepthPin = getBuffer(pinModDepthMs);
        auto modRatePin = getBuffer(pinModRateHz);
        auto widthPin = getBuffer(pinStereoWidth);
        auto freezePin = getBuffer(pinFreeze);
        auto randomOffsetPin = getBuffer(pinRandomOffsetMs);
        auto randomSeedPin = getBuffer(pinRandomSeed);
        auto matrixShufflePin = getBuffer(pinMatrixShuffle);
        auto fdnLinesPin = getBuffer(pinFdnLines);
        auto internalAllPassEnablePin = getBuffer(pinInternalAllPassEnable);
        auto internalDiffusionPin = getBuffer(pinInternalDiffusion);
        auto internalAllPassSizePin = getBuffer(pinInternalAllPassSize);

        std::array<float*, kLines> delayPins{};
        for (int i = 0; i < kLines; ++i)
            delayPins[static_cast<std::size_t>(i)] = getBuffer(pinDelayMs[static_cast<std::size_t>(i)]);

        auto wetL = getBuffer(pinWetL);
        auto wetR = getBuffer(pinWetR);

        updateSampleRateIfNeeded();

        const float targetRt60 = clampf(
            finiteOrZero(*rt60Pin) * kInternalToDisplayedUnits,
            kMinRt60, kMaxRt60);
        const float targetSize = clampf(
            finiteOrZero(*sizePin) * kInternalToDisplayedUnits,
            kMinSize, kMaxSize);
        const float targetLfRatio = clampf(
            finiteOrZero(*lfRatioPin) * kInternalToDisplayedUnits,
            kMinDecayRatio, kMaxDecayRatio);
        const float targetHfRatio = clampf(
            finiteOrZero(*hfRatioPin) * kInternalToDisplayedUnits,
            kMinDecayRatio, kMaxDecayRatio);
        const float targetLowCross = clampf(
            finiteOrZero(*lowCrossPin) * kInternalToDisplayedUnits,
            kMinCrossoverHz, kMaxLowCrossoverHz);
        float targetHighCross = clampf(
            finiteOrZero(*highCrossPin) * kInternalToDisplayedUnits,
            kMinHighCrossoverHz, kMaxHighCrossoverHz);
        targetHighCross = (std::max)(targetHighCross, targetLowCross + 100.0f);

        const float targetModDepth = clampf(
            finiteOrZero(*modDepthPin) * kInternalToDisplayedUnits,
            0.0f, kMaxModDepthMs);
        const float targetModRate = clampf(
            finiteOrZero(*modRatePin) * kInternalToDisplayedUnits,
            kMinModRateHz, kMaxModRateHz);
        const float targetWidth = clampf(
            finiteOrZero(*widthPin) * kInternalToDisplayedUnits,
            0.0f, kMaxStereoWidth);
        const float freezeDisplayed = finiteOrZero(*freezePin) * kInternalToDisplayedUnits;
        const float targetFreeze = freezeDisplayed >= 0.5f ? 1.0f : 0.0f;
        const float matrixShuffleDisplayed =
            finiteOrZero(*matrixShufflePin) * kInternalToDisplayedUnits;
        const float targetMatrixShuffle =
            matrixShuffleDisplayed >= 0.5f ? 1.0f : 0.0f;

        const float linesDisplayed =
            finiteOrZero(*fdnLinesPin) * kInternalToDisplayedUnits;
        int requestedLines = 8;
        if (linesDisplayed < 5.0f)
            requestedLines = 4;
        else if (linesDisplayed < 7.0f)
            requestedLines = 6;

        const float allPassDisplayed =
            finiteOrZero(*internalAllPassEnablePin) * kInternalToDisplayedUnits;
        const float targetInternalAllPassEnable =
            allPassDisplayed >= 0.5f ? 1.0f : 0.0f;
        const float targetInternalDiffusion = clampf(
            finiteOrZero(*internalDiffusionPin) * kInternalToDisplayedUnits,
            0.0f, 1.0f);
        const float targetInternalAllPassSize = clampf(
            finiteOrZero(*internalAllPassSizePin) * kInternalToDisplayedUnits,
            kMinInternalAllPassSize, kMaxInternalAllPassSize);

        if (requestedLines != activeLines_)
        {
            activeLines_ = requestedLines;
            regenerateMatrixPattern(activeSeed_, activeLines_);
            clearNetworkAndFade();
        }

        rt60Smoother_.setTarget(targetRt60);
        sizeSmoother_.setTarget(targetSize);
        lfRatioSmoother_.setTarget(targetLfRatio);
        hfRatioSmoother_.setTarget(targetHfRatio);
        lowCrossoverSmoother_.setTarget(targetLowCross);
        highCrossoverSmoother_.setTarget(targetHighCross);
        modDepthSmoother_.setTarget(targetModDepth);
        modRateSmoother_.setTarget(targetModRate);
        widthSmoother_.setTarget(targetWidth);
        freezeSmoother_.setTarget(targetFreeze);
        matrixShuffleSmoother_.setTarget(targetMatrixShuffle);
        internalAllPassEnableSmoother_.setTarget(targetInternalAllPassEnable);
        internalDiffusionSmoother_.setTarget(targetInternalDiffusion);
        internalAllPassSizeSmoother_.setTarget(targetInternalAllPassSize);

        const float randomAmountMs = clampf(
            finiteOrZero(*randomOffsetPin) * kInternalToDisplayedUnits,
            0.0f, kMaxRandomOffsetMs);

        const int requestedSeed = (std::max)(0, static_cast<int>(std::lround(
            finiteOrZero(*randomSeedPin) * kInternalToDisplayedUnits)));
        const std::uint32_t effectiveSeed = requestedSeed == 0
            ? autoSeed_
            : static_cast<std::uint32_t>(requestedSeed);
        if (effectiveSeed != activeSeed_)
        {
            regenerateSeedPattern(effectiveSeed);
            regenerateMatrixPattern(activeSeed_, activeLines_);
        }

        std::array<float, kLines> delayTargetsMs{};
        for (int i = 0; i < kLines; ++i)
        {
            const float baseMs = clampf(
                finiteOrZero(*delayPins[static_cast<std::size_t>(i)]) * kInternalToDisplayedUnits,
                0.10f, kMaxDelayMs);
            delayTargetsMs[static_cast<std::size_t>(i)] = baseMs;
            delayMsSmoothers_[static_cast<std::size_t>(i)].setTarget(baseMs);

            const float offset = randomPattern_[static_cast<std::size_t>(i)] * randomAmountMs;
            randomOffsetSmoothers_[static_cast<std::size_t>(i)].setTarget(offset);

            const float nominalMs = clampf((baseMs + offset) * targetSize, 0.10f, kMaxDelayMs);
            const float delaySec = nominalMs * 0.001f;
            const float midGain = rt60Gain(delaySec, targetRt60);
            const float lowGain = rt60Gain(delaySec, targetRt60 * targetLfRatio);
            const float highGain = rt60Gain(delaySec, targetRt60 * targetHfRatio);

            lowGainSmoothers_[static_cast<std::size_t>(i)].setTarget(lowGain);
            midGainSmoothers_[static_cast<std::size_t>(i)].setTarget(midGain);
            highGainSmoothers_[static_cast<std::size_t>(i)].setTarget(highGain);
        }

        for (int s = 0; s < sampleFrames; ++s)
        {
            const float inL = finiteOrZero(*inputL++);
            const float inR = finiteOrZero(*inputR++);

            const float size = sizeSmoother_.next();
            const float lowCross = lowCrossoverSmoother_.next();
            float highCross = highCrossoverSmoother_.next();
            highCross = (std::max)(highCross, lowCross + 100.0f);
            const float lowCoeff = onePoleCoeff(lowCross, currentSampleRate_);
            const float highCoeff = onePoleCoeff(highCross, currentSampleRate_);

            // Advance these even though normal decay gains were already converted
            // to targets at block start. This keeps control transitions smooth and
            // leaves the structure ready for future per-sample RT60 refinement.
            rt60Smoother_.next();
            lfRatioSmoother_.next();
            hfRatioSmoother_.next();

            const float modDepth = modDepthSmoother_.next();
            const float modRate = modRateSmoother_.next();
            const float width = widthSmoother_.next();
            const float freeze = freezeSmoother_.next();
            const float shuffle = matrixShuffleSmoother_.next();
            const float internalAllPassEnable = internalAllPassEnableSmoother_.next();
            const float internalDiffusion = internalDiffusionSmoother_.next();
            const float internalAllPassSize = internalAllPassSizeSmoother_.next();

            std::array<float, kLines> rawOutputs{};
            std::array<float, kLines> tankOutputs{};
            std::array<float, kLines> feedbackSignals{};

            for (int i = 0; i < activeLines_; ++i)
            {
                float baseMs = delayMsSmoothers_[static_cast<std::size_t>(i)].next();
                const float offsetMs = randomOffsetSmoothers_[static_cast<std::size_t>(i)].next();
                baseMs = (baseMs + offsetMs) * size;

                float lfo = 0.0f;
                if (freeze < 0.9995f && modDepth > 0.0f)
                {
                    lfo = std::sin(modPhases_[static_cast<std::size_t>(i)]);
                    const float lineRate = modRate * modRateRatios_[static_cast<std::size_t>(i)];
                    modPhases_[static_cast<std::size_t>(i)] +=
                        2.0f * kPi * lineRate / currentSampleRate_;
                    if (modPhases_[static_cast<std::size_t>(i)] >= 2.0f * kPi)
                        modPhases_[static_cast<std::size_t>(i)] -= 2.0f * kPi;
                }

                const float activeModDepth = modDepth * (1.0f - freeze);
                float delayMs = clampf(baseMs + lfo * activeModDepth, 0.10f, kMaxDelayMs);
                float delaySamples = delayMs * 0.001f * currentSampleRate_;

                // At full Freeze use exact integer taps. With unity decay gains and
                // an orthogonal matrix this avoids the small loss of fractional interpolation entirely and keeps full Freeze as an exact
                // integer-delay, orthogonal-matrix loop for the cleanest infinite hold.
                if (freeze >= 0.9995f)
                    delaySamples = std::round(delaySamples);

                rawOutputs[static_cast<std::size_t>(i)] =
                    delayLines_[static_cast<std::size_t>(i)].read(delaySamples);

                static constexpr std::array<float, kLines> kAllPassBaseMs =
                    {{ 1.7f, 2.9f, 4.1f, 5.3f, 6.7f, 8.3f, 10.1f, 12.7f }};
                static constexpr std::array<float, kLines> kAllPassSigns =
                    {{ 1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f }};

                const float apDelayMs = clampf(
                    kAllPassBaseMs[static_cast<std::size_t>(i)] * internalAllPassSize,
                    0.10f, kMaxInternalAllPassDelayMs);
                const float apDelaySamples =
                    apDelayMs * 0.001f * currentSampleRate_;
                const float apG =
                    kAllPassSigns[static_cast<std::size_t>(i)] *
                    (internalDiffusion * kMaxInternalAllPassG);

                // Always run the AP state so enabling it later does not expose
                // an empty delay buffer. The enable control crossfades dry/AP.
                const float apOut =
                    internalAllPasses_[static_cast<std::size_t>(i)].process(
                        rawOutputs[static_cast<std::size_t>(i)],
                        apDelaySamples,
                        apG);

                const float scattered =
                    rawOutputs[static_cast<std::size_t>(i)] +
                    internalAllPassEnable *
                    (apOut - rawOutputs[static_cast<std::size_t>(i)]);
                tankOutputs[static_cast<std::size_t>(i)] = scattered;

                const float normalLow = lowGainSmoothers_[static_cast<std::size_t>(i)].next();
                const float normalMid = midGainSmoothers_[static_cast<std::size_t>(i)].next();
                const float normalHigh = highGainSmoothers_[static_cast<std::size_t>(i)].next();

                if (freeze >= 0.9995f)
                {
                    // Exact bypass in full freeze. Filter states are not part of the
                    // lossless loop, preventing tiny reconstruction errors from
                    // accumulating over an extremely long hold.
                    feedbackSignals[static_cast<std::size_t>(i)] = tankOutputs[static_cast<std::size_t>(i)];
                }
                else
                {
                    const float lowGain = normalLow + freeze * (1.0f - normalLow);
                    const float midGain = normalMid + freeze * (1.0f - normalMid);
                    const float highGain = normalHigh + freeze * (1.0f - normalHigh);
                    feedbackSignals[static_cast<std::size_t>(i)] =
                        dampingStates_[static_cast<std::size_t>(i)].process(
                            tankOutputs[static_cast<std::size_t>(i)],
                            lowCoeff,
                            highCoeff,
                            lowGain,
                            midGain,
                            highGain);
                }
            }

            std::array<float, kLines> matrixInputs{};
            if (shuffle >= 0.5f)
            {
                for (int i = 0; i < activeLines_; ++i)
                {
                    const int src = matrixInputPermutation_[static_cast<std::size_t>(i)];
                    matrixInputs[static_cast<std::size_t>(i)] =
                        feedbackSignals[static_cast<std::size_t>(src)] *
                        matrixInputSigns_[static_cast<std::size_t>(i)];
                }
            }
            else
            {
                for (int i = 0; i < activeLines_; ++i)
                    matrixInputs[static_cast<std::size_t>(i)] =
                        feedbackSignals[static_cast<std::size_t>(i)];
            }

            std::array<float, kLines> matrixCore{};
            orthogonalMix(matrixInputs, matrixCore, activeLines_);

            std::array<float, kLines> matrixOutputs{};
            if (shuffle >= 0.5f)
            {
                for (int i = 0; i < activeLines_; ++i)
                {
                    const int dest = matrixOutputPermutation_[static_cast<std::size_t>(i)];
                    matrixOutputs[static_cast<std::size_t>(dest)] =
                        matrixCore[static_cast<std::size_t>(i)] *
                        matrixOutputSigns_[static_cast<std::size_t>(i)];
                }
            }
            else
            {
                for (int i = 0; i < activeLines_; ++i)
                    matrixOutputs[static_cast<std::size_t>(i)] =
                        matrixCore[static_cast<std::size_t>(i)];
            }

            // Two orthogonal input injection vectors. Normalization follows the
            // selected 4/6/8 line count.
            // normalization keeps input energy sensible even when both channels play.
            const float injectionGain = 1.0f - freeze;
            const float activeNorm = inverseSqrtForLines(activeLines_);
            for (int i = 0; i < activeLines_; ++i)
            {
                const float injectLSign = 1.0f;
                const float injectRSign = (i & 1) ? -1.0f : 1.0f;

                const float injection = injectionGain * activeNorm *
                    (injectLSign * inL + injectRSign * inR);

                const float writeValue =
                    matrixOutputs[static_cast<std::size_t>(i)] + injection;
                delayLines_[static_cast<std::size_t>(i)].write(writeValue);
            }

            // Orthogonal-ish signed output taps. They are independent from the
            // feedback matrix and avoid simply splitting delays 1-4 vs 5-8.
            static constexpr std::array<float, kLines> kOutL =
                {{ 1.0f,  1.0f, -1.0f,  1.0f, -1.0f,  1.0f,  1.0f, -1.0f }};
            static constexpr std::array<float, kLines> kOutR =
                {{ 1.0f, -1.0f,  1.0f,  1.0f,  1.0f, -1.0f,  1.0f, -1.0f }};

            float outL = 0.0f;
            float outR = 0.0f;
            for (int i = 0; i < activeLines_; ++i)
            {
                outL += kOutL[static_cast<std::size_t>(i)] *
                    tankOutputs[static_cast<std::size_t>(i)];
                outR += kOutR[static_cast<std::size_t>(i)] *
                    tankOutputs[static_cast<std::size_t>(i)];
            }
            const float outputNorm = inverseSqrtForLines(activeLines_);
            outL *= outputNorm;
            outR *= outputNorm;

            const float mid = 0.5f * (outL + outR);
            const float side = 0.5f * (outL - outR);
            outL = mid + width * side;
            outR = mid - width * side;

            outL = zapDenormal(safetyLimit(outL)) * resetFade_;
            outR = zapDenormal(safetyLimit(outR)) * resetFade_;

            if (resetFade_ < 1.0f)
                resetFade_ = (std::min)(1.0f, resetFade_ + resetFadeIncrement_);

            *wetL++ = outL;
            *wetR++ = outR;
        }
    }

    void onSetPins() override
    {
        pinWetL.setStreaming(true);
        pinWetR.setStreaming(true);

        // The FDN must keep evolving after the input stops because the tank can
        // have a long RT60 or be frozen indefinitely.
        setSleep(false);
        setSubProcess(&FDNReverbTank::subProcess);
    }

private:
    void updateSampleRateIfNeeded()
    {
        const float newRate = (std::max)(1000.0f, finiteOrZero(getSampleRate()));
        if (sampleRateInitialized_ && std::abs(newRate - currentSampleRate_) < 0.5f)
            return;

        const bool wasInitialized = sampleRateInitialized_;
        sampleRateInitialized_ = true;
        currentSampleRate_ = newRate;

        rt60Smoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        sizeSmoother_.setTime(kDelaySmoothingMs, currentSampleRate_);
        lfRatioSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        hfRatioSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        lowCrossoverSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        highCrossoverSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        modDepthSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        modRateSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        widthSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        freezeSmoother_.setTime(kFreezeSmoothingMs, currentSampleRate_);
        matrixShuffleSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        internalAllPassEnableSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        internalDiffusionSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        internalAllPassSizeSmoother_.setTime(kDelaySmoothingMs, currentSampleRate_);

        for (auto& smoother : delayMsSmoothers_)
            smoother.setTime(kDelaySmoothingMs, currentSampleRate_);
        for (auto& smoother : randomOffsetSmoothers_)
            smoother.setTime(kDelaySmoothingMs, currentSampleRate_);
        for (auto& smoother : lowGainSmoothers_)
            smoother.setTime(kParameterSmoothingMs, currentSampleRate_);
        for (auto& smoother : midGainSmoothers_)
            smoother.setTime(kParameterSmoothingMs, currentSampleRate_);
        for (auto& smoother : highGainSmoothers_)
            smoother.setTime(kParameterSmoothingMs, currentSampleRate_);

        for (auto& line : delayLines_)
            line.clear();
        for (auto& ap : internalAllPasses_)
            ap.clear();
        for (auto& state : dampingStates_)
            state.clear();

        if (wasInitialized)
        {
            resetFade_ = 0.0f;
            resetFadeIncrement_ = 1.0f / (currentSampleRate_ * (kResetFadeMs * 0.001f));
        }
        else
        {
            resetFade_ = 1.0f;
            resetFadeIncrement_ = 1.0f;
        }
    }

    void regenerateSeedPattern(std::uint32_t seed)
    {
        activeSeed_ = seed == 0 ? 0x41c64e6du : seed;
        std::uint32_t state = activeSeed_;

        for (int i = 0; i < kLines; ++i)
        {
            float r = randomBipolar(state);
            if (std::abs(r) < 0.12f)
                r = r < 0.0f ? -0.12f : 0.12f;
            randomPattern_[static_cast<std::size_t>(i)] = r;

            modRateRatios_[static_cast<std::size_t>(i)] = 0.72f + 0.56f * randomUnit(state);
            modPhases_[static_cast<std::size_t>(i)] = 2.0f * kPi * randomUnit(state);

        }

    }


    void clearNetworkAndFade()
    {
        for (auto& line : delayLines_)
            line.clear();
        for (auto& ap : internalAllPasses_)
            ap.clear();
        for (auto& state : dampingStates_)
            state.clear();

        resetFade_ = 0.0f;
        resetFadeIncrement_ =
            1.0f / ((std::max)(1000.0f, currentSampleRate_) *
                    (kResetFadeMs * 0.001f));
    }

    void regenerateMatrixPattern(std::uint32_t seed, int activeLines)
    {
        std::uint32_t state =
            (seed == 0 ? 0x41c64e6du : seed) ^
            (0x9e3779b9u + static_cast<std::uint32_t>(activeLines * 131));

        for (int i = 0; i < kLines; ++i)
        {
            matrixInputPermutation_[static_cast<std::size_t>(i)] = i;
            matrixOutputPermutation_[static_cast<std::size_t>(i)] = i;
            matrixInputSigns_[static_cast<std::size_t>(i)] =
                randomBipolar(state) >= 0.0f ? 1.0f : -1.0f;
            matrixOutputSigns_[static_cast<std::size_t>(i)] =
                randomBipolar(state) >= 0.0f ? 1.0f : -1.0f;
        }

        for (int i = activeLines - 1; i > 0; --i)
        {
            const int j = static_cast<int>(
                xorshift32(state) % static_cast<std::uint32_t>(i + 1));
            std::swap(
                matrixInputPermutation_[static_cast<std::size_t>(i)],
                matrixInputPermutation_[static_cast<std::size_t>(j)]);
        }

        for (int i = activeLines - 1; i > 0; --i)
        {
            const int j = static_cast<int>(
                xorshift32(state) % static_cast<std::uint32_t>(i + 1));
            std::swap(
                matrixOutputPermutation_[static_cast<std::size_t>(i)],
                matrixOutputPermutation_[static_cast<std::size_t>(j)]);
        }
    }

    AudioInPin pinInputL;
    AudioInPin pinInputR;
    AudioInPin pinRt60Seconds;
    AudioInPin pinMasterSize;
    AudioInPin pinLfDecayRatio;
    AudioInPin pinHfDecayRatio;
    AudioInPin pinLowCrossoverHz;
    AudioInPin pinHighCrossoverHz;
    AudioInPin pinModDepthMs;
    AudioInPin pinModRateHz;
    AudioInPin pinStereoWidth;
    AudioInPin pinFreeze;
    AudioInPin pinRandomOffsetMs;
    AudioInPin pinRandomSeed;
    AudioInPin pinMatrixShuffle;
    AudioInPin pinFdnLines;
    AudioInPin pinInternalAllPassEnable;
    AudioInPin pinInternalDiffusion;
    AudioInPin pinInternalAllPassSize;
    std::array<AudioInPin, kLines> pinDelayMs;

    AudioOutPin pinWetL;
    AudioOutPin pinWetR;

    std::array<FractionalDelay, kLines> delayLines_;
    std::array<ScatteringAllPass, kLines> internalAllPasses_;
    std::array<ThreeBandDecayState, kLines> dampingStates_;

    std::array<SmoothedValue, kLines> delayMsSmoothers_;
    std::array<SmoothedValue, kLines> randomOffsetSmoothers_;
    std::array<SmoothedValue, kLines> lowGainSmoothers_;
    std::array<SmoothedValue, kLines> midGainSmoothers_;
    std::array<SmoothedValue, kLines> highGainSmoothers_;

    SmoothedValue rt60Smoother_;
    SmoothedValue sizeSmoother_;
    SmoothedValue lfRatioSmoother_;
    SmoothedValue hfRatioSmoother_;
    SmoothedValue lowCrossoverSmoother_;
    SmoothedValue highCrossoverSmoother_;
    SmoothedValue modDepthSmoother_;
    SmoothedValue modRateSmoother_;
    SmoothedValue widthSmoother_;
    SmoothedValue freezeSmoother_;
    SmoothedValue matrixShuffleSmoother_;
    SmoothedValue internalAllPassEnableSmoother_;
    SmoothedValue internalDiffusionSmoother_;
    SmoothedValue internalAllPassSizeSmoother_;

    int activeLines_ = 8;

    std::array<float, kLines> randomPattern_{};
    std::array<float, kLines> modRateRatios_{};
    std::array<float, kLines> modPhases_{};
    std::array<int, kLines> matrixInputPermutation_{{0,1,2,3,4,5,6,7}};
    std::array<int, kLines> matrixOutputPermutation_{{0,1,2,3,4,5,6,7}};
    std::array<float, kLines> matrixInputSigns_{{1,1,1,1,1,1,1,1}};
    std::array<float, kLines> matrixOutputSigns_{{1,1,1,1,1,1,1,1}};

    std::uint32_t autoSeed_ = 1u;
    std::uint32_t activeSeed_ = 0u;

    float currentSampleRate_ = 44100.0f;
    bool sampleRateInitialized_ = false;
    float resetFade_ = 1.0f;
    float resetFadeIncrement_ = 1.0f;
};

namespace
{
    auto registration =
        Register<FDNReverbTank>::withId(L"Pandocrator FDN Reverb Tank v3");
}
