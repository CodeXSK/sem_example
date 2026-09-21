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
    constexpr int   kLanes = 4;
    constexpr int   kBanks = 4;
    constexpr int   kDelayCount = kLanes * kBanks;
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;
    constexpr float kMaxDelayMs = 250.0f;
    constexpr float kMaxSupportedSampleRate = 384000.0f;
    constexpr float kMaxTone = 1.0f;
    constexpr float kMaxRandomOffsetMs = 12.0f;
    constexpr float kMinMasterSize = 0.25f;
    constexpr float kMaxMasterSize = 4.0f;
    constexpr float kParameterSmoothingMs = 20.0f;
    constexpr float kDelaySmoothingMs = 25.0f;
    constexpr float kRandomOffsetSmoothingMs = 120.0f;
    constexpr float kInvSqrt2 = 0.7071067811865475244f;

    // SynthEdit audio/control pins use internal units where 1.0 = 10 displayed units.
    constexpr float kInternalToDisplayedUnits = 10.0f;

    constexpr int kDelayBufferSamples =
        static_cast<int>(kMaxSupportedSampleRate * (kMaxDelayMs * 0.001f)) + 8;

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

    private:
        float current_ = 0.0f;
        float target_ = 0.0f;
        float coefficient_ = 0.0f;
    };

    // Pure feed-forward fractional delay. There is NO internal feedback.
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
        }

        float process(float input, float delaySamples)
        {
            input = finiteOrZero(input);
            const float maxDelay = static_cast<float>(kDelayBufferSamples - 3);
            delaySamples = clampf(finiteOrZero(delaySamples), 1.0f, maxDelay);

            float readPosition = static_cast<float>(writeIndex_) - delaySamples;
            while (readPosition < 0.0f)
                readPosition += static_cast<float>(kDelayBufferSamples);

            const int index0 = static_cast<int>(readPosition);
            const int index1 = (index0 + 1) % kDelayBufferSamples;
            const float fraction = readPosition - static_cast<float>(index0);

            const float d0 = finiteOrZero(buffer_[static_cast<std::size_t>(index0)]);
            const float d1 = finiteOrZero(buffer_[static_cast<std::size_t>(index1)]);
            const float output = d0 + fraction * (d1 - d0);

            buffer_[static_cast<std::size_t>(writeIndex_)] = zapDenormal(input);
            ++writeIndex_;
            if (writeIndex_ >= kDelayBufferSamples)
                writeIndex_ = 0;

            return zapDenormal(finiteOrZero(output));
        }

    private:
        std::vector<float> buffer_;
        int writeIndex_ = 0;
    };

    // Very gentle progressive spectral shaping. No resonance and no feedback.
    class ToneFilter
    {
    public:
        void clear()
        {
            lowState_ = 0.0f;
            bassState_ = 0.0f;
        }

        void setSampleRate(float sampleRate, int bankIndex)
        {
            sampleRate_ = (std::max)(1000.0f, sampleRate);
            progress_ = clampf((static_cast<float>(bankIndex) + 1.0f) / static_cast<float>(kBanks), 0.25f, 1.0f);

            // Later banks may become progressively darker when Tone < 0.
            const float darkCutoffHz = 18000.0f + progress_ * (6500.0f - 18000.0f);
            const float bassCutoffHz = 45.0f + progress_ * (180.0f - 45.0f);
            lowCoeff_ = onePoleCoeff(darkCutoffHz, sampleRate_);
            bassCoeff_ = onePoleCoeff(bassCutoffHz, sampleRate_);
        }

        float process(float input, float tone)
        {
            input = finiteOrZero(input);
            tone = clampf(finiteOrZero(tone), -kMaxTone, kMaxTone);

            lowState_ += lowCoeff_ * (input - lowState_);
            lowState_ = zapDenormal(finiteOrZero(lowState_));

            bassState_ += bassCoeff_ * (input - bassState_);
            bassState_ = zapDenormal(finiteOrZero(bassState_));
            const float highPassed = finiteOrZero(input - bassState_);

            const float darkMix = (std::max)(0.0f, -tone) * (0.10f + 0.42f * progress_);
            const float brightMix = (std::max)(0.0f, tone) * (0.03f + 0.10f * progress_);

            float output = input + darkMix * (lowState_ - input);
            output += brightMix * (highPassed - output);
            return zapDenormal(finiteOrZero(output));
        }

    private:
        static float onePoleCoeff(float cutoffHz, float sampleRate)
        {
            const float fc = clampf(cutoffHz, 5.0f, 0.45f * sampleRate);
            return 1.0f - std::exp(-kTwoPi * fc / sampleRate);
        }

        float sampleRate_ = 44100.0f;
        float progress_ = 0.25f;
        float lowState_ = 0.0f;
        float bassState_ = 0.0f;
        float lowCoeff_ = 1.0f;
        float bassCoeff_ = 0.01f;
    };

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

    inline std::array<float, kLanes> hadamard4(const std::array<float, kLanes>& x)
    {
        // Orthonormal 4x4 Hadamard, scale = 1/sqrt(4) = 0.5.
        return {{
            0.5f * (x[0] + x[1] + x[2] + x[3]),
            0.5f * (x[0] - x[1] + x[2] - x[3]),
            0.5f * (x[0] + x[1] - x[2] - x[3]),
            0.5f * (x[0] - x[1] - x[2] + x[3])
        }};
    }
}

// Pandocrator Scattering Diffuser
// --------------------------------
// 4 lanes x 4 feed-forward delay/matrix stages.
// NO all-pass sections and NO recursive feedback.
//
// Input -> 4 delays -> H -> 4 delays -> H -> 4 delays -> H -> 4 delays -> H -> output
//
// The purpose is to build echo density through repeated orthonormal scattering instead
// of through recursive all-pass ringing. The impulse response is finite by construction.
class ScatteringDiffuser final : public MpBase2
{
public:
    ScatteringDiffuser()
    {
        // Pin order MUST match ScatteringDiffuser.xml.
        initializePin(pinSignalIn);
        initializePin(pinDensity);
        initializePin(pinTone);
        initializePin(pinMasterSize);
        initializePin(pinRandomOffsetMs);
        initializePin(pinRandomSeed);
        initializePin(pinMatrixEnable);
        initializePin(pinMatrixAmount);
        initializePin(pinShuffleTime);
        initializePin(pinShufflePolarity);
        initializePin(pinStereoSpread);

        for (auto& pin : pinDelayMs)
            initializePin(pin);

        initializePin(pinSignalOut);
        initializePin(pinLeftOut);
        initializePin(pinRightOut);
        for (auto& pin : pinLaneOut)
            initializePin(pin);

        const std::array<float, kDelayCount> defaults = {{
             3.70f,  5.30f,  7.10f,  8.90f,
             5.90f,  8.30f, 11.70f, 14.10f,
             9.10f, 12.70f, 17.30f, 21.10f,
            13.70f, 19.10f, 25.30f, 31.70f
        }};

        for (int i = 0; i < kDelayCount; ++i)
        {
            delaySmoothers_[static_cast<std::size_t>(i)].reset(defaults[static_cast<std::size_t>(i)]);
            randomOffsetSmoothers_[static_cast<std::size_t>(i)].reset(0.0f);
        }

        densitySmoother_.reset(4.0f);
        toneSmoother_.reset(-0.15f);
        masterSizeSmoother_.reset(1.0f);
        matrixEnableSmoother_.reset(1.0f);
        matrixAmountSmoother_.reset(1.0f);
        shuffleTimeSmoother_.reset(0.0f);
        shufflePolaritySmoother_.reset(0.0f);
        stereoSpreadSmoother_.reset(1.0f);

        const auto now = static_cast<std::uint64_t>(
            std::chrono::high_resolution_clock::now().time_since_epoch().count());
        const auto self = static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(this));
        autoSeed_ = static_cast<std::uint32_t>((now ^ (self + 0x9e3779b97f4a7c15ULL)) & 0xffffffffu);
        if (autoSeed_ == 0)
            autoSeed_ = 0x5bd1e995u;

        regeneratePattern(autoSeed_);
    }

    void subProcess(int sampleFrames)
    {
        updateSampleRateIfNeeded();

        auto signalIn = getBuffer(pinSignalIn);
        auto densityPin = getBuffer(pinDensity);
        auto tonePin = getBuffer(pinTone);
        auto masterSizePin = getBuffer(pinMasterSize);
        auto randomOffsetPin = getBuffer(pinRandomOffsetMs);
        auto randomSeedPin = getBuffer(pinRandomSeed);
        auto matrixEnablePin = getBuffer(pinMatrixEnable);
        auto matrixAmountPin = getBuffer(pinMatrixAmount);
        auto shuffleTimePin = getBuffer(pinShuffleTime);
        auto shufflePolarityPin = getBuffer(pinShufflePolarity);
        auto stereoSpreadPin = getBuffer(pinStereoSpread);

        std::array<float*, kDelayCount> delayPins{};
        for (int i = 0; i < kDelayCount; ++i)
            delayPins[static_cast<std::size_t>(i)] = getBuffer(pinDelayMs[static_cast<std::size_t>(i)]);

        auto signalOut = getBuffer(pinSignalOut);
        auto leftOut = getBuffer(pinLeftOut);
        auto rightOut = getBuffer(pinRightOut);
        std::array<float*, kLanes> laneOutputs{};
        for (int lane = 0; lane < kLanes; ++lane)
            laneOutputs[static_cast<std::size_t>(lane)] = getBuffer(pinLaneOut[static_cast<std::size_t>(lane)]);

        densitySmoother_.setTarget(clampf(
            finiteOrZero(*densityPin) * kInternalToDisplayedUnits,
            1.0f,
            4.0f));

        toneSmoother_.setTarget(clampf(
            finiteOrZero(*tonePin) * kInternalToDisplayedUnits,
            -kMaxTone,
            kMaxTone));

        masterSizeSmoother_.setTarget(clampf(
            finiteOrZero(*masterSizePin) * kInternalToDisplayedUnits,
            kMinMasterSize,
            kMaxMasterSize));

        const float randomAmountMs = clampf(
            finiteOrZero(*randomOffsetPin) * kInternalToDisplayedUnits,
            0.0f,
            kMaxRandomOffsetMs);

        const int requestedSeed = (std::max)(0, static_cast<int>(std::lround(
            finiteOrZero(*randomSeedPin) * kInternalToDisplayedUnits)));
        const std::uint32_t effectiveSeed = requestedSeed == 0
            ? autoSeed_
            : static_cast<std::uint32_t>(requestedSeed);
        if (effectiveSeed != activeSeed_)
            regeneratePattern(effectiveSeed);

        const float matrixEnableValue = finiteOrZero(*matrixEnablePin) * kInternalToDisplayedUnits;
        matrixEnableSmoother_.setTarget(matrixEnableValue >= 0.5f ? 1.0f : 0.0f);
        matrixAmountSmoother_.setTarget(clampf(
            finiteOrZero(*matrixAmountPin) * kInternalToDisplayedUnits,
            0.0f,
            1.0f));

        const float shuffleTimeValue = finiteOrZero(*shuffleTimePin) * kInternalToDisplayedUnits;
        shuffleTimeSmoother_.setTarget(shuffleTimeValue >= 0.5f ? 1.0f : 0.0f);

        const float shufflePolarityValue = finiteOrZero(*shufflePolarityPin) * kInternalToDisplayedUnits;
        shufflePolaritySmoother_.setTarget(shufflePolarityValue >= 0.5f ? 1.0f : 0.0f);

        stereoSpreadSmoother_.setTarget(clampf(
            finiteOrZero(*stereoSpreadPin) * kInternalToDisplayedUnits,
            0.0f,
            1.0f));

        for (int i = 0; i < kDelayCount; ++i)
        {
            const float ms = clampf(
                finiteOrZero(*delayPins[static_cast<std::size_t>(i)]) * kInternalToDisplayedUnits,
                0.1f,
                kMaxDelayMs);
            delaySmoothers_[static_cast<std::size_t>(i)].setTarget(ms);
            randomOffsetSmoothers_[static_cast<std::size_t>(i)].setTarget(
                randomPattern_[static_cast<std::size_t>(i)] * randomAmountMs);
        }

        for (int s = 0; s < sampleFrames; ++s)
        {
            const float input = finiteOrZero(*signalIn++);
            const float density = densitySmoother_.next();
            const float tone = toneSmoother_.next();
            const float size = masterSizeSmoother_.next();
            const float matrixEnable = matrixEnableSmoother_.next();
            const float matrixAmount = matrixAmountSmoother_.next() * matrixEnable;
            const float shuffleTime = shuffleTimeSmoother_.next();
            const float shufflePolarity = shufflePolaritySmoother_.next();
            const float stereoSpread = stereoSpreadSmoother_.next();

            std::array<float, kDelayCount> currentDelayMs{};
            for (int i = 0; i < kDelayCount; ++i)
            {
                const float base = delaySmoothers_[static_cast<std::size_t>(i)].next();
                const float offset = randomOffsetSmoothers_[static_cast<std::size_t>(i)].next();
                currentDelayMs[static_cast<std::size_t>(i)] = base + offset;
            }

            // Four equal-energy copies: 4 * (0.5^2) = 1.0 total energy.
            std::array<float, kLanes> lanes{{
                input * 0.5f, input * 0.5f, input * 0.5f, input * 0.5f
            }};

            std::array<std::array<float, kLanes>, kBanks> bankStates{};

            for (int bank = 0; bank < kBanks; ++bank)
            {
                std::array<float, kLanes> delayed{};
                for (int lane = 0; lane < kLanes; ++lane)
                {
                    const int directLocal = lane;
                    const int shuffledLocal = timePermutation_[static_cast<std::size_t>(bank)][static_cast<std::size_t>(lane)];
                    const int directIndex = bank * kLanes + directLocal;
                    const int shuffledIndex = bank * kLanes + shuffledLocal;

                    const float directMs = currentDelayMs[static_cast<std::size_t>(directIndex)];
                    const float shuffledMs = currentDelayMs[static_cast<std::size_t>(shuffledIndex)];
                    const float selectedMs = directMs + shuffleTime * (shuffledMs - directMs);
                    const float delayMs = clampf(selectedMs * size, 0.1f, kMaxDelayMs);

                    float y = delays_[static_cast<std::size_t>(directIndex)].process(
                        lanes[static_cast<std::size_t>(lane)],
                        delayMs * 0.001f * currentSampleRate_);
                    y = toneFilters_[static_cast<std::size_t>(directIndex)].process(y, tone);

                    const float sign = 1.0f + shufflePolarity *
                        (inputSigns_[static_cast<std::size_t>(bank)][static_cast<std::size_t>(lane)] - 1.0f);
                    delayed[static_cast<std::size_t>(lane)] = y * sign;
                }

                const auto h = hadamard4(delayed);
                std::array<float, kLanes> scattered{};

                for (int lane = 0; lane < kLanes; ++lane)
                {
                    // A different fixed row permutation per bank prevents four identical
                    // matrix orientations while remaining perfectly energy preserving.
                    const int row = fixedMatrixPermutation_[static_cast<std::size_t>(bank)][static_cast<std::size_t>(lane)];
                    const float outSign = 1.0f + shufflePolarity *
                        (outputSigns_[static_cast<std::size_t>(bank)][static_cast<std::size_t>(lane)] - 1.0f);
                    const float matrixSignal = h[static_cast<std::size_t>(row)] * outSign;

                    scattered[static_cast<std::size_t>(lane)] =
                        delayed[static_cast<std::size_t>(lane)] +
                        matrixAmount * (matrixSignal - delayed[static_cast<std::size_t>(lane)]);
                }

                lanes = scattered;
                bankStates[static_cast<std::size_t>(bank)] = lanes;
            }

            // Density is continuous 1..4. It crossfades between complete scattering stages
            // rather than abruptly switching a bank in/out.
            const float d = clampf(density, 1.0f, 4.0f);
            const int lowerBank = (std::min)(kBanks - 1, (std::max)(0, static_cast<int>(std::floor(d)) - 1));
            const int upperBank = (std::min)(kBanks - 1, lowerBank + 1);
            const float frac = (upperBank == lowerBank) ? 0.0f : d - std::floor(d);

            std::array<float, kLanes> selected{};
            for (int lane = 0; lane < kLanes; ++lane)
            {
                const float a = bankStates[static_cast<std::size_t>(lowerBank)][static_cast<std::size_t>(lane)];
                const float b = bankStates[static_cast<std::size_t>(upperBank)][static_cast<std::size_t>(lane)];
                selected[static_cast<std::size_t>(lane)] = a + frac * (b - a);
            }

            // Unit-norm decoders.
            const float mono = 0.5f * (selected[0] + selected[1] + selected[2] + selected[3]);
            const float leftWide = kInvSqrt2 * (selected[0] + selected[2]);
            const float rightWide = kInvSqrt2 * (selected[1] + selected[3]);
            const float left = mono + stereoSpread * (leftWide - mono);
            const float right = mono + stereoSpread * (rightWide - mono);

            *signalOut++ = zapDenormal(finiteOrZero(mono));
            *leftOut++ = zapDenormal(finiteOrZero(left));
            *rightOut++ = zapDenormal(finiteOrZero(right));
            for (int lane = 0; lane < kLanes; ++lane)
                *laneOutputs[static_cast<std::size_t>(lane)]++ = zapDenormal(finiteOrZero(selected[static_cast<std::size_t>(lane)]));
        }
    }

    void onSetPins() override
    {
        pinSignalOut.setStreaming(true);
        pinLeftOut.setStreaming(true);
        pinRightOut.setStreaming(true);
        for (auto& pin : pinLaneOut)
            pin.setStreaming(true);

        setSleep(false);
        setSubProcess(&ScatteringDiffuser::subProcess);
    }

private:
    void updateSampleRateIfNeeded()
    {
        const float newRate = (std::max)(1000.0f, finiteOrZero(getSampleRate()));
        if (std::abs(newRate - currentSampleRate_) < 0.5f)
            return;

        currentSampleRate_ = newRate;

        densitySmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        toneSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        masterSizeSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        matrixEnableSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        matrixAmountSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        shuffleTimeSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        shufflePolaritySmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        stereoSpreadSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);

        for (auto& smoother : delaySmoothers_)
            smoother.setTime(kDelaySmoothingMs, currentSampleRate_);
        for (auto& smoother : randomOffsetSmoothers_)
            smoother.setTime(kRandomOffsetSmoothingMs, currentSampleRate_);

        for (auto& delay : delays_)
            delay.clear();

        for (int i = 0; i < kDelayCount; ++i)
        {
            toneFilters_[static_cast<std::size_t>(i)].clear();
            toneFilters_[static_cast<std::size_t>(i)].setSampleRate(currentSampleRate_, i / kLanes);
        }
    }

    void regeneratePattern(std::uint32_t seed)
    {
        activeSeed_ = seed == 0 ? 0x5bd1e995u : seed;
        std::uint32_t state = activeSeed_;

        for (int i = 0; i < kDelayCount; ++i)
        {
            float r = randomBipolar(state);
            if (std::abs(r) < 0.10f)
                r = r < 0.0f ? -0.10f : 0.10f;
            randomPattern_[static_cast<std::size_t>(i)] = r;
        }

        for (int bank = 0; bank < kBanks; ++bank)
        {
            for (int lane = 0; lane < kLanes; ++lane)
            {
                timePermutation_[static_cast<std::size_t>(bank)][static_cast<std::size_t>(lane)] = lane;
                inputSigns_[static_cast<std::size_t>(bank)][static_cast<std::size_t>(lane)] =
                    randomBipolar(state) >= 0.0f ? 1.0f : -1.0f;
                outputSigns_[static_cast<std::size_t>(bank)][static_cast<std::size_t>(lane)] =
                    randomBipolar(state) >= 0.0f ? 1.0f : -1.0f;
            }

            auto& p = timePermutation_[static_cast<std::size_t>(bank)];
            for (int i = kLanes - 1; i > 0; --i)
            {
                const int j = static_cast<int>(xorshift32(state) % static_cast<std::uint32_t>(i + 1));
                std::swap(p[static_cast<std::size_t>(i)], p[static_cast<std::size_t>(j)]);
            }
        }
    }

    // Inputs / controls.
    AudioInPin pinSignalIn;
    AudioInPin pinDensity;
    AudioInPin pinTone;
    AudioInPin pinMasterSize;
    AudioInPin pinRandomOffsetMs;
    AudioInPin pinRandomSeed;
    AudioInPin pinMatrixEnable;
    AudioInPin pinMatrixAmount;
    AudioInPin pinShuffleTime;
    AudioInPin pinShufflePolarity;
    AudioInPin pinStereoSpread;
    std::array<AudioInPin, kDelayCount> pinDelayMs;

    // Outputs.
    AudioOutPin pinSignalOut;
    AudioOutPin pinLeftOut;
    AudioOutPin pinRightOut;
    std::array<AudioOutPin, kLanes> pinLaneOut;

    // DSP.
    std::array<FractionalDelay, kDelayCount> delays_;
    std::array<ToneFilter, kDelayCount> toneFilters_;
    std::array<SmoothedValue, kDelayCount> delaySmoothers_;
    std::array<SmoothedValue, kDelayCount> randomOffsetSmoothers_;

    SmoothedValue densitySmoother_;
    SmoothedValue toneSmoother_;
    SmoothedValue masterSizeSmoother_;
    SmoothedValue matrixEnableSmoother_;
    SmoothedValue matrixAmountSmoother_;
    SmoothedValue shuffleTimeSmoother_;
    SmoothedValue shufflePolaritySmoother_;
    SmoothedValue stereoSpreadSmoother_;

    std::array<float, kDelayCount> randomPattern_{};
    std::array<std::array<int, kLanes>, kBanks> timePermutation_{{
        {{0, 1, 2, 3}}, {{0, 1, 2, 3}}, {{0, 1, 2, 3}}, {{0, 1, 2, 3}}
    }};
    std::array<std::array<float, kLanes>, kBanks> inputSigns_{{
        {{1, 1, 1, 1}}, {{1, 1, 1, 1}}, {{1, 1, 1, 1}}, {{1, 1, 1, 1}}
    }};
    std::array<std::array<float, kLanes>, kBanks> outputSigns_{{
        {{1, 1, 1, 1}}, {{1, 1, 1, 1}}, {{1, 1, 1, 1}}, {{1, 1, 1, 1}}
    }};

    // Fixed row permutations: still orthonormal, but each bank presents a different matrix orientation.
    const std::array<std::array<int, kLanes>, kBanks> fixedMatrixPermutation_{{
        {{0, 1, 2, 3}},
        {{1, 3, 0, 2}},
        {{2, 0, 3, 1}},
        {{3, 2, 1, 0}}
    }};

    std::uint32_t autoSeed_ = 1u;
    std::uint32_t activeSeed_ = 0u;
    float currentSampleRate_ = 0.0f;
};

namespace
{
    auto registration =
        Register<ScatteringDiffuser>::withId(L"Pandocrator Scattering Diffuser v1");
}
