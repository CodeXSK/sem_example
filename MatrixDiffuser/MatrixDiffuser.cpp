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
    constexpr int   kStages = 8;
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;
    constexpr float kMaxDelayMs = 500.0f;
    constexpr float kMaxSupportedSampleRate = 384000.0f;
    constexpr float kMaxDiffusion = 0.97f;
    constexpr float kMaxFeedback = 0.95f;
    constexpr float kMaxTone = 1.0f;
    constexpr float kMaxRandomOffsetMs = 20.0f;
    constexpr float kMinMasterSize = 0.25f;
    constexpr float kMaxMasterSize = 4.0f;
    constexpr float kMaxMicroModMs = 5.0f;
    constexpr float kMinMicroRateHz = 0.01f;
    constexpr float kMaxMicroRateHz = 2.0f;
    constexpr float kParameterSmoothingMs = 15.0f;
    constexpr float kDelaySmoothingMs = 20.0f;
    constexpr float kRandomOffsetSmoothingMs = 100.0f;

    // SynthEdit audio/control pins use internal units where 1.0 = 10 V shown.
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

    class AllPassStage
    {
    public:
        AllPassStage()
            : buffer_(static_cast<std::size_t>(kDelayBufferSamples), 0.0f)
        {
        }

        void clear()
        {
            std::fill(buffer_.begin(), buffer_.end(), 0.0f);
            writeIndex_ = 0;
        }

        float process(float input, float g, float delaySamples)
        {
            input = finiteOrZero(input);
            g = clampf(finiteOrZero(g), -kMaxDiffusion, kMaxDiffusion);

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
            const float delayed = d0 + fraction * (d1 - d0);

            // Schroeder all-pass:
            // y[n] = delayed - g*x[n]
            // v[n] = x[n] + g*y[n]
            float output = delayed - g * input;
            output = finiteOrZero(output);

            float writeValue = input + g * output;
            writeValue = zapDenormal(finiteOrZero(writeValue));
            buffer_[static_cast<std::size_t>(writeIndex_)] = writeValue;

            ++writeIndex_;
            if (writeIndex_ >= kDelayBufferSamples)
                writeIndex_ = 0;

            return zapDenormal(output);
        }

    private:
        std::vector<float> buffer_;
        int writeIndex_ = 0;
    };

    // Gentle per-stage damping/tilt. Tone = 0 leaves the all-pass spectrum alone.
    class ToneFilter
    {
    public:
        void clear()
        {
            darkState_ = 0.0f;
            brightLowState_ = 0.0f;
        }

        void setSampleRate(float sampleRate, int stageIndex)
        {
            sampleRate_ = (std::max)(1000.0f, sampleRate);
            const float progress = clampf(
                (static_cast<float>(stageIndex) + 1.0f) / static_cast<float>(kStages),
                0.125f,
                1.0f);

            const float darkCutoffHz = 15000.0f + progress * (5000.0f - 15000.0f);
            const float brightCutoffHz = 30.0f + progress * (220.0f - 30.0f);

            darkCoeff_ = onePoleCoeff(darkCutoffHz, sampleRate_);
            brightCoeff_ = onePoleCoeff(brightCutoffHz, sampleRate_);
            progress_ = progress;
        }

        float process(float input, float tone)
        {
            input = finiteOrZero(input);
            tone = clampf(finiteOrZero(tone), -kMaxTone, kMaxTone);

            darkState_ += darkCoeff_ * (input - darkState_);
            darkState_ = zapDenormal(finiteOrZero(darkState_));

            brightLowState_ += brightCoeff_ * (input - brightLowState_);
            brightLowState_ = zapDenormal(finiteOrZero(brightLowState_));
            const float highPassed = finiteOrZero(input - brightLowState_);

            // Intentionally gentle. The second bank is coloured more than the first.
            const float darkMix = (std::max)(0.0f, -tone) * (0.08f + 0.28f * progress_);
            const float brightMix = (std::max)(0.0f, tone) * (0.05f + 0.16f * progress_);

            float output = input;
            output += darkMix * (darkState_ - output);
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
        float darkState_ = 0.0f;
        float brightLowState_ = 0.0f;
        float darkCoeff_ = 1.0f;
        float brightCoeff_ = 0.01f;
        float progress_ = 0.125f;
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
}

// Pandocrator Matrix Diffuser
// ---------------------------
// Matrix diffusion network with OPTIONAL global output-to-input feedback.
// Feedback defaults to zero, so v2 behaves like v1 until explicitly raised.
//
//            +--> AP1 --+                 +--> AP5 --+
//            +--> AP2 --+--> 4x4 matrix --+--> AP6 --+--> decode --> outputs
// Input -----+--> AP3 --+                 +--> AP7 --+
//            +--> AP4 --+                 +--> AP8 --+
//
// Bank A and Bank B are four PARALLEL all-pass lanes. The normalized Hadamard
// transform redistributes energy between the lanes before Bank B. This avoids
// repeatedly sending the entire signal through one long serial path, which was
// a major source of obvious periodic/metallic ringing in the earlier design.
class MatrixDiffuser final : public MpBase2
{
public:
    MatrixDiffuser()
    {
        // Pin order MUST match MatrixDiffuser.xml.
        initializePin(pinSignalIn);
        initializePin(pinFeedback);
        initializePin(pinDiffusion);
        initializePin(pinTone);
        initializePin(pinMasterSize);
        initializePin(pinRandomOffsetMs);
        initializePin(pinRandomSeed);
        initializePin(pinMatrixEnable);
        initializePin(pinMatrixAmount);
        initializePin(pinShuffleTime);
        initializePin(pinShufflePolarity);
        initializePin(pinMicroModDepthMs);
        initializePin(pinMicroModRateHz);
        initializePin(pinStereoSpread);

        for (auto& pin : pinDelayMs)
            initializePin(pin);

        initializePin(pinSignalOut);
        initializePin(pinLeftOut);
        initializePin(pinRightOut);

        for (auto& pin : pinLaneOut)
            initializePin(pin);

        const std::array<float, kStages> defaults = {
            4.71f, 7.83f, 11.19f, 13.67f,
            17.29f, 19.93f, 23.47f, 29.71f
        };

        for (int i = 0; i < kStages; ++i)
        {
            delaySmoothers_[static_cast<std::size_t>(i)].reset(defaults[static_cast<std::size_t>(i)]);
            randomOffsetSmoothers_[static_cast<std::size_t>(i)].reset(0.0f);
        }

        feedbackSmoother_.reset(0.0f);
        diffusionSmoother_.reset(0.58f);
        toneSmoother_.reset(-0.15f);
        masterSizeSmoother_.reset(1.0f);
        matrixEnableSmoother_.reset(1.0f);
        matrixAmountSmoother_.reset(1.0f);
        shuffleTimeSmoother_.reset(0.0f);
        shufflePolaritySmoother_.reset(0.0f);
        microModDepthSmoother_.reset(0.0f);
        microModRateSmoother_.reset(0.10f);
        stereoSpreadSmoother_.reset(1.0f);

        const auto now = static_cast<std::uint64_t>(
            std::chrono::high_resolution_clock::now().time_since_epoch().count());
        const auto self = static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(this));
        autoSeed_ = static_cast<std::uint32_t>((now ^ (self + 0x9e3779b97f4a7c15ULL)) & 0xffffffffu);
        if (autoSeed_ == 0)
            autoSeed_ = 0x41c64e6du;

        regeneratePattern(autoSeed_);

        // Different starting phases even when Micro Mod is later enabled.
        for (int i = 0; i < kStages; ++i)
            lfoPhase_[static_cast<std::size_t>(i)] =
                std::fmod(0.713f + static_cast<float>(i) * 1.173f, kTwoPi);
    }

    void subProcess(int sampleFrames)
    {
        updateSampleRateIfNeeded();

        auto signalIn = getBuffer(pinSignalIn);
        auto feedbackPin = getBuffer(pinFeedback);
        auto diffusionPin = getBuffer(pinDiffusion);
        auto tonePin = getBuffer(pinTone);
        auto masterSizePin = getBuffer(pinMasterSize);
        auto randomOffsetPin = getBuffer(pinRandomOffsetMs);
        auto randomSeedPin = getBuffer(pinRandomSeed);
        auto matrixEnablePin = getBuffer(pinMatrixEnable);
        auto matrixAmountPin = getBuffer(pinMatrixAmount);
        auto shuffleTimePin = getBuffer(pinShuffleTime);
        auto shufflePolarityPin = getBuffer(pinShufflePolarity);
        auto microModDepthPin = getBuffer(pinMicroModDepthMs);
        auto microModRatePin = getBuffer(pinMicroModRateHz);
        auto stereoSpreadPin = getBuffer(pinStereoSpread);

        std::array<float*, kStages> delayPins{};
        for (int i = 0; i < kStages; ++i)
            delayPins[static_cast<std::size_t>(i)] = getBuffer(pinDelayMs[static_cast<std::size_t>(i)]);

        auto signalOut = getBuffer(pinSignalOut);
        auto leftOut = getBuffer(pinLeftOut);
        auto rightOut = getBuffer(pinRightOut);

        std::array<float*, kLanes> laneOutputs{};
        for (int i = 0; i < kLanes; ++i)
            laneOutputs[static_cast<std::size_t>(i)] = getBuffer(pinLaneOut[static_cast<std::size_t>(i)]);

        feedbackSmoother_.setTarget(clampf(
            finiteOrZero(*feedbackPin) * kInternalToDisplayedUnits,
            -kMaxFeedback,
            kMaxFeedback));

        diffusionSmoother_.setTarget(clampf(
            finiteOrZero(*diffusionPin) * kInternalToDisplayedUnits,
            -kMaxDiffusion,
            kMaxDiffusion));

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

        microModDepthSmoother_.setTarget(clampf(
            finiteOrZero(*microModDepthPin) * kInternalToDisplayedUnits,
            0.0f,
            kMaxMicroModMs));

        microModRateSmoother_.setTarget(clampf(
            finiteOrZero(*microModRatePin) * kInternalToDisplayedUnits,
            kMinMicroRateHz,
            kMaxMicroRateHz));

        stereoSpreadSmoother_.setTarget(clampf(
            finiteOrZero(*stereoSpreadPin) * kInternalToDisplayedUnits,
            0.0f,
            1.0f));

        for (int i = 0; i < kStages; ++i)
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
            const float dryInput = finiteOrZero(*signalIn++);
            const float feedback = feedbackSmoother_.next();
            const float input = finiteOrZero(dryInput + feedback * feedbackState_);
            const float g = diffusionSmoother_.next();
            const float tone = toneSmoother_.next();
            const float size = masterSizeSmoother_.next();
            const float matrixEnable = matrixEnableSmoother_.next();
            const float matrixAmount = matrixAmountSmoother_.next() * matrixEnable;
            const float shuffleTime = shuffleTimeSmoother_.next();
            const float shufflePolarity = shufflePolaritySmoother_.next();
            const float microDepthMs = microModDepthSmoother_.next();
            const float microRateHz = microModRateSmoother_.next();
            const float stereoSpread = stereoSpreadSmoother_.next();

            std::array<float, kStages> currentDelayMs{};
            for (int i = 0; i < kStages; ++i)
            {
                const float base = delaySmoothers_[static_cast<std::size_t>(i)].next();
                const float randomOffset = randomOffsetSmoothers_[static_cast<std::size_t>(i)].next();
                currentDelayMs[static_cast<std::size_t>(i)] = base + randomOffset;
            }

            // BANK A: four PARALLEL all-pass lanes fed by the same input.
            std::array<float, kLanes> bankA{};
            for (int lane = 0; lane < kLanes; ++lane)
            {
                const int directIndex = lane;
                const int shuffledIndex = bankAPermutation_[static_cast<std::size_t>(lane)];
                const float directMs = currentDelayMs[static_cast<std::size_t>(directIndex)];
                const float shuffledMs = currentDelayMs[static_cast<std::size_t>(shuffledIndex)];
                const float selectedMs = directMs + shuffleTime * (shuffledMs - directMs);

                const float mod = nextMicroLfo(lane, microRateHz);
                const float delayMs = clampf(
                    selectedMs * size + mod * microDepthMs * microDepthScale_[static_cast<std::size_t>(lane)],
                    0.1f,
                    kMaxDelayMs);

                float y = stages_[static_cast<std::size_t>(lane)].process(
                    input,
                    g,
                    delayMs * 0.001f * currentSampleRate_);
                y = toneFilters_[static_cast<std::size_t>(lane)].process(y, tone);
                bankA[static_cast<std::size_t>(lane)] = y;
            }

            // Optional source permutation into the matrix. Shuffle Time crossfades
            // toward the seed-derived mapping instead of switching abruptly.
            std::array<float, kLanes> matrixInput{};
            for (int lane = 0; lane < kLanes; ++lane)
            {
                const float direct = bankA[static_cast<std::size_t>(lane)];
                const float shuffled = bankA[static_cast<std::size_t>(
                    matrixSourcePermutation_[static_cast<std::size_t>(lane)])];
                float v = direct + shuffleTime * (shuffled - direct);

                // Smoothly morph +1 to the requested random sign.
                const float sign = 1.0f + shufflePolarity *
                    (matrixInputSigns_[static_cast<std::size_t>(lane)] - 1.0f);
                matrixInput[static_cast<std::size_t>(lane)] = v * sign;
            }

            // Normalized/orthonormal 4x4 Hadamard transform.
            std::array<float, kLanes> h{};
            h[0] = 0.5f * (matrixInput[0] + matrixInput[1] + matrixInput[2] + matrixInput[3]);
            h[1] = 0.5f * (matrixInput[0] - matrixInput[1] + matrixInput[2] - matrixInput[3]);
            h[2] = 0.5f * (matrixInput[0] + matrixInput[1] - matrixInput[2] - matrixInput[3]);
            h[3] = 0.5f * (matrixInput[0] - matrixInput[1] - matrixInput[2] + matrixInput[3]);

            std::array<float, kLanes> bankBInput{};
            for (int lane = 0; lane < kLanes; ++lane)
            {
                const float sign = 1.0f + shufflePolarity *
                    (matrixOutputSigns_[static_cast<std::size_t>(lane)] - 1.0f);
                const float matrixSignal = h[static_cast<std::size_t>(lane)] * sign;

                // MatrixAmount = 0 -> four independent parallel paths.
                // MatrixAmount = 1 -> full normalized Hadamard mixing.
                bankBInput[static_cast<std::size_t>(lane)] =
                    bankA[static_cast<std::size_t>(lane)] +
                    matrixAmount * (matrixSignal - bankA[static_cast<std::size_t>(lane)]);
            }

            // BANK B: another four PARALLEL all-pass lanes.
            std::array<float, kLanes> bankB{};
            for (int lane = 0; lane < kLanes; ++lane)
            {
                const int directIndex = 4 + lane;
                const int shuffledIndex = 4 + bankBPermutation_[static_cast<std::size_t>(lane)];
                const float directMs = currentDelayMs[static_cast<std::size_t>(directIndex)];
                const float shuffledMs = currentDelayMs[static_cast<std::size_t>(shuffledIndex)];
                const float selectedMs = directMs + shuffleTime * (shuffledMs - directMs);

                const int stageIndex = 4 + lane;
                const float mod = nextMicroLfo(stageIndex, microRateHz);
                const float delayMs = clampf(
                    selectedMs * size + mod * microDepthMs * microDepthScale_[static_cast<std::size_t>(stageIndex)],
                    0.1f,
                    kMaxDelayMs);

                float y = stages_[static_cast<std::size_t>(stageIndex)].process(
                    bankBInput[static_cast<std::size_t>(lane)],
                    g,
                    delayMs * 0.001f * currentSampleRate_);
                y = toneFilters_[static_cast<std::size_t>(stageIndex)].process(y, tone);
                bankB[static_cast<std::size_t>(lane)] = y;
            }

            // Mono-safe decode: averaging four lanes avoids the large gain build-up
            // that a raw sum would create when the lanes are still correlated.
            const float mono = 0.25f * (bankB[0] + bankB[1] + bankB[2] + bankB[3]);

            // Two complementary lane pairs form a mono-to-stereo diffuser output.
            const float leftWide = 0.5f * (bankB[0] + bankB[2]);
            const float rightWide = 0.5f * (bankB[1] + bankB[3]);
            const float left = mono + stereoSpread * (leftWide - mono);
            const float right = mono + stereoSpread * (rightWide - mono);

            // One-sample state closes the internal output-to-input feedback loop
            // without creating an algebraic zero-delay loop.
            feedbackState_ = zapDenormal(finiteOrZero(mono));

            *signalOut++ = feedbackState_;
            *leftOut++ = zapDenormal(finiteOrZero(left));
            *rightOut++ = zapDenormal(finiteOrZero(right));

            for (int lane = 0; lane < kLanes; ++lane)
                *laneOutputs[static_cast<std::size_t>(lane)]++ = bankB[static_cast<std::size_t>(lane)];
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
        setSubProcess(&MatrixDiffuser::subProcess);
    }

private:
    void updateSampleRateIfNeeded()
    {
        const float newRate = (std::max)(1000.0f, finiteOrZero(getSampleRate()));
        if (std::abs(newRate - currentSampleRate_) < 0.5f)
            return;

        currentSampleRate_ = newRate;

        feedbackSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        diffusionSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        toneSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        masterSizeSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        matrixEnableSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        matrixAmountSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        shuffleTimeSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        shufflePolaritySmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        microModDepthSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        microModRateSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        stereoSpreadSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);

        for (auto& smoother : delaySmoothers_)
            smoother.setTime(kDelaySmoothingMs, currentSampleRate_);
        for (auto& smoother : randomOffsetSmoothers_)
            smoother.setTime(kRandomOffsetSmoothingMs, currentSampleRate_);

        feedbackState_ = 0.0f;

        for (auto& stage : stages_)
            stage.clear();

        for (int i = 0; i < kStages; ++i)
        {
            toneFilters_[static_cast<std::size_t>(i)].clear();
            toneFilters_[static_cast<std::size_t>(i)].setSampleRate(currentSampleRate_, i);
        }
    }

    float nextMicroLfo(int stageIndex, float baseRateHz)
    {
        const std::size_t i = static_cast<std::size_t>(stageIndex);
        const float result = std::sin(lfoPhase_[i]);
        const float rate = clampf(
            baseRateHz * microRateScale_[i],
            kMinMicroRateHz,
            kMaxMicroRateHz * 2.5f);

        lfoPhase_[i] += kTwoPi * rate / currentSampleRate_;
        if (lfoPhase_[i] >= kTwoPi)
            lfoPhase_[i] -= kTwoPi;

        return result;
    }

    void regeneratePattern(std::uint32_t seed)
    {
        activeSeed_ = seed == 0 ? 0x41c64e6du : seed;
        std::uint32_t state = activeSeed_;

        for (int i = 0; i < kStages; ++i)
        {
            float r = randomBipolar(state);
            if (std::abs(r) < 0.12f)
                r = r < 0.0f ? -0.12f : 0.12f;
            randomPattern_[static_cast<std::size_t>(i)] = r;
        }

        for (int i = 0; i < kLanes; ++i)
        {
            bankAPermutation_[static_cast<std::size_t>(i)] = i;
            bankBPermutation_[static_cast<std::size_t>(i)] = i;
            matrixSourcePermutation_[static_cast<std::size_t>(i)] = i;
            matrixInputSigns_[static_cast<std::size_t>(i)] = randomBipolar(state) >= 0.0f ? 1.0f : -1.0f;
            matrixOutputSigns_[static_cast<std::size_t>(i)] = randomBipolar(state) >= 0.0f ? 1.0f : -1.0f;
        }

        auto shuffle4 = [&state](std::array<int, kLanes>& p)
        {
            for (int i = kLanes - 1; i > 0; --i)
            {
                const int j = static_cast<int>(xorshift32(state) % static_cast<std::uint32_t>(i + 1));
                std::swap(p[static_cast<std::size_t>(i)], p[static_cast<std::size_t>(j)]);
            }
        };

        shuffle4(bankAPermutation_);
        shuffle4(bankBPermutation_);
        shuffle4(matrixSourcePermutation_);
    }

    // Inputs / controls.
    AudioInPin pinSignalIn;
    AudioInPin pinFeedback;
    AudioInPin pinDiffusion;
    AudioInPin pinTone;
    AudioInPin pinMasterSize;
    AudioInPin pinRandomOffsetMs;
    AudioInPin pinRandomSeed;
    AudioInPin pinMatrixEnable;
    AudioInPin pinMatrixAmount;
    AudioInPin pinShuffleTime;
    AudioInPin pinShufflePolarity;
    AudioInPin pinMicroModDepthMs;
    AudioInPin pinMicroModRateHz;
    AudioInPin pinStereoSpread;
    std::array<AudioInPin, kStages> pinDelayMs;

    // Outputs.
    AudioOutPin pinSignalOut;
    AudioOutPin pinLeftOut;
    AudioOutPin pinRightOut;
    std::array<AudioOutPin, kLanes> pinLaneOut;

    // DSP.
    std::array<AllPassStage, kStages> stages_;
    std::array<ToneFilter, kStages> toneFilters_;
    std::array<SmoothedValue, kStages> delaySmoothers_;
    std::array<SmoothedValue, kStages> randomOffsetSmoothers_;

    SmoothedValue feedbackSmoother_;
    SmoothedValue diffusionSmoother_;
    SmoothedValue toneSmoother_;
    SmoothedValue masterSizeSmoother_;
    SmoothedValue matrixEnableSmoother_;
    SmoothedValue matrixAmountSmoother_;
    SmoothedValue shuffleTimeSmoother_;
    SmoothedValue shufflePolaritySmoother_;
    SmoothedValue microModDepthSmoother_;
    SmoothedValue microModRateSmoother_;
    SmoothedValue stereoSpreadSmoother_;

    std::array<float, kStages> randomPattern_{};
    std::array<int, kLanes> bankAPermutation_{{0, 1, 2, 3}};
    std::array<int, kLanes> bankBPermutation_{{0, 1, 2, 3}};
    std::array<int, kLanes> matrixSourcePermutation_{{0, 1, 2, 3}};
    std::array<float, kLanes> matrixInputSigns_{{1.0f, 1.0f, 1.0f, 1.0f}};
    std::array<float, kLanes> matrixOutputSigns_{{1.0f, 1.0f, 1.0f, 1.0f}};

    std::array<float, kStages> lfoPhase_{};
    const std::array<float, kStages> microRateScale_{{
        0.73f, 0.89f, 1.03f, 1.19f, 1.37f, 1.57f, 1.79f, 2.03f
    }};
    const std::array<float, kStages> microDepthScale_{{
        1.00f, -0.71f, 0.47f, -0.86f, 0.63f, -1.00f, 0.54f, -0.78f
    }};

    std::uint32_t autoSeed_ = 1u;
    std::uint32_t activeSeed_ = 0u;
    float feedbackState_ = 0.0f;
    float currentSampleRate_ = 0.0f;
};

namespace
{
    auto registration =
        Register<MatrixDiffuser>::withId(L"Pandocrator Matrix Diffuser v2");
}
