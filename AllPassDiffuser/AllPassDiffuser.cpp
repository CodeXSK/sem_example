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
    constexpr int   kMaxStages = 10;
    constexpr int   kMinStages = 4;
    constexpr float kMaxDelayMs = 500.0f;
    constexpr float kMaxSupportedSampleRate = 384000.0f;
    constexpr int   kMaxOversamplingFactor = 4;
    constexpr float kMaxDiffusion = 0.97f;
    constexpr float kMaxFeedback = 0.90f;
    constexpr float kParameterSmoothingMs = 12.0f;
    constexpr float kStageCrossfadeMs = 20.0f;
    constexpr float kMaxTone = 1.0f;
    constexpr float kDefaultFeedbackDamping = 0.25f;
    constexpr float kMaxRandomOffsetMs = 20.0f;
    constexpr float kRandomOffsetSmoothingMs = 80.0f;
    constexpr float kMinMasterSize = 0.25f;
    constexpr float kMaxMasterSize = 4.0f;
    // SynthEdit internal audio/CV units: 1.0 == 10 V in the editor.
    constexpr float kInternalToDisplayedUnits = 10.0f;

    // Enough storage for 500 ms at 384 kHz, plus interpolation guard samples.
    constexpr int kDelayBufferSamples =
        static_cast<int>(kMaxSupportedSampleRate * static_cast<float>(kMaxOversamplingFactor) * (kMaxDelayMs * 0.001f)) + 8;

    inline float clampf(float x, float lo, float hi)
    {
        return (std::max)(lo, (std::min)(hi, x));
    }

    inline int clampi(int x, int lo, int hi)
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

    // One-pole parameter smoother. The coefficient is chosen so the remaining
    // error is about e^-1 after timeMs. It is intentionally modest: fast enough
    // for knobs, slow enough to suppress zipper noise.
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

    class ModulatedAllPassStage
    {
    public:
        ModulatedAllPassStage()
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

        float process(float input, float g, float delaySamples)
        {
            input = finiteOrZero(input);
            g = clampf(finiteOrZero(g), -kMaxDiffusion, kMaxDiffusion);

            // v9: Thiran all-pass fractional delay. Unlike linear interpolation,
            // this has unity magnitude for static delay values, so cascading many
            // stages does not silently turn the diffuser into a low-pass filter.
            const float maxDelay = static_cast<float>(kDelayBufferSamples - 4);
            delaySamples = clampf(finiteOrZero(delaySamples), 1.0f, maxDelay);

            int integerDelay = static_cast<int>(std::floor(delaySamples));
            float fraction = delaySamples - static_cast<float>(integerDelay);
            integerDelay = clampi(integerDelay, 1, kDelayBufferSamples - 4);
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
                fractionalInputPrev_ = integerDelayed;
                fractionalOutputPrev_ = integerDelayed;
            }

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
        float fractionalInputPrev_ = 0.0f;
        float fractionalOutputPrev_ = 0.0f;
    };

    class OutputToneFilter
    {
    public:
        void clear()
        {
            lowState_ = 0.0f;
        }

        void setSampleRate(float sampleRate)
        {
            sampleRate_ = (std::max)(1000.0f, sampleRate);
            const float cutoffHz = 1800.0f;
            const float fc = clampf(cutoffHz, 20.0f, 0.45f * sampleRate_);
            lowCoeff_ = 1.0f - std::exp(
                -2.0f * 3.14159265358979323846f * fc / sampleRate_);
        }

        float process(float input, float tone)
        {
            input = finiteOrZero(input);
            tone = clampf(finiteOrZero(tone), -1.0f, 1.0f);

            lowState_ += lowCoeff_ * (input - lowState_);
            lowState_ = zapDenormal(finiteOrZero(lowState_));
            const float high = input - lowState_;

            float lowGain = 1.0f;
            float highGain = 1.0f;

            if (tone >= 0.0f)
            {
                // Positive Tone is now a real bright tilt, applied only after
                // the recursive network so it cannot destabilise feedback.
                lowGain = dbToGain(-2.0f * tone);
                highGain = dbToGain(+4.0f * tone);
            }
            else
            {
                const float dark = -tone;
                lowGain = dbToGain(+0.5f * dark);
                highGain = dbToGain(-8.0f * dark);
            }

            return zapDenormal(finiteOrZero(
                lowState_ * lowGain + high * highGain));
        }

    private:
        static float dbToGain(float dB)
        {
            return std::pow(10.0f, dB / 20.0f);
        }

        float sampleRate_ = 44100.0f;
        float lowState_ = 0.0f;
        float lowCoeff_ = 0.1f;
    };

    class DiffuseFeedbackPath
    {
    public:
        DiffuseFeedbackPath()
            : buffer_(static_cast<std::size_t>(
                static_cast<int>(
                    kMaxSupportedSampleRate
                    * static_cast<float>(kMaxOversamplingFactor)
                    * 0.020f) + 16),
                0.0f)
        {
        }

        void clear()
        {
            std::fill(buffer_.begin(), buffer_.end(), 0.0f);
            writeIndex_ = 0;
            lowState_ = 0.0f;
        }

        void setSampleRate(float sampleRate)
        {
            sampleRate_ = (std::max)(1000.0f, sampleRate);
            tap1_ = msToSamples(2.31f);
            tap2_ = msToSamples(5.83f);
            tap3_ = msToSamples(9.47f);
        }

        float process(float input, float damping)
        {
            input = zapDenormal(finiteOrZero(input));
            damping = clampf(finiteOrZero(damping), 0.0f, 1.0f);

            // Feed-forward smear: several return path lengths, no new recursive
            // poles. Coefficients are L1-normalised (sum of magnitudes = 1).
            const float d1 = readTap(tap1_);
            const float d2 = readTap(tap2_);
            const float d3 = readTap(tap3_);

            const float smeared =
                0.45f * input
                + 0.25f * d1
                - 0.18f * d2
                + 0.12f * d3;

            buffer_[static_cast<std::size_t>(writeIndex_)] = input;
            ++writeIndex_;
            if (writeIndex_ >= static_cast<int>(buffer_.size()))
                writeIndex_ = 0;

            // 0 = almost open/bright, 1 = strongly damped.
            const float highCutHz =
                20000.0f * std::pow(3500.0f / 20000.0f, damping);
            const float fc = clampf(highCutHz, 1000.0f, 0.45f * sampleRate_);
            const float coeff = 1.0f - std::exp(
                -2.0f * 3.14159265358979323846f * fc / sampleRate_);

            lowState_ += coeff * (smeared - lowState_);
            lowState_ = zapDenormal(finiteOrZero(lowState_));
            return lowState_;
        }

    private:
        int msToSamples(float ms) const
        {
            const int maxTap = static_cast<int>(buffer_.size()) - 2;
            return clampi(
                static_cast<int>(std::lround(ms * 0.001f * sampleRate_)),
                1,
                maxTap);
        }

        float readTap(int delaySamples) const
        {
            int index = writeIndex_ - delaySamples;
            while (index < 0)
                index += static_cast<int>(buffer_.size());
            if (index >= static_cast<int>(buffer_.size()))
                index %= static_cast<int>(buffer_.size());

            return finiteOrZero(buffer_[static_cast<std::size_t>(index)]);
        }

        std::vector<float> buffer_;
        int writeIndex_ = 0;
        int tap1_ = 1;
        int tap2_ = 2;
        int tap3_ = 3;
        float sampleRate_ = 44100.0f;
        float lowState_ = 0.0f;
    };

    // Tiny deterministic PRNG used only to create a static per-instance delay pattern.
    // It never runs in the sample loop.
    inline std::uint32_t xorshift32(std::uint32_t& state)
    {
        if (state == 0)
            state = 0x6d2b79f5u;
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    inline float randomBipolar(std::uint32_t& state)
    {
        const std::uint32_t v = xorshift32(state);
        const float unit = static_cast<float>(v & 0x00ffffffu) / 16777215.0f;
        return unit * 2.0f - 1.0f;
    }

}

// Multi-stage all-pass diffuser for SynthEdit.
//
// Routing 0 (Series):
//     AP1 -> AP2 -> ... -> AP10, with the selected Stages tap used as Signal Out.
//
// Routing 1 (4 Out / 4 In):
//     The main chain is still continuous. AP1..AP4 are exposed as Bank A Out 1..4.
//     Bank B In 1..4 are ADDITIVE injection points before AP5..AP8. Therefore the
//     normal AP4 -> AP5 -> AP6 -> AP7 -> AP8 series path remains intact even when
//     the external inputs are left disconnected.
//
// Feedback is internal and sample-by-sample: the previous Signal Out sample is
// multiplied by Feedback and added to the new input before AP1. This avoids an
// external SynthEdit feedback loop.
//
// Optional Cross Routes (disabled by default):
//     AP4 -> AP8 input
//     AP5 -> AP7 input
//     AP6 -> AP10 input
// These are causal feed-forward branches (source stages always occur before the
// destination stage), so they do not create an internal zero-delay feedback loop.
// Cross Amount controls how much of each source stage is ADDED at its destination.
//
// v7 additions (all optional):
//   Master Size scales all base/static-random delay times.
//   Hadamard Matrix mixes AP1..AP4 through a normalized 4x4 Hadamard transform
//   and blends its four outputs into AP5..AP8. Shuffle Time permutes the temporal
//   tap assignment, and Shuffle Polarity applies deterministic per-instance sign
//   flips. Both shuffles are derived from Random Seed.
//
// v9 corrections:
 //   Fractional delays use Thiran all-pass interpolation instead of linear
 //   interpolation, preventing cumulative HF loss through many stages.
 //   Tone is a post-network tilt EQ; +1 genuinely brightens the wet output.
 //   Feedback uses a multi-tap diffuse return bus + short feed-forward smear
 //   + adjustable HF damping instead of direct Output -> Input recirculation.
 //
 // v8 addition:
 //   Oversampling Factor runs the complete internal diffuser network at 1x, 2x,
//   3x or 4x the host sample rate. Audio-rate inputs are linearly interpolated
//   between host samples; the final internal sub-sample is emitted at the host
//   boundary. Physical delay times and filter cutoffs are preserved in seconds/Hz.
class AllPassDiffuser final : public MpBase2
{
public:
    AllPassDiffuser()
    {
        // Pin order MUST match AllPassDiffuser.xml.
        initializePin(pinSignalIn);
        initializePin(pinStages);
        initializePin(pinRouting);
        initializePin(pinDiffusion);
        initializePin(pinFeedback);
        initializePin(pinFeedbackDamping);
        initializePin(pinTone);
        initializePin(pinModIn);
        initializePin(pinModDepthMs);
        initializePin(pinRandomOffsetMs);
        initializePin(pinRandomSeed);
        initializePin(pinCrossRoutes);
        initializePin(pinCrossAmount);
        initializePin(pinMasterSize);
        initializePin(pinOversamplingFactor);
        initializePin(pinHadamardEnable);
        initializePin(pinHadamardAmount);
        initializePin(pinShuffleTime);
        initializePin(pinShufflePolarity);

        for (auto& pin : pinDelayMs)
            initializePin(pin);

        for (auto& pin : pinBankBIn)
            initializePin(pin);

        initializePin(pinSignalOut);

        for (auto& pin : pinBankAOut)
            initializePin(pin);

        const std::array<float, kMaxStages> defaults = {
            4.71f, 7.83f, 11.19f, 13.67f, 17.29f,
            19.93f, 23.47f, 29.71f, 31.13f, 37.91f
        };

        for (int i = 0; i < kMaxStages; ++i)
            delaySmoothers_[static_cast<std::size_t>(i)].reset(defaults[static_cast<std::size_t>(i)]);

        diffusionSmoother_.reset(0.65f);
        feedbackSmoother_.reset(0.0f);
        feedbackDampingSmoother_.reset(kDefaultFeedbackDamping);
        toneSmoother_.reset(0.0f);
        modDepthSmoother_.reset(0.0f);
        routingSmoother_.reset(0.0f);
        crossRoutesSmoother_.reset(0.0f);
        crossAmountSmoother_.reset(0.35f);
        masterSizeSmoother_.reset(1.0f);
        hadamardEnableSmoother_.reset(0.0f);
        hadamardAmountSmoother_.reset(0.25f);
        shuffleTimeSmoother_.reset(0.0f);
        shufflePolaritySmoother_.reset(0.0f);

        // Auto mode (Seed = 0): every newly created module instance gets its own
        // static decorrelation pattern. The visible delay pins remain unchanged.
        const auto now = static_cast<std::uint64_t>(
            std::chrono::high_resolution_clock::now().time_since_epoch().count());
        const auto self = static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(this));
        autoSeed_ = static_cast<std::uint32_t>((now ^ (self + 0x9e3779b97f4a7c15ULL)) & 0xffffffffu);
        if (autoSeed_ == 0)
            autoSeed_ = 0x41c64e6du;
        regenerateRandomPattern(autoSeed_);

        for (auto& smoother : randomOffsetSmoothers_)
            smoother.reset(0.0f);
    }

    void subProcess(int sampleFrames)
    {
        auto signalIn = getBuffer(pinSignalIn);
        auto stagesPin = getBuffer(pinStages);
        auto routingPin = getBuffer(pinRouting);
        auto diffusionPin = getBuffer(pinDiffusion);
        auto feedbackPin = getBuffer(pinFeedback);
        auto feedbackDampingPin = getBuffer(pinFeedbackDamping);
        auto tonePin = getBuffer(pinTone);
        auto modIn = getBuffer(pinModIn);
        auto modDepthPin = getBuffer(pinModDepthMs);
        auto randomOffsetPin = getBuffer(pinRandomOffsetMs);
        auto randomSeedPin = getBuffer(pinRandomSeed);
        auto crossRoutesPin = getBuffer(pinCrossRoutes);
        auto crossAmountPin = getBuffer(pinCrossAmount);
        auto masterSizePin = getBuffer(pinMasterSize);
        auto oversamplingPin = getBuffer(pinOversamplingFactor);
        auto hadamardEnablePin = getBuffer(pinHadamardEnable);
        auto hadamardAmountPin = getBuffer(pinHadamardAmount);
        auto shuffleTimePin = getBuffer(pinShuffleTime);
        auto shufflePolarityPin = getBuffer(pinShufflePolarity);

        std::array<float*, kMaxStages> delayPins{};
        for (int i = 0; i < kMaxStages; ++i)
            delayPins[static_cast<std::size_t>(i)] = getBuffer(pinDelayMs[static_cast<std::size_t>(i)]);

        std::array<float*, 4> bankBInputs{};
        for (int i = 0; i < 4; ++i)
            bankBInputs[static_cast<std::size_t>(i)] = getBuffer(pinBankBIn[static_cast<std::size_t>(i)]);

        auto signalOut = getBuffer(pinSignalOut);

        std::array<float*, 4> bankAOutputs{};
        for (int i = 0; i < 4; ++i)
            bankAOutputs[static_cast<std::size_t>(i)] = getBuffer(pinBankAOut[static_cast<std::size_t>(i)]);

        // Oversampling is a discrete block-level setting. Displayed values 1..4
        // map to factors x1..x4. Changing it resets time-domain state because the
        // delay memories were written on a different internal time grid.
        const int requestedOversampling = clampi(
            static_cast<int>(std::lround(
                finiteOrZero(*oversamplingPin) * kInternalToDisplayedUnits)),
            1,
            kMaxOversamplingFactor);
        updateProcessingRateIfNeeded(requestedOversampling);

        // Control-rate targets are sampled once per SynthEdit processing block.
        // Their transitions are smoothed internally sample-by-sample. At >1x the
        // smoothers run at the internal sample rate, preserving the same time in ms.
        const float stagesValue = finiteOrZero(*stagesPin) * kInternalToDisplayedUnits;
        const int requestedStages = clampi(
            static_cast<int>(std::lround(stagesValue)),
            kMinStages,
            kMaxStages);

        if (requestedStages != targetStages_)
        {
            previousStages_ = targetStages_;
            targetStages_ = requestedStages;
            stageCrossfade_ = 0.0f;
        }

        const float routingValue = finiteOrZero(*routingPin) * kInternalToDisplayedUnits;
        routingSmoother_.setTarget(routingValue >= 0.5f ? 1.0f : 0.0f);

        const float crossRoutesValue = finiteOrZero(*crossRoutesPin) * kInternalToDisplayedUnits;
        crossRoutesSmoother_.setTarget(crossRoutesValue >= 0.5f ? 1.0f : 0.0f);
        crossAmountSmoother_.setTarget(clampf(
            finiteOrZero(*crossAmountPin) * kInternalToDisplayedUnits, 0.0f, 1.0f));

        masterSizeSmoother_.setTarget(clampf(
            finiteOrZero(*masterSizePin) * kInternalToDisplayedUnits,
            kMinMasterSize, kMaxMasterSize));

        const float hadamardEnableValue = finiteOrZero(*hadamardEnablePin) * kInternalToDisplayedUnits;
        hadamardEnableSmoother_.setTarget(hadamardEnableValue >= 0.5f ? 1.0f : 0.0f);
        hadamardAmountSmoother_.setTarget(clampf(
            finiteOrZero(*hadamardAmountPin) * kInternalToDisplayedUnits, 0.0f, 1.0f));

        const float shuffleTimeValue = finiteOrZero(*shuffleTimePin) * kInternalToDisplayedUnits;
        shuffleTimeSmoother_.setTarget(shuffleTimeValue >= 0.5f ? 1.0f : 0.0f);
        const float shufflePolarityValue = finiteOrZero(*shufflePolarityPin) * kInternalToDisplayedUnits;
        shufflePolaritySmoother_.setTarget(shufflePolarityValue >= 0.5f ? 1.0f : 0.0f);

        diffusionSmoother_.setTarget(clampf(
            finiteOrZero(*diffusionPin) * kInternalToDisplayedUnits, -kMaxDiffusion, kMaxDiffusion));

        feedbackSmoother_.setTarget(clampf(
            finiteOrZero(*feedbackPin) * kInternalToDisplayedUnits, -kMaxFeedback, kMaxFeedback));

        feedbackDampingSmoother_.setTarget(clampf(
            finiteOrZero(*feedbackDampingPin) * kInternalToDisplayedUnits, 0.0f, 1.0f));

        toneSmoother_.setTarget(clampf(
            finiteOrZero(*tonePin) * kInternalToDisplayedUnits, -kMaxTone, kMaxTone));

        modDepthSmoother_.setTarget(clampf(
            finiteOrZero(*modDepthPin) * kInternalToDisplayedUnits, 0.0f, kMaxDelayMs));

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
            regenerateRandomPattern(effectiveSeed);

        for (int i = 0; i < kMaxStages; ++i)
            randomOffsetSmoothers_[static_cast<std::size_t>(i)].setTarget(
                randomPattern_[static_cast<std::size_t>(i)] * randomAmountMs);

        for (int i = 0; i < kMaxStages; ++i)
        {
            const float ms = clampf(
                finiteOrZero(*delayPins[static_cast<std::size_t>(i)]) * kInternalToDisplayedUnits,
                0.0f,
                kMaxDelayMs);
            delaySmoothers_[static_cast<std::size_t>(i)].setTarget(ms);
        }

        for (int s = 0; s < sampleFrames; ++s)
        {
            const float currentInput = finiteOrZero(*signalIn++);
            const float currentModulation = clampf(
                finiteOrZero(*modIn++) * 2.0f,
                -1.0f,
                1.0f);

            std::array<float, 4> currentBankB{};
            for (int i = 0; i < 4; ++i)
                currentBankB[static_cast<std::size_t>(i)] =
                    finiteOrZero(*bankBInputs[static_cast<std::size_t>(i)]);

            if (!interpolationHistoryValid_)
            {
                previousSignalIn_ = currentInput;
                previousModulation_ = currentModulation;
                previousBankB_ = currentBankB;
                interpolationHistoryValid_ = true;
            }

            InternalFrameResult internalResult{};
            const float invFactor = 1.0f / static_cast<float>(activeOversamplingFactor_);

            for (int os = 0; os < activeOversamplingFactor_; ++os)
            {
                // First-order-hold interpolation provides intermediate samples to
                // the linear/time-varying network without adding a separate
                // nonlinear resampling stage. At x1, t=1 and this is bit-path
                // equivalent to feeding the current host sample directly.
                const float t = static_cast<float>(os + 1) * invFactor;
                const float internalInput = previousSignalIn_ +
                    t * (currentInput - previousSignalIn_);
                const float internalModulation = previousModulation_ +
                    t * (currentModulation - previousModulation_);

                std::array<float, 4> internalBankB{};
                for (int i = 0; i < 4; ++i)
                {
                    const auto idx = static_cast<std::size_t>(i);
                    internalBankB[idx] = previousBankB_[idx] +
                        t * (currentBankB[idx] - previousBankB_[idx]);
                }

                internalResult = processInternalSample(
                    internalInput,
                    internalModulation,
                    internalBankB);
            }

            previousSignalIn_ = currentInput;
            previousModulation_ = currentModulation;
            previousBankB_ = currentBankB;

            // A brief fade is used only after a live oversampling/sample-rate
            // change, because the delay memories must be cleared when their time
            // grid changes. Initial startup is not faded.
            const float resetFade = rateChangeFade_;
            if (rateChangeFade_ < 1.0f)
                rateChangeFade_ = (std::min)(1.0f, rateChangeFade_ + rateChangeFadeIncrement_);

            *signalOut++ = internalResult.signalOut * resetFade;
            for (int i = 0; i < 4; ++i)
                *bankAOutputs[static_cast<std::size_t>(i)]++ =
                    internalResult.bankA[static_cast<std::size_t>(i)] * resetFade;

            for (auto& ptr : bankBInputs)
                ++ptr;
        }
    }

    void onSetPins() override
    {
        pinSignalOut.setStreaming(true);
        for (auto& pin : pinBankAOut)
            pin.setStreaming(true);

        // Delay and feedback memories continue evolving even with a static input.
        setSleep(false);
        setSubProcess(&AllPassDiffuser::subProcess);
    }

private:
    struct InternalFrameResult
    {
        float signalOut = 0.0f;
        std::array<float, 4> bankA{};
    };

    InternalFrameResult processInternalSample(
        float input,
        float modulation,
        const std::array<float, 4>& bankBInputs)
    {
        const float g = diffusionSmoother_.next();
        const float feedback = feedbackSmoother_.next();
        const float feedbackDamping = feedbackDampingSmoother_.next();
        const float tone = toneSmoother_.next();
        const float modDepthMs = modDepthSmoother_.next();
        const float routingMix = routingSmoother_.next();
        const float crossEnable = crossRoutesSmoother_.next();
        const float crossAmount = crossAmountSmoother_.next() * crossEnable;
        const float masterSize = masterSizeSmoother_.next();
        const float hadamardEnable = hadamardEnableSmoother_.next();
        const float hadamardAmount = hadamardAmountSmoother_.next() * hadamardEnable;
        const float shuffleTime = shuffleTimeSmoother_.next();
        const float shufflePolarity = shufflePolaritySmoother_.next();

        float stageInput = input + feedback * feedbackState_;
        stageInput = finiteOrZero(stageInput);

        std::array<float, kMaxStages> stageOutputs{};
        std::array<float, 4> hadamardOutputs{};
        bool hadamardReady = false;

        for (int i = 0; i < kMaxStages; ++i)
        {
            if (i >= 4 && i <= 7)
            {
                const int bankIndex = i - 4;
                stageInput += routingMix * bankBInputs[static_cast<std::size_t>(bankIndex)];
            }

            if (crossAmount > 0.0f)
            {
                if (i == 6)
                    stageInput += crossAmount * stageOutputs[4];
                else if (i == 7)
                    stageInput += crossAmount * stageOutputs[3];
                else if (i == 9)
                    stageInput += crossAmount * stageOutputs[5];
            }

            if (i >= 4 && i <= 7 && hadamardAmount > 0.0f)
            {
                if (!hadamardReady)
                {
                    std::array<float, 4> x{};
                    for (int k = 0; k < 4; ++k)
                    {
                        const int sourceIndex = shuffleTime >= 0.5f
                            ? matrixSourcePermutation_[static_cast<std::size_t>(k)]
                            : k;
                        float v = stageOutputs[static_cast<std::size_t>(sourceIndex)];
                        if (shufflePolarity >= 0.5f)
                            v *= matrixInputSigns_[static_cast<std::size_t>(k)];
                        x[static_cast<std::size_t>(k)] = v;
                    }

                    hadamardOutputs[0] = 0.5f * (x[0] + x[1] + x[2] + x[3]);
                    hadamardOutputs[1] = 0.5f * (x[0] - x[1] + x[2] - x[3]);
                    hadamardOutputs[2] = 0.5f * (x[0] + x[1] - x[2] - x[3]);
                    hadamardOutputs[3] = 0.5f * (x[0] - x[1] - x[2] + x[3]);

                    if (shufflePolarity >= 0.5f)
                    {
                        for (int k = 0; k < 4; ++k)
                            hadamardOutputs[static_cast<std::size_t>(k)] *=
                                matrixOutputSigns_[static_cast<std::size_t>(k)];
                    }
                    hadamardReady = true;
                }

                const int bankIndex = i - 4;
                const int matrixIndex = shuffleTime >= 0.5f
                    ? matrixDestinationPermutation_[static_cast<std::size_t>(bankIndex)]
                    : bankIndex;
                const float matrixSignal = hadamardOutputs[static_cast<std::size_t>(matrixIndex)];
                stageInput += hadamardAmount * (matrixSignal - stageInput);
            }

            stageInput = finiteOrZero(stageInput);

            const float baseMs = delaySmoothers_[static_cast<std::size_t>(i)].next();
            const float staticRandomOffsetMs =
                randomOffsetSmoothers_[static_cast<std::size_t>(i)].next();
            const float sizedBaseMs = (baseMs + staticRandomOffsetMs) * masterSize;
            const float modulatedMs = clampf(
                sizedBaseMs + modulation * modDepthMs,
                0.0f,
                kMaxDelayMs);

            const float delaySamples =
                modulatedMs * 0.001f * currentInternalSampleRate_;

            float stageOut = stages_[static_cast<std::size_t>(i)].process(
                stageInput,
                g,
                delaySamples);

            stageOutputs[static_cast<std::size_t>(i)] = stageOut;
            stageInput = stageOut;
        }

        const float oldTap = stageOutputs[static_cast<std::size_t>(previousStages_ - 1)];
        const float newTap = stageOutputs[static_cast<std::size_t>(targetStages_ - 1)];

        if (stageCrossfade_ < 1.0f)
            stageCrossfade_ = (std::min)(1.0f, stageCrossfade_ + stageCrossfadeIncrement_);
        else
            previousStages_ = targetStages_;

        float rawOutput = oldTap + stageCrossfade_ * (newTap - oldTap);
        rawOutput = zapDenormal(finiteOrZero(rawOutput));

        // v9 feedback: do NOT feed the final output straight back to AP1.
        // Mix four temporally separated all-pass taps, then smear that mixture
        // with short feed-forward taps and HF damping. This breaks the single
        // dominant loop period that made the old feedback sound metallic.
        const int last = clampi(targetStages_ - 1, 3, kMaxStages - 1);
        const int tapB = clampi((2 * last) / 3, 0, last - 1);
        const int tapC = clampi(last / 3, 0, tapB);
        const int tapD = 0;

        const float feedbackRaw =
            0.40f * stageOutputs[static_cast<std::size_t>(last)]
            - 0.30f * stageOutputs[static_cast<std::size_t>(tapB)]
            + 0.18f * stageOutputs[static_cast<std::size_t>(tapC)]
            - 0.12f * stageOutputs[static_cast<std::size_t>(tapD)];

        feedbackState_ = feedbackPath_.process(feedbackRaw, feedbackDamping);

        // Tone is now post-network. Positive values can truly restore/boost
        // treble without changing the recursive loop gain or the all-pass math.
        const float output = outputToneFilter_.process(rawOutput, tone);

        InternalFrameResult result;
        result.signalOut = output;
        for (int i = 0; i < 4; ++i)
            result.bankA[static_cast<std::size_t>(i)] =
                stageOutputs[static_cast<std::size_t>(i)];
        return result;
    }

    void updateProcessingRateIfNeeded(int requestedOversampling)
    {
        const float newHostRate = (std::max)(1000.0f, finiteOrZero(getSampleRate()));
        requestedOversampling = clampi(requestedOversampling, 1, kMaxOversamplingFactor);
        const float newInternalRate =
            newHostRate * static_cast<float>(requestedOversampling);

        const bool hostRateChanged = std::abs(newHostRate - currentSampleRate_) >= 0.5f;
        const bool factorChanged = requestedOversampling != activeOversamplingFactor_;
        if (processingRateInitialized_ && !hostRateChanged && !factorChanged)
            return;

        const bool wasInitialized = processingRateInitialized_;
        processingRateInitialized_ = true;
        currentSampleRate_ = newHostRate;
        activeOversamplingFactor_ = requestedOversampling;
        currentInternalSampleRate_ = newInternalRate;

        diffusionSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        feedbackSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        feedbackDampingSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        toneSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        modDepthSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        routingSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        crossRoutesSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        crossAmountSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        masterSizeSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        hadamardEnableSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        hadamardAmountSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        shuffleTimeSmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);
        shufflePolaritySmoother_.setTime(kParameterSmoothingMs, currentInternalSampleRate_);

        for (auto& smoother : delaySmoothers_)
            smoother.setTime(kParameterSmoothingMs, currentInternalSampleRate_);

        for (auto& smoother : randomOffsetSmoothers_)
            smoother.setTime(kRandomOffsetSmoothingMs, currentInternalSampleRate_);

        stageCrossfadeIncrement_ =
            1.0f / (currentInternalSampleRate_ * (kStageCrossfadeMs * 0.001f));

        for (auto& stage : stages_)
            stage.clear();

        outputToneFilter_.clear();
        outputToneFilter_.setSampleRate(currentInternalSampleRate_);

        feedbackPath_.clear();
        feedbackPath_.setSampleRate(currentInternalSampleRate_);

        feedbackState_ = 0.0f;
        interpolationHistoryValid_ = false;

        // Avoid a hard discontinuity if the user changes oversampling live.
        if (wasInitialized)
        {
            rateChangeFade_ = 0.0f;
            rateChangeFadeIncrement_ =
                1.0f / (currentSampleRate_ * (kStageCrossfadeMs * 0.001f));
        }
        else
        {
            rateChangeFade_ = 1.0f;
            rateChangeFadeIncrement_ = 1.0f;
        }
    }

    void regenerateRandomPattern(std::uint32_t seed)
    {
        activeSeed_ = seed == 0 ? 0x41c64e6du : seed;
        std::uint32_t state = activeSeed_;

        // Generate one fixed bipolar offset per stage. This is STATIC jitter,
        // not modulation: it breaks left/right correlation without chorus motion.
        for (int i = 0; i < kMaxStages; ++i)
        {
            float r = randomBipolar(state);

            // Avoid offsets clustered too close to zero, which would make two
            // auto-seeded instances accidentally more similar than intended.
            if (std::abs(r) < 0.18f)
                r = (r < 0.0f ? -0.18f : 0.18f);

            randomPattern_[static_cast<std::size_t>(i)] = r;
        }

        // Build deterministic per-instance shuffles for the optional Hadamard
        // network. Permutations preserve the four tap energies; sign flips keep
        // the transform orthogonal while changing its phase/polarity pattern.
        for (int i = 0; i < 4; ++i)
        {
            matrixSourcePermutation_[static_cast<std::size_t>(i)] = i;
            matrixDestinationPermutation_[static_cast<std::size_t>(i)] = i;
            matrixInputSigns_[static_cast<std::size_t>(i)] = randomBipolar(state) >= 0.0f ? 1.0f : -1.0f;
            matrixOutputSigns_[static_cast<std::size_t>(i)] = randomBipolar(state) >= 0.0f ? 1.0f : -1.0f;
        }

        for (int i = 3; i > 0; --i)
        {
            const int j = static_cast<int>(xorshift32(state) % static_cast<std::uint32_t>(i + 1));
            std::swap(matrixSourcePermutation_[static_cast<std::size_t>(i)],
                      matrixSourcePermutation_[static_cast<std::size_t>(j)]);
        }
        for (int i = 3; i > 0; --i)
        {
            const int j = static_cast<int>(xorshift32(state) % static_cast<std::uint32_t>(i + 1));
            std::swap(matrixDestinationPermutation_[static_cast<std::size_t>(i)],
                      matrixDestinationPermutation_[static_cast<std::size_t>(j)]);
        }
    }

    // Main input and controls.
    AudioInPin pinSignalIn;
    AudioInPin pinStages;
    AudioInPin pinRouting;
    AudioInPin pinDiffusion;
    AudioInPin pinFeedback;
    AudioInPin pinFeedbackDamping;
    AudioInPin pinTone;
    AudioInPin pinModIn;
    AudioInPin pinModDepthMs;
    AudioInPin pinRandomOffsetMs;
    AudioInPin pinRandomSeed;
    AudioInPin pinCrossRoutes;
    AudioInPin pinCrossAmount;
    AudioInPin pinMasterSize;
    AudioInPin pinOversamplingFactor;
    AudioInPin pinHadamardEnable;
    AudioInPin pinHadamardAmount;
    AudioInPin pinShuffleTime;
    AudioInPin pinShufflePolarity;

    std::array<AudioInPin, kMaxStages> pinDelayMs;
    std::array<AudioInPin, 4> pinBankBIn;

    // Outputs.
    AudioOutPin pinSignalOut;
    std::array<AudioOutPin, 4> pinBankAOut;

    // DSP state.
    std::array<ModulatedAllPassStage, kMaxStages> stages_;
    OutputToneFilter outputToneFilter_;
    DiffuseFeedbackPath feedbackPath_;
    std::array<SmoothedValue, kMaxStages> delaySmoothers_;
    std::array<SmoothedValue, kMaxStages> randomOffsetSmoothers_;
    std::array<float, kMaxStages> randomPattern_{};
    std::array<int, 4> matrixSourcePermutation_{{0, 1, 2, 3}};
    std::array<int, 4> matrixDestinationPermutation_{{0, 1, 2, 3}};
    std::array<float, 4> matrixInputSigns_{{1.0f, 1.0f, 1.0f, 1.0f}};
    std::array<float, 4> matrixOutputSigns_{{1.0f, 1.0f, 1.0f, 1.0f}};
    SmoothedValue diffusionSmoother_;
    SmoothedValue feedbackSmoother_;
    SmoothedValue feedbackDampingSmoother_;
    SmoothedValue toneSmoother_;
    SmoothedValue modDepthSmoother_;
    SmoothedValue routingSmoother_;
    SmoothedValue crossRoutesSmoother_;
    SmoothedValue crossAmountSmoother_;
    SmoothedValue masterSizeSmoother_;
    SmoothedValue hadamardEnableSmoother_;
    SmoothedValue hadamardAmountSmoother_;
    SmoothedValue shuffleTimeSmoother_;
    SmoothedValue shufflePolaritySmoother_;

    std::uint32_t autoSeed_ = 1u;
    std::uint32_t activeSeed_ = 0u;

    float currentSampleRate_ = 0.0f;
    float currentInternalSampleRate_ = 0.0f;
    int activeOversamplingFactor_ = 1;
    bool processingRateInitialized_ = false;

    float previousSignalIn_ = 0.0f;
    float previousModulation_ = 0.0f;
    std::array<float, 4> previousBankB_{};
    bool interpolationHistoryValid_ = false;

    float rateChangeFade_ = 1.0f;
    float rateChangeFadeIncrement_ = 1.0f;
    float feedbackState_ = 0.0f;

    int previousStages_ = 8;
    int targetStages_ = 8;
    float stageCrossfade_ = 1.0f;
    float stageCrossfadeIncrement_ = 1.0f;
};

namespace
{
    auto registration =
        Register<AllPassDiffuser>::withId(L"Pandocrator Multi AllPass Diffuser v9");
}
