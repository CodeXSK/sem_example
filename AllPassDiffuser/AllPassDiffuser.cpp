#include "mp_sdk_audio.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

using namespace gmpi;

namespace
{
    constexpr int   kMaxStages = 10;
    constexpr int   kMinStages = 4;
    constexpr float kMaxDelayMs = 500.0f;
    constexpr float kMaxSupportedSampleRate = 384000.0f;
    constexpr float kMaxDiffusion = 0.97f;
    constexpr float kMaxFeedback = 0.98f;
    constexpr float kParameterSmoothingMs = 12.0f;
    constexpr float kStageCrossfadeMs = 20.0f;
    // SynthEdit internal audio/CV units: 1.0 == 10 V in the editor.
    constexpr float kInternalToDisplayedUnits = 10.0f;

    // Enough storage for 500 ms at 384 kHz, plus interpolation guard samples.
    constexpr int kDelayBufferSamples =
        static_cast<int>(kMaxSupportedSampleRate * (kMaxDelayMs * 0.001f)) + 8;

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
        }

        float process(float input, float g, float delaySamples)
        {
            input = finiteOrZero(input);
            g = clampf(finiteOrZero(g), -kMaxDiffusion, kMaxDiffusion);

            // At least one sample of delay is essential. Keep two guard samples
            // below the physical buffer length for safe linear interpolation.
            const float maxDelay = static_cast<float>(kDelayBufferSamples - 3);
            delaySamples = clampf(finiteOrZero(delaySamples), 1.0f, maxDelay);

            float readPosition = static_cast<float>(writeIndex_) - delaySamples;
            while (readPosition < 0.0f)
                readPosition += static_cast<float>(kDelayBufferSamples);

            const int index0 = static_cast<int>(readPosition);
            const int index1 = (index0 + 1) % kDelayBufferSamples;
            const float fraction = readPosition - static_cast<float>(index0);

            float delayed0 = finiteOrZero(buffer_[static_cast<std::size_t>(index0)]);
            float delayed1 = finiteOrZero(buffer_[static_cast<std::size_t>(index1)]);
            const float delayed = delayed0 + fraction * (delayed1 - delayed0);

            // Schroeder all-pass form:
            // y[n] = -g*x[n] + v[n-D]
            // v[n] =  x[n] + g*y[n]
            // For a fixed integer D this gives
            // H(z) = (z^-D - g) / (1 - g*z^-D).
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
        initializePin(pinModIn);
        initializePin(pinModDepthMs);

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
        modDepthSmoother_.reset(0.0f);
        routingSmoother_.reset(0.0f);
    }

    void subProcess(int sampleFrames)
    {
        updateSampleRateIfNeeded();

        auto signalIn = getBuffer(pinSignalIn);
        auto stagesPin = getBuffer(pinStages);
        auto routingPin = getBuffer(pinRouting);
        auto diffusionPin = getBuffer(pinDiffusion);
        auto feedbackPin = getBuffer(pinFeedback);
        auto modIn = getBuffer(pinModIn);
        auto modDepthPin = getBuffer(pinModDepthMs);

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

        // Control-rate targets are sampled once per SynthEdit processing block.
        // Their transitions are smoothed internally sample-by-sample.
        // SynthEdit audio/control pins use normalized internal units where
        // 1.0 corresponds to 10 V in the editor. Convert to the natural values
        // shown to the user before interpreting controls.
        const float stagesValue = finiteOrZero(*stagesPin) * kInternalToDisplayedUnits;
        const int requestedStages = clampi(
            static_cast<int>(std::lround(stagesValue)),
            kMinStages,
            kMaxStages);

        if (requestedStages != targetStages_)
        {
            // Crossfade between the two already-running stage taps. All ten stages
            // are always processed, so there is no stale delay memory when a stage
            // count is increased.
            previousStages_ = targetStages_;
            targetStages_ = requestedStages;
            stageCrossfade_ = 0.0f;
        }

        const float routingValue = finiteOrZero(*routingPin) * kInternalToDisplayedUnits;
        const float routingTarget = routingValue >= 0.5f ? 1.0f : 0.0f;
        routingSmoother_.setTarget(routingTarget);

        diffusionSmoother_.setTarget(clampf(
            finiteOrZero(*diffusionPin) * kInternalToDisplayedUnits, -kMaxDiffusion, kMaxDiffusion));

        feedbackSmoother_.setTarget(clampf(
            finiteOrZero(*feedbackPin) * kInternalToDisplayedUnits, -kMaxFeedback, kMaxFeedback));

        modDepthSmoother_.setTarget(clampf(
            finiteOrZero(*modDepthPin) * kInternalToDisplayedUnits, 0.0f, kMaxDelayMs));

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
            const float input = finiteOrZero(*signalIn++);
            // A standard bipolar SynthEdit audio/LFO signal is typically around +/-5 V,
            // which is +/-0.5 in internal units. Normalize that to +/-1 for modulation.
            const float modulation = clampf(
                finiteOrZero(*modIn++) * 2.0f,
                -1.0f,
                1.0f);

            const float g = diffusionSmoother_.next();
            const float feedback = feedbackSmoother_.next();
            const float modDepthMs = modDepthSmoother_.next();
            const float routingMix = routingSmoother_.next();

            // Internal global feedback is delayed by exactly one processed sample
            // at the network level, removing any algebraic loop.
            float stageInput = input + feedback * feedbackState_;
            stageInput = finiteOrZero(stageInput);

            std::array<float, kMaxStages> stageOutputs{};

            for (int i = 0; i < kMaxStages; ++i)
            {
                // Optional external 4-in injection bank before AP5..AP8.
                if (i >= 4 && i <= 7)
                {
                    const int bankIndex = i - 4;
                    const float injection = finiteOrZero(*bankBInputs[static_cast<std::size_t>(bankIndex)]);
                    stageInput += routingMix * injection;
                }

                const float baseMs = delaySmoothers_[static_cast<std::size_t>(i)].next();
                const float modulatedMs = clampf(
                    baseMs + modulation * modDepthMs,
                    0.0f,
                    kMaxDelayMs);

                // Convert milliseconds to fractional samples. The stage itself
                // enforces >= 1 sample and buffer-safe maximum delay.
                const float delaySamples =
                    modulatedMs * 0.001f * currentSampleRate_;

                const float stageOut = stages_[static_cast<std::size_t>(i)].process(
                    stageInput,
                    g,
                    delaySamples);

                stageOutputs[static_cast<std::size_t>(i)] = stageOut;
                stageInput = stageOut;
            }

            // First-bank taps are always available, independent of routing mode.
            for (int i = 0; i < 4; ++i)
                *bankAOutputs[static_cast<std::size_t>(i)]++ = stageOutputs[static_cast<std::size_t>(i)];

            const float oldTap = stageOutputs[static_cast<std::size_t>(previousStages_ - 1)];
            const float newTap = stageOutputs[static_cast<std::size_t>(targetStages_ - 1)];

            if (stageCrossfade_ < 1.0f)
            {
                stageCrossfade_ = (std::min)(1.0f, stageCrossfade_ + stageCrossfadeIncrement_);
            }
            else
            {
                previousStages_ = targetStages_;
            }

            float output = oldTap + stageCrossfade_ * (newTap - oldTap);
            output = zapDenormal(finiteOrZero(output));

            *signalOut++ = output;
            feedbackState_ = output;

            // The external bank inputs are audio-rate, therefore they must advance
            // every sample even when Routing is in Series mode.
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
    void updateSampleRateIfNeeded()
    {
        const float newRate = (std::max)(1000.0f, finiteOrZero(getSampleRate()));
        if (std::abs(newRate - currentSampleRate_) < 0.5f)
            return;

        currentSampleRate_ = newRate;

        diffusionSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        feedbackSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        modDepthSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        routingSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);

        for (auto& smoother : delaySmoothers_)
            smoother.setTime(kParameterSmoothingMs, currentSampleRate_);

        stageCrossfadeIncrement_ =
            1.0f / (currentSampleRate_ * (kStageCrossfadeMs * 0.001f));

        // A sample-rate change invalidates all stored delay-time history.
        for (auto& stage : stages_)
            stage.clear();

        feedbackState_ = 0.0f;
    }

    // Main input and controls.
    AudioInPin pinSignalIn;
    AudioInPin pinStages;
    AudioInPin pinRouting;
    AudioInPin pinDiffusion;
    AudioInPin pinFeedback;
    AudioInPin pinModIn;
    AudioInPin pinModDepthMs;

    std::array<AudioInPin, kMaxStages> pinDelayMs;
    std::array<AudioInPin, 4> pinBankBIn;

    // Outputs.
    AudioOutPin pinSignalOut;
    std::array<AudioOutPin, 4> pinBankAOut;

    // DSP state.
    std::array<ModulatedAllPassStage, kMaxStages> stages_;
    std::array<SmoothedValue, kMaxStages> delaySmoothers_;
    SmoothedValue diffusionSmoother_;
    SmoothedValue feedbackSmoother_;
    SmoothedValue modDepthSmoother_;
    SmoothedValue routingSmoother_;

    float currentSampleRate_ = 0.0f;
    float feedbackState_ = 0.0f;

    int previousStages_ = 8;
    int targetStages_ = 8;
    float stageCrossfade_ = 1.0f;
    float stageCrossfadeIncrement_ = 1.0f;
};

namespace
{
    auto registration =
        Register<AllPassDiffuser>::withId(L"Pandocrator Multi AllPass Diffuser v2");
}
