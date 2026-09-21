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
    constexpr float kMaxTone = 1.0f;
    constexpr float kMaxInternalMotionMs = 2.0f;
    constexpr float kMinMotionRateHz = 0.01f;
    constexpr float kMaxMotionRateHz = 2.0f;
    constexpr float kPerStageMaxDiffusion = 0.93f;
    constexpr float kPi = 3.14159265358979323846f;
    // SynthEdit internal audio/CV units: 1.0 == 10 V in the editor.
    constexpr float kInternalToDisplayedUnits = 10.0f;

    // Enough storage for 500 ms at 384 kHz, plus interpolation guard samples.
    constexpr int kDelayBufferSamples =
        static_cast<int>(kMaxSupportedSampleRate * (kMaxDelayMs * 0.001f)) + 8;



    // Stage-to-stage decorrelation. Using identical coefficients and identical
    // modulation on every all-pass can create a clearly metallic, comb-like
    // signature. These deterministic variations deliberately break that symmetry
    // while preserving a predictable, repeatable sound.
    constexpr std::array<float, kMaxStages> kStageGScale = {
        0.78f, 0.94f, 0.85f, 1.05f, 0.72f,
        0.98f, 0.81f, 1.08f, 0.75f, 0.91f
    };

    constexpr std::array<float, kMaxStages> kStagePolarity = {
        1.0f, -1.0f, 1.0f, -1.0f, 1.0f,
       -1.0f,  1.0f,-1.0f,  1.0f,-1.0f
    };

    // A single external LFO no longer moves all delay lines in lock-step.
    constexpr std::array<float, kMaxStages> kExternalModWeights = {
         1.00f, -0.73f,  0.41f, -0.91f,  0.58f,
        -0.36f,  0.84f, -0.52f,  0.67f, -0.29f
    };

    // Per-stage mixtures of three intentionally non-harmonic internal LFOs.
    // The rows are kept below roughly unity combined gain; the final result is
    // clamped before being converted to milliseconds.
    constexpr std::array<std::array<float, 3>, kMaxStages> kMotionMix = {{
        {{ 0.72f,  0.19f, -0.27f}},
        {{-0.31f,  0.68f,  0.22f}},
        {{ 0.18f, -0.42f,  0.74f}},
        {{-0.66f, -0.16f,  0.33f}},
        {{ 0.49f,  0.57f, -0.14f}},
        {{-0.24f,  0.29f, -0.71f}},
        {{ 0.61f, -0.53f,  0.17f}},
        {{-0.45f,  0.13f,  0.64f}},
        {{ 0.27f,  0.73f,  0.31f}},
        {{-0.58f, -0.35f,  0.46f}}
    }};

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
    class ProgressiveToneFilter
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
            const float progress = clampf((static_cast<float>(stageIndex) + 1.0f) / static_cast<float>(kMaxStages), 0.1f, 1.0f);

            // Later stages are intentionally more coloured than the early ones.
            // Dark side: progressively lower low-pass cutoff.
            const float darkCutoffHz = 14000.0f + progress * (4200.0f - 14000.0f);
            // Bright side: progressively higher high-pass cutoff (implemented as x - one-pole LP).
            const float brightCutoffHz = 35.0f + progress * (260.0f - 35.0f);

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

            // The colour accumulates gently through the chain. The limits below
            // keep the network musical and avoid making the last stages excessively
            // thin or muffled at extreme Tone settings.
            const float darkMix = (std::max)(0.0f, -tone) * (0.12f + 0.38f * progress_);
            const float brightMix = (std::max)(0.0f,  tone) * (0.08f + 0.22f * progress_);

            float output = input;
            if (darkMix > 0.0f)
                output += darkMix * (darkState_ - output);
            if (brightMix > 0.0f)
                output += brightMix * (highPassed - output);

            return zapDenormal(finiteOrZero(output));
        }

    private:
        static float onePoleCoeff(float cutoffHz, float sampleRate)
        {
            const float nyquistSafe = 0.45f * sampleRate;
            const float fc = clampf(cutoffHz, 5.0f, nyquistSafe);
            return 1.0f - std::exp(-2.0f * 3.14159265358979323846f * fc / sampleRate);
        }

        float sampleRate_ = 44100.0f;
        float darkState_ = 0.0f;
        float brightLowState_ = 0.0f;
        float darkCoeff_ = 1.0f;
        float brightCoeff_ = 0.01f;
        float progress_ = 0.1f;
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
        initializePin(pinTone);
        initializePin(pinMotionMs);
        initializePin(pinMotionRateHz);
        initializePin(pinModIn);
        initializePin(pinModDepthMs);

        for (auto& pin : pinDelayMs)
            initializePin(pin);

        for (auto& pin : pinBankBIn)
            initializePin(pin);

        initializePin(pinSignalOut);

        for (auto& pin : pinBankAOut)
            initializePin(pin);

        // Short, deliberately irregular diffuser times. Long, regularly related
        // delays tend to expose a "metal tube" signature in serial all-pass chains.
        const std::array<float, kMaxStages> defaults = {
            1.37f, 2.11f, 3.19f, 4.43f, 5.89f,
            7.31f, 9.17f, 11.53f, 14.29f, 17.87f
        };

        for (int i = 0; i < kMaxStages; ++i)
            delaySmoothers_[static_cast<std::size_t>(i)].reset(defaults[static_cast<std::size_t>(i)]);

        diffusionSmoother_.reset(0.65f);
        feedbackSmoother_.reset(0.0f);
        toneSmoother_.reset(-0.15f);
        motionDepthSmoother_.reset(0.12f);
        motionRateSmoother_.reset(0.11f);
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
        auto tonePin = getBuffer(pinTone);
        auto motionDepthPin = getBuffer(pinMotionMs);
        auto motionRatePin = getBuffer(pinMotionRateHz);
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

        toneSmoother_.setTarget(clampf(
            finiteOrZero(*tonePin) * kInternalToDisplayedUnits, -kMaxTone, kMaxTone));

        motionDepthSmoother_.setTarget(clampf(
            finiteOrZero(*motionDepthPin) * kInternalToDisplayedUnits, 0.0f, kMaxInternalMotionMs));

        motionRateSmoother_.setTarget(clampf(
            finiteOrZero(*motionRatePin) * kInternalToDisplayedUnits, kMinMotionRateHz, kMaxMotionRateHz));

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
            const float tone = toneSmoother_.next();
            const float motionDepthMs = motionDepthSmoother_.next();
            const float motionRateHz = motionRateSmoother_.next();
            const float modDepthMs = modDepthSmoother_.next();
            const float routingMix = routingSmoother_.next();

            // Three very slow, intentionally incommensurate internal sine LFOs.
            // Their sole purpose is to prevent the delay lattice from remaining
            // perfectly static. At the default 0.12 ms depth this should read as
            // "movement/smoothness", not as an obvious chorus.
            constexpr std::array<float, 3> rateRatios = { 0.73f, 1.0f, 1.37f };
            std::array<float, 3> internalLfo{};
            for (int l = 0; l < 3; ++l)
            {
                lfoPhase_[static_cast<std::size_t>(l)] +=
                    static_cast<double>(motionRateHz * rateRatios[static_cast<std::size_t>(l)] / currentSampleRate_);

                if (lfoPhase_[static_cast<std::size_t>(l)] >= 1.0)
                    lfoPhase_[static_cast<std::size_t>(l)] -= 1.0;

                internalLfo[static_cast<std::size_t>(l)] = static_cast<float>(
                    std::sin(2.0 * static_cast<double>(kPi) * lfoPhase_[static_cast<std::size_t>(l)]));
            }

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

                const std::size_t stageIndex = static_cast<std::size_t>(i);
                const float baseMs = delaySmoothers_[stageIndex].next();

                // External modulation is deliberately different for each stage;
                // a single sine no longer shifts every delay by the same amount.
                const float externalMotionMs =
                    modulation * modDepthMs * kExternalModWeights[stageIndex];

                // Internal "anti-metal" motion uses three slow LFOs mixed
                // differently at every stage. The mix is normalized conservatively
                // and clamped so Motion remains a true maximum-ish depth control.
                const auto& mix = kMotionMix[stageIndex];
                const float motionSignal = clampf(
                    0.68f * (mix[0] * internalLfo[0] + mix[1] * internalLfo[1] + mix[2] * internalLfo[2]),
                    -1.0f,
                    1.0f);
                const float internalMotionMs = motionDepthMs * motionSignal;

                const float modulatedMs = clampf(
                    baseMs + externalMotionMs + internalMotionMs,
                    0.0f,
                    kMaxDelayMs);

                // Convert milliseconds to fractional samples. The stage itself
                // enforces >= 1 sample and buffer-safe maximum delay.
                const float delaySamples =
                    modulatedMs * 0.001f * currentSampleRate_;

                // Each all-pass gets a slightly different coefficient and
                // alternating polarity. This is one of the strongest ways to break
                // the identical resonant signature of a simple serial chain.
                const float stageG = clampf(
                    g * kStageGScale[stageIndex] * kStagePolarity[stageIndex],
                    -kPerStageMaxDiffusion,
                    kPerStageMaxDiffusion);

                float stageOut = stages_[stageIndex].process(
                    stageInput,
                    stageG,
                    delaySamples);

                // Progressive spectral damping/tilt: negative Tone gradually
                // darkens later stages, positive Tone gradually removes more lows
                // so the diffusion/reverb becomes relatively brighter.
                stageOut = toneFilters_[static_cast<std::size_t>(i)].process(stageOut, tone);

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
        toneSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        motionDepthSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        motionRateSmoother_.setTime(80.0f, currentSampleRate_);
        modDepthSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);
        routingSmoother_.setTime(kParameterSmoothingMs, currentSampleRate_);

        for (auto& smoother : delaySmoothers_)
            smoother.setTime(kParameterSmoothingMs, currentSampleRate_);

        stageCrossfadeIncrement_ =
            1.0f / (currentSampleRate_ * (kStageCrossfadeMs * 0.001f));

        // A sample-rate change invalidates all stored delay-time history.
        for (auto& stage : stages_)
            stage.clear();

        for (int i = 0; i < kMaxStages; ++i)
        {
            toneFilters_[static_cast<std::size_t>(i)].clear();
            toneFilters_[static_cast<std::size_t>(i)].setSampleRate(currentSampleRate_, i);
        }

        feedbackState_ = 0.0f;
        lfoPhase_ = { 0.07, 0.39, 0.71 };
    }

    // Main input and controls.
    AudioInPin pinSignalIn;
    AudioInPin pinStages;
    AudioInPin pinRouting;
    AudioInPin pinDiffusion;
    AudioInPin pinFeedback;
    AudioInPin pinTone;
    AudioInPin pinMotionMs;
    AudioInPin pinMotionRateHz;
    AudioInPin pinModIn;
    AudioInPin pinModDepthMs;

    std::array<AudioInPin, kMaxStages> pinDelayMs;
    std::array<AudioInPin, 4> pinBankBIn;

    // Outputs.
    AudioOutPin pinSignalOut;
    std::array<AudioOutPin, 4> pinBankAOut;

    // DSP state.
    std::array<ModulatedAllPassStage, kMaxStages> stages_;
    std::array<ProgressiveToneFilter, kMaxStages> toneFilters_;
    std::array<SmoothedValue, kMaxStages> delaySmoothers_;
    SmoothedValue diffusionSmoother_;
    SmoothedValue feedbackSmoother_;
    SmoothedValue toneSmoother_;
    SmoothedValue motionDepthSmoother_;
    SmoothedValue motionRateSmoother_;
    SmoothedValue modDepthSmoother_;
    SmoothedValue routingSmoother_;

    float currentSampleRate_ = 0.0f;
    float feedbackState_ = 0.0f;
    std::array<double, 3> lfoPhase_ = { 0.07, 0.39, 0.71 };

    int previousStages_ = 8;
    int targetStages_ = 8;
    float stageCrossfade_ = 1.0f;
    float stageCrossfadeIncrement_ = 1.0f;
};

namespace
{
    auto registration =
        Register<AllPassDiffuser>::withId(L"Pandocrator Multi AllPass Diffuser v4");
}
