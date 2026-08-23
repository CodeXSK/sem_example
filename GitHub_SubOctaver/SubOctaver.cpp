#include "mp_sdk_audio.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace gmpi;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    // SynthEdit's normal internal audio range is around +/-1.0.  These limits
    // are deliberately very generous and exist only to stop pathological host
    // or upstream-module values from poisoning the DSP state.
    constexpr float kMaxSafeInput = 16.0f;
    constexpr float kMaxSafeOutput = 32.0f;
    constexpr float kDenormalThreshold = 1.0e-20f;

    constexpr float kDefaultDry = 0.0f;
    constexpr float kDefaultSub = 0.75f;
    constexpr float kDefaultTone = 0.45f;
    constexpr float kDefaultTracking = 0.50f;
    constexpr float kDefaultSensitivity = 0.65f;

    inline float clamp01(float value)
    {
        return (std::max)(0.0f, (std::min)(1.0f, value));
    }

    inline float sanitizeControl(float value, float fallback)
    {
        return std::isfinite(value) ? clamp01(value) : fallback;
    }

    inline float sanitizeAudioInput(float value)
    {
        if (!std::isfinite(value))
            return 0.0f;

        return (std::max)(-kMaxSafeInput, (std::min)(kMaxSafeInput, value));
    }

    inline float sanitizeAudioOutput(float value)
    {
        if (!std::isfinite(value))
            return 0.0f;

        return (std::max)(-kMaxSafeOutput, (std::min)(kMaxSafeOutput, value));
    }

    inline float zapDenormal(float value)
    {
        if (!std::isfinite(value) || std::abs(value) < kDenormalThreshold)
            return 0.0f;

        return value;
    }

    inline float exponentialMap(float normalized, float minimum, float maximum)
    {
        normalized = clamp01(normalized);
        return minimum * std::pow(maximum / minimum, normalized);
    }

    inline float onePoleAlpha(float cutoffHz, float sampleRate)
    {
        // Keep the coefficient meaningful even under an impossible/corrupt
        // sample-rate value. updateCoefficients() has already sanitized SR.
        const float maximumCutoff = (std::max)(1.0f, sampleRate * 0.45f);
        cutoffHz = (std::max)(1.0f, (std::min)(maximumCutoff, cutoffHz));

        const float alpha = 1.0f - std::exp(-2.0f * kPi * cutoffHz / sampleRate);
        return (std::max)(0.0f, (std::min)(1.0f, alpha));
    }

    inline float timeCoefficient(float seconds, float sampleRate)
    {
        seconds = (std::max)(0.000001f, seconds);
        return std::exp(-1.0f / (seconds * sampleRate));
    }
}

// Monophonic divide-by-two subharmonic generator.
//
// The detector uses:
//  1. input DC blocking,
//  2. two-pole low-pass filtering,
//  3. a Schmitt trigger,
//  4. a flip-flop that changes polarity on every accepted positive cycle.
//
// Changing polarity once per input period creates a waveform whose period is
// twice the input period, therefore its fundamental is one octave below (f / 2).
//
// Release-safe hardening in this version:
//  - finite-value guards for audio/control inputs,
//  - pathological-level clamping outside any normal SynthEdit audio range,
//  - self-recovery if an internal state ever becomes non-finite,
//  - gate hysteresis to prevent chatter around the threshold,
//  - smoother gate attack/release to reduce clicks,
//  - polarity reset only after the sub gate is effectively silent,
//  - denormal/subnormal cleanup once per processing block,
//  - coefficient recalculation only when controls/sample-rate change.
class SubOctaver final : public MpBase2
{
public:
    SubOctaver()
    {
        // Pin order must match SubOctaver.xml exactly.
        initializePin(pinSignalIn);
        initializePin(pinDryLevel);
        initializePin(pinSubLevel);
        initializePin(pinTone);
        initializePin(pinTracking);
        initializePin(pinSensitivity);
        initializePin(pinSubOut);
        initializePin(pinMixOut);
    }

    void subProcess(int sampleFrames)
    {
        if (sampleFrames <= 0)
            return;

        auto signalIn       = getBuffer(pinSignalIn);
        auto dryPin         = getBuffer(pinDryLevel);
        auto subPin         = getBuffer(pinSubLevel);
        auto tonePin        = getBuffer(pinTone);
        auto trackingPin    = getBuffer(pinTracking);
        auto sensitivityPin = getBuffer(pinSensitivity);
        auto subOut         = getBuffer(pinSubOut);
        auto mixOut         = getBuffer(pinMixOut);

        // If anything external ever managed to poison an old state, recover at
        // the next block boundary instead of propagating NaNs indefinitely.
        recoverInvalidStateIfNeeded();

        // Controls are sampled once per SynthEdit processing block, preserving
        // the behaviour of the original module while preventing invalid values.
        const float dryLevel = sanitizeControl(*dryPin, kDefaultDry);
        const float subLevel = sanitizeControl(*subPin, kDefaultSub);
        const float tone = sanitizeControl(*tonePin, kDefaultTone);
        const float tracking = sanitizeControl(*trackingPin, kDefaultTracking);
        const float sensitivity = sanitizeControl(*sensitivityPin, kDefaultSensitivity);

        refreshCoefficientsIfNeeded(tone, tracking, sensitivity);

        const float gateOpenThreshold = detectorThreshold_ * 1.50f;
        const float gateCloseThreshold = detectorThreshold_ * 0.65f;

        for (int s = sampleFrames; s > 0; --s)
        {
            const float input = sanitizeAudioInput(*signalIn++);

            // 1-pole DC blocker for the pitch detector.
            const float detectorHighPass =
                input - previousInput_ + detectorDcR_ * previousDetectorHighPass_;

            previousInput_ = input;
            previousDetectorHighPass_ = detectorHighPass;

            // Two cascaded one-pole low-pass filters suppress upper harmonics.
            detectorLowPass1_ += detectorAlpha_ * (detectorHighPass - detectorLowPass1_);
            detectorLowPass2_ += detectorAlpha_ * (detectorLowPass1_ - detectorLowPass2_);

            // Envelope follower controls the sub amplitude and its output gate.
            const float rectified = std::abs(detectorHighPass);
            const float envelopeCoefficient =
                rectified > envelope_ ? envelopeAttackCoefficient_
                                      : envelopeReleaseCoefficient_;

            envelope_ =
                rectified + envelopeCoefficient * (envelope_ - rectified);

            // Saturating counter: cannot overflow even after extremely long runs.
            if (samplesSinceAcceptedEdge_ < 0x3fffffff)
                ++samplesSinceAcceptedEdge_;

            // Schmitt trigger. Toggle only on a newly accepted positive-going cycle.
            if (!schmittHigh_)
            {
                if (detectorLowPass2_ > detectorThreshold_
                    && samplesSinceAcceptedEdge_ >= minimumEdgeDistanceSamples_)
                {
                    schmittHigh_ = true;
                    subPolarity_ = -subPolarity_;
                    samplesSinceAcceptedEdge_ = 0;
                }
            }
            else if (detectorLowPass2_ < -detectorThreshold_)
            {
                schmittHigh_ = false;
            }

            // Gate hysteresis: the gate opens at a higher level than the level at
            // which it closes. This prevents rapid ON/OFF chatter near threshold.
            if (!gateOpen_)
            {
                if (envelope_ > gateOpenThreshold)
                    gateOpen_ = true;
            }
            else
            {
                if (envelope_ < gateCloseThreshold)
                    gateOpen_ = false;
            }

            const float gateTarget = gateOpen_ ? 1.0f : 0.0f;
            const float gateCoefficient =
                gateTarget > gate_ ? gateAttackCoefficient_
                                   : gateReleaseCoefficient_;

            gate_ = gateTarget + gateCoefficient * (gate_ - gateTarget);

            // Reset stale detector state after silence. Polarity is reset only
            // after the smoothed gate is effectively silent, preventing an abrupt
            // -1 -> +1 discontinuity while audible signal is still present.
            if (samplesSinceAcceptedEdge_ > detectorResetSamples_
                && envelope_ < gateCloseThreshold)
            {
                schmittHigh_ = false;

                if (gate_ < 0.00001f)
                    subPolarity_ = 1.0f;
            }

            // Soft amplitude compression keeps the sub useful across widely
            // varying input levels while following the input envelope.
            const float followedAmplitude = std::tanh(2.5f * envelope_);
            const float rawSub = subPolarity_ * followedAmplitude * gate_;

            // Two-pole low-pass rounds the divided square-like waveform.
            toneLowPass1_ += toneAlpha_ * (rawSub - toneLowPass1_);
            toneLowPass2_ += toneAlpha_ * (toneLowPass1_ - toneLowPass2_);

            // Remove residual DC from uneven/interrupted tracking.
            const float filteredSub =
                toneLowPass2_ - previousToneSample_
                + subDcR_ * previousSubHighPass_;

            previousToneSample_ = toneLowPass2_;
            previousSubHighPass_ = filteredSub;

            const float safeSub = sanitizeAudioOutput(filteredSub);
            const float safeMix = sanitizeAudioOutput(
                input * dryLevel + safeSub * subLevel);

            *subOut++ = safeSub;
            *mixOut++ = safeMix;
        }

        // Flush very tiny subnormal states at block boundaries. This avoids
        // denormal-related CPU spikes after long decays/silence on some CPUs.
        zapDenormalStates();
    }

    void onSetPins() override
    {
        // Internal envelope/filter tails can continue after the input becomes
        // static, so keep processing active. This favors deterministic tails and
        // reliability over a small potential idle-CPU saving.
        pinSubOut.setStreaming(true);
        pinMixOut.setStreaming(true);
        setSleep(false);
        setSubProcess(&SubOctaver::subProcess);
    }

private:
    void refreshCoefficientsIfNeeded(float tone, float tracking, float sensitivity)
    {
        float sampleRate = getSampleRate();

        if (!std::isfinite(sampleRate))
            sampleRate = 48000.0f;

        sampleRate = (std::max)(1000.0f, (std::min)(768000.0f, sampleRate));

        if (sampleRate == lastSampleRate_
            && tone == lastTone_
            && tracking == lastTracking_
            && sensitivity == lastSensitivity_)
        {
            return;
        }

        updateCoefficients(tone, tracking, sensitivity, sampleRate);

        lastSampleRate_ = sampleRate;
        lastTone_ = tone;
        lastTracking_ = tracking;
        lastSensitivity_ = sensitivity;
    }

    void updateCoefficients(float tone, float tracking, float sensitivity, float sampleRate)
    {
        // Tone: approximately 40 Hz to 600 Hz at normal audio sample rates.
        const float toneHz = exponentialMap(tone, 40.0f, 600.0f);

        // Tracking filter: approximately 100 Hz to 3000 Hz.
        // Higher values follow higher notes, but may react more to harmonics.
        const float trackingHz = exponentialMap(tracking, 100.0f, 3000.0f);

        // Higher Sensitivity means a lower detector trigger threshold.
        detectorThreshold_ =
            0.08f * std::pow(0.0005f / 0.08f, sensitivity);

        detectorAlpha_ = onePoleAlpha(trackingHz, sampleRate);
        toneAlpha_ = onePoleAlpha(toneHz, sampleRate);

        detectorDcR_ = std::exp(-2.0f * kPi * 10.0f / sampleRate);
        subDcR_ = std::exp(-2.0f * kPi * 5.0f / sampleRate);

        envelopeAttackCoefficient_ = timeCoefficient(0.004f, sampleRate);
        envelopeReleaseCoefficient_ = timeCoefficient(0.090f, sampleRate);

        // Slower than v1 to avoid exposing a sudden fraction of a low-frequency
        // cycle. Hysteresis does the actual anti-chatter work.
        gateAttackCoefficient_ = timeCoefficient(0.008f, sampleRate);
        gateReleaseCoefficient_ = timeCoefficient(0.120f, sampleRate);

        // Reject impossible extra edges above roughly 4 kHz.
        minimumEdgeDistanceSamples_ =
            (std::max)(4, static_cast<int>(sampleRate / 4000.0f));

        // Reset stale cycle state after roughly 250 ms without an accepted edge.
        detectorResetSamples_ =
            (std::max)(1, static_cast<int>(sampleRate * 0.25f));
    }

    void resetDynamicState()
    {
        previousInput_ = 0.0f;
        previousDetectorHighPass_ = 0.0f;
        detectorLowPass1_ = 0.0f;
        detectorLowPass2_ = 0.0f;
        envelope_ = 0.0f;
        gate_ = 0.0f;
        gateOpen_ = false;

        schmittHigh_ = false;
        subPolarity_ = 1.0f;
        samplesSinceAcceptedEdge_ = 0;

        toneLowPass1_ = 0.0f;
        toneLowPass2_ = 0.0f;
        previousToneSample_ = 0.0f;
        previousSubHighPass_ = 0.0f;
    }

    void recoverInvalidStateIfNeeded()
    {
        const bool valid =
            std::isfinite(previousInput_)
            && std::isfinite(previousDetectorHighPass_)
            && std::isfinite(detectorLowPass1_)
            && std::isfinite(detectorLowPass2_)
            && std::isfinite(envelope_)
            && std::isfinite(gate_)
            && std::isfinite(subPolarity_)
            && std::isfinite(toneLowPass1_)
            && std::isfinite(toneLowPass2_)
            && std::isfinite(previousToneSample_)
            && std::isfinite(previousSubHighPass_);

        if (!valid)
            resetDynamicState();
    }

    void zapDenormalStates()
    {
        previousInput_ = zapDenormal(previousInput_);
        previousDetectorHighPass_ = zapDenormal(previousDetectorHighPass_);
        detectorLowPass1_ = zapDenormal(detectorLowPass1_);
        detectorLowPass2_ = zapDenormal(detectorLowPass2_);
        envelope_ = zapDenormal(envelope_);
        gate_ = zapDenormal(gate_);

        toneLowPass1_ = zapDenormal(toneLowPass1_);
        toneLowPass2_ = zapDenormal(toneLowPass2_);
        previousToneSample_ = zapDenormal(previousToneSample_);
        previousSubHighPass_ = zapDenormal(previousSubHighPass_);
    }

    // Inputs.
    AudioInPin pinSignalIn;
    AudioInPin pinDryLevel;
    AudioInPin pinSubLevel;
    AudioInPin pinTone;
    AudioInPin pinTracking;
    AudioInPin pinSensitivity;

    // Outputs.
    AudioOutPin pinSubOut;
    AudioOutPin pinMixOut;

    // Detector state.
    float previousInput_ = 0.0f;
    float previousDetectorHighPass_ = 0.0f;
    float detectorLowPass1_ = 0.0f;
    float detectorLowPass2_ = 0.0f;
    float envelope_ = 0.0f;
    float gate_ = 0.0f;

    bool schmittHigh_ = false;
    bool gateOpen_ = false;
    float subPolarity_ = 1.0f;
    int samplesSinceAcceptedEdge_ = 0;

    // Tone/output state.
    float toneLowPass1_ = 0.0f;
    float toneLowPass2_ = 0.0f;
    float previousToneSample_ = 0.0f;
    float previousSubHighPass_ = 0.0f;

    // Coefficients.
    float detectorAlpha_ = 0.01f;
    float toneAlpha_ = 0.01f;
    float detectorDcR_ = 0.999f;
    float subDcR_ = 0.999f;
    float detectorThreshold_ = 0.003f;
    float envelopeAttackCoefficient_ = 0.99f;
    float envelopeReleaseCoefficient_ = 0.999f;
    float gateAttackCoefficient_ = 0.99f;
    float gateReleaseCoefficient_ = 0.999f;
    int minimumEdgeDistanceSamples_ = 4;
    int detectorResetSamples_ = 12000;

    // Cached controls/sample rate: avoid repeated pow()/exp() when unchanged.
    float lastSampleRate_ = -1.0f;
    float lastTone_ = -1.0f;
    float lastTracking_ = -1.0f;
    float lastSensitivity_ = -1.0f;
};

namespace
{
    // Keep the same ID as v1 so existing SynthEdit projects identify this as
    // the same module when the binary is replaced.
    auto registration =
        Register<SubOctaver>::withId(L"Pandocrator Sub Octaver");
}
