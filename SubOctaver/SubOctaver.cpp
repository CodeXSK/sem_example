#include "mp_sdk_audio.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace gmpi;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    inline float clamp01(float value)
    {
        return (std::max)(0.0f, (std::min)(1.0f, value));
    }

    inline float exponentialMap(float normalized, float minimum, float maximum)
    {
        normalized = clamp01(normalized);
        return minimum * std::pow(maximum / minimum, normalized);
    }

    inline float onePoleAlpha(float cutoffHz, float sampleRate)
    {
        cutoffHz = (std::max)(1.0f, cutoffHz);
        return 1.0f - std::exp(-2.0f * kPi * cutoffHz / sampleRate);
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
//  4. a flip-flop that changes polarity on every positive input cycle.
//
// Changing polarity once per input period creates a square wave whose period is
// twice the input period, therefore its fundamental is one octave below (f / 2).
class SubOctaver final : public MpBase2
{
public:
    SubOctaver()
    {
        // Pin order must match SubOctaver.xml.
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
        auto signalIn   = getBuffer(pinSignalIn);
        auto dryPin     = getBuffer(pinDryLevel);
        auto subPin     = getBuffer(pinSubLevel);
        auto tonePin    = getBuffer(pinTone);
        auto trackingPin = getBuffer(pinTracking);
        auto sensitivityPin = getBuffer(pinSensitivity);
        auto subOut     = getBuffer(pinSubOut);
        auto mixOut     = getBuffer(pinMixOut);

        // Knob/control values are sampled once per SynthEdit processing block.
        // This is efficient and is sufficiently smooth at normal SE block sizes.
        const float dryLevel   = clamp01(*dryPin);
        const float subLevel   = clamp01(*subPin);
        const float tone       = clamp01(*tonePin);
        const float tracking   = clamp01(*trackingPin);
        const float sensitivity = clamp01(*sensitivityPin);

        updateCoefficients(tone, tracking, sensitivity);

        for (int s = sampleFrames; s > 0; --s)
        {
            const float input = *signalIn++;

            // 1-pole DC blocker for the pitch detector.
            const float detectorHighPass =
                input - previousInput_ + detectorDcR_ * previousDetectorHighPass_;

            previousInput_ = input;
            previousDetectorHighPass_ = detectorHighPass;

            // Two cascaded one-pole low-pass filters suppress upper harmonics.
            detectorLowPass1_ += detectorAlpha_ * (detectorHighPass - detectorLowPass1_);
            detectorLowPass2_ += detectorAlpha_ * (detectorLowPass1_ - detectorLowPass2_);

            // Envelope follower controls sub amplitude and closes the noise gate.
            const float rectified = std::abs(detectorHighPass);
            const float envelopeCoefficient =
                rectified > envelope_ ? envelopeAttackCoefficient_
                                      : envelopeReleaseCoefficient_;

            envelope_ =
                rectified + envelopeCoefficient * (envelope_ - rectified);

            if (samplesSinceAcceptedEdge_ < 0x3fffffff)
                ++samplesSinceAcceptedEdge_;

            // Schmitt trigger. We toggle only on a new positive-going cycle.
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

            // Reset stale detector state after silence, so a new note starts cleanly.
            if (samplesSinceAcceptedEdge_ > detectorResetSamples_
                && envelope_ < detectorThreshold_)
            {
                schmittHigh_ = false;
                subPolarity_ = 1.0f;
            }

            // Soft amplitude compression keeps the sub usable across different
            // input levels while still following the input envelope.
            const float gateTarget =
                envelope_ > detectorThreshold_ * 1.15f ? 1.0f : 0.0f;

            const float gateCoefficient =
                gateTarget > gate_ ? gateAttackCoefficient_
                                   : gateReleaseCoefficient_;

            gate_ = gateTarget + gateCoefficient * (gate_ - gateTarget);

            const float followedAmplitude = std::tanh(2.5f * envelope_);
            const float rawSub = subPolarity_ * followedAmplitude * gate_;

            // Two-pole low-pass rounds the divided square wave into a bass waveform.
            toneLowPass1_ += toneAlpha_ * (rawSub - toneLowPass1_);
            toneLowPass2_ += toneAlpha_ * (toneLowPass1_ - toneLowPass2_);

            // Remove any residual DC from uneven or interrupted tracking.
            const float filteredSub =
                toneLowPass2_ - previousToneSample_
                + subDcR_ * previousSubHighPass_;

            previousToneSample_ = toneLowPass2_;
            previousSubHighPass_ = filteredSub;

            *subOut++ = filteredSub;
            *mixOut++ = input * dryLevel + filteredSub * subLevel;
        }
    }

    void onSetPins() override
    {
        // The internal envelope and filters may continue decaying after the input
        // becomes static, so keep both outputs actively processing.
        pinSubOut.setStreaming(true);
        pinMixOut.setStreaming(true);
        setSleep(false);
        setSubProcess(&SubOctaver::subProcess);
    }

private:
    void updateCoefficients(float tone, float tracking, float sensitivity)
    {
        const float sampleRate = (std::max)(1000.0f, getSampleRate());

        // Tone: approximately 40 Hz to 600 Hz.
        const float toneHz = exponentialMap(tone, 40.0f, 600.0f);

        // Tracking filter: approximately 100 Hz to 3000 Hz.
        // Higher values follow higher notes, but may react more to harmonics.
        const float trackingHz = exponentialMap(tracking, 100.0f, 3000.0f);

        // Higher Sensitivity means a lower trigger threshold.
        detectorThreshold_ =
            0.08f * std::pow(0.0005f / 0.08f, sensitivity);

        detectorAlpha_ = onePoleAlpha(trackingHz, sampleRate);
        toneAlpha_ = onePoleAlpha(toneHz, sampleRate);

        detectorDcR_ = std::exp(-2.0f * kPi * 10.0f / sampleRate);
        subDcR_ = std::exp(-2.0f * kPi * 5.0f / sampleRate);

        envelopeAttackCoefficient_ = timeCoefficient(0.004f, sampleRate);
        envelopeReleaseCoefficient_ = timeCoefficient(0.090f, sampleRate);
        gateAttackCoefficient_ = timeCoefficient(0.002f, sampleRate);
        gateReleaseCoefficient_ = timeCoefficient(0.070f, sampleRate);

        // Reject impossible extra edges above about 4 kHz.
        minimumEdgeDistanceSamples_ =
            (std::max)(4, static_cast<int>(sampleRate / 4000.0f));

        // Reset detector after roughly 250 ms without a valid edge.
        detectorResetSamples_ =
            (std::max)(1, static_cast<int>(sampleRate * 0.25f));
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
};

namespace
{
    auto registration =
        Register<SubOctaver>::withId(L"Pandocrator Sub Octaver");
}
