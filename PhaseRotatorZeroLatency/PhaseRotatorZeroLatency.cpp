#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "mp_sdk_audio.h"

#include <algorithm>
#include <cmath>

using namespace gmpi;

// -----------------------------------------------------------------------------
// Phase Rotator Zero Latency - SynthEdit SDK3 / SEM
//
// Pin declaration order MUST match PhaseRotatorZeroLatency.xml.
//
// This is a first-order all-pass filter.
// "Phase" 0..1 means approximately 0..90 degrees of phase-lag magnitude
// at the 1 kHz reference frequency.
//
// It has NO fixed sample-delay buffer.
// The phase shift is frequency-dependent, as expected from a causal all-pass.
// -----------------------------------------------------------------------------
class PhaseRotatorZeroLatency final : public MpBase2
{
    AudioInPin  pinAudioIn;   // 0
    AudioOutPin pinAudioOut;  // 1
    FloatInPin  pinPhase;     // 2

public:
    PhaseRotatorZeroLatency()
    {
        // Keep exactly the same order as the XML pin IDs.
        initializePin(pinAudioIn);
        initializePin(pinAudioOut);
        initializePin(pinPhase);
    }

    void onGraphStart() override
    {
        sampleRate_ = getSampleRate();

        if (sampleRate_ < 8000.0f)
            sampleRate_ = 44100.0f;

        currentPhase_ =
            clamp01(static_cast<float>(pinPhase));

        previousInput_ = 0.0f;
        previousOutput_ = 0.0f;

        pinAudioOut.setStreaming(true);
        setSleep(false);
    }

    void onSetPins() override
    {
        pinAudioOut.setStreaming(true);
        setSleep(false);

        setSubProcess(
            &PhaseRotatorZeroLatency::subProcess);
    }

    void subProcess(int sampleFrames)
    {
        auto* input  = getBuffer(pinAudioIn);
        auto* output = getBuffer(pinAudioOut);

        const float targetPhase =
            clamp01(static_cast<float>(pinPhase));

        const float phaseStep =
            sampleFrames > 0
                ? (targetPhase - currentPhase_)
                    / static_cast<float>(sampleFrames)
                : 0.0f;

        for (int i = 0; i < sampleFrames; ++i)
        {
            const float x = input[i];

            currentPhase_ += phaseStep;

            // True dry path at exactly 0.
            if (currentPhase_ <= 0.000001f)
            {
                previousInput_ = x;
                previousOutput_ = x;
                output[i] = x;
                continue;
            }

            const float a =
                coefficientForPhase(currentPhase_);

            // First-order all-pass:
            //
            // y[n] = a*x[n] + x[n-1] - a*y[n-1]
            //
            const float y =
                a * x
                + previousInput_
                - a * previousOutput_;

            previousInput_ = x;
            previousOutput_ = y;

            output[i] = y;
        }

        currentPhase_ = targetPhase;
    }

private:
    static constexpr float kPi =
        3.14159265358979323846f;

    static constexpr float kHalfPi =
        1.57079632679489661923f;

    static constexpr float kReferenceFrequencyHz =
        1000.0f;

    float sampleRate_ = 44100.0f;
    float currentPhase_ = 0.0f;

    float previousInput_ = 0.0f;
    float previousOutput_ = 0.0f;

    static float clamp01(float value)
    {
        return (std::max)(
            0.0f,
            (std::min)(1.0f, value));
    }

    float coefficientForPhase(
        float normalizedPhase) const
    {
        normalizedPhase =
            clamp01(normalizedPhase);

        const float desiredPhase =
            normalizedPhase * kHalfPi;

        const float omega =
            2.0f
            * kPi
            * kReferenceFrequencyHz
            / sampleRate_;

        const float tanHalfOmega =
            std::tan(0.5f * omega);

        if (std::abs(tanHalfOmega) < 1.0e-12f)
            return 0.0f;

        const float ratio =
            std::tan(0.5f * desiredPhase)
            / tanHalfOmega;

        float a =
            (1.0f - ratio)
            / (1.0f + ratio);

        // Keep the pole strictly inside the unit circle.
        a = (std::max)(
            -0.9999f,
            (std::min)(0.9999f, a));

        return a;
    }
};

namespace
{
    // This ID MUST exactly match the XML Plugin id.
    auto registration =
        Register<PhaseRotatorZeroLatency>::withId(
            L"Pandocrator Phase Rotator Zero Latency SEM");
}
