#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "mp_sdk_audio.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>

using namespace gmpi;

// -----------------------------------------------------------------------------
// Phase Rotator FIR - SynthEdit SDK3 / SEM
//
// Pin declaration order MUST match PhaseRotatorFIR.xml.
//
// Phase:
//   0.00 ->   0 degrees
//   0.25 ->  45 degrees
//   0.50 ->  90 degrees
//   0.75 -> 135 degrees
//   1.00 -> 180 degrees
//
// Uses a 129-tap Blackman-windowed Hilbert transformer.
// Linear-phase group delay = 64 samples.
// -----------------------------------------------------------------------------
class PhaseRotatorFIR final : public MpBase2
{
    AudioInPin  pinAudioIn;   // 0
    AudioOutPin pinAudioOut;  // 1
    FloatInPin  pinPhase;     // 2

public:
    PhaseRotatorFIR()
    {
        // Keep exactly the same order as the XML pin IDs.
        initializePin(pinAudioIn);
        initializePin(pinAudioOut);
        initializePin(pinPhase);

        buildHilbertTransformer();
    }

    void onGraphStart() override
    {
        delayLine_.fill(0.0f);
        writeIndex_ = 0;

        currentPhase_ =
            clamp01(static_cast<float>(pinPhase));

        // Current SDK3 exposes ProcessorHost::SetLatency().
        // This tells SynthEdit that this DSP path has 64 samples delay.
        host.SetLatency(kGroupDelaySamples);

        pinAudioOut.setStreaming(true);
        setSleep(false);
    }

    void onSetPins() override
    {
        pinAudioOut.setStreaming(true);
        setSleep(false);

        setSubProcess(
            &PhaseRotatorFIR::subProcess);
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

            delayLine_[writeIndex_] = x;

            // Hilbert branch.
            float quadrature = 0.0f;

            int readIndex = writeIndex_;

            for (int tap = 0;
                 tap < kTapCount;
                 ++tap)
            {
                quadrature +=
                    hilbertCoefficients_[tap]
                    * delayLine_[readIndex];

                --readIndex;

                if (readIndex < 0)
                    readIndex = kTapCount - 1;
            }

            // Match direct branch to FIR group delay.
            int directIndex =
                writeIndex_ - kGroupDelaySamples;

            if (directIndex < 0)
                directIndex += kTapCount;

            const float delayedInput =
                delayLine_[directIndex];

            currentPhase_ += phaseStep;

            const float theta =
                currentPhase_ * kPi;

            // Hilbert branch is approximately -90 degrees.
            // Negating the sin branch gives the positive-rotation convention.
            output[i] =
                delayedInput * std::cos(theta)
                - quadrature * std::sin(theta);

            ++writeIndex_;

            if (writeIndex_ >= kTapCount)
                writeIndex_ = 0;
        }

        currentPhase_ = targetPhase;
    }

private:
    static constexpr int kTapCount = 129;

    static constexpr int kGroupDelaySamples =
        (kTapCount - 1) / 2;

    static constexpr float kPi =
        3.14159265358979323846f;

    std::array<float, kTapCount>
        hilbertCoefficients_ {};

    std::array<float, kTapCount>
        delayLine_ {};

    int writeIndex_ = 0;
    float currentPhase_ = 0.0f;

    static float clamp01(float value)
    {
        return (std::max)(
            0.0f,
            (std::min)(1.0f, value));
    }

    void buildHilbertTransformer()
    {
        constexpr int centre =
            kGroupDelaySamples;

        for (int n = 0;
             n < kTapCount;
             ++n)
        {
            const int m =
                n - centre;

            float h = 0.0f;

            // Ideal Hilbert impulse response.
            if (m != 0
                && (std::abs(m) & 1))
            {
                h =
                    2.0f
                    / (kPi
                       * static_cast<float>(m));
            }

            const float angle =
                2.0f
                * kPi
                * static_cast<float>(n)
                / static_cast<float>(
                    kTapCount - 1);

            const float blackman =
                0.42f
                - 0.50f * std::cos(angle)
                + 0.08f * std::cos(
                    2.0f * angle);

            hilbertCoefficients_[n] =
                h * blackman;
        }
    }
};

namespace
{
    // This ID MUST exactly match the XML Plugin id.
    auto registration =
        Register<PhaseRotatorFIR>::withId(
            L"Pandocrator Phase Rotator FIR SEM");
}
