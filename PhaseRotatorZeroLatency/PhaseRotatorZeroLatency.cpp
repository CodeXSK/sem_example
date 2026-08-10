#include "PhaseRotatorZeroLatency.h"

#include <algorithm>
#include <cmath>

REGISTER_PLUGIN(
    PhaseRotatorZeroLatency,
    L"Pandocrator Phase Rotator Zero Latency"
);

namespace
{
    constexpr float pi =
        3.14159265358979323846f;

    constexpr float halfPi =
        1.57079632679489661923f;
}

PhaseRotatorZeroLatency::PhaseRotatorZeroLatency(
    IMpUnknown* host
)
    : MpBase(host)
{
    initializePin(0, pinInput);
    initializePin(1, pinPhase);
    initializePin(2, pinOutput);
}

int32_t PhaseRotatorZeroLatency::open()
{
    MpBase::open();

    sampleRate =
        static_cast<double>(getSampleRate());

    if(sampleRate < 8000.0)
        sampleRate = 44100.0;

    currentPhase =
        std::clamp(
            static_cast<float>(pinPhase),
            0.0f,
            1.0f
        );

    previousInput  = 0.0f;
    previousOutput = 0.0f;

    SET_PROCESS(
        &PhaseRotatorZeroLatency::subProcess
    );

    return gmpi::MP_OK;
}

float PhaseRotatorZeroLatency::coefficientForPhase(
    float normalizedPhase
) const noexcept
{
    normalizedPhase =
        std::clamp(
            normalizedPhase,
            0.0f,
            1.0f
        );

    /*
        First-order all-pass:

            H(z) = (a + z^-1) / (1 + a z^-1)

        Difference equation:

            y[n] = a*x[n] + x[n-1] - a*y[n-1]

        |H| = 1 at all frequencies.

        The "Phase" control specifies the magnitude of the
        phase rotation AT 1 kHz:

            0.0 ->   0 degrees
            0.5 ->  45 degrees
            1.0 ->  90 degrees

        A causal first-order all-pass produces phase lag.
        Therefore the actual sign at 1 kHz is 0 ... -90 degrees.

        At other frequencies the phase angle is necessarily
        different. This is the price of doing the job sample-by-sample
        without the fixed 64-sample Hilbert delay used by the FIR module.
    */

    if(normalizedPhase <= 0.000001f)
        return 0.0f; // not used while true-bypassed.

    const float desiredPhase =
        normalizedPhase * halfPi;

    const float omega =
        2.0f
        * pi
        * referenceFrequencyHz
        / static_cast<float>(sampleRate);

    const float tanHalfOmega =
        std::tan(0.5f * omega);

    if(std::abs(tanHalfOmega) < 1.0e-12f)
        return 0.0f;

    const float ratio =
        std::tan(0.5f * desiredPhase)
        / tanHalfOmega;

    float a =
        (1.0f - ratio)
        / (1.0f + ratio);

    // Keep the pole comfortably inside the unit circle.
    a = std::clamp(
        a,
        -0.9995f,
         0.9995f
    );

    return a;
}

void PhaseRotatorZeroLatency::subProcess(
    int bufferOffset,
    int sampleFrames
)
{
    float* input =
        pinInput.getBuffer()
        + bufferOffset;

    float* output =
        pinOutput.getBuffer()
        + bufferOffset;

    const float targetPhase =
        std::clamp(
            static_cast<float>(pinPhase),
            0.0f,
            1.0f
        );

    const float phaseStep =
        sampleFrames > 0
            ? (targetPhase - currentPhase)
                / static_cast<float>(sampleFrames)
            : 0.0f;

    for(int i = 0; i < sampleFrames; ++i)
    {
        const float x = *input++;

        currentPhase += phaseStep;

        /*
            Exact 0 = dry bypass.

            We also continuously align the filter state while bypassed,
            so moving away from zero does not start from stale history.
        */
        if(currentPhase <= 0.000001f)
        {
            previousInput  = x;
            previousOutput = x;

            *output++ = x;
            continue;
        }

        const float a =
            coefficientForPhase(
                currentPhase
            );

        const float y =
            a * x
            + previousInput
            - a * previousOutput;

        previousInput  = x;
        previousOutput = y;

        *output++ = y;
    }

    currentPhase = targetPhase;
}
