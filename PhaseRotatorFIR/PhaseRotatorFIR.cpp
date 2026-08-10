#include "PhaseRotatorFIR.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

REGISTER_PLUGIN(
    PhaseRotatorFIR,
    L"Pandocrator Phase Rotator FIR"
);

namespace
{
    constexpr float pi =
        3.14159265358979323846f;
}

PhaseRotatorFIR::PhaseRotatorFIR(
    IMpUnknown* host
)
    : MpBase(host)
{
    initializePin(0, pinInput);
    initializePin(1, pinPhase);
    initializePin(2, pinOutput);
}

int32_t PhaseRotatorFIR::open()
{
    MpBase::open();

    buildHilbertTransformer();

    delayLine.fill(0.0f);
    writeIndex = 0;

    currentPhase =
        std::clamp(
            static_cast<float>(pinPhase),
            0.0f,
            1.0f
        );

    SET_PROCESS(
        &PhaseRotatorFIR::subProcess
    );

    return gmpi::MP_OK;
}

void PhaseRotatorFIR::buildHilbertTransformer()
{
    /*
        129-tap, odd-length FIR Hilbert transformer.

        Ideal Hilbert impulse response:

            h[m] = 2 / (pi*m),  m odd
            h[m] = 0,           m even or zero

        We apply a Blackman window to reduce ripple.

        The corresponding linear-phase group delay is:

            (129 - 1) / 2 = 64 samples
    */

    constexpr int centre =
        groupDelaySamples;

    for(int n = 0; n < tapCount; ++n)
    {
        const int m =
            n - centre;

        float h = 0.0f;

        if(
            m != 0
            && (std::abs(m) & 1)
        )
        {
            h =
                2.0f
                / (
                    pi
                    * static_cast<float>(m)
                );
        }

        const float angle =
            2.0f
            * pi
            * static_cast<float>(n)
            / static_cast<float>(
                tapCount - 1
            );

        const float blackman =
            0.42f
            - 0.50f * std::cos(angle)
            + 0.08f * std::cos(
                2.0f * angle
            );

        hilbertCoefficients[n] =
            h * blackman;
    }
}

void PhaseRotatorFIR::subProcess(
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

    for(int sample = 0;
        sample < sampleFrames;
        ++sample)
    {
        const float x =
            *input++;

        delayLine[writeIndex] = x;

        /*
            Hilbert branch:
            q[n] = sum h[k] * x[n-k]
        */
        float quadrature = 0.0f;

        int readIndex =
            writeIndex;

        for(int tap = 0;
            tap < tapCount;
            ++tap)
        {
            quadrature +=
                hilbertCoefficients[tap]
                * delayLine[readIndex];

            --readIndex;

            if(readIndex < 0)
                readIndex = tapCount - 1;
        }

        /*
            Match the direct branch to the FIR's 64-sample
            linear-phase group delay.
        */
        int directIndex =
            writeIndex
            - groupDelaySamples;

        if(directIndex < 0)
            directIndex += tapCount;

        const float delayedInput =
            delayLine[directIndex];

        currentPhase += phaseStep;

        /*
            Phase mapping:

                0.00 ->   0 degrees
                0.25 ->  45 degrees
                0.50 ->  90 degrees
                0.75 -> 135 degrees
                1.00 -> 180 degrees

            The Hilbert branch approximates -90 degrees.
            Subtracting it in the sin() term gives the
            positive-rotation convention used here.
        */
        const float phaseRadians =
            currentPhase * pi;

        const float dryCoefficient =
            std::cos(phaseRadians);

        const float quadratureCoefficient =
            std::sin(phaseRadians);

        *output++ =
            delayedInput
                * dryCoefficient
            - quadrature
                * quadratureCoefficient;

        ++writeIndex;

        if(writeIndex >= tapCount)
            writeIndex = 0;
    }

    currentPhase = targetPhase;
}
