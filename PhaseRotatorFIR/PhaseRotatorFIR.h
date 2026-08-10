#pragma once

#include "mp_sdk_audio.h"

#include <array>

class PhaseRotatorFIR : public MpBase
{
public:
    PhaseRotatorFIR(IMpUnknown* host);

    int32_t open() override;

    void subProcess(
        int bufferOffset,
        int sampleFrames
    );

private:
    static constexpr int tapCount  = 129;
    static constexpr int groupDelaySamples =
        (tapCount - 1) / 2;

    AudioInPin  pinInput;
    FloatInPin  pinPhase;
    AudioOutPin pinOutput;

    std::array<float, tapCount>
        hilbertCoefficients {};

    std::array<float, tapCount>
        delayLine {};

    int writeIndex = 0;

    float currentPhase = 0.0f;

    void buildHilbertTransformer();
};
