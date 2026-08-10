#pragma once

#include "mp_sdk_audio.h"

class PhaseRotatorZeroLatency : public MpBase
{
public:
    PhaseRotatorZeroLatency(IMpUnknown* host);

    int32_t open() override;

    void subProcess(
        int bufferOffset,
        int sampleFrames
    );

private:
    AudioInPin  pinInput;
    FloatInPin  pinPhase;
    AudioOutPin pinOutput;

    float currentPhase = 0.0f;

    // First-order all-pass state.
    float previousInput  = 0.0f;
    float previousOutput = 0.0f;

    double sampleRate = 44100.0;

    static constexpr float referenceFrequencyHz = 1000.0f;

    float coefficientForPhase(
        float normalizedPhase
    ) const noexcept;
};
