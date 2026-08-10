#include "mp_sdk_audio.h"

#include <algorithm>
#include <cmath>

using namespace gmpi;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kHalfPi = 1.57079632679489661923f;
    constexpr float kReferenceFrequencyHz = 1000.0f;
}

class PhaseRotatorZeroLatency final : public MpBase2
{
    AudioInPin  pinSignalIn;
    FloatInPin  pinPhase;
    AudioOutPin pinSignalOut;

    float sampleRate_ = 44100.0f;
    float currentCoefficient_ = 0.9995f;
    float previousInput_ = 0.0f;
    float previousOutput_ = 0.0f;

public:
    PhaseRotatorZeroLatency()
    {
        initializePin(pinSignalIn);
        initializePin(pinPhase);
        initializePin(pinSignalOut);
    }

    int32_t MP_STDCALL open() override
    {
        const auto result = MpBase2::open();

        sampleRate_ = getSampleRate();
        if (sampleRate_ < 8000.0f)
            sampleRate_ = 44100.0f;

        currentCoefficient_ = coefficientForPhase(
            std::clamp(static_cast<float>(pinPhase), 0.0f, 1.0f));

        previousInput_ = 0.0f;
        previousOutput_ = 0.0f;

        // This module has no fixed delay buffer and reports zero latency.
        host.SetLatency(0);

        return result;
    }

    void subProcess(int sampleFrames)
    {
        auto signalIn = getBuffer(pinSignalIn);
        auto signalOut = getBuffer(pinSignalOut);

        const float phase = std::clamp(
            static_cast<float>(pinPhase), 0.0f, 1.0f);

        const float targetCoefficient = coefficientForPhase(phase);

        const float coefficientStep = sampleFrames > 0
            ? (targetCoefficient - currentCoefficient_) / static_cast<float>(sampleFrames)
            : 0.0f;

        for (int s = sampleFrames; s > 0; --s)
        {
            currentCoefficient_ += coefficientStep;

            const float x = *signalIn++;

            // First-order causal all-pass:
            // y[n] = a*x[n] + x[n-1] - a*y[n-1]
            const float y =
                currentCoefficient_ * x
                + previousInput_
                - currentCoefficient_ * previousOutput_;

            previousInput_ = x;
            previousOutput_ = y;

            *signalOut++ = y;
        }

        currentCoefficient_ = targetCoefficient;
    }

    void onSetPins() override
    {
        pinSignalOut.setStreaming(true);
        setSubProcess(&PhaseRotatorZeroLatency::subProcess);
    }

private:
    float coefficientForPhase(float normalizedPhase) const noexcept
    {
        normalizedPhase = std::clamp(normalizedPhase, 0.0f, 1.0f);

        // The requested rotation is defined at 1 kHz:
        // 0.00 -> approximately   0 degrees
        // 0.50 -> approximately -45 degrees
        // 1.00 -> approximately -90 degrees
        //
        // A causal zero-fixed-latency all-pass cannot maintain the same
        // phase angle over the whole spectrum. The rotation is frequency
        // dependent, and is a phase lag.
        const float desiredPhase = normalizedPhase * kHalfPi;
        const float omega = 2.0f * kPi * kReferenceFrequencyHz / sampleRate_;

        const float tanHalfOmega = std::tan(0.5f * omega);
        if (std::abs(tanHalfOmega) < 1.0e-12f)
            return 0.9995f;

        const float ratio = std::tan(0.5f * desiredPhase) / tanHalfOmega;
        const float a = (1.0f - ratio) / (1.0f + ratio);

        // Keep the pole strictly inside the unit circle.
        // At Phase=0 the mathematical value is +1; 0.9995 is effectively
        // dry while avoiding a marginal pole/cancellation implementation.
        return std::clamp(a, -0.9995f, 0.9995f);
    }
};

namespace
{
    auto r = Register<PhaseRotatorZeroLatency>::withId(
        L"Pandocrator Phase Rotator Zero Latency");
}
