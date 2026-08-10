#include "mp_sdk_audio.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>

using namespace gmpi;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
}

class PhaseRotatorFIR final : public MpBase2
{
    static constexpr int kTapCount = 129;
    static constexpr int kGroupDelaySamples = (kTapCount - 1) / 2; // 64

    AudioInPin  pinSignalIn;
    FloatInPin  pinPhase;
    AudioOutPin pinSignalOut;

    std::array<float, kTapCount> hilbertCoefficients_{};
    std::array<float, kTapCount> delayLine_{};

    int writeIndex_ = 0;
    float currentPhase_ = 0.0f;

public:
    PhaseRotatorFIR()
    {
        initializePin(pinSignalIn);
        initializePin(pinPhase);
        initializePin(pinSignalOut);

        buildHilbertTransformer();
    }

    int32_t MP_STDCALL open() override
    {
        const auto result = MpBase2::open();

        delayLine_.fill(0.0f);
        writeIndex_ = 0;
        currentPhase_ = std::clamp(
            static_cast<float>(pinPhase), 0.0f, 1.0f);

        // Inform SynthEdit that this module has 64 samples of fixed latency.
        host.SetLatency(kGroupDelaySamples);

        return result;
    }

    void subProcess(int sampleFrames)
    {
        auto signalIn = getBuffer(pinSignalIn);
        auto signalOut = getBuffer(pinSignalOut);

        const float targetPhase = std::clamp(
            static_cast<float>(pinPhase), 0.0f, 1.0f);

        const float phaseStep = sampleFrames > 0
            ? (targetPhase - currentPhase_) / static_cast<float>(sampleFrames)
            : 0.0f;

        for (int s = sampleFrames; s > 0; --s)
        {
            const float x = *signalIn++;
            delayLine_[writeIndex_] = x;

            // FIR Hilbert branch.
            float quadrature = 0.0f;
            int readIndex = writeIndex_;

            for (int tap = 0; tap < kTapCount; ++tap)
            {
                quadrature += hilbertCoefficients_[tap] * delayLine_[readIndex];

                --readIndex;
                if (readIndex < 0)
                    readIndex = kTapCount - 1;
            }

            // Match the direct branch to the Hilbert transform's
            // linear-phase group delay.
            int directIndex = writeIndex_ - kGroupDelaySamples;
            if (directIndex < 0)
                directIndex += kTapCount;

            const float delayedInput = delayLine_[directIndex];

            currentPhase_ += phaseStep;
            const float theta = currentPhase_ * kPi;

            const float c = std::cos(theta);
            const float q = std::sin(theta);

            // Hilbert(x) is approximately -90 degrees for positive
            // frequencies. Subtracting it produces the positive rotation:
            //
            // 0.00 ->   0 degrees
            // 0.25 ->  45 degrees
            // 0.50 ->  90 degrees
            // 0.75 -> 135 degrees
            // 1.00 -> 180 degrees
            *signalOut++ = delayedInput * c - quadrature * q;

            ++writeIndex_;
            if (writeIndex_ >= kTapCount)
                writeIndex_ = 0;
        }

        currentPhase_ = targetPhase;
    }

    void onSetPins() override
    {
        pinSignalOut.setStreaming(true);
        setSubProcess(&PhaseRotatorFIR::subProcess);
    }

private:
    void buildHilbertTransformer()
    {
        constexpr int centre = kGroupDelaySamples;

        // Windowed ideal odd-symmetric Hilbert transformer.
        // h[m] = 2/(pi*m) for odd m; 0 for even m and m=0.
        // A Blackman window reduces truncation ripple.
        for (int n = 0; n < kTapCount; ++n)
        {
            const int m = n - centre;
            float h = 0.0f;

            if (m != 0 && (std::abs(m) & 1))
                h = 2.0f / (kPi * static_cast<float>(m));

            const float windowAngle =
                2.0f * kPi * static_cast<float>(n)
                / static_cast<float>(kTapCount - 1);

            const float blackman =
                0.42f
                - 0.50f * std::cos(windowAngle)
                + 0.08f * std::cos(2.0f * windowAngle);

            hilbertCoefficients_[n] = h * blackman;
        }
    }
};

namespace
{
    auto r = Register<PhaseRotatorFIR>::withId(
        L"Pandocrator Phase Rotator FIR");
}
