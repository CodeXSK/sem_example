#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "mp_sdk_audio.h"
#include "StrongestPeakEngine.h"

using namespace gmpi;

// -----------------------------------------------------------------------------
// Strongest Harmonic EQ - SynthEdit SDK3 / SEM
//
// IMPORTANT:
// This detects the strongest SPECTRAL PEAK, not necessarily the true
// fundamental frequency.
//
// Pin declaration order MUST match StrongestHarmonicEQ.xml.
// -----------------------------------------------------------------------------
class StrongestHarmonicEQ final : public MpBase2
{
    AudioInPin  pinAudioIn;       // 0
    AudioOutPin pinAudioOut;      // 1

    FloatInPin  pinBoostDb;       // 2
    FloatInPin  pinQ;             // 3
    FloatInPin  pinMinFrequency;  // 4
    FloatInPin  pinMaxFrequency;  // 5
    FloatInPin  pinTrackingMs;    // 6

    FloatOutPin pinDetectedHz;     // 7
    FloatOutPin pinEqHz;           // 8

public:
    StrongestHarmonicEQ()
    {
        // SDK3 pin registration.
        // Keep exactly the same order as the XML pin IDs.
        initializePin(pinAudioIn);
        initializePin(pinAudioOut);

        initializePin(pinBoostDb);
        initializePin(pinQ);
        initializePin(pinMinFrequency);
        initializePin(pinMaxFrequency);
        initializePin(pinTrackingMs);

        initializePin(pinDetectedHz);
        initializePin(pinEqHz);
    }

    void onGraphStart() override
    {
        engine_.prepare(getSampleRate());
        updateParameters();

        pinAudioOut.setStreaming(true);

        // The FFT needs continuous history while this module is active.
        setSleep(false);

        pinDetectedHz = 0.0f;
        pinEqHz = 0.0f;
    }

    void onSetPins() override
    {
        // SynthEdit does not guarantee incoming control values are inside
        // metadata ranges. StrongestPeakEngine clamps all of them internally.
        if (pinBoostDb.isUpdated()
            || pinQ.isUpdated()
            || pinMinFrequency.isUpdated()
            || pinMaxFrequency.isUpdated()
            || pinTrackingMs.isUpdated())
        {
            updateParameters();
        }

        pinAudioOut.setStreaming(true);
        setSleep(false);

        setSubProcess(&StrongestHarmonicEQ::subProcess);
    }

    void subProcess(int sampleFrames)
    {
        auto* input  = getBuffer(pinAudioIn);
        auto* output = getBuffer(pinAudioOut);

        for (int i = 0; i < sampleFrames; ++i)
        {
            float processed = 0.0f;

            const bool analysisUpdated =
                engine_.processSample(input[i], processed);

            output[i] = processed;

            if (analysisUpdated)
            {
                // FloatOutPin supports timestamped updates when the module's
                // block position is made exact for this sample.
                TempBlockPositionSetter exactPosition(
                    this,
                    getBlockPosition() + i);

                pinDetectedHz = engine_.getDetectedHz();
                pinEqHz       = engine_.getTrackedHz();
            }
        }
    }

private:
    void updateParameters()
    {
        engine_.setParameters(
            static_cast<float>(pinBoostDb),
            static_cast<float>(pinQ),
            static_cast<float>(pinMinFrequency),
            static_cast<float>(pinMaxFrequency),
            static_cast<float>(pinTrackingMs));
    }

    StrongestPeakEngine engine_;
};

namespace
{
    // This ID MUST exactly match the XML Plugin id.
    auto registration =
        Register<StrongestHarmonicEQ>::withId(
            L"Pandocrator Strongest Harmonic EQ SEM");
}
