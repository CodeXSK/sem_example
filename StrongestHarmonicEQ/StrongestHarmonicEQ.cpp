#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "mp_sdk_audio.h"
#include "StrongestPeakEngine.h"
#include "HarmonicIsolator.h"

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

    AudioOutPin pinHarmonicOnly;   // 9 (append-only: preserve existing IDs)
    FloatInPin pinIsolationQ;      // 10

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

        initializePin(pinHarmonicOnly);
        initializePin(pinIsolationQ);

        // Be robust to host/startup callback ordering. onSetPins() installs
        // the same function again during normal SynthEdit startup.
        setSubProcess(&StrongestHarmonicEQ::subProcess);
    }

    void onGraphStart() override
    {
        engine_.prepare(getSampleRate());
        isolator_.prepare(getSampleRate());
        isolator_.setQ(static_cast<float>(pinIsolationQ));
        updateParameters();

        pinAudioOut.setStreaming(true);
        pinHarmonicOnly.setStreaming(true);

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

        if (pinIsolationQ.isUpdated())
            isolator_.setQ(static_cast<float>(pinIsolationQ));

        pinAudioOut.setStreaming(true);
        pinHarmonicOnly.setStreaming(true);
        setSleep(false);

        setSubProcess(&StrongestHarmonicEQ::subProcess);
    }

    void subProcess(int sampleFrames)
    {
        auto* input  = getBuffer(pinAudioIn);
        auto* output = getBuffer(pinAudioOut);
        auto* harmonicOutput = getBuffer(pinHarmonicOnly);

        for (int i = 0; i < sampleFrames; ++i)
        {
            float processed = 0.0f;

            // Cache dry sample FIRST: an in-place host may alias the input
            // and normal output buffers. The extra output must see dry audio.
            const float drySample = input[i];

            const bool analysisUpdated =
                engine_.processSample(drySample, processed);

            // Completely independent band-pass path. The original EQ output
            // is computed by the untouched v5 engine, sample for sample.
            const float isolated = isolator_.process(
                drySample, engine_.getTrackedHz(),
                engine_.getDetectedHz() > 0.0f);

            output[i] = processed;
            harmonicOutput[i] = isolated;

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
    HarmonicIsolator isolator_;
};

namespace
{
    // This ID MUST exactly match the XML Plugin id.
    auto registration =
        Register<StrongestHarmonicEQ>::withId(
            L"Pandocrator Strongest Harmonic EQ SEM");
}
