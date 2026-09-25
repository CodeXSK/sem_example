#include "mp_sdk_audio.h"

using namespace gmpi;

// Lightweight DSP/control bridge. The visual parameters are exposed as real
// SynthEdit structure pins. The matching parameterIds in the XML mirror their
// values to the GUI side, so the visualizer can be driven by SE voltage/control
// signals while the GUI repaints independently.
class ReverbSpaceVisualizerV3 final : public MpBase2
{
public:
    ReverbSpaceVisualizerV3()
    {
        initializePin(pinSize);
        initializePin(pinDiffusion);
        initializePin(pinDecay);
        initializePin(pinLowDamp);
        initializePin(pinHighDamp);
        initializePin(pinPreDelay);
        initializePin(pinEarlyLevel);
        initializePin(pinLateLevel);
        initializePin(pinDensity);
        initializePin(pinGlow);
        initializePin(pinLineWidth);
        initializePin(pinMirror);
        initializePin(pinBrightness);
        initializePin(pinCurveLevel);

        initializePin(pinBgR);
        initializePin(pinBgG);
        initializePin(pinBgB);
        initializePin(pinMainR);
        initializePin(pinMainG);
        initializePin(pinMainB);
        initializePin(pinHiR);
        initializePin(pinHiG);
        initializePin(pinHiB);

        initializePin(pinWidthPx);
        initializePin(pinHeightPx);
    }

    void onSetPins() override
    {
        // No audio processing is required. Keeping a DSP-side class gives the
        // SEM a normal processor factory and real structure pins, while the
        // host's parameter bridge communicates values to the GUI pins.
        setSleep(true);
    }

private:
    AudioInPin pinSize;
    AudioInPin pinDiffusion;
    AudioInPin pinDecay;
    AudioInPin pinLowDamp;
    AudioInPin pinHighDamp;
    AudioInPin pinPreDelay;
    AudioInPin pinEarlyLevel;
    AudioInPin pinLateLevel;
    AudioInPin pinDensity;
    AudioInPin pinGlow;
    AudioInPin pinLineWidth;
    AudioInPin pinMirror;
    AudioInPin pinBrightness;
    AudioInPin pinCurveLevel;

    AudioInPin pinBgR;
    AudioInPin pinBgG;
    AudioInPin pinBgB;
    AudioInPin pinMainR;
    AudioInPin pinMainG;
    AudioInPin pinMainB;
    AudioInPin pinHiR;
    AudioInPin pinHiG;
    AudioInPin pinHiB;

    IntInPin pinWidthPx;
    IntInPin pinHeightPx;
};

namespace
{
    auto registration =
        Register<ReverbSpaceVisualizerV3>::withId(L"Pandocrator Reverb Space Visualizer v3");
}
