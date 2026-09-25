#include "mp_sdk_audio.h"

#include <algorithm>
#include <cmath>

using namespace gmpi;

namespace
{
    inline float clamp01(float v)
    {
        if (!std::isfinite(v))
            return 0.0f;
        return (std::max)(0.0f, (std::min)(1.0f, v));
    }

    // SynthEdit's displayed structure voltage is 10x the SDK's internal value.
    // Therefore a displayed 0..1 V cable arrives approximately as 0.0..0.1.
    inline float volts01ToNormalized(float internalVoltage)
    {
        return clamp01(internalVoltage * 10.0f);
    }
}

// GUI/control-only visualizer. The DSP side exists only so the module has
// normal cable-connectable Structure View pins and can forward those values
// to the GUI. No audio is processed or output.
class ReverbSpaceVisualizerV6 final : public MpBase2
{
public:
    ReverbSpaceVisualizerV6()
    {
        // IMPORTANT: initializePin() order matches the <Audio> pin order in XML.
        // Visible Structure View inputs first.
        initializePin(inSize);          // 0
        initializePin(inDiffusion);     // 1
        initializePin(inFeedback);      // 2
        initializePin(inDecay);         // 3
        initializePin(inLowDamp);       // 4
        initializePin(inHighDamp);      // 5
        initializePin(inPreDelay);      // 6
        initializePin(inEarlyLevel);    // 7
        initializePin(inLateLevel);     // 8
        initializePin(inDensity);       // 9
        initializePin(inGlow);          // 10
        initializePin(inLineWidth);     // 11
        initializePin(inMirror);        // 12
        initializePin(inBrightness);    // 13
        initializePin(inCurveLevel);    // 14
        initializePin(inBgR);           // 15
        initializePin(inBgG);           // 16
        initializePin(inBgB);           // 17
        initializePin(inMainR);         // 18
        initializePin(inMainG);         // 19
        initializePin(inMainB);         // 20
        initializePin(inHiR);           // 21
        initializePin(inHiG);           // 22
        initializePin(inHiB);           // 23
        initializePin(inWidthPx);       // 24
        initializePin(inHeightPx);      // 25

        // Private DSP -> GUI bridge outputs.
        initializePin(outSize);         // 26
        initializePin(outDiffusion);    // 27
        initializePin(outFeedback);     // 28
        initializePin(outDecay);        // 29
        initializePin(outLowDamp);      // 30
        initializePin(outHighDamp);     // 31
        initializePin(outPreDelay);     // 32
        initializePin(outEarlyLevel);   // 33
        initializePin(outLateLevel);    // 34
        initializePin(outDensity);      // 35
        initializePin(outGlow);         // 36
        initializePin(outLineWidth);    // 37
        initializePin(outMirror);       // 38
        initializePin(outBrightness);   // 39
        initializePin(outCurveLevel);   // 40
        initializePin(outBgR);          // 41
        initializePin(outBgG);          // 42
        initializePin(outBgB);          // 43
        initializePin(outMainR);        // 44
        initializePin(outMainG);        // 45
        initializePin(outMainB);        // 46
        initializePin(outHiR);          // 47
        initializePin(outHiG);          // 48
        initializePin(outHiB);          // 49
        initializePin(outWidthPx);      // 50
        initializePin(outHeightPx);     // 51
    }

    void subProcess(int sampleFrames)
    {
        if (sampleFrames <= 0)
            return;

        samplesUntilGuiUpdate_ -= sampleFrames;
        if (samplesUntilGuiUpdate_ > 0)
            return;

        // ~30 GUI updates/sec: smooth enough visually without audio-rate GUI traffic.
        const float sr = (std::max)(1000.0f, getSampleRate());
        samplesUntilGuiUpdate_ = (std::max)(1, static_cast<int>(sr / 30.0f));

        const int samplePos = sampleFrames - 1;
        copyToGui(samplePos);
    }

    void onSetPins() override
    {
        // Also react immediately to static/control changes.
        copyToGui(-1);
        setSleep(false);
        setSubProcess(&ReverbSpaceVisualizerV6::subProcess);
    }

private:
    void copyToGui(int samplePos)
    {
        const auto readNorm = [samplePos](const AudioInPin& p)
        {
            return volts01ToNormalized(p.getValue(samplePos));
        };

        outSize       = readNorm(inSize);
        outDiffusion  = readNorm(inDiffusion);
        outFeedback   = readNorm(inFeedback);
        outDecay      = readNorm(inDecay);
        outLowDamp    = readNorm(inLowDamp);
        outHighDamp   = readNorm(inHighDamp);
        outPreDelay   = readNorm(inPreDelay);
        outEarlyLevel = readNorm(inEarlyLevel);
        outLateLevel  = readNorm(inLateLevel);
        outDensity    = readNorm(inDensity);
        outGlow       = readNorm(inGlow);
        outLineWidth  = readNorm(inLineWidth);
        outMirror     = readNorm(inMirror);
        outBrightness = readNorm(inBrightness);
        outCurveLevel = readNorm(inCurveLevel);

        outBgR   = readNorm(inBgR);
        outBgG   = readNorm(inBgG);
        outBgB   = readNorm(inBgB);
        outMainR = readNorm(inMainR);
        outMainG = readNorm(inMainG);
        outMainB = readNorm(inMainB);
        outHiR   = readNorm(inHiR);
        outHiG   = readNorm(inHiG);
        outHiB   = readNorm(inHiB);

        int w = inWidthPx.getValue();
        int h = inHeightPx.getValue();
        w = (std::max)(160, (std::min)(2400, w));
        h = (std::max)(90,  (std::min)(1400, h));
        outWidthPx = w;
        outHeightPx = h;
    }

    // Visible cable-connectable Structure View inputs.
    AudioInPin inSize;
    AudioInPin inDiffusion;
    AudioInPin inFeedback;
    AudioInPin inDecay;
    AudioInPin inLowDamp;
    AudioInPin inHighDamp;
    AudioInPin inPreDelay;
    AudioInPin inEarlyLevel;
    AudioInPin inLateLevel;
    AudioInPin inDensity;
    AudioInPin inGlow;
    AudioInPin inLineWidth;
    AudioInPin inMirror;
    AudioInPin inBrightness;
    AudioInPin inCurveLevel;
    AudioInPin inBgR;
    AudioInPin inBgG;
    AudioInPin inBgB;
    AudioInPin inMainR;
    AudioInPin inMainG;
    AudioInPin inMainB;
    AudioInPin inHiR;
    AudioInPin inHiG;
    AudioInPin inHiB;
    IntInPin inWidthPx;
    IntInPin inHeightPx;

    // Private bridge outputs bound to private parameters and GUI pins.
    FloatOutPin outSize;
    FloatOutPin outDiffusion;
    FloatOutPin outFeedback;
    FloatOutPin outDecay;
    FloatOutPin outLowDamp;
    FloatOutPin outHighDamp;
    FloatOutPin outPreDelay;
    FloatOutPin outEarlyLevel;
    FloatOutPin outLateLevel;
    FloatOutPin outDensity;
    FloatOutPin outGlow;
    FloatOutPin outLineWidth;
    FloatOutPin outMirror;
    FloatOutPin outBrightness;
    FloatOutPin outCurveLevel;
    FloatOutPin outBgR;
    FloatOutPin outBgG;
    FloatOutPin outBgB;
    FloatOutPin outMainR;
    FloatOutPin outMainG;
    FloatOutPin outMainB;
    FloatOutPin outHiR;
    FloatOutPin outHiG;
    FloatOutPin outHiB;
    IntOutPin outWidthPx;
    IntOutPin outHeightPx;

    int samplesUntilGuiUpdate_ = 0;
};

namespace
{
    auto registration =
        Register<ReverbSpaceVisualizerV6>::withId(L"Pandocrator Reverb Space Visualizer v6");
}
