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

    // SynthEdit structure voltages are stored internally at 1/10 of their
    // displayed value. A displayed 0..1 V control therefore arrives here
    // approximately as 0.0..0.1.
    inline float volts01ToNormalized(float internalVoltage)
    {
        return clamp01(internalVoltage * 10.0f);
    }
}

// V4 deliberately separates the *visible structure pins* from the private
// DSP->GUI bridge. The inputs below are real AudioInPin pins, so a SynthEdit
// voltage cable can be connected directly to Size, Diffusion, Decay, etc.
// Private output parameters carry a low-rate copy to the GUI for repainting.
class ReverbSpaceVisualizerV5 final : public MpBase2
{
public:
    ReverbSpaceVisualizerV5()
    {
        // Private DSP -> GUI parameter outputs, IDs 0..24.
        initializePin(outSize);
        initializePin(outDiffusion);
        initializePin(outDecay);
        initializePin(outLowDamp);
        initializePin(outHighDamp);
        initializePin(outPreDelay);
        initializePin(outEarlyLevel);
        initializePin(outLateLevel);
        initializePin(outDensity);
        initializePin(outGlow);
        initializePin(outLineWidth);
        initializePin(outMirror);
        initializePin(outBrightness);
        initializePin(outCurveLevel);
        initializePin(outBgR);
        initializePin(outBgG);
        initializePin(outBgB);
        initializePin(outMainR);
        initializePin(outMainG);
        initializePin(outMainB);
        initializePin(outHiR);
        initializePin(outHiG);
        initializePin(outHiB);
        initializePin(outWidthPx);
        initializePin(outHeightPx);

        // Visible structure-view input pins, IDs 25..49.
        initializePin(inSize);
        initializePin(inDiffusion);
        initializePin(inDecay);
        initializePin(inLowDamp);
        initializePin(inHighDamp);
        initializePin(inPreDelay);
        initializePin(inEarlyLevel);
        initializePin(inLateLevel);
        initializePin(inDensity);
        initializePin(inGlow);
        initializePin(inLineWidth);
        initializePin(inMirror);
        initializePin(inBrightness);
        initializePin(inCurveLevel);
        initializePin(inBgR);
        initializePin(inBgG);
        initializePin(inBgB);
        initializePin(inMainR);
        initializePin(inMainG);
        initializePin(inMainB);
        initializePin(inHiR);
        initializePin(inHiG);
        initializePin(inHiB);
        initializePin(inWidthPx);
        initializePin(inHeightPx);
    }

    void subProcess(int bufferOffset, int sampleFrames)
    {
        if (sampleFrames <= 0)
            return;

        samplesUntilGuiUpdate_ -= sampleFrames;
        if (samplesUntilGuiUpdate_ > 0)
            return;

        // Around 30 GUI updates/second. This is fast enough for smooth visual
        // modulation without sending parameter messages at audio rate.
        const float sr = (std::max)(1000.0f, getSampleRate());
        samplesUntilGuiUpdate_ = (std::max)(1, static_cast<int>(sr / 30.0f));

        const int samplePos = bufferOffset + sampleFrames - 1;
        copyToGui(samplePos);
    }

    void onSetPins() override
    {
        // Send constant/control changes immediately too, then keep sampling any
        // streaming modulation at ~30 fps.
        copyToGui(-1);
        setSleep(false);
        setSubProcess(&ReverbSpaceVisualizerV5::subProcess);
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

    // Private outputs bound to private Parameters and the GUI pins.
    FloatOutPin outSize;
    FloatOutPin outDiffusion;
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

    // REAL cable-connectable Structure View inputs.
    AudioInPin inSize;
    AudioInPin inDiffusion;
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

    int samplesUntilGuiUpdate_ = 0;
};

namespace
{
    auto registration =
        Register<ReverbSpaceVisualizerV5>::withId(L"Pandocrator Reverb Space Visualizer v5");
}
