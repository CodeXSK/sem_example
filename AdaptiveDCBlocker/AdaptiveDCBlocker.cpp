// Adaptive DC Blocker + zero-tail state reset, stereo linked.
// SynthEdit SDK3 / legacy SEM module (MpBase2 API).
#include "mp_sdk_audio.h"
#include "AdaptiveDCBlockerDSP.h"

using namespace gmpi;

class AdaptiveDCBlocker final : public MpBase2
{
public:
    AdaptiveDCBlocker()
    {
        // Pin order is EXACTLY the order in AdaptiveDCBlocker.xml.
        initializePin(pinInL);
        initializePin(pinInR);
        initializePin(pinDetectIn);
        initializePin(pinDetectExternal);
        initializePin(pinCutoffHz);
        initializePin(pinSlope);
        initializePin(pinThresholdDb);
        initializePin(pinHoldMs);
        initializePin(pinTailFadeMs);
        initializePin(pinWakeFadeMs);
        initializePin(pinOutL);
        initializePin(pinOutR);
        initializePin(pinActivityGainOut);
    }

    void onSetPins() override
    {
        const auto sr = getSampleRate();
        if (sr != lastSampleRate_)
        {
            dsp_.prepare(sr);
            lastSampleRate_ = sr;
        }

        pinOutL.setStreaming(true);
        pinOutR.setStreaming(true);
        pinActivityGainOut.setStreaming(true);
        setSleep(false); // Process filter/fade state even after audio pins go static.
        setSubProcess(&AdaptiveDCBlocker::subProcess);
    }

    void subProcess(int sampleFrames)
    {
        if (sampleFrames <= 0)
            return;

        const auto sr = getSampleRate();
        if (sr != lastSampleRate_)
        {
            dsp_.prepare(sr);
            lastSampleRate_ = sr;
        }

        auto inL = getBuffer(pinInL);
        auto inR = getBuffer(pinInR);
        auto detector = getBuffer(pinDetectIn);
        const bool externalDetect = *getBuffer(pinDetectExternal) >= 0.5f;
        const float cutoff = *getBuffer(pinCutoffHz);
        const float slope = *getBuffer(pinSlope);
        const float threshold = *getBuffer(pinThresholdDb);
        const float holdMs = *getBuffer(pinHoldMs);
        const float tailMs = *getBuffer(pinTailFadeMs);
        const float wakeMs = *getBuffer(pinWakeFadeMs);
        auto outL = getBuffer(pinOutL);
        auto outR = getBuffer(pinOutR);
        auto gate = getBuffer(pinActivityGainOut);

        // Read knobs at each SynthEdit processing block, like the working v7 SEM.
        dsp_.update(cutoff, threshold, holdMs, tailMs, wakeMs, slope);

        for (int s = 0; s < sampleFrames; ++s)
        {
            const auto sample = dsp_.process(*inL++, *inR++, *detector++, externalDetect);
            *outL++ = sample.left;
            *outR++ = sample.right;
            *gate++ = sample.gain;
        }
    }

private:
    AudioInPin pinInL;
    AudioInPin pinInR;
    AudioInPin pinDetectIn;
    AudioInPin pinDetectExternal;
    AudioInPin pinCutoffHz;
    AudioInPin pinSlope;
    AudioInPin pinThresholdDb;
    AudioInPin pinHoldMs;
    AudioInPin pinTailFadeMs;
    AudioInPin pinWakeFadeMs;
    AudioOutPin pinOutL;
    AudioOutPin pinOutR;
    AudioOutPin pinActivityGainOut;

    AdaptiveDCBlockerDSP dsp_;
    float lastSampleRate_ = -1.0f;
};

namespace
{
    // Unique identifier. Existing patches/modules remain untouched.
    auto registration =
        Register<AdaptiveDCBlocker>::withId(L"Pandocrator Adaptive DC Blocker v1");
}
