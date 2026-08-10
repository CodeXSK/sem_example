#include "mp_sdk_audio.h"

using namespace gmpi;

class SemProbe final : public MpBase2
{
    AudioInPin  pinSignalIn;
    AudioOutPin pinSignalOut;

public:
    SemProbe()
    {
        initializePin(pinSignalIn);
        initializePin(pinSignalOut);
    }

    void subProcess(int sampleFrames)
    {
        auto in = getBuffer(pinSignalIn);
        auto out = getBuffer(pinSignalOut);

        for (int s = sampleFrames; s > 0; --s)
            *out++ = *in++;
    }

    void onSetPins() override
    {
        pinSignalOut.setStreaming(true);
        setSubProcess(&SemProbe::subProcess);
    }
};

REGISTER_PLUGIN2(
    SemProbe,
    L"Pandocrator SEM Probe"
);
