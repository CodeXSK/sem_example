#include "mp_sdk_audio.h"
#include <algorithm>
#include <cmath>
using namespace gmpi;
class ModulationRateLimiter : public MpBase2
{
    AudioInPin input;
    AudioOutPin pass, scaled, held, reference;
    int remaining = 0;
    float last = 0.0f;
public:
    ModulationRateLimiter()
    {
        initializePin(input);
        initializePin(pass);
        initializePin(scaled);
        initializePin(held);
        initializePin(reference);
    }
    void onSetPins() override
    {
        pass.setStreaming(true);
        scaled.setStreaming(true);
        held.setStreaming(true);
        reference.setStreaming(true);
        setSleep(false);
        setSubProcess(&ModulationRateLimiter::subProcess);
    }
    void subProcess(int n)
    {
        if(n <= 0) return;
        auto x = getBuffer(input);
        auto y = getBuffer(pass);
        auto z = getBuffer(scaled);
        auto h = getBuffer(held);
        auto r = getBuffer(reference);
        float sr = getSampleRate();
        if(!std::isfinite(sr) || sr < 1000.f || sr > 768000.f) sr=48000.f;
        const int interval = static_cast<int>(std::ceil(sr / 30.f));
        for(int i=0;i<n;++i)
        {
            const float v=std::isfinite(x[i])?x[i]:0.f;
            const float s=(std::max)(0.f,(std::min)(1.f,v*.5f));
            if(remaining<=0){last=s;remaining=interval;}
            --remaining;
            y[i]=v; z[i]=s; h[i]=last; r[i]=0.5f;
        }
    }
};
namespace { auto registration=Register<ModulationRateLimiter>::withId(L"Pandocrator Modulation Rate Limiter 1.3"); }
