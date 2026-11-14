#include "mp_sdk_audio.h"
#include "WaveshaperDSP.h"
#include <cmath>
using namespace gmpi;
using pandocrator::clip;
using pandocrator::finite;

class Waveshaper final : public MpBase2 {
public:
    Waveshaper(){
        initializePin(in_);initializePin(mode_);initializePin(drive_);initializePin(a_);
        initializePin(b_);initializePin(bias_);initializePin(wet_);initializePin(output_);
        initializePin(dcBlock_);initializePin(oversampling_);initializePin(wetOut_);initializePin(mixOut_);
    }
    void onSetPins() override {setSubProcess(&Waveshaper::process);}
    // Same pin scale as AllPassDiffuser v9: SynthEdit displays 10 units
    // for each internal audio-pin float unit. XML defaults are /10.
    // Applies ONLY to controls; audio signal processing is unchanged.
    static constexpr float kControlPinToDisplayed = 10.0f;
    static float controlValue(float raw, float fallback = 0.f) {
        return finite(raw, fallback / kControlPinToDisplayed) * kControlPinToDisplayed;
    }
    void process(int sampleFrames){
        const auto input=getBuffer(in_), modes=getBuffer(mode_), drives=getBuffer(drive_);
        const auto as=getBuffer(a_), bs=getBuffer(b_), biases=getBuffer(bias_);
        const auto wets=getBuffer(wet_), gains=getBuffer(output_), dcs=getBuffer(dcBlock_);
        const auto oss=getBuffer(oversampling_);
        auto wetOutput=getBuffer(wetOut_), mixedOutput=getBuffer(mixOut_);
        const float sr=clip(finite(getSampleRate(),44100.f),8000.f,384000.f);
        const float r=std::exp(-2.f*pandocrator::pi*5.f/sr);
        for(int i=0;i<sampleFrames;++i){
            // SynthEdit: +/-10 V is +/-1.0 normalized full-scale.
            const float x=clip(finite(input[i])/pandocrator::kVoltsPerFullScale,-8.f,8.f);
            const int mode=static_cast<int>(std::round(clip(controlValue(modes[i]),0.f,39.f)));
            const float drive=clip(controlValue(drives[i],1.f),0.f,32.f);
            const float a=clip(controlValue(as[i],1.f),-8.f,8.f);
            const float b=clip(controlValue(bs[i],1.f),-8.f,8.f);
            const float bias=clip(controlValue(biases[i]),-4.f,4.f);
            const float mix=clip(controlValue(wets[i],1.f),0.f,1.f);
            const float gain=clip(controlValue(gains[i],1.f),0.f,8.f);
            // Input is enumerated index 0..4, mapped to x1, x2, x4, x8, x16.
            const int stages=static_cast<int>(std::round(clip(controlValue(oss[i]),0.f,4.f)));
            // Align dry path to the measured integer group delay of FIR stages.
            // Stage changes reset the FIRs; they can still click during playback.
            constexpr int dryLatency[5]={0,23,35,41,44};
            if(stages!=previousStages_){
                dryDelay_.fill(0.f);
                previousStages_=stages;
            }
            dryDelay_[dryWrite_]=x;
            const int delayedIndex=(dryWrite_+static_cast<int>(dryDelay_.size())-dryLatency[stages])%static_cast<int>(dryDelay_.size());
            const float delayedDry=dryDelay_[delayedIndex];
            dryWrite_=(dryWrite_+1)%static_cast<int>(dryDelay_.size());
            const float staticZero=pandocrator::shaper(bias,mode,a,b);
            float s=oversampler_.process(x,stages,[&](float highRate){
                const float z=clip(drive*highRate+bias,-24.f,24.f);
                return clip(finite(pandocrator::shaper(z,mode,a,b)-staticZero),-32.f,32.f);
            });
            // Running DC high-pass filter, optional audio pin bypass.
            float hp=r*(highPrevious_+s-inputPrevious_);
            inputPrevious_=s;
            highPrevious_=std::abs(hp)<1.e-20f?0.f:hp;
            if(controlValue(dcs[i],1.f)>=.5f)s=hp;
            // DSP operates normalized; return to native SynthEdit audio volts.
            wetOutput[i]=clip(s*gain,-32.f,32.f)*pandocrator::kVoltsPerFullScale;
            mixedOutput[i]=clip(((1.f-mix)*delayedDry+mix*s)*gain,-32.f,32.f)*pandocrator::kVoltsPerFullScale;
        }
    }
private:
    AudioInPin in_, mode_, drive_, a_, b_, bias_, wet_, output_, dcBlock_, oversampling_;
    AudioOutPin wetOut_, mixOut_;
    pandocrator::Oversampler oversampler_;
    float highPrevious_=0.f,inputPrevious_=0.f;
    std::array<float,128> dryDelay_{};
    int dryWrite_=0,previousStages_=-1;
};
namespace {auto registration=Register<Waveshaper>::withId(L"Pandocrator Waveshaper Toolkit");}
