#include "mp_sdk_audio.h"
#include <algorithm>
#include <cmath>
using namespace gmpi;
class ModulationIndicatorBridge : public MpBase2 {
 AudioInPin input;
 FloatOutPin value, rawVolts, heartbeat, display;
 AudioOutPin pass;
 FloatInPin rate;
 int remaining=0, period=1600, heartbeatRemaining=0;
 bool beat=false;
 void configure(){const float sr=getSampleRate(); const float r=rate; period=static_cast<int>(std::ceil((std::isfinite(sr)&&sr>=1000&&sr<=10000000?sr:48000.0f)/(std::isfinite(r)?(std::max)(1.f,(std::min)(60.f,r)):30.f)));}
public:
 ModulationIndicatorBridge(){initializePin(input);initializePin(value);initializePin(rawVolts);initializePin(heartbeat);initializePin(display);initializePin(pass);initializePin(rate);setSubProcess(&ModulationIndicatorBridge::subProcess);}
 void onGraphStart() override {remaining=heartbeatRemaining=0;beat=false;configure();setSubProcess(&ModulationIndicatorBridge::subProcess);setSleep(false);pass.setStreaming(true);}
 void onSetPins() override {configure();setSubProcess(&ModulationIndicatorBridge::subProcess);setSleep(false);pass.setStreaming(true);}
 void subProcess(int n){if(n<=0)return;const float* x=getBuffer(input);float* y=getBuffer(pass);for(int i=0;i<n;++i)y[i]=std::isfinite(x[i])?x[i]:0.f;
  int i=remaining;while(i<n){float v=std::isfinite(x[i])?x[i]*10.f:0.f;if(!std::isfinite(v))v=0;float p=(std::max)(0.f,(std::min)(10.f,v*.5f));const int t=getBlockPosition()+i;value.setValue(p,t);rawVolts.setValue(v,t);display.setValue(p,t);i+=period;}remaining=i-n;
  const int halfSecond=(std::max)(1,static_cast<int>(getSampleRate()*.5f));i=heartbeatRemaining;while(i<n){beat=!beat;heartbeat.setValue(beat?1.f:0.f,getBlockPosition()+i);i+=halfSecond;}heartbeatRemaining=i-n;
 }
};
namespace {auto registration=Register<ModulationIndicatorBridge>::withId(L"Pandocrator Modulation Bridge Probe 1.2");}
