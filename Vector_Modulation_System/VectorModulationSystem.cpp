#include "mp_sdk_audio.h"
#include "IndicatorCore.h"
using namespace gmpi;
// Use the same audio-pin processing setup as the user-confirmed v1.3.
class VectorModulationSystem : public MpBase2 {
 AudioInPin input;
 FloatInPin minV,maxV,rate,smoothing,deadband;
 FloatOutPin display;
 AudioOutPin pass,held;
 indicator::Core core;
 float last=0;
 double preparedRate=0;
public:
 VectorModulationSystem(){initializePin(input);initializePin(minV);initializePin(maxV);initializePin(rate);initializePin(smoothing);initializePin(deadband);initializePin(display);initializePin(pass);initializePin(held);}
 void onSetPins() override {
  const double sr=getSampleRate();
  if(sr!=preparedRate){core.prepare(sr);preparedRate=sr;}
  core.configure(minV,maxV,rate,smoothing,deadband,true,false,.5f,.5f);
  pass.setStreaming(true);held.setStreaming(true);
  setSleep(false);setSubProcess(&VectorModulationSystem::subProcess);
 }
 void subProcess(int n){
  if(n<=0)return;
  const auto x=getBuffer(input);auto y=getBuffer(pass);auto h=getBuffer(held);
  for(int i=0;i<n;++i){y[i]=indicator::finite(x[i],0);}
  int written=0;
  core.process(x,n,[&](int i,float v){
   for(;written<i;++written)h[written]=last;
   last=v;
   // Standard SDK parameter assignment, rather than manual event timestamps.
   display=v;
  });
  for(;written<n;++written)h[written]=last;
 }
};
namespace {auto registration=Register<VectorModulationSystem>::withId(L"Pandocrator Vector Modulation System 3");}
