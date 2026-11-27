#include "mp_sdk_audio.h"
#include "IndicatorCore.h"
using namespace gmpi;
class ModulationIndicatorBridge : public MpBase2 {
 AudioInPin input;
 FloatInPin minV,maxV,rate,smoothing,deadband;
 BoolInPin enable,offsetMode;
 FloatInPin base,depth;
 FloatOutPin display, debug;
 FloatInPin outputMin,outputMax;
 indicator::Core core;
 void configure(){core.configure(minV,maxV,rate,smoothing,deadband,enable,offsetMode,base,depth); if(outputMin.isUpdated()||outputMax.isUpdated())core.sent=false;}
 public:
 ModulationIndicatorBridge(){initializePin(input);initializePin(minV);initializePin(maxV);initializePin(rate);initializePin(smoothing);initializePin(deadband);initializePin(enable);initializePin(offsetMode);initializePin(base);initializePin(depth);initializePin(display);initializePin(outputMin);initializePin(outputMax);initializePin(debug);}
 void onGraphStart() override {core.prepare(getSampleRate());configure();setSubProcess(&ModulationIndicatorBridge::subProcess);setSleep(false);}
 void onSetPins() override {configure();setSubProcess(&ModulationIndicatorBridge::subProcess);setSleep(false);}
 void subProcess(int n){core.process(getBuffer(input),n,[this](int i,float x){const float lo=indicator::clamp(indicator::finite(outputMin,0),-10000,10000); const float hi=indicator::clamp(indicator::finite(outputMax,10),-10000,10000); const float v=lo+(hi-lo)*x; display.setValue(v,getBlockPosition()+i); debug.setValue(v,getBlockPosition()+i);});}
};
namespace {auto registration=Register<ModulationIndicatorBridge>::withId(L"Pandocrator Modulation Indicator Bridge Diagnostic 1.1");}
