#include "mp_sdk_audio.h"
#include "IndicatorCore.h"
using namespace gmpi;
class ModulationIndicatorBridge : public MpBase2 {
 AudioInPin input;
 FloatInPin minV,maxV,rate,smoothing,deadband;
 BoolInPin enable,offsetMode;
 FloatInPin base,depth;
 FloatOutPin display;
 indicator::Core core;
 void configure(){core.configure(minV,maxV,rate,smoothing,deadband,enable,offsetMode,base,depth);}
 public:
 ModulationIndicatorBridge(){initializePin(input);initializePin(minV);initializePin(maxV);initializePin(rate);initializePin(smoothing);initializePin(deadband);initializePin(enable);initializePin(offsetMode);initializePin(base);initializePin(depth);initializePin(display);}
 void onGraphStart() override {core.prepare(getSampleRate());configure();setSubProcess(&ModulationIndicatorBridge::subProcess);setSleep(false);}
 void onSetPins() override {configure();setSubProcess(&ModulationIndicatorBridge::subProcess);setSleep(false);}
 void subProcess(int n){core.process(getBuffer(input),n,[this](int i,float x){display.setValue(x,getBlockPosition()+i);});}
};
namespace {auto registration=Register<ModulationIndicatorBridge>::withId(L"Pandocrator Modulation Indicator Bridge");}
