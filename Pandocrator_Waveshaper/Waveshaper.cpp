#include "mp_sdk_audio.h"
#include <algorithm>
#include <cmath>
#include <limits>
using namespace gmpi;
namespace {
constexpr float pi=3.14159265358979323846f;
inline float clamp(float x,float lo,float hi){return (std::max)(lo,(std::min)(hi,x));}
inline float safe(float v,float fallback=0.0f){return std::isfinite(v)?v:fallback;}
inline float signedPow(float x,float p){return std::copysign(std::pow(std::abs(x),p),x);}
// All kernels use normalized drive-domain input (v), parameters A/B, with individually useful interpretations.
inline float shape(float v,int mode,float a,float b){
    v=clamp(v,-32.0f,32.0f);
    switch(mode){
    case 0: return std::tanh(v);                                  // symmetric tanh
    case 1: return (2.0f/pi)*std::atan(v);                       // arctan
    case 2: return std::tanh(clamp(v+a*v*v,-30.0f,30.0f));       // asymmetric tube; A controls x^2
    case 3: { // biased tanh, centered at zero input; A=bias
        float bias=clamp(a,-4.0f,4.0f);
        return std::tanh(v+bias)-std::tanh(bias);
    }
    case 4: { // cubic clip, bounded at +-1 to prevent foldback outside interval
        float z=clamp(v,-1.0f,1.0f); return 1.5f*z-0.5f*z*z*z;
    }
    case 5: return v/std::sqrt(1.0f+v*v);                       // algebraic soft clip
    case 6: return std::copysign(1.0f-std::exp(-std::abs(v)),v); // exponential soft clip
    case 7: return clamp(v,-1.0f,1.0f);                          // hard clip
    case 8: { // rational soft clip, A governs curvature: A >= 0
        float k=clamp(std::abs(a),0.05f,8.0f);
        return v/(1.0f+k*std::abs(v));
    }
    case 9: { // wavefold, A is threshold (0.05..2), B output factor (optional)
        float t=clamp(std::abs(a),0.05f,2.0f);
        float z=std::fmod(v+t,4.0f*t);
        if(z<0.0f) z+=4.0f*t;
        return (2.0f*t-std::abs(z-2.0f*t))/t-1.0f;
    }
    case 10: { // sign-preserving power, A exponent > 0
        return signedPow(v,clamp(std::abs(a),0.2f,6.0f));
    }
    case 11: { // sine fold, A sets frequency multiplier
        float k=clamp(std::abs(a),0.1f,12.0f);
        return std::sin(k*v);
    }
    case 12: { // polynomial mix: v + A*v^2 + B*v^3, softly bounded
        float z=v + clamp(a,-2.0f,2.0f)*v*v + clamp(b,-2.0f,2.0f)*v*v*v;
        return std::tanh(clamp(z,-30.0f,30.0f));
    }
    case 13: { // generalized asymmetric sigmoid: separate + / - hardness
        float pos=clamp(std::abs(a),0.1f,8.0f), neg=clamp(std::abs(b),0.1f,8.0f);
        return v>=0 ? std::tanh(pos*v) : std::tanh(neg*v);
    }
    default:return std::tanh(v);
    }
}
}
class Waveshaper final : public MpBase2 {
public:
    Waveshaper(){
        initializePin(in_); initializePin(mode_); initializePin(drive_); initializePin(a_);
        initializePin(b_); initializePin(bias_); initializePin(wet_); initializePin(output_);
        initializePin(dcBlock_); initializePin(wetOut_); initializePin(mixOut_);
    }
    void onSetPins() override { setSubProcess(&Waveshaper::process); }
    void process(int sampleFrames){
        auto in=getBuffer(in_), mode=getBuffer(mode_), drive=getBuffer(drive_);
        auto a=getBuffer(a_), b=getBuffer(b_), bias=getBuffer(bias_);
        auto wet=getBuffer(wet_), output=getBuffer(output_), dc=getBuffer(dcBlock_);
        auto wo=getBuffer(wetOut_), mo=getBuffer(mixOut_);
        const float sr=clamp(safe(getSampleRate(),44100.f),8000.f,384000.f);
        // 5 Hz one-pole high pass for optional DC blocking
        const float r=std::exp(-2.0f*pi*5.0f/sr);
        // Audio pins allow patchable parameters. Control values are evaluated per sample.
        for(int i=0;i<sampleFrames;++i){
            float x=clamp(safe(in[i]),-16.f,16.f);
            int m=static_cast<int>(std::round(clamp(safe(mode[i]),0.f,13.f)));
            float d=clamp(safe(drive[i],1.0f),0.f,32.f);
            float aa=clamp(safe(a[i],1.0f),-8.f,8.f);
            float bb=clamp(safe(b[i],1.0f),-8.f,8.f);
            float biasVal=clamp(safe(bias[i]),-4.f,4.f);
            float mix=clamp(safe(wet[i],1.0f),0.f,1.f);
            float gain=clamp(safe(output[i],1.0f),0.f,8.f);
            float biased=clamp(d*x+biasVal,-32.f,32.f);
            // Remove static operating-point offset due solely to Bias parameter.
            float s=shape(biased,m,aa,bb)-shape(biasVal,m,aa,bb);
            s=clamp(s,-32.f,32.f);
            // Keep HP filter state running, even when bypassed, to prevent stale-state transients.
            float high=r*(prevHigh_+s-prevInput_);
            prevInput_=s; prevHigh_=std::abs(high)<1.e-20f?0.0f:high;
            float processed=(dc[i]>=0.5f?high:s);
            float wetSample=clamp(processed*gain,-32.f,32.f);
            wo[i]=wetSample;
            mo[i]=clamp(((1.f-mix)*x+mix*processed)*gain,-32.f,32.f);
        }
    }
private:
    AudioInPin in_,mode_,drive_,a_,b_,bias_,wet_,output_,dcBlock_;
    AudioOutPin wetOut_,mixOut_;
    float prevInput_=0.f,prevHigh_=0.f;
};
namespace { auto registration=Register<Waveshaper>::withId(L"Pandocrator Waveshaper Toolkit"); }
