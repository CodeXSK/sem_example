#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace pandocrator {
constexpr float pi = 3.14159265358979323846f;
constexpr int kAlgorithms = 40;
constexpr int kFirTaps = 49;
constexpr float kVoltsPerFullScale = 10.0f;
inline float clip(float x, float lo, float hi) { return (std::max)(lo, (std::min)(x,hi)); }
inline float finite(float x, float alternative=0.f){ return std::isfinite(x)?x:alternative; }
inline float signedPower(float x, float p) {return std::copysign(std::pow(std::abs(x),p),x);}
inline float saturateExp(float x,float k){return std::copysign(1.f-std::exp(-k*std::abs(x)),x);}
inline float fold(float x,float t){
    t=clip(t,0.025f,3.f);
    float z=std::fmod(x+t,4.f*t);
    if(z<0)z+=4.f*t;
    return (2.f*t-std::abs(z-2.f*t))/t-1.f;
}
inline float shaper(float x,int mode,float a,float b){
    x=clip(finite(x),-24.f,24.f);
    a=clip(finite(a,1.f),-8.f,8.f);
    b=clip(finite(b,1.f),-8.f,8.f);
    const float u=std::abs(x);
    switch(mode){
    case 0: return std::tanh(x);
    case 1: return (2.f/pi)*std::atan(x);
    case 2: return std::tanh(clip(x+a*x*x,-30.f,30.f));
    case 3: {const float k=clip(a,-4.f,4.f);return std::tanh(x+k)-std::tanh(k);}
    case 4: {float z=clip(x,-1.f,1.f);return 1.5f*z-0.5f*z*z*z;}
    case 5: return x/std::sqrt(1.f+x*x);
    case 6: return saturateExp(x,1.f);
    case 7: return clip(x,-1.f,1.f);
    case 8: return x/(1.f+clip(std::abs(a),.05f,8.f)*u);
    case 9: return fold(x, std::abs(a));
    case 10:return signedPower(x,clip(std::abs(a),.2f,6.f));
    case 11:return std::sin(clip(std::abs(a),.1f,12.f)*x);
    case 12:return std::tanh(clip(x+a*x*x+b*x*x*x,-30.f,30.f));
    case 13:return x>=0?std::tanh(clip(std::abs(a),.1f,8.f)*x):std::tanh(clip(std::abs(b),.1f,8.f)*x);
    case 14:return (2.f/pi)*std::asin(clip(x,-1.f,1.f)); // arcsine
    case 15:return std::asinh(x)/std::asinh(1.f+std::abs(a)); // inverse hyperbolic sine
    case 16:return x/(1.f+clip(std::abs(a),.01f,8.f)*x*x); // rational bend/fold beyond peak
    case 17:return x*std::exp(-clip(std::abs(a),.05f,8.f)*u); // bell fold
    case 18:return std::tanh(x)*(1.f+clip(a,-1.f,2.f)*std::tanh(x*x)); // even asymmetric enhancement
    case 19: {float t=clip(std::abs(a),.05f,2.f);return x>=0?(std::min)(x,t)/t:(std::max)(x,-clip(std::abs(b),.05f,2.f))/clip(std::abs(b),.05f,2.f);}
    case 20: {float t=clip(std::abs(a),.05f,2.f);return std::copysign(u<=t?u:(2.f*t-u),x)/t;} // single reflection (not bounded)
    case 21:return std::sin(x+clip(a,-2.f,2.f)*std::sin(clip(std::abs(b),.1f,8.f)*x)); // nested sine
    case 22:return std::tanh(x+clip(a,-4.f,4.f)*std::sin(clip(std::abs(b),.1f,8.f)*x));
    case 23:return std::atan(x+clip(a,-2.f,2.f)*x*x)*(2.f/pi);
    case 24:return std::tanh(x+clip(a,-2.f,2.f)*x*x*x*x); // fourth-order asymmetry
    case 25:return std::tanh(x+clip(a,-2.f,2.f)*x*x*x*x*x); // fifth-order shaping
    case 26:return std::tanh(x)*(1.f-clip(a,0.f,1.f))+clip(a,0.f,1.f)*std::sin(x); // tanh-sine blend
    case 27:return std::copysign(std::pow(std::tanh(u),clip(std::abs(a),.2f,4.f)),x);
    case 28:return std::copysign(1.f-std::pow(1.f/(1.f+u),clip(std::abs(a),.2f,5.f)),x);
    case 29:{float t=clip(std::abs(a),.1f,2.f);return std::copysign((std::max)(0.f,u-t),x);} // dead zone
    case 30:{float t=clip(std::abs(a),.05f,2.f);return std::copysign(u<t?0.f:1.f,x);} // comparator
    case 31:{float q=clip(std::abs(a),2.f,128.f);return std::round(clip(x,-1.f,1.f)*q)/q;} // quantizer
    case 32:{float t=clip(std::abs(a),.05f,3.f);float q=std::fmod(u,t*2.f);return std::copysign(q>t?2.f*t-q:q,x)/t;} // triangle fold
    case 33:return std::sin(x)*std::cos(clip(std::abs(a),.1f,8.f)*x);
    case 34:return std::tanh(x+clip(a,-2.f,2.f)*x*x +clip(b,-2.f,2.f)*x*x*x*x);
    case 35:return std::copysign(std::sqrt(1.f-std::exp(-clip(std::abs(a),.1f,8.f)*u)),x);
    case 36:{float k=clip(std::abs(a),.1f,8.f);return std::tanh(k*x)/std::tanh(k);} // normalized tanh
    case 37:{float k=clip(std::abs(a),.1f,8.f);return std::atan(k*x)/std::atan(k);} // normalized atan
    case 38:return std::tanh(x+clip(a,-2.f,2.f)*x*x)*std::cos(clip(std::abs(b),.1f,6.f)*x);
    case 39:{float p=clip(std::abs(a),.1f,5.f);return std::copysign(std::log1p(p*u)/std::log1p(p),x);} // logarithmic
    default:return std::tanh(x);
    }
}
// A windowed-sinc, linear-phase halfband lowpass. Cutoff = Fs_high/4.
// Used in both interpolation (2x gain) and anti-aliasing decimation.
struct FirHalfband {
    std::array<float,kFirTaps> coeff{};
    std::array<float,kFirTaps> delay{};
    int write=0;
    FirHalfband(){
        constexpr float cutoff=.25f;
        float sum=0.f;
        for(int i=0;i<kFirTaps;++i){
            int n=i-(kFirTaps-1)/2;
            float sinc=n==0?2.f*cutoff:std::sin(2.f*pi*cutoff*n)/(pi*n);
            float window=.42f-.5f*std::cos(2.f*pi*i/(kFirTaps-1))+.08f*std::cos(4.f*pi*i/(kFirTaps-1));
            coeff[i]=sinc*window;
            sum+=coeff[i];
        }
        for(float& k: coeff)k/=sum;
    }
    float push(float x){
        delay[write]=x;
        float out=0.f;
        int j=write;
        for(int i=0;i<kFirTaps;++i){out+=coeff[i]*delay[j];if(--j<0)j=kFirTaps-1;}
        if(++write==kFirTaps)write=0;
        return out;
    }
    void reset(){delay.fill(0);write=0;}
};
struct Oversampler {
    std::array<FirHalfband,4> ups;
    std::array<FirHalfband,4> downs;
    int lastStages=-1;
    void reset(){for(auto& f:ups)f.reset();for(auto& f:downs)f.reset();}
    template<class Func> float process(float x,int stages,Func&& processor){
        stages=(std::max)(0,(std::min)(stages,4));
        if(stages!=lastStages){reset();lastStages=stages;}
        if(stages==0)return processor(x);
        std::array<float,16> signal{};
        signal[0]=x;
        int count=1;
        for(int stage=0;stage<stages;++stage){
            // In-place backwards expansion is safe since the destination indices
            // are always >= source indices. FIR states need chronological order,
            // so buffer into a distinct local array.
            std::array<float,16> next{};
            for(int i=0;i<count;++i){
                next[i*2]=2.f*ups[stage].push(signal[i]);
                next[i*2+1]=2.f*ups[stage].push(0.f);
            }
            count*=2;
            signal=next;
        }
        for(int i=0;i<count;++i) signal[i]=processor(signal[i]);
        for(int stage=stages-1;stage>=0;--stage){
            std::array<float,16> next{};
            for(int i=0;i<count/2;++i){
                downs[stage].push(signal[i*2]);
                next[i]=downs[stage].push(signal[i*2+1]);
            }
            count/=2;
            signal=next;
        }
        return signal[0];
    }
};
}
