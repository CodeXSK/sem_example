#pragma once
#include <cmath>
#include <algorithm>
namespace indicator {
inline float finite(float x,float fallback){return std::isfinite(x)?x:fallback;}
inline float clamp(float x,float lo,float hi){return (std::max)(lo,(std::min)(hi,x));}
struct Core {
 double sr=48000; int period=1600, remaining=0; float lo=-5,hi=5,base=.5f,depth=.5f,dead=.001f,alpha=1,filtered=0,last=0; bool offset=false, enabled=true, initialized=false, sent=false;
 void prepare(double rate){sr=std::isfinite(rate)&&rate>=1000&&rate<=10000000?rate:48000; remaining=0; initialized=sent=false;}
 void configure(float minV,float maxV,float hz,float smoothMs,float db,bool enable,bool offsetMode,float b,float d){
  lo=clamp(finite(minV,-5),-10000,10000);hi=clamp(finite(maxV,5),-10000,10000);
  const int p=static_cast<int>(std::ceil(sr/clamp(finite(hz,30),1,60)));
  if(p>period && sent) remaining=(std::max)(remaining,p);
  period=p;
  smoothMs=clamp(finite(smoothMs,0),0,1000); alpha=smoothMs==0?1:static_cast<float>(-std::expm1(-1000.0*period/(sr*smoothMs)));
  dead=clamp(finite(db,.001f),0,1);enabled=enable;offset=offsetMode;base=clamp(finite(b,.5f),0,1);depth=clamp(finite(d,.5f),-1,1);
 }
 float target(float raw) const { if(!enabled)return offset?base:0; const double span=double(hi)-lo; const float u=std::abs(span)<1e-8?0:static_cast<float>((std::max)(0.0,(std::min)(1.0,(double(finite(raw,0))*10-lo)/span))); return offset?clamp(base+(2*u-1)*depth,0,1):u; }
 template<class Emit> void process(const float* input,int n,Emit emit){if(!input||n<=0)return; int i=remaining; while(i<n){float t=target(input[i]); if(!initialized||!enabled){filtered=t;initialized=true;} else {filtered+=alpha*(t-filtered);if(std::abs(t-filtered)<1e-6f)filtered=t;} if(!sent|| (filtered!=last && (std::abs(filtered-last)>=dead||filtered==0||filtered==1||!enabled))){last=filtered;sent=true;emit(i,last);} if(n-i<=period){remaining=period-(n-i);return;}i+=period;}remaining=i-n;}
};
}
