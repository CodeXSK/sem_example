#pragma once
#include "IndicatorCore.h"
namespace indicator {
struct Visual {
 float base, value, low, high;
};
inline Visual visual(float base,float depth,float normalized,bool offset,bool enabled){
 base=clamp(finite(base,.5f),0,1); depth=clamp(finite(depth,.25f),-1,1); normalized=clamp(finite(normalized,.5f),0,1);
 float d=std::abs(depth); return {base,enabled?(offset?clamp(base+(2*normalized-1)*depth,0,1):normalized):base,offset?clamp(base-d,0,1):0,offset?clamp(base+d,0,1):1};
}
struct Layout {float radius,width,face,scale;};
inline Layout layout(float w,float h,float size,float radius,float thickness,float face,float glow){
 float s=clamp(finite(size,1),.05f,1);float half=(std::min)(finite(w,0),finite(h,0))*.5f*s;
 if(half<2)return {0,0,0,0};
 float line=clamp(finite(thickness,.06f),.005f,.35f)*half;
 float r=clamp(finite(radius,.86f),.1f,1)*half;
 r=(std::min)(r,half-line*(.5f+clamp(finite(glow,0),0,1))-1);
 if(r<=0)return {0,0,0,0};
 float f=(std::min)(r*clamp(finite(face,.73f),.05f,.98f),(std::max)(0.f,r-line*1.5f));
 return {r,line,f,half};
}
}
