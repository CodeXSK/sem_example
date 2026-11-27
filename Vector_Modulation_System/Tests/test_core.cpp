#include "../IndicatorCore.h"
#include <cassert>
#include <vector>
#include <iostream>
#include <limits>
int main(){using indicator::Core; for(double sr:{8000.,44100.,48000.,96000.,192000.,768000.})for(float hz:{1.f,15.f,30.f,60.f}){Core c;c.prepare(sr);c.configure(-5,5,hz,0,0,true,false,.5,.5);assert(c.target(-.5)==0&&c.target(.5)==1&&c.target(0)==.5);std::vector<float>x(static_cast<int>(sr*2));for(size_t i=0;i<x.size();++i)x[i]=.5f*std::sin(i*.001);int prev=-1,count=0;for(int pos=0;pos<(int)x.size();){int n=std::min(1+pos%511,(int)x.size()-pos);c.process(x.data()+pos,n,[&](int i,float v){assert(i>=0&&i<n&&v>=0&&v<=1&&std::isfinite(v));if(prev>=0)assert(pos+i-prev>=c.period);prev=pos+i;++count;});pos+=n;}assert(count<=2*hz+1);std::cout<<sr<<" Hz / "<<hz<<" fps: "<<count<<" events\n";}
Core c;c.prepare(48000);c.configure(-5,5,30,0,.001,true,true,.5,.25);assert(c.target(-.5)==.25&&c.target(.5)==.75);assert(c.target(std::numeric_limits<float>::quiet_NaN())==.5);assert(c.target(std::numeric_limits<float>::infinity())==.5);std::vector<float>x(48000,.5);int count=0;c.process(x.data(),x.size(),[&](int,float){++count;});assert(count==1);c.configure(5,5,30,0,0,true,false,.5,.5);assert(c.target(.5)==0);c.configure(5,-5,30,0,0,true,false,.5,.5);assert(c.target(.5)==0&&c.target(-.5)==1);c.configure(-5,5,30,0,0,false,true,.4,.5);assert(c.target(.5)==.4f);std::cout<<"All tests passed\n";}
