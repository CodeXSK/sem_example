#include "../IndicatorCore.h"
#include "../VectorModel.h"
#include <cassert>
#include <iostream>
int main(){for(float v: {0.f,5.f,10.f,15.f,20.f}){indicator::Core c;c.prepare(48000);c.configure(0,20,20,0,.001,true,false,.5,.5);float input=v/10;float out=-1;c.process(&input,1,[&](int,float x){out=x;});assert(out==v/20);auto integrated=indicator::visual(.5,.25,out,false,true);auto external=indicator::visual(.5,.25,(out*10)*.1f,false,true);assert(integrated.value==external.value);std::cout<<v<<" V -> "<<out<<" normalized -> "<<out*10<<" V held\n";}}
