#include "../IndicatorCore.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){indicator::Core c;c.prepare(48000);c.configure(0,20,30,0,.001,true,false,.5,.5);for(int v=0;v<=20;++v){float x=c.target(v/10.f)*10;assert(std::abs(x-v/2.f)<1e-5);}std::cout<<"0..20 V -> 0..10 passed\n";}
