#include "../VectorModel.h"
#include <cassert>
#include <limits>
#include <random>
#include <iostream>
int main(){using namespace indicator;auto m=visual(.5,.25,1,true,true);assert(m.value==.75f&&m.low==.25f&&m.high==.75f);assert(visual(.5,-.25,1,true,true).value==.25f);assert(visual(.9,.9,1,true,true).value==1.f);assert(visual(.3,.4,.7,false,true).value==.7f);assert(visual(.3,.4,.7,false,false).value==.3f);std::mt19937 gen(21);std::uniform_real_distribution<float>d(-3,3);for(int i=0;i<100000;++i){auto m=visual(d(gen),d(gen),d(gen),i%2,i%3);assert(m.base>=0&&m.base<=1&&m.value>=0&&m.value<=1&&m.low<=m.high);auto l=layout(200*d(gen),200*d(gen),d(gen),d(gen),d(gen),d(gen),d(gen));assert(l.radius>=0&&l.face>=0&&l.face<=l.radius);if(l.radius>0)assert(l.radius+l.width*.5f<=l.scale);}float nan=std::numeric_limits<float>::quiet_NaN();auto n=visual(nan,nan,nan,true,true);assert(n.value==.5f);assert(layout(nan,50,1,1,1,1,1).radius==0);std::cout<<"Visual model: 100000 randomized cases + edge cases passed\n";}
