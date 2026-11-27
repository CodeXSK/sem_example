#include "mp_sdk_gui.h"
#include <cmath>
class ModulationIndicatorBridgeGui : public MpGuiBase2 {
 FloatGuiPin incoming,animation;
 void update(){float x=incoming;animation=std::isfinite(x)?(x<0?0:x>1?1:x):0;}
 public:
 ModulationIndicatorBridgeGui(){initializePin(incoming,&ModulationIndicatorBridgeGui::update);initializePin(animation);}
};
namespace { auto registration = gmpi::Register<ModulationIndicatorBridgeGui>::withId(L"Pandocrator Modulation Indicator Bridge"); }
