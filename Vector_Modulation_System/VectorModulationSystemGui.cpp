#include "mp_sdk_gui2.h"
#include "VectorModel.h"
#include <cstdint>
#include <string>
#include <cstdio>
using namespace GmpiDrawing;
using namespace gmpi_gui;
template<bool External> class VectorModulationGui : public MpGuiGfxBase {
 FloatGuiPin incoming,base,depth,scale,radius,thickness,faceSize,start,end,glow,dragPixels;
 BoolGuiPin offset,enabled,showKnob,showRange,showTrack,showTicks,showDot,showValue,interactive;
 StringGuiPin activeColor,rangeColor,trackColor,faceTop,faceBottom,needleColor,backgroundColor;
 FloatGuiPin animation,baseOut;
 BoolGuiPin draggingOut;
 float dragY=0,dragBase=0;
 indicator::Visual previous{}; bool hasPrevious=false;
 static Color parse(const std::wstring& s,uint32_t fallback){
  size_t i=0;if(!s.empty()&&s[0]==L'#')i=1;auto n=s.size()-i;if(n!=6&&n!=8)return Color::FromArgb(fallback);
  uint32_t v=0;for(;i<s.size();++i){wchar_t c=s[i];unsigned d;if(c>=L'0'&&c<=L'9')d=c-L'0';else if(c>=L'a'&&c<=L'f')d=c-L'a'+10;else if(c>=L'A'&&c<=L'F')d=c-L'A'+10;else return Color::FromArgb(fallback);v=(v<<4)|d;}
  if(n==6)v|=0xff000000u;return Color::FromArgb(v);
 }
 indicator::Visual model(){return indicator::visual(base,depth,External ? float(incoming)*0.1f : float(incoming),offset,enabled);}
 void refresh(){auto m=model();bool changed=!hasPrevious||m.base!=previous.base||m.value!=previous.value||m.low!=previous.low||m.high!=previous.high;previous=m;hasPrevious=true;animation=m.value;baseOut=m.base;if(changed&&getGuiHost())invalidateRect();}
 void modeChanged(){refresh();appearance();}
 void appearance(){if(getGuiHost())invalidateRect();}
 public:
 VectorModulationGui(){
  initializePin(incoming,&VectorModulationGui::refresh);initializePin(base,&VectorModulationGui::refresh);initializePin(depth,&VectorModulationGui::refresh);initializePin(offset,&VectorModulationGui::modeChanged);initializePin(enabled,&VectorModulationGui::modeChanged);
  initializePin(scale,&VectorModulationGui::appearance);initializePin(radius,&VectorModulationGui::appearance);initializePin(thickness,&VectorModulationGui::appearance);initializePin(faceSize,&VectorModulationGui::appearance);initializePin(start,&VectorModulationGui::appearance);initializePin(end,&VectorModulationGui::appearance);initializePin(glow,&VectorModulationGui::appearance);
  initializePin(showKnob,&VectorModulationGui::appearance);initializePin(showRange,&VectorModulationGui::appearance);initializePin(showTrack,&VectorModulationGui::appearance);initializePin(showTicks,&VectorModulationGui::appearance);initializePin(showDot,&VectorModulationGui::appearance);initializePin(showValue,&VectorModulationGui::appearance);initializePin(interactive);initializePin(dragPixels);
  initializePin(activeColor,&VectorModulationGui::appearance);initializePin(rangeColor,&VectorModulationGui::appearance);initializePin(trackColor,&VectorModulationGui::appearance);initializePin(faceTop,&VectorModulationGui::appearance);initializePin(faceBottom,&VectorModulationGui::appearance);initializePin(needleColor,&VectorModulationGui::appearance);initializePin(backgroundColor,&VectorModulationGui::appearance);
  initializePin(animation);initializePin(baseOut);initializePin(draggingOut);
 }
 int32_t MP_STDCALL OnRender(GmpiDrawing_API::IMpDeviceContext* dc) override {
  Graphics g(dc);auto rect=getRect();float w=rect.right-rect.left,h=rect.bottom-rect.top;
  auto l=indicator::layout(w,h,scale,radius,thickness,faceSize,glow);if(l.radius<=0)return gmpi::MP_OK;
  Point center(rect.left+w*.5f,rect.top+h*.5f);auto m=model();
  float a=indicator::clamp(indicator::finite(start,135),-720,720),b=indicator::clamp(indicator::finite(end,405),-720,720);float sweep=indicator::clamp(b-a,-360,360);
  auto point=[&](float v,float r){float angle=(a+v*sweep)*.01745329251994329577f;return Point(center.x+r*std::cos(angle),center.y+r*std::sin(angle));};
  auto bg=parse(backgroundColor.getValue(),0);auto brush=g.CreateSolidColorBrush(bg);if(bg.a>0)g.FillRectangle(rect,brush);
  auto arc=[&](float x,float y,float width,Color color){if(std::abs(x-y)*std::abs(sweep)<.001f)return;auto path=g.GetFactory().CreatePathGeometry();auto sink=path.Open();int steps=(std::max)(1,static_cast<int>(std::ceil(std::abs(y-x)*std::abs(sweep)/3.f)));sink.BeginFigure(point(x,l.radius));for(int i=1;i<=steps;++i)sink.AddLine(point(x+(y-x)*i/steps,l.radius));sink.EndFigure(FigureEnd::Open);sink.Close();brush.SetColor(color);g.DrawGeometry(path,brush,width);g.FillCircle(point(x,l.radius),width*.5f,brush);g.FillCircle(point(y,l.radius),width*.5f,brush);};
  auto active=parse(activeColor.getValue(),0xff42e8ad);auto needle=parse(needleColor.getValue(),0xffeeeeee);auto track=parse(trackColor.getValue(),0xff3b434e);
  if(showTrack)arc(0,1,l.width,track);
  if(enabled&&showRange)arc(m.low,m.high,l.width,parse(rangeColor.getValue(),0xff1b5649));
  if(enabled){float from=offset?m.base:0.f;float strength=indicator::clamp(indicator::finite(glow,0),0,1);if(strength>0){auto c=active;c.a*=.18f*strength;arc(from,m.value,l.width*(1+2*strength),c);}arc(from,m.value,l.width,active);if(showDot){brush.SetColor(active);g.FillCircle(point(m.value,l.radius),l.width*.65f,brush);}}
  if(showTicks && l.radius>l.width*2){brush.SetColor(track);for(int i=0;i<=30;++i)g.DrawLine(point(i/30.f,l.radius-l.width*1.8f),point(i/30.f,l.radius-l.width*1.15f),brush,(std::max)(.5f,l.scale*.009f));}
  if(showKnob&&l.face>1){
   auto top=parse(faceTop.getValue(),0xff59616b),bottom=parse(faceBottom.getValue(),0xff20252c);
   auto gradient=g.CreateLinearGradientBrush(Point(center.x,center.y-l.face),Point(center.x,center.y+l.face),top,bottom);g.FillCircle(center,l.face,gradient);
   brush.SetColor(track);g.DrawCircle(center,l.face,brush,(std::max)(.5f,l.scale*.01f));g.DrawCircle(center,l.face*.81f,brush,(std::max)(.5f,l.scale*.008f));
   brush.SetColor(needle);g.DrawLine(point(m.base,l.face*.66f),point(m.base,l.face*.91f),brush,(std::max)(1.f,l.scale*.035f));
   if(showValue&&l.face>=12){char text[20];std::snprintf(text,sizeof(text),"%.0f%%",m.base*100);auto fmt=g.GetFactory().CreateTextFormat(l.face*.32f,"Arial");fmt.SetTextAlignment(TextAlignment::Center);fmt.SetParagraphAlignment(ParagraphAlignment::Center);g.DrawTextU(std::string(text),fmt,Rect(center.x-l.face*.8f,center.y-l.face*.4f,center.x+l.face*.8f,center.y+l.face*.4f),brush);}
  }
  return gmpi::MP_OK;
 }
 int32_t MP_STDCALL hitTest(GmpiDrawing_API::MP1_POINT) override {return showKnob&&interactive?gmpi::MP_OK:gmpi::MP_UNHANDLED;}
 int32_t MP_STDCALL onPointerDown(int32_t flags,GmpiDrawing_API::MP1_POINT p) override {if(!showKnob||!interactive||!(flags & gmpi_gui_api::GG_POINTER_FLAG_FIRSTBUTTON))return gmpi::MP_UNHANDLED;dragY=p.y;dragBase=model().base;setCapture();draggingOut=true;return gmpi::MP_OK;}
 int32_t MP_STDCALL onPointerMove(int32_t flags,GmpiDrawing_API::MP1_POINT p) override {if(!getCapture())return gmpi::MP_UNHANDLED;float speed=indicator::clamp(indicator::finite(dragPixels,150),10,2000);if(flags & gmpi_gui_api::GG_POINTER_KEY_SHIFT)speed*=10;float v=indicator::clamp(dragBase+(dragY-p.y)/speed,0,1);if(v!=model().base){base=v;refresh();}return gmpi::MP_OK;}
 int32_t MP_STDCALL onPointerUp(int32_t,GmpiDrawing_API::MP1_POINT) override {if(!getCapture())return gmpi::MP_UNHANDLED;releaseCapture();draggingOut=false;return gmpi::MP_OK;}
};
namespace {auto vectorGuiRegistration=gmpi::Register<VectorModulationGui<false>>::withId(L"Pandocrator Vector Modulation System 3");auto standaloneGuiRegistration=gmpi::Register<VectorModulationGui<true>>::withId(L"Pandocrator Vector Modulation Display 3");}
