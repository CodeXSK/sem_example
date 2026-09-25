#include "mp_sdk_gui2.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace gmpi;
using namespace gmpi_gui;
using namespace GmpiDrawing;

namespace
{
    inline float clamp01(float v)
    {
        if (!std::isfinite(v)) return 0.0f;
        return (std::max)(0.0f, (std::min)(1.0f, v));
    }

    inline float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    inline float hash01(std::uint32_t x)
    {
        x ^= x >> 16;
        x *= 0x7feb352du;
        x ^= x >> 15;
        x *= 0x846ca68bu;
        x ^= x >> 16;
        return static_cast<float>(x & 0x00ffffffu) / 16777215.0f;
    }

    struct Rgb
    {
        float r, g, b;
    };

    inline Rgb blendRgb(const Rgb& a, const Rgb& b, float t)
    {
        t = clamp01(t);
        return {
            lerp(a.r, b.r, t),
            lerp(a.g, b.g, t),
            lerp(a.b, b.b, t)
        };
    }

    inline Color asColor(const Rgb& c, float alpha = 1.0f)
    {
        return Color(clamp01(c.r), clamp01(c.g), clamp01(c.b), clamp01(alpha));
    }
}

// Cross-platform GMPI GUI. Do NOT use SeGuiCompositedGfxBase here: in the
// current SDK its implementation is legacy 32-bit-only. MpGuiGfxBase is the
// supported SynthEdit 1.5 / x64 / macOS drawing path.
class ReverbSpaceVisualizerV6Gui final : public MpGuiGfxBase
{
public:
    ReverbSpaceVisualizerV6Gui()
    {
        initializePin(pinSize,       static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinDiffusion,  static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinFeedback,   static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinDecay,      static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinLowDamp,    static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinHighDamp,   static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinPreDelay,   static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinEarlyLevel, static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinLateLevel,  static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinDensity,    static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinGlow,       static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinLineWidth,  static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinMirror,     static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinBrightness, static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinCurveLevel, static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinBgR,        static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinBgG,        static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinBgB,        static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinMainR,      static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinMainG,      static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinMainB,      static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinHiR,        static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinHiG,        static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinHiB,        static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(pinWidthPx,    static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onSizeChanged));
        initializePin(pinHeightPx,   static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV6Gui::onSizeChanged));
    }

    void onChanged()
    {
        invalidateRect();
    }

    void onSizeChanged()
    {
        // Ask SynthEdit to run layout again, then redraw.
        if (getGuiHost())
        {
            invalidateMeasure();
            invalidateRect();
        }
    }

    int32_t MP_STDCALL measure(GmpiDrawing_API::MP1_SIZE availableSize,
                               GmpiDrawing_API::MP1_SIZE* returnDesiredSize) override
    {
        (void) availableSize;
        int w = pinWidthPx.getValue();
        int h = pinHeightPx.getValue();
        w = (std::max)(160, (std::min)(2400, w));
        h = (std::max)(90,  (std::min)(1400, h));
        returnDesiredSize->width = static_cast<float>(w);
        returnDesiredSize->height = static_cast<float>(h);
        return gmpi::MP_OK;
    }

    int32_t MP_STDCALL OnRender(GmpiDrawing_API::IMpDeviceContext* drawingContext) override
    {
        Graphics g(drawingContext);
        const auto r = getRect();
        const float W = (std::max)(1.0f, r.right - r.left);
        const float H = (std::max)(1.0f, r.bottom - r.top);

        const float size       = clamp01(pinSize.getValue());
        const float diffusion  = clamp01(pinDiffusion.getValue());
        const float feedback   = clamp01(pinFeedback.getValue());
        const float decay      = clamp01(pinDecay.getValue());
        const float lowDamp    = clamp01(pinLowDamp.getValue());
        const float highDamp   = clamp01(pinHighDamp.getValue());
        const float preDelay   = clamp01(pinPreDelay.getValue());
        const float earlyLevel = clamp01(pinEarlyLevel.getValue());
        const float lateLevel  = clamp01(pinLateLevel.getValue());
        const float density    = clamp01(pinDensity.getValue());
        const float glow       = clamp01(pinGlow.getValue());
        const float lineWidth  = clamp01(pinLineWidth.getValue());
        const float mirror     = clamp01(pinMirror.getValue());
        const float brightness = clamp01(pinBrightness.getValue());
        const float curveLevel = clamp01(pinCurveLevel.getValue());

        const Rgb bg{
            clamp01(pinBgR.getValue()), clamp01(pinBgG.getValue()), clamp01(pinBgB.getValue())
        };
        const Rgb main{
            clamp01(pinMainR.getValue()), clamp01(pinMainG.getValue()), clamp01(pinMainB.getValue())
        };
        const Rgb hi{
            clamp01(pinHiR.getValue()), clamp01(pinHiG.getValue()), clamp01(pinHiB.getValue())
        };

        auto brush = g.CreateSolidColorBrush(asColor(bg));
        g.FillRectangle(Rect(0.0f, 0.0f, W, H), brush);

        const float marginX = (std::max)(6.0f, W * 0.022f);
        const float marginY = (std::max)(5.0f, H * 0.04f);
        const float left = marginX;
        const float right = W - marginX;
        const float top = marginY;
        const float bottom = H - marginY;
        const float plotW = (std::max)(1.0f, right - left);
        const float plotH = (std::max)(1.0f, bottom - top);
        const float horizon = top + plotH * (0.69f - 0.13f * mirror);
        const float upperH = (std::max)(1.0f, horizon - top);
        const float lowerH = (std::max)(1.0f, bottom - horizon);

        // subtle horizontal grid
        brush.SetColor(asColor(blendRgb(bg, main, 0.18f)));
        for (int i = 1; i < 5; ++i)
        {
            const float y = top + upperH * static_cast<float>(i) / 5.0f;
            g.DrawLine(Point(left, y), Point(right, y), brush, 1.0f);
        }

        // horizon glow + bright horizon
        brush.SetColor(asColor(blendRgb(bg, main, 0.55f)));
        g.DrawLine(Point(left, horizon), Point(right, horizon), brush, 2.0f + glow * 5.0f);
        brush.SetColor(asColor(hi));
        g.DrawLine(Point(left, horizon), Point(right, horizon), brush, 1.0f);

        const float pre = 0.015f + preDelay * 0.18f;
        const float earlyCenter = pre + 0.08f + 0.18f * size;
        const float earlyWidth = 0.055f + 0.16f * size;
        const float decayRate = lerp(8.5f, 1.15f, decay) * lerp(1.0f, 0.52f, feedback);
        const float dampCurve = lerp(1.0f, 3.6f, highDamp);
        const float lowBody = lerp(0.65f, 1.10f, 1.0f - lowDamp);
        const float feedbackGain = lerp(0.72f, 1.28f, feedback);
        int bars = static_cast<int>(lerp(36.0f, 220.0f, density));
        bars = (std::max)(36, (std::min)(220, bars));
        const float pxLine = (std::max)(1.0f, lerp(1.0f, 4.0f, lineWidth) * (W / 900.0f));

        for (int i = 0; i < bars; ++i)
        {
            const float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(bars);
            if (u < pre) continue;

            const float rndA = hash01(static_cast<std::uint32_t>(i * 9781u + 17u));
            const float rndB = hash01(static_cast<std::uint32_t>(i * 6271u + 913u));
            const float rndC = hash01(static_cast<std::uint32_t>(i * 3181u + 701u));
            const float earlyX = (u - earlyCenter) / earlyWidth;
            const float early = std::exp(-0.5f * earlyX * earlyX);
            const float tailU = clamp01((u - pre) / (1.0f - pre));
            const float tail = std::exp(-decayRate * tailU);
            const float highEnergy = std::pow((std::max)(0.0f, 1.0f - tailU), dampCurve);
            const float diffuseFloor = (0.06f + 0.24f * diffusion) * tail * lateLevel;

            float amplitude =
                earlyLevel * early * (0.40f + 0.75f * rndA) +
                lateLevel * feedbackGain * tail * (0.18f + 0.55f * rndB) +
                diffuseFloor * (0.4f + 0.6f * rndC);
            amplitude *= lowBody;
            amplitude *= lerp(0.72f, 1.0f, diffusion);
            amplitude = clamp01(amplitude);

            const float fine = clamp01(0.35f + 0.90f * highEnergy * rndC);
            const float x = left + u * plotW;
            const float y = horizon - amplitude * upperH * 0.86f;

            if (glow > 0.01f)
            {
                brush.SetColor(asColor(blendRgb(bg, main, 0.35f + 0.25f * glow)));
                g.DrawLine(Point(x, horizon), Point(x, y), brush, pxLine + 2.0f + glow * 5.0f);
            }

            brush.SetColor(asColor(blendRgb(main, hi, fine * 0.75f)));
            g.DrawLine(Point(x, horizon), Point(x, y), brush, pxLine);

            const float s = (std::max)(1.0f, pxLine + 1.0f);
            brush.SetColor(asColor(blendRgb(main, hi, 0.75f)));
            g.FillRectangle(Rect(x - s, y - s, x + s, y + s), brush);

            if (mirror > 0.01f)
            {
                const float my = (std::min)(bottom, horizon + (horizon - y) * mirror * 0.46f);
                brush.SetColor(asColor(blendRgb(bg, main, 0.20f + 0.25f * mirror)));
                g.DrawLine(Point(x, horizon + 1.0f), Point(x, my), brush, 1.0f);
            }
        }

        // smooth envelope curve
        const int segments = (std::max)(48, (std::min)(320, static_cast<int>(plotW / 3.0f)));
        auto curveY = [&](float u)
        {
            if (u < pre) return horizon;
            const float earlyX = (u - earlyCenter) / (earlyWidth * 1.28f);
            const float early = std::exp(-0.5f * earlyX * earlyX);
            const float tu = clamp01((u - pre) / (1.0f - pre));
            const float tail = std::exp(-decayRate * 0.72f * tu);
            const float env = curveLevel * clamp01(0.72f * earlyLevel * early + 0.48f * lateLevel * feedbackGain * tail);
            return horizon - env * upperH * 0.72f;
        };

        if (glow > 0.01f)
        {
            brush.SetColor(asColor(blendRgb(bg, hi, 0.42f)));
            for (int i = 1; i <= segments; ++i)
            {
                const float u0 = static_cast<float>(i - 1) / static_cast<float>(segments);
                const float u1 = static_cast<float>(i) / static_cast<float>(segments);
                g.DrawLine(Point(left + u0 * plotW, curveY(u0)),
                           Point(left + u1 * plotW, curveY(u1)),
                           brush, 3.0f + glow * 7.0f);
            }
        }

        brush.SetColor(asColor(hi));
        for (int i = 1; i <= segments; ++i)
        {
            const float u0 = static_cast<float>(i - 1) / static_cast<float>(segments);
            const float u1 = static_cast<float>(i) / static_cast<float>(segments);
            g.DrawLine(Point(left + u0 * plotW, curveY(u0)),
                       Point(left + u1 * plotW, curveY(u1)),
                       brush, pxLine);
        }

        // water/reflection ripples
        if (mirror > 0.01f && lowerH > 4.0f)
        {
            brush.SetColor(asColor(blendRgb(bg, main, 0.20f + mirror * 0.16f)));
            for (int i = 1; i <= 11; ++i)
            {
                const float t = static_cast<float>(i) / 12.0f;
                const float y = horizon + t * lowerH;
                const float inset = plotW * t * 0.08f;
                g.DrawLine(Point(left + inset, y), Point(right - inset, y), brush, 1.0f);
            }
        }

        // frame
        brush.SetColor(asColor(blendRgb(bg, main, 0.45f)));
        g.DrawRectangle(Rect(0.5f, 0.5f, (std::max)(0.5f, W - 0.5f), (std::max)(0.5f, H - 0.5f)), brush, 1.0f);

        return gmpi::MP_OK;
    }

private:
    FloatGuiPin pinSize, pinDiffusion, pinFeedback, pinDecay, pinLowDamp, pinHighDamp;
    FloatGuiPin pinPreDelay, pinEarlyLevel, pinLateLevel, pinDensity, pinGlow;
    FloatGuiPin pinLineWidth, pinMirror, pinBrightness, pinCurveLevel;
    FloatGuiPin pinBgR, pinBgG, pinBgB;
    FloatGuiPin pinMainR, pinMainG, pinMainB;
    FloatGuiPin pinHiR, pinHiG, pinHiB;
    IntGuiPin pinWidthPx, pinHeightPx;
};

GMPI_REGISTER_GUI(MP_SUB_TYPE_GUI2, ReverbSpaceVisualizerV6Gui, L"Pandocrator Reverb Space Visualizer v6");
