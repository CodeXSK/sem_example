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
        return (std::max)(0.0f, (std::min)(1.0f, v));
    }

    inline float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    inline float smoothstep(float a, float b, float x)
    {
        if (a == b)
            return x < a ? 0.0f : 1.0f;
        const float t = clamp01((x - a) / (b - a));
        return t * t * (3.0f - 2.0f * t);
    }

    // Deterministic 0..1 hash. Stable while controls move: no visual "fizz".
    inline float hash01(std::uint32_t x)
    {
        x ^= x >> 16;
        x *= 0x7feb352du;
        x ^= x >> 15;
        x *= 0x846ca68bu;
        x ^= x >> 16;
        return static_cast<float>(x & 0x00ffffffu) / 16777215.0f;
    }

    inline Color makeColor(float r, float g, float b, float a = 1.0f)
    {
        return Color(clamp01(r), clamp01(g), clamp01(b), clamp01(a));
    }
}

class ReverbSpaceVisualizerV3Gui final : public MpGuiGfxBase
{
public:
    ReverbSpaceVisualizerV3Gui()
    {
        initializePin(pinSize,        static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinDiffusion,   static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinDecay,       static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinLowDamp,     static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinHighDamp,    static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinPreDelay,    static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinEarlyLevel,  static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinLateLevel,   static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinDensity,     static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinGlow,        static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinLineWidth,   static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinMirror,      static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinBrightness,  static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinCurveLevel,  static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));

        initializePin(pinBgR,         static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinBgG,         static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinBgB,         static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinMainR,       static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinMainG,       static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinMainB,       static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinHiR,         static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinHiG,         static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinHiB,         static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));

        initializePin(pinWidthPx,     static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
        initializePin(pinHeightPx,    static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceVisualizerV3Gui::refresh));
    }

    void refresh()
    {
        invalidateRect();
    }

    int32_t MP_STDCALL measure(
        GmpiDrawing_API::MP1_SIZE availableSize,
        GmpiDrawing_API::MP1_SIZE* returnDesiredSize) override
    {
        (void) availableSize;

        int w = pinWidthPx.getValue();
        int h = pinHeightPx.getValue();

        w = (std::max)(160, (std::min)(2400, w));
        h = (std::max)(90,  (std::min)(1400, h));

        returnDesiredSize->width = static_cast<float>(w);
        returnDesiredSize->height = static_cast<float>(h);
        return MP_OK;
    }

    int32_t MP_STDCALL OnRender(GmpiDrawing_API::IMpDeviceContext* drawingContext) override
    {
        Graphics g(drawingContext);
        const auto rect = getRect();

        const float W = (std::max)(1.0f, rect.right - rect.left);
        const float H = (std::max)(1.0f, rect.bottom - rect.top);

        const float size       = clamp01(pinSize.getValue() * 10.0f);
        const float diffusion  = clamp01(pinDiffusion.getValue() * 10.0f);
        const float decay      = clamp01(pinDecay.getValue() * 10.0f);
        const float lowDamp    = clamp01(pinLowDamp.getValue() * 10.0f);
        const float highDamp   = clamp01(pinHighDamp.getValue() * 10.0f);
        const float preDelay   = clamp01(pinPreDelay.getValue() * 10.0f);
        const float earlyLevel = clamp01(pinEarlyLevel.getValue() * 10.0f);
        const float lateLevel  = clamp01(pinLateLevel.getValue() * 10.0f);
        const float density    = clamp01(pinDensity.getValue() * 10.0f);
        const float glow       = clamp01(pinGlow.getValue() * 10.0f);
        const float lineWidth  = clamp01(pinLineWidth.getValue() * 10.0f);
        const float mirror     = clamp01(pinMirror.getValue() * 10.0f);
        const float brightness = clamp01(pinBrightness.getValue() * 10.0f);
        const float curveLevel = clamp01(pinCurveLevel.getValue() * 10.0f);

        const float bgR = clamp01(pinBgR.getValue() * 10.0f);
        const float bgG = clamp01(pinBgG.getValue() * 10.0f);
        const float bgB = clamp01(pinBgB.getValue() * 10.0f);
        const float mainR = clamp01(pinMainR.getValue() * 10.0f);
        const float mainG = clamp01(pinMainG.getValue() * 10.0f);
        const float mainB = clamp01(pinMainB.getValue() * 10.0f);
        const float hiR = clamp01(pinHiR.getValue() * 10.0f);
        const float hiG = clamp01(pinHiG.getValue() * 10.0f);
        const float hiB = clamp01(pinHiB.getValue() * 10.0f);

        auto brush = g.CreateSolidColorBrush(makeColor(bgR, bgG, bgB, 1.0f));
        g.FillRectangle(rect, brush);

        // Inner plotting area. Everything is proportional so arbitrary host sizes work.
        const float marginX = (std::max)(4.0f, W * 0.018f);
        const float marginY = (std::max)(4.0f, H * 0.035f);
        const float left = rect.left + marginX;
        const float right = rect.right - marginX;
        const float top = rect.top + marginY;
        const float bottom = rect.bottom - marginY;
        const float plotW = (std::max)(1.0f, right - left);
        const float plotH = (std::max)(1.0f, bottom - top);

        // The horizon leaves room for the water/mirror image underneath.
        const float horizon = top + plotH * (0.69f - 0.13f * mirror);
        const float upperH = (std::max)(1.0f, horizon - top);
        const float lowerH = (std::max)(1.0f, bottom - horizon);

        // Very subtle background horizontal layers.
        brush.SetColor(makeColor(mainR, mainG, mainB, 0.035f + glow * 0.045f));
        const int gridLines = 5;
        for (int i = 1; i < gridLines; ++i)
        {
            const float y = top + upperH * static_cast<float>(i) / static_cast<float>(gridLines);
            g.FillRectangle(left, y, right, y + 1.0f, brush);
        }

        // Horizon glow.
        const float horizonGlow = 1.0f + glow * (std::max)(2.0f, H * 0.028f);
        brush.SetColor(makeColor(mainR, mainG, mainB, 0.10f + 0.24f * glow));
        g.FillRectangle(left, horizon - horizonGlow, right, horizon + horizonGlow, brush);
        brush.SetColor(makeColor(hiR, hiG, hiB, 0.40f + 0.45f * brightness));
        g.FillRectangle(left, horizon - 0.5f, right, horizon + 0.75f, brush);

        // Visual model.
        // Size moves/widens the early-reflection body.
        // Diffusion controls density and irregularity.
        // Decay controls the tail envelope.
        const float pre = 0.015f + preDelay * 0.18f;
        const float earlyCenter = pre + 0.08f + 0.18f * size;
        const float earlyWidth = 0.055f + 0.16f * size;
        const float decayRate = lerp(8.5f, 1.15f, decay);
        const float dampCurve = lerp(1.0f, 3.6f, highDamp);
        const float lowBody = lerp(0.65f, 1.10f, 1.0f - lowDamp);

        const int minBars = 36;
        const int maxBars = 220;
        int bars = static_cast<int>(lerp(static_cast<float>(minBars), static_cast<float>(maxBars), density));
        bars = (std::max)(minBars, (std::min)(maxBars, bars));

        const float pxLine = (std::max)(0.65f, lerp(0.75f, 4.0f, lineWidth) * (W / 900.0f));

        for (int i = 0; i < bars; ++i)
        {
            const float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(bars);
            if (u < pre)
                continue;

            const float rndA = hash01(static_cast<std::uint32_t>(i * 9781u + 17u));
            const float rndB = hash01(static_cast<std::uint32_t>(i * 6271u + 913u));
            const float rndC = hash01(static_cast<std::uint32_t>(i * 3181u + 701u));

            const float earlyX = (u - earlyCenter) / earlyWidth;
            const float early = std::exp(-0.5f * earlyX * earlyX);

            const float tailU = clamp01((u - pre) / (1.0f - pre));
            const float tail = std::exp(-decayRate * tailU);

            // High damping makes the fine/bright upper energy disappear faster.
            const float highEnergy = std::pow((std::max)(0.0f, 1.0f - tailU), dampCurve);
            const float diffuseFloor = (0.06f + 0.24f * diffusion) * tail * lateLevel;

            float amplitude =
                earlyLevel * early * (0.40f + 0.75f * rndA) +
                lateLevel * tail * (0.18f + 0.55f * rndB) +
                diffuseFloor * (0.55f + 0.45f * rndC);

            // Diffusion evens the reflection height distribution.
            const float smoothAmp = earlyLevel * early * 0.72f + lateLevel * tail * 0.40f;
            amplitude = lerp(amplitude, smoothAmp + diffuseFloor, diffusion * 0.55f);
            amplitude *= lowBody;
            amplitude = clamp01(amplitude);

            const float x = left + u * plotW;
            const float h = amplitude * upperH * 0.92f;
            const float yTop = horizon - h;

            const float brightnessNoise = 0.62f + 0.38f * rndC;
            const float alpha = clamp01((0.22f + 0.65f * brightness) * brightnessNoise);

            // Broad glow behind each reflection.
            if (glow > 0.01f)
            {
                const float glowWidth = pxLine + 2.0f + glow * 7.0f;
                brush.SetColor(makeColor(mainR, mainG, mainB, alpha * glow * 0.11f));
                g.FillRectangle(x - glowWidth * 0.5f, yTop, x + glowWidth * 0.5f, horizon, brush);
            }

            // Main reflection line. A warm highlight is mixed toward the tallest/earliest spikes.
            const float hot = clamp01(early * earlyLevel * 1.25f + highEnergy * 0.18f);
            const float r = lerp(mainR, hiR, hot * 0.72f);
            const float gg = lerp(mainG, hiG, hot * 0.72f);
            const float b = lerp(mainB, hiB, hot * 0.72f);
            brush.SetColor(makeColor(r, gg, b, alpha));
            g.FillRectangle(x - pxLine * 0.5f, yTop, x + pxLine * 0.5f, horizon, brush);

            // Tiny cap on stronger reflections.
            if (amplitude > 0.22f)
            {
                const float cap = pxLine * 1.5f + 1.2f;
                brush.SetColor(makeColor(hiR, hiG, hiB, alpha * (0.35f + 0.55f * highEnergy)));
                g.FillRectangle(x - cap * 0.5f, yTop - cap * 0.5f, x + cap * 0.5f, yTop + cap * 0.5f, brush);
            }

            // Mirror/reflection below horizon.
            if (mirror > 0.01f)
            {
                const float reflectedH = (std::min)(lowerH * 0.92f, h * (0.18f + 0.55f * mirror));
                const float ripple = 0.60f + 0.40f * std::sin(19.0f * u + rndB * 5.0f);
                brush.SetColor(makeColor(mainR, mainG, mainB, alpha * mirror * 0.18f * ripple));
                const float mirrorWidth = pxLine * (1.3f + 2.4f * mirror);
                g.FillRectangle(x - mirrorWidth * 0.5f, horizon, x + mirrorWidth * 0.5f, horizon + reflectedH, brush);
            }
        }

        // Smooth decay-energy curve. Drawn as small connected rectangles to avoid
        // backend-specific path/geometry edge cases.
        const int curveSegments = (std::max)(48, (std::min)(320, static_cast<int>(plotW / 3.0f))));
        float prevX = left;
        float prevY = horizon;
        for (int i = 0; i <= curveSegments; ++i)
        {
            const float u = static_cast<float>(i) / static_cast<float>(curveSegments);
            const float x = left + u * plotW;

            float env = 0.0f;
            if (u >= pre)
            {
                const float ex = (u - earlyCenter) / (earlyWidth * 1.35f);
                const float early = std::exp(-0.5f * ex * ex);
                const float tu = clamp01((u - pre) / (1.0f - pre));
                const float tail = std::exp(-decayRate * 0.72f * tu);
                env = curveLevel * clamp01(0.72f * earlyLevel * early + 0.48f * lateLevel * tail);
            }

            const float y = horizon - env * upperH * 0.72f;
            if (i > 0)
            {
                const float x1 = (std::min)(prevX, x);
                const float x2 = (std::max)(prevX, x) + 1.0f;
                const float y1 = (std::min)(prevY, y);
                const float y2 = (std::max)(prevY, y);
                const float thickness = (std::max)(1.0f, pxLine * 0.80f);

                if (glow > 0.01f)
                {
                    brush.SetColor(makeColor(hiR, hiG, hiB, 0.06f + 0.16f * glow));
                    g.FillRectangle(x1 - 2.0f - glow * 3.0f, y1 - thickness - 2.0f - glow * 3.0f,
                                    x2 + 2.0f + glow * 3.0f, y2 + thickness + 2.0f + glow * 3.0f, brush);
                }

                brush.SetColor(makeColor(hiR, hiG, hiB, 0.55f + 0.40f * brightness));
                g.FillRectangle(x1, y1 - thickness * 0.5f, x2, y2 + thickness * 0.5f, brush);
            }
            prevX = x;
            prevY = y;
        }

        // Water-like horizontal ripples under the horizon.
        if (mirror > 0.01f && lowerH > 4.0f)
        {
            const int ripples = 12;
            for (int i = 0; i < ripples; ++i)
            {
                const float t = (static_cast<float>(i) + 1.0f) / static_cast<float>(ripples + 1);
                const float y = horizon + t * lowerH;
                const float inset = plotW * t * 0.08f;
                brush.SetColor(makeColor(mainR, mainG, mainB, mirror * (0.075f * (1.0f - t))));
                g.FillRectangle(left + inset, y, right - inset, y + 1.0f, brush);
            }
        }

        // Thin frame.
        brush.SetColor(makeColor(mainR, mainG, mainB, 0.18f + 0.12f * brightness));
        g.FillRectangle(rect.left, rect.top, rect.right, rect.top + 1.0f, brush);
        g.FillRectangle(rect.left, rect.bottom - 1.0f, rect.right, rect.bottom, brush);
        g.FillRectangle(rect.left, rect.top, rect.left + 1.0f, rect.bottom, brush);
        g.FillRectangle(rect.right - 1.0f, rect.top, rect.right, rect.bottom, brush);

        return MP_OK;
    }

private:
    FloatGuiPin pinSize;
    FloatGuiPin pinDiffusion;
    FloatGuiPin pinDecay;
    FloatGuiPin pinLowDamp;
    FloatGuiPin pinHighDamp;
    FloatGuiPin pinPreDelay;
    FloatGuiPin pinEarlyLevel;
    FloatGuiPin pinLateLevel;
    FloatGuiPin pinDensity;
    FloatGuiPin pinGlow;
    FloatGuiPin pinLineWidth;
    FloatGuiPin pinMirror;
    FloatGuiPin pinBrightness;
    FloatGuiPin pinCurveLevel;

    FloatGuiPin pinBgR;
    FloatGuiPin pinBgG;
    FloatGuiPin pinBgB;
    FloatGuiPin pinMainR;
    FloatGuiPin pinMainG;
    FloatGuiPin pinMainB;
    FloatGuiPin pinHiR;
    FloatGuiPin pinHiG;
    FloatGuiPin pinHiB;

    IntGuiPin pinWidthPx;
    IntGuiPin pinHeightPx;
};

GMPI_REGISTER_GUI(
    MP_SUB_TYPE_GUI2,
    ReverbSpaceVisualizerV3Gui,
    L"Pandocrator Reverb Space Visualizer v3"
);
