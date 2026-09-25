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
        // Width/Height arrive over the DSP->GUI bridge. During VST startup the GUI
        // may be measured before that bridge has delivered its first values. Never
        // request a layout from an invalid/zero startup value.
        const int rawW = pinWidthPx.getValue();
        const int rawH = pinHeightPx.getValue();
        if (rawW < 160 || rawH < 90)
        {
            invalidateRect();
            return;
        }

        const int w = (std::max)(160, (std::min)(2400, rawW));
        const int h = (std::max)(90,  (std::min)(1400, rawH));
        if (w == requestedWidth_ && h == requestedHeight_)
            return;

        requestedWidth_ = w;
        requestedHeight_ = h;

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

        // Critical export fix: uninitialised private GUI pins can briefly read 0
        // before the DSP bridge starts. The old code clamped 0 to 160x90, which
        // explains the intermittent tiny black VST GUI. Use the intended default
        // size until valid Width/Height values are available.
        int w = pinWidthPx.getValue();
        int h = pinHeightPx.getValue();
        if (w < 160) w = requestedWidth_;
        if (h < 90)  h = requestedHeight_;
        w = (std::max)(160, (std::min)(2400, w));
        h = (std::max)(90,  (std::min)(1400, h));

        requestedWidth_ = w;
        requestedHeight_ = h;
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

        // ------------------------------------------------------------
        // Reverb-space visual model
        //
        // SIZE      = spacing / physical spread of reflections.
        //             Small room -> events are compressed together.
        //             Large room -> the same family of events is stretched
        //             over a much wider time/space area, leaving visible gaps.
        // DIFFUSION = event count. 0 = only a few discrete reflections,
        //             1 = a dense cloud of fine reflections.
        // DECAY     = how long the visual energy survives.
        // FEEDBACK  = late recirculation / repeated energy packets.
        // ------------------------------------------------------------
        const float pre = 0.012f + preDelay * 0.19f;

        // SIZE deliberately does not change the envelope amplitude. It stretches
        // the reflection field horizontally so the room *looks* physically larger.
        const float roomSpan = lerp(0.30f, 0.985f, std::pow(size, 0.78f));
        const float usableAfterPre = (std::max)(0.05f, 1.0f - pre);

        // DECAY owns the tail behaviour. Long decay = slower falloff and the
        // envelope remains visible much farther into the display.
        const float decayRate = lerp(10.5f, 0.62f, std::pow(decay, 0.90f));
        const float decayReach = lerp(0.46f, 1.0f, std::pow(decay, 0.70f));

        const float lowBody = lerp(1.08f, 0.63f, lowDamp);
        const float feedbackGain = lerp(0.78f, 1.34f, feedback);
        const float pxLine = (std::max)(1.0f, lerp(0.85f, 3.2f, lineWidth) * (W / 900.0f));

        // Diffusion is now the principal point-count control. Density remains a
        // secondary trim, useful if the user wants to calibrate a particular GUI.
        const float diffusionShape = std::pow(diffusion, 1.35f);
        const float densityTrim = lerp(0.68f, 1.22f, density);
        int events = static_cast<int>((9.0f + 132.0f * diffusionShape) * densityTrim);
        // Do not draw more vertical events than the current pixel width can show.
        // This keeps large diffusion values visually dense without wasting CPU.
        const int pixelEventCap = (std::max)(24, static_cast<int>(plotW / 5.0f));
        events = (std::max)(8, (std::min)((std::min)(150, pixelEventCap), events));

        // A restrained perspective band gives the horizon more depth without
        // turning the display into a spectrum analyser.
        brush.SetColor(asColor(blendRgb(bg, main, 0.10f + 0.10f * brightness)));
        for (int i = 1; i <= 3; ++i)
        {
            const float t = static_cast<float>(i) / 4.0f;
            const float y = horizon - upperH * (0.10f + 0.15f * t);
            const float inset = plotW * 0.08f * t;
            g.DrawLine(Point(left + inset, y), Point(right - inset, y), brush, 1.0f);
        }

        // Deterministic reflection field. Events are almost evenly distributed,
        // with a little jitter, which makes SIZE produce clean, readable spacing.
        for (int i = 0; i < events; ++i)
        {
            const float q = (static_cast<float>(i) + 0.5f) / static_cast<float>(events);
            const float rndA = hash01(static_cast<std::uint32_t>(i * 9781u + 17u));
            const float rndB = hash01(static_cast<std::uint32_t>(i * 6271u + 913u));
            const float rndC = hash01(static_cast<std::uint32_t>(i * 3181u + 701u));

            // Keep jitter modest so low diffusion still looks like discrete echoes.
            const float cell = 1.0f / static_cast<float>(events);
            const float jitter = (rndA - 0.5f) * cell * lerp(0.22f, 0.82f, diffusion);
            const float eventT = clamp01(q + jitter);

            // SIZE stretches the complete reflection field. This is the key visual
            // change requested: bigger size => larger gaps between reflections.
            const float u = pre + eventT * usableAfterPre * roomSpan;
            if (u >= 0.998f)
                continue;

            // Normalized age inside the room's reflection field.
            const float age = clamp01(eventT / (std::max)(0.08f, decayReach));
            const float tail = std::exp(-decayRate * age);

            // A handful of stronger early reflections. Their spacing also follows Size.
            const float earlyWindow = lerp(0.26f, 0.13f, diffusion);
            const float early = std::exp(-0.5f * std::pow(eventT / earlyWindow, 2.0f));

            // Feedback produces soft late-energy waves rather than simply making
            // every bar taller. It reads much more like recirculating reverb.
            const float feedbackWaves =
                0.72f +
                feedback * 0.28f * (0.5f + 0.5f * std::cos(eventT * 23.0f + rndB * 2.4f));

            float amplitude =
                earlyLevel * early * (0.48f + 0.70f * rndB) +
                lateLevel * feedbackGain * tail * feedbackWaves * (0.22f + 0.64f * rndC);

            // High damping increasingly suppresses late/high-frequency energy.
            const float highLoss = std::pow((std::max)(0.0f, 1.0f - age), lerp(0.30f, 3.9f, highDamp));
            const float dampGain = lerp(0.72f, 1.0f, highLoss);
            amplitude *= lowBody * dampGain;

            // Very low diffusion should feel sparse and discrete; high diffusion
            // fills the floor between individual echoes.
            amplitude += lateLevel * tail * diffusion * 0.13f * (0.35f + 0.65f * rndA);
            amplitude = clamp01(amplitude);

            const float x = left + u * plotW;
            const float y = horizon - amplitude * upperH * 0.84f;

            // Late reflections get slightly dimmer/warmer with high damping.
            const float spectralLife = clamp01((1.0f - highDamp * age) * (0.55f + 0.45f * rndC));
            const Rgb eventColor = blendRgb(main, hi, spectralLife * (0.58f + 0.32f * brightness));

            // Glow is deliberately soft and thin so dense diffusion does not turn
            // into a solid wall of light.
            if (glow > 0.01f)
            {
                // At high diffusion there are many overlapping reflections. Draw
                // the expensive glow pass on a subset only; the main bars remain
                // complete, so the appearance stays dense while CPU/GPU work drops.
                const int glowStride = diffusion > 0.78f ? 3 : (diffusion > 0.48f ? 2 : 1);
                if ((i % glowStride) == 0)
                {
                    brush.SetColor(asColor(blendRgb(bg, eventColor, 0.22f + 0.20f * glow)));
                    g.DrawLine(Point(x, horizon), Point(x, y), brush,
                               pxLine + 1.5f + glow * lerp(2.0f, 4.5f, 1.0f - diffusion));
                }
            }

            brush.SetColor(asColor(eventColor));
            g.DrawLine(Point(x, horizon), Point(x, y), brush, pxLine);

            // At low diffusion the individual reflection points are deliberately
            // larger; at high diffusion they become numerous and delicate.
            const float pointRadius = (std::max)(1.0f,
                lerp(3.3f, 1.25f, diffusion) * (0.72f + 0.45f * amplitude) * (W / 900.0f));

            if (glow > 0.03f)
            {
                const int pointGlowStride = diffusion > 0.78f ? 3 : (diffusion > 0.48f ? 2 : 1);
                if ((i % pointGlowStride) == 0)
                {
                    const float gr = pointRadius * (1.7f + glow * 1.8f);
                    brush.SetColor(asColor(blendRgb(bg, eventColor, 0.28f + 0.14f * glow)));
                    g.FillRectangle(Rect(x - gr, y - gr, x + gr, y + gr), brush);
                }
            }

            brush.SetColor(asColor(blendRgb(eventColor, hi, 0.36f)));
            g.FillRectangle(Rect(x - pointRadius, y - pointRadius,
                                 x + pointRadius, y + pointRadius), brush);

            if (mirror > 0.01f)
            {
                const float my = (std::min)(bottom, horizon + (horizon - y) * mirror * 0.42f);
                const float mirrorFade = (1.0f - 0.58f * age) * mirror;
                brush.SetColor(asColor(blendRgb(bg, eventColor, 0.12f + 0.19f * mirrorFade)));
                g.DrawLine(Point(x, horizon + 1.0f), Point(x, my), brush, 1.0f);
            }
        }

        // Smooth reverb envelope. SIZE stretches it horizontally; DECAY determines
        // how slowly it falls. This makes the two controls visually unambiguous.
        const int segments = (std::max)(48, (std::min)(160, static_cast<int>(plotW / 5.0f)));
        auto curveY = [&](float u)
        {
            if (u < pre)
                return horizon;

            const float fieldEnd = pre + usableAfterPre * roomSpan;
            if (u > fieldEnd)
                return horizon;

            const float eventT = clamp01((u - pre) / (std::max)(0.0001f, fieldEnd - pre));
            const float age = clamp01(eventT / (std::max)(0.08f, decayReach));
            const float tail = std::exp(-decayRate * 0.73f * age);
            const float early = std::exp(-0.5f * std::pow(eventT / lerp(0.31f, 0.17f, diffusion), 2.0f));
            const float fbWave = 0.90f + feedback * 0.10f * std::cos(eventT * 16.0f);
            const float env = curveLevel * clamp01(
                0.58f * earlyLevel * early +
                0.58f * lateLevel * feedbackGain * tail * fbWave);
            return horizon - env * upperH * 0.70f;
        };

        if (glow > 0.01f)
        {
            brush.SetColor(asColor(blendRgb(bg, hi, 0.32f + 0.10f * brightness)));
            for (int i = 1; i <= segments; ++i)
            {
                const float u0 = static_cast<float>(i - 1) / static_cast<float>(segments);
                const float u1 = static_cast<float>(i) / static_cast<float>(segments);
                g.DrawLine(Point(left + u0 * plotW, curveY(u0)),
                           Point(left + u1 * plotW, curveY(u1)),
                           brush, 2.0f + glow * 5.0f);
            }
        }

        brush.SetColor(asColor(blendRgb(main, hi, 0.82f)));
        for (int i = 1; i <= segments; ++i)
        {
            const float u0 = static_cast<float>(i - 1) / static_cast<float>(segments);
            const float u1 = static_cast<float>(i) / static_cast<float>(segments);
            g.DrawLine(Point(left + u0 * plotW, curveY(u0)),
                       Point(left + u1 * plotW, curveY(u1)),
                       brush, (std::max)(1.0f, pxLine * 0.88f));
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

    // Safe startup dimensions. These also prevent the first VST layout pass from
    // collapsing to the minimum size before bridge parameters become available.
    int requestedWidth_ = 900;
    int requestedHeight_ = 320;
};

GMPI_REGISTER_GUI(MP_SUB_TYPE_GUI2, ReverbSpaceVisualizerV6Gui, L"Pandocrator Reverb Space Visualizer v6");
