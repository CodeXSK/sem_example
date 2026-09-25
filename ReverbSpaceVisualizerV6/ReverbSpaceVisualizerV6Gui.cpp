#include "mp_sdk_gui.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace gmpi;

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

    inline int c255(float x)
    {
        return static_cast<int>(clamp01(x) * 255.0f + 0.5f);
    }

    inline COLORREF colorFrom(float r, float g, float b, float brightness = 1.0f)
    {
        return RGB(c255(r * brightness), c255(g * brightness), c255(b * brightness));
    }

    inline COLORREF blend(COLORREF a, COLORREF b, float t)
    {
        t = clamp01(t);
        auto ch = [t](int x, int y) { return static_cast<int>(x + (y - x) * t + 0.5f); };
        return RGB(
            ch(GetRValue(a), GetRValue(b)),
            ch(GetGValue(a), GetGValue(b)),
            ch(GetBValue(a), GetBValue(b))
        );
    }
}

// IMPORTANT: this is deliberately a *composited* SynthEdit GUI, matching the
// architecture used by the official Scope3 module. That allows one module to
// have BOTH real DSP/structure pins and a custom panel visual.
class ReverbSpaceVisualizerV6Gui final : public SeGuiCompositedGfxBase
{
public:
    explicit ReverbSpaceVisualizerV6Gui(IMpUnknown* host)
        : SeGuiCompositedGfxBase(host)
    {
        initializePin(0,  pinSize,       static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(1,  pinDiffusion,  static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(2,  pinFeedback,   static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(3,  pinDecay,      static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(4,  pinLowDamp,    static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(5,  pinHighDamp,   static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(6,  pinPreDelay,   static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(7,  pinEarlyLevel, static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(8,  pinLateLevel,  static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(9,  pinDensity,    static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(10, pinGlow,       static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(11, pinLineWidth,  static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(12, pinMirror,     static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(13, pinBrightness, static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(14, pinCurveLevel, static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(15, pinBgR,        static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(16, pinBgG,        static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(17, pinBgB,        static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(18, pinMainR,      static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(19, pinMainG,      static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(20, pinMainB,      static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(21, pinHiR,        static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(22, pinHiG,        static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(23, pinHiB,        static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(24, pinWidthPx,    static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
        initializePin(25, pinHeightPx,   static_cast<MpGuiBaseMemberPtr>(&ReverbSpaceVisualizerV6Gui::onChanged));
    }

    void onChanged()
    {
        invalidateRect();
    }

    int32_t MP_STDCALL measure(MpSize availableSize, MpSize& returnDesiredSize) override
    {
        (void) availableSize;
        int w = pinWidthPx.getValue();
        int h = pinHeightPx.getValue();
        w = (std::max)(160, (std::min)(2400, w));
        h = (std::max)(90,  (std::min)(1400, h));
        returnDesiredSize.x = static_cast<float>(w);
        returnDesiredSize.y = static_cast<float>(h);
        return gmpi::MP_OK;
    }

    int32_t MP_STDCALL paint(HDC hDC) override
    {
        const MpRect r = getRect();
        const int W = (std::max)(1, static_cast<int>(r.right - r.left));
        const int H = (std::max)(1, static_cast<int>(r.bottom - r.top));

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

        const float bgR = clamp01(pinBgR.getValue());
        const float bgG = clamp01(pinBgG.getValue());
        const float bgB = clamp01(pinBgB.getValue());
        const float mainR = clamp01(pinMainR.getValue());
        const float mainG = clamp01(pinMainG.getValue());
        const float mainB = clamp01(pinMainB.getValue());
        const float hiR = clamp01(pinHiR.getValue());
        const float hiG = clamp01(pinHiG.getValue());
        const float hiB = clamp01(pinHiB.getValue());

        const COLORREF bg = colorFrom(bgR, bgG, bgB, 1.0f);
        const COLORREF main = colorFrom(mainR, mainG, mainB, 0.45f + 0.75f * brightness);
        const COLORREF hi = colorFrom(hiR, hiG, hiB, 0.55f + 0.70f * brightness);

        RECT rr{0, 0, W + 1, H + 1};
        HBRUSH back = CreateSolidBrush(bg);
        FillRect(hDC, &rr, back);
        DeleteObject(back);

        const int marginX = (std::max)(6, static_cast<int>(W * 0.022f));
        const int marginY = (std::max)(5, static_cast<int>(H * 0.04f));
        const int left = marginX;
        const int right = W - marginX;
        const int top = marginY;
        const int bottom = H - marginY;
        const int plotW = (std::max)(1, right - left);
        const int plotH = (std::max)(1, bottom - top);
        const int horizon = top + static_cast<int>(plotH * (0.69f - 0.13f * mirror));
        const int upperH = (std::max)(1, horizon - top);
        const int lowerH = (std::max)(1, bottom - horizon);

        // subtle grid
        HPEN gridPen = CreatePen(PS_SOLID, 1, blend(bg, main, 0.18f));
        HGDIOBJ oldPen = SelectObject(hDC, gridPen);
        for (int i = 1; i < 5; ++i)
        {
            int y = top + upperH * i / 5;
            MoveToEx(hDC, left, y, nullptr);
            LineTo(hDC, right, y);
        }
        SelectObject(hDC, oldPen);
        DeleteObject(gridPen);

        // horizon glow: draw thicker darker light under a thin bright line.
        HPEN horizonGlowPen = CreatePen(PS_SOLID, 1 + static_cast<int>(1 + glow * 5), blend(bg, main, 0.55f));
        oldPen = SelectObject(hDC, horizonGlowPen);
        MoveToEx(hDC, left, horizon, nullptr); LineTo(hDC, right, horizon);
        SelectObject(hDC, oldPen); DeleteObject(horizonGlowPen);
        HPEN horizonPen = CreatePen(PS_SOLID, 1, hi);
        oldPen = SelectObject(hDC, horizonPen);
        MoveToEx(hDC, left, horizon, nullptr); LineTo(hDC, right, horizon);
        SelectObject(hDC, oldPen); DeleteObject(horizonPen);

        const float pre = 0.015f + preDelay * 0.18f;
        const float earlyCenter = pre + 0.08f + 0.18f * size;
        const float earlyWidth = 0.055f + 0.16f * size;
        const float decayRate = lerp(8.5f, 1.15f, decay) * lerp(1.0f, 0.52f, feedback);
        const float dampCurve = lerp(1.0f, 3.6f, highDamp);
        const float lowBody = lerp(0.65f, 1.10f, 1.0f - lowDamp);
        const float feedbackGain = lerp(0.72f, 1.28f, feedback);
        int bars = static_cast<int>(lerp(36.0f, 220.0f, density));
        bars = (std::max)(36, (std::min)(220, bars));
        const int pxLine = (std::max)(1, static_cast<int>(lerp(1.0f, 4.0f, lineWidth) * (W / 900.0f) + 0.5f));

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
            const int x = left + static_cast<int>(u * plotW);
            const int y = horizon - static_cast<int>(amplitude * upperH * 0.86f);

            if (glow > 0.01f)
            {
                HPEN p = CreatePen(PS_SOLID, pxLine + 2 + static_cast<int>(glow * 5), blend(bg, main, 0.35f + 0.25f * glow));
                HGDIOBJ o = SelectObject(hDC, p);
                MoveToEx(hDC, x, horizon, nullptr); LineTo(hDC, x, y);
                SelectObject(hDC, o); DeleteObject(p);
            }

            HPEN p = CreatePen(PS_SOLID, pxLine, blend(main, hi, fine * 0.75f));
            HGDIOBJ o = SelectObject(hDC, p);
            MoveToEx(hDC, x, horizon, nullptr); LineTo(hDC, x, y);
            SelectObject(hDC, o); DeleteObject(p);

            // top sparkle
            const int s = (std::max)(1, pxLine + 1);
            RECT sr{x - s, y - s, x + s + 1, y + s + 1};
            HBRUSH sb = CreateSolidBrush(blend(main, hi, 0.75f));
            FillRect(hDC, &sr, sb); DeleteObject(sb);

            if (mirror > 0.01f)
            {
                const int my = horizon + static_cast<int>((horizon - y) * mirror * 0.46f);
                HPEN mp = CreatePen(PS_SOLID, 1, blend(bg, main, 0.20f + 0.25f * mirror));
                HGDIOBJ mo = SelectObject(hDC, mp);
                MoveToEx(hDC, x, horizon + 1, nullptr); LineTo(hDC, x, (std::min)(bottom, my));
                SelectObject(hDC, mo); DeleteObject(mp);
            }
        }

        // smooth decay/envelope curve as connected line segments.
        const int segments = (std::max)(48, (std::min)(320, plotW / 3));
        auto curveY = [&](float u)
        {
            if (u < pre) return horizon;
            const float earlyX = (u - earlyCenter) / (earlyWidth * 1.28f);
            const float early = std::exp(-0.5f * earlyX * earlyX);
            const float tu = clamp01((u - pre) / (1.0f - pre));
            const float tail = std::exp(-decayRate * 0.72f * tu);
            const float env = curveLevel * clamp01(0.72f * earlyLevel * early + 0.48f * lateLevel * feedbackGain * tail);
            return horizon - static_cast<int>(env * upperH * 0.72f);
        };

        if (glow > 0.01f)
        {
            HPEN gp = CreatePen(PS_SOLID, 3 + static_cast<int>(glow * 7), blend(bg, hi, 0.42f));
            HGDIOBJ go = SelectObject(hDC, gp);
            for (int i = 0; i <= segments; ++i)
            {
                float u = static_cast<float>(i) / static_cast<float>(segments);
                int x = left + static_cast<int>(u * plotW);
                int y = curveY(u);
                if (i == 0) MoveToEx(hDC, x, y, nullptr); else LineTo(hDC, x, y);
            }
            SelectObject(hDC, go); DeleteObject(gp);
        }
        HPEN cp = CreatePen(PS_SOLID, (std::max)(1, pxLine), hi);
        HGDIOBJ co = SelectObject(hDC, cp);
        for (int i = 0; i <= segments; ++i)
        {
            float u = static_cast<float>(i) / static_cast<float>(segments);
            int x = left + static_cast<int>(u * plotW);
            int y = curveY(u);
            if (i == 0) MoveToEx(hDC, x, y, nullptr); else LineTo(hDC, x, y);
        }
        SelectObject(hDC, co); DeleteObject(cp);

        // water ripples
        if (mirror > 0.01f && lowerH > 4)
        {
            HPEN rp = CreatePen(PS_SOLID, 1, blend(bg, main, 0.20f + mirror * 0.16f));
            HGDIOBJ ro = SelectObject(hDC, rp);
            for (int i = 1; i <= 11; ++i)
            {
                float t = static_cast<float>(i) / 12.0f;
                int y = horizon + static_cast<int>(t * lowerH);
                int inset = static_cast<int>(plotW * t * 0.08f);
                MoveToEx(hDC, left + inset, y, nullptr); LineTo(hDC, right - inset, y);
            }
            SelectObject(hDC, ro); DeleteObject(rp);
        }

        // frame
        HPEN fp = CreatePen(PS_SOLID, 1, blend(bg, main, 0.45f));
        HGDIOBJ fo = SelectObject(hDC, fp);
        Rectangle(hDC, 0, 0, W, H);
        SelectObject(hDC, fo); DeleteObject(fp);

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

REGISTER_GUI_PLUGIN(ReverbSpaceVisualizerV6Gui, L"Pandocrator Reverb Space Visualizer v6");
