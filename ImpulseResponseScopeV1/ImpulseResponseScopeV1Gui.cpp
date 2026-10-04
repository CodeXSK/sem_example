#include "mp_sdk_gui2.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

using namespace gmpi;
using namespace gmpi_gui;
using namespace GmpiDrawing;

namespace
{
    constexpr int kDisplayBins = 1536;

    inline float clamp01(float v)
    {
        if (!std::isfinite(v)) return 0.0f;
        return (std::max)(0.0f, (std::min)(1.0f, v));
    }

    inline float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    inline float expMap(float lo, float hi, float t)
    {
        t = clamp01(t);
        return lo * std::pow(hi / lo, t);
    }

    struct ScopeHeader
    {
        std::uint32_t magic;
        std::uint32_t version;
        std::uint32_t binCount;
        std::uint32_t reserved;
        float sampleRate;
        float windowSeconds;
        float preTriggerFraction;
        float peak;
        float triggerTimeSeconds;
        float captureSamples;
    };

    struct Rgba
    {
        float a, r, g, b;
    };

    inline Color toColor(const Rgba& c)
    {
        return Color(clamp01(c.r), clamp01(c.g), clamp01(c.b), clamp01(c.a));
    }
}

class ImpulseResponseScopeV1Gui final : public MpGuiGfxBase
{
public:
    ImpulseResponseScopeV1Gui()
    {
        initializePin(pinCapture,      static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinTimeWindow,   static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinPreTrigger,   static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinThreshold,    static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinVerticalGain, static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinFreeze,       static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinBgA,          static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinBgR,          static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinBgG,          static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinBgB,          static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinWaveA,        static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinWaveR,        static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinWaveG,        static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinWaveB,        static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinGridA,        static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onChanged));
        initializePin(pinWidthPx,      static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onSizeChanged));
        initializePin(pinHeightPx,     static_cast<MpGuiBaseMemberPtr2>(&ImpulseResponseScopeV1Gui::onSizeChanged));
    }

    void onChanged()
    {
        invalidateRect();
    }

    void onSizeChanged()
    {
        int w = pinWidthPx.getValue();
        int h = pinHeightPx.getValue();
        if (w < 240 || h < 120)
        {
            invalidateRect();
            return;
        }
        w = (std::max)(240, (std::min)(3000, w));
        h = (std::max)(120, (std::min)(1600, h));
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
        (void)availableSize;
        int w = pinWidthPx.getValue();
        int h = pinHeightPx.getValue();
        if (w < 240) w = requestedWidth_;
        if (h < 120) h = requestedHeight_;
        w = (std::max)(240, (std::min)(3000, w));
        h = (std::max)(120, (std::min)(1600, h));
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
        const float midY = H * 0.5f;

        const Rgba bg{clamp01(pinBgA.getValue()), clamp01(pinBgR.getValue()), clamp01(pinBgG.getValue()), clamp01(pinBgB.getValue())};
        const Rgba wave{clamp01(pinWaveA.getValue()), clamp01(pinWaveR.getValue()), clamp01(pinWaveG.getValue()), clamp01(pinWaveB.getValue())};
        const float gridA = clamp01(pinGridA.getValue());
        const float displayGain = expMap(0.25f, 64.0f, clamp01(pinVerticalGain.getValue()));

        auto brush = g.CreateSolidColorBrush(toColor(bg));
        if (bg.a > 0.0001f)
            g.FillRectangle(Rect(0.0f, 0.0f, W, H), brush);

        // Grid uses waveform RGB with independent alpha, so it stays coherent with custom themes.
        brush.SetColor(Color(wave.r, wave.g, wave.b, gridA * 0.33f));
        g.DrawLine(Point(0.0f, midY), Point(W, midY), brush, 1.0f);
        for (int i = 1; i < 10; ++i)
        {
            const float x = W * static_cast<float>(i) / 10.0f;
            g.DrawLine(Point(x, 0.0f), Point(x, H), brush, 1.0f);
        }
        for (int i = 1; i < 4; ++i)
        {
            const float y = H * static_cast<float>(i) / 4.0f;
            if (std::fabs(y - midY) > 1.0f)
                g.DrawLine(Point(0.0f, y), Point(W, y), brush, 1.0f);
        }

        const int rawSize = pinCapture.rawSize();
        const void* raw = pinCapture.rawData();
        if (raw && rawSize >= static_cast<int>(sizeof(ScopeHeader)))
        {
            ScopeHeader header{};
            std::memcpy(&header, raw, sizeof(ScopeHeader));
            const size_t expected = sizeof(ScopeHeader) + static_cast<size_t>(header.binCount) * 3u * sizeof(float);
            if (header.magic == 0x49525331u && header.version == 1 && header.binCount > 0 && header.binCount <= 16384 && static_cast<size_t>(rawSize) >= expected)
            {
                const auto* bytes = reinterpret_cast<const std::uint8_t*>(raw);
                const float* mins = reinterpret_cast<const float*>(bytes + sizeof(ScopeHeader));
                const float* maxs = mins + header.binCount;
                const float* rms = maxs + header.binCount;

                // Trigger marker.
                const float trigX = clamp01(header.preTriggerFraction) * W;
                brush.SetColor(Color(wave.r, wave.g, wave.b, wave.a * 0.22f));
                g.DrawLine(Point(trigX, 0.0f), Point(trigX, H), brush, 1.0f);

                // RMS silhouette: subtle indication of tail energy.
                brush.SetColor(Color(wave.r, wave.g, wave.b, wave.a * 0.18f));
                const int drawBins = static_cast<int>(header.binCount);
                for (int b = 0; b < drawBins; ++b)
                {
                    const float x = (drawBins <= 1) ? 0.0f : W * static_cast<float>(b) / static_cast<float>(drawBins - 1);
                    const float amp = clamp01(std::fabs(rms[b]) * displayGain);
                    const float hh = amp * (H * 0.46f);
                    if (hh > 0.25f)
                        g.DrawLine(Point(x, midY - hh), Point(x, midY + hh), brush, 1.0f);
                }

                // Main min/max trace. This is the anti-missed-click path: every source
                // sample contributes to min/max, so a one-sample impulse survives zoom-out.
                brush.SetColor(toColor(wave));
                const float half = H * 0.46f;
                for (int b = 0; b < drawBins; ++b)
                {
                    const float x = (drawBins <= 1) ? 0.0f : W * static_cast<float>(b) / static_cast<float>(drawBins - 1);
                    float mn = mins[b] * displayGain;
                    float mx = maxs[b] * displayGain;
                    mn = (std::max)(-1.2f, (std::min)(1.2f, mn));
                    mx = (std::max)(-1.2f, (std::min)(1.2f, mx));
                    const float yTop = midY - mx * half;
                    const float yBottom = midY - mn * half;
                    g.DrawLine(Point(x, yTop), Point(x, yBottom), brush, 1.25f);
                }
            }
        }

        // Fine border. With BG alpha=0 this is also transparent except for this line.
        brush.SetColor(Color(wave.r, wave.g, wave.b, wave.a * 0.28f));
        g.DrawRectangle(Rect(0.5f, 0.5f, W - 0.5f, H - 0.5f), brush, 1.0f);
        return gmpi::MP_OK;
    }

private:
    BlobGuiPin pinCapture;
    FloatGuiPin pinTimeWindow, pinPreTrigger, pinThreshold, pinVerticalGain, pinFreeze;
    FloatGuiPin pinBgA, pinBgR, pinBgG, pinBgB;
    FloatGuiPin pinWaveA, pinWaveR, pinWaveG, pinWaveB;
    FloatGuiPin pinGridA;
    IntGuiPin pinWidthPx, pinHeightPx;

    int requestedWidth_ = 1200;
    int requestedHeight_ = 360;
};

GMPI_REGISTER_GUI(MP_SUB_TYPE_GUI2, ImpulseResponseScopeV1Gui, L"Pandocrator Impulse Response Scope v1");
