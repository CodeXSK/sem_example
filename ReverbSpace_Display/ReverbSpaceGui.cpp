#define _USE_MATH_DEFINES
#include "ReverbSpaceGui.h"

#include <algorithm>
#include <cmath>

using namespace gmpi;
using namespace gmpi_gui;
using namespace GmpiDrawing;

GMPI_REGISTER_GUI(MP_SUB_TYPE_GUI2, ReverbSpaceGui, L"Pandocrator Reverb Space Display");

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    float smoothstep(float a, float b, float x)
    {
        if (a == b)
            return x >= b ? 1.0f : 0.0f;

        x = (x - a) / (b - a);
        x = (std::max)(0.0f, (std::min)(1.0f, x));
        return x * x * (3.0f - 2.0f * x);
    }
}

ReverbSpaceGui::ReverbSpaceGui()
{
    initializePin(pinSize,      static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceGui::onValueChanged));
    initializePin(pinDiffusion, static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceGui::onValueChanged));
    initializePin(pinDecay,     static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceGui::onValueChanged));
    initializePin(pinLowDamp,   static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceGui::onValueChanged));
    initializePin(pinHighDamp,  static_cast<MpGuiBaseMemberPtr2>(&ReverbSpaceGui::onValueChanged));
}

float ReverbSpaceGui::clamp01(float v)
{
    return (std::max)(0.0f, (std::min)(1.0f, v));
}

void ReverbSpaceGui::onValueChanged()
{
    invalidateRect();
}

int32_t ReverbSpaceGui::measure(
    GmpiDrawing_API::MP1_SIZE /*availableSize*/,
    GmpiDrawing_API::MP1_SIZE* returnDesiredSize)
{
    returnDesiredSize->width = 1000.0f;
    returnDesiredSize->height = 260.0f;
    return gmpi::MP_OK;
}

int32_t ReverbSpaceGui::OnRender(GmpiDrawing_API::IMpDeviceContext* drawingContext)
{
    Graphics g(drawingContext);
    const auto r = getRect();

    const float W = r.right - r.left;
    const float H = r.bottom - r.top;
    if (W <= 2.0f || H <= 2.0f)
        return gmpi::MP_OK;

    const float size      = clamp01(pinSize.getValue());
    const float diffusion = clamp01(pinDiffusion.getValue());
    const float decay     = clamp01(pinDecay.getValue());
    const float lowDamp   = clamp01(pinLowDamp.getValue());
    const float highDamp  = clamp01(pinHighDamp.getValue());

    // ---------------------------------------------------------------------
    // SKY / ROOM FIELD
    // Procedural layered background: no bitmap dependency.
    // ---------------------------------------------------------------------
    auto brush = g.CreateSolidColorBrush(Color(0.025f, 0.045f, 0.070f, 1.0f));
    g.FillRectangle(r, brush);

    const int skyLayers = 28;
    const float horizonY = r.top + H * 0.66f;

    for (int i = 0; i < skyLayers; ++i)
    {
        const float t0 = static_cast<float>(i) / skyLayers;
        const float t1 = static_cast<float>(i + 1) / skyLayers;

        // High damping darkens the high/upper atmosphere.
        const float highLoss = 1.0f - 0.58f * highDamp * (1.0f - t0);
        const float roomLift = 0.05f + 0.12f * size;

        const float rr = (0.030f + 0.090f * t0 + roomLift * t0) * highLoss;
        const float gg = (0.055f + 0.155f * t0 + roomLift * 0.70f * t0) * highLoss;
        const float bb = (0.085f + 0.230f * t0 + roomLift * 0.55f * t0) * highLoss;

        brush.SetColor(Color(rr, gg, bb, 1.0f));
        const float y0 = lerp(r.top, horizonY, t0);
        const float y1 = lerp(r.top, horizonY, t1) + 1.0f;
        g.FillRectangle(Rect(r.left, y0, r.right, y1), brush);
    }

    // ---------------------------------------------------------------------
    // WATER / FLOOR
    // Low damping controls how much low-frequency mass/reflection remains.
    // ---------------------------------------------------------------------
    const int waterLayers = 22;
    for (int i = 0; i < waterLayers; ++i)
    {
        const float t0 = static_cast<float>(i) / waterLayers;
        const float t1 = static_cast<float>(i + 1) / waterLayers;
        const float lowEnergy = 1.0f - 0.60f * lowDamp;

        const float rr = 0.045f + 0.060f * (1.0f - t0) * lowEnergy;
        const float gg = 0.090f + 0.120f * (1.0f - t0) * lowEnergy;
        const float bb = 0.125f + 0.165f * (1.0f - t0) * lowEnergy;

        brush.SetColor(Color(rr, gg, bb, 1.0f));
        const float y0 = lerp(horizonY, r.bottom, t0);
        const float y1 = lerp(horizonY, r.bottom, t1) + 1.0f;
        g.FillRectangle(Rect(r.left, y0, r.right, y1), brush);
    }

    // Horizon glow.
    const float cx = (r.left + r.right) * 0.5f;
    const float span = W * (0.16f + 0.34f * size);
    const int glowBands = 18;
    for (int i = glowBands - 1; i >= 0; --i)
    {
        const float t = static_cast<float>(i) / glowBands;
        const float halfW = span * (0.25f + 0.90f * t);
        const float halfH = H * (0.008f + 0.065f * t);
        const float alpha = 0.020f + 0.055f * (1.0f - t) + 0.025f * decay;

        brush.SetColor(Color(0.78f, 0.88f, 1.0f, alpha));
        g.FillRectangle(Rect(cx - halfW, horizonY - halfH, cx + halfW, horizonY + halfH), brush);
    }

    // Central excitation / source light.
    for (int i = 0; i < 12; ++i)
    {
        const float t = static_cast<float>(i) / 11.0f;
        const float x = cx + (t - 0.5f) * (2.0f + 22.0f * size);
        const float a = (1.0f - std::fabs(t - 0.5f) * 2.0f);
        brush.SetColor(Color(0.95f, 0.98f, 1.0f, 0.18f + 0.25f * a));
        g.DrawLine(Point(x, horizonY - H * (0.11f + 0.20f * decay)), Point(x, horizonY + H * 0.22f), brush, 1.0f + 1.5f * a);
    }

    // ---------------------------------------------------------------------
    // REVERB ENVELOPE CURVE
    // Size widens the lobes, decay lifts/extends the tails.
    // Diffusion adds local micro-structure.
    // ---------------------------------------------------------------------
    auto curveGeo = g.GetFactory().CreatePathGeometry();
    auto curveSink = curveGeo.Open();

    const int points = 260;
    const float usable = W * (0.92f + 0.05f * size);
    const float xStart = cx - usable * 0.5f;
    const float xEnd = cx + usable * 0.5f;

    bool first = true;
    for (int i = 0; i < points; ++i)
    {
        const float u = static_cast<float>(i) / (points - 1);
        const float x = lerp(xStart, xEnd, u);
        const float d = std::fabs(u - 0.5f) * 2.0f; // 0 center -> 1 edge

        const float envelope = std::pow((std::max)(0.0f, 1.0f - d), 0.48f + 1.45f * (1.0f - decay));
        const float roomWave = std::sin((u * (2.25f + 2.7f * size) + 0.11f) * 2.0f * kPi);
        const float micro = std::sin((u * (11.0f + 25.0f * diffusion) + 0.31f) * 2.0f * kPi);

        const float amp = H * (0.065f + 0.20f * decay);
        const float y = horizonY
            - amp * (0.28f + 0.72f * envelope)
            - roomWave * H * (0.010f + 0.035f * size)
            - micro * H * (0.0015f + 0.010f * diffusion) * envelope;

        if (first)
        {
            curveSink.BeginFigure(Point(x, y));
            first = false;
        }
        else
        {
            curveSink.AddLine(Point(x, y));
        }
    }
    curveSink.EndFigure(FigureEnd::Open);
    curveSink.Close();

    // soft under-glow then white/gold core.
    brush.SetColor(Color(0.53f, 0.78f, 1.0f, 0.20f + 0.18f * decay));
    g.DrawGeometry(curveGeo, brush, 5.0f + 2.0f * diffusion);

    brush.SetColor(Color(0.96f, 0.90f, 0.74f, 0.92f));
    g.DrawGeometry(curveGeo, brush, 1.35f + 0.45f * size);

    // ---------------------------------------------------------------------
    // EARLY / LATE REFLECTION BARS
    // Diffusion = density. Decay = height/tail. Damping shapes spectral tilt.
    // ---------------------------------------------------------------------
    const int barCount = 24 + static_cast<int>(104.0f * diffusion);
    const float highKeep = 1.0f - 0.80f * highDamp;
    const float lowKeep  = 1.0f - 0.70f * lowDamp;

    for (int i = 0; i < barCount; ++i)
    {
        const float u = (i + 0.5f) / barCount;
        const float x = lerp(r.left + W * 0.045f, r.right - W * 0.045f, u);
        const float d = std::fabs(u - 0.5f) * 2.0f;
        const float centerWeight = std::pow(1.0f - d, 0.45f + 1.8f * (1.0f - decay));

        // Deterministic pseudo-random looking modulation.
        const float jitter = 0.5f + 0.5f * std::sin((i * 2.399963f + 0.7f) * (1.0f + diffusion * 0.4f));
        const float spectralPosition = u;
        const float dampingTilt = lerp(lowKeep, highKeep, spectralPosition);

        float h = H * (0.045f + 0.30f * centerWeight * (0.30f + 0.70f * decay));
        h *= (0.68f + 0.48f * jitter);
        h *= (0.45f + 0.55f * dampingTilt);

        const float top = horizonY - h;
        const float alpha = 0.22f + 0.55f * centerWeight * dampingTilt;

        brush.SetColor(Color(0.68f, 0.86f, 1.0f, alpha));
        g.DrawLine(Point(x, horizonY - 1.0f), Point(x, top), brush, 1.0f + 1.1f * diffusion);

        // brighter tip
        brush.SetColor(Color(0.93f, 0.98f, 1.0f, 0.34f + 0.58f * centerWeight));
        g.DrawLine(Point(x, top), Point(x, top + 1.5f), brush, 2.0f);

        // mirrored water reflection, attenuated heavily.
        const float reflectionDepth = h * (0.18f + 0.24f * lowKeep) * (0.55f + 0.45f * decay);
        brush.SetColor(Color(0.62f, 0.80f, 0.96f, 0.06f + 0.12f * centerWeight * lowKeep));
        g.DrawLine(Point(x, horizonY + 2.0f), Point(x, horizonY + reflectionDepth), brush, 1.0f);
    }

    // Water ripples / reflected tail.
    const int rippleCount = 12 + static_cast<int>(18.0f * diffusion);
    for (int i = 0; i < rippleCount; ++i)
    {
        const float t = static_cast<float>(i) / (std::max)(1, rippleCount - 1);
        const float y = horizonY + H * (0.025f + t * 0.30f);
        const float width = W * (0.08f + t * (0.20f + 0.25f * size));
        const float fade = (1.0f - t) * (0.18f + 0.18f * decay) * lowKeep;

        brush.SetColor(Color(0.74f, 0.87f, 1.0f, fade));
        g.DrawLine(Point(cx - width, y), Point(cx + width, y), brush, 1.0f);
    }

    // Thin frame.
    brush.SetColor(Color(0.78f, 0.86f, 0.93f, 0.20f));
    g.DrawRectangle(Rect(r.left + 0.5f, r.top + 0.5f, r.right - 0.5f, r.bottom - 0.5f), brush, 1.0f);

    return gmpi::MP_OK;
}
