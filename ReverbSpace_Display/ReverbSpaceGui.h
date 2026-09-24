#pragma once

#include "../se_sdk3/mp_sdk_gui2.h"

class ReverbSpaceGui final : public gmpi_gui::MpGuiGfxBase
{
    FloatGuiPin pinSize;
    FloatGuiPin pinDiffusion;
    FloatGuiPin pinDecay;
    FloatGuiPin pinLowDamp;
    FloatGuiPin pinHighDamp;

    void onValueChanged();
    static float clamp01(float v);

public:
    ReverbSpaceGui();

    int32_t MP_STDCALL measure(
        GmpiDrawing_API::MP1_SIZE availableSize,
        GmpiDrawing_API::MP1_SIZE* returnDesiredSize) override;

    int32_t MP_STDCALL OnRender(
        GmpiDrawing_API::IMpDeviceContext* drawingContext) override;
};
