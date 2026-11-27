# Pandocrator Modulation Indicator Bridge v1.0
Source project for the legacy SynthEdit SDK (SE 1.5 target), sem_example layout.

## Build
Copy the entire ModulationIndicatorBridge folder into the sem_example root, alongside your earlier modules. Append `add_subdirectory(ModulationIndicatorBridge)` to the root CMakeLists.txt. Run your existing Windows and macOS GitHub Actions build. The helper needs HAS_DSP and HAS_GUI. Do not copy the core test into the build target. Install the resulting .sem as usual. This archive contains source, not a compiled SEM.

## Connection
Audio side: LFO or the actual final modulated control voltage -> Modulation In.
GUI side: Animation Position Out -> existing vector ring Animation Position. Do not connect this to the actual knob value or the audio parameter. No separate Patch Memory Float Out block is required. The bridge internally uses an output parameter (private requested in XML), the official SDK parameter mechanism, not the raw message pipe. Whether your particular SE 1.5 build honors all XML flags, and host behavior, require testing.

### Direct mode (default)
Offset Mode = false. Min=-5 Max=5 for bipolar +/-5 V LFO, or 0 and 10 for 0..10 V, 0 and 5 for 0..5 V. Voltage fields are ordinary float pins in actual VOLTS, not audio-buffer values. SDK audio buffer 1.0 corresponds to 10 V. Output is always normalized 0..1. Reverse Min/Max to invert. If equal, position=0. Use the actual post-modulation voltage for an accurate indicator; avoid changing the sound path.

### Offset mode
Offset Mode = true. Base Position 0..1 (ordinary DSP float pin), Depth -1..1. With Min=-5, Max=5, Base=.5, Depth=.25, output spans .25..75. Formula clamp(Base + Depth*(2*normalizedLFO-1),0,1). A negative depth reverses the movement. Base can be connected to the DSP-side normalized knob value; if unavailable use direct mode with the existing modulated signal. To draw an arc from base to current position, the existing graphics must support that behavior; this module outputs the current position only. The module does not draw the ring or replace your knob.

Refresh Rate Hz: 1..60, default 30. Smoothing ms: 0..1000, default 0. Deadband: 0..1, default .001 (0.1% of full travel); set 0 for exact changes. Enable=false sends 0 in direct mode or Base in offset mode at the next scheduled tick. It does not hide a graphic automatically. Use your existing visibility control if needed. No automatic GUI visibility detection in v1. The module deliberately remains scheduled while processing; static signals are not retransmitted, but each audio block still invokes a cheap scheduler. Fast LFOs can alias visually; this is a display sampler, not an audio resampler. Use 30 or 60 Hz with slow LFOs; the audio modulation itself remains unaffected.

## First host test
Use just one instance, set 20 Hz and connect the GUI output to a simple numeric readout first; check -5/0/+5 V gives 0/.5/1. Then connect vector animation. Compare GUI open/closed, static/active LFO, then 16/32 rings; inspect CPU and cursor, automation list, preset save/restore, reopen editor. Verify GUI pins appear and the internal output is not exposed as an editable DAW parameter. If GUI still hangs, isolate vector redraw from bridge using a numeric readout. Test Windows then Mac.

## Validation
Tests/test_core.cpp is standalone (g++ -std=c++17 -O2 Tests/test_core.cpp -o test_core). Tests cover 8, 44.1, 48, 96, 192, 768 kHz and 1, 15, 30, 60 Hz rates, irregular blocks, static values, NaN/Inf, reversed/equal range, offset and disabled behavior. Actual SDK headers were inspected online but could not be downloaded into the build environment (network blocked). No Windows/macOS build or SynthEdit/DAW run was performed. No claim of eliminating the DAW hang until host tested. Legacy SDK docs recommend a shared output parameter rather than unbounded raw messaging.
