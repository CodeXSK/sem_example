# Build / CI — Pandocrator Waveshaper v3

1. Place all repository files at the root. Include `.github/workflows/build.yml`.
2. Go to Actions → Build SynthEdit Waveshaper (Windows + macOS) → Run workflow.
3. Download platform-specific artifacts from successful runs.

**macOS:** CMake enables `RC` only on Windows. On macOS the XML is packaged into the `.sem` bundle resources.
**Windows:** the `.rc` resources embed plugin metadata as configured by the existing project.

Local Windows: `cmake -S . -B build -A x64` then `cmake --build build --config Release`.
Local Mac: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` then `cmake --build build --config Release`.

The SDK is fetched from `JeffMcClintock/SynthEdit_SDK` unless `-DSE_SDK_ROOT=...` is supplied.

**Engineering notes:** native audio ±10 V = normalized ±1. Oversampling 0/1/2/3/4 selects 1x/2x/4x/8x/16x. No host latency reporting (PDC) is implemented. Dry mix uses integer sample alignment. Switching oversampling modes can click. FIR filtering attenuates aliasing but cannot fully eliminate it.


## Control scaling v3.2
All audio-rate **control** pins are decoded using `raw / 10`. XML defaults are in native units (10 = 1 control unit). Audio signal input/output scaling is unchanged. See `README_GR.md`.
