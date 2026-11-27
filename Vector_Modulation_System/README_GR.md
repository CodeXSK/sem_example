# Vector Modulation System 3 — source, not a validated SEM binary

Build: copy VectorModulationSystem into the legacy sem_example repository and add `add_subdirectory(VectorModulationSystem)` to root CMakeLists.txt. New IDs allow old SEMs to remain installed. Windows/macOS SDK compilation and SE 1.5 runtime are unverified.

## Integrated module
Pandocrator DSP > Vector Modulation System 3. Connect LFO 0–20 V to Modulation In. Defaults: Min 0, Max 20, Refresh 20 Hz, Direct mode. Panel contains the native vector knob; resize it in panel edit mode. Show Knob=false gives ring-only overlay. Base Position is 0–1, Depth is -1..1, Offset Mode enables bipolar modulation around base. The display is not automatically bound to an automatable parameter; use Base Position Out for external parameter routing. Audio Pass Through and Held 0–10 V are optional diagnostic audio outputs; never connect DC to speakers. If host prunes DSP, connect Pass Through to a voltmeter. This module attempts parameter transport via ordinary FloatOutPin assignment, as documented in SDK3, on the same streaming audio path as v1.3. No guarantee of host behavior until tested.

## Independent native display (fallback)
Vector Modulation Display 3 is GUI-only. Connect existing working GUI Float (0–10) from the user's Patch Memory Float Out directly to Position 0–10. Default direct mode. To use the proven v1.3 rate limiter: LFO -> v1.3 Held 30 Hz -> existing working Volts-to-GUI chain -> Position 0–10. If the integrated module has a working Held output, that can replace v1.3. The exact existing Patch Memory chain is installation-dependent; do not connect DSP Float directly to GUI Float. This fallback preserves native drawing but is not a single-module solution and still uses the host's parameter communication.

## Graphics
Sizing, radius, thickness, start/end angles, face ratio, glow, colors AARRGGBB, transparency, range, ticks, dot, value and mouse drag. Standalone input is 0–10; integrated input is 0–20 V by default. Animation Position Out remains normalized 0–1. All code inherited from v2 except input domain, module IDs and integrated DSP wiring. Native renderer has not been compiled against the real SDK here. Core tests and visual math tests passed, including mapping tests at 0,5,10,15,20 V. These do not validate native GUI or host transport.
