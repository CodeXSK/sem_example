Soft Curve Saturator - SynthEdit 1.5 / Jeff sem_example
======================================================

PINS
----
Signal In       Audio input. There is no fixed +/-1 input or output clamp.
Shape           0.0 = exactly linear.
                Mid values = progressively stronger soft S-curve saturation.
                1.0 = almost-hard clip with a small residual slope (not a flat hard ceiling).
Min Limit       Negative-side knee reference. Default -1.0.
Max Limit       Positive-side knee reference. Default +1.0.
Oversampling    Feed 1, 2, 4 or 8. Default 4x.
Signal Out      Processed output; deliberately not clamped to +/-1.

ASYMMETRIC DISTORTION
---------------------
Use different Min/Max magnitudes, for example:
    Min Limit = -0.60
    Max Limit = +1.20
The negative half starts saturating earlier than the positive half.
Asymmetric waveshaping can generate DC offset; add a DC blocker / gentle HPF after the
module when that is undesirable.

OVERSAMPLING
------------
1x = lowest CPU, most aliasing at aggressive settings.
2x = light.
4x = recommended default.
8x = highest CPU, best of the included modes.

The oversampling path uses interpolated intermediate samples, a 4th-order Butterworth
anti-imaging filter before the nonlinear curve, and an 8th-order Butterworth anti-alias
filter before decimation. The 1x path has no oversampling filters.

IMPORTANT
---------
The Min/Max values are KNEE REFERENCES, not hard digital ceilings. Even at Shape=1,
the curve retains a 1.5% slope after the knee. Therefore large input levels can still
produce output above Min/Max, and there is no internal clamp at +/-1.

INSTALL INTO JEFF'S sem_example
-------------------------------
1. Copy the whole SoftCurveSaturator folder into the root of your sem_example fork.
2. Open the root CMakeLists.txt.
3. Add:
       add_subdirectory(SoftCurveSaturator)
   near the existing add_subdirectory(Crusher) line.
4. Commit + push to GitHub.
5. Open GitHub Actions. The workflow should build the Windows and macOS SEM artifacts.

The module itself follows the current JeffMcClintock/sem_example DSP pattern:
FilterBase + AudioInPin/AudioOutPin + build_gmpi_plugin(HAS_DSP).
