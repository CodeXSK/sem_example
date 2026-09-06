Soft Curve Saturator - SynthEdit 1.5 / Jeff sem_example
Version 2 - corrected SynthEdit voltage scaling and stronger waveshaping

PINS
----
Signal In
Shape       : 0..10 V. 0 = exactly linear; 10 = almost-hard smooth clip.
Min Limit   : negative-side saturation level. Default -5 V.
Max Limit   : positive-side saturation level. Default +5 V.
Oversampling: 1, 2, 4 or 8. Default 4x.
Signal Out

IMPORTANT SYNTHEDIT SCALING
---------------------------
SynthEdit audio pins use internal floats where 10 V = 1.0.
Therefore XML default 0.4 appears as 4 V, and 0.5 appears as 5 V.
The module accounts for this for the oversampling selector.

WAVESHAPER
----------
The Shape control morphs continuously:
  0       = y = x exactly
  middle  = rounded S-curve saturation
  10      = near-hard but continuously smooth saturation

There is NO hidden fixed +/-1 DSP clamp. Min Limit and Max Limit are the user's
own saturation bounds and can be moved well beyond +/-10 V if desired.
The XML metadata merely suggests a slider range of -40..+40 V; SynthEdit does
not enforce that metadata range.

ASYMMETRIC EXAMPLE
------------------
Min Limit = -2.5 V
Max Limit = +7.0 V

This clips/saturates the negative side earlier than the positive side, producing
an asymmetric transfer curve and stronger even-order harmonics. Strong asymmetry
can create DC offset; add a DC blocker / very-low HPF after it if required.

OVERSAMPLING
------------
1x, 2x, 4x, 8x are quantized from the Oversampling pin.
The nonlinear stage runs at the selected internal factor and is followed by an
8th-order Butterworth low-pass anti-alias filter before decimation.

BUILD
-----
Place the SoftCurveSaturator folder next to Crusher in Jeff's sem_example repo.
Add to the parent CMakeLists.txt:

    add_subdirectory(SoftCurveSaturator)

Then build via the existing GitHub Actions workflow.
