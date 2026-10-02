SUB OCTAVER -1 TUNABLE v7
=========================

IMPORTANT
---------
This is a NEW SynthEdit module, not a replacement for the original Sub Octaver.
Plugin ID: Pandocrator Sub Octaver Tunable
Module name: Sub Octaver -1 Tunable

This prevents old patches from crashing because the original module's pin layout is untouched.

LIVE ANTI-TAIL CONTROLS
-----------------------
Envelope Release ms   default 45 ms   safe range in DSP: 5..500 ms
Gate Release ms       default 35 ms   safe range in DSP: 2..300 ms
Tail Fade ms          default 5 ms    safe range in DSP: 0.5..100 ms
Zero-Cross Timeout ms default 30 ms   safe range in DSP: 2..200 ms

Suggested starting points for heavy distortion after the sub-octaver:

Balanced:
Envelope Release 30
Gate Release 20
Tail Fade 3
Zero-Cross Timeout 15

Very tight:
Envelope Release 15
Gate Release 8
Tail Fade 1.5
Zero-Cross Timeout 8

If you hear a click, raise Tail Fade first (e.g. 3 -> 5 -> 8 ms).
If you hear a long tail, lower Gate Release and Envelope Release.
If the final tail hangs before hard silence, lower Zero-Cross Timeout.

The DSP clamps these controls internally to safe ranges, so extreme values will not produce runaway timing values.
