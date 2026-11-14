# Pin scale comparison
Working reference: AllPassDiffuser v9

AllPass XML: Stages default=0.8; C++ displayed=raw*10 -> 8.
Waveshaper v3.4: Algorithm default=0; Drive default=0.1 -> DSP 1; A/B default=0.1 -> DSP 1; Wet default=0.1 -> DSP 1; Gain default=0.1 -> DSP 1; DC default=0.1 -> DSP 1; oversampling default=0.

Every control AudioInPin uses raw *10 once. Signal In has separate path and retains its prior scaling.
