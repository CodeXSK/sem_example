# Pandocrator Waveshaper Toolkit — SynthEdit SDK3

Source code for a Windows x64 SEM, adapted to the build/registration layout of the provided SubOctaver SDK3 example. This is source-only: no compiled `.sem` is included; compilation and live testing in SynthEdit have NOT been performed here.

## Build
Install Visual Studio 2022/2026 C++ desktop workload and CMake + Git.

```bat
cmake -S . -B build -A x64
cmake --build build --config Release
```

With local SDK checkout:
```bat
cmake -S . -B build -A x64 -DSE_SDK_ROOT="C:/path/to/SynthEdit_SDK"
cmake --build build --config Release
```

Find `Waveshaper.sem` in the Release output folder and install/load it according to your version of SynthEdit. The `.xml` is embedded as GMPXML resource. Uses the official SDK source code during the CMake build (if not provided locally).

## Signal pins
All inputs are audio-rate, useful for externally connected parameters. Algorithm is rounded to the nearest integer. Inputs can be changed while running. `Drive` is linear amplitude multiplication, not decibels. `Bias` offsets the input drive signal and the corresponding static response at zero input is subtracted. `A` and `B` have different meanings depending on algorithm. `Wet Out` gives processed signal multiplied by output gain. `Mix Out` crossfades dry/processed then applies output gain. `DC Block` = 1 activates a ~5 Hz HP stage. Output is intentionally NOT level-matched.

## Modes
| Mode | Algorithm | A | B |
|---|---|---|---|
| 0 | tanh(v) | unused | unused |
| 1 | 2/pi * atan(v) | unused | unused |
| 2 | tanh(v + A*v^2) asymmetric tube-like | asymmetry -8..8 | unused |
| 3 | tanh(v+A)-tanh(A) biased tube-like | bias -4..4 | unused |
| 4 | 1.5*z - 0.5*z^3, z=clip(v,-1,1) | unused | unused |
| 5 | v/sqrt(1+v^2) | unused | unused |
| 6 | sign(v)*(1-exp(-abs(v))) | unused | unused |
| 7 | clip(v,-1,1) hardclip | unused | unused |
| 8 | v/(1+abs(A)*abs(v)) | knee hardness .05..8 | unused |
| 9 | triangle wave fold (threshold) | threshold .05..2 | unused |
| 10 | sign(v)*abs(v)^abs(A) | exponent .2..6 | unused |
| 11 | sin(abs(A)*v) | fold rate .1..12 | unused |
| 12 | tanh(v+A*v^2+B*v^3) | quadratic -2..2 | cubic -2..2 |
| 13 | positive/negative independent tanh | positive hardness .1..8 | negative hardness .1..8 |

Where `v = Drive*x + Bias`. The external Bias baseline is subtracted before optional DC blocker. Modes 2/3/12/13 can still create signal-dependent DC offsets.

**Starter presets:**
- Warm tube: Algorithm 2, Drive 2, A 0.2, B 1, Bias 0, DC Block 1.
- Symmetric drive: Algorithm 0, Drive 3, Bias 0, DC Block 0.
- Tape-like basic saturator: Algorithm 0 or 5, Drive 1.5, then external EQ before/after. This is NOT physical tape hysteresis.
- Wavefold: Algorithm 9, Drive 2, A .65.
- Cubic: Algorithm 4, Drive 1.

## Important limitations
- No internal oversampling / anti-alias filtering. For bright/heavy distortion put external oversampling around this SEM or add polyphase oversampling before production use.
- No physical vacuum tube, transformer, magnetic hysteresis, frequency-dependent tape response, or hysteretic memory model.
- Algorithm/mode switching is immediate and may click. Smooth or switch only in silence for product use.
- Audio-rate A/B controls and drive are processed per sample, so audible modulation is possible, and sudden values can click.
- The hard bounded drive/input/outputs are defensive, not a limiter.
- This is a first prototype, not yet host-tested. Verify pin defaults, build and compatibility in your SynthEdit installation.
