# Pandocrator Waveshaper Toolkit v3 — 40 DSP Algorithms, 16x Oversampling

Βασισμένο στο cross-platform SynthEdit SDK3 project που έστειλες. Διατηρεί το plugin ID `Pandocrator Waveshaper Toolkit`, τα αρχικά pins με την ίδια σειρά και προσθέτει **Oversampling** πριν από τα δύο outputs. Το module ελέγχεται εξολοκλήρου από pins και δεν έχει δικό του GUI.

## ΣΗΜΑΝΤΙΚΟ: Κλίμακα SynthEdit (10 V)

**10 V στο `Signal In` = normalized +1.0**. **1 V = normalized +0.1**.
Εσωτερικά: `x = SignalIn / 10`, επεξεργασία σε normalized signal, έπειτα `AudioOut = normalizedOut * 10`.
**Drive, A, B, Bias, Wet, Output Gain, Algorithm και Oversampling είναι απλοί αριθμητικοί συντελεστές, όχι audio-volts.** Για παράδειγμα `Drive=2` σημαίνει 2, όχι 20.
Σε περιβάλλον SynthEdit, συνδέεις ελέγχους που βγάζουν την επιθυμητή αριθμητική τιμή στα control pins. Αν περνάνε από μετατροπέα με scaling 10 V, ρύθμισέ τον ανάλογα.

## Pins

- Signal In: audio volts
- Algorithm: integer 0..39
- Drive: 0..32
- A: -8..8 (ερμηνεία ανά αλγόριθμο)
- B: -8..8 (ερμηνεία ανά αλγόριθμο)
- Bias: -4..4 (normalized offset **πριν** τη nonlinear function)
- Wet: 0..1
- Output Gain: 0..8
- DC Block: 0/1 (one-pole HP περίπου 5 Hz, bypass όταν 0)
- Oversampling: **0=1x, 1=2x, 2=4x, 3=8x, 4=16x**
- Wet Out: επεξεργασμένο σε SynthEdit audio volts
- Mix Out: dry/wet με latency-aligned dry σε SynthEdit audio volts

## Algorithms

| ID | Όνομα | Παράμετροι A / B |
|---:|---|---|
| 0 | Tanh | — |
| 1 | Arctan | — |
| 2 | Asymmetric tube-like tanh | A: x² asymmetry |
| 3 | Shifted/Biased tanh | A: shift |
| 4 | Cubic soft clip | — |
| 5 | Algebraic saturation | — |
| 6 | Exponential saturation | — |
| 7 | Hard clip | — |
| 8 | Rational soft clip | A: curvature |
| 9 | Triangle wavefold | A: threshold |
| 10 | Signed power | A: exponent |
| 11 | Sine fold | A: frequency multiplier |
| 12 | Polynomial tanh | A: x², B: x³ |
| 13 | Dual asymmetric tanh | A: positive slope, B: negative slope |
| 14 | Arc sine | — |
| 15 | asinh drive curve | A: output normalization |
| 16 | Rational resonant fold | A: bend |
| 17 | Bell attenuation | A: decay |
| 18 | Tanh + even component | A: second component |
| 19 | Asymmetric hard limiter | A: positive threshold, B: negative threshold |
| 20 | Single reflection fold | A: threshold |
| 21 | Nested sine fold | A: depth, B: inner frequency |
| 22 | Tanh + sine modulation | A: depth, B: inner frequency |
| 23 | Arctan asymmetric | A: x² coefficient |
| 24 | Fourth-power asymmetric tanh | A: x⁴ coefficient |
| 25 | Fifth-power tanh | A: x⁵ coefficient |
| 26 | Tanh–sine morph | A: morph 0..1 |
| 27 | Powered tanh | A: exponent |
| 28 | Hyperbolic rational family | A: exponent |
| 29 | Deadzone | A: threshold |
| 30 | Comparator | A: threshold |
| 31 | Quantizer | A: number of steps |
| 32 | Triangle folding | A: threshold |
| 33 | Sine × cosine | A: cos multiplier |
| 34 | Even-order polynomial tanh | A: x², B: x⁴ |
| 35 | Root-exponential | A: strength |
| 36 | Normalized tanh | A: hardness |
| 37 | Normalized arctan | A: hardness |
| 38 | Modulated asymmetric tanh | A: x², B: cosine multiplier |
| 39 | Logarithmic shaping | A: curvature |

Οι αλγόριθμοι 14–39 είναι **μαθηματικοί waveshapers** — όχι ακριβείς αναλογικές εξομοιώσεις tube/tape/hysteresis.
Οι μη συμμετρικοί waveshapers μπορούν να παράγουν DC offset, γι' αυτό προτείνεται `DC Block=1`.

## Oversampling

Χρησιμοποιείται πραγματικό **2× cascaded FIR upsampling και anti-aliasing FIR downsampling** έως 16×.
Κάθε στάδιο: 49-tap Blackman-windowed sinc halfband FIR. Αυτό μειώνει το aliasing, αλλά **δεν εγγυάται μηδενικό aliasing** και απαιτεί περισσότερη CPU σε μεγαλύτερα factors. Αρχικά δοκίμασε 4×. Οι βαθμίδες FIR δίνουν group delay περίπου 0, 23, 35, 41, 44 samples για 1×,2×,4×,8×,16× αντίστοιχα· το dry path στο Mix Out ευθυγραμμίζεται με την integer καθυστέρηση.

**ΠΡΟΣΟΧΗ:** Το συγκεκριμένο module δεν δηλώνει PDC latency στον host, άρα με εξωτερικά parallel DAW tracks μπορεί να προκύψουν phase shifts. Οι αλλαγές στο oversampling factor κατά το playback μπορεί να προκαλέσουν click, επειδή γίνεται reset των φίλτρων. Προτίμησε αλλαγές με σταματημένη αναπαραγωγή. Επίσης η ακριβής ομαδική καθυστέρηση και η απόκριση στις υψηλές συχνότητες εξαρτώνται από τη σχεδίαση των FIR.

## GitHub Actions / Build

Ανέβασε τα αρχεία **απευθείας στο root του repository**, μαζί με τον κρυφό φάκελο `.github/`.
GitHub → Actions → **Build SynthEdit Waveshaper (Windows + macOS)** → Run workflow.
Artifacts: Windows `.sem` και macOS `.sem` bundle. Windows builds χρειάζονται MSVC, macOS AppleClang.

Δεν έχει δοκιμαστεί compile σε πραγματικό SDK3 environment ή λειτουργία μέσα στο SynthEdit σε αυτή τη συνεδρία. Τα ανεξάρτητα DSP C++ tests έχουν εκτελεστεί επιτυχώς.
