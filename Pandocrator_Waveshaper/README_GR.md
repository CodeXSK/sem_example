# Pandocrator Waveshaper Toolkit v3.2 — 40 DSP Algorithms, 16x Oversampling

Βασισμένο στο cross-platform SynthEdit SDK3 project που έστειλες. Διατηρεί το plugin ID `Pandocrator Waveshaper Toolkit`, τα αρχικά pins με την ίδια σειρά και προσθέτει **Oversampling** πριν από τα δύο outputs. Το module ελέγχεται εξολοκλήρου από pins και δεν έχει δικό του GUI.

## ΣΗΜΑΝΤΙΚΟ: SynthEdit control scaling (διορθώθηκε στη v3.2)

Το προηγούμενο module διάβαζε τα control audio pins απευθείας. Έτσι αν το
control του SynthEdit απέδιδε 10 μονάδες για κάθε 1 ορατή μονάδα παραμέτρου,
Drive 32 εμφανιζόταν/συμπεριφερόταν ως 320 και Algorithm 39 ως 390.

**Στη v3.2:** όλα τα control pins διαβάζονται ως `raw / 10`:
Algorithm, Drive, A, B, Bias, Wet, Output Gain, DC Block, Oversampling.
Το XML έχει προσαρμοσμένες default τιμές: raw 10 σημαίνει πραγματική τιμή 1.

Παραδείγματα: εμφανιζόμενο Drive 2 -> raw 20 -> DSP Drive 2,
Algorithm 39 -> raw 390 -> DSP Algorithm 39,
Wet 0.5 -> raw 5 -> DSP Wet 0.5, Oversampling 4 -> raw 40 -> DSP x16.

Τα **Signal In, Wet Out, Mix Out** παραμένουν σε audio volts, με
Signal In / 10 στο DSP και έξοδο DSP * 10 όπως πριν.

Αν δίνεις *απευθείας* αριθμητικές τιμές σε κάποιο audio pin χωρίς το
SynthEdit knob/control scaling, πρέπει να τις μετατρέψεις σε 10× πριν
το pin (π.χ. raw 20 για Drive 2). Το scaling αυτό είναι μία πρακτική
συμβατότητας με το setup των controls σου, όχι υποχρεωτικό χαρακτηριστικό
όλων των πηγών σημάτων του SynthEdit.

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
