# Pandocrator Waveshaper Toolkit v3.2 — 40 DSP Algorithms, 16x Oversampling

Βασισμένο στο cross-platform SynthEdit SDK3 project που έστειλες. Διατηρεί το plugin ID `Pandocrator Waveshaper Toolkit`, τα αρχικά pins με την ίδια σειρά και προσθέτει **Oversampling** πριν από τα δύο outputs. Το module ελέγχεται εξολοκλήρου από pins και δεν έχει δικό του GUI.

## v3.3: Τιμές controls (σημαντικό)

**Διορθώθηκε η λανθασμένη υπόθεση της v3.2**: Δεν υπάρχει γενικός κανόνας
ότι όλες οι έξοδοι των control widgets στο SynthEdit είναι 10 φορές μεγαλύτερες.
Το κάθε `rate="audio"` pin εδώ λαμβάνει τον αριθμό του audio δείγματος
**απευθείας**. Το `Algorithm` αντιστοιχεί σε δείγμα 0..39, το `Drive` σε
0..32, τα `A/B` σε -8..8. Π.χ. `Algorithm=12` σημαίνει 12 στο input pin.

**ΑΝ το εξωτερικό knob σου δείχνει 0..4000**, πρέπει να ρυθμίσεις το εύρος
του ίδιου του knob/slider ή τη μετατροπή της πηγής του πριν το pin. Το module
**δεν μπορεί** να επιβάλλει την κλίμακα της οθόνης του εξωτερικού widget.
Μια μονάδα 1.0 που φτάνει στο pin Algorithm σημαίνει επιλογή αλγορίθμου 1,
και όχι 10 ή 100. Το module περιορίζει τη λειτουργία του στο 0..39,
ακόμη κι αν το knob εμφανίζει μεγαλύτερο αριθμό.

Η κλίμακα audio path παραμένει *ανεξάρτητη*, όπως στο αρχικό project:
`Signal In / 10` για DSP και `Output * 10` στην έξοδο. Αυτή είναι
συμβατική παραδοχή του συγκεκριμένου plugin· πρέπει να επιβεβαιωθεί με
τον τύπο signal/voltage των πηγών στο SynthEdit project σου.

**Ενδεικτικά εύρη πηγών control pins:**

| Pin | Αριθμητική τιμή που πρέπει να φτάνει στο pin |
|---|---|
| Algorithm | 0..39, βήμα 1 |
| Drive | 0..32 |
| A, B | -8..8 |
| Bias | -4..4 |
| Wet | 0..1 |
| Output Gain | 0..8 |
| DC Block | 0 ή 1 |
| Oversampling | 0..4, βήμα 1 |

Εάν βλέπεις 0..4000 στον έλεγχο GUI, αυτό **δεν σημαίνει** ότι ο
αλγόριθμος έχει 4000 modes. Χρειαζόμαστε screenshot του widget και
του module pin connection για να διορθώσουμε την ένδειξη του UI.

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


## v3.4: Pin scaling based on the working AllPassDiffuser v9

Στο λειτουργικό AllPassDiffuser v9: XML default 0.8 -> C++ raw*10 -> Stages 8.
Το Waveshaper εφαρμόζει πλέον την ίδια σύμβαση για ΟΛΑ τα control pins.
Π.χ. Drive XML default 0.1 -> DSP Drive 1.0; Wet XML 0.1 -> DSP Wet 1.0.
To Signal In και οι έξοδοι audio παραμένουν αμετάβλητες σε σχέση με v3.3.

Σημείωση: Αυτή η διόρθωση αναπαράγει τη μετατροπή του αναφορικού module,
αλλά η οπτική κλίμακα ενός εξωτερικού knob εξαρτάται και από τις ρυθμίσεις του.
Για Algorithm χρησιμοποίησε displayed numeric 0..39, και για Oversampling 0..4.
Δεν έχει γίνει compilation σε native SynthEdit runner.
