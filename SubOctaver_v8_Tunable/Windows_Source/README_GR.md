# Sub Octaver -1 για SynthEdit SDK3

Το module δημιουργεί subharmonic **μία οκτάβα κάτω**, δηλαδή συχνότητα:

`f_sub = f_input / 2`

Τεχνικά δεν λέγεται «-1 αρμονική». Είναι **divide-by-two subharmonic** ή
**octave-down**.

## Τι περιέχει

- `SubOctaver.cpp` — DSP και registration του module.
- `SubOctaver.xml` — pins και metadata.
- `SubOctaver.rc` / `resource.h` — ενσωμάτωση του XML μέσα στο `.sem`.
- `CMakeLists.txt` — build project για Visual Studio/CMake.
- `LICENSE.txt` — άδεια χρήσης του συγκεκριμένου source code.

## Pins

1. **Signal In** — μονοφωνικό audio input.
2. **Dry Level** — ποσότητα αρχικού ήχου, 0 έως 1.
3. **Sub Level** — ποσότητα sub στο Mix Out, 0 έως 1.
4. **Tone** — χαμηλά = πιο βαθύ/στρογγυλό, ψηλά = πιο φωτεινό.
5. **Tracking** — χαμηλά για bass, ψηλότερα για υψηλότερες νότες.
6. **Sensitivity** — ψηλότερα = πιάνει πιο αδύναμα σήματα.
7. **Sub Out** — μόνο η octave-down συνιστώσα.
8. **Mix Out** — Dry + Sub.

Αρχικές ρυθμίσεις:

- Dry Level: `0.0`
- Sub Level: `0.75`
- Tone: `0.45`
- Tracking: `0.50`
- Sensitivity: `0.65`

## Build σε Windows / Visual Studio 2022

### Προαπαιτούμενα

Εγκατάστησε:

- Visual Studio 2022 Community
- workload **Desktop development with C++**
- CMake
- Git for Windows
- SynthEdit

### Αυτόματο κατέβασμα του SDK

Άνοιξε Command Prompt ή PowerShell μέσα στον φάκελο και τρέξε:

```powershell
cmake -S . -B build -A x64
cmake --build build --config Release
```

Το CMake θα κατεβάσει το επίσημο `SynthEdit_SDK` από το GitHub.

Το αποτέλεσμα συνήθως θα βρίσκεται εδώ:

```text
build\Release\SubOctaver.sem
```

### Build με SDK που έχεις ήδη

```powershell
cmake -S . -B build -A x64 -DSE_SDK_ROOT="C:\SynthEdit_SDK"
cmake --build build --config Release
```

Το `SE_SDK_ROOT` πρέπει να δείχνει στον φάκελο που περιέχει τον υποφάκελο
`se_sdk3`.

## Εγκατάσταση στο SynthEdit

Αντέγραψε το:

```text
SubOctaver.sem
```

στο:

```text
C:\Program Files\Common Files\SynthEdit\modules\community_modules\
```

Μπορεί να χρειαστεί άδεια Administrator. Μετά κλείσε και ξανάνοιξε το
SynthEdit. Θα εμφανιστεί στην κατηγορία:

```text
Pandocrator DSP
```

με όνομα:

```text
Sub Octaver -1
```

## Πώς λειτουργεί

1. Αφαιρεί DC από το input detector.
2. Φιλτράρει τις ψηλές αρμονικές.
3. Με Schmitt trigger αναγνωρίζει κάθε νέο θετικό κύκλο.
4. Ένα flip-flop αλλάζει πρόσημο μία φορά ανά κύκλο εισόδου.
5. Επειδή χρειάζονται δύο input cycles για να ολοκληρωθεί ένας output cycle,
   το αποτέλεσμα είναι μία οκτάβα κάτω.
6. Δύο low-pass στάδια στρογγυλεύουν το square sub.
7. Envelope follower και gate μειώνουν θόρυβο και άσκοπα flips στη σιωπή.

## Σημαντικοί περιορισμοί

- Είναι **μονοφωνικός octaver**. Δουλεύει καλύτερα με μία νότα τη φορά.
- Chords, πολύπλοκα drums, reverb, chorus και δυνατό distortion πριν από το
  detector μπορούν να κάνουν το tracking ασταθές.
- Για synth βάλε τον octaver όσο γίνεται νωρίτερα στην αλυσίδα, πριν από
  chorus/reverb και συνήθως πριν από βαρύ distortion.
- Για stereo source, είναι συνήθως καλύτερο να φτιάξεις mono detector και να
  κρατήσεις το sub στο κέντρο.
- Το `Mix Out` δεν έχει limiter. Χαμήλωσε Dry/Sub αν ξεπερνά το επιθυμητό
  επίπεδο.

## Ρύθμιση tracking

Αν ακούς διπλά ή ασταθή flips:

1. Μείωσε το **Tracking**.
2. Μείωσε λίγο το **Sensitivity**.
3. Δώσε καθαρότερο, μονοφωνικό input.
4. Βάλε low-pass πριν από το module.

Αν δεν πιάνει τη νότα:

1. Αύξησε το **Sensitivity**.
2. Αύξησε το **Tracking** για ψηλότερες νότες.
3. Δώσε δυνατότερο input.

---

## v2 Release Safe

Η v2 κρατά το ίδιο module ID και τα ίδια pins/defaults, αλλά προσθέτει:

- προστασία από NaN / Infinity / ακραίες στάθμες,
- αυτόματη ανάκαμψη invalid DSP state,
- denormal cleanup,
- gate hysteresis για λιγότερο chatter,
- πιο ομαλό gate 8 ms attack / 120 ms release,
- ασφαλές polarity reset μόνο όταν το sub έχει ουσιαστικά σβήσει,
- validation sample rate,
- coefficient caching,
- αφαίρεση `/fp:fast` και `/GS-` από το Windows build.

Το module ID παραμένει `Pandocrator Sub Octaver`, ώστε να λειτουργεί ως drop-in
replacement της αρχικής έκδοσης.
