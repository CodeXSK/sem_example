StrongestHarmonicEQ — Release Safe v4
DROP-IN FOR JeffMcClintock/sem_example

1. Put this entire StrongestHarmonicEQ folder beside Crusher in your fork.
2. In the repository ROOT CMakeLists.txt add:
       add_subdirectory(StrongestHarmonicEQ)
3. Commit and Push.
4. GitHub Actions builds the macOS SEM artifact.

Important:
- Keep this CMakeLists.txt. It deliberately overrides `/fp:fast /GS-` on the
  StrongestHarmonicEQ target because the DSP contains NaN/Inf containment.
- Plugin ID and pin order are unchanged, so this is intended as a replacement
  for the previous Strongest Harmonic EQ SEM.

v4:
- Peak Gain -24..+24 dB
- TPT moving peak EQ
- DC-safe analyzer
- bit-level NaN/Inf checks
- subnormal containment
- catastrophe output guard
- sample-level parameter smoothing
