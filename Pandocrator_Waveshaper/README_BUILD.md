# Pandocrator Waveshaper — GitHub Actions Build

Upload **the contents of this folder** to the repository root (keep `.github/workflows/build.yml`).
Go to **Actions → Build SynthEdit Waveshaper (Windows + macOS) → Run workflow**.
After both jobs finish, download the corresponding artifacts.

## What changed
- macOS: CMake no longer requires Windows RC compiler.
- macOS: `.xml` packaged in `.sem/Contents/Resources/` bundle.
- Windows: `.rc`/`resource.h` compiled only on Windows.
- Automated cross-platform builds and artifact uploads.

## Notes
The source has not been validated by running full GitHub builds. A subsequent compiler/API error may still need a fix. `Node.js 20 is deprecated` is a runner warning, not itself a C++ compilation error. If a job fails, copy its **first `error:`** message and surrounding lines.
