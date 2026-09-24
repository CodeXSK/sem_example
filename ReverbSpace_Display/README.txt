REVERB SPACE DISPLAY - SynthEdit 1.5 GUI module
================================================

Purpose
-------
Visual-only reverb display. It does NOT process audio and it does NOT implement a reverb.
The graphic reacts in real time to five normalized float controls:

  Size       0..1
  Diffusion  0..1
  Decay      0..1
  Low Damp   0..1
  High Damp  0..1

Visual mapping
--------------
Size       : widens the room/envelope and reflected field.
Diffusion  : increases reflection density and micro-structure.
Decay      : raises/extends the reverb envelope and reflections.
Low Damp   : removes energy from the water/low-frequency reflection region.
High Damp  : darkens and shortens the upper/high-frequency reflection region.

The drawing is procedural. No image files are required.

Important about "GUI only"
--------------------------
This version is a true GUI-only SEM: the five pins are GUI/sub-control pins.
Use it with SynthEdit GUI/sub-control signals.

If you specifically need five ordinary STRUCTURE-side voltage pins that can be wired
from DSP/control modules in the structure view, that requires a tiny non-audio bridge
section (Audio/Processor pins -> Parameters -> GUI). It still would not process audio;
it would only copy the five values to this display.

Build - easiest method
----------------------
1. In SynthEdit 1.5 create a GUI code skeleton (or use your existing Jeff sem_example setup).
2. Name the project ReverbSpace.
3. Replace the generated GUI source/XML with:
     ReverbSpaceGui.cpp
     ReverbSpaceGui.h
     ReverbSpace.xml
4. If using sem_example, include this folder from the parent CMake project and use the
   included CMakeLists.txt.
5. Configure/build Release. With SE_LOCAL_BUILD enabled the .sem should be copied to
   SynthEdit's community_modules folder by the template.
6. Rescan modules in SynthEdit.

Notes
-----
- Values are clamped internally to 0..1 even if an upstream control overshoots.
- Redraw occurs only when a pin changes, so idle CPU usage stays very low.
- The default requested size is 1000 x 260, but the drawing scales to the actual panel size.
- Source uses GmpiGui / MpGuiGfxBase and the cross-platform SynthEdit drawing API.
