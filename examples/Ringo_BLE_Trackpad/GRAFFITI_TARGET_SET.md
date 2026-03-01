# Graffiti Target Set (Ringo)

This file is the stroke-source reference for the firmware recognizer.

## Source references
- Palm handbook (Graffiti alphabet section): https://manualzilla.com/doc/5796862/handbook
- Wikimedia chart image mirroring the Palm gestures: https://commons.wikimedia.org/wiki/File:Palm_Graffiti_gestures.png

## Encoding convention
- Single-stroke letters only.
- Stroke is represented as directional segments in normalized screen coordinates (`+X` right, `+Y` down).
- Segment tokens: `U`, `D`, `L`, `R`, `UL`, `UR`, `DL`, `DR`.

## Phase 1: Active symbols in firmware

These are the symbols currently implemented in `GraffitiRecognizer.cpp`.

| Symbol | Graffiti intent | Segment sequence |
| --- | --- | --- |
| `a` | caret-like A (`^`) | `UR, DR` |
| `i` | vertical stroke | `D` |
| `l` | down then right | `D, R` |
| `n` | N unistroke | `U, DR, U` |
| `o` | closed loop O | `R, D, L, U` (continuous curve) |
| `u` | U shape | `D, DR, UR, U` |
| `v` | V shape | `DR, UR` |
| `z` | Z zig-zag | `R, DL, R` |
| `\b` | Backspace gesture | `L` |
| `SPACE` | Space gesture | `R` |

## Next expansion

- Add full Graffiti-1 letter/digit/punctuation set to this table first.
- Keep this file as the canonical shape spec.
- Keep template anchors in code aligned to this file (not vice versa).
