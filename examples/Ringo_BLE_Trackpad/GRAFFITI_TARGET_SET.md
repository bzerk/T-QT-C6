# Graffiti Target Set (Ringo)

This file is the stroke-source reference for the firmware recognizer.

## Source references
- Palm handbook (Graffiti alphabet section): https://manualzilla.com/doc/5796862/handbook
- Wikimedia chart image mirroring the Palm gestures: https://commons.wikimedia.org/wiki/File:Palm_Graffiti_gestures.png

## Encoding convention
- Single-stroke letters only.
- Stroke is represented as directional segments in normalized screen coordinates (`+X` right, `+Y` down).
- Segment tokens: `U`, `D`, `L`, `R`, `UL`, `UR`, `DL`, `DR`.

## Phase 2: Trie Token Set (current firmware)

These token sequences back the trie+beam decoder in `GraffitiRecognizer.cpp`.
They are intentionally simple, hand-tunable direction signatures from the
Graffiti chart and are expected to be iteratively tuned with device testing.

### Letters

| Symbol | Segment sequence |
| --- | --- |
| `a` | `UR, DR` |
| `b` | `D, UR, DR, DL` |
| `c` | `UR, UL, DL, DR` |
| `d` | `D, UR, DR, DL, U` |
| `e` | `R, DL, R` |
| `f` | `R, D` |
| `g` | `UR, UL, DL, DR, R` |
| `h` | `D, U, D` |
| `i` | `D` |
| `j` | `D, DL` |
| `k` | `D, UR, DL, UR` |
| `l` | `D, R` |
| `m` | `D, U, D, U, D` |
| `n` | `U, DR, U` |
| `o` | `DR, DL, UL, UR` |
| `p` | `D, UR, DR, D` |
| `q` | `DR, DL, UL, UR, D` |
| `r` | `D, UR, DR` |
| `s` | `R, DL, L, DR, R` |
| `t` | `D, R` |
| `u` | `D, DR, UR, U` |
| `v` | `DR, UR` |
| `w` | `DR, UR, DR, UR` |
| `x` | `DR, UL, DR` |
| `y` | `D, DR, UR, D` |
| `z` | `R, DL, R` |

### Digits

| Symbol | Segment sequence |
| --- | --- |
| `0` | `DR, DL, UL, UR` |
| `1` | `D` |
| `2` | `R, DL, R` |
| `3` | `R, DL, R, DL, R` |
| `4` | `D, R, U` |
| `5` | `R, D, L, R` |
| `6` | `DL, D, R, U, L` |
| `7` | `R, DL` |
| `8` | `DR, UR, DL, UR, DR` |
| `9` | `DR, DL, UL, UR, D` |

### Controls

| Symbol | Segment sequence |
| --- | --- |
| `SPACE` | `R` |
| `RET` | `DL` |
| `BKSP` | `L` |

## Notes

- Some symbols are intentionally near each other in token space (e.g. `z` and
  `2`), and are disambiguated by trie edit cost + fallback point matcher.
- Keep this file as the source of truth for token edits.
