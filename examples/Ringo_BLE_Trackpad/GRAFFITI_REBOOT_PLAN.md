# Graffiti Reboot Plan

This note freezes the current state of the Ringo graffiti work and defines the
restart path.

## Archive point

- Heuristic/trie recognizer archive branch:
  - `backup/ringo-graffiti-trie-20260312`
- Reboot branch:
  - `feature/ringo-graffiti-reboot`

## Verdict

The current recognizer should be treated as archived, not iterated.

Why:
- It has accumulated multiple overlapping heuristics and still fails the most
  basic product test: reliable everyday input.
- The current backend mixes token rules, template matching, ambiguity hacks,
  and runtime thresholds without a clean data/measurement loop.
- More tuning is unlikely to cross the gap from "sometimes works" to
  "production usable."

## Hard constraints

- Current sketch size, measured on this branch:
  - `1246155 / 1310720` bytes flash used (`95%`)
  - `57640 / 327680` bytes global RAM used (`17%`)
- Board build config in this repo does not currently enable PSRAM.
- The current firmware already carries BLE HID, display UI, IMU logic, touch
  logic, PMIC init, and diagnostics in a single Arduino sketch.

Consequence:
- A TinyML recognizer is not ruled out, but under the current Arduino build and
  partition budget it is immediately flash-constrained.
- Any ML path must be justified by data, and may require first shrinking the
  firmware or moving to a leaner integration model.

## External findings

### Palm Graffiti

- The public Palm Graffiti references are clear on the alphabet itself:
  - Palm handbook: https://manualzilla.com/doc/5796862/handbook
  - Gesture chart: https://commons.wikimedia.org/wiki/File:Palm_Graffiti_gestures.png
- Graffiti's success came from a deliberately differentiable, single-stroke
  alphabet, not from generic handwriting recognition.

### Reverse engineering

- I did not find a clean public source-level reimplementation of Palm's shipped
  recognizer during this pass.
- That does not mean it is impossible. It means the likely path is binary
  reverse engineering from Palm ROM images or libraries, which is a larger task
  than using the public alphabet spec and building a recognizer around the
  actual Ringo touch data.
- If we later want to inspect Palm binaries, that should be a separate
  workstream with ROM acquisition and Ghidra analysis.

### TinyML on ESP32-C6

- Espressif maintains a TensorFlow Lite Micro port:
  - https://github.com/espressif/esp-tflite-micro
- The ESP32-C6 product documentation lists `320 KB ROM` and `512 KB HP SRAM`:
  - https://www.espressif.com/en/products/socs/esp32-c6

Consequence:
- TinyML is feasible in principle for a very small model.
- TinyML is not the first thing to do here, because we do not yet have the
  training data or the firmware headroom to justify it.

## What to keep

Keep these parts intact until a new backend is proven:

- Stroke capture and release timing in
  - `sampleGraffitiTouchState()`
  - `sampleGraffitiExitTapOnly()`
- Mode switching and inverted command pose handling in
  - `updateGraffitiTapOnlyMode()`
  - `setInputMode()`
  - `toggleInputMode()`
- HID keyboard output in
  - `mapSymbolToKeyboardUsage()`
  - `sendKeyboardSymbol()`
  - `applyGraffitiSymbol()`

## What to replace

- Replace `GraffitiRecognizer.h` and `GraffitiRecognizer.cpp` wholesale.
- Replace the classifier-specific portion of `finalizeGraffitiStroke()` with a
  backend-neutral call boundary.

Recommended boundary:

```cpp
GraffitiResult result = g_graffitiEngine.classify(points, count);
```

That boundary should be the only place where the sketch knows anything about
the recognizer backend.

## Restart plan

### Phase 1: Clean seam

- Initial step already done on this branch:
  - `GraffitiEngine.h/.cpp` now wraps the old recognizer behind a backend-neutral
    boundary so the next backend can be swapped with less sketch churn.
- Make `finalizeGraffitiStroke()` a thin adapter.
- Define a backend-neutral `GraffitiResult`:
  - `symbol`
  - `accepted`
  - `confidence`
  - optional `debug_label`
- Remove old engine-specific fields from the public API.

### Phase 2: Data collection

- Add a capture mode that logs raw strokes plus intended labels over USB CDC.
- Collect samples from multiple users, not just one operator.
- Record:
  - raw points
  - per-point timestamps
  - device orientation state
  - label
  - accept/reject outcome

Without this, an ML path is premature and a classical path cannot be tuned
honestly.

### Phase 3: Strong non-ML baseline

Build a recognizer specifically for Graffiti on a tiny touch surface:

- No rotation invariance.
- No world-orientation invariance.
- Input normalized to the device frame only.
- Feature candidates:
  - resampled delta vectors
  - cumulative turn angles
  - start and end direction
  - aspect ratio
  - corner count
  - stroke length profile
- Candidate algorithms:
  - Rubine-style linear classifier
  - DTW over delta-angle sequences
  - Protractor-like vector matcher with orientation locked

This gives us a small, explainable, low-latency baseline that can often beat a
badly-trained neural net on structured unistrokes.

## TinyML gate

Only move to TinyML if the classical baseline fails after real data collection.

If that happens:
- Train offline on fixed-length sequence features, not raw arbitrary-length
  point clouds.
- Start with a very small model sized for microcontroller inference.
- Keep the model vocabulary narrow at first:
  - letters only
  - then controls
  - then digits
- Measure:
  - flash growth
  - inference latency
  - BLE/input loop impact
  - accuracy on held-out users

## Success criteria

- Median recognition latency: low enough to feel immediate on-device.
- Single-user accuracy: high enough to type without fighting the device.
- Cross-user accuracy: materially robust without per-user retraining.
- False positives: low enough that command-mode gestures and normal use do not
  interfere.

## Next implementation move

The next code change should not be another recognizer tweak.

The next code change should be:
- isolate the recognizer seam
- add capture/export infrastructure
- then build the first clean baseline backend against recorded data
