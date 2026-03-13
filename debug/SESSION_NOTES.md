# Ringo BLE AirMouse Session Notes

## Current firmware
- Sketch: `examples/Ringo_BLE_Trackpad/Ringo_BLE_Trackpad.ino`
- Port: `/dev/cu.usbmodem1101`
- BLE name: `Ringo`

## Input model
- Cursor: IMU gyro + gravity-based tilt compensation.
- Touch:
  - `Single Click` = left click
  - `Single Click` then `Long Press` (within arm window) = lock left button (drag lock)
  - While locked: `Single Click` = unlock left button
  - `Long Press` (not armed, not locked) = right click
  - `Swipe Up/Down` = wheel

## Tuning state
- High gain + acceleration enabled:
  - `kAirMouseYawGain = 3.80`
  - `kAirMousePitchGain = 3.10`
  - `kRateAccelRefDps = 60`
  - `kRateAccelMaxBoost = 5.0`
- Axis sign:
  - `kYawSign = -1`
  - `kPitchSign = 1`

## New changes in this pass
- Display path switched to line-level dirty updates with fixed-width text + background erase to reduce blur/flicker.
- Added roll-rate damping/suppression during fast roll motion:
  - starts damping above `kRollRateDampStartDps = 45`
  - near full suppression by `kRollRateSuppressDps = 140`

## Follow-ups
- Verify display quality after this pass.
- Verify roll suppression feel during aggressive roll.
- Further tune gains/deadzone against user target: near full-screen traversal with small wrist twitch.

## Latest pass (post-display blur report)
- Display text path changed from rectangle clears to fixed-width text redraw using `setTextColor(fg,bg)` and padded strings.
- Added IMU roll damping counterweight: strong roll still damps cursor, but high yaw/pitch activity partially restores translation.
- Added singularity/sign-flip mitigation for pitch axis basis: choose best reference cross-product vs gravity and enforce continuity with previous right-axis hemisphere.
- Click handling moved to raw touch press/release state machine for reliability:
  - short tap release = single click
  - tap-arm + hold = left-lock
  - while left-locked: tap release = unlock
  - hold without tap-arm = right-click
  - release-time swipe fallback sends wheel up/down
- `handleGestureIfAny()` now only applies CST swipe gestures; click/hold logic no longer depends on CST click gesture decoding.

## Recovery + new branch (2026-02-28 early AM)
- Confirmed IMU regression came from singularity continuity + roll counterweight patch.
- Reverted IMU basis/counterweight to pre-regression behavior in commit `73065e9`.
- New working branch: `feature/ringo-touch-click-drag` (commit `6e4ef2d`).
- Touch changes on this branch:
  - Added periodic touch polling (`kTouchPollMs=12`) so press/release is not interrupt-edge dependent.
  - Drag-hold semantics:
    - tap then hold (within arm window) => left button down (`DRAG`)
    - finger release => left button up (`Drag Release`)
  - Preserved right-click long press path.
  - Added per-touch swipe gating to avoid duplicate wheel emits from both gesture and release fallback.

## 2026-03-02: Build + CDC verification (post-compaction)
- Verified clean rebuild includes `GraffitiRecognizer.cpp`:
  - `arduino-cli compile --clean -v --build-path .build --fqbn esp32:esp32:esp32c6 --libraries libraries examples/Ringo_BLE_Trackpad`
  - Log: `/tmp/ringo_compile_verbose_fresh.log`
  - Evidence lines include:
    - `Compiling sketch...`
    - `.build/sketch/GraffitiRecognizer.cpp -o .build/sketch/GraffitiRecognizer.cpp.o`
    - Link step includes `GraffitiRecognizer.cpp.o`
- CDC behavior root cause:
  - Board option default compiles with `-DARDUINO_USB_CDC_ON_BOOT=0` for `esp32:esp32:esp32c6`.
  - The sketch has a fallback `#define ARDUINO_USB_CDC_ON_BOOT 1`, but that only applies when the macro is not already defined by build flags.
  - Therefore, CDC can appear unavailable/inconsistent unless board option is explicitly enabled.
- Confirmed no recent in-repo change flipped CDC setting:
  - `#ifndef/#define ARDUINO_USB_CDC_ON_BOOT` block exists since commit `465cc81`.
  - Recent recognizer work did not modify CDC flags.
- Current observed USB serial port during this check: `/dev/cu.usbmodem101`.
- For reliable command I/O in Arduino IDE:
  - Tools -> USB CDC On Boot -> Enabled
  - 115200 baud, Newline (or Both NL & CR)

## 2026-03-02: Graffiti rejection hotfix pass
- User report after clean flash: recognizer accepted only a few glyphs (`N/U/V/L`) and occasionally wrong digit (`5`), rejecting most strokes.
- Changes applied in `GraffitiRecognizer.cpp/.h`:
  - Added direction-run smoothing in token extraction:
    - accumulated vector segments before quantization
    - absorbed tiny near-collinear turns
    - collapsed short `A-B-A` jitter bounces
  - Relaxed trie scoring strictness:
    - `kDirectionRejectCutoff` 1.18 -> 1.45
    - reduced start/end/length penalties
    - substitution step factor 0.52 -> 0.42
    - minimum stroke diagonal 8.0 -> 6.5 px
  - Added slight class bias to reduce accidental digit/control picks in ambiguous cases.
  - Added last token sequence debug string (e.g. `UR-DR`) exposed via API.
- Changes applied in `Ringo_BLE_Trackpad.ino`:
  - `status` / `recog?` now print last token sequence (`seq=`).
  - Graffiti ACCEPT/REJECT logs now include distance + token sequence.
- Build and flash status:
  - Compile OK (`Sketch uses 1245777 bytes`).
  - Flashed to `/dev/cu.usbmodem101` successfully after this hotfix.

## 2026-03-12: Graffiti reboot
- User concluded current graffiti recognizer is not usable and asked to restart from scratch while archiving the existing work.
- Explicit archive branch created:
  - `backup/ringo-graffiti-trie-20260312`
- New working branch:
  - `feature/ringo-graffiti-reboot`
- Added reboot note:
  - `examples/Ringo_BLE_Trackpad/GRAFFITI_REBOOT_PLAN.md`
- Added backend seam wrapper:
  - `examples/Ringo_BLE_Trackpad/GraffitiEngine.h`
  - `examples/Ringo_BLE_Trackpad/GraffitiEngine.cpp`
- Sketch now calls the wrapper instead of talking to `GraffitiRecognizer` directly.
- Current compile on reboot branch:
  - `Sketch uses 1246155 / 1310720 bytes (95%)`
  - `Global variables use 57640 / 327680 bytes (17%)`
- External findings captured in reboot note:
  - public Graffiti alphabet references are available
  - no clear public source-level Palm recognizer reimplementation found in this pass
  - TinyML is possible in principle on ESP32-C6 but is flash-constrained under the current firmware footprint

## 2026-03-13: Live capture console
- User requested an on-computer capture console with `capture`, `stop`, and `continuous` modes, realtime stroke display, labeling, and persistent storage.
- Added firmware stroke trace protocol in `examples/Ringo_BLE_Trackpad/Ringo_BLE_Trackpad.ino`:
  - UART commands:
    - `trace?`
    - `trace off`
    - `trace once`
    - `trace cont`
  - Event stream:
    - `[trace] begin ...`
    - `[trace] point ...`
    - `[trace] end ...`
- Added host console tool:
  - `tools/graffiti_capture_console.py`
  - curses UI, live stroke pane, last-stroke metadata, label editing, manual save, autosave, JSONL output.
- Current flashed firmware:
  - reboot branch plus live trace protocol
  - compile footprint: `1250049 / 1310720` bytes flash, `58024 / 327680` bytes RAM
  - flashed to `/dev/cu.usbmodem101`

## 2026-03-13: Flash path correction
- Root cause of the immediate GUI failure was not the GUI itself: the device had not actually been updated to the trace-capable image.
- Important finding:
  - `arduino-cli upload` was unreliable in this session for this board and repeatedly failed with:
    - `Serial data stream stopped: Possible serial noise or corruption.`
- However, the bootloader was reachable without manual intervention:
  - direct probe succeeded:
    - `/Users/noah/Library/Arduino15/packages/esp32/tools/esptool_py/4.9.dev3/esptool --chip esp32c6 --port /dev/cu.usbmodem101 --baud 115200 chip_id`
- Direct flash via `esptool` succeeded and is now the preferred path:
  - helper script added:
    - `tools/flash_ringo_esptool.sh`
  - notes added:
    - `debug/FLASHING_NOTES.md`
- Verified after flash:
  - device answers CDC commands on `/dev/cu.usbmodem101`
  - `ping`, `help`, `trace?`, `mode?`, and `status` all respond
  - active flashed branch/commit on host build:
    - `feature/ringo-graffiti-reboot`
    - `bdb71af`
- Capture GUI relaunched against the now-working firmware:
  - `tools/graffiti_capture_gui.py --port /dev/cu.usbmodem101 --output debug/graffiti_capture/samples.jsonl`
  - GUI launch behavior changed so it no longer starts in `trace off`
  - on launch it now sends:
    - `mode graffiti`
    - `trace cont`
    - `status`
  - practical effect: the visible GUI should immediately start streaming live touch trace data without requiring a button click first

## 2026-03-13: Scripted capture + TinyML export
- `tools/graffiti_capture_gui.py` now supports a scripted collection workflow:
  - default target list: `abcdefghijklmnopqrstuvwxyz`
  - default repetitions per target: `5`
  - `Start Script` prompts one target at a time and auto-advances after each saved stroke
  - scripted capture forces `mode graffiti` + `trace cont`
- Saves now emit both raw and training-friendly outputs:
  - raw/annotated JSONL:
    - `debug/graffiti_capture/samples.jsonl`
  - TinyML JSONL:
    - `debug/graffiti_capture/tinyml_dataset.jsonl`
  - TinyML CSV:
    - `debug/graffiti_capture/tinyml_dataset.csv`
- TinyML feature format:
  - `resampled_xy32_plus_dxy32_v1`
  - each sample includes:
    - 32 normalized XY points
    - 32 normalized delta XY vectors
    - label + scripted session metadata + path length + bounding box

## 2026-03-13: Collector reliability + conditioned trainer
- `tools/graffiti_capture_gui.py` collector flow tightened up:
  - custom target parsing now correctly expands compact mixed sequences such as `abcdefghijklmnopqrstuvwxyz SPACE BKSP . , ... 0123456789`
  - scripted capture supports conditions via prompt presets:
    - `letters`
    - `punct`
    - `numeric`
    - `all`
  - startup/script begin now waits for real firmware acknowledgements instead of treating any serial text as "ready"
  - readiness is based on CDC replies for:
    - `mode graffiti`
    - `trace cont`
    - `status`
  - boot timing was stretched because opening the CDC port resets the board and firmware boot takes materially longer than the original 900 ms timeout
  - startup behavior now:
    - open serial
    - wait for boot settle
    - configure graffiti mode + continuous trace
    - only then enable capture controls / scripted run callbacks
  - `Undo Last` remains in place and rewinds all three dataset outputs:
    - raw JSONL
    - TinyML JSONL
    - TinyML CSV
  - TinyML CSV schema guard now backs up mismatched older CSV files before rewriting a new header
- New host-side conditioned trainer from the training worker:
  - `tools/train_graffiti_model.py`
  - `docs/GRAFFITI_MODEL_TRAINING.md`
- Trainer behavior:
  - single conditioned classifier label space:
    - lowercase `a-z`
    - punctuation/control `SPACE BKSP . , ( ) - _ # * ? '`
    - numerics `0-9`
  - condition input:
    - `letters`
    - `punct`
    - `numeric`
  - prefers TensorFlow/Keras + TFLite export when available
  - otherwise falls back to prototype baseline export
- Current host Python environment still lacks ML packages (`tensorflow`, `numpy`, `scikit-learn`, etc.), so fallback backend is the only path unless that stack is installed later.
- Relaunched updated collector GUI via Terminal after these changes:
  - `python3 tools/graffiti_capture_gui.py --port /dev/cu.usbmodem101 --output debug/graffiti_capture/samples.jsonl`
- Added collector-side reboot recovery in `tools/graffiti_capture_gui.py`:
  - detects ESP32 ROM/boot banners on the open CDC stream
  - treats mid-session reboot as a recovery event instead of random log noise
  - clears the in-flight unsaved stroke, waits for boot, reconfigures `mode graffiti` + `trace cont`, and resumes the active scripted target/rep
  - avoids duplicate startup/recovery configure timers by using a single deferred-config path
- Collector fresh-start workflow updated:
  - default launch now creates a per-attempt session directory under `debug/graffiti_capture/attempt_YYYYMMDD_HHMMSS/`
  - session directory contains `samples.jsonl`, `tinyml_dataset.jsonl`, `tinyml_dataset.csv`, `gui.log`, and `metadata.json`
  - new CLI flags:
    - `--session-dir <dir>` to reuse or target a specific dataset folder
    - `--fresh` to avoid reusing a populated session directory
  - explicit `--output` is still supported for legacy/manual flows, but it disables the automatic per-attempt directory behavior

## 2026-03-14 prototype backend deployment
- Replaced `GraffitiEngine` trie wrapper with a conditioned nearest-prototype backend using exported model data in `examples/Ringo_BLE_Trackpad/GraffitiPrototypeModel.generated.cpp`.
- `tools/train_graffiti_model.py` now auto-selects the latest attempt dataset and exports a firmware artifact for prototype deployments.
- Added `glyph letters|punct|numeric` device commands to control the classifier condition explicitly.
- Lowered `kGraffitiStrokeMinPoints` from `4` to `2` so `.` can be entered as a short stroke.
- Verified compile with `arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc --libraries libraries --output-dir debug/flash_backups/build_model_proto1 examples/Ringo_BLE_Trackpad`.
- Verified flash with `tools/flash_ringo_esptool.sh /dev/cu.usbmodem101 /Users/noah/projects/T-QT-C6/debug/flash_backups/build_model_proto1`.
- Post-flash CDC sanity check returned `recognizer=PROTOTYPE cond=letters`.
