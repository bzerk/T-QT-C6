# T-QT-C6 Flashing Notes

## Current known-good host build

- Branch: `feature/ringo-graffiti-reboot`
- Commit: `bdb71af`
- Build dir: `debug/flash_backups/build_bdb71af`
- Port seen most recently: `/dev/cu.usbmodem101` and `/dev/tty.usbmodem101`

## Build command

```sh
arduino-cli compile --clean \
  --build-path debug/flash_backups/build_bdb71af \
  --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc \
  --libraries libraries \
  examples/Ringo_BLE_Trackpad
```

## Upload command

`arduino-cli upload` has been unreliable on this board in this session even though the same
underlying bootloader is reachable. The stable path is to use `esptool` directly.

```sh
/Users/noah/Library/Arduino15/packages/esp32/tools/esptool_py/4.9.dev3/esptool \
  --chip esp32c6 \
  --port /dev/cu.usbmodem101 \
  --baud 115200 \
  --before default_reset \
  --after hard_reset \
  write_flash -z \
  --flash_mode keep \
  --flash_freq keep \
  --flash_size keep \
  0x0 debug/flash_backups/build_bdb71af/Ringo_BLE_Trackpad.ino.bootloader.bin \
  0x8000 debug/flash_backups/build_bdb71af/Ringo_BLE_Trackpad.ino.partitions.bin \
  0xe000 /Users/noah/Library/Arduino15/packages/esp32/hardware/esp32/3.2.0/tools/partitions/boot_app0.bin \
  0x10000 debug/flash_backups/build_bdb71af/Ringo_BLE_Trackpad.ino.bin
```

Repo helper:

```sh
tools/flash_ringo_esptool.sh /dev/cu.usbmodem101
```

## Recovery path when upload handshake fails

Project docs explicitly say to hold the board's `BOOT-0` button and retry download:

- `README.md:191`
- `README.md:276`

Operationally:

1. Hold `BOOT`.
2. Tap `RESET`.
3. Keep holding `BOOT` for about 1-2 seconds.
4. Start or retry the upload while the board is in ROM download mode.

## Post-flash verification

If CDC is enabled in the image, these should answer at `115200` baud:

```text
ping
help
trace?
status
```

## Observed behavior

- `esptool --chip esp32c6 --port /dev/cu.usbmodem101 chip_id` succeeds without manual intervention.
- Direct `esptool write_flash` succeeds cleanly.
- `arduino-cli upload` intermittently failed here with:
  - `Serial data stream stopped: Possible serial noise or corruption.`
- Practical conclusion: use the direct `esptool` path above for this device unless/until the wrapper failure is root-caused.
