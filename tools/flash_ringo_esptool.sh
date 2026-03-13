#!/bin/zsh
set -euo pipefail
ROOT="/Users/noah/projects/T-QT-C6"
PORT="${1:-/dev/cu.usbmodem101}"
BUILD_DIR="${2:-$ROOT/debug/flash_backups/build_bdb71af}"
ESPTOOL="/Users/noah/Library/Arduino15/packages/esp32/tools/esptool_py/4.9.dev3/esptool"
BOOT_APP0="/Users/noah/Library/Arduino15/packages/esp32/hardware/esp32/3.2.0/tools/partitions/boot_app0.bin"
cd "$ROOT"
if [[ ! -f "$BUILD_DIR/Ringo_BLE_Trackpad.ino.bin" ]]; then
  echo "build artifact not found in $BUILD_DIR" >&2
  exit 1
fi
"$ESPTOOL" --chip esp32c6 --port "$PORT" --baud 115200 --before default_reset --after hard_reset write_flash -z \
  --flash_mode keep --flash_freq keep --flash_size keep \
  0x0 "$BUILD_DIR/Ringo_BLE_Trackpad.ino.bootloader.bin" \
  0x8000 "$BUILD_DIR/Ringo_BLE_Trackpad.ino.partitions.bin" \
  0xe000 "$BOOT_APP0" \
  0x10000 "$BUILD_DIR/Ringo_BLE_Trackpad.ino.bin"
