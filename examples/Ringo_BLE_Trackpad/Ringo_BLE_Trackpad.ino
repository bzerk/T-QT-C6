#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 1
#endif
#include <Arduino.h>
#include <algorithm>
#include <esp_system.h>
#include <math.h>

#include "Arduino_DriveBus_Library.h"
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "GraffitiRecognizer.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEHIDDevice.h>
#include <BLEServer.h>
#include <BLESecurity.h>

namespace {
constexpr char kDeviceName[] = "Ringo";
constexpr uint8_t kMouseReportId = 0x01;
constexpr uint8_t kKeyboardReportId = 0x02;

constexpr uint32_t kDisplayRefreshMs = 140;
constexpr uint32_t kTouchReleaseTimeoutMs = 120;
constexpr uint32_t kTouchPollMs = 12;
constexpr uint32_t kGraffitiTouchPollMs = 12;
constexpr uint32_t kImuPollMs = 8;
constexpr int32_t kI2cBusHz = 100000;
constexpr uint32_t kGraffitiImuPollMs = 35;
constexpr uint32_t kImuDebugIntervalMs = 500;
constexpr uint16_t kGraffitiStrokeMaxPoints = 180;
constexpr uint16_t kGraffitiStrokeMinPoints = 4;
constexpr uint32_t kGraffitiStrokeMaxDurationMs = 2200;
constexpr int32_t kGraffitiMinPointDistanceSq = 1;
constexpr uint32_t kGraffitiTapMaxDurationMs = 520;
constexpr int16_t kGraffitiTapMoveThresholdPx = 42;
constexpr uint16_t kGraffitiTapMaxStrokePoints = 6;
constexpr int16_t kModeExitTapSeparationPx = 42;
constexpr uint32_t kModeExitDoubleTapWindowMs = 800;
constexpr uint8_t kGraffitiInvertAxis = 2;  // Device Z axis
constexpr float kGraffitiInvertThreshold = -0.20f;
constexpr float kGraffitiInvertExitThreshold = -0.12f;
constexpr uint32_t kGraffitiExitSwipeMaxDurationMs = 900;
constexpr int16_t kGraffitiExitSwipeMinDyPx = 24;
constexpr int16_t kGraffitiExitSwipeMaxDxPx = 40;
constexpr uint32_t kGraffitiIdleTouchPollMs = 40;
constexpr uint32_t kGraffitiReleaseHoldMs = 180;
constexpr int16_t kCstIdleX = 60;
constexpr int16_t kCstIdleY = 150;
constexpr bool kTouchGestureYInverted = true;
constexpr uint8_t kSerialCmdMaxLen = 64;
constexpr uint32_t kSerialBootWaitMs = 250;
constexpr uint8_t kSetupShiftConnectAttempts = 4;
constexpr uint32_t kSetupShiftInitialDelayMs = 450;
constexpr uint32_t kSetupShiftRetryMs = 320;

constexpr uint16_t kImuGyroCalibrationSamples = 160;
constexpr float kGyroDeadzoneDps = 4.0f;
constexpr float kAirMouseYawGain = 3.80f;
constexpr float kAirMousePitchGain = 3.10f;
constexpr float kGravityLpfAlpha = 0.14f;
constexpr float kVelocityLpfAlpha = 0.20f;
constexpr float kRightAxisLpfAlpha = 0.20f;
constexpr float kAccelScaleMgPerLsb = 0.488f;   // ±16g mode
constexpr float kGyroScaleMdpsPerLsb = 70.0f;   // ±2000 dps mode
constexpr float kRateAccelRefDps = 60.0f;
constexpr float kRateAccelMaxBoost = 5.00f;
constexpr uint32_t kTapHoldArmWindowMs = 700;
constexpr uint32_t kTouchTapMaxDurationMs = 260;
constexpr uint32_t kTouchLongPressMs = 420;
constexpr uint32_t kTouchSwipeMaxDurationMs = 380;
constexpr int16_t kTouchTapMoveThresholdPx = 12;
constexpr int16_t kTouchSwipeMinDistancePx = 24;
constexpr float kRollRateDampStartDps = 45.0f;
constexpr float kRollRateSuppressDps = 140.0f;
constexpr int kYawSign = -1;
constexpr int kPitchSign = 1;

enum class InputMode : uint8_t {
  Mouse,
  Graffiti,
};

volatile bool g_touchInterrupt = false;
bool g_bleConnected = false;
bool g_touchActive = false;
bool g_imuReady = false;
InputMode g_inputMode = InputMode::Mouse;

uint32_t g_lastTouchEventMs = 0;
uint32_t g_lastTouchPollMs = 0;
uint32_t g_lastImuPollMs = 0;
uint32_t g_lastImuDebugMs = 0;
uint32_t g_lastImuSampleUs = 0;
uint32_t g_lastDisplayRefreshMs = 0;

int16_t g_touchX = -1;
int16_t g_touchY = -1;
int8_t g_lastDeltaX = 0;
int8_t g_lastDeltaY = 0;
String g_lastGesture = "NONE";
String g_clickMode = "FREE";
String g_lastCmdAck = "BOOT";

float g_gyroBiasX = 0.0f;  // mdps
float g_gyroBiasY = 0.0f;  // mdps
float g_gyroBiasZ = 0.0f;  // mdps

float g_gravityX = 0.0f;
float g_gravityY = 0.0f;
float g_gravityZ = 1.0f;

float g_rightX = 1.0f;
float g_rightY = 0.0f;
float g_rightZ = 0.0f;

float g_airVelX = 0.0f;
float g_airVelY = 0.0f;
float g_airResidualX = 0.0f;
float g_airResidualY = 0.0f;
float g_yawRateDps = 0.0f;
float g_pitchRateDps = 0.0f;
float g_rollRateDps = 0.0f;
float g_rollDamp = 1.0f;

uint8_t g_buttonMask = 0;
bool g_leftLockActive = false;
bool g_tapArmed = false;
uint32_t g_tapArmDeadlineMs = 0;
bool g_statusFrameDrawn = false;
String g_prevStatusText[9];
uint16_t g_prevStatusColor[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
bool g_touchDown = false;
bool g_touchLongActionFired = false;
uint32_t g_touchDownStartMs = 0;
int16_t g_touchDownX = -1;
int16_t g_touchDownY = -1;
bool g_swipeHandledThisTouch = false;
GraffitiRecognizer g_graffitiRecognizer;
GraffitiRecognizer::Point g_graffitiStroke[kGraffitiStrokeMaxPoints];
uint16_t g_graffitiStrokeCount = 0;
uint16_t g_graffitiLastStrokePoints = 0;
uint32_t g_graffitiTouchStartMs = 0;
String g_graffitiText = "";
String g_graffitiStatus = "IDLE";
String g_graffitiLastMatch = "NONE";
float g_graffitiLastScore = 0.0f;
char g_serialCmdBuffer[kSerialCmdMaxLen] = {0};
uint8_t g_serialCmdLen = 0;
uint32_t g_graffitiTapArmedMs = 0;
bool g_graffitiTapArmed = false;
int16_t g_graffitiTapAnchorX = -1;
int16_t g_graffitiTapAnchorY = -1;
uint8_t g_graffitiNoFingerSamples = 0;
uint32_t g_graffitiLastCoordMs = 0;
int16_t g_graffitiLastDeltaX = 0;
int16_t g_graffitiLastDeltaY = 0;
uint16_t g_graffitiLastPressMs = 0;
bool g_graffitiReadFault = false;
bool g_graffitiTapOnlyActive = false;
bool g_modeFlipRefReady = false;
uint8_t g_modeFlipRefAxis = 2;
float g_modeFlipRefSign = 1.0f;
bool g_setupShiftPending = false;
uint8_t g_setupShiftAttemptsRemaining = 0;
uint32_t g_setupShiftNextMs = 0;

void renderStatus(bool force = false);
void updateImuOrientationOnly();

BLEHIDDevice *g_hid = nullptr;
BLECharacteristic *g_inputMouse = nullptr;
BLECharacteristic *g_inputKeyboard = nullptr;
BLECharacteristic *g_outputKeyboard = nullptr;
BLEAdvertising *g_advertising = nullptr;

Arduino_DataBus *bus = new Arduino_HWSPI(
    LCD_DC /* DC */, LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_MOSI /* MOSI */, -1 /* MISO */);

Arduino_GFX *gfx = new Arduino_GC9107(
    bus, LCD_RST /* RST */, 0 /* rotation */, true /* IPS */,
    LCD_WIDTH /* width */, LCD_HEIGHT /* height */,
    2 /* col offset 1 */, 1 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
    std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

std::unique_ptr<Arduino_IIC> CST816T(new Arduino_CST816x(IIC_Bus, CST816T_DEVICE_ADDRESS,
                                                         TP_RST, TP_INT, []() {
                                                             g_touchInterrupt = true;
                                                         }));

std::unique_ptr<Arduino_IIC> LSM6DSL(new Arduino_LSM6DSL(IIC_Bus, LSM6DSL_DEVICE_ADDRESS,
                                                         DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));

std::unique_ptr<Arduino_IIC> ETA4662(new Arduino_ETA4662(IIC_Bus, ETA4662_DEVICE_ADDRESS,
                                                         DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));

std::unique_ptr<Arduino_IIC> SGM41562(new Arduino_SGM41562(IIC_Bus, SGM41562_DEVICE_ADDRESS,
                                                           DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));

struct ImuSample {
  float ax_mg;
  float ay_mg;
  float az_mg;
  float gx_mdps;
  float gy_mdps;
  float gz_mdps;
};

class ServerCallbacks final : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    g_bleConnected = true;
    Serial.println("[ble] Connected");
    g_setupShiftPending = true;
    g_setupShiftAttemptsRemaining = kSetupShiftConnectAttempts;
    g_setupShiftNextMs = millis() + kSetupShiftInitialDelayMs;
    Serial.println("[ble] setup shift queued");
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    g_bleConnected = false;
    g_buttonMask = 0;
    g_leftLockActive = false;
    g_clickMode = "FREE";
    g_tapArmed = false;
    g_setupShiftPending = false;
    g_setupShiftAttemptsRemaining = 0;
    Serial.println("[ble] Disconnected, advertising");
    if (g_advertising != nullptr) {
      delay(20);
      g_advertising->start();
    }
  }
};

int8_t clampToInt8(int value) {
  value = std::max(-127, std::min(127, value));
  return static_cast<int8_t>(value);
}

void sendMouseReport(uint8_t buttons, int8_t x, int8_t y, int8_t wheel = 0) {
  if (!g_bleConnected || g_inputMouse == nullptr) {
    return;
  }

  uint8_t report[4];
  report[0] = buttons;
  report[1] = static_cast<uint8_t>(x);
  report[2] = static_cast<uint8_t>(y);
  report[3] = static_cast<uint8_t>(wheel);

  g_inputMouse->setValue(report, sizeof(report));
  g_inputMouse->notify();
}

void sendKeyboardReport(uint8_t modifier, uint8_t keyUsage) {
  if (!g_bleConnected || g_inputKeyboard == nullptr) {
    return;
  }

  uint8_t report[8] = {0};
  report[0] = modifier;
  report[2] = keyUsage;

  g_inputKeyboard->setValue(report, sizeof(report));
  g_inputKeyboard->notify();
}

void tapKeyboardUsage(uint8_t modifier, uint8_t keyUsage) {
  sendKeyboardReport(modifier, keyUsage);
  delay(8);
  sendKeyboardReport(0, 0);
}

bool mapSymbolToKeyboardUsage(char symbol, uint8_t &modifier, uint8_t &usage) {
  modifier = 0;
  usage = 0;

  if (symbol >= 'a' && symbol <= 'z') {
    usage = static_cast<uint8_t>(0x04 + (symbol - 'a'));
    return true;
  }
  if (symbol >= 'A' && symbol <= 'Z') {
    usage = static_cast<uint8_t>(0x04 + (symbol - 'A'));
    modifier = 0x02;  // Left Shift.
    return true;
  }

  switch (symbol) {
    case ' ':
      usage = 0x2C;
      return true;
    case '\b':
      usage = 0x2A;
      return true;
    case '\n':
    case '\r':
      usage = 0x28;
      return true;
    default:
      return false;
  }
}

bool sendKeyboardSymbol(char symbol) {
  uint8_t modifier = 0;
  uint8_t usage = 0;
  if (!mapSymbolToKeyboardUsage(symbol, modifier, usage)) {
    return false;
  }
  tapKeyboardUsage(modifier, usage);
  return true;
}

void queueSetupShift(uint8_t attempts, uint32_t delayMs) {
  if (attempts == 0) {
    g_setupShiftPending = false;
    g_setupShiftAttemptsRemaining = 0;
    return;
  }
  g_setupShiftPending = true;
  g_setupShiftAttemptsRemaining = attempts;
  g_setupShiftNextMs = millis() + delayMs;
}

void serviceSetupShift() {
  if (!g_setupShiftPending || !g_bleConnected) {
    return;
  }
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - g_setupShiftNextMs) < 0) {
    return;
  }

  // Modifier-only report: left shift pressed and released.
  tapKeyboardUsage(0x02, 0x00);

  if (g_setupShiftAttemptsRemaining > 0) {
    --g_setupShiftAttemptsRemaining;
  }

  if (g_setupShiftAttemptsRemaining == 0) {
    g_setupShiftPending = false;
    Serial.println("[kbd] setup shift done");
  } else {
    g_setupShiftNextMs = now + kSetupShiftRetryMs;
  }
}

void setButtonMask(uint8_t mask) {
  g_buttonMask = mask;
  sendMouseReport(g_buttonMask, 0, 0, 0);
}

void clickButton(uint8_t buttonMask) {
  const uint8_t previous = g_buttonMask;
  setButtonMask(previous | buttonMask);
  delay(16);
  setButtonMask(previous);
}

void sendLeftClick() {
  clickButton(0x01);
}

void sendRightClick() {
  clickButton(0x02);
}

void setLeftLock(bool enable) {
  g_leftLockActive = enable;
  if (enable) {
    setButtonMask(g_buttonMask | 0x01);
    g_clickMode = "DRAG";
    Serial.println("[touch] drag hold ON");
  } else {
    setButtonMask(g_buttonMask & static_cast<uint8_t>(~0x01));
    g_clickMode = g_tapArmed ? "TAP-ARM" : "FREE";
    Serial.println("[touch] drag hold OFF");
  }
}

void armTapWindow() {
  g_tapArmed = true;
  g_tapArmDeadlineMs = millis() + kTapHoldArmWindowMs;
  g_clickMode = "TAP-ARM";
}

void onTapReleased() {
  if (g_leftLockActive) {
    return;
  }

  g_clickMode = "L-CLICK";
  sendLeftClick();
  armTapWindow();
}

const char *modeName(InputMode mode) {
  return (mode == InputMode::Graffiti) ? "GRAFFITI" : "MOUSE";
}

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:
      return "UNKNOWN";
    case ESP_RST_POWERON:
      return "POWERON";
    case ESP_RST_EXT:
      return "EXT";
    case ESP_RST_SW:
      return "SW";
    case ESP_RST_PANIC:
      return "PANIC";
    case ESP_RST_INT_WDT:
      return "INT_WDT";
    case ESP_RST_TASK_WDT:
      return "TASK_WDT";
    case ESP_RST_WDT:
      return "WDT";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    case ESP_RST_SDIO:
      return "SDIO";
    case ESP_RST_USB:
      return "USB";
    case ESP_RST_JTAG:
      return "JTAG";
    case ESP_RST_EFUSE:
      return "EFUSE";
    case ESP_RST_PWR_GLITCH:
      return "PWR_GLITCH";
    case ESP_RST_CPU_LOCKUP:
      return "CPU_LOCKUP";
    default:
      return "OTHER";
  }
}

String textTail(const String &text, uint8_t maxLen) {
  if (text.length() <= maxLen) {
    return text;
  }
  return text.substring(text.length() - maxLen);
}

String normalizeTouchGesture(const String &rawGesture) {
  if (!kTouchGestureYInverted) {
    return rawGesture;
  }

  if (rawGesture == "Swipe Up") {
    return "Swipe Down";
  }
  if (rawGesture == "Swipe Down") {
    return "Swipe Up";
  }
  return rawGesture;
}

void resetGraffitiStrokeState() {
  g_graffitiStrokeCount = 0;
  g_graffitiTouchStartMs = 0;
  g_graffitiNoFingerSamples = 0;
  g_graffitiLastDeltaX = 0;
  g_graffitiLastDeltaY = 0;
  g_graffitiLastPressMs = 0;
  g_graffitiReadFault = false;
}

void resetGraffitiTapSwitchState() {
  g_graffitiTapArmed = false;
  g_graffitiTapArmedMs = 0;
  g_graffitiTapAnchorX = -1;
  g_graffitiTapAnchorY = -1;
}

float gravityAxisValue(uint8_t axis) {
  if (axis == 0) {
    return g_gravityX;
  }
  if (axis == 1) {
    return g_gravityY;
  }
  return g_gravityZ;
}

void maybeCaptureModeFlipReference() {
  if (g_modeFlipRefReady) {
    return;
  }

  const float ax = fabsf(g_gravityX);
  const float ay = fabsf(g_gravityY);
  const float az = fabsf(g_gravityZ);

  if (ax > ay && ax > az) {
    g_modeFlipRefAxis = 0;
  } else if (ay > az) {
    g_modeFlipRefAxis = 1;
  } else {
    g_modeFlipRefAxis = 2;
  }

  const float v = gravityAxisValue(g_modeFlipRefAxis);
  g_modeFlipRefSign = (v >= 0.0f) ? 1.0f : -1.0f;
  g_modeFlipRefReady = true;
  Serial.printf("[mode] flip-ref axis=%u sign=%.0f\n", g_modeFlipRefAxis, g_modeFlipRefSign);
}

bool isModeFlipInverted() {
  if (!g_modeFlipRefReady) {
    return false;
  }

  const float projection = gravityAxisValue(g_modeFlipRefAxis) * g_modeFlipRefSign;
  return projection <= kGraffitiInvertThreshold;
}

float modeFlipProjection() {
  if (!g_modeFlipRefReady) {
    return 1.0f;
  }
  return gravityAxisValue(g_modeFlipRefAxis) * g_modeFlipRefSign;
}

bool isGraffitiUpsideDown() {
  return gravityAxisValue(kGraffitiInvertAxis) <= kGraffitiInvertThreshold;
}

float graffitiInvertProjection() {
  return gravityAxisValue(kGraffitiInvertAxis);
}

void updateGraffitiTapOnlyMode() {
  if (g_inputMode != InputMode::Graffiti) {
    g_graffitiTapOnlyActive = false;
    return;
  }

  const float projection = graffitiInvertProjection();
  const bool nextTapOnly =
      g_graffitiTapOnlyActive ? (projection <= kGraffitiInvertExitThreshold)
                              : (projection <= kGraffitiInvertThreshold);

  if (nextTapOnly == g_graffitiTapOnlyActive) {
    return;
  }

  g_graffitiTapOnlyActive = nextTapOnly;
  resetGraffitiStrokeState();
  g_touchDown = false;
  g_touchActive = false;

  if (g_graffitiTapOnlyActive) {
    g_graffitiStatus = "EXIT TAP";
    Serial.printf("[graffiti] tap-only ON (proj=%.2f)\n", projection);
  } else {
    resetGraffitiTapSwitchState();
    if (g_inputMode == InputMode::Graffiti) {
      g_graffitiStatus = "READY";
    }
    Serial.printf("[graffiti] tap-only OFF (proj=%.2f)\n", projection);
  }
}

void resetInputTransientState() {
  g_touchDown = false;
  g_touchLongActionFired = false;
  g_swipeHandledThisTouch = false;
  g_touchActive = false;
  g_tapArmed = false;
  g_lastGesture = "NONE";
  g_clickMode = "FREE";

  if (g_leftLockActive) {
    setLeftLock(false);
  }

  g_airVelX = 0.0f;
  g_airVelY = 0.0f;
  g_airResidualX = 0.0f;
  g_airResidualY = 0.0f;
  g_lastDeltaX = 0;
  g_lastDeltaY = 0;
  resetGraffitiTapSwitchState();
  resetGraffitiStrokeState();
}

void setInputMode(InputMode mode) {
  if (g_inputMode == mode) {
    return;
  }

  resetInputTransientState();
  if (mode == InputMode::Graffiti) {
    g_graffitiTapOnlyActive = false;
    g_graffitiStatus = "READY";
    g_graffitiLastMatch = "NONE";
    g_graffitiLastScore = 0.0f;
    resetGraffitiStrokeState();
    resetGraffitiTapSwitchState();
  } else {
    g_graffitiTapOnlyActive = false;
    g_graffitiStatus = "IDLE";
    g_graffitiLastMatch = "NONE";
    g_graffitiLastScore = 0.0f;
    resetGraffitiStrokeState();
    resetGraffitiTapSwitchState();
  }

  g_inputMode = mode;
  Serial.printf("[mode] switched to %s\n", modeName(g_inputMode));
  renderStatus(true);
}

void drawStatusLine(uint8_t index, int16_t y, const String &text, uint16_t color, bool force) {
  if (!force && g_prevStatusText[index] == text && g_prevStatusColor[index] == color) {
    return;
  }

  char padded[32];
  snprintf(padded, sizeof(padded), "%-24.24s", text.c_str());
  gfx->setCursor(2, y);
  gfx->setTextColor(color, BLACK);
  gfx->print(padded);

  g_prevStatusText[index] = text;
  g_prevStatusColor[index] = color;
}

void printCommandHelp() {
  Serial.println("[cmd] g|m|mode graffiti|mode mouse|mode?|status|shift|ping|help");
}

void setCommandAck(const String &ack) {
  g_lastCmdAck = ack;
  Serial.printf("[cmd] %s\n", g_lastCmdAck.c_str());
}

void processSerialCommand(const char *line) {
  if (line == nullptr) {
    return;
  }

  String cmd(line);
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.isEmpty()) {
    return;
  }

  if (cmd == "g" || cmd == "graffiti" || cmd == "mode graffiti") {
    setInputMode(InputMode::Graffiti);
    setCommandAck(String("ok mode=") + modeName(g_inputMode));
    return;
  }
  if (cmd == "m" || cmd == "mouse" || cmd == "mode mouse") {
    setInputMode(InputMode::Mouse);
    setCommandAck(String("ok mode=") + modeName(g_inputMode));
    return;
  }
  if (cmd == "mode?" || cmd == "mode" || cmd == "status") {
    setCommandAck(String("mode=") + modeName(g_inputMode));
    Serial.printf("[cmd] mode=%s touchDown=%u touchActive=%u n=%u l=%u invert=%u proj=%.2f\n",
                  modeName(g_inputMode), g_touchDown ? 1U : 0U, g_touchActive ? 1U : 0U,
                  g_graffitiStrokeCount, g_graffitiLastStrokePoints,
                  g_graffitiTapOnlyActive ? 1U : 0U, graffitiInvertProjection());
    return;
  }
  if (cmd == "shift") {
    if (!g_bleConnected) {
      setCommandAck("shift no-link");
    } else {
      queueSetupShift(1, 20);
      setCommandAck("shift queued");
    }
    return;
  }
  if (cmd == "ping") {
    setCommandAck("pong");
    return;
  }
  if (cmd == "help" || cmd == "?") {
    setCommandAck("help");
    printCommandHelp();
    return;
  }

  setCommandAck(String("unknown: ") + cmd);
  printCommandHelp();
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    const int in = Serial.read();
    if (in < 0) {
      return;
    }

    const char c = static_cast<char>(in);
    if (c == '\r' || c == '\n' || c == ';') {
      if (g_serialCmdLen > 0) {
        g_serialCmdBuffer[g_serialCmdLen] = '\0';
        processSerialCommand(g_serialCmdBuffer);
        g_serialCmdLen = 0;
      }
      continue;
    }

    if (g_serialCmdLen < (kSerialCmdMaxLen - 1)) {
      g_serialCmdBuffer[g_serialCmdLen++] = c;
    } else {
      g_serialCmdLen = 0;
      Serial.println("[cmd] command too long");
    }
  }
}

String graffitiSymbolLabel(char symbol) {
  if (symbol == ' ') {
    return "SPACE";
  }
  if (symbol == '\b') {
    return "BKSP";
  }
  char buf[2] = {symbol, '\0'};
  return String(buf);
}

bool applyGraffitiSymbol(char symbol) {
  if (symbol == '\b') {
    if (!g_graffitiText.isEmpty()) {
      g_graffitiText.remove(g_graffitiText.length() - 1);
    }
    return sendKeyboardSymbol(symbol);
  }

  g_graffitiText += symbol;
  if (g_graffitiText.length() > 96) {
    g_graffitiText = g_graffitiText.substring(g_graffitiText.length() - 96);
  }
  return sendKeyboardSymbol(symbol);
}

void graffitiAddPoint(int16_t x, int16_t y) {
  if (g_graffitiStrokeCount > 0) {
    const float dx = static_cast<float>(x) - g_graffitiStroke[g_graffitiStrokeCount - 1].x;
    const float dy = static_cast<float>(y) - g_graffitiStroke[g_graffitiStrokeCount - 1].y;
    const float distanceSq = (dx * dx) + (dy * dy);
    if (distanceSq < static_cast<float>(kGraffitiMinPointDistanceSq)) {
      return;
    }
  }

  if (g_graffitiStrokeCount >= kGraffitiStrokeMaxPoints) {
    return;
  }

  g_graffitiStroke[g_graffitiStrokeCount].x = static_cast<float>(x);
  g_graffitiStroke[g_graffitiStrokeCount].y = static_cast<float>(y);
  g_graffitiStrokeCount++;
}

void finalizeGraffitiStroke() {
  if (g_graffitiTapOnlyActive) {
    g_graffitiStatus = "EXIT TAP";
    resetGraffitiStrokeState();
    return;
  }

  g_graffitiLastStrokePoints = g_graffitiStrokeCount;
  if (g_graffitiStrokeCount < kGraffitiStrokeMinPoints) {
    g_graffitiStatus = "SHORT";
    g_graffitiLastMatch = "NONE";
    g_graffitiLastScore = 0.0f;
    resetGraffitiTapSwitchState();
    resetGraffitiStrokeState();
    return;
  }

  GraffitiRecognizer::Result result;
  if (!g_graffitiRecognizer.recognize(g_graffitiStroke, g_graffitiStrokeCount, result)) {
    g_graffitiStatus = "ERROR";
    g_graffitiLastMatch = "NONE";
    g_graffitiLastScore = 0.0f;
    resetGraffitiTapSwitchState();
    resetGraffitiStrokeState();
    return;
  }

  g_graffitiLastScore = result.score;
  g_graffitiLastMatch = String(result.name) + ":" + graffitiSymbolLabel(result.symbol);
  if (result.score >= GraffitiRecognizer::kAcceptScore) {
    g_graffitiStatus = "ACCEPT";
    const bool keySent = applyGraffitiSymbol(result.symbol);
    Serial.printf("[graffiti] ACCEPT %s score=%.2f kbd=%s\n", g_graffitiLastMatch.c_str(),
                  g_graffitiLastScore, keySent ? "ok" : "skip");
  } else {
    g_graffitiStatus = "REJECT";
    Serial.printf("[graffiti] REJECT %s score=%.2f\n", g_graffitiLastMatch.c_str(), g_graffitiLastScore);
  }

  resetGraffitiTapSwitchState();
  resetGraffitiStrokeState();
}

bool isGraffitiSwipeDownExit(uint32_t pressDurationMs, int16_t deltaX, int16_t deltaY, uint16_t pointCount) {
  if (pointCount < 2) {
    return false;
  }
  if (pressDurationMs > kGraffitiExitSwipeMaxDurationMs) {
    return false;
  }
  if (deltaY < kGraffitiExitSwipeMinDyPx) {
    return false;
  }
  if (abs(deltaX) > kGraffitiExitSwipeMaxDxPx) {
    return false;
  }
  return true;
}

void sampleGraffitiExitTapOnly() {
  const uint32_t now = millis();
  const int16_t finger = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

  if (finger <= 0) {
    if (!g_touchDown) {
      g_touchActive = false;
      return;
    }

    const uint32_t pressDuration = now - g_touchDownStartMs;
    const int16_t deltaX = g_touchX - g_touchDownX;
    const int16_t deltaY = g_touchY - g_touchDownY;
    const int16_t absDx = abs(deltaX);
    const int16_t absDy = abs(deltaY);
    const bool tapLike = (pressDuration <= kTouchTapMaxDurationMs &&
                          absDx <= kTouchTapMoveThresholdPx &&
                          absDy <= kTouchTapMoveThresholdPx);

    g_graffitiLastDeltaX = deltaX;
    g_graffitiLastDeltaY = deltaY;
    g_graffitiLastPressMs = static_cast<uint16_t>(std::min<uint32_t>(65535, pressDuration));
    g_touchDown = false;
    g_touchActive = false;
    resetGraffitiStrokeState();

    if (!tapLike) {
      g_graffitiStatus = "EXIT TAP";
      resetGraffitiTapSwitchState();
      return;
    }

    if (g_graffitiTapArmed &&
        (now - g_graffitiTapArmedMs) <= kModeExitDoubleTapWindowMs) {
      const int16_t sepX = abs(g_touchDownX - g_graffitiTapAnchorX);
      const int16_t sepY = abs(g_touchDownY - g_graffitiTapAnchorY);
      if (sepX <= kModeExitTapSeparationPx && sepY <= kModeExitTapSeparationPx) {
        g_graffitiStatus = "MODE->MOUSE";
        resetGraffitiTapSwitchState();
        setInputMode(InputMode::Mouse);
        return;
      }
    }

    g_graffitiTapArmed = true;
    g_graffitiTapArmedMs = now;
    g_graffitiTapAnchorX = g_touchDownX;
    g_graffitiTapAnchorY = g_touchDownY;
    g_graffitiStatus = "EXIT TAP1";
    return;
  }

  const int16_t x = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  const int16_t y = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
  if (x < 0 || y < 0) {
    g_graffitiReadFault = true;
    return;
  }

  g_graffitiReadFault = false;
  g_touchActive = true;
  g_touchX = x;
  g_touchY = y;
  g_lastTouchEventMs = now;
  g_graffitiLastCoordMs = now;

  if (!g_touchDown) {
    g_touchDown = true;
    g_touchDownStartMs = now;
    g_touchDownX = x;
    g_touchDownY = y;
  }
  if (!g_graffitiTapArmed) {
    g_graffitiStatus = "EXIT TAP";
  }
}

void sampleGraffitiTouchState() {
  if (g_graffitiTapOnlyActive) {
    sampleGraffitiExitTapOnly();
    return;
  }

  const uint32_t now = millis();
  const int16_t x = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  const int16_t y = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
  if (x < 0 || y < 0) {
    g_graffitiReadFault = true;
    if (g_touchDown &&
        g_graffitiLastCoordMs != 0 &&
        (now - g_graffitiLastCoordMs) > kGraffitiReleaseHoldMs) {
      g_touchDown = false;
      g_touchActive = false;
      g_graffitiStatus = "T-ERR";
      resetGraffitiTapSwitchState();
      resetGraffitiStrokeState();
    }
    return;
  }
  g_graffitiReadFault = false;
  const bool coordValid = (x >= 0 && y >= 0);
  const bool coordIdle = coordValid && (x == kCstIdleX) && (y == kCstIdleY);
  const bool coordTouch = coordValid && !coordIdle;
  const bool touchPresentRaw = coordTouch;

  if (coordTouch) {
    g_touchActive = true;
    g_touchX = x;
    g_touchY = y;
    g_lastTouchEventMs = now;
    g_graffitiLastCoordMs = now;
  }

  if (touchPresentRaw) {
    g_touchActive = true;
    g_graffitiNoFingerSamples = 0;

    if (!g_touchDown) {
      if (!coordTouch) {
        return;
      }
      resetGraffitiStrokeState();
      g_touchDown = true;
      g_graffitiStatus = "DRAW";
      g_graffitiTouchStartMs = now;
      g_touchDownStartMs = now;
      g_touchDownX = g_touchX;
      g_touchDownY = g_touchY;
      g_graffitiLastCoordMs = now;
    }

    if (coordTouch) {
      graffitiAddPoint(g_touchX, g_touchY);
    }
    if ((now - g_graffitiTouchStartMs) > kGraffitiStrokeMaxDurationMs) {
      g_touchDown = false;
      g_touchActive = false;
      g_graffitiStatus = "TIMEOUT";
      finalizeGraffitiStroke();
    }
    return;
  }
  if (!g_touchDown) {
    return;
  }

  if (g_graffitiLastCoordMs != 0 &&
      (now - g_graffitiLastCoordMs) <= kGraffitiReleaseHoldMs) {
    return;
  }

  const uint32_t pressDuration = now - g_touchDownStartMs;
  const int16_t deltaX = g_touchX - g_touchDownX;
  const int16_t deltaY = g_touchY - g_touchDownY;
  const int16_t absDx = abs(deltaX);
  const int16_t absDy = abs(deltaY);
  const bool tapLikeByMotion = (pressDuration <= kGraffitiTapMaxDurationMs &&
                                absDx <= kGraffitiTapMoveThresholdPx &&
                                absDy <= kGraffitiTapMoveThresholdPx);
  const bool tapLikeByStrokeSize = (pressDuration <= (kGraffitiTapMaxDurationMs + 180) &&
                                    g_graffitiStrokeCount <= kGraffitiTapMaxStrokePoints);
  const bool tapLike = tapLikeByMotion || tapLikeByStrokeSize;
  g_graffitiLastDeltaX = deltaX;
  g_graffitiLastDeltaY = deltaY;
  g_graffitiLastPressMs = static_cast<uint16_t>(std::min<uint32_t>(65535, pressDuration));

  g_touchDown = false;
  g_touchActive = false;

  if (isGraffitiSwipeDownExit(pressDuration, deltaX, deltaY, g_graffitiStrokeCount)) {
    g_graffitiStatus = "MODE->MOUSE";
    resetGraffitiTapSwitchState();
    setInputMode(InputMode::Mouse);
    return;
  }

  if (tapLike) {
    resetGraffitiTapSwitchState();
    g_graffitiStatus = "TAP";
    resetGraffitiStrokeState();
    return;
  }

  if (g_graffitiStrokeCount > 0) {
    finalizeGraffitiStroke();
  } else {
    g_graffitiStatus = "READY";
  }
}

bool normalize3(float &x, float &y, float &z) {
  const float n = sqrtf((x * x) + (y * y) + (z * z));
  if (n < 1e-6f) {
    return false;
  }
  x /= n;
  y /= n;
  z /= n;
  return true;
}

bool readLsm6dslAxis(uint8_t lowReg, uint8_t highReg, int16_t &outRaw) {
  uint8_t lo = 0;
  uint8_t hi = 0;
  if (!IIC_Bus->IIC_ReadC8D8(LSM6DSL_DEVICE_ADDRESS, lowReg, &lo)) {
    return false;
  }
  if (!IIC_Bus->IIC_ReadC8D8(LSM6DSL_DEVICE_ADDRESS, highReg, &hi)) {
    return false;
  }

  outRaw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  return true;
}

bool readImuSample(ImuSample &sample) {
  uint8_t status = 0;
  if (!IIC_Bus->IIC_ReadC8D8(LSM6DSL_DEVICE_ADDRESS, LSM6DSL_RD_STATUS_REG, &status)) {
    return false;
  }
  if ((status & 0x03) == 0) {
    return false;
  }

  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  int16_t azRaw = 0;
  int16_t gxRaw = 0;
  int16_t gyRaw = 0;
  int16_t gzRaw = 0;

  if (!readLsm6dslAxis(LSM6DSL_RD_OUTX_L_XL, LSM6DSL_RD_OUTX_H_XL, axRaw) ||
      !readLsm6dslAxis(LSM6DSL_RD_OUTY_L_XL, LSM6DSL_RD_OUTY_H_XL, ayRaw) ||
      !readLsm6dslAxis(LSM6DSL_RD_OUTZ_L_XL, LSM6DSL_RD_OUTZ_H_XL, azRaw) ||
      !readLsm6dslAxis(LSM6DSL_RD_OUTX_L_G, LSM6DSL_RD_OUTX_H_G, gxRaw) ||
      !readLsm6dslAxis(LSM6DSL_RD_OUTY_L_G, LSM6DSL_RD_OUTY_H_G, gyRaw) ||
      !readLsm6dslAxis(LSM6DSL_RD_OUTZ_L_G, LSM6DSL_RD_OUTZ_H_G, gzRaw)) {
    return false;
  }

  sample.ax_mg = static_cast<float>(axRaw) * kAccelScaleMgPerLsb;
  sample.ay_mg = static_cast<float>(ayRaw) * kAccelScaleMgPerLsb;
  sample.az_mg = static_cast<float>(azRaw) * kAccelScaleMgPerLsb;

  sample.gx_mdps = static_cast<float>(gxRaw) * kGyroScaleMdpsPerLsb;
  sample.gy_mdps = static_cast<float>(gyRaw) * kGyroScaleMdpsPerLsb;
  sample.gz_mdps = static_cast<float>(gzRaw) * kGyroScaleMdpsPerLsb;
  return true;
}

bool calibrateGyroBias() {
  Serial.println("[imu] Calibrating gyro bias; keep device still...");

  gfx->fillScreen(BLACK);
  gfx->setCursor(2, 24);
  gfx->setTextColor(WHITE);
  gfx->print("IMU calibration");
  gfx->setCursor(2, 40);
  gfx->print("Keep still...");

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  uint16_t collected = 0;

  const uint32_t startMs = millis();
  while (collected < kImuGyroCalibrationSamples && (millis() - startMs) < 7000) {
    ImuSample sample;
    if (readImuSample(sample)) {
      sumX += sample.gx_mdps;
      sumY += sample.gy_mdps;
      sumZ += sample.gz_mdps;
      collected++;
    }
    delay(3);
  }

  if (collected < (kImuGyroCalibrationSamples / 2)) {
    Serial.printf("[imu] Calibration failed, collected %u/%u samples\n", collected, kImuGyroCalibrationSamples);
    return false;
  }

  g_gyroBiasX = sumX / collected;
  g_gyroBiasY = sumY / collected;
  g_gyroBiasZ = sumZ / collected;

  Serial.printf("[imu] Gyro bias mdps: x=%.2f y=%.2f z=%.2f\n", g_gyroBiasX, g_gyroBiasY, g_gyroBiasZ);
  return true;
}

void initBoardPowerAndDisplay() {
  Arduino_IIC *powerChip = nullptr;
  if (ETA4662->begin(kI2cBusHz)) {
    Serial.println("[power] ETA4662 init ok");
    powerChip = ETA4662.get();
  } else if (SGM41562->begin(kI2cBusHz)) {
    Serial.println("[power] SGM41562 init ok");
    powerChip = SGM41562.get();
  } else {
    Serial.println("[power] Power chip init failed");
  }

  if (powerChip != nullptr) {
    const bool chargeEnabled = powerChip->IIC_Write_Device_State(
        Arduino_IIC_Power::Device::POWER_DEVICE_CHARGING_MODE,
        Arduino_IIC_Power::Device_State::POWER_DEVICE_ON);
    const bool watchdogDisabled = powerChip->IIC_Write_Device_State(
        Arduino_IIC_Power::Device::POWER_DEVICE_WATCHDOG_MODE,
        Arduino_IIC_Power::Device_State::POWER_DEVICE_OFF);
    const String chargeState = powerChip->IIC_Read_Device_State(
        Arduino_IIC_Power::Status_Information::POWER_CHARGING_STATUS);
    const String inputState = powerChip->IIC_Read_Device_State(
        Arduino_IIC_Power::Status_Information::POWER_INPUT_SOURCE_STATUS);
    Serial.printf("[power] charge=%s watchdog=%s\n", chargeEnabled ? "on" : "unchanged",
                  watchdogDisabled ? "off" : "unchanged");
    Serial.printf("[power] pmic charge=%s input=%s\n", chargeState.c_str(), inputState.c_str());
  }

  pinMode(BREATHING_LIGHT, OUTPUT);
  ledcAttach(BREATHING_LIGHT, 2000, 8);
  ledcWrite(BREATHING_LIGHT, 255);

  pinMode(BATTERY_ADC_DATA, INPUT_PULLDOWN);
  pinMode(BATTERY_MEASUREMENT_CONTROL, OUTPUT);
  digitalWrite(BATTERY_MEASUREMENT_CONTROL, HIGH);
  analogReadResolution(12);

  pinMode(LCD_BL, OUTPUT);
  ledcAttach(LCD_BL, 2000, 8);
  ledcWrite(LCD_BL, 0);

  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(1);
  gfx->setTextWrap(false);
}

void initTouch() {
  uint8_t attempt = 0;
  while (!CST816T->begin(kI2cBusHz)) {
    attempt++;
    Serial.printf("[touch] init failed (%u), retrying...\n", attempt);
    delay(600);
  }

  CST816T->IIC_Write_Device_State(
      CST816T->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
      CST816T->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);

  Serial.printf("[touch] init ok, id=0x%X\n", (uint8_t)CST816T->IIC_Device_ID());
}

void initImu() {
#if defined T_QT_C6_Battery_V1_0_V1_1
  pinMode(LSM6DSL_IIC_ADDRESS_MODE, OUTPUT);
  digitalWrite(LSM6DSL_IIC_ADDRESS_MODE, LOW);
#elif defined T_QT_C6_Battery_V1_2
  pinMode(LSM6DSL_EN, OUTPUT);
  digitalWrite(LSM6DSL_EN, HIGH);
#endif

  uint8_t attempt = 0;
  while (!LSM6DSL->begin(kI2cBusHz)) {
    attempt++;
    Serial.printf("[imu] init failed (%u), retrying...\n", attempt);
    delay(700);
  }

  LSM6DSL->IIC_Write_Device_State(LSM6DSL->Arduino_IIC_IMU::Device::IMU_ACCELERATION_POWER_MODE,
                                  LSM6DSL->Arduino_IIC_IMU::Device_Mode::IMU_DEVICE_NORMAL_POWER);
  LSM6DSL->IIC_Write_Device_Value(LSM6DSL->Arduino_IIC_IMU::Device_Value::IMU_ACCELERATION_SENSITIVITY,
                                  16);

  LSM6DSL->IIC_Write_Device_State(LSM6DSL->Arduino_IIC_IMU::Device::IMU_GYROSCOPE_POWER_MODE,
                                  LSM6DSL->Arduino_IIC_IMU::Device_Mode::IMU_DEVICE_NORMAL_POWER);
  LSM6DSL->IIC_Write_Device_Value(LSM6DSL->Arduino_IIC_IMU::Device_Value::IMU_GYROSCOPE_SENSITIVITY,
                                  2000);

  const bool calibrated = calibrateGyroBias();
  if (!calibrated) {
    g_gyroBiasX = 0.0f;
    g_gyroBiasY = 0.0f;
    g_gyroBiasZ = 0.0f;
    Serial.println("[imu] Using zero bias fallback");
  }

  g_imuReady = true;
  g_lastImuSampleUs = micros();
  Serial.printf("[imu] ready, id=0x%X\n", (uint8_t)LSM6DSL->IIC_Device_ID());
}

void initBleMouse() {
  static uint8_t reportMap[] = {
      0x05, 0x01,        // Usage Page (Generic Desktop)
      0x09, 0x06,        // Usage (Keyboard)
      0xA1, 0x01,        // Collection (Application)
      0x85, kKeyboardReportId,
      0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
      0x19, 0xE0,        //   Usage Minimum (Keyboard Left Control)
      0x29, 0xE7,        //   Usage Maximum (Keyboard Right GUI)
      0x15, 0x00,        //   Logical Minimum (0)
      0x25, 0x01,        //   Logical Maximum (1)
      0x75, 0x01,        //   Report Size (1)
      0x95, 0x08,        //   Report Count (8)
      0x81, 0x02,        //   Input (Data,Var,Abs)
      0x95, 0x01,        //   Report Count (1)
      0x75, 0x08,        //   Report Size (8)
      0x81, 0x01,        //   Input (Const,Array,Abs)
      0x95, 0x05,        //   Report Count (5)
      0x75, 0x01,        //   Report Size (1)
      0x05, 0x08,        //   Usage Page (LEDs)
      0x19, 0x01,        //   Usage Minimum (Num Lock)
      0x29, 0x05,        //   Usage Maximum (Kana)
      0x91, 0x02,        //   Output (Data,Var,Abs)
      0x95, 0x01,        //   Report Count (1)
      0x75, 0x03,        //   Report Size (3)
      0x91, 0x01,        //   Output (Const,Array,Abs)
      0x95, 0x06,        //   Report Count (6)
      0x75, 0x08,        //   Report Size (8)
      0x15, 0x00,        //   Logical Minimum (0)
      0x25, 0x65,        //   Logical Maximum (101)
      0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
      0x19, 0x00,        //   Usage Minimum (Reserved)
      0x29, 0x65,        //   Usage Maximum (Keyboard Application)
      0x81, 0x00,        //   Input (Data,Array,Abs)
      0xC0,              // End Collection

      0x05, 0x01,        // Usage Page (Generic Desktop)
      0x09, 0x02,        // Usage (Mouse)
      0xA1, 0x01,        // Collection (Application)
      0x85, kMouseReportId,
      0x09, 0x01,        //   Usage (Pointer)
      0xA1, 0x00,        //   Collection (Physical)
      0x05, 0x09,        //     Usage Page (Buttons)
      0x19, 0x01,        //     Usage Minimum (1)
      0x29, 0x03,        //     Usage Maximum (3)
      0x15, 0x00,        //     Logical Minimum (0)
      0x25, 0x01,        //     Logical Maximum (1)
      0x95, 0x03,        //     Report Count (3)
      0x75, 0x01,        //     Report Size (1)
      0x81, 0x02,        //     Input (Data,Var,Abs)
      0x95, 0x01,        //     Report Count (1)
      0x75, 0x05,        //     Report Size (5)
      0x81, 0x03,        //     Input (Const,Var,Abs)
      0x05, 0x01,        //     Usage Page (Generic Desktop)
      0x09, 0x30,        //     Usage (X)
      0x09, 0x31,        //     Usage (Y)
      0x09, 0x38,        //     Usage (Wheel)
      0x15, 0x81,        //     Logical Minimum (-127)
      0x25, 0x7F,        //     Logical Maximum (127)
      0x75, 0x08,        //     Report Size (8)
      0x95, 0x03,        //     Report Count (3)
      0x81, 0x06,        //     Input (Data,Var,Rel)
      0xC0,              //   End Collection
      0xC0,              // End Collection
  };

  BLEDevice::init(kDeviceName);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  g_hid = new BLEHIDDevice(server);
  g_inputMouse = g_hid->inputReport(kMouseReportId);
  g_inputKeyboard = g_hid->inputReport(kKeyboardReportId);
  g_outputKeyboard = g_hid->outputReport(kKeyboardReportId);

  g_hid->manufacturer()->setValue("LILYGO");
  g_hid->pnp(0x02, 0x05AC, 0x820A, 0x0210);
  g_hid->hidInfo(0x00, 0x01);
  g_hid->reportMap(reportMap, sizeof(reportMap));
  g_hid->startServices();
  g_hid->setBatteryLevel(95);

  BLESecurity *security = new BLESecurity();
  security->setAuthenticationMode(ESP_LE_AUTH_BOND);
  security->setCapability(ESP_IO_CAP_NONE);
  security->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  security->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);

  g_advertising = server->getAdvertising();
  g_advertising->setAppearance(GENERIC_HID);
  g_advertising->addServiceUUID(g_hid->hidService()->getUUID());
  g_advertising->start();

  Serial.println("[ble] HID mouse+keyboard started, advertising as 'Ringo'");
}

void renderStatus(bool force) {
  const uint32_t now = millis();
  if (!force && (now - g_lastDisplayRefreshMs) < kDisplayRefreshMs) {
    return;
  }
  g_lastDisplayRefreshMs = now;

  if (force || !g_statusFrameDrawn) {
    gfx->fillScreen(BLACK);
    g_statusFrameDrawn = true;
    for (uint8_t i = 0; i < 9; ++i) {
      g_prevStatusText[i] = "";
      g_prevStatusColor[i] = 0xFFFF;
    }
  }

  char buf[48];
  drawStatusLine(0, 10, String(kDeviceName), CYAN, force);
  drawStatusLine(1, 24, String("Mode: ") + modeName(g_inputMode), WHITE, force);
  snprintf(buf, sizeof(buf), "Roll:%5.1f D:%1.2f", g_rollRateDps, g_rollDamp);
  drawStatusLine(2, 38, String(buf), WHITE, force);

  snprintf(buf, sizeof(buf), "Touch:%s X:%3dY:%3d", g_touchActive ? "ON " : "OFF", g_touchX, g_touchY);
  drawStatusLine(3, 52, String(buf), WHITE, force);

  snprintf(buf, sizeof(buf), "Yaw:%5.1f Pit:%5.1f", g_yawRateDps, g_pitchRateDps);
  drawStatusLine(4, 66, String(buf), WHITE, force);

  if (g_inputMode == InputMode::Mouse) {
    snprintf(buf, sizeof(buf), "Move:%4d,%4d", g_lastDeltaX, g_lastDeltaY);
    drawStatusLine(5, 80, String(buf), WHITE, force);
    drawStatusLine(6, 94, String("G:") + g_lastGesture, WHITE, force);
    drawStatusLine(7, 104, String("Click:") + g_clickMode, GREEN, force);
    drawStatusLine(8, 114, String("Cmd:") + textTail(g_lastCmdAck, 16), YELLOW, force);
  } else {
    snprintf(buf, sizeof(buf), "dX:%4d dY:%4d t:%3u", g_graffitiLastDeltaX, g_graffitiLastDeltaY,
             g_graffitiLastPressMs);
    drawStatusLine(5, 80, String(buf), WHITE, force);
    snprintf(buf, sizeof(buf), "Gra:%s n%u l%u", g_graffitiStatus.c_str(), g_graffitiStrokeCount,
             g_graffitiLastStrokePoints);
    drawStatusLine(6, 94, String(buf), WHITE, force);
    snprintf(buf, sizeof(buf), "I:%c %.2f %s", g_graffitiTapOnlyActive ? 'Y' : 'N',
             graffitiInvertProjection(), g_graffitiReadFault ? "T-ERR" : "T-OK");
    drawStatusLine(7, 104, String(buf), GREEN, force);
    drawStatusLine(8, 114, String("Cmd:") + textTail(g_lastCmdAck, 16), YELLOW, force);
  }
}

void handleGestureIfAny() {
  String rawGesture = CST816T->IIC_Read_Device_State(
      CST816T->Arduino_IIC_Touch::Status_Information::TOUCH_GESTURE_ID);

  if (rawGesture == "NONE" || rawGesture.startsWith("->")) {
    return;
  }

  String gesture = normalizeTouchGesture(rawGesture);
  g_lastGesture = gesture;
  Serial.printf("[touch] gesture raw=%s norm=%s\n", rawGesture.c_str(), gesture.c_str());

  if (g_inputMode == InputMode::Mouse) {
    if (gesture == "Swipe Up") {
      g_tapArmed = false;
      g_swipeHandledThisTouch = true;
      g_lastGesture = "Mode->Graffiti";
      setInputMode(InputMode::Graffiti);
    } else if (gesture == "Swipe Down") {
      g_tapArmed = false;
      g_swipeHandledThisTouch = true;
      sendMouseReport(g_buttonMask, 0, 0, -1);
    }
    return;
  }

  // Graffiti mode uses raw coordinate state machine for stroke and mode exits.
  return;
}

void sampleTouchState() {
  const uint32_t now = millis();
  const int16_t finger = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

  if (finger <= 0) {
    if (g_touchDown) {
      if (g_leftLockActive) {
        g_tapArmed = false;
        setLeftLock(false);
        g_lastGesture = "Drag Release";
      }

      const uint32_t pressDuration = now - g_touchDownStartMs;
      const int16_t deltaX = g_touchX - g_touchDownX;
      const int16_t deltaY = g_touchY - g_touchDownY;
      const int16_t absDx = abs(deltaX);
      const int16_t absDy = abs(deltaY);

      bool handledOnRelease = false;
      if (!g_swipeHandledThisTouch &&
          !g_touchLongActionFired &&
          pressDuration <= kTouchSwipeMaxDurationMs &&
          absDy >= kTouchSwipeMinDistancePx && absDy > (absDx + 8)) {
        const int8_t wheelStep = (deltaY < 0) ? 1 : -1;
        uint8_t steps = static_cast<uint8_t>(std::min<int16_t>(3, std::max<int16_t>(1, absDy / 28)));
        for (uint8_t i = 0; i < steps; ++i) {
          sendMouseReport(g_buttonMask, 0, 0, wheelStep);
          delay(3);
        }
        g_tapArmed = false;
        g_lastGesture = (deltaY < 0) ? "Swipe Up" : "Swipe Down";
        handledOnRelease = true;
      }

      if (!handledOnRelease &&
          !g_touchLongActionFired &&
          pressDuration <= kTouchTapMaxDurationMs &&
          absDx <= kTouchTapMoveThresholdPx &&
          absDy <= kTouchTapMoveThresholdPx) {
        onTapReleased();
      }
    }

    g_touchDown = false;
    g_touchLongActionFired = false;
    g_swipeHandledThisTouch = false;
    g_touchActive = false;
    return;
  }

  const int16_t x = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  const int16_t y = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  if (x < 0 || y < 0) {
    return;
  }

  g_touchActive = true;
  g_touchX = x;
  g_touchY = y;
  g_lastTouchEventMs = now;

  if (!g_touchDown) {
    g_touchDown = true;
    g_touchLongActionFired = false;
    g_touchDownStartMs = now;
    g_touchDownX = x;
    g_touchDownY = y;
    g_swipeHandledThisTouch = false;
    return;
  }

  if (!g_touchLongActionFired) {
    const uint32_t holdDuration = now - g_touchDownStartMs;
    const int16_t absDx = abs(x - g_touchDownX);
    const int16_t absDy = abs(y - g_touchDownY);
    const bool mostlyStationary = (absDx <= kTouchTapMoveThresholdPx && absDy <= kTouchTapMoveThresholdPx);

    if (holdDuration >= kTouchLongPressMs && mostlyStationary) {
      if (g_tapArmed && static_cast<int32_t>(g_tapArmDeadlineMs - now) >= 0 && !g_leftLockActive) {
        g_tapArmed = false;
        setLeftLock(true);
        g_lastGesture = "Drag Hold";
      } else if (!g_leftLockActive) {
        g_tapArmed = false;
        g_clickMode = "R-CLICK";
        sendRightClick();
        g_clickMode = "FREE";
        g_lastGesture = "Long Press";
      }
      g_touchLongActionFired = true;
    }
  }
}

void updateImuOrientationOnly() {
  ImuSample sample;
  if (!readImuSample(sample)) {
    return;
  }

  float ax = sample.ax_mg;
  float ay = sample.ay_mg;
  float az = sample.az_mg;
  if (normalize3(ax, ay, az)) {
    g_gravityX = ((1.0f - kGravityLpfAlpha) * g_gravityX) + (kGravityLpfAlpha * ax);
    g_gravityY = ((1.0f - kGravityLpfAlpha) * g_gravityY) + (kGravityLpfAlpha * ay);
    g_gravityZ = ((1.0f - kGravityLpfAlpha) * g_gravityZ) + (kGravityLpfAlpha * az);
    normalize3(g_gravityX, g_gravityY, g_gravityZ);
    maybeCaptureModeFlipReference();
  }
}

void updateAirMouse() {
  ImuSample sample;
  if (!readImuSample(sample)) {
    g_lastDeltaX = 0;
    g_lastDeltaY = 0;
    return;
  }

  const uint32_t nowUs = micros();
  float dt = (nowUs - g_lastImuSampleUs) * 1e-6f;
  g_lastImuSampleUs = nowUs;
  dt = std::max(0.002f, std::min(0.030f, dt));

  float ax = sample.ax_mg;
  float ay = sample.ay_mg;
  float az = sample.az_mg;
  if (normalize3(ax, ay, az)) {
    g_gravityX = ((1.0f - kGravityLpfAlpha) * g_gravityX) + (kGravityLpfAlpha * ax);
    g_gravityY = ((1.0f - kGravityLpfAlpha) * g_gravityY) + (kGravityLpfAlpha * ay);
    g_gravityZ = ((1.0f - kGravityLpfAlpha) * g_gravityZ) + (kGravityLpfAlpha * az);
    normalize3(g_gravityX, g_gravityY, g_gravityZ);
    maybeCaptureModeFlipReference();
  }

  constexpr float kForwardX = 0.0f;
  constexpr float kForwardY = 1.0f;
  constexpr float kForwardZ = 0.0f;

  float rx = (kForwardY * g_gravityZ) - (kForwardZ * g_gravityY);
  float ry = (kForwardZ * g_gravityX) - (kForwardX * g_gravityZ);
  float rz = (kForwardX * g_gravityY) - (kForwardY * g_gravityX);
  if (normalize3(rx, ry, rz)) {
    g_rightX = ((1.0f - kRightAxisLpfAlpha) * g_rightX) + (kRightAxisLpfAlpha * rx);
    g_rightY = ((1.0f - kRightAxisLpfAlpha) * g_rightY) + (kRightAxisLpfAlpha * ry);
    g_rightZ = ((1.0f - kRightAxisLpfAlpha) * g_rightZ) + (kRightAxisLpfAlpha * rz);
    normalize3(g_rightX, g_rightY, g_rightZ);
  }

  const float gx = (sample.gx_mdps - g_gyroBiasX) / 1000.0f;
  const float gy = (sample.gy_mdps - g_gyroBiasY) / 1000.0f;
  const float gz = (sample.gz_mdps - g_gyroBiasZ) / 1000.0f;

  float yawRate = (gx * g_gravityX) + (gy * g_gravityY) + (gz * g_gravityZ);
  float pitchRate = (gx * g_rightX) + (gy * g_rightY) + (gz * g_rightZ);
  const float rollRate = (gx * kForwardX) + (gy * kForwardY) + (gz * kForwardZ);
  yawRate = std::max(-450.0f, std::min(450.0f, yawRate));
  pitchRate = std::max(-450.0f, std::min(450.0f, pitchRate));

  if (fabsf(yawRate) < kGyroDeadzoneDps) {
    yawRate = 0.0f;
  }
  if (fabsf(pitchRate) < kGyroDeadzoneDps) {
    pitchRate = 0.0f;
  }

  g_yawRateDps = yawRate;
  g_pitchRateDps = pitchRate;
  g_rollRateDps = rollRate;

  const float rateMag = sqrtf((yawRate * yawRate) + (pitchRate * pitchRate));
  const float accelNorm = std::min(1.0f, rateMag / kRateAccelRefDps);
  const float accel = 1.0f + (kRateAccelMaxBoost * accelNorm * accelNorm);

  const float rollAbs = fabsf(rollRate);
  float rollDamp = 1.0f;
  if (rollAbs > kRollRateDampStartDps) {
    float t = (rollAbs - kRollRateDampStartDps) / (kRollRateSuppressDps - kRollRateDampStartDps);
    t = std::max(0.0f, std::min(1.0f, t));
    rollDamp = 1.0f - t;
    rollDamp *= rollDamp;
  }
  g_rollDamp = rollDamp;

  const float targetVelX = static_cast<float>(kYawSign) * yawRate * kAirMouseYawGain * accel * rollDamp;
  const float targetVelY = static_cast<float>(kPitchSign) * pitchRate * kAirMousePitchGain * accel * rollDamp;

  g_airVelX = ((1.0f - kVelocityLpfAlpha) * g_airVelX) + (kVelocityLpfAlpha * targetVelX);
  g_airVelY = ((1.0f - kVelocityLpfAlpha) * g_airVelY) + (kVelocityLpfAlpha * targetVelY);

  g_airResidualX += g_airVelX * dt;
  g_airResidualY += g_airVelY * dt;

  int sendX = 0;
  int sendY = 0;

  if (fabsf(g_airResidualX) >= 0.75f) {
    sendX = clampToInt8((int)lroundf(g_airResidualX));
    g_airResidualX -= sendX;
  }
  if (fabsf(g_airResidualY) >= 0.75f) {
    sendY = clampToInt8((int)lroundf(g_airResidualY));
    g_airResidualY -= sendY;
  }

  g_lastDeltaX = sendX;
  g_lastDeltaY = sendY;

  if (sendX != 0 || sendY != 0) {
    sendMouseReport(g_buttonMask, sendX, sendY, 0);
  }

  const uint32_t nowMs = millis();
  if ((nowMs - g_lastImuDebugMs) >= kImuDebugIntervalMs) {
    g_lastImuDebugMs = nowMs;
    Serial.printf("[imu] yaw=%.2f pitch=%.2f roll=%.2f damp=%.2f move=(%d,%d)\n",
                  g_yawRateDps, g_pitchRateDps, g_rollRateDps, g_rollDamp, sendX, sendY);
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < kSerialBootWaitMs) {
    delay(10);
  }

  Serial.println();
  Serial.println("[boot] Ringo BLE trackpad starting");
  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("[boot] reset_reason=%s (%d)\n", resetReasonName(resetReason), static_cast<int>(resetReason));
  printCommandHelp();

  initBoardPowerAndDisplay();
  initTouch();
  initImu();
  initBleMouse();

  renderStatus(true);
}

void loop() {
  const uint32_t now = millis();
  bool touchEdge = false;
  pollSerialCommands();
  serviceSetupShift();

  if (g_touchInterrupt) {
    g_touchInterrupt = false;
    g_lastTouchEventMs = now;
    touchEdge = true;
    if (g_inputMode == InputMode::Mouse) {
      handleGestureIfAny();
    }
  }

  if (g_inputMode == InputMode::Graffiti) {
    if (g_imuReady && (now - g_lastImuPollMs) >= kGraffitiImuPollMs) {
      g_lastImuPollMs = now;
      updateImuOrientationOnly();
    }
    updateGraffitiTapOnlyMode();
  }

  bool touchNeedsPolling = false;
  if (g_inputMode == InputMode::Mouse) {
    touchNeedsPolling = touchEdge || g_touchDown || g_touchActive;
  } else {
    // Keep release/tap detection reliable in both stroke and tap-only submodes.
    touchNeedsPolling = touchEdge || g_touchDown || g_touchActive;
  }

  const uint32_t touchPollMs = (g_inputMode == InputMode::Graffiti) ? kGraffitiTouchPollMs : kTouchPollMs;
  if (touchNeedsPolling && (now - g_lastTouchPollMs) >= touchPollMs) {
    g_lastTouchPollMs = now;
    if (g_inputMode == InputMode::Mouse) {
      sampleTouchState();
    } else {
      sampleGraffitiTouchState();
    }
  }

  if (g_touchActive && (now - g_lastTouchEventMs) > kTouchReleaseTimeoutMs) {
    g_touchActive = false;
  }

  if (g_inputMode == InputMode::Mouse) {
    if (g_tapArmed && static_cast<int32_t>(g_tapArmDeadlineMs - now) < 0) {
      g_tapArmed = false;
      if (!g_leftLockActive) {
        g_clickMode = "FREE";
      }
    }

    if (g_imuReady && (now - g_lastImuPollMs) >= kImuPollMs) {
      g_lastImuPollMs = now;
      updateAirMouse();
    }
  } else {
    if (g_graffitiTapArmed && (now - g_graffitiTapArmedMs) > kModeExitDoubleTapWindowMs) {
      resetGraffitiTapSwitchState();
      if (!g_touchDown) {
        g_graffitiStatus = g_graffitiTapOnlyActive ? "EXIT TAP" : "READY";
      }
    }
  }

  renderStatus();
  delay(2);
}
