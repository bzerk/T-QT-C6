#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 1
#endif
#include <Arduino.h>
#include <algorithm>
#include <esp_system.h>
#include <math.h>
#include <Preferences.h>

#include "Arduino_DriveBus_Library.h"
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "GraffitiEngine.h"

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
constexpr uint8_t kDisplayBacklightPwm = 160;
constexpr uint8_t kDisplayBacklightOffPwm = 255;
constexpr uint32_t kRejectFlashMs = 70;
constexpr uint32_t kTouchReleaseTimeoutMs = 120;
constexpr uint32_t kTouchPollMs = 12;
constexpr uint32_t kGraffitiTouchPollMs = 12;
constexpr uint32_t kTouchFaultBackoffBaseMs = 24;
constexpr uint32_t kTouchFaultBackoffMaxMs = 320;
constexpr uint32_t kImuPollMs = 8;
constexpr int32_t kI2cBusHz = 100000;
constexpr uint32_t kGraffitiImuPollMs = 35;
constexpr uint32_t kImuDebugIntervalMs = 500;
constexpr uint16_t kGraffitiStrokeMaxPoints = 180;
constexpr uint16_t kGraffitiStrokeMinPoints = 2;
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
constexpr uint32_t kGraffitiReleaseHoldMs = 36;
constexpr uint8_t kGraffitiReleaseSamples = 2;
constexpr uint8_t kGraffitiCompletedQueueDepth = 4;
constexpr uint32_t kGraffitiCommandLongPressMs = 520;
constexpr int16_t kGraffitiCommandSwipeMinDyPx = 28;
constexpr int16_t kGraffitiCommandSwipeMaxDxPx = 40;
constexpr uint32_t kArrowRepeatInitialMs = 280;
constexpr uint32_t kArrowRepeatMs = 85;
constexpr uint32_t kArrowIntroMs = 3000;
constexpr int16_t kArrowEdgeMarginPx = 26;
constexpr int16_t kArrowCenterHalfSizePx = 18;
constexpr int16_t kArrowEdgeBarThicknessPx = 6;
constexpr int16_t kCstIdleX = 60;
constexpr int16_t kCstIdleY = 150;
constexpr uint8_t kGraffitiCaptureLabelMaxLen = 16;
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

constexpr char kGyroBiasNvsNamespace[] = "ringo_imu";
constexpr char kGyroBiasNvsKey[] = "gyro_bias";
constexpr uint32_t kGyroBiasRecordMagic = 0x52424759;  // "RGBY"
constexpr uint16_t kGyroBiasRecordVersion = 1;
constexpr char kUiCfgNvsNamespace[] = "ringo_ui";
constexpr char kUiCfgNvsKey[] = "mouse_cfg";
constexpr uint32_t kUiCfgRecordMagic = 0x52435549;  // "RCUI"
constexpr uint16_t kUiCfgRecordVersion = 1;
constexpr float kDefaultScrollGain = 0.70f;
constexpr float kMinScrollGain = 0.10f;
constexpr float kMaxScrollGain = 2.00f;
constexpr uint32_t kAutoBiasMinStillMs = 3500;
constexpr uint16_t kAutoBiasMinSamples = 100;
constexpr float kAutoBiasStillRateDps = 1.1f;
constexpr float kAutoBiasStillAccelTolG = 0.10f;
constexpr float kAutoBiasApplyDeltaMdps = 80.0f;
constexpr float kAutoBiasBlendAlpha = 0.25f;
constexpr float kAutoBiasSaveDeltaMdps = 180.0f;
constexpr uint32_t kAutoBiasSaveIntervalMs = 180000;

enum class InputMode : uint8_t {
  Mouse,
  Graffiti,
};

enum class GraffitiGlyphMode : uint8_t {
  Alpha,
  Numeric,
  Arrow,
};

enum class GraffitiShiftMode : uint8_t {
  Off,
  OneShot,
  CapsLock,
};

enum class GraffitiTraceMode : uint8_t {
  Off,
  Once,
  Continuous,
};

enum class GraffitiCommandPoseAction : uint8_t {
  None,
  ToggleMode,
  ArmCtrl,
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
float g_scrollGain = kDefaultScrollGain;
float g_scrollResidual = 0.0f;

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
GraffitiEngine g_graffitiEngine;
struct QueuedGraffitiStroke {
  GraffitiEngine::Point points[kGraffitiStrokeMaxPoints];
  uint16_t timeMs[kGraffitiStrokeMaxPoints];
  uint16_t count = 0;
  uint16_t pressMs = 0;
  int16_t deltaX = 0;
  int16_t deltaY = 0;
  uint32_t traceStrokeId = 0;
  bool captureArmed = false;
  char captureLabel[kGraffitiCaptureLabelMaxLen] = {0};
};

GraffitiEngine::Point g_graffitiStroke[kGraffitiStrokeMaxPoints];
uint16_t g_graffitiStrokeTimeMs[kGraffitiStrokeMaxPoints] = {0};
uint16_t g_graffitiStrokeCount = 0;
uint16_t g_graffitiLastStrokePoints = 0;
uint32_t g_graffitiTouchStartMs = 0;
String g_graffitiText = "";
String g_graffitiStatus = "IDLE";
String g_graffitiLastMatch = "NONE";
float g_graffitiLastScore = 0.0f;
uint32_t g_graffitiLastInferUs = 0;
uint32_t g_graffitiMaxInferUs = 0;
uint64_t g_graffitiInferAccumUs = 0;
uint32_t g_graffitiInferCount = 0;
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
uint8_t g_touchReadFaultCount = 0;
uint32_t g_touchBackoffUntilMs = 0;
bool g_graffitiTapOnlyActive = false;
bool g_captureNextStrokeArmed = false;
char g_captureNextLabel[kGraffitiCaptureLabelMaxLen] = {0};
uint32_t g_captureStrokeId = 0;
GraffitiTraceMode g_graffitiTraceMode = GraffitiTraceMode::Off;
uint32_t g_graffitiTraceStrokeId = 0;
uint32_t g_activeGraffitiTraceStrokeId = 0;
QueuedGraffitiStroke g_completedGraffitiQueue[kGraffitiCompletedQueueDepth];
uint8_t g_completedGraffitiHead = 0;
uint8_t g_completedGraffitiTail = 0;
uint8_t g_completedGraffitiCount = 0;
uint32_t g_completedGraffitiDropped = 0;
bool g_keyboardReleasePending = false;
uint32_t g_keyboardReleaseDueMs = 0;
bool g_displayEnabled = false;
uint32_t g_rejectFlashUntilMs = 0;
GraffitiGlyphMode g_graffitiGlyphMode = GraffitiGlyphMode::Alpha;
GraffitiShiftMode g_graffitiShiftMode = GraffitiShiftMode::Off;
bool g_graffitiOneShotPunct = false;
bool g_graffitiOneShotCtrl = false;
bool g_ctrlPreviewActive = false;
char g_ctrlPreviewSymbol = 0;
GraffitiCommandPoseAction g_commandPosePendingAction = GraffitiCommandPoseAction::None;
uint8_t g_arrowHeldUsage = 0;
uint32_t g_arrowRepeatDueMs = 0;
uint32_t g_arrowModeEnteredMs = 0;
bool g_modeFlipRefReady = false;
uint8_t g_modeFlipRefAxis = 2;
float g_modeFlipRefSign = 1.0f;
bool g_setupShiftPending = false;
uint8_t g_setupShiftAttemptsRemaining = 0;
uint32_t g_setupShiftNextMs = 0;
uint32_t g_stationarySinceMs = 0;
uint16_t g_stationarySamples = 0;
float g_stationaryGyroSumX = 0.0f;
float g_stationaryGyroSumY = 0.0f;
float g_stationaryGyroSumZ = 0.0f;
uint32_t g_lastBiasSaveMs = 0;

void renderStatus(bool force = false);
void updateImuOrientationOnly();
struct ImuSample;
void sendMouseReport(uint8_t buttons, int8_t x, int8_t y, int8_t wheel);
void serviceKeyboardRelease();
void serviceArrowRepeat();
void tapKeyboardKeyUsage(uint8_t usage);
void updateDisplayBacklight();
bool readTouchSnapshot(int16_t &finger, int16_t &x, int16_t &y);
bool loadGyroBiasFromNvs();
bool saveGyroBiasToNvs();
bool loadUiConfigFromNvs();
bool saveUiConfigToNvs();
void resetStationaryBiasEstimator();
void serviceAutoGyroBiasCorrection(const ImuSample &sample, float gx_dps, float gy_dps, float gz_dps);
bool recalibrateGyroBias(bool persistBias);
void setCommandAck(const String &ack);
void toggleInputMode();
String graffitiSymbolLabel(char symbol);
const char *graffitiEngineLabel(GraffitiEngine::Backend engine);
bool isGraffitiSwipeDownExit(uint32_t pressDurationMs, int16_t deltaX, int16_t deltaY, uint16_t pointCount);
bool applyGraffitiSymbol(char symbol);
const char *graffitiGlyphModeName(GraffitiGlyphMode mode);
const char *graffitiShiftModeName(GraffitiShiftMode mode);
bool sendCtrlModifiedSymbol(char symbol);
void clearCtrlPreview();
void sampleGraffitiCtrlPreviewTouch();

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

struct GyroBiasRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  float biasX_mdps;
  float biasY_mdps;
  float biasZ_mdps;
};

struct UiConfigRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  float scrollGain;
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

float clampScrollGain(float gain) {
  if (!isfinite(gain)) {
    return kDefaultScrollGain;
  }
  return std::max(kMinScrollGain, std::min(kMaxScrollGain, gain));
}

void sendWheelTicks(int8_t wheel, uint8_t count) {
  if (wheel == 0 || count == 0) {
    return;
  }
  for (uint8_t i = 0; i < count; ++i) {
    sendMouseReport(g_buttonMask, 0, 0, wheel);
    delay(2);
  }
}

void emitScaledWheel(float wheelUnits) {
  if (!isfinite(wheelUnits) || fabsf(wheelUnits) < 1e-4f) {
    return;
  }

  g_scrollResidual += wheelUnits * g_scrollGain;
  const int steps = static_cast<int>(truncf(g_scrollResidual));
  if (steps == 0) {
    return;
  }

  const int clamped = std::max(-6, std::min(6, steps));
  g_scrollResidual -= static_cast<float>(clamped);
  sendWheelTicks(clamped > 0 ? 1 : -1, static_cast<uint8_t>(abs(clamped)));
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
  if (g_keyboardReleasePending) {
    sendKeyboardReport(0, 0);
    g_keyboardReleasePending = false;
  }
  sendKeyboardReport(modifier, keyUsage);
  g_keyboardReleasePending = true;
  g_keyboardReleaseDueMs = millis() + 8;
}

void serviceKeyboardRelease() {
  if (!g_keyboardReleasePending) {
    return;
  }
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - g_keyboardReleaseDueMs) < 0) {
    return;
  }
  sendKeyboardReport(0, 0);
  g_keyboardReleasePending = false;
}

void serviceArrowRepeat() {
  if (g_arrowHeldUsage == 0 || !g_touchDown) {
    return;
  }
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - g_arrowRepeatDueMs) < 0) {
    return;
  }
  tapKeyboardKeyUsage(g_arrowHeldUsage);
  g_arrowRepeatDueMs = now + kArrowRepeatMs;
}

void updateDisplayBacklight() {
  const uint32_t now = millis();
  const bool rejectFlashActive = static_cast<int32_t>(now - g_rejectFlashUntilMs) < 0;
  const bool overlayActive =
      (g_graffitiShiftMode != GraffitiShiftMode::Off) || g_graffitiOneShotPunct || rejectFlashActive;
  const uint8_t pwm = (g_displayEnabled || overlayActive) ? kDisplayBacklightPwm : kDisplayBacklightOffPwm;
  ledcWrite(LCD_BL, pwm);
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
  if (symbol >= '1' && symbol <= '9') {
    usage = static_cast<uint8_t>(0x1E + (symbol - '1'));
    return true;
  }
  if (symbol == '0') {
    usage = 0x27;
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
    case 0x1B:
      usage = 0x29;
      return true;
    case '.':
      usage = 0x37;
      return true;
    case ',':
      usage = 0x36;
      return true;
    case '-':
      usage = 0x2D;
      return true;
    case '_':
      usage = 0x2D;
      modifier = 0x02;
      return true;
    case '?':
      usage = 0x38;
      modifier = 0x02;
      return true;
    case '\'':
      usage = 0x34;
      return true;
    case '#':
      usage = 0x20;
      modifier = 0x02;
      return true;
    case '*':
      usage = 0x25;
      modifier = 0x02;
      return true;
    case '(':
      usage = 0x26;
      modifier = 0x02;
      return true;
    case ')':
      usage = 0x27;
      modifier = 0x02;
      return true;
    case '[':
      usage = 0x2F;
      return true;
    case ']':
      usage = 0x30;
      return true;
    case '{':
      usage = 0x2F;
      modifier = 0x02;
      return true;
    case '}':
      usage = 0x30;
      modifier = 0x02;
      return true;
    case '<':
      usage = 0x36;
      modifier = 0x02;
      return true;
    case '>':
      usage = 0x37;
      modifier = 0x02;
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

bool sendCtrlModifiedSymbol(char symbol) {
  if (symbol == 0x1B) {
    tapKeyboardUsage(0x01, 0x29);
    return true;
  }

  uint8_t modifier = 0;
  uint8_t usage = 0;
  if (!mapSymbolToKeyboardUsage(symbol, modifier, usage)) {
    return false;
  }
  tapKeyboardUsage(static_cast<uint8_t>(modifier | 0x01), usage);
  return true;
}

bool readTouchSnapshot(int16_t &finger, int16_t &x, int16_t &y) {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - g_touchBackoffUntilMs) < 0) {
    return false;
  }

  finger = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
  x = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  y = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  if (finger < 0 || x < 0 || y < 0) {
    g_graffitiReadFault = true;
    if (g_touchReadFaultCount < 7) {
      ++g_touchReadFaultCount;
    }
    const uint32_t backoff = std::min<uint32_t>(
        kTouchFaultBackoffMaxMs, kTouchFaultBackoffBaseMs << (g_touchReadFaultCount - 1));
    g_touchBackoffUntilMs = now + backoff;
    return false;
  }

  g_graffitiReadFault = false;
  g_touchReadFaultCount = 0;
  g_touchBackoffUntilMs = 0;
  return true;
}

void tapKeyboardKeyUsage(uint8_t usage) {
  tapKeyboardUsage(0, usage);
}

bool sendEscKey() {
  tapKeyboardKeyUsage(0x29);
  return true;
}

void clearCtrlPreview() {
  g_ctrlPreviewActive = false;
  g_ctrlPreviewSymbol = 0;
}

void setGraffitiGlyphMode(GraffitiGlyphMode mode) {
  g_graffitiGlyphMode = mode;
  if (mode == GraffitiGlyphMode::Arrow) {
    g_graffitiOneShotPunct = false;
    g_arrowModeEnteredMs = millis();
  } else {
    g_arrowHeldUsage = 0;
    g_arrowModeEnteredMs = 0;
  }
}

uint8_t arrowUsageForPoint(int16_t x, int16_t y) {
  if (x < kArrowEdgeMarginPx) {
    return 0x50;
  }
  if (x > (LCD_WIDTH - kArrowEdgeMarginPx)) {
    return 0x4F;
  }
  if (y < kArrowEdgeMarginPx) {
    return 0x52;
  }
  if (y > (LCD_HEIGHT - kArrowEdgeMarginPx)) {
    return 0x51;
  }
  if (abs(x - (LCD_WIDTH / 2)) <= kArrowCenterHalfSizePx &&
      abs(y - (LCD_HEIGHT / 2)) <= kArrowCenterHalfSizePx) {
    return 0x28;
  }
  return 0;
}

GraffitiEngine::Condition activeGraffitiCondition() {
  if (g_graffitiOneShotPunct) {
    return GraffitiEngine::Condition::Punct;
  }
  if (g_graffitiGlyphMode == GraffitiGlyphMode::Numeric) {
    return GraffitiEngine::Condition::Numeric;
  }
  return GraffitiEngine::Condition::Letters;
}

char applyGraffitiShift(char symbol) {
  if (symbol < 'a' || symbol > 'z') {
    return symbol;
  }
  if (g_graffitiShiftMode == GraffitiShiftMode::CapsLock ||
      g_graffitiShiftMode == GraffitiShiftMode::OneShot) {
    return static_cast<char>('A' + (symbol - 'a'));
  }
  return symbol;
}

void consumeGraffitiOneShotModes(char symbol) {
  if (g_graffitiOneShotPunct) {
    g_graffitiOneShotPunct = false;
  }
  if (g_graffitiShiftMode == GraffitiShiftMode::OneShot && symbol != 0x0F) {
    g_graffitiShiftMode = GraffitiShiftMode::Off;
  }
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

bool saveGyroBiasToNvs() {
  Preferences prefs;
  if (!prefs.begin(kGyroBiasNvsNamespace, false)) {
    Serial.println("[imu] NVS open failed (write)");
    return false;
  }

  GyroBiasRecord record = {
      kGyroBiasRecordMagic,
      kGyroBiasRecordVersion,
      0,
      g_gyroBiasX,
      g_gyroBiasY,
      g_gyroBiasZ,
  };
  const size_t written = prefs.putBytes(kGyroBiasNvsKey, &record, sizeof(record));
  prefs.end();
  if (written != sizeof(record)) {
    Serial.println("[imu] NVS save failed");
    return false;
  }

  g_lastBiasSaveMs = millis();
  Serial.printf("[imu] Bias saved to NVS: x=%.2f y=%.2f z=%.2f\n", g_gyroBiasX, g_gyroBiasY, g_gyroBiasZ);
  return true;
}

bool loadGyroBiasFromNvs() {
  Preferences prefs;
  if (!prefs.begin(kGyroBiasNvsNamespace, true)) {
    Serial.println("[imu] NVS open failed (read)");
    return false;
  }

  const size_t storedLen = prefs.getBytesLength(kGyroBiasNvsKey);
  if (storedLen != sizeof(GyroBiasRecord)) {
    prefs.end();
    return false;
  }

  GyroBiasRecord record;
  const size_t readLen = prefs.getBytes(kGyroBiasNvsKey, &record, sizeof(record));
  prefs.end();
  if (readLen != sizeof(record)) {
    return false;
  }

  if (record.magic != kGyroBiasRecordMagic || record.version != kGyroBiasRecordVersion) {
    return false;
  }
  if (!isfinite(record.biasX_mdps) || !isfinite(record.biasY_mdps) || !isfinite(record.biasZ_mdps)) {
    return false;
  }
  if (fabsf(record.biasX_mdps) > 20000.0f || fabsf(record.biasY_mdps) > 20000.0f ||
      fabsf(record.biasZ_mdps) > 20000.0f) {
    return false;
  }

  g_gyroBiasX = record.biasX_mdps;
  g_gyroBiasY = record.biasY_mdps;
  g_gyroBiasZ = record.biasZ_mdps;
  g_lastBiasSaveMs = millis();
  Serial.printf("[imu] Bias loaded from NVS: x=%.2f y=%.2f z=%.2f\n", g_gyroBiasX, g_gyroBiasY, g_gyroBiasZ);
  return true;
}

bool saveUiConfigToNvs() {
  Preferences prefs;
  if (!prefs.begin(kUiCfgNvsNamespace, false)) {
    Serial.println("[cfg] NVS open failed (write)");
    return false;
  }

  UiConfigRecord record = {
      kUiCfgRecordMagic,
      kUiCfgRecordVersion,
      0,
      clampScrollGain(g_scrollGain),
  };
  const size_t written = prefs.putBytes(kUiCfgNvsKey, &record, sizeof(record));
  prefs.end();
  if (written != sizeof(record)) {
    Serial.println("[cfg] NVS save failed");
    return false;
  }

  Serial.printf("[cfg] saved scroll_gain=%.2f\n", record.scrollGain);
  return true;
}

bool loadUiConfigFromNvs() {
  Preferences prefs;
  if (!prefs.begin(kUiCfgNvsNamespace, true)) {
    Serial.println("[cfg] NVS open failed (read)");
    return false;
  }

  const size_t storedLen = prefs.getBytesLength(kUiCfgNvsKey);
  if (storedLen != sizeof(UiConfigRecord)) {
    prefs.end();
    return false;
  }

  UiConfigRecord record;
  const size_t readLen = prefs.getBytes(kUiCfgNvsKey, &record, sizeof(record));
  prefs.end();
  if (readLen != sizeof(record)) {
    return false;
  }

  if (record.magic != kUiCfgRecordMagic || record.version != kUiCfgRecordVersion) {
    return false;
  }
  if (!isfinite(record.scrollGain)) {
    return false;
  }

  g_scrollGain = clampScrollGain(record.scrollGain);
  Serial.printf("[cfg] loaded scroll_gain=%.2f\n", g_scrollGain);
  return true;
}

void resetStationaryBiasEstimator() {
  g_stationarySinceMs = 0;
  g_stationarySamples = 0;
  g_stationaryGyroSumX = 0.0f;
  g_stationaryGyroSumY = 0.0f;
  g_stationaryGyroSumZ = 0.0f;
}

void resetCompletedGraffitiQueue() {
  g_completedGraffitiHead = 0;
  g_completedGraffitiTail = 0;
  g_completedGraffitiCount = 0;
}

void serviceAutoGyroBiasCorrection(const ImuSample &sample, float gx_dps, float gy_dps, float gz_dps) {
  const float accelMagG = sqrtf((sample.ax_mg * sample.ax_mg) +
                                (sample.ay_mg * sample.ay_mg) +
                                (sample.az_mg * sample.az_mg)) /
                          1000.0f;
  const bool still = fabsf(gx_dps) <= kAutoBiasStillRateDps &&
                     fabsf(gy_dps) <= kAutoBiasStillRateDps &&
                     fabsf(gz_dps) <= kAutoBiasStillRateDps &&
                     fabsf(accelMagG - 1.0f) <= kAutoBiasStillAccelTolG &&
                     !g_touchDown && !g_touchActive;
  if (!still) {
    resetStationaryBiasEstimator();
    return;
  }

  const uint32_t now = millis();
  if (g_stationarySinceMs == 0) {
    g_stationarySinceMs = now;
    g_stationarySamples = 0;
    g_stationaryGyroSumX = 0.0f;
    g_stationaryGyroSumY = 0.0f;
    g_stationaryGyroSumZ = 0.0f;
  }

  g_stationaryGyroSumX += sample.gx_mdps;
  g_stationaryGyroSumY += sample.gy_mdps;
  g_stationaryGyroSumZ += sample.gz_mdps;
  if (g_stationarySamples < 65535) {
    g_stationarySamples++;
  }

  if ((now - g_stationarySinceMs) < kAutoBiasMinStillMs || g_stationarySamples < kAutoBiasMinSamples) {
    return;
  }

  const float candidateX = g_stationaryGyroSumX / g_stationarySamples;
  const float candidateY = g_stationaryGyroSumY / g_stationarySamples;
  const float candidateZ = g_stationaryGyroSumZ / g_stationarySamples;
  const float dx = candidateX - g_gyroBiasX;
  const float dy = candidateY - g_gyroBiasY;
  const float dz = candidateZ - g_gyroBiasZ;
  const float deltaMag = sqrtf((dx * dx) + (dy * dy) + (dz * dz));
  if (deltaMag < kAutoBiasApplyDeltaMdps) {
    resetStationaryBiasEstimator();
    return;
  }

  g_gyroBiasX += dx * kAutoBiasBlendAlpha;
  g_gyroBiasY += dy * kAutoBiasBlendAlpha;
  g_gyroBiasZ += dz * kAutoBiasBlendAlpha;
  Serial.printf("[imu] Auto-bias trim %.1f mdps -> x=%.2f y=%.2f z=%.2f\n", deltaMag,
                g_gyroBiasX, g_gyroBiasY, g_gyroBiasZ);

  if (deltaMag >= kAutoBiasSaveDeltaMdps &&
      (now - g_lastBiasSaveMs) >= kAutoBiasSaveIntervalMs) {
    saveGyroBiasToNvs();
  }
  resetStationaryBiasEstimator();
}

const char *modeName(InputMode mode) {
  return (mode == InputMode::Graffiti) ? "GRAFFITI" : "MOUSE";
}

const char *graffitiTraceModeName(GraffitiTraceMode mode) {
  switch (mode) {
    case GraffitiTraceMode::Once:
      return "capture";
    case GraffitiTraceMode::Continuous:
      return "continuous";
    default:
      return "stop";
  }
}

const char *graffitiGlyphModeName(GraffitiGlyphMode mode) {
  switch (mode) {
    case GraffitiGlyphMode::Numeric:
      return "NUM";
    case GraffitiGlyphMode::Arrow:
      return "ARR";
    default:
      return "ALPHA";
  }
}

const char *graffitiShiftModeName(GraffitiShiftMode mode) {
  switch (mode) {
    case GraffitiShiftMode::OneShot:
      return "1X";
    case GraffitiShiftMode::CapsLock:
      return "CAPS";
    default:
      return "OFF";
  }
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

bool isValidCaptureLabel(const String &label) {
  if (label.isEmpty() || label.length() >= kGraffitiCaptureLabelMaxLen) {
    return false;
  }
  for (uint16_t i = 0; i < label.length(); ++i) {
    const char c = label.charAt(i);
    if (!isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-') {
      return false;
    }
  }
  return true;
}

void disarmGraffitiCapture() {
  g_captureNextStrokeArmed = false;
  g_captureNextLabel[0] = '\0';
}

void armGraffitiCapture(const String &label) {
  const size_t copyLen = std::min(static_cast<size_t>(label.length()),
                                  static_cast<size_t>(kGraffitiCaptureLabelMaxLen - 1));
  memcpy(g_captureNextLabel, label.c_str(), copyLen);
  g_captureNextLabel[copyLen] = '\0';
  g_captureNextStrokeArmed = true;
}

bool graffitiTraceActive() {
  return g_graffitiTraceMode != GraffitiTraceMode::Off;
}

void setGraffitiTraceMode(GraffitiTraceMode mode) {
  g_graffitiTraceMode = mode;
  g_activeGraffitiTraceStrokeId = 0;
}

void beginGraffitiTraceStroke() {
  if (!graffitiTraceActive()) {
    return;
  }
  ++g_graffitiTraceStrokeId;
  g_activeGraffitiTraceStrokeId = g_graffitiTraceStrokeId;
  Serial.printf("[trace] begin id=%lu mode=%s\n",
                static_cast<unsigned long>(g_activeGraffitiTraceStrokeId),
                graffitiTraceModeName(g_graffitiTraceMode));
}

void emitGraffitiTracePoint(int16_t x, int16_t y, uint16_t t_ms) {
  if (!graffitiTraceActive() || g_activeGraffitiTraceStrokeId == 0) {
    return;
  }
  Serial.printf("[trace] point id=%lu x=%d y=%d t=%u\n",
                static_cast<unsigned long>(g_activeGraffitiTraceStrokeId),
                static_cast<int>(x), static_cast<int>(y), static_cast<unsigned>(t_ms));
}

void populateQueuedGraffitiStroke(QueuedGraffitiStroke &out) {
  out.count = g_graffitiStrokeCount;
  out.pressMs = g_graffitiLastPressMs;
  out.deltaX = g_graffitiLastDeltaX;
  out.deltaY = g_graffitiLastDeltaY;
  out.traceStrokeId = g_activeGraffitiTraceStrokeId;
  out.captureArmed = g_captureNextStrokeArmed;
  if (out.captureArmed) {
    strlcpy(out.captureLabel, g_captureNextLabel, sizeof(out.captureLabel));
  } else {
    out.captureLabel[0] = '\0';
  }
  for (uint16_t i = 0; i < out.count; ++i) {
    out.points[i] = g_graffitiStroke[i];
    out.timeMs[i] = g_graffitiStrokeTimeMs[i];
  }
}

bool enqueueGraffitiStroke(const QueuedGraffitiStroke &stroke) {
  if (g_completedGraffitiCount >= kGraffitiCompletedQueueDepth) {
    ++g_completedGraffitiDropped;
    return false;
  }
  g_completedGraffitiQueue[g_completedGraffitiTail] = stroke;
  g_completedGraffitiTail = (g_completedGraffitiTail + 1) % kGraffitiCompletedQueueDepth;
  ++g_completedGraffitiCount;
  return true;
}

bool dequeueGraffitiStroke(QueuedGraffitiStroke &out) {
  if (g_completedGraffitiCount == 0) {
    return false;
  }
  out = g_completedGraffitiQueue[g_completedGraffitiHead];
  g_completedGraffitiHead = (g_completedGraffitiHead + 1) % kGraffitiCompletedQueueDepth;
  --g_completedGraffitiCount;
  return true;
}

void endGraffitiTraceStroke(const char *status, const GraffitiEngine::Result *result) {
  if (!graffitiTraceActive() || g_activeGraffitiTraceStrokeId == 0) {
    return;
  }

  String predicted = "-";
  float confidence = 0.0f;
  float distance = 0.0f;
  bool accepted = false;
  uint8_t tokenCount = 0;
  const char *backend = "NONE";
  const char *seq = g_graffitiEngine.lastTokenSequence();
  if (result != nullptr) {
    predicted = graffitiSymbolLabel(result->symbol);
    confidence = result->confidence;
    distance = result->rawDistance;
    accepted = result->accepted;
    tokenCount = result->tokenCount;
    backend = graffitiEngineLabel(result->backend);
  }

  Serial.printf("[trace] end id=%lu status=%s pred=%s accepted=%u score=%.2f dist=%.2f backend=%s tok=%u seq=%s points=%u dur=%u dx=%d dy=%d\n",
                static_cast<unsigned long>(g_activeGraffitiTraceStrokeId), status,
                predicted.c_str(), accepted ? 1U : 0U, confidence, distance, backend,
                static_cast<unsigned>(tokenCount), seq, static_cast<unsigned>(g_graffitiStrokeCount),
                static_cast<unsigned>(g_graffitiLastPressMs), g_graffitiLastDeltaX, g_graffitiLastDeltaY);

  g_activeGraffitiTraceStrokeId = 0;
  if (g_graffitiTraceMode == GraffitiTraceMode::Once) {
    g_graffitiTraceMode = GraffitiTraceMode::Off;
    setCommandAck("trace=stop");
  }
}

void endQueuedGraffitiTraceStroke(const QueuedGraffitiStroke &stroke, const char *status,
                                  const GraffitiEngine::Result *result) {
  if (!graffitiTraceActive() || stroke.traceStrokeId == 0) {
    return;
  }

  String predicted = "-";
  float confidence = 0.0f;
  float distance = 0.0f;
  bool accepted = false;
  uint8_t tokenCount = 0;
  const char *backend = "NONE";
  const char *seq = g_graffitiEngine.lastTokenSequence();
  if (result != nullptr) {
    predicted = graffitiSymbolLabel(result->symbol);
    confidence = result->confidence;
    distance = result->rawDistance;
    accepted = result->accepted;
    tokenCount = result->tokenCount;
    backend = graffitiEngineLabel(result->backend);
  }

  Serial.printf("[trace] end id=%lu status=%s pred=%s accepted=%u score=%.2f dist=%.2f backend=%s tok=%u seq=%s points=%u dur=%u dx=%d dy=%d\n",
                static_cast<unsigned long>(stroke.traceStrokeId), status, predicted.c_str(),
                accepted ? 1U : 0U, confidence, distance, backend, static_cast<unsigned>(tokenCount),
                seq, static_cast<unsigned>(stroke.count), static_cast<unsigned>(stroke.pressMs),
                stroke.deltaX, stroke.deltaY);

  if (g_graffitiTraceMode == GraffitiTraceMode::Once) {
    g_graffitiTraceMode = GraffitiTraceMode::Off;
    setCommandAck("trace=stop");
  }
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
  g_activeGraffitiTraceStrokeId = 0;
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
  // Once a command-pose touch begins, hold the pose state until release.
  // This avoids IMU/touch contention and prevents de-inverting mid-touch from
  // collapsing the gesture path before release handling runs.
  if (g_graffitiTapOnlyActive && (g_touchDown || g_touchActive)) {
    return;
  }

  const float projection = graffitiInvertProjection();
  const bool nextTapOnly =
      g_graffitiTapOnlyActive ? (projection <= kGraffitiInvertExitThreshold)
                              : (projection <= kGraffitiInvertThreshold);

  if (nextTapOnly == g_graffitiTapOnlyActive) {
    return;
  }

  GraffitiCommandPoseAction exitAction = GraffitiCommandPoseAction::None;
  if (g_graffitiTapOnlyActive && !nextTapOnly) {
    if (g_commandPosePendingAction != GraffitiCommandPoseAction::None) {
      exitAction = g_commandPosePendingAction;
    } else if (g_touchDown) {
      const uint32_t now = millis();
      const uint32_t pressDuration = now - g_touchDownStartMs;
      const int16_t deltaX = g_touchX - g_touchDownX;
      const int16_t deltaY = g_touchY - g_touchDownY;
      if (pressDuration >= kGraffitiCommandLongPressMs &&
          abs(deltaX) <= kTouchTapMoveThresholdPx &&
          abs(deltaY) <= kTouchTapMoveThresholdPx) {
        exitAction = GraffitiCommandPoseAction::ToggleMode;
      }
    }
  }

  g_graffitiTapOnlyActive = nextTapOnly;
  resetGraffitiStrokeState();
  resetGraffitiTapSwitchState();
  g_touchDown = false;
  g_touchActive = false;
  g_touchLongActionFired = false;
  g_swipeHandledThisTouch = false;
  g_tapArmed = false;

  if (g_leftLockActive) {
    setLeftLock(false);
  }

  if (g_graffitiTapOnlyActive) {
    g_graffitiStatus = "CMD TAP";
    Serial.printf("[mode] command pose ON (proj=%.2f)\n", projection);
  } else {
    if (exitAction == GraffitiCommandPoseAction::ToggleMode) {
      g_graffitiStatus = (g_inputMode == InputMode::Mouse) ? "MODE->GRAF" : "MODE->MOUSE";
      toggleInputMode();
    } else if (exitAction == GraffitiCommandPoseAction::ArmCtrl) {
      g_graffitiOneShotCtrl = true;
      g_graffitiStatus = "CTRL 1X";
    } else if (g_inputMode == InputMode::Graffiti) {
      g_graffitiStatus = "READY";
    } else {
      g_graffitiStatus = "IDLE";
    }
    g_commandPosePendingAction = GraffitiCommandPoseAction::None;
    if (g_inputMode == InputMode::Graffiti) {
      resetGraffitiStrokeState();
    }
    Serial.printf("[mode] command pose OFF (proj=%.2f)\n", projection);
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
  g_scrollResidual = 0.0f;
  g_lastDeltaX = 0;
  g_lastDeltaY = 0;
  g_arrowHeldUsage = 0;
  resetCompletedGraffitiQueue();
  g_keyboardReleasePending = false;
  g_ctrlPreviewActive = false;
  g_ctrlPreviewSymbol = 0;
  g_commandPosePendingAction = GraffitiCommandPoseAction::None;
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
    setGraffitiGlyphMode(GraffitiGlyphMode::Alpha);
    g_graffitiShiftMode = GraffitiShiftMode::Off;
    g_graffitiOneShotPunct = false;
    g_graffitiOneShotCtrl = false;
    g_graffitiStatus = "READY";
    g_graffitiLastMatch = "NONE";
    g_graffitiLastScore = 0.0f;
    resetGraffitiStrokeState();
    resetGraffitiTapSwitchState();
  } else {
    g_graffitiTapOnlyActive = false;
    setGraffitiGlyphMode(GraffitiGlyphMode::Alpha);
    g_graffitiShiftMode = GraffitiShiftMode::Off;
    g_graffitiOneShotPunct = false;
    g_graffitiOneShotCtrl = false;
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

void toggleInputMode() {
  if (g_inputMode == InputMode::Mouse) {
    setInputMode(InputMode::Graffiti);
  } else {
    setInputMode(InputMode::Mouse);
  }
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
  Serial.println("[cmd] g|m|mode graffiti|mode mouse|mode?|status|cal|scroll?|scroll <0.10..2.00>|recog?|recog reset|recog thr|recog thr <0.005..0.050>|glyph?|glyph letters|glyph punct|glyph numeric|trace?|trace off|trace once|trace cont|cap?|cap off|cap <label>|shift|ping|help");
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
    Serial.printf("[cmd] bias x=%.2f y=%.2f z=%.2f\n", g_gyroBiasX, g_gyroBiasY, g_gyroBiasZ);
    Serial.printf("[cmd] scroll_gain=%.2f residual=%.2f\n", g_scrollGain, g_scrollResidual);
    const unsigned long inferAvgUs =
        (g_graffitiInferCount == 0) ? 0UL
                                    : static_cast<unsigned long>(g_graffitiInferAccumUs / g_graffitiInferCount);
    Serial.printf("[cmd] recognizer=%s cond=%s glyph=%s shift=%s punct1x=%u thr=%.3f accept=%lu reject=%lu lastTok=%u seq=%s infer_us=%lu avg_us=%lu max_us=%lu\n",
                  "PROTOTYPE", g_graffitiEngine.conditionName(),
                  graffitiGlyphModeName(g_graffitiGlyphMode), graffitiShiftModeName(g_graffitiShiftMode),
                  g_graffitiOneShotPunct ? 1U : 0U,
                  g_graffitiEngine.acceptConfidence(),
                  static_cast<unsigned long>(g_graffitiEngine.trieAcceptCount()),
                  static_cast<unsigned long>(g_graffitiEngine.pointAcceptCount()),
                  g_graffitiEngine.lastTokenCount(),
                  g_graffitiEngine.lastTokenSequence(),
                  static_cast<unsigned long>(g_graffitiLastInferUs),
                  inferAvgUs,
                  static_cast<unsigned long>(g_graffitiMaxInferUs));
    Serial.printf("[cmd] trace=%s active_id=%lu\n", graffitiTraceModeName(g_graffitiTraceMode),
                  static_cast<unsigned long>(g_activeGraffitiTraceStrokeId));
    Serial.printf("[cmd] capture=%s label=%s\n", g_captureNextStrokeArmed ? "armed" : "off",
                  g_captureNextStrokeArmed ? g_captureNextLabel : "-");
    Serial.printf("[cmd] queue depth=%u dropped=%lu keyrel=%u\n",
                  static_cast<unsigned>(g_completedGraffitiCount),
                  static_cast<unsigned long>(g_completedGraffitiDropped),
                  g_keyboardReleasePending ? 1U : 0U);
    return;
  }
  if (cmd == "cal" || cmd == "calibrate" || cmd == "imu cal" || cmd == "bias cal") {
    const bool ok = recalibrateGyroBias(true);
    setCommandAck(ok ? "ok cal" : "err cal");
    return;
  }
  if (cmd == "scroll" || cmd == "scroll?" || cmd == "scroll get") {
    setCommandAck(String("scroll=") + String(g_scrollGain, 2));
    return;
  }
  if (cmd.startsWith("scroll ")) {
    String arg = cmd.substring(7);
    arg.trim();
    const float parsed = arg.toFloat();
    if (!arg.length() || (parsed == 0.0f && arg != "0" && arg != "0.0")) {
      setCommandAck("err scroll parse");
      return;
    }

    g_scrollGain = clampScrollGain(parsed);
    g_scrollResidual = 0.0f;
    const bool saved = saveUiConfigToNvs();
    setCommandAck(String("ok scroll=") + String(g_scrollGain, 2) + (saved ? "" : " (nosave)"));
    return;
  }
  if (cmd == "recog" || cmd == "recog?" || cmd == "recog mode") {
    setCommandAck(String("recog=prototype cond=") + g_graffitiEngine.conditionName());
    const unsigned long inferAvgUs =
        (g_graffitiInferCount == 0) ? 0UL
                                    : static_cast<unsigned long>(g_graffitiInferAccumUs / g_graffitiInferCount);
    Serial.printf("[cmd] accept=%lu reject=%lu lastTok=%u seq=%s cond=%s glyph=%s shift=%s punct1x=%u thr=%.3f infer_us=%lu avg_us=%lu max_us=%lu q=%u drop=%lu\n",
                  static_cast<unsigned long>(g_graffitiEngine.trieAcceptCount()),
                  static_cast<unsigned long>(g_graffitiEngine.pointAcceptCount()),
                  g_graffitiEngine.lastTokenCount(),
                  g_graffitiEngine.lastTokenSequence(),
                  g_graffitiEngine.conditionName(),
                  graffitiGlyphModeName(g_graffitiGlyphMode), graffitiShiftModeName(g_graffitiShiftMode),
                  g_graffitiOneShotPunct ? 1U : 0U,
                  g_graffitiEngine.acceptConfidence(),
                  static_cast<unsigned long>(g_graffitiLastInferUs),
                  inferAvgUs,
                  static_cast<unsigned long>(g_graffitiMaxInferUs),
                  static_cast<unsigned>(g_completedGraffitiCount),
                  static_cast<unsigned long>(g_completedGraffitiDropped));
    return;
  }
  if (cmd == "recog reset") {
    g_graffitiEngine.resetStats();
    g_graffitiLastInferUs = 0;
    g_graffitiMaxInferUs = 0;
    g_graffitiInferAccumUs = 0;
    g_graffitiInferCount = 0;
    setCommandAck("ok recog=prototype");
    return;
  }
  if (cmd == "recog thr" || cmd == "recog threshold" || cmd == "thr") {
    setCommandAck(String("recog thr=") + String(g_graffitiEngine.acceptConfidence(), 3));
    return;
  }
  if (cmd.startsWith("recog thr ")) {
    String arg = cmd.substring(10);
    arg.trim();
    const float parsed = arg.toFloat();
    if (!arg.length() || (parsed == 0.0f && arg != "0" && arg != "0.0")) {
      setCommandAck("err recog thr parse");
      return;
    }
    const float threshold = constrain(parsed, 0.005f, 0.050f);
    g_graffitiEngine.setAcceptConfidence(threshold);
    setCommandAck(String("ok recog thr=") + String(g_graffitiEngine.acceptConfidence(), 3));
    return;
  }
  if (cmd == "glyph" || cmd == "glyph?" || cmd == "cond" || cmd == "cond?") {
    setCommandAck(String("ok glyph=") + g_graffitiEngine.conditionName());
    return;
  }
  if (cmd == "glyph letters" || cmd == "cond letters" || cmd == "letters") {
    g_graffitiEngine.setCondition(GraffitiEngine::Condition::Letters);
    setCommandAck("ok glyph=letters");
    return;
  }
  if (cmd == "glyph punct" || cmd == "cond punct" || cmd == "punct") {
    g_graffitiEngine.setCondition(GraffitiEngine::Condition::Punct);
    setCommandAck("ok glyph=punct");
    return;
  }
  if (cmd == "glyph numeric" || cmd == "cond numeric" || cmd == "numeric") {
    g_graffitiEngine.setCondition(GraffitiEngine::Condition::Numeric);
    setCommandAck("ok glyph=numeric");
    return;
  }
  if (cmd == "trace" || cmd == "trace?" || cmd == "stream" || cmd == "stream?") {
    setCommandAck(String("trace=") + graffitiTraceModeName(g_graffitiTraceMode));
    Serial.printf("[cmd] trace=%s active_id=%lu\n", graffitiTraceModeName(g_graffitiTraceMode),
                  static_cast<unsigned long>(g_activeGraffitiTraceStrokeId));
    return;
  }
  if (cmd == "trace off" || cmd == "stream off" || cmd == "stop") {
    setGraffitiTraceMode(GraffitiTraceMode::Off);
    setCommandAck("trace=stop");
    return;
  }
  if (cmd == "trace once" || cmd == "stream once" || cmd == "capture once") {
    setGraffitiTraceMode(GraffitiTraceMode::Once);
    setCommandAck("trace=capture");
    return;
  }
  if (cmd == "trace cont" || cmd == "trace continuous" || cmd == "stream cont" ||
      cmd == "stream continuous" || cmd == "continuous") {
    setGraffitiTraceMode(GraffitiTraceMode::Continuous);
    setCommandAck("trace=continuous");
    return;
  }
  if (cmd == "cap" || cmd == "cap?" || cmd == "capture" || cmd == "capture?") {
    setCommandAck(String("capture=") + (g_captureNextStrokeArmed ? "armed" : "off"));
    Serial.printf("[cmd] capture=%s label=%s\n", g_captureNextStrokeArmed ? "armed" : "off",
                  g_captureNextStrokeArmed ? g_captureNextLabel : "-");
    return;
  }
  if (cmd == "cap off" || cmd == "capture off") {
    disarmGraffitiCapture();
    setCommandAck("ok capture off");
    return;
  }
  if (cmd.startsWith("cap ") || cmd.startsWith("capture ")) {
    const uint16_t prefixLen = cmd.startsWith("capture ") ? 8 : 4;
    String arg = cmd.substring(prefixLen);
    arg.trim();
    if (arg == "off") {
      disarmGraffitiCapture();
      setCommandAck("ok capture off");
      return;
    }
    if (!isValidCaptureLabel(arg)) {
      setCommandAck("err cap label");
      return;
    }
    armGraffitiCapture(arg);
    setCommandAck(String("ok cap=") + arg);
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
  if (symbol == '\n' || symbol == '\r') {
    return "RET";
  }
  if (symbol == 0x1B) {
    return "ESC";
  }
  if (symbol == 0x0F) {
    return "SHIFT";
  }
  char buf[2] = {symbol, '\0'};
  return String(buf);
}

const char *graffitiEngineLabel(GraffitiEngine::Backend engine) {
  switch (engine) {
    case GraffitiEngine::Backend::Prototype:
      return "PROTO";
    default:
      return "NONE";
  }
}

bool applyGraffitiSymbol(char symbol) {
  if (g_graffitiOneShotPunct && symbol == '\b') {
    g_graffitiStatus = "PUNCT OFF";
    return true;
  }
  if (symbol == 0x0F) {
    if (g_graffitiShiftMode == GraffitiShiftMode::Off) {
      g_graffitiShiftMode = GraffitiShiftMode::OneShot;
      g_graffitiStatus = "SHIFT 1X";
    } else if (g_graffitiShiftMode == GraffitiShiftMode::OneShot) {
      g_graffitiShiftMode = GraffitiShiftMode::CapsLock;
      g_graffitiStatus = "SHIFT CAPS";
    } else {
      g_graffitiShiftMode = GraffitiShiftMode::Off;
      g_graffitiStatus = "SHIFT OFF";
    }
    return true;
  }
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

void emitGraffitiCapture(const char *status, const GraffitiEngine::Result *result) {
  if (!g_captureNextStrokeArmed) {
    return;
  }

  ++g_captureStrokeId;
  String predicted = "-";
  float confidence = 0.0f;
  float distance = 0.0f;
  bool accepted = false;
  uint8_t tokenCount = 0;
  const char *backend = "NONE";
  const char *seq = g_graffitiEngine.lastTokenSequence();
  if (result != nullptr) {
    predicted = graffitiSymbolLabel(result->symbol);
    confidence = result->confidence;
    distance = result->rawDistance;
    accepted = result->accepted;
    tokenCount = result->tokenCount;
    backend = graffitiEngineLabel(result->backend);
  }

  Serial.printf("[cap] id=%lu label=%s status=%s pred=%s accepted=%u score=%.2f dist=%.2f backend=%s tok=%u seq=%s points=%u dur=%u dx=%d dy=%d pts=",
                static_cast<unsigned long>(g_captureStrokeId), g_captureNextLabel, status,
                predicted.c_str(), accepted ? 1U : 0U, confidence, distance, backend,
                static_cast<unsigned>(tokenCount), seq, static_cast<unsigned>(g_graffitiStrokeCount),
                static_cast<unsigned>(g_graffitiLastPressMs), g_graffitiLastDeltaX, g_graffitiLastDeltaY);
  for (uint16_t i = 0; i < g_graffitiStrokeCount; ++i) {
    Serial.printf("%s[%d,%d,%u]", (i == 0) ? "" : ",",
                  static_cast<int>(g_graffitiStroke[i].x),
                  static_cast<int>(g_graffitiStroke[i].y),
                  static_cast<unsigned>(g_graffitiStrokeTimeMs[i]));
  }
  Serial.println();
  disarmGraffitiCapture();
}

void emitQueuedGraffitiCapture(const QueuedGraffitiStroke &stroke, const char *status,
                               const GraffitiEngine::Result *result) {
  if (!stroke.captureArmed) {
    return;
  }

  ++g_captureStrokeId;
  String predicted = "-";
  float confidence = 0.0f;
  float distance = 0.0f;
  bool accepted = false;
  uint8_t tokenCount = 0;
  const char *backend = "NONE";
  const char *seq = g_graffitiEngine.lastTokenSequence();
  if (result != nullptr) {
    predicted = graffitiSymbolLabel(result->symbol);
    confidence = result->confidence;
    distance = result->rawDistance;
    accepted = result->accepted;
    tokenCount = result->tokenCount;
    backend = graffitiEngineLabel(result->backend);
  }

  Serial.printf("[cap] id=%lu label=%s status=%s pred=%s accepted=%u score=%.2f dist=%.2f backend=%s tok=%u seq=%s points=%u dur=%u dx=%d dy=%d pts=",
                static_cast<unsigned long>(g_captureStrokeId), stroke.captureLabel, status,
                predicted.c_str(), accepted ? 1U : 0U, confidence, distance, backend,
                static_cast<unsigned>(tokenCount), seq, static_cast<unsigned>(stroke.count),
                static_cast<unsigned>(stroke.pressMs), stroke.deltaX, stroke.deltaY);
  for (uint16_t i = 0; i < stroke.count; ++i) {
    Serial.printf("%s[%d,%d,%u]", (i == 0) ? "" : ",",
                  static_cast<int>(stroke.points[i].x),
                  static_cast<int>(stroke.points[i].y),
                  static_cast<unsigned>(stroke.timeMs[i]));
  }
  Serial.println();
}

void graffitiAddPoint(int16_t x, int16_t y, uint32_t now) {
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
  g_graffitiStrokeTimeMs[g_graffitiStrokeCount] =
      (g_graffitiTouchStartMs == 0) ? 0
                                    : static_cast<uint16_t>(std::min<uint32_t>(65535, now - g_graffitiTouchStartMs));
  emitGraffitiTracePoint(x, y, g_graffitiStrokeTimeMs[g_graffitiStrokeCount]);
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
    endGraffitiTraceStroke("SHORT", nullptr);
    emitGraffitiCapture("SHORT", nullptr);
    resetGraffitiTapSwitchState();
    resetGraffitiStrokeState();
    return;
  }

  QueuedGraffitiStroke completed;
  populateQueuedGraffitiStroke(completed);
  disarmGraffitiCapture();
  resetGraffitiTapSwitchState();
  resetGraffitiStrokeState();
  if (enqueueGraffitiStroke(completed)) {
    g_graffitiStatus = "QUEUED";
    return;
  }

  g_graffitiStatus = "QFULL";
  g_graffitiLastMatch = "NONE";
  g_graffitiLastScore = 0.0f;
  endQueuedGraffitiTraceStroke(completed, "QFULL", nullptr);
  emitQueuedGraffitiCapture(completed, "QFULL", nullptr);
}

void serviceCompletedGraffitiQueue() {
  QueuedGraffitiStroke stroke;
  if (!dequeueGraffitiStroke(stroke)) {
    return;
  }

  g_graffitiEngine.setCondition(activeGraffitiCondition());
  GraffitiEngine::Result result;
  const uint32_t inferStartUs = micros();
  if (!g_graffitiEngine.classify(stroke.points, stroke.count, result)) {
    g_graffitiLastInferUs = micros() - inferStartUs;
    g_graffitiMaxInferUs = max(g_graffitiMaxInferUs, g_graffitiLastInferUs);
    g_graffitiInferAccumUs += g_graffitiLastInferUs;
    g_graffitiInferCount++;
    g_graffitiStatus = "ERROR";
    g_graffitiLastMatch = "NONE";
    g_graffitiLastScore = 0.0f;
    endQueuedGraffitiTraceStroke(stroke, "ERROR", nullptr);
    emitQueuedGraffitiCapture(stroke, "ERROR", nullptr);
    return;
  }

  g_graffitiLastInferUs = micros() - inferStartUs;
  g_graffitiMaxInferUs = max(g_graffitiMaxInferUs, g_graffitiLastInferUs);
  g_graffitiInferAccumUs += g_graffitiLastInferUs;
  g_graffitiInferCount++;
  char resolvedSymbol = result.symbol;
  if (g_graffitiGlyphMode == GraffitiGlyphMode::Arrow && resolvedSymbol == 'o') {
    resolvedSymbol = 0x1B;
  } else {
    resolvedSymbol = applyGraffitiShift(resolvedSymbol);
  }
  g_graffitiLastScore = result.confidence;
  g_graffitiLastMatch = String(result.label) + ":" + graffitiSymbolLabel(resolvedSymbol);

  if (result.accepted) {
    g_graffitiStatus = "ACCEPT";
    bool keySent = false;
    if (g_graffitiOneShotCtrl && resolvedSymbol != 0x0F) {
      g_ctrlPreviewActive = true;
      g_ctrlPreviewSymbol = resolvedSymbol;
      g_graffitiOneShotCtrl = false;
      g_graffitiStatus = "CTRL ?";
      consumeGraffitiOneShotModes(resolvedSymbol);
    } else if (resolvedSymbol == 0x1B) {
      keySent = sendEscKey();
    } else {
      keySent = applyGraffitiSymbol(resolvedSymbol);
      if (resolvedSymbol == 0x0F) {
        // Keep one-shot ctrl armed until a real symbol is selected.
        g_graffitiStatus = "SHIFT";
      }
    }
    if (!(g_ctrlPreviewActive && g_ctrlPreviewSymbol == resolvedSymbol)) {
      consumeGraffitiOneShotModes(resolvedSymbol);
    }
    Serial.printf("[graffiti] ACCEPT %s score=%.3f thr=%.3f margin=%.3f dist=%.2f eng=%s tok=%u seq=%s infer_us=%lu q=%u kbd=%s mode=%s shift=%s\n",
                  g_graffitiLastMatch.c_str(), g_graffitiLastScore,
                  g_graffitiEngine.acceptConfidence(), result.rawScore,
                  result.rawDistance,
                  graffitiEngineLabel(result.backend), result.tokenCount,
                  g_graffitiEngine.lastTokenSequence(),
                  static_cast<unsigned long>(g_graffitiLastInferUs),
                  static_cast<unsigned>(g_completedGraffitiCount),
                  keySent ? "ok" : "skip",
                  graffitiGlyphModeName(g_graffitiGlyphMode),
                  graffitiShiftModeName(g_graffitiShiftMode));
  } else {
    g_graffitiStatus = "REJECT";
    g_rejectFlashUntilMs = millis() + kRejectFlashMs;
    Serial.printf("[graffiti] REJECT %s score=%.3f thr=%.3f margin=%.3f dist=%.2f eng=%s tok=%u seq=%s infer_us=%lu q=%u mode=%s shift=%s\n",
                  g_graffitiLastMatch.c_str(), g_graffitiLastScore,
                  g_graffitiEngine.acceptConfidence(), result.rawScore,
                  result.rawDistance,
                  graffitiEngineLabel(result.backend), result.tokenCount,
                  g_graffitiEngine.lastTokenSequence(),
                  static_cast<unsigned long>(g_graffitiLastInferUs),
                  static_cast<unsigned>(g_completedGraffitiCount),
                  graffitiGlyphModeName(g_graffitiGlyphMode),
                  graffitiShiftModeName(g_graffitiShiftMode));
  }

  endQueuedGraffitiTraceStroke(stroke, g_graffitiStatus.c_str(), &result);
  emitQueuedGraffitiCapture(stroke, g_graffitiStatus.c_str(), &result);
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
  int16_t finger = 0;
  int16_t x = -1;
  int16_t y = -1;
  const bool readOk = readTouchSnapshot(finger, x, y);

  if (!readOk || finger <= 0) {
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
    const bool swipeUp = (pressDuration <= kGraffitiExitSwipeMaxDurationMs &&
                          deltaY <= -kGraffitiCommandSwipeMinDyPx &&
                          absDx <= kGraffitiCommandSwipeMaxDxPx);
    const bool swipeDown = (pressDuration <= kGraffitiExitSwipeMaxDurationMs &&
                            deltaY >= kGraffitiCommandSwipeMinDyPx &&
                            absDx <= kGraffitiCommandSwipeMaxDxPx);
    const bool swipeRight = (pressDuration <= kGraffitiExitSwipeMaxDurationMs &&
                             deltaX >= kGraffitiCommandSwipeMinDyPx &&
                             absDy <= kGraffitiCommandSwipeMaxDxPx);
    const bool longPress = (pressDuration >= kGraffitiCommandLongPressMs &&
                            absDx <= kTouchTapMoveThresholdPx &&
                            absDy <= kTouchTapMoveThresholdPx);

    g_graffitiLastDeltaX = deltaX;
    g_graffitiLastDeltaY = deltaY;
    g_graffitiLastPressMs = static_cast<uint16_t>(std::min<uint32_t>(65535, pressDuration));
    g_touchDown = false;
    g_touchActive = false;
    resetGraffitiStrokeState();

    if (swipeUp) {
      if (g_graffitiGlyphMode == GraffitiGlyphMode::Numeric) {
        setGraffitiGlyphMode(GraffitiGlyphMode::Alpha);
        g_graffitiStatus = "MODE ALPHA";
      } else {
        setGraffitiGlyphMode(GraffitiGlyphMode::Numeric);
        g_graffitiStatus = "MODE NUM";
      }
      resetGraffitiTapSwitchState();
      return;
    }

    if (swipeDown) {
      if (g_graffitiGlyphMode == GraffitiGlyphMode::Arrow) {
        setGraffitiGlyphMode(GraffitiGlyphMode::Alpha);
        g_graffitiStatus = "ARROW OFF";
      } else {
        setGraffitiGlyphMode(GraffitiGlyphMode::Arrow);
        g_graffitiShiftMode = GraffitiShiftMode::Off;
        g_graffitiOneShotPunct = false;
        g_graffitiStatus = "ARROW ON";
      }
      resetGraffitiTapSwitchState();
      return;
    }

    if (swipeRight) {
      g_displayEnabled = !g_displayEnabled;
      updateDisplayBacklight();
      if (g_displayEnabled) {
        g_graffitiStatus = "DISP ON";
        renderStatus(true);
      } else {
        gfx->fillScreen(BLACK);
        g_graffitiStatus = "DISP OFF";
      }
      resetGraffitiTapSwitchState();
      return;
    }

    if (longPress) {
      g_commandPosePendingAction = GraffitiCommandPoseAction::ToggleMode;
      g_graffitiStatus = "MODE ARM";
      resetGraffitiTapSwitchState();
      return;
    }

    if (!tapLike) {
      g_graffitiStatus = "CMD TAP";
      resetGraffitiTapSwitchState();
      return;
    }

    g_commandPosePendingAction = GraffitiCommandPoseAction::ArmCtrl;
    g_graffitiStatus = "CTRL ARM";
    return;
  }
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
    g_graffitiStatus = "CMD TAP";
  }
}

void sampleGraffitiCtrlPreviewTouch() {
  const uint32_t now = millis();
  int16_t finger = 0;
  int16_t x = -1;
  int16_t y = -1;
  const bool readOk = readTouchSnapshot(finger, x, y);

  if (!readOk || finger <= 0) {
    if (!g_touchDown) {
      g_touchActive = false;
      return;
    }

    const uint32_t pressDuration = now - g_touchDownStartMs;
    const int16_t deltaX = g_touchX - g_touchDownX;
    const int16_t deltaY = g_touchY - g_touchDownY;
    const bool swipeRight = (pressDuration <= kGraffitiExitSwipeMaxDurationMs &&
                             deltaX >= kGraffitiCommandSwipeMinDyPx &&
                             abs(deltaY) <= kGraffitiCommandSwipeMaxDxPx);
    const bool swipeLeft = (pressDuration <= kGraffitiExitSwipeMaxDurationMs &&
                            deltaX <= -kGraffitiCommandSwipeMinDyPx &&
                            abs(deltaY) <= kGraffitiCommandSwipeMaxDxPx);

    g_touchDown = false;
    g_touchActive = false;
    g_graffitiLastDeltaX = deltaX;
    g_graffitiLastDeltaY = deltaY;
    g_graffitiLastPressMs = static_cast<uint16_t>(std::min<uint32_t>(65535, pressDuration));

    if (swipeRight) {
      const bool sent = sendCtrlModifiedSymbol(g_ctrlPreviewSymbol);
      g_graffitiStatus = sent ? "CTRL OK" : "CTRL SKIP";
      clearCtrlPreview();
      return;
    }
    if (swipeLeft) {
      g_graffitiStatus = "CTRL NO";
      clearCtrlPreview();
      return;
    }
    g_graffitiStatus = "CTRL ?";
    return;
  }

  g_touchActive = true;
  g_touchX = x;
  g_touchY = y;
  g_lastTouchEventMs = now;

  if (!g_touchDown) {
    g_touchDown = true;
    g_touchDownStartMs = now;
    g_touchDownX = x;
    g_touchDownY = y;
  }
}

void sampleGraffitiTouchState() {
  if (g_ctrlPreviewActive) {
    sampleGraffitiCtrlPreviewTouch();
    return;
  }
  if (g_graffitiTapOnlyActive) {
    sampleGraffitiExitTapOnly();
    return;
  }

  const uint32_t now = millis();
  int16_t finger = 0;
  int16_t x = -1;
  int16_t y = -1;
  const bool readOk = readTouchSnapshot(finger, x, y);
  if (!readOk) {
    if (g_touchDown &&
        g_graffitiLastCoordMs != 0 &&
        (now - g_graffitiLastCoordMs) > kGraffitiReleaseHoldMs) {
      g_touchDown = false;
      g_touchActive = false;
      g_graffitiStatus = "T-ERR";
      endGraffitiTraceStroke("T-ERR", nullptr);
      resetGraffitiTapSwitchState();
      resetGraffitiStrokeState();
    }
    return;
  }
  const bool fingerPresent = finger > 0;
  const bool coordValid = (x >= 0 && y >= 0);
  const bool coordIdle = coordValid && (x == kCstIdleX) && (y == kCstIdleY);
  const bool coordTouch = coordValid && !coordIdle;
  const bool touchPresentRaw = fingerPresent && coordTouch;

  if (g_graffitiGlyphMode == GraffitiGlyphMode::Arrow) {
    if (touchPresentRaw) {
      g_touchActive = true;
      g_touchX = x;
      g_touchY = y;
      g_lastTouchEventMs = now;
      const uint8_t usage = arrowUsageForPoint(x, y);
      if (!g_touchDown) {
        g_touchDown = true;
        g_touchDownStartMs = now;
        g_touchDownX = x;
        g_touchDownY = y;
        if (usage != 0) {
          tapKeyboardKeyUsage(usage);
          g_arrowHeldUsage = usage;
          g_arrowRepeatDueMs = now + kArrowRepeatInitialMs;
          g_graffitiStatus = (usage == 0x28) ? "AR ENTER" : "AR TAP";
          return;
        }
        resetGraffitiStrokeState();
        g_graffitiTouchStartMs = now;
        g_graffitiStatus = "AR DRAW";
        g_graffitiLastCoordMs = now;
        beginGraffitiTraceStroke();
      }

      if (g_arrowHeldUsage != 0) {
        return;
      }
      g_graffitiLastCoordMs = now;
      graffitiAddPoint(x, y, now);
      return;
    }

    if (g_arrowHeldUsage != 0) {
      g_touchDown = false;
      g_touchActive = false;
      g_arrowHeldUsage = 0;
      g_graffitiStatus = "AR READY";
      return;
    }
  }

  if (fingerPresent && coordTouch) {
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
      beginGraffitiTraceStroke();
    }

    if (coordTouch) {
      graffitiAddPoint(g_touchX, g_touchY, now);
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

  if (!fingerPresent) {
    if (g_graffitiNoFingerSamples < 255) {
      ++g_graffitiNoFingerSamples;
    }
  } else {
    g_graffitiNoFingerSamples = 0;
  }

  if (g_graffitiNoFingerSamples < kGraffitiReleaseSamples &&
      g_graffitiLastCoordMs != 0 &&
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

  if (tapLike) {
    resetGraffitiTapSwitchState();
    if (g_graffitiOneShotPunct) {
      g_graffitiStatus = "PUNCT .";
      applyGraffitiSymbol('.');
      consumeGraffitiOneShotModes('.');
    } else if (g_graffitiGlyphMode != GraffitiGlyphMode::Arrow) {
      g_graffitiOneShotPunct = true;
      g_graffitiStatus = "PUNCT 1X";
    } else {
      g_graffitiStatus = "TAP";
    }
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

bool recalibrateGyroBias(bool persistBias) {
  const bool calibrated = calibrateGyroBias();
  if (!calibrated) {
    return false;
  }

  resetStationaryBiasEstimator();
  g_airVelX = 0.0f;
  g_airVelY = 0.0f;
  g_airResidualX = 0.0f;
  g_airResidualY = 0.0f;
  g_lastDeltaX = 0;
  g_lastDeltaY = 0;

  if (persistBias) {
    saveGyroBiasToNvs();
  }
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
  ledcWrite(LCD_BL, kDisplayBacklightOffPwm);

  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(1);
  gfx->setTextWrap(false);
  updateDisplayBacklight();
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

  const bool loaded = loadGyroBiasFromNvs();
  if (!loaded) {
    Serial.println("[imu] No stored bias, calibrating...");
    const bool calibrated = recalibrateGyroBias(true);
    if (!calibrated) {
      g_gyroBiasX = 0.0f;
      g_gyroBiasY = 0.0f;
      g_gyroBiasZ = 0.0f;
      Serial.println("[imu] Using zero bias fallback");
    }
  }

  resetStationaryBiasEstimator();
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
  const bool rejectFlashActive = static_cast<int32_t>(now - g_rejectFlashUntilMs) < 0;
  const bool overlayActive =
      (g_graffitiGlyphMode == GraffitiGlyphMode::Arrow) ||
      (g_graffitiShiftMode != GraffitiShiftMode::Off) || g_graffitiOneShotPunct ||
      g_graffitiOneShotCtrl || g_ctrlPreviewActive || rejectFlashActive;
  if (!g_displayEnabled && !overlayActive) {
    return;
  }
  if (!force && (now - g_lastDisplayRefreshMs) < kDisplayRefreshMs) {
    return;
  }
  g_lastDisplayRefreshMs = now;

  if (rejectFlashActive) {
    gfx->fillScreen(WHITE);
    g_statusFrameDrawn = false;
    return;
  }

  if (!g_displayEnabled) {
    gfx->fillScreen(BLACK);
    if (g_graffitiGlyphMode == GraffitiGlyphMode::Arrow) {
      const bool showIntro = static_cast<int32_t>(now - (g_arrowModeEnteredMs + kArrowIntroMs)) < 0;
      uint8_t arrowUsage = g_arrowHeldUsage;
      if (arrowUsage == 0 && g_touchActive) {
        arrowUsage = arrowUsageForPoint(g_touchX, g_touchY);
      }
      if (arrowUsage == 0x50) {
        gfx->fillRect(0, 0, kArrowEdgeBarThicknessPx, LCD_HEIGHT, CYAN);
      } else if (arrowUsage == 0x4F) {
        gfx->fillRect(LCD_WIDTH - kArrowEdgeBarThicknessPx, 0, kArrowEdgeBarThicknessPx, LCD_HEIGHT, CYAN);
      } else if (arrowUsage == 0x52) {
        gfx->fillRect(0, 0, LCD_WIDTH, kArrowEdgeBarThicknessPx, CYAN);
      } else if (arrowUsage == 0x51) {
        gfx->fillRect(0, LCD_HEIGHT - kArrowEdgeBarThicknessPx, LCD_WIDTH, kArrowEdgeBarThicknessPx, CYAN);
      }
      if (showIntro) {
        gfx->drawCircle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 10, WHITE);
      }
    }
    if (g_graffitiShiftMode != GraffitiShiftMode::Off) {
      gfx->fillRect(0, 0, LCD_WIDTH, LCD_HEIGHT / 8, WHITE);
    }
    if (g_graffitiOneShotPunct) {
      gfx->fillCircle(LCD_WIDTH - 8 - 7, 8 + 7, 7, YELLOW);
    }
    if (g_graffitiOneShotCtrl || g_ctrlPreviewActive) {
      gfx->fillCircle(8 + 7, 8 + 7, 7, BLUE);
    }
    if (g_ctrlPreviewActive) {
      const String preview = graffitiSymbolLabel(g_ctrlPreviewSymbol);
      const uint8_t textSize = (preview.length() <= 1) ? 3 : 2;
      gfx->setTextColor(WHITE, BLACK);
      gfx->setTextSize(textSize);
      const int16_t charW = 6 * textSize;
      const int16_t textW = preview.length() * charW;
      const int16_t textX = std::max<int16_t>(0, (LCD_WIDTH - textW) / 2);
      const int16_t textY = std::max<int16_t>(16, (LCD_HEIGHT / 2) - (4 * textSize));
      gfx->setCursor(textX, textY);
      gfx->print(preview);
      gfx->setTextSize(1);
    }
    g_statusFrameDrawn = false;
    return;
  }

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
    snprintf(buf, sizeof(buf), "G:%s I:%c", textTail(g_lastGesture, 10).c_str(),
             g_graffitiTapOnlyActive ? 'Y' : 'N');
    drawStatusLine(6, 94, String(buf), WHITE, force);
    drawStatusLine(7, 104, String("Click:") + g_clickMode, GREEN, force);
    drawStatusLine(8, 114, String("Cmd:") + textTail(g_lastCmdAck, 16), YELLOW, force);
  } else {
    snprintf(buf, sizeof(buf), "dX:%4d dY:%4d t:%3u", g_graffitiLastDeltaX, g_graffitiLastDeltaY,
             g_graffitiLastPressMs);
    drawStatusLine(5, 80, String(buf), WHITE, force);
    snprintf(buf, sizeof(buf), "Gra:%s n%u q%u", g_graffitiStatus.c_str(), g_graffitiStrokeCount,
             static_cast<unsigned>(g_completedGraffitiCount));
    drawStatusLine(6, 94, String(buf), WHITE, force);
    snprintf(buf, sizeof(buf), "%s %s P:%c", graffitiGlyphModeName(g_graffitiGlyphMode),
             graffitiShiftModeName(g_graffitiShiftMode), g_graffitiOneShotPunct ? 'Y' : 'N');
    drawStatusLine(7, 104, String(buf), GREEN, force);
    drawStatusLine(8, 114, String("Cmd:") + textTail(g_lastCmdAck, 16), YELLOW, force);
  }

  if (g_graffitiOneShotCtrl || g_ctrlPreviewActive) {
    gfx->fillCircle(8 + 7, 8 + 7, 7, BLUE);
  }
  if (g_graffitiOneShotPunct) {
    gfx->fillCircle(LCD_WIDTH - 8 - 7, 8 + 7, 7, YELLOW);
  }
  if (g_ctrlPreviewActive) {
    const String preview = graffitiSymbolLabel(g_ctrlPreviewSymbol);
    const uint8_t textSize = (preview.length() <= 1) ? 3 : 2;
    gfx->setTextColor(WHITE, BLACK);
    gfx->setTextSize(textSize);
    const int16_t charW = 6 * textSize;
    const int16_t textW = preview.length() * charW;
    const int16_t textX = std::max<int16_t>(0, (LCD_WIDTH - textW) / 2);
    const int16_t textY = std::max<int16_t>(18, (LCD_HEIGHT / 2) - (4 * textSize));
    gfx->setCursor(textX, textY);
    gfx->print(preview);
    gfx->setTextSize(1);
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

  if (g_inputMode == InputMode::Mouse && !g_graffitiTapOnlyActive) {
    if (gesture == "Swipe Up" || gesture == "Swipe Down") {
      g_tapArmed = false;
      g_swipeHandledThisTouch = true;
      const float wheelUnits = (gesture == "Swipe Up") ? 1.0f : -1.0f;
      emitScaledWheel(wheelUnits);
    }
    return;
  }

  // Graffiti mode uses raw coordinate state machine for stroke and mode exits.
  return;
}

void sampleTouchState() {
  const uint32_t now = millis();
  int16_t finger = 0;
  int16_t x = -1;
  int16_t y = -1;
  const bool readOk = readTouchSnapshot(finger, x, y);

  if (!readOk || finger <= 0) {
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
        const float wheelSign = (deltaY < 0) ? 1.0f : -1.0f;
        const float rawSteps = static_cast<float>(std::min<int16_t>(3, std::max<int16_t>(1, absDy / 28)));
        emitScaledWheel(wheelSign * rawSteps);
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
  serviceAutoGyroBiasCorrection(sample, gx, gy, gz);

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
  if (!loadUiConfigFromNvs()) {
    g_scrollGain = kDefaultScrollGain;
    Serial.printf("[cfg] using default scroll_gain=%.2f\n", g_scrollGain);
  }
  initBleMouse();
  Serial.printf("[graffiti] recognizer mode=PROTOTYPE cond=%s\n",
                g_graffitiEngine.conditionName());

  renderStatus(true);
}

void loop() {
  const uint32_t now = millis();
  bool touchEdge = false;
  pollSerialCommands();
  serviceKeyboardRelease();
  serviceArrowRepeat();
  serviceSetupShift();

  if (g_touchInterrupt) {
    g_touchInterrupt = false;
    g_lastTouchEventMs = now;
    touchEdge = true;
    if (g_inputMode == InputMode::Mouse && !g_graffitiTapOnlyActive) {
      handleGestureIfAny();
    }
  }

  if (g_imuReady) {
    const bool airMouseActive = (g_inputMode == InputMode::Mouse) && !g_graffitiTapOnlyActive;
    const bool commandPoseTouchActive = g_graffitiTapOnlyActive && (g_touchDown || g_touchActive);
    const bool graffitiStrokeActive = (g_inputMode == InputMode::Graffiti) && (g_touchDown || g_touchActive);
    const uint32_t imuPollMs = airMouseActive ? kImuPollMs : kGraffitiImuPollMs;
    if (!graffitiStrokeActive && !commandPoseTouchActive && (now - g_lastImuPollMs) >= imuPollMs) {
      g_lastImuPollMs = now;
      if (airMouseActive) {
        updateAirMouse();
      } else {
        updateImuOrientationOnly();
      }
    }
  }

  updateGraffitiTapOnlyMode();

  const bool touchNeedsPolling = touchEdge || g_touchDown || g_touchActive || g_graffitiTapOnlyActive;
  const uint32_t touchPollMs = (g_inputMode == InputMode::Graffiti || g_graffitiTapOnlyActive)
                                   ? kGraffitiTouchPollMs
                                   : kTouchPollMs;
  if (touchNeedsPolling && (now - g_lastTouchPollMs) >= touchPollMs) {
    g_lastTouchPollMs = now;
    if (g_graffitiTapOnlyActive) {
      sampleGraffitiExitTapOnly();
    } else if (g_inputMode == InputMode::Mouse) {
      sampleTouchState();
    } else {
      sampleGraffitiTouchState();
    }
  }

  if (g_touchActive && (now - g_lastTouchEventMs) > kTouchReleaseTimeoutMs) {
    g_touchActive = false;
  }

  if (g_inputMode == InputMode::Mouse && !g_graffitiTapOnlyActive) {
    if (g_tapArmed && static_cast<int32_t>(g_tapArmDeadlineMs - now) < 0) {
      g_tapArmed = false;
      if (!g_leftLockActive) {
        g_clickMode = "FREE";
      }
    }
  }

  if (g_graffitiTapArmed && (now - g_graffitiTapArmedMs) > kModeExitDoubleTapWindowMs) {
    resetGraffitiTapSwitchState();
    if (!g_touchDown) {
      if (g_graffitiTapOnlyActive) {
        if (!g_graffitiOneShotPunct) {
          g_graffitiStatus = "CMD TAP";
        }
      } else if (g_inputMode == InputMode::Graffiti) {
        g_graffitiStatus = "READY";
      } else {
        g_graffitiStatus = "IDLE";
      }
    }
  }

  serviceCompletedGraffitiQueue();
  updateDisplayBacklight();
  renderStatus();
  delay(2);
}
