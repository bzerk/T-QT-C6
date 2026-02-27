#include <Arduino.h>
#include <algorithm>
#include <math.h>

#include "Arduino_DriveBus_Library.h"
#include "Arduino_GFX_Library.h"
#include "pin_config.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEHIDDevice.h>
#include <BLEServer.h>
#include <BLESecurity.h>

namespace {
constexpr char kDeviceName[] = "Ringo";
constexpr uint8_t kMouseReportId = 0x01;

constexpr uint32_t kDisplayRefreshMs = 140;
constexpr uint32_t kTouchReleaseTimeoutMs = 120;
constexpr uint32_t kTouchPollMs = 12;
constexpr uint32_t kImuPollMs = 8;
constexpr uint32_t kImuDebugIntervalMs = 500;

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

volatile bool g_touchInterrupt = false;
bool g_bleConnected = false;
bool g_touchActive = false;
bool g_imuReady = false;

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

BLEHIDDevice *g_hid = nullptr;
BLECharacteristic *g_inputMouse = nullptr;
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
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    g_bleConnected = false;
    g_buttonMask = 0;
    g_leftLockActive = false;
    g_clickMode = "FREE";
    g_tapArmed = false;
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
  if (ETA4662->begin()) {
    Serial.println("[power] ETA4662 init ok");
  } else if (SGM41562->begin()) {
    Serial.println("[power] SGM41562 init ok");
  } else {
    Serial.println("[power] Power chip init failed");
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
  while (!CST816T->begin()) {
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
  while (!LSM6DSL->begin()) {
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
  g_advertising->setAppearance(HID_MOUSE);
  g_advertising->addServiceUUID(g_hid->hidService()->getUUID());
  g_advertising->start();

  Serial.println("[ble] HID mouse started, advertising as 'Ringo'");
}

void renderStatus(bool force = false) {
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
  drawStatusLine(1, 24, String("BLE: ") + (g_bleConnected ? "CONNECTED" : "ADVERTISING"), WHITE, force);
  snprintf(buf, sizeof(buf), "Roll:%5.1f D:%1.2f", g_rollRateDps, g_rollDamp);
  drawStatusLine(2, 38, String(buf), WHITE, force);

  snprintf(buf, sizeof(buf), "Touch:%s X:%3dY:%3d", g_touchActive ? "ON " : "OFF", g_touchX, g_touchY);
  drawStatusLine(3, 52, String(buf), WHITE, force);

  snprintf(buf, sizeof(buf), "Yaw:%5.1f Pit:%5.1f", g_yawRateDps, g_pitchRateDps);
  drawStatusLine(4, 66, String(buf), WHITE, force);

  snprintf(buf, sizeof(buf), "Move:%4d,%4d", g_lastDeltaX, g_lastDeltaY);
  drawStatusLine(5, 80, String(buf), WHITE, force);

  drawStatusLine(6, 94, String("G:") + g_lastGesture, WHITE, force);
  drawStatusLine(7, 104, String("Click:") + g_clickMode, GREEN, force);
  drawStatusLine(8, 114, "cu.usbmodem1101", YELLOW, force);
}

void handleGestureIfAny() {
  String gesture = CST816T->IIC_Read_Device_State(
      CST816T->Arduino_IIC_Touch::Status_Information::TOUCH_GESTURE_ID);

  if (gesture == "NONE" || gesture.startsWith("->")) {
    return;
  }

  g_lastGesture = gesture;
  Serial.printf("[touch] gesture=%s\n", gesture.c_str());

  if (gesture == "Swipe Up") {
    g_tapArmed = false;
    g_swipeHandledThisTouch = true;
    sendMouseReport(g_buttonMask, 0, 0, 1);
  } else if (gesture == "Swipe Down") {
    g_tapArmed = false;
    g_swipeHandledThisTouch = true;
    sendMouseReport(g_buttonMask, 0, 0, -1);
  }
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
  while (!Serial && (millis() - serialWaitStart) < 1500) {
    delay(10);
  }

  Serial.println();
  Serial.println("[boot] Ringo BLE trackpad starting");

  initBoardPowerAndDisplay();
  initTouch();
  initImu();
  initBleMouse();

  renderStatus(true);
}

void loop() {
  const uint32_t now = millis();
  bool touchEdge = false;

  if (g_touchInterrupt) {
    g_touchInterrupt = false;
    g_lastTouchEventMs = now;
    touchEdge = true;
    handleGestureIfAny();
  }

  const bool touchNeedsPolling = touchEdge || g_touchDown || g_touchActive;
  if (touchNeedsPolling && (now - g_lastTouchPollMs) >= kTouchPollMs) {
    g_lastTouchPollMs = now;
    sampleTouchState();
  }

  if (g_touchActive && (now - g_lastTouchEventMs) > kTouchReleaseTimeoutMs) {
    g_touchActive = false;
  }

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

  renderStatus();
  delay(2);
}
