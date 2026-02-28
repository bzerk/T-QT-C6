#include <Arduino.h>

#include "Arduino_DriveBus_Library.h"
#include "Arduino_GFX_Library.h"
#include "pin_config.h"

#include "GraffitiRecognizer.h"

namespace {
constexpr char kDeviceName[] = "Ringo Graffiti";

constexpr uint32_t kDisplayRefreshMs = 120;
constexpr uint32_t kTouchPollMs = 12;
constexpr uint32_t kTouchReleaseTimeoutMs = 150;
constexpr uint32_t kStrokeMaxDurationMs = 2200;
constexpr uint16_t kStrokeMaxPoints = 180;
constexpr uint16_t kStrokeMinPoints = 6;
constexpr int32_t kMinPointDistanceSq = 9;

volatile bool g_touchInterrupt = false;

uint32_t g_lastTouchPollMs = 0;
uint32_t g_lastTouchEventMs = 0;
uint32_t g_lastDisplayRefreshMs = 0;
uint32_t g_touchStartMs = 0;

bool g_touchActive = false;
bool g_touchDown = false;
int16_t g_touchX = -1;
int16_t g_touchY = -1;

GraffitiRecognizer g_recognizer;
GraffitiRecognizer::Point g_stroke[kStrokeMaxPoints];
uint16_t g_strokeCount = 0;

String g_textBuffer = "";
String g_status = "IDLE";
String g_lastMatch = "NONE";
float g_lastScore = 0.0f;

bool g_statusFrameDrawn = false;
String g_prevStatusText[8];
uint16_t g_prevStatusColor[8] = {0, 0, 0, 0, 0, 0, 0, 0};

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

std::unique_ptr<Arduino_IIC> ETA4662(new Arduino_ETA4662(IIC_Bus, ETA4662_DEVICE_ADDRESS,
                                                         DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));

std::unique_ptr<Arduino_IIC> SGM41562(new Arduino_SGM41562(IIC_Bus, SGM41562_DEVICE_ADDRESS,
                                                           DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));

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

String textTail(const String &text, uint8_t maxLen) {
  if (text.length() <= maxLen) {
    return text;
  }
  return text.substring(text.length() - maxLen);
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
    for (uint8_t i = 0; i < 8; ++i) {
      g_prevStatusText[i] = "";
      g_prevStatusColor[i] = 0xFFFF;
    }
  }

  char buf[48];
  drawStatusLine(0, 10, String(kDeviceName), CYAN, force);
  snprintf(buf, sizeof(buf), "Touch:%s X:%3dY:%3d", g_touchDown ? "DOWN" : "UP  ", g_touchX, g_touchY);
  drawStatusLine(1, 24, String(buf), WHITE, force);

  snprintf(buf, sizeof(buf), "Stroke:%3u %s", g_strokeCount, g_status.c_str());
  drawStatusLine(2, 38, String(buf), WHITE, force);

  snprintf(buf, sizeof(buf), "Match:%s %.2f", g_lastMatch.c_str(), g_lastScore);
  drawStatusLine(3, 52, String(buf), WHITE, force);

  drawStatusLine(4, 66, String("Text:") + textTail(g_textBuffer, 18), GREEN, force);
  drawStatusLine(5, 80, "Glyphs: a i l n o u v z", WHITE, force);
  drawStatusLine(6, 94, "->SPACE <-BKSP", WHITE, force);
  drawStatusLine(7, 108, "cu.usbmodem1101", YELLOW, force);
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

void addStrokePoint(int16_t x, int16_t y) {
  if (g_strokeCount > 0) {
    const float dx = static_cast<float>(x) - g_stroke[g_strokeCount - 1].x;
    const float dy = static_cast<float>(y) - g_stroke[g_strokeCount - 1].y;
    const float distanceSq = (dx * dx) + (dy * dy);
    if (distanceSq < static_cast<float>(kMinPointDistanceSq)) {
      return;
    }
  }

  if (g_strokeCount >= kStrokeMaxPoints) {
    return;
  }

  g_stroke[g_strokeCount].x = static_cast<float>(x);
  g_stroke[g_strokeCount].y = static_cast<float>(y);
  g_strokeCount++;
}

String symbolLabel(char symbol) {
  if (symbol == ' ') {
    return "SPACE";
  }
  if (symbol == '\b') {
    return "BKSP";
  }
  char c[2] = {symbol, '\0'};
  return String(c);
}

void applyRecognizedSymbol(char symbol) {
  if (symbol == '\b') {
    if (!g_textBuffer.isEmpty()) {
      g_textBuffer.remove(g_textBuffer.length() - 1);
    }
    return;
  }

  g_textBuffer += symbol;
  if (g_textBuffer.length() > 64) {
    g_textBuffer = g_textBuffer.substring(g_textBuffer.length() - 64);
  }
}

void finalizeStroke() {
  if (g_strokeCount < kStrokeMinPoints) {
    g_lastMatch = "SHORT";
    g_lastScore = 0.0f;
    g_status = "IDLE";
    g_strokeCount = 0;
    return;
  }

  GraffitiRecognizer::Result result;
  if (!g_recognizer.recognize(g_stroke, g_strokeCount, result)) {
    g_lastMatch = "ERROR";
    g_lastScore = 0.0f;
    g_status = "IDLE";
    g_strokeCount = 0;
    return;
  }

  g_lastScore = result.score;
  g_lastMatch = String(result.name) + " " + symbolLabel(result.symbol);

  if (result.score >= GraffitiRecognizer::kAcceptScore) {
    applyRecognizedSymbol(result.symbol);
    g_status = "ACCEPT";
    Serial.printf("[graffiti] ACCEPT %s score=%.2f\n", g_lastMatch.c_str(), g_lastScore);
  } else {
    g_status = "REJECT";
    Serial.printf("[graffiti] REJECT %s score=%.2f\n", g_lastMatch.c_str(), g_lastScore);
  }

  g_strokeCount = 0;
}

void sampleTouchState() {
  const uint32_t now = millis();

  const int16_t finger = (int16_t)CST816T->IIC_Read_Device_Value(
      CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

  if (finger <= 0) {
    if (g_touchDown) {
      g_touchDown = false;
      g_touchActive = false;
      finalizeStroke();
    }
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
    g_touchStartMs = now;
    g_strokeCount = 0;
    g_status = "DRAW";
  }

  addStrokePoint(x, y);

  if ((now - g_touchStartMs) > kStrokeMaxDurationMs) {
    g_touchDown = false;
    g_touchActive = false;
    g_status = "TIMEOUT";
    finalizeStroke();
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
  Serial.println("[boot] Ringo Graffiti recognizer");

  initBoardPowerAndDisplay();
  initTouch();
  renderStatus(true);
}

void loop() {
  const uint32_t now = millis();
  bool touchEdge = false;

  if (g_touchInterrupt) {
    g_touchInterrupt = false;
    touchEdge = true;
  }

  const bool touchNeedsPolling = touchEdge || g_touchDown || g_touchActive;
  if (touchNeedsPolling && (now - g_lastTouchPollMs) >= kTouchPollMs) {
    g_lastTouchPollMs = now;
    sampleTouchState();
  }

  if (g_touchActive && !g_touchDown && (now - g_lastTouchEventMs) > kTouchReleaseTimeoutMs) {
    g_touchActive = false;
  }

  renderStatus();
  delay(2);
}
