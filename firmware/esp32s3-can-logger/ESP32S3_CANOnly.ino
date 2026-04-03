/*
  ESP32-S3 CAN-only Web UI

  - CAN via MCP2515 (mcp_can)
  - No SD card
  - No GPS
  - Web UI served from SPIFFS
  - WebSocket broadcast for live frames

  Target: ESP32-S3 DevKit
*/

// Arduino IDE note:
// If you copy/paste this file into a single .ino, the Arduino build system
// auto-generates function prototypes after the last #include line. Those
// prototypes can reference our structs (LogicalInput / EspNowCanPacket, etc)
// before they are defined. These forward declarations keep that build path
// working.
struct LogicalInput;
struct InputDef;
struct LogicalOutput;
struct EspNowCanPacket;

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>

// Types used throughout the sketch.
// These are kept in this file (not a separate header) so the code can be
// copy/pasted into an Arduino sketch without missing local includes.
struct LogicalInput {
  int gpio;        // ESP32 GPIO number, or -1 for unused
  bool activeLow;  // true if pressed/active pulls low (use pull-up)
};

struct InputDef {
  const char *label;
  int gpio;
  bool activeLow;
};

struct LogicalOutput {
  int gpio;           // ESP32 GPIO number, or -1 for unused
  bool activeHigh;    // true if output ON means drive HIGH
  bool pwmCapable;    // reserved for future PWM support
  uint16_t offset;    // table offset where the source byte lives
  uint8_t bit;        // which bit in that byte
};

#include <SD.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>

#include <esp_system.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>

// If USB CDC is supported/enabled, allow runtime init so you can get logs
// over the same USB port you use for flashing.
#if defined(CONFIG_TINYUSB_CDC_ENABLED) && CONFIG_TINYUSB_CDC_ENABLED
#include "USB.h"
#define HAS_USB_CDC 1
#else
#define HAS_USB_CDC 0
#endif

// ------------------------------
// Serial debugging
// ------------------------------
// 0 = off, 1 = basic, 2 = verbose
#ifndef DEBUG_SERIAL_LEVEL
#define DEBUG_SERIAL_LEVEL 2
#endif

// Per-frame logging is VERY noisy and will reduce performance.
#ifndef DEBUG_SERIAL_FRAMES
#define DEBUG_SERIAL_FRAMES 0
#endif

static unsigned long dbgLastStatusMs = 0;
static unsigned long dbgLastWifiStateMs = 0;
static wl_status_t dbgLastWifiStatus = WL_IDLE_STATUS;
static bool dbgLastCanOk = false;

// Mirror debug prints to both UART Serial and USB CDC (when available).
class DualPrint : public Print {
public:
  Print *a = nullptr;
  Print *b = nullptr;

  size_t write(uint8_t c) override {
    size_t wrote = 0;
    if (a) wrote |= (a->write(c) == 1);
    if (b) wrote |= (b->write(c) == 1);
    return wrote ? 1 : 0;
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    size_t wa = 0, wb = 0;
    if (a) wa = a->write(buffer, size);
    if (b) wb = b->write(buffer, size);
    return (wa > wb) ? wa : wb;
  }
};

static DualPrint DBG;

// ------------------------------
// Robustness / harsh automotive hardening
// ------------------------------
// Watchdog: resets MCU if loop hangs (EMI, stack, WiFi driver lockups, etc)
static const int WDT_TIMEOUT_S = 5;
static bool wdtEnabled = false;

static inline void feedWdt() {
  if (wdtEnabled) esp_task_wdt_reset();
}

// Boot-loop safe mode: if we reboot repeatedly before the system is stable,
// start in a reduced-feature mode to make recovery easier.
// RTC memory survives warm resets without flash wear.
RTC_DATA_ATTR static uint32_t bootFailCount = 0;
static bool SAFE_MODE = false;
static const uint32_t SAFE_MODE_BOOT_THRESHOLD = 4; // N quick-fail boots => SAFE_MODE

static const unsigned long WIFI_RECONNECT_INTERVAL_MS = 8000;
static unsigned long wifiLastReconnectMs = 0;

static const unsigned long CAN_REINIT_INTERVAL_MS = 2000;
static unsigned long canLastReinitAttemptMs = 0;
static byte canSendFailStreak = 0;
static const byte CAN_SEND_FAIL_STREAK_MAX = 10;

static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN: return "UNKNOWN";
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "?";
  }
}

static void initWdt() {
  // IDF v5+ supports esp_task_wdt_config_t; older IDF uses a simpler init call.
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t cfg = {
    .timeout_ms = (uint32_t)(WDT_TIMEOUT_S * 1000),
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_init(&cfg) == ESP_OK) {
    esp_task_wdt_add(NULL);
    wdtEnabled = true;
  }
#else
  if (esp_task_wdt_init((uint32_t)WDT_TIMEOUT_S, true) == ESP_OK) {
    esp_task_wdt_add(NULL);
    wdtEnabled = true;
  }
#endif
}

static void dbgPrintHexByte(uint8_t b) {
  if (b < 16) DBG.print('0');
  DBG.print(b, HEX);
}

static void dbgPrintCanFrame(unsigned long id, byte len, const byte *buf) {
  DBG.print("CAN rx id=0x");
  DBG.print(id, HEX);
  DBG.print(" len=");
  DBG.print((int)len);
  DBG.print(" data=");
  for (int i = 0; i < (int)len; ++i) {
    dbgPrintHexByte(buf[i]);
    if (i + 1 < (int)len) DBG.print(' ');
  }
  DBG.println();
}

// Optional I2C display (recommended: SSD1306 128x32/64).
// If you don't have these libraries installed, set to 0.
#define ENABLE_I2C_DISPLAY 1

#if ENABLE_I2C_DISPLAY
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

// ------------------------------
// Pins (ESP32-S3)
// ------------------------------
static const int I2C_SDA = 3;
static const int I2C_SCL = 10;

static const int CAN_INT = 37;
static const int CAN_SCK = 47;
static const int CAN_MOSI = 21; // SI
static const int CAN_MISO = 36; // SO
static const int CAN_CS  = 38;

// SD logging (shares the same SPI bus as the MCP2515 by default).
// NOTE: Most microSD modules are SPI, not I2C. If your module is truly I2C-based,
// tell me the exact module/chip so we can use the correct library.
#ifndef ENABLE_SD_LOGGING
#define ENABLE_SD_LOGGING 1
#endif

// Pick a free GPIO for your SD module CS.
// This must NOT conflict with CAN_CS or your inputs.
#ifndef SD_CS_PIN
#define SD_CS_PIN 14
#endif

static const int SD_CS = SD_CS_PIN;

// SPI bring-up: start conservative for noisy wiring, then raise after init.
static const uint32_t SPI_BAUD_INIT = 1000000;  // 1 MHz
static const uint32_t SPI_BAUD_RUN  = 8000000;  // 8 MHz

// Optional onboard RGB LED (WS2812-style). Many ESP32-S3 devkits use GPIO48.
// If your board differs, set this to your LED GPIO.
#ifndef STATUS_LED_PIN
#if defined(RGB_BUILTIN)
#define STATUS_LED_PIN RGB_BUILTIN
#else
#define STATUS_LED_PIN 48
#endif
#endif

// ------------------------------
// CAN
// ------------------------------
MCP_CAN CAN_MICRO(CAN_CS);
static bool CANOK = false;
static unsigned long lastCanMsg = 0;
static const unsigned long canTimeoutMs = 1500;
static unsigned long canCount = 0;

// MCP2515 specifics (your board: 8MHz oscillator)
static const uint8_t MCP_CLOCK = MCP_8MHZ;
static const uint8_t CAN_BITRATE = CAN_500KBPS;

// ------------------------------
// SD logging (non-blocking writer task)
// ------------------------------
static File logFile;
static bool SDOK = false;

static QueueHandle_t sdQueue = NULL;
static const int SD_QUEUE_SIZE = 64;
static String sdLogPath;

static bool timeValid = false;

static unsigned long getTimeSeconds() {
  if (timeValid) {
    time_t now = time(nullptr);
    if ((uint32_t)now > 1700000000UL) return (unsigned long)now;
  }
  return millis() / 1000;
}

static int monthFromName(const char *mon) {
  if (!mon) return 1;
  if (!strncmp(mon, "Jan", 3)) return 1;
  if (!strncmp(mon, "Feb", 3)) return 2;
  if (!strncmp(mon, "Mar", 3)) return 3;
  if (!strncmp(mon, "Apr", 3)) return 4;
  if (!strncmp(mon, "May", 3)) return 5;
  if (!strncmp(mon, "Jun", 3)) return 6;
  if (!strncmp(mon, "Jul", 3)) return 7;
  if (!strncmp(mon, "Aug", 3)) return 8;
  if (!strncmp(mon, "Sep", 3)) return 9;
  if (!strncmp(mon, "Oct", 3)) return 10;
  if (!strncmp(mon, "Nov", 3)) return 11;
  if (!strncmp(mon, "Dec", 3)) return 12;
  return 1;
}

static String nextTripPath() {
  // Prefer SNTP time when available; fall back to build date to avoid 1970.
  time_t secs = 0;
  time_t now = time(nullptr);
  if ((uint32_t)now > 1700000000UL) {
    secs = now;
    timeValid = true;
  } else {
    const char *bd = __DATE__; // "Mmm dd yyyy"
    char mon[4] = {0};
    int day = 1;
    int year = 1970;
    if (sscanf(bd, "%3s %d %d", mon, &day, &year) >= 3) {
      struct tm bt;
      memset(&bt, 0, sizeof(bt));
      bt.tm_year = year - 1900;
      bt.tm_mon = monthFromName(mon) - 1;
      bt.tm_mday = day;
      bt.tm_hour = 0;
      bt.tm_min = 0;
      bt.tm_sec = 0;
      secs = mktime(&bt);
    } else {
      secs = (time_t)(millis() / 1000);
    }
  }

  struct tm t;
  gmtime_r(&secs, &t);
  char dateDir[32];
  snprintf(dateDir, sizeof(dateDir), "/logs/%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);

  if (!SD.exists("/logs")) SD.mkdir("/logs");
  if (!SD.exists(dateDir)) SD.mkdir(dateDir);

  for (int n = 1; n < 1000; ++n) {
    char fname[64];
    snprintf(fname, sizeof(fname), "%s/trip%03d.csv", dateDir, n);
    if (!SD.exists(String(fname))) return String(fname);
  }
  return String(dateDir) + String("/trip999.csv");
}

static void startNewTrip() {
  sdLogPath = nextTripPath();
  DBG.print("Opening log file: ");
  DBG.println(sdLogPath);
  logFile = SD.open(sdLogPath, FILE_WRITE);
  if (logFile) {
    logFile.println("type,ts,ms,a,b,c,d,e,f");
    logFile.flush();
  } else {
    DBG.println("Failed to open log file");
  }
}

static void sdEnqueueLog(const String &line) {
  if (!SDOK || !sdQueue) return;
  char *buf = (char *)malloc(line.length() + 1);
  if (!buf) return;
  strcpy(buf, line.c_str());
  if (xQueueSend(sdQueue, &buf, 0) != pdTRUE) {
    char *old;
    if (xQueueReceive(sdQueue, &old, 0) == pdTRUE) free(old);
    xQueueSend(sdQueue, &buf, 0);
  }
}

static void sdWriterTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    char *buf = NULL;
    if (xQueueReceive(sdQueue, &buf, portMAX_DELAY) == pdTRUE) {
      if (buf) {
        if (!logFile) {
          if (sdLogPath.length() == 0) sdLogPath = nextTripPath();
          logFile = SD.open(sdLogPath, FILE_WRITE);
          if (logFile) {
            logFile.println("type,ts,ms,a,b,c,d,e,f");
          }
        }
        if (logFile) {
          logFile.println((const char *)buf);
          logFile.flush();
        }
        free(buf);
      }
    }
  }
}

static void appendFilesInDir(const String &path, String &out) {
  File dir = SD.open(path);
  if (!dir) return;
  File entry = dir.openNextFile();
  while (entry) {
    String name = String(entry.name());
    if (entry.isDirectory()) {
      appendFilesInDir(name, out);
    } else {
      String n = name;
      if (n.startsWith("/")) n = n.substring(1);
      out += n + "\n";
    }
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();
}

// ------------------------------
// Microsquirt CAN expansion (remote table write, no ACK/handshake)
//
// MANDATORY frame format (per your spec):
//   CAN ID: 5 (11-bit)
//   DLC: 8
//   [0]=table (7)
//   [1]=off_hi (0)
//   [2]=off_lo (75)
//   [3]=port1 bits
//   [4]=port2 bits
//   [5]=port3 bits
//   [6]=0
//   [7]=0
static const uint16_t MICRO_CAN_ID = 5;
static const uint8_t MICRO_TABLE = 7;
static const uint16_t MICRO_OFFSET = 75;

// Push rate: 20–50 Hz (default 50 Hz)
static const uint32_t INPUT_PUSH_INTERVAL_MS = 20;

// ------------------------------
// User inputs (debounced)
// ------------------------------
// b* are momentary push buttons pulling GPIO to GND
// sw* are SPDT switches pulling GPIO to GND
static const uint32_t INPUT_DEBOUNCE_MS = 25;

static const InputDef USER_INPUTS[] = {
  {"B1", 4, true},
  {"B2", 5, true},
  {"B3", 6, true},
  {"SW1", 7, true},
  {"SW2", 15, true},
  {"SW3", 16, true},
};

static const size_t USER_INPUTS_N = sizeof(USER_INPUTS) / sizeof(USER_INPUTS[0]);
static bool inputStableActive[USER_INPUTS_N] = {0};
static bool inputPrevRawActive[USER_INPUTS_N] = {0};
static unsigned long inputPrevRawMs[USER_INPUTS_N] = {0};

static int findUserInputIndexByGpio(int gpio) {
  for (size_t i = 0; i < USER_INPUTS_N; ++i) {
    if (USER_INPUTS[i].gpio == gpio) return (int)i;
  }
  return -1;
}

static bool readActiveFromGpio(int gpio, bool activeLow) {
  if (gpio < 0) return false;
  int v = digitalRead(gpio);
  bool active = (v != 0);
  if (activeLow) active = !active;
  return active;
}

static void pollUserInputs(unsigned long nowMs) {
  for (size_t i = 0; i < USER_INPUTS_N; ++i) {
    bool rawActive = readActiveFromGpio(USER_INPUTS[i].gpio, USER_INPUTS[i].activeLow);
    if (rawActive != inputPrevRawActive[i]) {
      inputPrevRawActive[i] = rawActive;
      inputPrevRawMs[i] = nowMs;
    }
    if ((nowMs - inputPrevRawMs[i]) >= INPUT_DEBOUNCE_MS) {
      if (rawActive != inputStableActive[i]) {
        inputStableActive[i] = rawActive;
#if DEBUG_SERIAL_LEVEL > 0
        DBG.print("INPUT ");
        DBG.print(USER_INPUTS[i].label);
        DBG.print(" (GPIO");
        DBG.print(USER_INPUTS[i].gpio);
        DBG.print(") = ");
        DBG.println(inputStableActive[i] ? "ON" : "OFF");
#endif

#if ENABLE_SD_LOGGING
        if (SDOK) {
          String line;
          line.reserve(96);
          line += "I,";
          line += String(getTimeSeconds());
          line += ",";
          line += String(nowMs);
          line += ",";
          line += USER_INPUTS[i].label;
          line += ",";
          line += (inputStableActive[i] ? "1" : "0");
          sdEnqueueLog(line);
        }
#endif
      }
    }
  }
}

// Map ESP32 GPIOs to logical inputs (Port 1..3, Bits 0..7).
// GPIO numbers never appear on CAN; only packed bitfields are sent.
// Set gpio=-1 to leave unused.
static const LogicalInput PORT1_MAP[8] = {
  // Inputs are active-low (wired to pull GPIO to GND), so we use INPUT_PULLUP.
  /*b0*/ {4, true},   // b1
  /*b1*/ {5, true},   // b2
  /*b2*/ {6, true},   // b3
  /*b3*/ {7, true},   // sw1
  /*b4*/ {15, true},  // sw2
  /*b5*/ {16, true},  // sw3
  /*b6*/ {-1, true},
  /*b7*/ {-1, true},
};

static const LogicalInput PORT2_MAP[8] = {
  /*b0*/ {-1, true},
  /*b1*/ {-1, true},
  /*b2*/ {-1, true},
  /*b3*/ {-1, true},
  /*b4*/ {-1, true},
  /*b5*/ {-1, true},
  /*b6*/ {-1, true},
  /*b7*/ {-1, true},
};

static const LogicalInput PORT3_MAP[8] = {
  /*b0*/ {-1, true},
  /*b1*/ {-1, true},
  /*b2*/ {-1, true},
  /*b3*/ {-1, true},
  /*b4*/ {-1, true},
  /*b5*/ {-1, true},
  /*b6*/ {-1, true},
  /*b7*/ {-1, true},
};

static unsigned long lastInputPushMs = 0;
static unsigned long inputPushCount = 0;
static byte lastPort1 = 0;
static byte lastPort2 = 0;
static byte lastPort3 = 0;
static byte lastInputPushRc = 255;

static void configureLogicalInputs() {
  auto cfgOne = [](const LogicalInput &li) {
    if (li.gpio < 0) return;
    // Prefer internal pulls to keep wiring simple.
    if (li.activeLow) pinMode(li.gpio, INPUT_PULLUP);
    else pinMode(li.gpio, INPUT_PULLDOWN);
  };
  for (int i = 0; i < 8; ++i) cfgOne(PORT1_MAP[i]);
  for (int i = 0; i < 8; ++i) cfgOne(PORT2_MAP[i]);
  for (int i = 0; i < 8; ++i) cfgOne(PORT3_MAP[i]);
}

static bool readLogicalInput(const LogicalInput &li) {
  if (li.gpio < 0) return false;

  // If this gpio is one of the user-mapped inputs, use the debounced state.
  int idx = findUserInputIndexByGpio(li.gpio);
  if (idx >= 0) return inputStableActive[(size_t)idx];

  return readActiveFromGpio(li.gpio, li.activeLow);
}

static byte packPortByte(const LogicalInput portMap[8]) {
  byte out = 0;
  for (int bit = 0; bit < 8; ++bit) {
    if (readLogicalInput(portMap[bit])) out |= (1U << bit);
  }
  return out;
}

static void pushMicrosquirtInputsIfDue() {
  unsigned long now = millis();
  if (now - lastInputPushMs < INPUT_PUSH_INTERVAL_MS) return;
  lastInputPushMs = now;

  lastPort1 = packPortByte(PORT1_MAP);
  lastPort2 = packPortByte(PORT2_MAP);
  lastPort3 = packPortByte(PORT3_MAP);

  byte out[8] = {0};
  out[0] = MICRO_TABLE;
  out[1] = (byte)(MICRO_OFFSET >> 8);
  out[2] = (byte)(MICRO_OFFSET & 0xFF);
  out[3] = lastPort1;
  out[4] = lastPort2;
  out[5] = lastPort3;
  out[6] = 0;
  out[7] = 0;

  lastInputPushRc = CAN_MICRO.sendMsgBuf(MICRO_CAN_ID, 0, 8, out);
  if (lastInputPushRc == CAN_OK) {
    canSendFailStreak = 0;
    // Treat successful TX as "CAN alive" even if the bus is otherwise quiet.
    lastCanMsg = now;
  } else {
    if (canSendFailStreak < 255) canSendFailStreak++;
  }
  inputPushCount++;

#if ENABLE_SD_LOGGING
  // Periodic summary logging (1 Hz) so we can reconstruct state without logging every frame.
  static unsigned long sdLastSummaryMs = 0;
  if (SDOK && (now - sdLastSummaryMs) >= 1000UL) {
    sdLastSummaryMs = now;
    String line;
    line.reserve(128);
    line += "S,";
    line += String(getTimeSeconds());
    line += ",";
    line += String(now);
    line += ",rx=";
    line += String((unsigned long)canCount);
    line += ",push=";
    line += String((unsigned long)inputPushCount);
    line += ",rc=";
    line += String((int)lastInputPushRc);
    line += ",P1=0x";
    line += String(lastPort1, HEX);
    line += ",P2=0x";
    line += String(lastPort2, HEX);
    line += ",P3=0x";
    line += String(lastPort3, HEX);
    sdEnqueueLog(line);
  }
#endif
}

static bool initCanController(bool verbose) {
  if (verbose) Serial.println("CAN: initializing MCP2515...");
  bool ok = (CAN_MICRO.begin(MCP_ANY, CAN_BITRATE, MCP_CLOCK) == CAN_OK);
  if (ok) {
    CAN_MICRO.setMode(MCP_NORMAL);
    if (verbose) Serial.println("CAN: init OK");
  } else {
    if (verbose) Serial.println("CAN: init FAIL");
  }
  return ok;
}

static uint8_t mcp2515ReadReg(uint8_t reg, uint8_t spiMode) {
  SPI.beginTransaction(SPISettings(SPI_BAUD_INIT, MSBFIRST, spiMode));
  digitalWrite(CAN_CS, LOW);
  SPI.transfer(0x03); // READ
  SPI.transfer(reg);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(CAN_CS, HIGH);
  SPI.endTransaction();
  return v;
}

static void mcp2515Reset() {
  SPI.beginTransaction(SPISettings(SPI_BAUD_INIT, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS, LOW);
  SPI.transfer(0xC0); // RESET
  digitalWrite(CAN_CS, HIGH);
  SPI.endTransaction();
  delay(10);
}

static void probeMcp2515Once() {
#if DEBUG_SERIAL_LEVEL > 0
  // CANSTAT=0x0E, CANCTRL=0x0F. After reset, CANSTAT opmode bits usually show CONFIG (0x80).
  DBG.println("MCP2515 probe (SPI READ regs):");
  uint8_t canstat_m0 = mcp2515ReadReg(0x0E, SPI_MODE0);
  uint8_t canctrl_m0 = mcp2515ReadReg(0x0F, SPI_MODE0);
  DBG.print("  MODE0 CANSTAT=0x"); dbgPrintHexByte(canstat_m0);
  DBG.print(" CANCTRL=0x"); dbgPrintHexByte(canctrl_m0);
  DBG.println();

  uint8_t canstat_m3 = mcp2515ReadReg(0x0E, SPI_MODE3);
  uint8_t canctrl_m3 = mcp2515ReadReg(0x0F, SPI_MODE3);
  DBG.print("  MODE3 CANSTAT=0x"); dbgPrintHexByte(canstat_m3);
  DBG.print(" CANCTRL=0x"); dbgPrintHexByte(canctrl_m3);
  DBG.println();

  DBG.println("  Sending MCP2515 RESET...");
  mcp2515Reset();
  uint8_t canstat2 = mcp2515ReadReg(0x0E, SPI_MODE0);
  uint8_t canctrl2 = mcp2515ReadReg(0x0F, SPI_MODE0);
  DBG.print("  After reset CANSTAT=0x"); dbgPrintHexByte(canstat2);
  DBG.print(" CANCTRL=0x"); dbgPrintHexByte(canctrl2);
  DBG.println();

  if (canstat2 == 0x00 || canstat2 == 0xFF) {
    DBG.println("  PROBE WARNING: MCP2515 may not be responding (wiring/power/pins). ");
    DBG.println("  Check: 3.3V+GND, CS pin, SCK/MOSI/MISO, and module actually uses MCP2515.");
  }
#endif
}

static void maybeRecoverCan(unsigned long now) {
  if (SAFE_MODE) return;

  bool timedOut = (now - lastCanMsg) > canTimeoutMs;
  bool txFailing = (canSendFailStreak >= CAN_SEND_FAIL_STREAK_MAX);
  if (!timedOut && !txFailing) return;
  if (now - canLastReinitAttemptMs < CAN_REINIT_INTERVAL_MS) return;
  canLastReinitAttemptMs = now;

#if DEBUG_SERIAL_LEVEL > 0
  Serial.print("CAN: recovery attempt, reason=");
  if (timedOut) Serial.print("timeout ");
  if (txFailing) Serial.print("tx_fail ");
  Serial.print(" ageMs="); Serial.print((unsigned long)(now - lastCanMsg));
  Serial.print(" txFailStreak="); Serial.println((int)canSendFailStreak);
#endif

  SPI.end();
  delay(10);
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
  SPI.setFrequency(8000000);
  CANOK = initCanController(false);
  canSendFailStreak = 0;
}

static void maybeRecoverWifi(unsigned long now) {
  if (SAFE_MODE) return;
  if (WiFi.getMode() != WIFI_STA) return;
  if (WiFi.status() == WL_CONNECTED) return;
  if (now - wifiLastReconnectMs < WIFI_RECONNECT_INTERVAL_MS) return;
  wifiLastReconnectMs = now;

#if DEBUG_SERIAL_LEVEL > 0
  Serial.println("WiFi: reconnect attempt...");
#endif
  WiFi.disconnect(false);
  WiFi.reconnect();
}

static unsigned long lastFrameId = 0;
static byte lastFrameLen = 0;
static unsigned long lastFrameTs = 0;
static uint32_t drainedLastLoop = 0;

// Mirror of Microsquirt remote table 7 writes observed on the bus.
// This is used for debugging now and for output decoding later.
static const size_t MS_TABLE7_SIZE = 128;
static uint8_t msTable7Mirror[MS_TABLE7_SIZE] = {0};
static unsigned long msTable7RxCount = 0;
static unsigned long msTable7LastRxMs = 0;
static uint16_t msTable7LastOffset = 0;
static byte msTable7LastPayload[5] = {0};

static const LogicalOutput OUTPUTS_DIGITAL[] = {
  // Example placeholders (disabled): set gpio/offset/bit when ready.
  // {gpio, activeHigh, pwmCapable, offset, bit}
  {-1, true, false, 0, 0},
};

static void configureLogicalOutputs() {
  // Safe default: nothing configured until you assign gpios.
  for (size_t i = 0; i < (sizeof(OUTPUTS_DIGITAL) / sizeof(OUTPUTS_DIGITAL[0])); ++i) {
    if (OUTPUTS_DIGITAL[i].gpio < 0) continue;
    pinMode(OUTPUTS_DIGITAL[i].gpio, OUTPUTS_DIGITAL[i].activeHigh ? OUTPUT : OUTPUT);
    digitalWrite(OUTPUTS_DIGITAL[i].gpio, OUTPUTS_DIGITAL[i].activeHigh ? LOW : HIGH);
  }
}

static void applyLogicalOutputsFromMirror() {
  // Safe default: do nothing unless outputs are configured.
  for (size_t i = 0; i < (sizeof(OUTPUTS_DIGITAL) / sizeof(OUTPUTS_DIGITAL[0])); ++i) {
    const LogicalOutput &o = OUTPUTS_DIGITAL[i];
    if (o.gpio < 0) continue;
    if (o.offset >= MS_TABLE7_SIZE) continue;
    bool on = ((msTable7Mirror[o.offset] >> o.bit) & 0x01) != 0;
    if (!o.activeHigh) on = !on;
    digitalWrite(o.gpio, on ? HIGH : LOW);
  }
}

static void handleMicrosquirtTable7WriteFrame(unsigned long canId, byte len, const byte *buf) {
  // Microsquirt remote table write frames (observed on bus)
  // ID: 5, DLC: 8, [0]=table, [1]=off_hi, [2]=off_lo, [3..7]=payload bytes
  if (canId != MICRO_CAN_ID) return;
  if (len != 8) return;
  if (buf[0] != MICRO_TABLE) return;

  uint16_t off = (uint16_t)((buf[1] << 8) | buf[2]);
  msTable7LastOffset = off;
  for (int i = 0; i < 5; ++i) msTable7LastPayload[i] = buf[3 + i];

  // Mirror the 5 payload bytes into our local table image.
  for (int i = 0; i < 5; ++i) {
    uint16_t idx = (uint16_t)(off + i);
    if (idx < MS_TABLE7_SIZE) msTable7Mirror[idx] = buf[3 + i];
  }

  msTable7RxCount++;
  msTable7LastRxMs = millis();

  // Future outputs: if ECU writes output bytes into table 7, we can decode from mirror.
  applyLogicalOutputsFromMirror();
}

// Performance knobs
static const uint32_t CAN_DRAIN_MAX_PER_LOOP = 64;
static const uint32_t WS_BATCH_MAX = 10;
static const uint32_t WS_PUSH_INTERVAL_MS = 25; // ~40 Hz UI updates

// ------------------------------
// Recent frames ring buffer
// ------------------------------
#define MAX_FRAME_LIST 96
struct FrameRecord { unsigned long id; char data[80]; unsigned long ts; };
static FrameRecord framesBuf[MAX_FRAME_LIST];
static int framesHead = 0;
static unsigned long framesTotal = 0;

// Loop statistics
static unsigned long loopCount = 0;
static unsigned long loopRateHz = 0;
static unsigned long lastLoopRateSampleMs = 0;
static unsigned long lastLoopCountSample = 0;

static unsigned long getUptimeSeconds() {
  return millis() / 1000;
}

static void pushFrame(unsigned long id, const byte* buf, byte len) {
  int idx = framesHead % MAX_FRAME_LIST;
  framesBuf[idx].id = id;

  int p = 0;
  for (byte i = 0; i < len && p < (int)(sizeof(framesBuf[idx].data) - 4); ++i) {
    sprintf(&framesBuf[idx].data[p], "%02X ", buf[i]);
    p += 3;
  }
  if (p > 0 && p < (int)sizeof(framesBuf[idx].data)) {
    framesBuf[idx].data[p - 1] = '\0';
  } else {
    framesBuf[idx].data[min(p, (int)sizeof(framesBuf[idx].data) - 1)] = '\0';
  }

  framesBuf[idx].ts = getUptimeSeconds();
  framesHead++;
  framesTotal++;
}

// ------------------------------
// Web / WiFi / SPIFFS
// ------------------------------
WebServer server(80);
WebSocketsServer webSocket(81);

static bool WIFIOK = false;
static String webURL;

// ------------------------------
// ESP-NOW (wireless sensor nodes -> CAN)
// ------------------------------
#ifndef ENABLE_ESPNOW
#define ENABLE_ESPNOW 1
#endif

static bool ESPNOW_OK = false;
static unsigned long espnowRxCount = 0;
static unsigned long espnowDropNotAllowed = 0;
static unsigned long espnowDropBadPacket = 0;
static unsigned long espnowForwardOk = 0;
static unsigned long espnowForwardFail = 0;
static unsigned long espnowLastRxMs = 0;
static uint8_t espnowLastMac[6] = {0};

// Allowlist: only MACs in this list are accepted.
// Stored in SPIFFS at /espnow_peers.json and configurable via HTTP endpoints.
// If empty AND ESPNOW_ALLOW_ALL_IF_EMPTY=1, all peers are accepted.
#ifndef ESPNOW_ALLOW_ALL_IF_EMPTY
#define ESPNOW_ALLOW_ALL_IF_EMPTY 0
#endif

static const char* espnowPeersFile = "/espnow_peers.json";
static const size_t ESPNOW_MAX_PEERS = 12;
static uint8_t espnowAllowedMacs[ESPNOW_MAX_PEERS][6];
static size_t espnowAllowedCount = 0;

static bool macEquals(const uint8_t a[6], const uint8_t b[6]) {
  for (int i = 0; i < 6; ++i) if (a[i] != b[i]) return false;
  return true;
}

static bool isMacAllowed(const uint8_t mac[6]) {
  if (espnowAllowedCount == 0) return (ESPNOW_ALLOW_ALL_IF_EMPTY != 0);
  for (size_t i = 0; i < espnowAllowedCount; ++i) {
    if (macEquals(espnowAllowedMacs[i], mac)) return true;
  }
  return false;
}

static String macToString(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

static bool parseMacString(const String &s, uint8_t out[6]) {
  // Accept AA:BB:CC:DD:EE:FF or AABBCCDDEEFF
  String t = s;
  t.trim();
  t.replace("-", "");
  t.replace(":", "");
  t.replace(" ", "");
  if (t.length() != 12) return false;
  auto hexVal = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };
  for (int i = 0; i < 6; ++i) {
    int a = hexVal(t.charAt(i * 2));
    int b = hexVal(t.charAt(i * 2 + 1));
    if (a < 0 || b < 0) return false;
    out[i] = (uint8_t)((a << 4) | b);
  }
  return true;
}

static void ensureEspNowPeersFileExists() {
  if (SPIFFS.exists(espnowPeersFile)) return;
  File f = SPIFFS.open(espnowPeersFile, FILE_WRITE);
  if (!f) return;
  f.print("{\"peers\":[]}");
  f.close();
}

static bool saveEspNowPeersToSpiffs() {
  StaticJsonDocument<512> doc;
  JsonArray arr = doc.createNestedArray("peers");
  for (size_t i = 0; i < espnowAllowedCount; ++i) {
    arr.add(macToString(espnowAllowedMacs[i]));
  }
  File f = SPIFFS.open(espnowPeersFile, FILE_WRITE);
  if (!f) return false;
  serializeJson(doc, f);
  f.close();
  return true;
}

static void refreshEspNowPeersInDriver() {
  if (!ESPNOW_OK) return;
  // Add configured peers into esp-now driver so we can later support outbound ACKs/encryption.
  // Receiving still uses our allowlist filter.
  for (size_t i = 0; i < espnowAllowedCount; ++i) {
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, espnowAllowedMacs[i], 6);
    peer.channel = 0; // current channel
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }
}

static void loadEspNowPeersFromSpiffs() {
  ensureEspNowPeersFileExists();
  File f = SPIFFS.open(espnowPeersFile, FILE_READ);
  if (!f) return;

  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;

  JsonArray arr = doc["peers"].as<JsonArray>();
  if (arr.isNull()) return;

  espnowAllowedCount = 0;
  for (JsonVariant v : arr) {
    if (espnowAllowedCount >= ESPNOW_MAX_PEERS) break;
    String s = v.as<String>();
    uint8_t mac[6];
    if (!parseMacString(s, mac)) continue;
    bool dup = false;
    for (size_t i = 0; i < espnowAllowedCount; ++i) {
      if (macEquals(espnowAllowedMacs[i], mac)) { dup = true; break; }
    }
    if (dup) continue;
    memcpy(espnowAllowedMacs[espnowAllowedCount++], mac, 6);
  }

  refreshEspNowPeersInDriver();
}

// Packet format: sender packs a CAN frame to forward.
// Keep this small + explicit so it’s easy to implement on other ESP32s.
struct EspNowCanPacket {
  uint32_t magic;   // 'C''A''N''1' = 0x314E4143
  uint8_t version;  // 1
  uint8_t ext;      // 0=11-bit, 1=29-bit
  uint8_t len;      // 0..8
  uint8_t reserved;
  uint32_t can_id;
  uint8_t data[8];
};

static const uint32_t ESPNOW_CAN_MAGIC = 0x314E4143UL;

// Queue to move work out of the WiFi/ESP-NOW callback context.
static const uint8_t ESPNOW_Q_SIZE = 16;
static EspNowCanPacket espnowQ[ESPNOW_Q_SIZE];
static volatile uint8_t espnowQHead = 0;
static volatile uint8_t espnowQTail = 0;
static portMUX_TYPE espnowQMutex = portMUX_INITIALIZER_UNLOCKED;

static bool espnowQueuePush(const EspNowCanPacket &pkt) {
  bool ok = false;
  portENTER_CRITICAL(&espnowQMutex);
  uint8_t next = (uint8_t)((espnowQHead + 1) % ESPNOW_Q_SIZE);
  if (next != espnowQTail) {
    espnowQ[espnowQHead] = pkt;
    espnowQHead = next;
    ok = true;
  }
  portEXIT_CRITICAL(&espnowQMutex);
  return ok;
}

static bool espnowQueuePop(EspNowCanPacket &out) {
  bool ok = false;
  portENTER_CRITICAL(&espnowQMutex);
  if (espnowQTail != espnowQHead) {
    out = espnowQ[espnowQTail];
    espnowQTail = (uint8_t)((espnowQTail + 1) % ESPNOW_Q_SIZE);
    ok = true;
  }
  portEXIT_CRITICAL(&espnowQMutex);
  return ok;
}

// ESP-NOW recv callback signature differs between Arduino-ESP32 core generations.
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len <= 0) return;
  const uint8_t *src = info->src_addr;
  espnowRxCount++;
  espnowLastRxMs = millis();
  memcpy(espnowLastMac, src, 6);

  if (!isMacAllowed(src)) {
    espnowDropNotAllowed++;
    return;
  }

  if (len < (int)sizeof(EspNowCanPacket)) { espnowDropBadPacket++; return; }
  EspNowCanPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != ESPNOW_CAN_MAGIC) { espnowDropBadPacket++; return; }
  if (pkt.version != 1) { espnowDropBadPacket++; return; }
  if (pkt.len > 8) { espnowDropBadPacket++; return; }
  if (!pkt.ext && (pkt.can_id > 0x7FFUL)) { espnowDropBadPacket++; return; }
  if (pkt.ext && (pkt.can_id > 0x1FFFFFFFUL)) { espnowDropBadPacket++; return; }

  (void)espnowQueuePush(pkt);
}
#else
static void onEspNowRecv(const uint8_t *src, const uint8_t *data, int len) {
  if (!src || !data || len <= 0) return;
  espnowRxCount++;
  espnowLastRxMs = millis();
  memcpy(espnowLastMac, src, 6);

  if (!isMacAllowed(src)) {
    espnowDropNotAllowed++;
    return;
  }

  if (len < (int)sizeof(EspNowCanPacket)) { espnowDropBadPacket++; return; }
  EspNowCanPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != ESPNOW_CAN_MAGIC) { espnowDropBadPacket++; return; }
  if (pkt.version != 1) { espnowDropBadPacket++; return; }
  if (pkt.len > 8) { espnowDropBadPacket++; return; }
  if (!pkt.ext && (pkt.can_id > 0x7FFUL)) { espnowDropBadPacket++; return; }
  if (pkt.ext && (pkt.can_id > 0x1FFFFFFFUL)) { espnowDropBadPacket++; return; }

  (void)espnowQueuePush(pkt);
}
#endif

static bool initEspNow() {
#if !ENABLE_ESPNOW
  return false;
#else
  // ESP-NOW requires WiFi to be initialized in STA/AP/AP+STA mode.
  // We keep it in STA mode initially; if we later start an AP for UI setup,
  // we use WIFI_AP_STA so ESP-NOW remains active.
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Ensure WiFi is started (esp_wifi is initialized by the Arduino WiFi lib).
  esp_err_t err = esp_now_init();
  if (err != ESP_OK) {
#if DEBUG_SERIAL_LEVEL > 0
    DBG.print("ESP-NOW init failed: ");
    DBG.println((int)err);
#endif
    return false;
  }

  esp_now_register_recv_cb(onEspNowRecv);

#if DEBUG_SERIAL_LEVEL > 0
  DBG.println("ESP-NOW init OK (recv enabled)");
#endif
  return true;
#endif
}

static void drainEspNowToCan() {
#if ENABLE_ESPNOW
  if (!ESPNOW_OK) return;
  EspNowCanPacket pkt;
  while (espnowQueuePop(pkt)) {
    byte rc = CAN_MICRO.sendMsgBuf(pkt.can_id, pkt.ext ? 1 : 0, pkt.len, (byte *)pkt.data);
    if (rc == CAN_OK) espnowForwardOk++;
    else espnowForwardFail++;
  }
#endif
}

static unsigned long wsLastPushMs = 0;
static unsigned long wsFramesSincePush = 0;

static const String wifiFile = "/wifi.json";
static const String adminTokenFile = "/admin.token";

// derive simple XOR key from chip MAC
static void deriveKey(uint8_t *key, size_t len) {
  uint64_t mac = ESP.getEfuseMac();
  for (size_t i = 0; i < len; ++i) key[i] = (mac >> ((i % 8) * 8)) & 0xFF;
}

static String toHex(const uint8_t *data, size_t len) {
  String s;
  s.reserve(len * 2 + 1);
  const char hex[] = "0123456789ABCDEF";
  for (size_t i = 0; i < len; ++i) {
    s += hex[(data[i] >> 4) & 0x0F];
    s += hex[data[i] & 0x0F];
  }
  return s;
}

static size_t hexToBytes(const String &hex, uint8_t *out, size_t maxOut) {
  size_t hlen = hex.length();
  size_t cnt = 0;
  for (size_t i = 0; i + 1 < hlen && cnt < maxOut; i += 2) {
    char a = hex.charAt(i);
    char b = hex.charAt(i + 1);
    auto val = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return 0;
    };
    out[cnt++] = (val(a) << 4) | val(b);
  }
  return cnt;
}

static String encryptString(const String &plain) {
  if (plain.length() == 0) return String("");
  size_t n = plain.length();
  uint8_t key[8];
  deriveKey(key, sizeof(key));
  uint8_t *buf = (uint8_t *)malloc(n);
  if (!buf) return String("");
  for (size_t i = 0; i < n; ++i) buf[i] = ((uint8_t)plain.charAt(i)) ^ key[i % 8];
  String h = toHex(buf, n);
  free(buf);
  return String("enc:") + h;
}

static String decryptString(const String &cipher) {
  if (!cipher.startsWith("enc:")) return cipher;
  String hex = cipher.substring(4);
  size_t hlen = hex.length() / 2;
  uint8_t *buf = (uint8_t *)malloc(hlen);
  if (!buf) return String("");
  size_t got = hexToBytes(hex, buf, hlen);
  uint8_t key[8];
  deriveKey(key, sizeof(key));
  String out;
  out.reserve(got + 1);
  for (size_t i = 0; i < got; ++i) out += (char)(buf[i] ^ key[i % 8]);
  free(buf);
  return out;
}

static void ensureWifiJsonExists() {
  if (SPIFFS.exists(wifiFile)) return;
  File f = SPIFFS.open(wifiFile, FILE_WRITE);
  if (!f) return;
  f.print("{\"networks\":[]}");
  f.close();
}

static void ensureAdminTokenExists() {
  if (SPIFFS.exists(adminTokenFile)) return;
  uint8_t t[16];
  for (int i = 0; i < 16; ++i) {
    uint32_t r = esp_random();
    t[i] = (r >> (8 * (i % 4))) & 0xFF;
  }
  String tok = toHex(t, 16);
  File f = SPIFFS.open(adminTokenFile, FILE_WRITE);
  if (f) {
    f.print(tok);
    f.close();
  }
}

static String readAdminToken() {
  if (!SPIFFS.exists(adminTokenFile)) return String("");
  File f = SPIFFS.open(adminTokenFile, FILE_READ);
  if (!f) return String("");
  String t;
  while (f.available()) t += (char)f.read();
  f.close();
  t.trim();
  return t;
}

static bool checkAuth() {
  String provided;
  if (server.hasHeader("X-Auth-Token")) provided = server.header("X-Auth-Token");
  else if (server.hasHeader("Authorization")) {
    String a = server.header("Authorization");
    if (a.startsWith("Bearer ")) provided = a.substring(7);
  }
  if (provided.length() == 0 && server.hasArg("token")) provided = server.arg("token");

  String real = readAdminToken();
  if (real.length() > 0 && provided.equals(real)) return true;

  // allow same /24 LAN without token
  IPAddress rem = server.client().remoteIP();
  IPAddress loc = WiFi.localIP();
  if (rem == IPAddress(0, 0, 0, 0) || rem == loc) return true;
  if (rem[0] == loc[0] && rem[1] == loc[1] && rem[2] == loc[2]) return true;

  return false;
}

static bool connectSavedNetworks() {
  ensureWifiJsonExists();
  File f = SPIFFS.open(wifiFile, FILE_READ);
  if (!f) return false;

  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  JsonArray arr = doc["networks"].as<JsonArray>();
  if (arr.isNull() || arr.size() == 0) return false;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  for (JsonObject o : arr) {
    String ss = String((const char *)o["ssid"]);
    String pw = String((const char *)o["pass"]);
    pw = decryptString(pw);

    DBG.print("Trying WiFi: ");
    DBG.println(ss);

    WiFi.begin(ss.c_str(), pw.c_str());

    unsigned long start = millis();
    while (millis() - start < 9000) {
      if (WiFi.status() == WL_CONNECTED) {
        DBG.println("WiFi connected");
        DBG.println(WiFi.localIP());
        webURL = String("http://") + WiFi.localIP().toString();
        return true;
      }
      feedWdt();
      delay(150);
    }

    WiFi.disconnect(true);
    delay(200);
  }

  return false;
}

static void startAPMode() {
  // Keep STA enabled so ESP-NOW remains active while running AP.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAP("CAN-Expander", "password");
  IPAddress ip = WiFi.softAPIP();
  DBG.print("AP mode, IP: ");
  DBG.println(ip);
  webURL = String("http://") + ip.toString();
}

static String contentTypeForPath(const String &p) {
  if (p.endsWith(".html")) return "text/html";
  if (p.endsWith(".css")) return "text/css";
  if (p.endsWith(".js")) return "application/javascript";
  if (p.endsWith(".json")) return "application/json";
  if (p.endsWith(".svg")) return "image/svg+xml";
  if (p.endsWith(".png")) return "image/png";
  if (p.endsWith(".jpg") || p.endsWith(".jpeg")) return "image/jpeg";
  return "text/plain";
}

static bool serveSpiffsFile(String path) {
  if (path == "/") path = "/index.html";
  if (!SPIFFS.exists(path)) return false;

  File f = SPIFFS.open(path, FILE_READ);
  if (!f) return false;

  server.streamFile(f, contentTypeForPath(path));
  f.close();
  return true;
}

static bool otaAuthorized = false;

static void handleRoot() {
  if (!serveSpiffsFile("/index.html")) server.send(404, "text/plain", "Not found");
}

static void handleGenericFile() {
  String path = server.uri();
  if (!serveSpiffsFile(path)) server.send(404, "text/plain", "Not found: " + path);
}

static void handleAddWiFi() {
  if (!checkAuth()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (!server.hasArg("ssid") || !server.hasArg("pass")) {
    server.send(400, "text/plain", "Missing ssid/pass");
    return;
  }

  String ss = server.arg("ssid");
  String pw = server.arg("pass");

  ensureWifiJsonExists();

  File f = SPIFFS.open(wifiFile, FILE_READ);
  if (!f) {
    server.send(500, "text/plain", "Failed to open wifi.json");
    return;
  }

  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    server.send(500, "text/plain", "wifi.json parse error");
    return;
  }

  JsonArray arr = doc["networks"].as<JsonArray>();
  if (arr.isNull()) arr = doc.createNestedArray("networks");

  JsonObject obj = arr.createNestedObject();
  obj["ssid"] = ss;
  obj["pass"] = encryptString(pw);

  File w = SPIFFS.open(wifiFile, FILE_WRITE);
  if (!w) {
    server.send(500, "text/plain", "Failed to write wifi.json");
    return;
  }
  serializeJson(doc, w);
  w.close();

  server.sendHeader("Location", "/wifi.html", true);
  server.send(302, "text/plain", "OK");
}

static void handleOTAUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    DBG.println("OTA: begin");
    otaAuthorized = checkAuth();
    if (!otaAuthorized) {
      DBG.println("OTA: unauthorized upload attempt");
      return;
    }
    if (!Update.begin()) DBG.println("OTA begin failed");
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (!otaAuthorized) return;
    if (Update.write(up.buf, up.currentSize) != up.currentSize) DBG.println("OTA write failed");
  } else if (up.status == UPLOAD_FILE_END) {
    if (!otaAuthorized) {
      DBG.println("OTA aborted: unauthorized");
      return;
    }
    if (Update.end(true)) DBG.println("OTA success, rebooting...");
    else DBG.println("OTA end failed");
  }
}

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  (void)payload;
  (void)length;
  if (type == WStype_CONNECTED) {
    IPAddress ip = webSocket.remoteIP(num);
    DBG.print("WebSocket client connected: ");
    DBG.println(ip);
  }
}

static void wsBroadcast(String txt) {
  webSocket.broadcastTXT(txt);
}

static void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
#if (STATUS_LED_PIN >= 0)
  // neopixelWrite is provided by Arduino-ESP32 core for WS2812-style LEDs
  neopixelWrite(STATUS_LED_PIN, r, g, b);
#else
  (void)r; (void)g; (void)b;
#endif
}

static void updateStatusLed() {
  // Heartbeat pulse on top of state color.
  // CAN down => red blink. CAN ok + WiFi ok => green pulse. CAN ok + not WiFi => yellow pulse.
  unsigned long now = millis();
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);

  uint8_t r = 0, g = 0, b = 0;
  if (SAFE_MODE) {
    r = 12; g = 0; b = 18; // purple-ish
  } else if (!CANOK) {
    r = 40; g = 0; b = 0;
  } else if (wifiConnected) {
    r = 0; g = 30; b = 0;
  } else {
    r = 25; g = 10; b = 0;
  }

  // Pulse / blink shaping
  if (!CANOK) {
    // 1 Hz red blink
    bool on = ((now % 1000UL) < 250UL);
    setStatusLed(on ? r : 0, on ? g : 0, on ? b : 0);
    return;
  }

  // Heartbeat: quick double-pulse each second.
  uint16_t t = (uint16_t)(now % 1000UL);
  uint8_t boost = 0;
  if (t < 60) boost = 35;
  else if (t >= 140 && t < 200) boost = 20;

  auto satAdd = [](uint8_t base, uint8_t add) -> uint8_t {
    uint16_t v = (uint16_t)base + (uint16_t)add;
    return (v > 255) ? 255 : (uint8_t)v;
  };

  setStatusLed(satAdd(r, boost), satAdd(g, boost), satAdd(b, boost));
}

static bool parseId(const String &s, unsigned long &outId) {
  if (s.length() == 0) return false;
  const char *c = s.c_str();
  char *endp = nullptr;
  unsigned long v = 0;
  if (s.startsWith("0x") || s.startsWith("0X")) v = strtoul(c + 2, &endp, 16);
  else v = strtoul(c, &endp, 0);
  if (endp == c) return false;
  outId = v;
  return true;
}

static bool parseHexBytes(const String &hex, byte *out, byte &outLen) {
  // Accept forms like: "01 02 0A FF" or "01020AFF" (spaces optional)
  outLen = 0;
  String s = hex;
  s.replace(" ", "");
  s.replace("\n", "");
  s.replace("\r", "");
  s.replace("\t", "");
  s.trim();
  if (s.length() == 0) { outLen = 0; return true; }
  if ((s.length() % 2) != 0) return false;

  byte n = (byte)min((int)(s.length() / 2), 8);
  for (byte i = 0; i < n; ++i) {
    char a = s.charAt(i * 2);
    char b = s.charAt(i * 2 + 1);
    auto val = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return -1;
    };
    int va = val(a);
    int vb = val(b);
    if (va < 0 || vb < 0) return false;
    out[i] = (byte)((va << 4) | vb);
  }
  outLen = n;
  return true;
}

static void broadcastFramesBatch(uint32_t maxFrames) {
  if (webSocket.connectedClients() == 0) return;

  unsigned long total = min((unsigned long)MAX_FRAME_LIST, framesTotal);
  if (total == 0) return;

  uint32_t sendCount = min((unsigned long)maxFrames, total);

  // Build a compact JSON array: [{id:"0x123",data:"..",ts:123},...]
  // Manual string build is faster than allocating a DynamicJsonDocument at high rate.
  String out;
  out.reserve(sendCount * 64);
  out += '[';

  for (uint32_t i = 0; i < sendCount; ++i) {
    int idx = (framesHead - 1 - (int)i + MAX_FRAME_LIST) % MAX_FRAME_LIST;

    char idbuf[16];
    sprintf(idbuf, "0x%lX", framesBuf[idx].id);

    if (i) out += ',';
    out += "{\"id\":\"";
    out += idbuf;
    out += "\",\"data\":\"";
    out += framesBuf[idx].data;
    out += "\",\"ts\":";
    out += String(framesBuf[idx].ts);
    out += '}';
  }

  out += ']';
  wsBroadcast(out);
}

#if ENABLE_I2C_DISPLAY
static Adafruit_SSD1306 display(128, 32, &Wire, -1);
static bool DISPLAY_OK = false;
static unsigned long displayLastMs = 0;

static void updateDisplay() {
  if (!DISPLAY_OK) return;
  unsigned long now = millis();
  if (now - displayLastMs < 100) return; // ~10 Hz display refresh
  displayLastMs = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("CAN:"); display.print(CANOK ? "OK" : "NO");
  display.print("  WiFi:"); display.print((WiFi.status() == WL_CONNECTED) ? "OK" : "NO");

  // Line 2: buttons
  display.setCursor(0, 10);
  display.print("B1:"); display.print(inputStableActive[0] ? 1 : 0);
  display.print(" B2:"); display.print(inputStableActive[1] ? 1 : 0);
  display.print(" B3:"); display.print(inputStableActive[2] ? 1 : 0);

  // Line 3: switches + packed byte
  display.setCursor(0, 20);
  display.print("S1:"); display.print(inputStableActive[3] ? 1 : 0);
  display.print(" S2:"); display.print(inputStableActive[4] ? 1 : 0);
  display.print(" S3:"); display.print(inputStableActive[5] ? 1 : 0);
  display.print(" P1:");
  if (lastPort1 < 16) display.print('0');
  display.print(lastPort1, HEX);

  display.display();
}
#endif

void setup() {
  Serial.begin(115200);
  delay(250);

#if HAS_USB_CDC
  // Bring up USB CDC at runtime so logs can appear on the USB port.
  USB.begin();
  delay(50);
#endif

  // Route debug output to UART Serial and USB CDC (if present)
  DBG.a = &Serial;
  DBG.b = nullptr;

  // WDT early so init phases are protected.
  initWdt();
  feedWdt();

  // Boot-loop detection: if we keep rebooting before reaching "stable" runtime,
  // increment a counter in RTC and enter SAFE_MODE.
  bootFailCount++;
  if (bootFailCount >= SAFE_MODE_BOOT_THRESHOLD) SAFE_MODE = true;

  esp_reset_reason_t rr = esp_reset_reason();

#if DEBUG_SERIAL_LEVEL > 0
  DBG.println();
  DBG.println("--- ESP32-S3 CAN-Only Boot ---");
  DBG.print("Reset reason: "); DBG.println(resetReasonStr(rr));
  DBG.print("bootFailCount: "); DBG.print((unsigned long)bootFailCount);
  DBG.print(" SAFE_MODE: "); DBG.println(SAFE_MODE ? "YES" : "NO");
  DBG.print("CPU MHz: "); DBG.println(getCpuFrequencyMhz());
  DBG.print("Chip rev: "); DBG.println((int)ESP.getChipRevision());
  DBG.print("SDK: "); DBG.println(ESP.getSdkVersion());
  DBG.print("Flash size: "); DBG.println((unsigned long)ESP.getFlashChipSize());
  DBG.print("Pins: SDA="); DBG.print(I2C_SDA);
  DBG.print(" SCL="); DBG.print(I2C_SCL);
  DBG.print(" INT="); DBG.print(CAN_INT);
  DBG.print(" SCK="); DBG.print(CAN_SCK);
  DBG.print(" MOSI="); DBG.print(CAN_MOSI);
  DBG.print(" MISO="); DBG.print(CAN_MISO);
  DBG.print(" CS="); DBG.println(CAN_CS);
  DBG.print("CAN: bitrate=500k, mcp_clock=8MHz, id="); DBG.println((int)MICRO_CAN_ID);
#endif

  // I2C (if you later add an OLED again)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  feedWdt();

#if ENABLE_I2C_DISPLAY
  // Common SSD1306 I2C address is 0x3C
  DISPLAY_OK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!DISPLAY_OK) {
    // Some modules use 0x3D
    DISPLAY_OK = display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }
  if (DISPLAY_OK) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ESP32-S3 CAN");
    display.println("Booting...");
    display.display();
  }
#endif

  setStatusLed(0, 0, 30); // boot blue

  // SPI for MCP2515
  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
  // Start conservative for bring-up; raise after CAN init succeeds.
  SPI.setFrequency(SPI_BAUD_INIT);

  // Quick SPI-level probe so we know if the chip is even responding.
  probeMcp2515Once();

  // SPIFFS
  DBG.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(false)) {
    DBG.println("SPIFFS mount failed, trying format");
    if (!SPIFFS.begin(true)) {
      DBG.println("SPIFFS still failed");
    }
  }
  ensureWifiJsonExists();
  ensureAdminTokenExists();
  ensureEspNowPeersFileExists();
  feedWdt();

  // Load ESP-NOW peer allowlist (after SPIFFS is mounted)
  loadEspNowPeersFromSpiffs();

  // ESP-NOW (bring up early; works with STA/AP+STA)
  ESPNOW_OK = initEspNow();

#if ENABLE_SD_LOGGING
  // SD (logging only; web UI still served from SPIFFS)
  DBG.println("Starting SD...");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  SDOK = SD.begin(SD_CS, SPI);
  DBG.print("SD status: ");
  DBG.println(SDOK ? "OK" : "FAIL");

  if (SDOK) {
    startNewTrip();
    sdQueue = xQueueCreate(SD_QUEUE_SIZE, sizeof(char *));
    if (sdQueue) {
      xTaskCreate(sdWriterTask, "sdWriter", 4096, NULL, 1, NULL);
    }
  }
#endif

  // CAN
  DBG.println("Starting CAN...");
  CANOK = initCanController(true);
  if (CANOK) {
    SPI.setFrequency(SPI_BAUD_RUN);
  }
  pinMode(CAN_INT, INPUT);
  updateStatusLed();

  configureLogicalInputs();
  configureLogicalOutputs();

  // WiFi
  DBG.println("Connecting WiFi...");
  if (SAFE_MODE) {
    DBG.println("SAFE_MODE: AP-only (skipping STA connect)");
    startAPMode();
    WIFIOK = false;
  } else {
    if (connectSavedNetworks()) {
      WIFIOK = true;
    } else {
      startAPMode();
      WIFIOK = false;
    }
  }
  feedWdt();

  // Try to acquire SNTP time when STA connected (helps SD log folder names)
  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    unsigned long tstart = millis();
    while (millis() - tstart < 2500) {
      time_t nowt = time(nullptr);
      if ((uint32_t)nowt > 1700000000UL) {
        timeValid = true;
        break;
      }
      feedWdt();
      delay(50);
    }
    if (timeValid) DBG.println("Time: SNTP OK");
    else DBG.println("Time: SNTP not set (yet)");
  }

#if DEBUG_SERIAL_LEVEL > 0
  if (WiFi.status() == WL_CONNECTED) {
    DBG.print("WiFi STA connected, IP: ");
    DBG.println(WiFi.localIP());
  } else {
    DBG.print("WiFi AP mode, IP: ");
    DBG.println(WiFi.softAPIP());
  }
  DBG.print("Web URL: ");
  DBG.println(webURL);
#endif

  // mDNS (optional)
  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin("can-expander")) {
      DBG.println("mDNS started: http://can-expander.local");
      webURL = String("http://can-expander.local");
    }
  }

  DBG.print("Web URL: ");
  DBG.println(webURL);

  // Web routes
  server.on("/", handleRoot);
  server.on("/addwifi", HTTP_GET, handleAddWiFi);
  server.on("/addwifi", HTTP_POST, handleAddWiFi);

  server.on("/update", HTTP_POST,
            []() {
              if (!checkAuth()) {
                server.send(401, "text/plain", "Unauthorized");
                return;
              }
              server.send(200, "text/plain", "OTA done, rebooting");
              delay(500);
              ESP.restart();
            },
            handleOTAUpload);

  server.on("/status.json", HTTP_GET, []() {
    StaticJsonDocument<384> doc;
    doc["safe_mode"] = SAFE_MODE;
    doc["boot_fail"] = (unsigned long)bootFailCount;
    doc["gps"] = "DISABLED";
    doc["sd"] = SDOK ? "OK" : "NO";
    doc["can"] = CANOK ? "OK" : "NO";
    doc["wifi"] = (WiFi.status() == WL_CONNECTED) ? "OK" : "NO";
    doc["frames"] = (unsigned long)canCount;
    doc["push"] = (unsigned long)inputPushCount;
    doc["p1"] = lastPort1;
    doc["p2"] = lastPort2;
    doc["p3"] = lastPort3;
    doc["push_rc"] = (int)lastInputPushRc;
      doc["t7_rx"] = (unsigned long)msTable7RxCount;
      doc["t7_off"] = (unsigned long)msTable7LastOffset;
    doc["espnow"] = ESPNOW_OK ? "OK" : "NO";
    doc["en_rx"] = (unsigned long)espnowRxCount;
    doc["en_drop_na"] = (unsigned long)espnowDropNotAllowed;
    doc["en_drop_bad"] = (unsigned long)espnowDropBadPacket;
    doc["en_fwd_ok"] = (unsigned long)espnowForwardOk;
    doc["en_fwd_fail"] = (unsigned long)espnowForwardFail;
    doc["en_last_ms"] = (unsigned long)espnowLastRxMs;
    doc["en_last_mac"] = macToString(espnowLastMac);
    doc["en_peers"] = (unsigned long)espnowAllowedCount;
    doc["mph"] = 0;
    doc["lat"] = 0.0;
    doc["lon"] = 0.0;
    doc["sats"] = 0;
    String out;
    serializeJson(doc, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
  });

#if ENABLE_ESPNOW
  // ESP-NOW peer allowlist management
  // GET /espnow/peers
  // POST /espnow/peers/add?mac=AA:BB:CC:DD:EE:FF
  // POST /espnow/peers/del?mac=AA:BB:CC:DD:EE:FF
  server.on("/espnow/peers", HTTP_GET, []() {
    StaticJsonDocument<512> doc;
    doc["allow_all_if_empty"] = (ESPNOW_ALLOW_ALL_IF_EMPTY != 0);
    JsonArray arr = doc.createNestedArray("peers");
    for (size_t i = 0; i < espnowAllowedCount; ++i) arr.add(macToString(espnowAllowedMacs[i]));
    String out;
    serializeJson(doc, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
  });

  server.on("/espnow/peers/add", HTTP_POST, []() {
    if (!checkAuth()) { server.send(401, "text/plain", "Unauthorized"); return; }
    if (!server.hasArg("mac")) { server.send(400, "text/plain", "mac required"); return; }
    uint8_t mac[6];
    if (!parseMacString(server.arg("mac"), mac)) { server.send(400, "text/plain", "bad mac"); return; }
    for (size_t i = 0; i < espnowAllowedCount; ++i) {
      if (macEquals(espnowAllowedMacs[i], mac)) { server.send(200, "text/plain", "exists"); return; }
    }
    if (espnowAllowedCount >= ESPNOW_MAX_PEERS) { server.send(400, "text/plain", "peer list full"); return; }
    memcpy(espnowAllowedMacs[espnowAllowedCount++], mac, 6);
    saveEspNowPeersToSpiffs();
    refreshEspNowPeersInDriver();
    server.send(200, "text/plain", "OK");
  });

  server.on("/espnow/peers/del", HTTP_POST, []() {
    if (!checkAuth()) { server.send(401, "text/plain", "Unauthorized"); return; }
    if (!server.hasArg("mac")) { server.send(400, "text/plain", "mac required"); return; }
    uint8_t mac[6];
    if (!parseMacString(server.arg("mac"), mac)) { server.send(400, "text/plain", "bad mac"); return; }
    bool found = false;
    for (size_t i = 0; i < espnowAllowedCount; ++i) {
      if (macEquals(espnowAllowedMacs[i], mac)) {
        found = true;
        for (size_t j = i + 1; j < espnowAllowedCount; ++j) {
          memcpy(espnowAllowedMacs[j - 1], espnowAllowedMacs[j], 6);
        }
        espnowAllowedCount--;
        break;
      }
    }
    saveEspNowPeersToSpiffs();
    server.send(found ? 200 : 404, "text/plain", found ? "OK" : "not found");
  });
#endif

#if ENABLE_SD_LOGGING
  // SD log listing + downloading (mirrors the parent sketch behavior)
  server.on("/sd/list", HTTP_GET, []() {
    if (!SDOK || !SD.exists("/logs")) {
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/plain", "");
      return;
    }
    String out;
    appendFilesInDir(String("/logs"), out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", out);
  });

  server.on("/sd/download", HTTP_GET, []() {
    if (!SDOK) { server.send(500, "text/plain", "SD not mounted"); return; }
    if (!server.hasArg("name")) { server.send(400, "text/plain", "name required"); return; }
    String name = server.arg("name");
    if (!name.startsWith("/")) name = "/" + name;
    if (!SD.exists(name)) { server.send(404, "text/plain", "not found"); return; }
    File f = SD.open(name, FILE_READ);
    if (!f) { server.send(500, "text/plain", "open failed"); return; }
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.streamFile(f, "application/octet-stream");
    f.close();
  });
#endif

  // Transmit a CAN frame (authenticated)
  // POST (or GET) /can/send?id=0x123&data=11223344&ext=0&token=...
  // data: up to 8 bytes hex, with or without spaces ("11 22 33 44" is ok)
  server.on("/can/send", HTTP_POST, []() {
    if (!checkAuth()) { server.send(401, "text/plain", "Unauthorized"); return; }
    if (!server.hasArg("id") || !server.hasArg("data")) { server.send(400, "text/plain", "id and data required"); return; }

    unsigned long id = 0;
    if (!parseId(server.arg("id"), id)) { server.send(400, "text/plain", "bad id"); return; }

    byte buf[8];
    byte len = 0;
    if (!parseHexBytes(server.arg("data"), buf, len)) { server.send(400, "text/plain", "bad data hex"); return; }

    bool ext = false;
    if (server.hasArg("ext")) {
      String e = server.arg("ext");
      ext = (e == "1" || e.equalsIgnoreCase("true") || e.equalsIgnoreCase("yes"));
    }

    byte rc = CAN_MICRO.sendMsgBuf(id, ext ? 1 : 0, len, buf);
    StaticJsonDocument<192> doc;
    doc["ok"] = (rc == CAN_OK);
    doc["rc"] = (int)rc;
    doc["id"] = String("0x") + String(id, HEX);
    doc["len"] = (int)len;
    String out; serializeJson(doc, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send((rc == CAN_OK) ? 200 : 500, "application/json", out);
  });

  server.on("/can/send", HTTP_GET, []() {
    // Convenience for quick testing from a browser/address bar
    if (!checkAuth()) { server.send(401, "text/plain", "Unauthorized"); return; }
    if (!server.hasArg("id") || !server.hasArg("data")) { server.send(400, "text/plain", "id and data required"); return; }

    unsigned long id = 0;
    if (!parseId(server.arg("id"), id)) { server.send(400, "text/plain", "bad id"); return; }

    byte buf[8];
    byte len = 0;
    if (!parseHexBytes(server.arg("data"), buf, len)) { server.send(400, "text/plain", "bad data hex"); return; }

    bool ext = false;
    if (server.hasArg("ext")) {
      String e = server.arg("ext");
      ext = (e == "1" || e.equalsIgnoreCase("true") || e.equalsIgnoreCase("yes"));
    }

    byte rc = CAN_MICRO.sendMsgBuf(id, ext ? 1 : 0, len, buf);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send((rc == CAN_OK) ? 200 : 500, "text/plain", (rc == CAN_OK) ? "OK" : "FAIL");
  });

  server.on("/frames", HTTP_GET, []() {
    DynamicJsonDocument doc(4096);
    JsonArray arr = doc.to<JsonArray>();
    unsigned long total = min((unsigned long)MAX_FRAME_LIST, framesTotal);
    for (unsigned long i = 0; i < total; ++i) {
      int idx = (framesHead - 1 - i + MAX_FRAME_LIST) % MAX_FRAME_LIST;
      JsonObject o = arr.createNestedObject();
      char idbuf[16];
      sprintf(idbuf, "0x%lX", framesBuf[idx].id);
      o["id"] = idbuf;
      o["data"] = framesBuf[idx].data;
      o["ts"] = framesBuf[idx].ts;
    }
    String out;
    serializeJson(doc, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
  });

  server.on("/frames.txt", HTTP_GET, []() {
    String out;
    unsigned long total = min((unsigned long)MAX_FRAME_LIST, framesTotal);
    for (unsigned long i = 0; i < total; ++i) {
      int idx = (framesHead - 1 - i + MAX_FRAME_LIST) % MAX_FRAME_LIST;
      out += String("0x") + String(framesBuf[idx].id, HEX) + "|" + String(framesBuf[idx].data) + "|" + String(framesBuf[idx].ts) + "\n";
    }
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", out);
  });

  server.onNotFound(handleGenericFile);
  server.begin();
  DBG.println("HTTP server started");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  lastLoopRateSampleMs = millis();
  lastLoopCountSample = 0;
}

void loop() {
  feedWdt();
  server.handleClient();
  webSocket.loop();

  // Forward ESP-NOW packets onto CAN (done in main loop for safety).
  drainEspNowToCan();

  loopCount++;
  unsigned long now = millis();

  // Read/debounce user inputs frequently so OLED + CAN payloads are stable.
  pollUserInputs(now);

  if (now - lastLoopRateSampleMs >= 1000) {
    unsigned long delta = loopCount - lastLoopCountSample;
    loopRateHz = delta;
    lastLoopCountSample = loopCount;
    lastLoopRateSampleMs = now;
  }

  // Drain CAN RX as fast as possible while INT is asserted.
  uint32_t drained = 0;
  while (!digitalRead(CAN_INT) && drained < CAN_DRAIN_MAX_PER_LOOP) {
    unsigned long id;
    byte len;
    byte buf[8];
    CAN_MICRO.readMsgBuf(&id, &len, buf);

    lastCanMsg = now;
    canCount++;
    drained++;
    wsFramesSincePush++;

    lastFrameId = id;
    lastFrameLen = len;
    lastFrameTs = now;

    pushFrame(id, buf, len);

  #if DEBUG_SERIAL_FRAMES
    dbgPrintCanFrame(id, len, buf);
  #endif

    // Observe Microsquirt table writes (for future outputs / debug)
    handleMicrosquirtTable7WriteFrame(id, len, buf);
  }

  drainedLastLoop = drained;

  // Push packed digital inputs into Microsquirt remote table at 20–50 Hz.
  // This is independent of receiving traffic.
  pushMicrosquirtInputsIfDue();

  // Recovery hooks (noise/EMI can wedge peripherals)
  maybeRecoverCan(now);
  maybeRecoverWifi(now);

  // If we have been stable for a while, clear the boot-loop counter so future resets
  // don't get stuck in SAFE_MODE unnecessarily.
  if (millis() > 60000) {
    bootFailCount = 0;
  }

  CANOK = (now - lastCanMsg < canTimeoutMs);
  updateStatusLed();

#if DEBUG_SERIAL_LEVEL > 0
  // WiFi state changes (rate-limited)
  if (now - dbgLastWifiStateMs >= 500) {
    dbgLastWifiStateMs = now;
    wl_status_t ws = WiFi.status();
    if (ws != dbgLastWifiStatus) {
      dbgLastWifiStatus = ws;
      DBG.print("WiFi status changed: ");
      DBG.println((int)ws);
      if (ws == WL_CONNECTED) {
        DBG.print("WiFi IP: ");
        DBG.println(WiFi.localIP());
      }
    }
  }

  if (CANOK != dbgLastCanOk) {
    dbgLastCanOk = CANOK;
    DBG.print("CANOK changed: ");
    DBG.println(CANOK ? "OK" : "NO");
  }

  // Periodic status (lots of useful debug, but not per-frame)
  const unsigned long statusInterval = (DEBUG_SERIAL_LEVEL >= 2) ? 500 : 1500;
  if (now - dbgLastStatusMs >= statusInterval) {
    dbgLastStatusMs = now;

    DBG.print("STAT ms="); DBG.print(now);
    DBG.print(" loopHz="); DBG.print((unsigned long)loopRateHz);
    DBG.print(" drained="); DBG.print((unsigned long)drainedLastLoop);
    DBG.print(" rx="); DBG.print((unsigned long)canCount);
    DBG.print(" push="); DBG.print((unsigned long)inputPushCount);
    DBG.print(" rc="); DBG.print((int)lastInputPushRc);
    DBG.print(" P=");
    dbgPrintHexByte((uint8_t)lastPort1); DBG.print(' ');
    dbgPrintHexByte((uint8_t)lastPort2); DBG.print(' ');
    dbgPrintHexByte((uint8_t)lastPort3);
    DBG.print(" lastId=0x"); DBG.print(lastFrameId, HEX);
    DBG.print(" lastAgeMs="); DBG.print((unsigned long)(now - lastFrameTs));
    DBG.print(" t7rx="); DBG.print((unsigned long)msTable7RxCount);
    DBG.print(" t7off="); DBG.print((unsigned long)msTable7LastOffset);
    DBG.println();
  }
#endif

#if ENABLE_I2C_DISPLAY
  updateDisplay();
#endif

  // Batch WebSocket pushes for higher throughput.
  if (webSocket.connectedClients() > 0 && (now - wsLastPushMs) >= WS_PUSH_INTERVAL_MS) {
    if (wsFramesSincePush > 0) {
      broadcastFramesBatch(WS_BATCH_MAX);
      wsFramesSincePush = 0;
    }
    wsLastPushMs = now;
  }
}
