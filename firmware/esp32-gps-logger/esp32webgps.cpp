/*
  ESP32 CAN / GPS Logger
  - GPS via TinyGPSPlus
  - CAN via MCP2515 (MCP_CAN)
  - SD card for logs and web files
  - SPIFFS for wifi.json
  - WebServer: serves index.html, css, js, etc from SD
  - OTA firmware upload (/update)
  - OLED 128x32 status with scrolling lines
  - mDNS: http://logger.local (when STA connected)

  Board: ESP32 Dev Module
  Partition scheme: Default 4MB with spiffs
*/

#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <time.h>
#include <WebSocketsServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Forward declarations to avoid Arduino auto-prototype issues
struct FrameRecord;
extern File logFile;
void broadcastFrame(const FrameRecord &fr);
unsigned long getTimeSeconds();
String nextTripPath();

// ------------------------------
// OLED
// ------------------------------
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 32, &Wire, -1);
bool OLED_INITED = false;

// Web URL to display (mdns or IP or AP)
String webURL = "";

// --- Boot message queue (non-blocking) ---
struct BootMsg { String text; unsigned long dur; };
const int BOOT_QUEUE_LEN = 8;
BootMsg bootQueue[BOOT_QUEUE_LEN];
int bootHead = 0, bootTail = 0;
bool bootActive = false;
BootMsg currentBoot;
unsigned long bootStartMillis = 0;

// enqueue a boot message (overwrites oldest if full)
void enqueueBootMessage(const String &txt, unsigned long durMs) {
  bootQueue[bootTail] = {txt, durMs};
  bootTail = (bootTail + 1) % BOOT_QUEUE_LEN;
  if (bootTail == bootHead) { // full, drop oldest
    bootHead = (bootHead + 1) % BOOT_QUEUE_LEN;
  }
}

void processBootMessages() {
  if (!OLED_INITED) return;
  unsigned long now = millis();
  if (!bootActive) {
    if (bootHead != bootTail) {
      currentBoot = bootQueue[bootHead];
      bootHead = (bootHead + 1) % BOOT_QUEUE_LEN;
      bootMessage(currentBoot.text);
      bootStartMillis = now;
      bootActive = true;
    }
  } else {
    if (now - bootStartMillis >= currentBoot.dur) {
      // clear and move on
      display.clearDisplay(); display.display();
      bootActive = false;
    }
  }
}

// --- WebSocket server for pushing frames/status ---
WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.print("WebSocket client connected: "); Serial.println(ip);
  }
}

// helper to broadcast JSON text to all websocket clients
// take by value because WebSocketsServer::broadcastTXT requires a non-const String&
void wsBroadcast(String txt) {
  webSocket.broadcastTXT(txt);
}

// --- SD writer task ---
static QueueHandle_t sdQueue = NULL;
const int SD_QUEUE_SIZE = 64;

// enqueue a log line to be written by the SD writer task
void sdEnqueueLog(const String &line) {
  if (!sdQueue) return;
  char *buf = (char*)malloc(line.length() + 1);
  if (!buf) return;
  strcpy(buf, line.c_str());
  if (xQueueSend(sdQueue, &buf, 0) != pdTRUE) {
    // queue full, drop oldest: receive one then send
    char *old; if (xQueueReceive(sdQueue, &old, 0) == pdTRUE) { free(old); }
    xQueueSend(sdQueue, &buf, 0);
  }
}

void sdWriterTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    char *buf = NULL;
    if (xQueueReceive(sdQueue, &buf, portMAX_DELAY) == pdTRUE) {
      if (buf) {
        // ensure logFile open
        if (!logFile) {
          // attempt to (re)open using nextTripPath()
          String fname = nextTripPath();
          logFile = SD.open(fname, FILE_WRITE);
          if (logFile) {
            logFile.println("time,lat,lon,kmph,mph,altm,altft,sats,rpm,map,tps,clt,iat,batt");
          }
        }
        if (logFile) {
          logFile.println((const char*)buf);
          logFile.flush();
        }
        free(buf);
      }
    }
  }
}

// Character based scrolling for 3 lines
const int OLED_CHARS_PER_LINE = 20;       // ~20 chars fit at textSize=1
int lineOffset[3] = {0, 0, 0};
unsigned long lineLastStep[3] = {0, 0, 0};
const unsigned long SCROLL_INTERVAL = 500;  // ms between scroll steps

// ------------------------------
// GPS
// ------------------------------
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
const int GPS_RX = 17;
const int GPS_TX = 16;

// ------------------------------
// SD
// ------------------------------
const int SD_CS = 5;
File logFile;
bool SDOK = false;

// ------------------------------
// CAN
// ------------------------------
const int CAN_CS = 26;
const int CAN_INT = 27;
MCP_CAN CAN_MICRO(CAN_CS);
bool CANOK = false;

unsigned long lastCanMsg = 0;
unsigned long canTimeout = 1500;
unsigned long canCount = 0;

// CAN decoded data
int rpm = 0;
int mapv = 0;
int tps = 0;
int clt = 0;
int iat = 0;
float batt = 0;

// ------------------------------
// Recent frames ring buffer
// ------------------------------
#define MAX_FRAME_LIST 96
struct FrameRecord { unsigned long id; char data[80]; unsigned long ts; };
FrameRecord framesBuf[MAX_FRAME_LIST];
int framesHead = 0; // next write index
unsigned long framesTotal = 0; // total pushed

void pushFrame(unsigned long id, const byte* buf, byte len) {
  int idx = framesHead % MAX_FRAME_LIST;
  framesBuf[idx].id = id;
  // format data as hex
  int p = 0;
  for (byte i = 0; i < len && p < (int)(sizeof(framesBuf[idx].data) - 4); ++i) {
    sprintf(&framesBuf[idx].data[p], "%02X ", buf[i]);
    p += 3;
  }
  if (p > 0 && p < (int)sizeof(framesBuf[idx].data)) framesBuf[idx].data[p-1] = '\0'; else framesBuf[idx].data[p] = '\0';
  framesBuf[idx].ts = getTimeSeconds();
  framesHead++;
  framesTotal++;
  // broadcast this frame to any websocket clients (non-blocking)
  broadcastFrame(framesBuf[idx]);
}

// ------------------------------
// WiFi / SPIFFS
// ------------------------------
WebServer server(80);
bool WIFIOK = false;
String wifiFile = "/wifi.json";
String adminTokenFile = "/admin.token";

// ------------------------------
// Security helpers
// ------------------------------
// derive simple XOR key from chip MAC
void deriveKey(uint8_t *key, size_t len) {
  uint64_t mac = ESP.getEfuseMac();
  for (size_t i = 0; i < len; ++i) key[i] = (mac >> ((i % 8) * 8)) & 0xFF;
}

String toHex(const uint8_t *data, size_t len) {
  String s;
  s.reserve(len * 2 + 1);
  const char hex[] = "0123456789ABCDEF";
  for (size_t i = 0; i < len; ++i) {
    s += hex[(data[i] >> 4) & 0x0F];
    s += hex[data[i] & 0x0F];
  }
  return s;
}

// convert hex string to bytes; returns length
size_t hexToBytes(const String &hex, uint8_t *out, size_t maxOut) {
  size_t hlen = hex.length();
  size_t cnt = 0;
  for (size_t i = 0; i + 1 < hlen && cnt < maxOut; i += 2) {
    char a = hex.charAt(i);
    char b = hex.charAt(i+1);
    auto val = [](char c)->int { if (c>='0' && c<='9') return c-'0'; if (c>='A' && c<='F') return c-'A'+10; if (c>='a'&&c<='f') return c-'a'+10; return 0; };
    out[cnt++] = (val(a) << 4) | val(b);
  }
  return cnt;
}

String encryptString(const String &plain) {
  if (plain.length() == 0) return String("");
  size_t n = plain.length();
  uint8_t key[8]; deriveKey(key, sizeof(key));
  uint8_t *buf = (uint8_t*)malloc(n);
  for (size_t i = 0; i < n; ++i) buf[i] = ((uint8_t)plain.charAt(i)) ^ key[i % 8];
  String h = toHex(buf, n);
  free(buf);
  return String("enc:") + h;
}

String decryptString(const String &cipher) {
  if (!cipher.startsWith("enc:")) return cipher; // already plain
  String hex = cipher.substring(4);
  size_t hlen = hex.length() / 2;
  uint8_t *buf = (uint8_t*)malloc(hlen);
  size_t got = hexToBytes(hex, buf, hlen);
  uint8_t key[8]; deriveKey(key, sizeof(key));
  String out;
  out.reserve(got+1);
  for (size_t i = 0; i < got; ++i) {
    char c = (char)(buf[i] ^ key[i % 8]);
    out += c;
  }
  free(buf);
  return out;
}

// admin token helpers
void ensureAdminTokenExists() {
  if (SPIFFS.exists(adminTokenFile)) return;
  // generate token using esp_random
  uint8_t t[16];
  for (int i = 0; i < 16; ++i) {
    uint32_t r = esp_random();
    t[i] = (r >> (8 * (i % 4))) & 0xFF;
  }
  String tok = toHex(t, 16);
  File f = SPIFFS.open(adminTokenFile, FILE_WRITE);
  if (f) { f.print(tok); f.close(); /* do not print token to serial to avoid exposure */ }
}

// helper: convert 3-letter month name to month number (1-12)
int monthFromName(const char *m) {
  const char *months[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  for (int i = 0; i < 12; ++i) {
    if (strncmp(m, months[i], 3) == 0) return i + 1;
  }
  return 1;
}

String readAdminToken() {
  if (!SPIFFS.exists(adminTokenFile)) return String("");
  File f = SPIFFS.open(adminTokenFile, FILE_READ);
  if (!f) return String("");
  String t; while (f.available()) t += (char)f.read(); f.close();
  return t;
}

bool checkAuth() {
  // check header X-Auth-Token or Authorization: Bearer or form arg token
  String provided = String("");
  if (server.hasHeader("X-Auth-Token")) provided = server.header("X-Auth-Token");
  else if (server.hasHeader("Authorization")) {
    String a = server.header("Authorization");
    if (a.startsWith("Bearer ")) provided = a.substring(7);
  }
  if (provided.length() == 0 && server.hasArg("token")) provided = server.arg("token");
  String real = readAdminToken();
  // If token matches stored token, OK
  if (real.length() > 0 && provided.equals(real)) return true;

  // Allow local LAN clients (same /24) to access without token so device is "boot ready"
  IPAddress rem = server.client().remoteIP();
  IPAddress loc = WiFi.localIP();
  // accept loopback or exact match
  // INADDR_ANY is not available in ESP32 Arduino; compare against 0.0.0.0 instead
  if (rem == IPAddress(0,0,0,0) || rem == loc) return true;
  // compare first 3 octets (/24)
  if (rem[0] == loc[0] && rem[1] == loc[1] && rem[2] == loc[2]) return true;

  // otherwise unauthorized
  return false;
}

// ------------------------------
// GPS state
// ------------------------------
unsigned long lastGpsFix = 0;
double fallbackSpeed = 0;
bool GPSOK = false;

// ------------------------------
// Trip logging
// ------------------------------
unsigned long lastWrite = 0;
unsigned long writeRate = 1000;
int tripNumber = 1;

// ------------------------------
// Time
// ------------------------------
bool timeValid = false;
unsigned long baseMillis = 0;
unsigned long baseSeconds = 0;

unsigned long getTimeSeconds() {
  if (gps.time.isValid() && gps.date.isValid()) {
    baseSeconds =
      gps.date.year()   * 31556926UL +
      gps.date.month()  * 2629743UL +
      gps.date.day()    * 86400UL +
      gps.time.hour()   * 3600UL +
      gps.time.minute() * 60UL +
      gps.time.second();

    baseMillis = millis();
    timeValid = true;
  }
  if (!timeValid) return millis() / 1000;
  return baseSeconds + (millis() - baseMillis) / 1000;
}

// ------------------------------
// Trip file helpers (SD)
// ------------------------------
String nextTripName() {
  // Deprecated: kept for compatibility but prefer nextTripPath
  return String("/trip000.csv");
}
// create logs directory and date folder, return next trip path
String nextTripPath() {
  // determine date folder
  time_t secs = 0;
  if (timeValid) {
    secs = (time_t)getTimeSeconds();
  } else {
    // fallback: use compile/build date (__DATE__) to avoid 1970-01-01
    // __DATE__ format: "Mmm dd yyyy" (e.g. "Dec  1 2025")
    const char *bd = __DATE__;
    char mon[4] = {0};
    int day = 1;
    int year = 1970;
    if (sscanf(bd, "%3s %d %d", mon, &day, &year) >= 3) {
      int m = monthFromName(mon); // 1-12
      struct tm bt;
      memset(&bt, 0, sizeof(bt));
      bt.tm_year = year - 1900;
      bt.tm_mon = m - 1;
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

  // ensure /logs and dateDir exist
  if (!SD.exists("/logs")) SD.mkdir("/logs");
  if (!SD.exists(dateDir)) SD.mkdir(dateDir);

  // find next available tripNNN.csv in dateDir
  for (int n = 1; n < 1000; ++n) {
    char fname[64];
    snprintf(fname, sizeof(fname), "%s/trip%03d.csv", dateDir, n);
    if (!SD.exists(String(fname))) return String(fname);
  }
  // fallback
  return String(dateDir) + String("/trip999.csv");
}

void startNewTrip() {
  String fname = nextTripPath();
  Serial.print("Opening trip file: ");
  Serial.println(fname);

  logFile = SD.open(fname, FILE_WRITE);
  if (logFile) {
    logFile.println("time,lat,lon,kmph,mph,altm,altft,sats,rpm,map,tps,clt,iat,batt");
    logFile.flush();
    Serial.println("Trip header written");
  } else {
    Serial.println("Failed to open trip file");
  }
}

// ------------------------------
// WiFi JSON (SPIFFS) helpers
// ------------------------------
void ensureWifiJsonExists() {
  if (!SPIFFS.exists(wifiFile)) {
    Serial.println("wifi.json not found, creating empty networks array");
    StaticJsonDocument<256> doc;
    JsonArray arr = doc.createNestedArray("networks");
    (void)arr;

    File f = SPIFFS.open(wifiFile, FILE_WRITE);
    if (!f) {
      Serial.println("Failed to create wifi.json");
      return;
    }
    serializeJson(doc, f);
    f.close();
  }
}

// recursively append files under a path into out (relative names without leading slash)
void appendFilesInDir(const String &path, String &out) {
  File dir = SD.open(path);
  if (!dir) return;
  File entry = dir.openNextFile();
  while (entry) {
    String name = String(entry.name());
    if (entry.isDirectory()) {
      // recurse into directory
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

bool otaAuthorized = false;


bool connectSavedNetworks() {
  if (!SPIFFS.exists(wifiFile)) {
    Serial.println("wifi.json missing, cannot connect to saved networks");
    return false;
  }

  File f = SPIFFS.open(wifiFile, FILE_READ);
  if (!f) {
    Serial.println("Failed to open wifi.json for read");
    return false;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    Serial.print("wifi.json parse error: ");
    Serial.println(err.c_str());
    return false;
  }

  JsonArray arr = doc["networks"].as<JsonArray>();
  if (arr.isNull()) {
    Serial.println("wifi.json has no networks array");
    return false;
  }

  for (JsonObject o : arr) {
    String ss = o["ssid"].as<String>();
    String pw = o["pass"].as<String>();
    pw = decryptString(pw);

    Serial.print("Trying WiFi: ");
    Serial.println(ss);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ss.c_str(), pw.c_str());
    unsigned long t = millis();

    while (millis() - t < 8000) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
          // try to start mDNS as logger.local; prefer the mDNS name as web URL
          if (MDNS.begin("logger")) {
            Serial.println("mDNS started: http://logger.local");
            webURL = "http://logger.local";
          } else {
            Serial.println("mDNS start failed");
            webURL = String("http://") + WiFi.localIP().toString();
          }
          return true;
      }
      delay(100);
    }

    Serial.println("WiFi connect timeout, trying next network (if any)");
  }

  return false;
}

void startAPMode() {
  Serial.println("Starting AP mode...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("LoggerSetup", "password");
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP mode, IP: ");
  Serial.println(ip);
  WIFIOK = false;
  webURL = String("http://") + ip.toString();
  if (OLED_INITED) {
    String msg = String("AP mode IP: ") + ip.toString();
    enqueueBootMessage(msg, 2000);
  }
}

// Note: fallback embedded pages removed to save SPIFFS/SPI flash space.
// Web UI is served exclusively from the SD card. SPIFFS is still used
// for small configuration files (wifi.json, admin token), but not for
// web assets.

// ------------------------------
// SD file serving
// ------------------------------
bool serveSDFile(const String &path) {
  // Serve web assets exclusively from the SD card. Do not use SPIFFS
  // to avoid wasting flash space with embedded pages.
  String p = path;
  if (p == "/") p = "/index.html";

  if (!SD.exists(p)) return false;

  File f = SD.open(p, FILE_READ);
  if (!f) return false;

  String contentType = "text/plain";
  if (p.endsWith(".html")) contentType = "text/html";
  else if (p.endsWith(".css")) contentType = "text/css";
  else if (p.endsWith(".js")) contentType = "application/javascript";
  else if (p.endsWith(".csv")) contentType = "text/csv";

  server.streamFile(f, contentType);
  f.close();
  return true;
}

// ------------------------------
// Web handlers
// ------------------------------
void handleRoot() {
  if (!serveSDFile("/index.html")) {
    server.send(404, "text/plain", "Not found");
  }
}

void handleGenericFile() {
  String path = server.uri();
  if (!serveSDFile(path)) {
    server.send(404, "text/plain", "Not found: " + path);
  }
}

void handleAddWiFi() {
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

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    server.send(500, "text/plain", "wifi.json parse error");
    return;
  }

  JsonArray arr = doc["networks"].as<JsonArray>();
  if (arr.isNull()) {
    arr = doc.createNestedArray("networks");
  }

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

void handleOTAUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    Serial.println("OTA: begin");
    // require auth (token) for OTA
    otaAuthorized = checkAuth();
    if (!otaAuthorized) {
      Serial.println("OTA: unauthorized upload attempt");
      return;
    }
    if (!Update.begin()) {
      Serial.println("OTA begin failed");
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (!otaAuthorized) return;
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      Serial.println("OTA write failed");
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (!otaAuthorized) { Serial.println("OTA aborted: unauthorized"); return; }
    if (Update.end(true)) {
      Serial.println("OTA success, rebooting...");
    } else {
      Serial.println("OTA end failed");
    }
  }
}

// ------------------------------
// OLED scrolling helpers
// ------------------------------
void drawScrollingLine(const String &text, int y, int lineIndex) {
  if (lineIndex < 0 || lineIndex > 2) return;

  int len = text.length();
  int window = OLED_CHARS_PER_LINE;

  // If it fits, no scroll needed
  if (len <= window) {
    lineOffset[lineIndex] = 0;
    display.setCursor(0, y);
    display.print(text);
    return;
  }

  unsigned long now = millis();
  if (now - lineLastStep[lineIndex] > SCROLL_INTERVAL) {
    lineOffset[lineIndex]++;
    if (lineOffset[lineIndex] > len) {
      lineOffset[lineIndex] = 0;
    }
    lineLastStep[lineIndex] = now;
  }

  int start = lineOffset[lineIndex];
  String view;

  if (start + window <= len) {
    view = text.substring(start, start + window);
  } else {
    int firstLen = len - start;
    int secondLen = window - firstLen;
    view = text.substring(start, len) + " " + text.substring(0, max(0, secondLen));
  }

  display.setCursor(0, y);
  display.print(view);
}

// ------------------------------
// OLED update
// ------------------------------
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // line 0: full status words
  String statusLine = "GPS:" + String(GPSOK ? "OK " : "NO ") +
                      "CAN:" + String(CANOK ? "OK " : "NO ") +
                      "SD:"  + String(SDOK  ? "OK " : "NO ") +
                      "WiFi:" + String(WIFIOK ? "OK" : "NO");
  drawScrollingLine(statusLine, 0, 0);

  // line 1: altitude + satellites (scrolls)
  double altm = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  double altf = altm * 3.28084;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  char buf1[64];
  snprintf(buf1, sizeof(buf1), "Alt:%.1fm %.0fft Sats:%d", altm, altf, sats);
  drawScrollingLine(String(buf1), 10, 1);

  // line 2: uptime + speed & rpm (scrolls)
  String up = "Uptime:" + formatUptime();
  double kmph = gps.speed.isValid() ? gps.speed.kmph() : fallbackSpeed;
  double mph = kmph * 0.621371;
  char buf2[96];
  snprintf(buf2, sizeof(buf2), "%s Speed:%.1fmph RPM:%d", up.c_str(), mph, rpm);
  drawScrollingLine(String(buf2), 20, 2);

  display.display();
}

// helper to broadcast a frame over WebSocket
void broadcastFrame(const FrameRecord &fr) {
  StaticJsonDocument<256> doc;
  char idbuf[16]; sprintf(idbuf, "0x%lX", fr.id);
  doc["id"] = idbuf;
  doc["data"] = fr.data;
  doc["ts"] = fr.ts;
  String out; serializeJson(doc, out);
  wsBroadcast(out);
}

String formatUptime() {
    unsigned long s = millis() / 1000;
    unsigned long m = s / 60;
    unsigned long h = m / 60;

    char buf[20];
    sprintf(buf, "%02uh:%02um:%02us", (unsigned)h, (unsigned)(m % 60), (unsigned)(s % 60));
    return String(buf);
}

void bootMessage(const String &msg) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(msg);
    display.display();
}

// ------------------------------
// SETUP
// ------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== ESP32 CAN/GPS Logger boot ===");

  // init scroll state
  for (int i = 0; i < 3; i++) {
    lineOffset[i] = 0;
    lineLastStep[i] = 0;
  }

  // OLED
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
  } else {
    OLED_INITED = true;
    Serial.println("OLED init OK");
    // small delay to allow the OLED controller to power up and present the first frame
    delay(200);
  }

  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS UART started");
  if (OLED_INITED) { enqueueBootMessage("GPS UART started", 1500); }

  // SPIFFS
  Serial.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(false)) {
    Serial.println("SPIFFS mount failed, trying format");
    if (SPIFFS.begin(true)) {
      Serial.println("SPIFFS formatted and mounted");
      if (OLED_INITED) { enqueueBootMessage("SPIFFS formatted and mounted", 1500); }
    } else {
      Serial.println("SPIFFS still failed");
    }
  } else {
    Serial.println("SPIFFS mount OK");
    if (OLED_INITED) { enqueueBootMessage("SPIFFS mount OK", 1000); }
  }
  ensureWifiJsonExists();
  ensureAdminTokenExists();

  // show startup messages on OLED with 5s pause each (only if OLED initialized)
  if (OLED_INITED) {
    enqueueBootMessage("=== ESP32 CAN/GPS Logger boot ===", 1500);
  }

  // SD
  Serial.println("Starting SD...");
  SPI.begin(18, 19, 23, SD_CS);
  SDOK = SD.begin(SD_CS, SPI);
  Serial.print("SD status: ");
  Serial.println(SDOK ? "OK" : "FAIL");
  if (OLED_INITED) { enqueueBootMessage(String("SD status: ") + (SDOK?"OK":"FAIL"), 1500); }
  if (SDOK) {
    startNewTrip();
  }

  // CAN
  Serial.println("Starting CAN...");
  CANOK = (CAN_MICRO.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK);
    if (CANOK) {
    CAN_MICRO.setMode(MCP_NORMAL);
    Serial.println("CAN init OK");
    if (OLED_INITED) { enqueueBootMessage("CAN init OK", 1200); }
  } else {
    Serial.println("CAN init FAIL");
    if (OLED_INITED) { enqueueBootMessage("CAN init FAIL", 1200); }
  }
  pinMode(CAN_INT, INPUT);

  // WiFi
  Serial.println("Connecting WiFi...");
  if (connectSavedNetworks()) {
    WIFIOK = true;
    if (OLED_INITED) { enqueueBootMessage("WiFi connected", 1200); }
  } else {
    startAPMode();
  }

  // Show the available web URL (mdns name or IP) so users can connect to the device
  if (OLED_INITED && webURL.length() > 0) {
    enqueueBootMessage(String("Open: ") + webURL, 2000);
  }

  // Web routes
  server.on("/", handleRoot);
  // support both GET (legacy) and POST (safer) for adding WiFi
  server.on("/addwifi", HTTP_GET, handleAddWiFi);
  server.on("/addwifi", HTTP_POST, handleAddWiFi);
  server.on("/update", HTTP_POST, []() {
    if (!checkAuth()) { server.send(401, "text/plain", "Unauthorized"); return; }
    server.send(200, "text/plain", "OTA done, rebooting");
    delay(500);
    ESP.restart();
  }, handleOTAUpload);

  // API endpoints for UI
  server.on("/status.json", HTTP_GET, [](){
    StaticJsonDocument<512> doc;
    doc["gps"] = GPSOK ? "OK" : "NO";
    doc["can"] = CANOK ? "OK" : "NO";
    doc["sd"]  = SDOK ? "OK" : "NO";
    doc["wifi"] = WIFIOK ? "OK" : "NO";
    double kmph = gps.speed.isValid() ? gps.speed.kmph() : fallbackSpeed;
    doc["mph"] = kmph * 0.621371;
    doc["lat"] = gps.location.isValid() ? gps.location.lat() : 0.0;
    doc["lon"] = gps.location.isValid() ? gps.location.lng() : 0.0;
    doc["sats"] = gps.satellites.isValid() ? gps.satellites.value() : 0;
    doc["frames"] = (unsigned long)canCount;
    doc["rpm"] = rpm;
    doc["batt"] = batt;
    String out; serializeJson(doc, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
  });

  server.on("/frames", HTTP_GET, [](){
    DynamicJsonDocument doc(4096);
    JsonArray arr = doc.to<JsonArray>();
    unsigned long total = min((unsigned long)MAX_FRAME_LIST, framesTotal);
    for (unsigned long i = 0; i < total; ++i) {
      int idx = (framesHead - 1 - i + MAX_FRAME_LIST) % MAX_FRAME_LIST;
      JsonObject o = arr.createNestedObject();
      char idbuf[16]; sprintf(idbuf, "0x%lX", framesBuf[idx].id);
      o["id"] = idbuf;
      o["data"] = framesBuf[idx].data;
      o["ts"] = framesBuf[idx].ts;
    }
    String out; serializeJson(doc, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
  });

  server.on("/frames.txt", HTTP_GET, [](){
    String out;
    unsigned long total = min((unsigned long)MAX_FRAME_LIST, framesTotal);
    for (unsigned long i = 0; i < total; ++i) {
      int idx = (framesHead - 1 - i + MAX_FRAME_LIST) % MAX_FRAME_LIST;
      out += String("0x") + String(framesBuf[idx].id, HEX) + "|" + String(framesBuf[idx].data) + "|" + String(framesBuf[idx].ts) + "\n";
    }
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", out);
  });

  server.on("/list", HTTP_GET, [](){
    // list files under /logs recursively
    String out;
    if (!SD.exists("/logs")) { server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(200, "text/plain", ""); return; }
    File root = SD.open("/logs");
    if (!root) { server.send(500, "text/plain", "SD open failed"); return; }
    // recursive traversal using helper
    appendFilesInDir(String("/logs"), out);
    root.close();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", out);
  });

  server.on("/download", HTTP_GET, [](){
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

  // WiFi management helpers
  server.on("/wifi/list", HTTP_GET, [](){
    if (!SPIFFS.exists(wifiFile)) { server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(200, "application/json", "{}" ); return; }
    File f = SPIFFS.open(wifiFile, FILE_READ);
    if (!f) { server.send(500, "text/plain", "open failed"); return; }
    StaticJsonDocument<512> doc; DeserializationError err = deserializeJson(doc, f); f.close();
    if (err) { server.send(500, "text/plain", "parse error"); return; }
    JsonArray arr = doc["networks"].as<JsonArray>();
    DynamicJsonDocument outDoc(512);
    JsonArray outArr = outDoc.to<JsonArray>();
    for (JsonObject o : arr) {
      JsonObject no = outArr.createNestedObject();
      no["ssid"] = String((const char*)o["ssid"]);
      no["pass"] = "****"; // mask
    }
    String out; serializeJson(outArr, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
  });

  server.on("/wifi/delete", HTTP_GET, [](){
    if (!checkAuth()) { server.send(401, "text/plain", "Unauthorized"); return; }
    if (!server.hasArg("ssid")) { server.send(400, "text/plain", "ssid required"); return; }
    String ss = server.arg("ssid");
    File f = SPIFFS.open(wifiFile, FILE_READ);
    if (!f) { server.send(500, "text/plain", "open failed"); return; }
    StaticJsonDocument<512> doc; DeserializationError err = deserializeJson(doc, f); f.close();
    if (err) { server.send(500, "text/plain", "parse error"); return; }
    JsonArray arr = doc["networks"].as<JsonArray>();
    for (int i = arr.size()-1; i >= 0; --i) { if (String((const char*)arr[i]["ssid"]) == ss) arr.remove(i); }
    File w = SPIFFS.open(wifiFile, FILE_WRITE); if (!w) { server.send(500, "text/plain", "write failed"); return; }
    serializeJson(doc, w); w.close();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{}" );
  });

  server.on("/wifi/connect", HTTP_GET, [](){
    if (!checkAuth()) { server.send(401, "text/plain", "Unauthorized"); return; }
    if (!server.hasArg("ssid")) { server.send(400, "text/plain", "ssid required"); return; }
    String ss = server.arg("ssid");
    File f = SPIFFS.open(wifiFile, FILE_READ);
    if (!f) { server.send(500, "text/plain", "open failed"); return; }
    StaticJsonDocument<512> doc; deserializeJson(doc, f); f.close();
    JsonArray arr = doc["networks"].as<JsonArray>();
    String pass="";
    for (JsonObject o : arr) { if (String((const char*)o["ssid"]) == ss) pass = String((const char*)o["pass"]); }
    if (pass.length()==0) { server.send(404, "text/plain", "not found"); return; }
    pass = decryptString(pass);
    WiFi.disconnect(true); delay(200);
    WiFi.begin(ss.c_str(), pass.c_str());
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"result\":\"connecting\"}");
  });

  server.on("/wifi/status", HTTP_GET, [](){
    StaticJsonDocument<256> doc;
    doc["connected"] = (WiFi.status() == WL_CONNECTED);
    doc["ssid"] = WiFi.SSID();
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    String out; serializeJson(doc, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
  });

  server.onNotFound(handleGenericFile);
  server.begin();
  Serial.println("HTTP server started");
  if (OLED_INITED) { enqueueBootMessage("HTTP server started", 1000); }

  // start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // create SD writer queue and task
  sdQueue = xQueueCreate(SD_QUEUE_SIZE, sizeof(char*));
  if (sdQueue) {
    xTaskCreate(sdWriterTask, "sdWriter", 4096, NULL, 1, NULL);
  }

  updateOLED();
}

// ------------------------------
// LOOP
// ------------------------------
void loop() {

  server.handleClient();

  // GPS ingest
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }
  GPSOK = gps.location.isValid();

  unsigned long now = millis();

  // speed fallback
  double kmph = gps.speed.isValid() ? gps.speed.kmph() : fallbackSpeed;
  if (gps.speed.isValid()) {
    fallbackSpeed = kmph;
    lastGpsFix = now;
  } else if (now - lastGpsFix > 1500) {
    fallbackSpeed *= 0.98;
  }

  // CAN read (Microsquirt 0x151 frame)
  if (!digitalRead(CAN_INT)) {
    unsigned long id;
    byte len;
    byte buf[8];
    CAN_MICRO.readMsgBuf(&id, &len, buf);

    if (id == 0x151 && len >= 7) {
      rpm  = (buf[1] << 8) | buf[0];
      mapv = (buf[3] << 8) | buf[2];
      tps  = buf[4];
      clt  = buf[5];
      iat  = buf[6];
      batt = buf[7] * 0.1f;

      lastCanMsg = now;
      canCount++;
      // push into recent frames buffer for web UI
      pushFrame(id, buf, len);
    }
  }

  CANOK = (now - lastCanMsg < canTimeout);

  // SD logging
  if (SDOK && (now - lastWrite) >= writeRate) {
    lastWrite = now;
    // enqueue log line for SD writer task (non-blocking)
    String line;
    line.reserve(256);
    line += String(getTimeSeconds()); line += ",";
    line += String(gps.location.lat(), 6); line += ",";
    line += String(gps.location.lng(), 6); line += ",";
    line += String(kmph, 2); line += ",";
    line += String(kmph * 0.621371, 2); line += ",";
    line += String(gps.altitude.meters(), 1); line += ",";
    line += String(gps.altitude.meters() * 3.28084, 1); line += ",";
    line += String(gps.satellites.value()); line += ",";
    line += String(rpm); line += ",";
    line += String(mapv); line += ",";
    line += String(tps); line += ",";
    line += String(clt); line += ",";
    line += String(iat); line += ",";
    line += String(batt, 1);
    sdEnqueueLog(line);
  }

  updateOLED();
}
