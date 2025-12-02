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

// ------------------------------
// OLED
// ------------------------------
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 32, &Wire, -1);

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
// WiFi / SPIFFS
// ------------------------------
WebServer server(80);
bool WIFIOK = false;
String wifiFile = "/wifi.json";

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
  while (true) {
    String name = "/trip";
    if (tripNumber < 10) name += "00";
    else if (tripNumber < 100) name += "0";
    name += String(tripNumber) + ".csv";

    if (!SD.exists(name)) return name;
    tripNumber++;
  }
}

void startNewTrip() {
  String fname = nextTripName();
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
        if (MDNS.begin("logger")) {
          Serial.println("mDNS started: http://logger.local");
        } else {
          Serial.println("mDNS start failed");
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
}

// ------------------------------
// HTML style + fallback pages
// ------------------------------
String neonStyle = R"(
<style>
body { background:#000; color:#0f0; font-family:monospace; padding:20px; }
h1 { color:#0f0; font-size:26px; }
a.btn, button {
  display:inline-block; padding:10px 16px; margin:8px 4px;
  background:#111; border:1px solid #0f0; border-radius:4px;
  color:#0f0; text-decoration:none; font-size:18px;
}
.box {
  border:1px solid #0f0; padding:12px; margin-bottom:16px;
}
</style>
)";

String wrapPage(const String &title, const String &body) {
  String s;
  s += "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  s += neonStyle;
  s += "<title>" + title + "</title></head><body>";
  s += "<h1>" + title + "</h1>";
  s += body;
  s += "</body></html>";
  return s;
}

String fallbackHome() {
  String out;
  out += "<div class='box'>";
  out += "GPS: " + String(GPSOK ? "OK" : "NO") + "<br>";
  out += "CAN: " + String(CANOK ? "OK" : "NO") + "<br>";
  out += "SD: "  + String(SDOK  ? "OK" : "NO") + "<br>";
  out += "WiFi: " + String(WIFIOK ? "OK" : "NO") + "<br>";
  out += "</div>";

  out += "<a class='btn' href='/gps.html'>GPS</a>";
  out += "<a class='btn' href='/can.html'>CAN</a>";
  out += "<a class='btn' href='/files.html'>Files</a>";
  out += "<a class='btn' href='/wifi.html'>WiFi</a>";
  out += "<a class='btn' href='/ota.html'>OTA</a>";

  return wrapPage("Logger", out);
}

// ------------------------------
// SD file serving
// ------------------------------
bool serveSDFile(const String &path) {
  if (!SD.exists(path)) {
    return false;
  }

  File f = SD.open(path, FILE_READ);
  if (!f) return false;

  String contentType = "text/plain";
  if (path.endsWith(".html")) contentType = "text/html";
  else if (path.endsWith(".css")) contentType = "text/css";
  else if (path.endsWith(".js")) contentType = "application/javascript";
  else if (path.endsWith(".csv")) contentType = "text/csv";

  server.streamFile(f, contentType);
  f.close();
  return true;
}

// ------------------------------
// Web handlers
// ------------------------------
void handleRoot() {
  if (!serveSDFile("/index.html")) {
    server.send(200, "text/html", fallbackHome());
  }
}

void handleGenericFile() {
  String path = server.uri();
  if (!serveSDFile(path)) {
    server.send(404, "text/plain", "Not found: " + path);
  }
}

void handleAddWiFi() {
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
  obj["pass"] = pw;

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
    if (!Update.begin()) {
      Serial.println("OTA begin failed");
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      Serial.println("OTA write failed");
    }
  } else if (up.status == UPLOAD_FILE_END) {
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

  // line 1: lat / lon
  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
  String latlon = "Lat:" + String(lat, 6) + " Lon:" + String(lon, 6);
  drawScrollingLine(latlon, 10, 1);

  // line 2: speed / rpm / sats
  double kmph = gps.speed.isValid() ? gps.speed.kmph() : fallbackSpeed;
  double mph = kmph * 0.621371;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  String info = "Speed:" + String(mph, 1) + "mph RPM:" + String(rpm) + " Sat:" + String(sats);
  drawScrollingLine(info, 20, 2);

  display.display();
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
    Serial.println("OLED init OK");
  }

  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS UART started");

  // SPIFFS
  Serial.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(false)) {
    Serial.println("SPIFFS mount failed, trying format");
    if (SPIFFS.begin(true)) {
      Serial.println("SPIFFS formatted and mounted");
    } else {
      Serial.println("SPIFFS still failed");
    }
  } else {
    Serial.println("SPIFFS mount OK");
  }
  ensureWifiJsonExists();

  // SD
  Serial.println("Starting SD...");
  SPI.begin(18, 19, 23, SD_CS);
  SDOK = SD.begin(SD_CS, SPI);
  Serial.print("SD status: ");
  Serial.println(SDOK ? "OK" : "FAIL");
  if (SDOK) {
    startNewTrip();
  }

  // CAN
  Serial.println("Starting CAN...");
  CANOK = (CAN_MICRO.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK);
  if (CANOK) {
    CAN_MICRO.setMode(MCP_NORMAL);
    Serial.println("CAN init OK");
  } else {
    Serial.println("CAN init FAIL");
  }
  pinMode(CAN_INT, INPUT);

  // WiFi
  Serial.println("Connecting WiFi...");
  if (connectSavedNetworks()) {
    WIFIOK = true;
  } else {
    startAPMode();
  }

  // Web routes
  server.on("/", handleRoot);
  server.on("/addwifi", handleAddWiFi);
  server.on("/update", HTTP_POST, []() {
    server.send(200, "text/plain", "OTA done, rebooting");
    delay(500);
    ESP.restart();
  }, handleOTAUpload);

  server.onNotFound(handleGenericFile);
  server.begin();
  Serial.println("HTTP server started");

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
    }
  }

  CANOK = (now - lastCanMsg < canTimeout);

  // SD logging
  if (SDOK && (now - lastWrite) >= writeRate) {
    lastWrite = now;

    logFile.print(getTimeSeconds()); logFile.print(",");
    logFile.print(gps.location.lat(), 6); logFile.print(",");
    logFile.print(gps.location.lng(), 6); logFile.print(",");
    logFile.print(kmph, 2); logFile.print(",");
    logFile.print(kmph * 0.621371, 2); logFile.print(",");
    logFile.print(gps.altitude.meters(), 1); logFile.print(",");
    logFile.print(gps.altitude.meters() * 3.28084, 1); logFile.print(",");
    logFile.print(gps.satellites.value()); logFile.print(",");
    logFile.print(rpm); logFile.print(",");
    logFile.print(mapv); logFile.print(",");
    logFile.print(tps); logFile.print(",");
    logFile.print(clt); logFile.print(",");
    logFile.print(iat); logFile.print(",");
    logFile.println(batt, 1);
    logFile.flush();
  }

  updateOLED();
}
