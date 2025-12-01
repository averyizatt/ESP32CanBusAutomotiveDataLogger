/*
  ESP32 CAN/GPS Logger
  Neon HUD UI (SD-hosted)
  SPIFFS wifi.json
  OLED 128x32 status strip
  CAN, GPS, SD logging
  Web dashboard with fallback internal pages
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
#include <ArduinoJson.h>
#include <Update.h>
#include <SPIFFS.h>

#define OLED_ADDR 0x3C

// ------------------------------
// OLED
// ------------------------------
Adafruit_SSD1306 display(128, 32, &Wire, -1);

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

// CAN data
int rpm = 0;
int mapv = 0;
int tps = 0;
int clt = 0;
int iat = 0;
float batt = 0;

// ------------------------------
// WIFI/SPIFFS
// ------------------------------
WebServer server(80);
bool WIFIOK = false;
String wifiFile = "/wifi.json";

// ------------------------------
// GPS handling
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
// TIME
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
  if (!timeValid) return millis()/1000;
  return baseSeconds + (millis() - baseMillis)/1000;
}

// ------------------------------
// FILE HELPERS
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
  logFile = SD.open(fname, FILE_WRITE);

  if (logFile) {
    logFile.println("time,lat,lon,kmph,mph,altm,altft,sats,rpm,map,tps,clt,iat,batt");
    logFile.flush();
  }
}

// ------------------------------
// WIFI JSON STORAGE
// ------------------------------
void loadNetworks() {
  if (!SPIFFS.exists(wifiFile)) {
    DynamicJsonDocument doc(256);
    doc["networks"] = JsonArray();
    File f = SPIFFS.open(wifiFile, "w");
    serializeJson(doc, f);
    f.close();
  }
}

bool connectSavedNetworks() {
  File f = SPIFFS.open(wifiFile, "r");
  if (!f) return false;

  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, f)) return false;
  f.close();

  JsonArray arr = doc["networks"].as<JsonArray>();

  for (JsonObject o : arr) {
    String ss = o["ssid"].as<String>();
    String pw = o["pass"].as<String>();

    WiFi.begin(ss.c_str(), pw.c_str());
    unsigned long t = millis();

    while (millis() - t < 7000) {
      if (WiFi.status() == WL_CONNECTED) return true;
      delay(100);
    }
  }
  return false;
}

// AP fallback
void startAPMode() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("LoggerSetup", "password");
  WIFIOK = false;
}

// ------------------------------
// HTML FALLBACK PAGES
// ------------------------------
String neonStyle = R"(
<style>
body { background:black;color:#0f0;font-family:monospace;padding:20px; }
h1 { color:#0f0;font-size:28px; }
button,a { padding:12px 18px;margin:10px 0;display:block;
  background:#111;border:1px solid #0f0;border-radius:6px;color:#0f0;
  text-decoration:none;font-size:20px;text-align:center; }
.box { border:1px solid #0f0;padding:15px;margin-bottom:20px; }
</style>
)";

String wrapPage(String title, String body) {
  return "<html><head>" + neonStyle + "</head><body><h1>" + title + "</h1>" +
         body + "</body></html>";
}

// fallback homepage
String pageHomeFallback() {
  String out;
  out += "<div class='box'>";
  out += "GPS " + String(GPSOK) + "<br>";
  out += "CAN " + String(CANOK) + "<br>";
  out += "SD " + String(SDOK) + "<br>";
  out += "WiFi " + String(WIFIOK) + "<br>";
  out += "</div>";
  out += "<a href='/gps'>GPS</a>";
  out += "<a href='/can'>CAN</a>";
  out += "<a href='/wifi'>WiFi</a>";
  out += "<a href='/files'>Files</a>";
  return wrapPage("Logger", out);
}

// ------------------------------
// SD SERVING
// ------------------------------
bool serveSDFile(String path) {
  if (!SD.exists(path)) return false;

  File f = SD.open(path, "r");
  if (!f) return false;

  String type = "text/plain";
  if (path.endsWith(".html")) type="text/html";
  if (path.endsWith(".css")) type="text/css";
  if (path.endsWith(".js")) type="application/javascript";

  server.streamFile(f, type);
  f.close();
  return true;
}

// ------------------------------
// MAIN WEB ROUTES
// ------------------------------
void handleRoot() {
  if (!serveSDFile("/index.html")) {
    server.send(200,"text/html",pageHomeFallback());
  }
}

void handleGenericFile() {
  String path = server.uri();
  if (!serveSDFile(path)) server.send(404,"text/plain","Not found");
}

void handleAddWiFi() {
  if (!server.hasArg("ssid") || !server.hasArg("pass")) {
    server.send(400,"text/plain","missing ssid/pass");
    return;
  }

  String ss = server.arg("ssid");
  String pw = server.arg("pass");

  if (!SPIFFS.exists("/wifi.json")) {
    DynamicJsonDocument d(256);
    d["networks"]=JsonArray();
    File f=SPIFFS.open("/wifi.json","w");
    serializeJson(d,f);
    f.close();
  }

  File f = SPIFFS.open("/wifi.json","r");
  DynamicJsonDocument doc(512);
  deserializeJson(doc,f);
  f.close();

  JsonArray arr = doc["networks"];
  JsonObject o = arr.createNestedObject();
  o["ssid"]=ss;
  o["pass"]=pw;

  File w=SPIFFS.open("/wifi.json","w");
  serializeJson(doc,w);
  w.close();

  server.sendHeader("Location","/wifi.html",true);
  server.send(302,"text/plain","");
}

// OTA upload
void handleOTAUpload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) Update.begin();
  else if (up.status == UPLOAD_FILE_WRITE) Update.write(up.buf, up.currentSize);
  else if (up.status == UPLOAD_FILE_END) Update.end(true);
}

// ------------------------------
// OLED DISPLAY
// ------------------------------
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.print("GPS ");
  display.print(GPSOK?"OK ":"NO ");
  display.print("CAN ");
  display.print(CANOK?"OK":"NO");

  display.setCursor(0,10);
  display.print("SD ");
  display.print(SDOK?"OK ":"NO ");
  display.print("WiFi ");
  display.print(WIFIOK?"OK":"NO");

  display.setCursor(0,20);
  display.print("LAT ");
  display.print(gps.location.lat(),4);

  display.setCursor(0,28);
  display.print("LON ");
  display.print(gps.location.lng(),4);

  display.display();
}

// ------------------------------
// SETUP
// ------------------------------
void setup() {
  Serial.begin(115200);
  delay(1200);

  Wire.begin(21,22);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // SPIFFS
  SPIFFS.begin(false);
  loadNetworks();

  // SD
  SPI.begin(18,19,23,SD_CS);
  SDOK = SD.begin(SD_CS,SPI);
  if (SDOK) startNewTrip();

  // CAN
  CANOK = (CAN_MICRO.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ)==CAN_OK);
  if (CANOK) CAN_MICRO.setMode(MCP_NORMAL);
  pinMode(CAN_INT,INPUT);

  // WiFi
  if (connectSavedNetworks()) {
    WIFIOK=true;
  } else {
    startAPMode();
  }

  // WEB ROUTES
  server.on("/", handleRoot);
  server.on("/addwifi", handleAddWiFi);
  server.on("/update", HTTP_POST, [](){
    server.send(200,"text/plain","OK");
    delay(500);
    ESP.restart();
  }, handleOTAUpload);

  // Generic SD files
  server.onNotFound(handleGenericFile);

  server.begin();
}

// ------------------------------
// LOOP
// ------------------------------
void loop() {

  server.handleClient();

  // GPS ingest
  while (SerialGPS.available()) gps.encode(SerialGPS.read());
  GPSOK = gps.location.isValid();

  unsigned long now=millis();

  // fallback speed
  double kmph = gps.speed.isValid()?gps.speed.kmph():fallbackSpeed;

  if (gps.speed.isValid()) {
    fallbackSpeed=kmph;
    lastGpsFix=now;
  } else if (now-lastGpsFix > 1500) {
    fallbackSpeed*=0.98;
  }

  // CAN read
  if (!digitalRead(CAN_INT)) {
    unsigned long id;
    byte len, buf[8];
    CAN_MICRO.readMsgBuf(&id,&len,buf);

    if (id==0x151) {
      rpm  = (buf[1]<<8)|buf[0];
      mapv = (buf[3]<<8)|buf[2];
      tps  = buf[4];
      clt  = buf[5];
      iat  = buf[6];
      batt = buf[7]*0.1;
      lastCanMsg=now;
      canCount++;
    }
  }

  CANOK = (now-lastCanMsg < canTimeout);

  // SD LOGGING
  if (SDOK && (now-lastWrite)>=writeRate) {
    lastWrite=now;
    logFile.print(getTimeSeconds()); logFile.print(",");
    logFile.print(gps.location.lat(),6); logFile.print(",");
    logFile.print(gps.location.lng(),6); logFile.print(",");
    logFile.print(kmph,2); logFile.print(",");
    logFile.print(kmph*0.621371,2); logFile.print(",");
    logFile.print(gps.altitude.meters(),1); logFile.print(",");
    logFile.print(gps.altitude.meters()*3.28084,1); logFile.print(",");
    logFile.print(gps.satellites.value()); logFile.print(",");
    logFile.print(rpm); logFile.print(",");
    logFile.print(mapv); logFile.print(",");
    logFile.print(tps); logFile.print(",");
    logFile.print(clt); logFile.print(",");
    logFile.print(iat); logFile.print(",");
    logFile.println(batt,1);
    logFile.flush();
  }

  updateOLED();
}
