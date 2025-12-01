#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

const int GPS_RX = 17;
const int GPS_TX = 16;

const int SD_CS = 5;
const int LED_PIN = 14;

const int CAN_CS = 26;
const int CAN_INT = 27;

MCP_CAN CAN_MICRO(CAN_CS);

Adafruit_SSD1306 display(128, 32, &Wire, -1);

File logFile;

int tripNumber = 1;

bool timeValid = false;
unsigned long baseMillis = 0;
unsigned long baseSeconds = 0;

unsigned long getTimeSeconds() {
  if (gps.time.isValid() && gps.date.isValid()) {
    baseSeconds =
      gps.date.year() * 31556926UL +
      gps.date.month() * 2629743UL +
      gps.date.day() * 86400UL +
      gps.time.hour() * 3600UL +
      gps.time.minute() * 60UL +
      gps.time.second();

    baseMillis = millis();
    timeValid = true;
  }
  if (!timeValid) return millis() / 1000;
  return baseSeconds + ((millis() - baseMillis) / 1000);
}

double fallbackSpeed = 0;
unsigned long lastGpsFix = 0;

unsigned long lastWrite = 0;
unsigned long writeRate = 1000;

unsigned long lastCanMsg = 0;
unsigned long canTimeout = 1500;
unsigned long canCount = 0;

double IATLimit = 50;
double CLTLimit = 105;

int rpm = 0;
int mapv = 0;
int tps = 0;
int clt = 0;
int iat = 0;
float batt = 0;

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
  String filename = nextTripName();
  logFile = SD.open(filename, FILE_WRITE);
  if (logFile) {
    logFile.println("time,lat,lon,kmph,mph,altm,altft,sats,rpm,map,tps,clt,iat,batt");
    logFile.flush();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(21, 22);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  SPI.begin(18, 19, 23, SD_CS);

  bool sdOK = SD.begin(SD_CS, SPI);
  if (sdOK) startNewTrip();

  bool canOK = false;
  if (CAN_MICRO.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    CAN_MICRO.setMode(MCP_NORMAL);
    canOK = true;
  }

  pinMode(CAN_INT, INPUT);
}

void updateOLED(bool gpsOK, bool sdOK, bool canOK, double lat, double lon, double mph, int rpm, bool warnIAT, bool warnCLT) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("GPS:");
  display.print(gpsOK ? "OK  " : "NO  ");
  display.print("CAN:");
  display.print(canOK ? "OK  " : "NO  ");
  display.print("SD:");
  display.print(sdOK ? "OK" : "NO");

  display.setCursor(0, 10);
  display.print(lat, 4);
  display.print(" ");
  display.print(lon, 4);

  display.setCursor(0, 20);
  display.print("M ");
  display.print(mph, 1);
  display.print("  R ");
  display.print(rpm);

  display.setCursor(0, 28);
  if (warnIAT) display.print("IAT HOT ");
  if (warnCLT) display.print("CLT HOT");

  display.display();
}

void loop() {

  while (SerialGPS.available()) gps.encode(SerialGPS.read());

  unsigned long now = millis();

  bool gpsOK = gps.location.isValid();
  bool sdOK = logFile;
  bool canOK = (now - lastCanMsg < canTimeout);

  double lat = gpsOK ? gps.location.lat() : 0;
  double lon = gpsOK ? gps.location.lng() : 0;
  double kmph = gps.speed.isValid() ? gps.speed.kmph() : fallbackSpeed;
  double mph = kmph * 0.621371;
  double altm = gps.altitude.isValid() ? gps.altitude.meters() : 0;
  double altft = altm * 3.28084;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  if (gps.speed.isValid()) {
    fallbackSpeed = kmph;
    lastGpsFix = now;
  } else {
    if (now - lastGpsFix > 1500 && fallbackSpeed > 0.1) {
      fallbackSpeed *= 0.98;
      kmph = fallbackSpeed;
      mph = kmph * 0.621371;
    }
  }

  if (!digitalRead(CAN_INT)) {
    unsigned long canId;
    byte len;
    byte buf[8];
    CAN_MICRO.readMsgBuf(&canId, &len, buf);

    if (canId == 0x151) {
      rpm = (buf[1] << 8) | buf[0];
      mapv = (buf[3] << 8) | buf[2];
      tps = buf[4];
      clt = buf[5];
      iat = buf[6];
      batt = buf[7] * 0.1;
      lastCanMsg = now;
      canCount++;
    }
  }

  unsigned int vss = (unsigned int)(mph * 100);
  byte vssdata[2];
  vssdata[0] = vss & 0xFF;
  vssdata[1] = vss >> 8;
  CAN_MICRO.sendMsgBuf(0x301, 0, 2, vssdata);

  if (now - lastWrite >= writeRate && logFile) {
    lastWrite = now;
    digitalWrite(LED_PIN, HIGH);

    logFile.print(getTimeSeconds()); logFile.print(",");
    logFile.print(lat, 6); logFile.print(",");
    logFile.print(lon, 6); logFile.print(",");
    logFile.print(kmph, 2); logFile.print(",");
    logFile.print(mph, 2); logFile.print(",");
    logFile.print(altm, 1); logFile.print(",");
    logFile.print(altft, 1); logFile.print(",");
    logFile.print(sats); logFile.print(",");
    logFile.print(rpm); logFile.print(",");
    logFile.print(mapv); logFile.print(",");
    logFile.print(tps); logFile.print(",");
    logFile.print(clt); logFile.print(",");
    logFile.print(iat); logFile.print(",");
    logFile.println(batt, 1);

    logFile.flush();
    digitalWrite(LED_PIN, LOW);
  }

  bool warnIAT = iat > IATLimit;
  bool warnCLT = clt > CLTLimit;

  updateOLED(gpsOK, sdOK, canOK, lat, lon, mph, rpm, warnIAT, warnCLT);

  delay(5);
}
