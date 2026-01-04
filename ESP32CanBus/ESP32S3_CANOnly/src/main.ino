// PlatformIO entrypoint.
//
// The actual firmware is kept in ../ESP32S3_CANOnly.ino so it's still a single
// Arduino sketch file for Arduino IDE users.
//
// We include the key headers here so PlatformIO's dependency finder (LDF)
// picks up the built-in framework libraries (notably SD.h) and external libs.

#include <Arduino.h>

#include <SPI.h>
#include <SD.h>
#include <SPIFFS.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoJson.h>

#include <mcp_can.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "../ESP32S3_CANOnly.ino"
