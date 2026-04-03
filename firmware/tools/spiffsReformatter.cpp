#include <SPIFFS.h>

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("FORMATTING SPIFFS...");
  if (SPIFFS.begin(true)) {   // force format
    Serial.println("SPIFFS READY.");
  } else {
    Serial.println("SPIFFS FAILED!");
  }
}

void loop() {}
