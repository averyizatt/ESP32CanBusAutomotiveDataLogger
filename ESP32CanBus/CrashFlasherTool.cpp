// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("Erasing flash...");
//   for (int i = 0; i < 0x400000; i += 4096) {
//     ESP.flashEraseSector(i / 4096);
//   }
//   Serial.println("Done. Reboot and upload your real sketch.");
// }

// void loop() {}





// #include <Arduino.h>
// #include "esp_partition.h"

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   Serial.println("Erasing flash safely...");
//     Serial.printf("Flash size: %d bytes\n", ESP.getFlashChipSize());


//   const esp_partition_t* part = esp_partition_find_first(
//       ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);

//   if (!part) {
//     Serial.println("Could not find flash partition");
//     return;
//   }

//   esp_err_t result = esp_partition_erase_range(part, 0, 0x3F0000);  

//   if (result == ESP_OK) {
//     Serial.println("Flash erase complete");
//   } else {
//     Serial.printf("Flash erase failed: %d\n", result);
//   }
// }

// void loop() {}




#include <Arduino.h>
#include "esp_spi_flash.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Erasing all flash NOW...");

  // erase entire 4MB flash in blocks of 64KB
  const size_t FLASH_SIZE = 4 * 1024 * 1024;
  const size_t BLOCK = 64 * 1024;

  for (size_t offset = 0; offset < FLASH_SIZE; offset += BLOCK) {
    Serial.printf("Erasing block at 0x%06X...\n", offset);
    esp_err_t err = spi_flash_erase_range(offset, BLOCK);
    if (err != ESP_OK) {
      Serial.printf("Erase error at 0x%06X : %d\n", offset, err);
      return;
    }
  }

  Serial.println("FLASH ERASE COMPLETE!");
}

void loop() {}
