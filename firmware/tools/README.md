# Tools & Utilities

Standalone utility sketches for hardware testing and recovery.

| File | Purpose |
|---|---|
| `esp32CanBusHardwareTest.cpp` | Basic MCP2515 SPI ping test — confirms CAN hardware is wired correctly |
| `arduinoR3slaveTest.cpp` | Arduino R3 I²C slave test harness |
| `CrashFlasherTool.cpp` | Emergency flash-erase tool for recovering a bricked device |
| `spiffsReformatter.cpp` | Reformats the SPIFFS partition (use when SPIFFS is corrupted) |

## Usage

Each file is a self-contained Arduino sketch. Open the desired `.cpp` file in the Arduino IDE, rename the extension to `.ino` if required, select your board, and upload.

> **⚠️ Warning:** `CrashFlasherTool.cpp` and `spiffsReformatter.cpp` will **erase flash/SPIFFS** — do not upload them to a device with data you want to keep.
