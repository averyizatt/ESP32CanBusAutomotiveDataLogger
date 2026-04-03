# ESP32 GPS + CAN Logger

Firmware for the original **ESP32 Dev Module** (38-pin) combining GPS track logging with CAN bus capture.

| File | Description |
|---|---|
| `esp32gps.cpp` | GPS + CAN + SD logger — headless, logs to SD card |
| `esp32webgps.cpp` | Web + GPS + CAN logger — also hosts a web UI (files served from SD) |
| `WebPage/` | Web UI assets for `esp32webgps` (copy to SD card root) |

## Libraries Required

- TinyGPSPlus
- MCP_CAN (`coryjfowler/MCP_CAN_lib`)
- Adafruit SSD1306 + GFX
- ArduinoJson
- WebSockets (`links2004/WebSockets`)
- SD (Arduino-ESP32 core)

## Pin Map (ESP32 Dev Module)

| Signal | GPIO |
|---|:---:|
| MCP2515 CS | 26 |
| MCP2515 INT | 27 |
| SPI SCK | 18 |
| SPI MOSI | 23 |
| SPI MISO | 19 |
| SD CS | 5 |
| GPS RX (ESP RX2) | 16 |
| GPS TX (ESP TX2) | 17 |
| I²C SDA | 21 |
| I²C SCL | 22 |

See [`docs/hardware.md`](../../docs/hardware.md) for full wiring diagrams.
