# ESP32 CAN Bus Automotive Data Logger

[![Platform](https://img.shields.io/badge/platform-ESP32%20%7C%20ESP32--S3-blue?logo=espressif)](https://www.espressif.com/)
[![Framework](https://img.shields.io/badge/framework-Arduino%20%7C%20PlatformIO-orange?logo=arduino)](https://platformio.org/)
[![CAN](https://img.shields.io/badge/CAN-OBD--II%20%7C%20HS%20500k%20%7C%20MS%20125k-green)](https://en.wikipedia.org/wiki/OBD-II_PIDs)
[![License](https://img.shields.io/badge/license-MIT-informational)](LICENSE)

> A full-featured open-source automotive CAN bus data logger and real-time monitor built on the ESP32 / ESP32-S3, featuring a live web dashboard, OBD-II PID queries, GPS tracking, SD card logging, ESP-NOW wireless CAN forwarding, and a serial CAN commander tool.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Hardware](#hardware)
  - [Bill of Materials](#bill-of-materials)
  - [Pin Mapping (ESP32-S3 variant)](#pin-mapping-esp32-s3-variant)
- [Getting Started](#getting-started)
  - [Arduino IDE](#arduino-ide)
  - [PlatformIO](#platformio)
- [Firmware Variants](#firmware-variants)
- [Web UI & API Reference](#web-ui--api-reference)
- [Serial CAN Commander](#serial-can-commander)
- [OBD-II PID Support](#obd-ii-pid-support)
- [ESP-NOW Wireless Forwarding](#esp-now-wireless-forwarding)
- [SD Card Logging](#sd-card-logging)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project turns an ESP32 or ESP32-S3 development board into a powerful automotive CAN bus tool. It connects to your vehicle's CAN bus via an MCP2515 SPI CAN controller and exposes a clean web interface over Wi-Fi so you can monitor live traffic, query OBD-II parameters, and download logged trip files — all from a phone or laptop without any additional software.

It was originally developed for a **1989 Ford Mustang Foxbody** (retrofitted with modern CAN-connected accessories), but the firmware is generic and works with any vehicle that exposes a standard CAN or OBD-II bus.

---

## Features

| Feature | ESP32-S3 Variant | GPS Logger Variant |
|---|:---:|:---:|
| Live CAN frame viewer (WebSocket) | ✅ | ✅ |
| OBD-II PID queries (Mode 01) | ✅ | ✅ |
| Web dashboard (SPIFFS / SD) | ✅ | ✅ |
| SD card trip logging | ✅ | ✅ |
| GPS track logging | — | ✅ |
| OLED status display | ✅ | ✅ |
| RGB status LED | ✅ | — |
| OTA firmware update | ✅ | ✅ |
| mDNS (`http://logger.local`) | ✅ | ✅ |
| ESP-NOW → CAN forwarding | ✅ | — |
| Serial CAN commander tool | ✅ | — |
| Authenticated admin API | ✅ | ✅ |
| Safe-mode / watchdog recovery | ✅ | — |

---

## Project Structure

```
ESP32CanBusAutomotiveDataLogger/
├── README.md                          ← You are here
├── .gitignore
├── docs/
│   ├── hardware.md                    ← Wiring diagrams and pinout tables
│   └── obd2-pids.md                   ← Supported OBD-II PID reference
└── firmware/
    ├── esp32s3-can-logger/            ← ★ Main production firmware (ESP32-S3)
    │   ├── ESP32S3_CANOnly.ino        ← Single-file Arduino sketch
    │   ├── platformio.ini             ← PlatformIO project config
    │   ├── src/main.ino               ← PlatformIO entry shim
    │   ├── data/                      ← SPIFFS web UI assets
    │   │   ├── index.html
    │   │   ├── can.html
    │   │   ├── settings.html
    │   │   ├── ota.html
    │   │   ├── wifi.html
    │   │   ├── script.js
    │   │   └── style.css
    │   ├── ESPNOW_SECURITY_AND_USAGE.txt
    │   └── README.md
    ├── esp32-gps-logger/              ← GPS + CAN logger (original ESP32)
    │   ├── esp32gps.cpp               ← GPS + CAN + SD logger
    │   ├── esp32webgps.cpp            ← Web + GPS + CAN logger
    │   └── WebPage/                   ← Web UI for this variant
    ├── can-commander/                 ← Reusable serial CAN commander module
    │   ├── CanCommanderModule.h
    │   └── CanCommanderModule.cpp
    └── tools/                         ← Utility and test sketches
        ├── esp32CanBusHardwareTest.cpp
        ├── arduinoR3slaveTest.cpp
        ├── CrashFlasherTool.cpp
        └── spiffsReformatter.cpp
```

---

## Hardware

### Bill of Materials

| Component | Notes |
|---|---|
| **ESP32-S3 DevKitC-1** | Main MCU (ESP32 Dev Module works for the GPS variant) |
| **MCP2515 CAN module** | SPI CAN controller + TJA1050 transceiver. Use an 8 MHz crystal version. |
| **OBD-II breakout / connector** | Connect to vehicle CAN-H / CAN-L pins (typically pins 6 & 14) |
| **microSD module** (optional) | SPI microSD; shares the bus with MCP2515 via separate CS |
| **SSD1306 OLED display** (optional) | 128×32 I²C at address `0x3C` |
| **GPS module** (optional, GPS variant only) | Any UART NMEA GPS (e.g. u-blox NEO-6M) |
| **WS2812 RGB LED** (optional) | Built-in on most S3 dev boards (`RGB_BUILTIN`) |
| Automotive-rated fuse + wire | Tap 12 V from the OBD-II port (pin 16) |

> **⚠️ Important:** The MCP2515 transceiver operates at **5 V** logic on some breakout boards. Use a 3.3 V-compatible module or add a level shifter to protect the ESP32.

### Pin Mapping (ESP32-S3 variant)

| Signal | GPIO | Notes |
|---|:---:|---|
| SPI SCK | **47** | Shared: MCP2515 + SD |
| SPI MOSI | **21** | Shared: MCP2515 + SD |
| SPI MISO | **36** | Shared: MCP2515 + SD |
| MCP2515 CS | **38** | |
| MCP2515 INT | **37** | Active-low interrupt |
| SD CS | **14** | Optional; set `ENABLE_SD_LOGGING` |
| I²C SDA | **3** | OLED display |
| I²C SCL | **10** | OLED display |
| RGB LED | `RGB_BUILTIN` | Falls back to GPIO 48 |

> All pins are configurable at the top of `ESP32S3_CANOnly.ino`.

See [`docs/hardware.md`](docs/hardware.md) for full wiring diagrams and OBD-II connector pinout.

---

## Getting Started

### Prerequisites

- Arduino IDE 2.x **or** VS Code + PlatformIO extension
- ESP32 Arduino core (`espressif/arduino-esp32` ≥ 3.x) installed in Board Manager
- Required libraries (auto-installed by PlatformIO; manual install for Arduino IDE):

  | Library | Source |
  |---|---|
  | MCP_CAN | [coryjfowler/MCP_CAN_lib](https://github.com/coryjfowler/MCP_CAN_lib) |
  | ArduinoJson | `bblanchon/ArduinoJson` ≥ 7.x |
  | WebSockets | `links2004/WebSockets` ≥ 2.4 |
  | Adafruit SSD1306 | `adafruit/Adafruit SSD1306` |
  | Adafruit GFX | `adafruit/Adafruit GFX Library` |
  | TinyGPSPlus | GPS variant only |

### Arduino IDE

1. Open `firmware/esp32s3-can-logger/ESP32S3_CANOnly.ino` in the Arduino IDE.
2. Select **Board → ESP32S3 Dev Module** and set the partition scheme to *Default 4 MB with SPIFFS*.
3. Install the libraries listed above via **Tools → Manage Libraries**.
4. Set your Wi-Fi credentials and admin token near the top of the sketch.
5. Upload the sketch.
6. Upload SPIFFS web assets using the [ESP32 Sketch Data Upload plugin](https://github.com/lorol/LITTLEFS).

### PlatformIO

```bash
# 1. Open the project folder
cd firmware/esp32s3-can-logger

# 2. Build firmware
pio run

# 3. Upload firmware
pio run -t upload

# 4. Upload SPIFFS web UI
pio run -t uploadfs

# 5. Open serial monitor
pio device monitor
```

Alternatively, use the PlatformIO sidebar in VS Code: **Build → Upload → Upload Filesystem Image**.

---

## Firmware Variants

### `firmware/esp32s3-can-logger/` ← Recommended

The flagship build targeting the **ESP32-S3 DevKitC-1**. Includes the full web UI served from SPIFFS, SD card logging, ESP-NOW CAN forwarding, OLED display, RGB status LED, watchdog/safe-mode recovery, and an authenticated REST + WebSocket API.

### `firmware/esp32-gps-logger/`

The original **ESP32 Dev Module** build with GPS support. Uses an SD card to host web files and log GPS + CAN tracks simultaneously. Suitable for classic-car retrofits where GPS track logging is the primary requirement.

### `firmware/can-commander/`

A reusable Arduino module (`CanCommanderModule.h/.cpp`) providing an interactive serial CAN tool. Drop it into any sketch that already has an MCP2515 initialized. See [Serial CAN Commander](#serial-can-commander).

### `firmware/tools/`

Utility sketches:
- **`esp32CanBusHardwareTest.cpp`** — Basic ping test; confirms the MCP2515 is wired correctly.
- **`arudinoR3slaveTest.cpp`** — Arduino R3 I²C slave test harness.
- **`CrashFlasherTool.cpp`** — Emergency flash-erase tool for recovering a bricked device.
- **`spiffsReformatter.cpp`** — Reformats the SPIFFS partition.

---

## Web UI & API Reference

Once flashed and connected to Wi-Fi, browse to `http://logger.local` (or the device IP shown in the serial monitor).

### Pages

| URL | Description |
|---|---|
| `/` or `/index.html` | Live dashboard (CAN rate, GPS, speed gauge) |
| `/can.html` | Live CAN frame viewer with WebSocket updates |
| `/settings.html` | Admin settings (token, CAN bitrate, etc.) |
| `/wifi.html` | Wi-Fi configuration |
| `/ota.html` | Over-the-air firmware update |

### REST Endpoints

| Method | Endpoint | Description |
|---|---|---|
| GET | `/status.json` | System status (CAN, Wi-Fi, GPS, SD, ESP-NOW counters) |
| GET | `/frames` | Latest CAN frames as JSON array |
| GET | `/frames.txt` | Latest CAN frames as plain text |
| GET | `/sd/list` | List SD log files |
| GET | `/sd/download?path=/logs/…` | Download a log file |
| POST | `/can/send?id=0x7DF&data=0201050000&token=…` | Send a CAN frame |
| POST | `/espnow/peers/add?mac=AA:BB:CC:DD:EE:FF&token=…` | Add ESP-NOW peer |
| POST | `/espnow/peers/del?mac=AA:BB:CC:DD:EE:FF&token=…` | Remove ESP-NOW peer |
| GET | `/espnow/peers` | List current ESP-NOW allowlist |
| POST | `/update` | OTA firmware upload (multipart/form-data) |

### WebSocket

Connect to `ws://<device-ip>/ws` to receive live CAN frames as JSON objects:

```json
{ "id": "0x201", "dlc": 8, "data": "A1 B2 C3 D4 E5 F6 00 00", "ts": 1712102400 }
```

---

## Serial CAN Commander

The `can-commander` module provides an interactive serial interface for CAN analysis. Include it in your sketch and call `canCommanderSetup()` / `canCommanderLoop()` from `setup()` / `loop()`.

Open the serial monitor at **115200 baud** and select a mode:

| Mode | Key | Description |
|---|:---:|---|
| Read all traffic | `1` | Print every CAN frame; toggle ASCII with `a` |
| Write a frame | `2` | Enter ID / DLC / data interactively, then set send rate |
| Speed test | `3` | Count frames/second |
| Read filtered traffic | `4` | Set a hardware mask + filter before reading |
| Value tracker | `5` | Watch a filtered ID and report byte-level changes with min/max |
| Diagnostics / PID manager | `6` | Send OBD-II PID requests and decode responses |

Press `s` in any mode to stop and reset the device.

---

## OBD-II PID Support

The PID manager (`Mode 6` in the serial commander, and the `/can/send` endpoint) supports OBD-II **Service 01** (current data) PIDs. Decoded values are printed to serial and can be read via the WebSocket stream.

| PID | Parameter | Unit |
|---|---|---|
| `0x04` | Engine load | % |
| `0x05` | Coolant temperature | °C |
| `0x0A` | Fuel pressure | kPa |
| `0x0C` | Engine RPM | RPM |
| `0x0D` | Vehicle speed | km/h |
| `0x0F` | Intake air temperature | °C |
| `0x11` | Throttle position | % |
| `0x1F` | Engine run time | s |
| `0x2F` | Fuel level | % |
| `0x31` | Distance since codes cleared | km |
| `0x33` | Barometric pressure | kPa |
| `0x42` | Control module voltage | V |
| `0x46` | Ambient air temperature | °C |
| `0x5C` | Engine oil temperature | °C |
| `0x5E` | Engine fuel rate | L/h |
| `0xA6` | Odometer | km |

See [`docs/obd2-pids.md`](docs/obd2-pids.md) for the full formula reference.

---

## ESP-NOW Wireless Forwarding

The ESP32-S3 variant can receive **ESP-NOW** packets from sensor nodes and forward them onto the CAN bus. This allows remote GPIO inputs (switches, sensors) to inject CAN frames wirelessly without extra wiring.

**Security model:** MAC-address allowlist; empty list = deny all (default). Peers are managed via the HTTP API and persisted in SPIFFS.

See `firmware/esp32s3-can-logger/ESPNOW_SECURITY_AND_USAGE.txt` for the full packet format and troubleshooting guide.

---

## SD Card Logging

When SD logging is enabled (`#define ENABLE_SD_LOGGING 1`), the device creates a new trip file on each boot at `/logs/trip_NNNN.csv`. Each row contains:

```
timestamp_ms,can_id,dlc,b0,b1,b2,b3,b4,b5,b6,b7
```

Files can be downloaded via the `/sd/download` endpoint or listed with `/sd/list`.

---

## Contributing

Contributions are welcome! To get started:

1. Fork this repository and create a feature branch.
2. Make your changes inside the relevant `firmware/` sub-folder.
3. Test on hardware before submitting a PR.
4. Open a pull request with a clear description of what was changed and why.

Please keep individual sketches self-contained and avoid adding external dependencies unless they are widely available in the Arduino Library Manager or as a PlatformIO registry package.

---

## License

This project is released under the [MIT License](LICENSE). Use it freely in your own builds; attribution is appreciated but not required.
