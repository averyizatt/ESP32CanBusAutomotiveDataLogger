# ESP32S3 CAN-only variant

This folder is a **new** ESP32-S3 DevKit build of the project focused on:

- **CAN via MCP2515 + Web UI** (served from **SPIFFS**)
- **Optional SD logging** (SPI microSD module)
- **Optional ESP-NOW → CAN forwarding** (with a MAC allowlist)

It keeps the original code in `ESP32CanBus/` unchanged.

## Pin map (ESP32-S3)

- I2C SDA: **3**
- I2C SCL: **10**

- MCP2515 INT: **37**
- SPI SCK: **47**
- SPI MOSI (SI): **21**
- SPI MISO (SO): **36**
- MCP2515 CS: **38**

- SD CS (if enabled): **14**

These are the current defaults in `ESP32S3_CANOnly.ino` and can be changed there to match your wiring.

## Files

- Main sketch: `ESP32S3_CANOnly.ino`
- SPIFFS web assets: `data/` (upload to SPIFFS)
- ESP-NOW allowlist/security notes: `ESPNOW_SECURITY_AND_USAGE.txt`

## Uploading web assets

This build serves `/index.html`, `/can.html`, etc from SPIFFS.

- Arduino IDE: use an ESP32 "Sketch Data Upload" tool/plugin to upload the `data/` folder to SPIFFS.
- PlatformIO: use `pio run -t uploadfs`.

## PlatformIO (VS Code)

This folder includes a ready-to-use PlatformIO config:

- `platformio.ini`
- `src/main.cpp` (a tiny shim that includes `../ESP32S3_CANOnly.ino` so the sketch remains single-file for Arduino IDE)

Typical workflow in the PlatformIO extension:

- **Build**: PlatformIO: Build
- **Upload firmware**: PlatformIO: Upload
- **Upload SPIFFS (web UI)**: PlatformIO: Upload Filesystem Image

If you use the CLI instead of the VS Code buttons:

- `pio run`
- `pio run -t upload`
- `pio run -t uploadfs`

## Notes

- `/status.json` includes CAN/WiFi status plus SD + ESP-NOW counters (when enabled).
- CAN traffic is pushed to the UI via `/frames` (JSON) and `/frames.txt` (text fallback).

## SD logging

SD logging is enabled by default in this variant (`#define ENABLE_SD_LOGGING 1`). It uses an SPI microSD module and (by default) shares the SPI bus with the MCP2515 using separate chip-select pins.

Web endpoints:

- `GET /sd/list` (list log files)
- `GET /sd/download?path=/logs/...` (download a specific file)

If you run into SD init issues, double-check that your SD module is wired for **SPI mode** and that the SD CS pin matches the sketch.

## I2C display

This build optionally uses an I2C display on SDA/SCL to show quick debug stats (CAN/WiFi status, frame count, loop Hz, last frame ID).

- Enabled by default via `#define ENABLE_I2C_DISPLAY 1` in `ESP32S3_CANOnly.ino`.
- Assumes an SSD1306 I2C display at address `0x3C` (common 128x32).
- If you don’t have the Adafruit display libraries installed, set `ENABLE_I2C_DISPLAY` to `0`.

## Onboard RGB status LED

The sketch drives a WS2812-style RGB LED for status.

- Default pin is `RGB_BUILTIN` if your board defines it, otherwise GPIO48.
- Override by defining `STATUS_LED_PIN`.

## Which CAN library to use?

Depends on your hardware:

1) **If you’re using an MCP2515 over SPI (your current pinout suggests this):**
	- This sketch uses the `mcp_can` (MCP_CAN) library.
	- It’s widely used and works well for typical OBD2 and general CAN sniffing.

2) **If you can switch to a native CAN transceiver (no MCP2515):**
	- ESP32-S3 has built-in TWAI (CAN controller) and can be faster / lower latency.
	- You’d use the ESP-IDF TWAI driver (`driver/twai.h`) or an Arduino wrapper library.

If you tell me whether your MCP2515 oscillator is **8 MHz or 16 MHz** (and your bus speed), I can set the correct defaults.

## This build (your setup)

- MCP2515 oscillator: **8 MHz** (configured in `ESP32S3_CANOnly.ino`)
- Default CAN bitrate: **500 kbps** (change `CAN_BITRATE` if you need 250k/125k, etc)

## ESP-NOW → CAN forwarding

This sketch can receive ESP-NOW packets from other ESP32s and forward them onto CAN.

- Default security behavior is **deny by default**: if the allowlist is empty, packets are rejected.
- Configure peers via HTTP:
	- `GET /espnow/peers`
	- `POST /espnow/peers/add?mac=AA:BB:CC:DD:EE:FF&token=...`
	- `POST /espnow/peers/del?mac=AA:BB:CC:DD:EE:FF&token=...`

Packet format and troubleshooting counters are documented in `ESPNOW_SECURITY_AND_USAGE.txt`.

## Sending CAN frames

This sketch includes an authenticated endpoint for sending frames:

- `POST /can/send?id=0x123&data=11223344&ext=0&token=...`
- `data` can be `11223344` or `11 22 33 44` (max 8 bytes)
- `ext=1` to send 29-bit extended IDs

## Microsquirt CAN expansion (your spec)

Your Microsquirt remote table protocol (per your spec) is **write-only**: the ESP pushes packed input bytes into Microsquirt RAM.

Configured in the sketch:

- CAN bitrate: `500k`
- MCP2515 oscillator: `8 MHz`
- CAN ID: `5` (11-bit)
- Table number: `7`
- Offset: `75` (so bytes `75..77` carry the 3 port bytes)

Frame format sent by the ESP at 20–50 Hz:

- DLC: 8
- `[0]=7, [1]=0, [2]=75, [3]=P1, [4]=P2, [5]=P3, [6]=0, [7]=0`

Where:

- Byte 75 (P1) = Port 1 digital inputs bits 0–7
- Byte 76 (P2) = Port 2 digital inputs bits 0–7
- Byte 77 (P3) = Port 3 digital inputs bits 0–7

## GPIO to logical input mapping

Edit the mapping arrays in `ESP32S3_CANOnly.ino`:

- `PORT1_MAP[8]`
- `PORT2_MAP[8]`
- `PORT3_MAP[8]`

Each entry is `{gpio, activeLow}`.

- Use `gpio=-1` for unused bits.
- If `activeLow=true`, the pin is configured as `INPUT_PULLUP` and the bit becomes 1 when the switch pulls to GND.
- If `activeLow=false`, the pin is configured as `INPUT_PULLDOWN` and the bit becomes 1 when the pin is driven high.
