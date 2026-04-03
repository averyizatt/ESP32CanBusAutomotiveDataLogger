# Hardware Guide — ESP32 CAN Bus Automotive Data Logger

This document covers wiring, connector pinouts, and hardware notes for both firmware variants.

---

## OBD-II Connector Pinout

The standard OBD-II (J1962) port is a 16-pin trapezoid connector located under the dashboard on the driver's side. The relevant pins for CAN bus access are:

```
OBD-II J1962 Female (vehicle side — face-on view)
┌──────────────────────────────────────────────┐
│  1   2   3   4   5   6   7   8              │
│                                              │
│  9  10  11  12  13  14  15  16              │
└──────────────────────────────────────────────┘
```

| Pin | Signal | Notes |
|:---:|---|---|
| 4 | Chassis Ground | Connect to GND |
| 5 | Signal Ground | Connect to GND |
| 6 | CAN-H (HS-CAN) | High-speed CAN bus high (+) |
| 14 | CAN-L (HS-CAN) | High-speed CAN bus low (−) |
| 16 | Battery + (12 V) | Fused power supply; always on |

> **Note:** Some vehicles use a medium-speed CAN bus on pins 3 / 11 at 125 kbps. If you see no traffic at 500 kbps, try those pins at 125 kbps (`CAN_BITRATE 125000`).

---

## ESP32-S3 Variant Wiring

### MCP2515 CAN Module

```
ESP32-S3             MCP2515 Module
────────────────     ──────────────
3.3 V        ──────► VCC  (use a 3.3 V module, or add a level shifter)
GND          ──────► GND
GPIO 47 SCK  ──────► SCK
GPIO 21 MOSI ──────► SI   (MOSI)
GPIO 36 MISO ◄──────  SO  (MISO)
GPIO 38      ──────► CS
GPIO 37      ◄──────  INT (active-low interrupt)
```

CAN transceiver output → OBD-II connector:
```
MCP2515 CANH ──────► OBD-II pin 6  (CAN-H)
MCP2515 CANL ──────► OBD-II pin 14 (CAN-L)
GND          ──────► OBD-II pins 4 & 5
```

### microSD Card Module (optional)

Shares the SPI bus with the MCP2515 — only the CS pin differs:

```
ESP32-S3             microSD Module
────────────────     ──────────────
3.3 V        ──────► VCC  (or 5 V if your module has a regulator)
GND          ──────► GND
GPIO 47 SCK  ──────► CLK
GPIO 21 MOSI ──────► MOSI (CMD)
GPIO 36 MISO ◄──────  MISO (DAT0)
GPIO 14      ──────► CS
```

### SSD1306 OLED Display (optional, 128×32)

```
ESP32-S3             SSD1306 OLED
────────────────     ────────────
3.3 V        ──────► VCC
GND          ──────► GND
GPIO 3 SDA   ──────► SDA
GPIO 10 SCL  ──────► SCL
```

The display I²C address defaults to `0x3C`. If your module uses `0x3D`, update `OLED_ADDR` in the sketch.

### RGB Status LED

The sketch uses `RGB_BUILTIN` if it is defined by the board package (ESP32-S3 DevKitC-1 has one built in). If not, it falls back to GPIO 48. Override by defining `STATUS_LED_PIN`.

---

## ESP32 Dev Module (GPS Logger Variant) Wiring

### MCP2515

```
ESP32 Dev Module     MCP2515 Module
────────────────     ──────────────
3.3 V        ──────► VCC
GND          ──────► GND
GPIO 18 SCK  ──────► SCK
GPIO 23 MOSI ──────► SI
GPIO 19 MISO ◄──────  SO
GPIO 26      ──────► CS
GPIO 27      ◄──────  INT
```

### GPS Module (UART)

```
ESP32 Dev Module     GPS Module
────────────────     ──────────
3.3 V / 5 V  ──────► VCC  (check your module's voltage)
GND          ──────► GND
GPIO 16 RX2  ◄──────  TX
GPIO 17 TX2  ──────► RX
```

### microSD Module

```
ESP32 Dev Module     microSD
────────────────     ───────
5 V          ──────► VCC
GND          ──────► GND
GPIO 18 SCK  ──────► CLK
GPIO 23 MOSI ──────► MOSI
GPIO 19 MISO ◄──────  MISO
GPIO 5       ──────► CS
```

### SSD1306 OLED

```
ESP32 Dev Module     SSD1306
────────────────     ───────
3.3 V        ──────► VCC
GND          ──────► GND
GPIO 21 SDA  ──────► SDA
GPIO 22 SCL  ──────► SCL
```

---

## Power Considerations

| Scenario | Current draw (approx.) |
|---|---|
| ESP32-S3 active (Wi-Fi TX) | ~240 mA peak |
| ESP32-S3 idle (Wi-Fi connected) | ~80 mA |
| MCP2515 module | ~10 mA |
| SSD1306 OLED | ~5 mA |
| microSD (active) | ~100 mA peak |
| **Total (worst case)** | **~360 mA @ 3.3 V** |

The OBD-II port provides 12 V / 1–2 A. Use a compact automotive-grade DC-DC buck converter (12 V → 5 V, ≥ 1 A) followed by the ESP32 dev board's onboard 3.3 V LDO, or a 12 V → 3.3 V converter rated for ≥ 500 mA.

Add an automotive-rated blade fuse (1–2 A) in series with the 12 V line.

---

## CAN Bus Termination

The OBD-II port is on an **in-vehicle** CAN network that is already terminated (120 Ω at each end of the bus). **Do not add additional termination resistors** — doing so will upset the bus impedance and cause communication errors.

Some MCP2515 breakout boards include a 120 Ω termination resistor that can be removed by cutting a jumper or desoldering a resistor (refer to your module's silkscreen).

---

## MCP2515 Oscillator Frequency

| Crystal on module | `mcp_can` constant | Notes |
|:---:|---|---|
| 8 MHz | `MCP_8MHZ` | Most common; used by default in this firmware |
| 16 MHz | `MCP_16MHZ` | Some modules; change `CAN_OSC_FREQ` in the sketch |

If the crystal frequency is wrong, the MCP2515 will fail to initialize or produce garbled frames.
