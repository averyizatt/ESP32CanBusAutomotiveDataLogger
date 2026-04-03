# OBD-II PID Reference

This document lists the OBD-II **Service 01** (current data) PIDs supported by this firmware, along with the raw byte formula to convert the ECU response into a human-readable value.

OBD-II requests are sent to CAN ID `0x7DF` (broadcast) using a standard 8-byte frame. The ECU responds on IDs `0x7E8`–`0x7EF`.

---

## Request Frame Format

```
Byte  Value  Meaning
  0   0x02   Number of additional bytes following
  1   0x01   Service 01 (current data)
  2   PID    The PID you are requesting
  3–7  0xCC  Padding (don't-care bytes)
```

## Response Frame Format

```
Byte  Meaning
  0   Number of additional bytes following
  1   0x41  (Service 01 response = 0x40 + 0x01)
  2   PID   (echo of requested PID)
  3+  Data bytes A, B, C, D …
```

---

## Supported PIDs

### Engine & Drivetrain

| PID | Name | Formula | Range | Unit |
|:---:|---|---|---|---|
| `0x04` | Engine Load | `A × 100 / 255` | 0–100 | % |
| `0x05` | Coolant Temperature | `A − 40` | −40–215 | °C |
| `0x0A` | Fuel Pressure | `A × 3` | 0–765 | kPa (gauge) |
| `0x0C` | Engine RPM | `(A × 256 + B) / 4` | 0–16 383.75 | RPM |
| `0x0D` | Vehicle Speed | `A` | 0–255 | km/h |
| `0x11` | Throttle Position | `A × 100 / 255` | 0–100 | % |
| `0x1F` | Engine Run Time | `A × 256 + B` | 0–65 535 | s |
| `0x5C` | Engine Oil Temperature | `A − 40` | −40–215 | °C |
| `0x5E` | Engine Fuel Rate | `(A × 256 + B) / 20` | 0–3 276.75 | L/h |

### Air & Fuel

| PID | Name | Formula | Range | Unit |
|:---:|---|---|---|---|
| `0x0F` | Intake Air Temperature | `A − 40` | −40–215 | °C |
| `0x2F` | Fuel Level | `A × 100 / 255` | 0–100 | % |
| `0x33` | Barometric Pressure | `A` | 0–255 | kPa (absolute) |
| `0x46` | Ambient Air Temperature | `A − 40` | −40–215 | °C |

### Lambda / O₂ Sensors

| PID | Name | Formula | Unit |
|:---:|---|---|---|
| `0x14` | O₂ Sensor 1 Voltage | `B / 200` | V |
| `0x24` | O₂ S1 Equivalence Ratio (λ) + Current | `((A × 256 + B) / 32 768) × 2` (λ), `(C × 256 + D) / 256 − 128` (mA) | λ, mA |
| `0x44` | Fuel–Air Equivalence Ratio (λ) | `(A × 256 + B) × 2 / 65 536` | — |

### Vehicle & Distance

| PID | Name | Formula | Range | Unit |
|:---:|---|---|---|---|
| `0x31` | Distance Since DTC Cleared | `A × 256 + B` | 0–65 535 | km |
| `0x42` | Control Module Voltage | `(A × 256 + B) / 1000` | 0–65.535 | V |
| `0xA6` | Odometer | `(A × 2^24 + B × 2^16 + C × 2^8 + D) / 10` | 0–429 496 729.5 | km |

### Fuel Rail Pressure

| PID | Name | Formula | Unit |
|:---:|---|---|---|
| `0x59` | Fuel Rail Absolute Pressure | `(A × 256 + B) × 10` | kPa |
| `0x5A` | Relative Accelerator Pedal Position | `A × 100 / 255` | % |

---

## Checking Supported PIDs

Send PID `0x00` (Service 01) to request a bitmask of PIDs `0x01`–`0x20` supported by the vehicle's ECU. Each subsequent group of 32 PIDs can be probed by requesting PIDs `0x20`, `0x40`, `0x60`, `0x80`, `0xA0`, and `0xC0`.

```
Request:  7DF 02 01 00 CC CC CC CC CC
Response: 7E8 06 41 00 [A] [B] [C] [D]
```

Bits in `A`–`D` correspond to PIDs `0x01`–`0x20` (MSB first). A `1` bit means that PID is supported.

---

## Adding Custom PIDs

To add a new PID to the firmware:

1. Add a `#define PID_MY_PARAM 0xXX` constant in `CanCommanderModule.cpp`.
2. Add a decoding `else if` block inside `managePID()` following the existing pattern.
3. For the web UI, update `/status.json` endpoint handling in `ESP32S3_CANOnly.ino` to include the new value.

---

## References

- [OBD-II PIDs — Wikipedia](https://en.wikipedia.org/wiki/OBD-II_PIDs)
- [SAE J1979 / ISO 15031-5](https://www.sae.org/standards/content/j1979_201702/)
