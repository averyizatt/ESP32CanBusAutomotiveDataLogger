# CAN Commander Module

A reusable Arduino module that adds an interactive serial CAN analysis tool to any sketch that already has an MCP2515 initialized.

## Usage

1. Copy `CanCommanderModule.h` and `CanCommanderModule.cpp` into your sketch folder.
2. Declare `const int CAN_CS = <your CS pin>;` in your main sketch.
3. Call `canCommanderSetup()` from `setup()` (or on demand when entering commander mode).
4. Call `canCommanderLoop()` from `loop()`.

```cpp
#include "CanCommanderModule.h"

const int CAN_CS = 38;

void setup() {
  Serial.begin(115200);
  canCommanderSetup();   // blocks until a mode is selected
}

void loop() {
  canCommanderLoop();
}
```

## Modes

| Key | Mode | Description |
|:---:|---|---|
| `1` | Read All | Print every CAN frame; press `a` to toggle ASCII |
| `2` | Write | Enter ID/DLC/data, then set messages-per-second rate |
| `3` | Speed Test | Count frames/second |
| `4` | Read Filtered | Apply hardware mask + filter before reading |
| `5` | Value Tracker | Watch a filtered ID; report changed bytes + min/max |
| `6` | PID Manager | Send OBD-II Mode 01 PID requests and decode responses |

Press `s` in any mode to stop and reset the device.

## Dependencies

- [MCP2515 library](https://github.com/autowp/arduino-mcp2515) (`autowp/arduino-mcp2515`)
