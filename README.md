# ESP32 CAN/GPS Automotive Data Logger

Overview
This project implements an ESP32-based data logger that reads GPS (TinyGPSPlus) and CAN bus frames (MCP2515 via ) and logs structured CSV data to an SD card. The device hosts a web UI served from the SD card and provides API endpoints for a modern dashboard. The firmware supports OTA updates (protected by an admin token), mDNS advertising (), a small OLED (Adafruit SSD1306) for local status, and optional WebSocket streaming for live frames.

Key features
- CAN bus capture (MCP2515 driver via ) and parsed fields (RPM, TPS, MAP, temperatures, battery voltage).
- GPS parsing via  (latitude, longitude, speed, altitude, satellites).
- Web UI served from SD card; dynamic API endpoints (, , , ).
- WebSocket server for low-latency frame streaming to the dashboard ( library).
- Non-blocking SD logging via a FreeRTOS-backed queue and background writer task.
- Small configuration kept on SPIFFS (, ) to save SPI flash space.

Dependencies / Libraries
- TinyGPSPlus — GPS decoding
- mcp_can / MCP_CAN — MCP2515 CAN controller driver
- Adafruit_SSD1306 & Adafruit_GFX — OLED display
- SD / SPIFFS — file systems and web assets
- WebServer & ESPmDNS — HTTP server and mDNS advertisement
- WebSockets — WebSocket server for live streaming
- ArduinoJson — JSON serialization for API responses
- Update — OTA firmware updates
- FreeRTOS primitives for background tasks / queues

Hardware
- ESP32 development board
- MCP2515 CAN controller (SPI)
- GPS module (UART)
- SD card (SPI)
- 128x32 I2C SSD1306 OLED (optional)

Build & Deployment
- Build: either PlatformIO (recommended) or Arduino IDE with ESP32 boards package.
- Before first deploy: copy the contents of  (HTML/CSS/JS) to the SD card root; the firmware no longer serves embedded fallback pages.
- After flashing, configure Wi‑Fi networks by uploading a  to SPIFFS or via the web UI.

File layout (key files)
-  — main firmware
-  — web assets (index.html, script.js, style.css)

Security & Operational Notes
- Admin token is stored in SPIFFS (file ) — firmware checks this for OTA and admin operations.
- Wi‑Fi credentials in  are obfuscated using a simple XOR derived from ESP efuse MAC (intended to deter casual inspection, not cryptographically secure).
- Logs are written to  on SD to keep days separate.

Support
- For build or runtime issues, consult the serial output at 115200 baud and ensure the SD card contains the web UI files.
