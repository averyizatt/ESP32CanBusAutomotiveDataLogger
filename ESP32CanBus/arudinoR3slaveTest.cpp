#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS = 10;
const int CAN_INT = 2;

MCP_CAN CAN0(CAN_CS);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Arduino CAN basic test starting");

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("CAN init OK");
  } else {
    Serial.println("CAN init fail");
    while (1);
  }

  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
}

void loop() {

  // check for ESP32 PING
  if (!digitalRead(CAN_INT)) {
    unsigned long id;
    byte len;
    byte buf[8];

    CAN0.readMsgBuf(&id, &len, buf);

    if (id == 0x100) {
      if (buf[0] == 'P' && buf[1] == 'I') {

        Serial.println("Received PING from ESP32");

        byte reply[2] = {'O','K'};
        CAN0.sendMsgBuf(0x101, 0, 2, reply);
        Serial.println("Sent OK back");
      }
    }
  }
}
