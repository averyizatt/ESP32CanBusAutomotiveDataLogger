#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS = 26;
const int CAN_INT = 27;

MCP_CAN CAN0(CAN_CS);

unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("ESP32 CAN basic test starting");

  SPI.begin(18, 19, 23, CAN_CS);

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

  unsigned long now = millis();

  // send every 500 ms
  if (now - lastSend > 500) {
    lastSend = now;

    byte pingMsg[4] = {'P','I','N','G'};

    CAN0.sendMsgBuf(0x100, 0, 4, pingMsg);
    Serial.println("Sent PING");
  }

  // check for reply
  if (!digitalRead(CAN_INT)) {
    unsigned long id;
    byte len;
    byte buf[8];

    CAN0.readMsgBuf(&id, &len, buf);

    if (id == 0x101) {
      if (buf[0] == 'O' && buf[1] == 'K') {
        Serial.println("Arduino connected");
      }
    }
  }
}
