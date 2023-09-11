//CAN受信基本コード
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);

void setup()
{
  Serial.begin(9600);
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）)
  } else {
    Serial.println("Can init fail");
  }
}

void loop() {
  unsigned long id;
  byte len;
  byte buf[8];
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&id, &len, buf);
    Serial.print("ID:");
    Serial.print(id);
    Serial.print(" DATA");
    for (byte i = 0; i < len; i++) {
      Serial.print(":");
      Serial.print(buf[i], HEX);
    }
    Serial.println();
  }//delay(500);
}
