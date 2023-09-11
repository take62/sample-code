//ID:785 送信テスト用プログラム

#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);
void setup()
{
  Serial.begin(9600);
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
}

void loop()
{
  static byte buf_785[] = {0x13, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xA0};
  static byte buf_801[] = {0x37, 0x00, 0x00, 0x7D, 0xA, 0x00, 0x00, 0x00};
  CAN.sendMsgBuf(0x311, 0, 8, buf_785);//ID:0x__, 標準フレーム:0, データ長:8
  delay(10);
  CAN.sendMsgBuf(0x321, 0, 8, buf_801);//ID:0x__, 標準フレーム:0, データ長:8
  delay(10);
}
