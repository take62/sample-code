//20230211
//CAN送信基本コード
#include <mcp_can.h>
#include <SPI.h>
//byte can_count ;
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

  static byte buf[] = {0x11, 0x22, 0x33};
  //static byte buf[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x00};
  //buf[7]=can_count;

  unsigned char sndStat = CAN.sendMsgBuf(0x02, 0, 8, buf); //ID:0x02, 標準フレーム:0, データ長:8
  if (sndStat == CAN_OK) {
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }
  //Serial.println(buf[7],DEC);
  //can_count ++;
  //delay(1000);
}
