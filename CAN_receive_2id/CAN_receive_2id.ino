//ID別 全表示

#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);

void setup() {
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
    //ID:769
    if (0) {
      if (id == 0x301) {
        Serial.print("ID:");
        Serial.print(id);
        Serial.print(" => ");
        int ECU = bitRead(buf[0], 0);
        switch (ECU) {
          case 0:
            Serial.print("ECU:Off");
            break;
          case 1:
            Serial.print("ECU:ON");
            break;
        }
        int CAP = bitRead(buf[0], 1);
        switch (CAP) {
          case 0:
            Serial.print("     dis_off");
            break;
          case 1:
            Serial.print("     dis_on");
            break;
        }
        int TOR = (buf[1]) * 0.5;
        Serial.print("     tor:");
        Serial.print(TOR, DEC);
        Serial.print("[Nm]");
        Serial.println();
      }
    }
    //ID:785
    else if (id == 0x311) {
      unsigned short Motor_cur, Voltage, Anomaly_sig;
      short Motor_rev;
      unsigned short Mg_ecu, Op_status, Gate_sta;
      Serial.print("ID:");
      Serial.print(id);
      Serial.print(" => ");
      //MG-ECUシャットダウン許可
      Mg_ecu = bitRead(buf[0], 0);

      //制御状態
      Op_status = buf[0];
      for (int n = 6; n < 8; n++) {
        bitClear(Op_status, n);  //LSBから6-7ビット目を「0」ビットにする
      }
      Op_status = Op_status >> 3;

      //ゲート駆動状態
      Gate_sta = buf[0];
      for (int i = 3; i < 8; i++) {
        bitClear(Gate_sta, i);  //LSBから3-7ビット目を「0」ビットにする
      }
      Gate_sta = Gate_sta >> 1;

      //モータ回転数[rpm]
      Motor_rev = (buf[2] << 8) | buf[1];
      Motor_rev = Motor_rev - 14000;  //モータ回転数[rpm],オフセット-14000

      //モータ相電流
      Motor_cur = (buf[4] << 8) | buf[3];  //モータ相電流3byteと4byteを結合
      for (int m = 10; m < 16; m++) {
        bitClear(Motor_cur, m);  //LSBから10-15ビット目を「0」ビットにする
      }
      Motor_cur = Motor_cur * 0.5;  //モータ相電流[Arms]

      //モータ電圧
      Voltage = (buf[5] << 8) | buf[4];  //モータ電圧4byteと5byteを結合
      for (int j = 12; j < 16; j++) {
        bitClear(Voltage, j);  //LSBから12-15ビット目を「0」ビットにする
      }
      Voltage = Voltage >> 2;  //モータ電圧[V]

      //異常状態 信号
      Anomaly_sig = buf[7] >> 5;  //異常状態 信号

      ////////////////////信号を文字に割り当てる//////////////////
      //MG-ECUシャットダウン許可
      if (0) {
        Serial.print(" => ECUsd:");
        if (Mg_ecu == B0) {
          Serial.print("Not Enable");
        } else if (Mg_ecu == B1) {
          Serial.print("Enable");
        }
      }
      //ゲート駆動状態
      if (0) {
        Serial.print("  gate:");
        if (Gate_sta == B00) {
          Serial.print("Short");  //Short circuit
        } else if (Gate_sta == B01) {
          Serial.print("Free");  //freeWheel
        } else if (Gate_sta == B10) {
          Serial.print("Run");  //PWM run
        } else {
          Serial.print("--");  //--:Reserved
        }
      }
      //制御状態
      Serial.print(";  OpeSta: ");
      if (Op_status == B000) {
        Serial.print("init");
      } else if (Op_status == B001) {
        Serial.print("Precharge");
      } else if (Op_status == B010) {
        Serial.print("standby");
      } else if (Op_status == B011) {
        Serial.print("torque control");
      } else if (Op_status == B111) {
        Serial.print("rapid discharge");
      } else {
        Serial.print("--");  //--:Reserved
      }
      if (1) {
        Serial.print(";  Rev: ");
        Serial.print(Motor_rev, DEC);
        Serial.print(" [rpm]");
        Serial.print(";  Cur: ");
        Serial.print(Motor_cur, DEC);
        Serial.print(" [Arms]");
        Serial.print(";  Vol: ");
        Serial.print(Voltage, DEC);
        Serial.print(" [V]");
      }

      //異常状態
      if (1) {
        Serial.print(";  Error: ");
        if (Anomaly_sig == B000) {
          Serial.print("No Error");
        } else if (Anomaly_sig == B001) {
          Serial.print("power limit");  //derating, モータ出力制限
        } else if (Anomaly_sig == B010) {
          Serial.print("Warning");
        } else if (Anomaly_sig == B100) {
          Serial.print("Error");
        } else if (Anomaly_sig == B101) {
          Serial.print("Critical Error");
        } else {
          Serial.print("--");  //--:Reserved
        }
      }
      Serial.println();
    }
    //ID:801
    // else if (id == 0x321) {
    //   Serial.print("ID:");
    //   Serial.print(id);
    //   if (0) {
    //     // 温度
    //     short INV_deg = buf[0] - 40;  //インバータ温度[℃],オフセット-40
    //     short Mo_deg = buf[4] - 40;   //モータ温度[℃],オフセット-40
    //     Serial.print(" => INV:");
    //     Serial.print(INV_deg, DEC);
    //     Serial.print("[℃]");
    //     Serial.print("  Motor:");
    //     Serial.print(Mo_deg, DEC);
    //     Serial.print("[℃]");
    //   }
    //トルク制限
    if (0) {
      short upper_limit, under_limit;
      upper_limit = (buf[2] << 8) | buf[1];  //モータ上限制限トルク（ビットシフト前）
      for (int k = 12; k < 16; k++) {
        bitClear(upper_limit, k);  //LSBから12-15ビット目を「0」ビットにする
      }
      upper_limit = upper_limit * 0.5;  //モータ上限制限トルク

      under_limit = (buf[3] << 8) | buf[2];             //モータ下限制限トルク（ビットシフト前）
      under_limit = ((under_limit >> 4) - 1000) * 0.5;  //モータ下限制限トルク,オフセット-1000

      Serial.print("  upper_limit:");
      Serial.print(upper_limit, DEC);
      Serial.print("[Nm]");
      Serial.print("  under_limit:");
      Serial.print(under_limit, DEC);
      Serial.print("[Nm]");
    }
    //Serial.println();  //改行
  }
}
