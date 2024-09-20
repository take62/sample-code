/*メイン制御コード/YFR_main_code_ver2_Shakedown
このプログラム概要：
                  シェイクダウン用のプログラム
                  APPSは１系統として、偏差チェックとAPPSの選定は未実装
                  インバータへの指令送信は基本1回行う，送信失敗➡成功するまで送信する．
                  懸念点：こちらの送信のタイミングとINVの受信タイミングが合わない場合があるかも(現時点で問題は確認されない)
トルク値リミットorフィードバックを実装
*/

#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);  //CAN通信ポート

const int IGNSW_PIN = 5;      //IGNSWのデジタル入力ピン
const int Precharge_PIN = 6;  //Precharge制御マイコンのデジタル入力ピン
const int PreLED_PIN = 2;     //Precharge完了LED出力ピン

const int RtoD_PIN = 8;     //Ready to Drive 入力ピン9->8->9
const int RtoDLED_PIN = 3;  //Ready to Drive LED出力ピン

//####APPS_1
const int APPS_PIN_1 = A4;  //APPS,アナログ入力ピン
int APPS_val_1 = 0;         //APPS,アナログ入力の変数

//####APPS_2
const int APPS_PIN_2 = A5;  //APPS,アナログ入力ピン
int APPS_val_2 = 0;         //APPS,アナログ入力の変数

//####BSE
const int Brake_PIN = A6;  //Brake, アナログ入力ピン
int Brake_val = 0;         //Brake,アナログ入力の変数
int Brake_RtoD_Thold = 150;  // Brake, Ready to Drive用の閾値, とりあえず毎回調整
int Brake_Check_Thold = 160;  // Brake, Apps妥当性チェック, とりあえず毎回調整

const float min_val = 1024 * 0.1;                           //APPS,PST360-G2 出力関数：0°＝10%
const float max_val = 1024 * 0.9;                           //APPS,PST360-G2 出力関数：360°＝90%
const int deg_0 = 14;                                       //APPS,作動開始角度, 遊びを含む，ここ開始点でトルク0
const int deg_m = 23;                                       //APPS,作動限界角度，ここ終点でトルクMAX
const float APPS_tread_threshold = (deg_m - deg_0) * 0.25;  //APPSの妥当性チェックのしきい値，作動範囲の25%
const float APPS_rest_threshold = (deg_m - deg_0) * 0.05;   //APPSの妥当性チェックの解除しきい値，作動範囲の5%
const int TorMin = 0;                                    //APPS,入力値下限, DEC:2000～2120, HEX:0x7d0～0x848, INV:0～60[Nm]
const int TorMax = 70;                                    //APPs,入力値上限 TorMax=0.5*トルク上限値

//ファン稼働のしきい値温度

short Deg_Val = 50;     //[℃]
const int Fan_PIN = 7;  //ファンの制御ピン，アナログでPWM制御してもok

//APPSとブレーキ操作の妥当性チェック, 25%条件
bool APPS_BK_Check = false;

//回転数リミッターの変数//  まだ車両でのテストができていない
//int T_delta = TorMin;　
const int N_lim = 10000/9;  //回転数limit[rpm]

//トルク制限用の変数
short Motor_lim = 2500/9; //(25km/h)

double Ki = -0.1; //指数関数の基数 

//ピン入力状態モニタ用変数＆定数
unsigned int MonTimerMS=0;
const unsigned int MonTimerMS_Interval  =1500;

void setup() {
  Serial.begin(9600);
  pinMode(IGNSW_PIN, INPUT);
  pinMode(Precharge_PIN, INPUT);
  pinMode(PreLED_PIN, OUTPUT);
  pinMode(APPS_PIN_1, INPUT);
  pinMode(APPS_PIN_2, INPUT);
  pinMode(RtoD_PIN, INPUT);//2024/05/08INPUT_PULLUP->2024/07/18INPUT
  pinMode(RtoDLED_PIN, OUTPUT);
  pinMode(Brake_PIN, INPUT);
  pinMode(Fan_PIN, OUTPUT);

  //Serial.priFFntln("Serial Monitor");

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）)
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
}

void loop() {
  static byte buf_s[] = { 0x00, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };  //CAN通信送信バッファ
  unsigned long id;                                                          //ID
  byte len;                                                                  //フレームの長さ
  byte buf_r[8];                                                             //CAN通信受信バッファ
  int RtoD_Bra = 0;
  int RtoD = LOW;

  unsigned short Motor_cur, Voltage, Anomaly_sig;
  short Motor_rev;
  short Motor_revpo; //正の値に変換
  unsigned short Gate_sta;
  static unsigned short Mg_ecu, Op_status;
  unsigned char sndStat;

  int IGNSWsta = digitalRead(IGNSW_PIN);  //IGNSWの状態，HIGH or LOW
  static int ECUsta = 0;                  //MG-ECUの状態変数，ON：1，OFF：0
  static int Dissta_on = 0;               //Co放電要求 Active状態 1:送信済み or 0：未送信
  static int Dissta_off = 0;              //Co放電要求 Inactive状態 1：送信済み or 0：未送信

  short INV_deg;
  short Mo_deg;

  //CAN処理
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&id, &len, buf_r);
    //ID:785
    if (id == 0x311) {
      if (0) {
        Serial.print("ID:");
        Serial.print(id);
        Serial.print(" => ");
      }
      //MG-ECUシャットダウン許可
      Mg_ecu = bitRead(buf_r[0], 0);
      if (0) {
        Serial.print("ECUsd:");
        if (Mg_ecu == B0) {
          Serial.print("Not Enable;  ");
        } else if (Mg_ecu == B1) {
          Serial.print("Enable;  ");
        }
      }

      //制御状態
      Op_status = buf_r[0];
      for (int n = 6; n < 8; n++) {
        bitClear(Op_status, n);  //LSBから6-7ビット目を「0」ビットにする
      }
      Op_status = Op_status >> 3;
      //モニター使用時はif(1), 不使用はif(0)
      if (1) {
        Serial.print("Status:");
        if (Op_status == B000) {
          Serial.print("init;  ");  //init
        } else if (Op_status == B001) {
          Serial.print("Precharge;  ");  //Precharge
        } else if (Op_status == B010) {
          Serial.print("Standby;  ");  //Standby
        } else if (Op_status == B011) {
          Serial.print("Troque Contorol;  ");  //Torque control
        } else if (Op_status == B111) {
          Serial.print("Rapid Discharge;  ");  //Rapid discharge
        } else {
          Serial.print("--;  ");  //--:Reserved
        }
      }
      //ゲート駆動状態
      Gate_sta = buf_r[0];
      for (int i = 3; i < 8; i++) {
        bitClear(Gate_sta, i);  //LSBから3-7ビット目を「0」ビットにする
      }
      Gate_sta = Gate_sta >> 1;
      if (0) {
        Serial.print("gate:");
        if (Gate_sta == B00) {
          Serial.print("Short;  ");  //Short circuit
        } else if (Gate_sta == B01) {
          Serial.print("Free;  ");  //freeWheel
        } else if (Gate_sta == B10) {
          Serial.print("Run;  ");  //PWM run
        } else {
          Serial.print("--;  ");  //--:Reserved
        }
      }

      //モータ回転数[rpm]
      Motor_rev = (buf_r[2] << 8) | buf_r[1];
      Motor_rev = Motor_rev - 14000;  //モータ回転数[rpm],オフセット-14000，実車の正誤はまだ確認できていない(8/25)
      Motor_revpo=abs(Motor_rev)/9;

      if (0) {
        Serial.print("Rev:");
        Serial.print(Motor_rev);
        Serial.print(" [rpm];  ");
      }

      //モータ相電流
      Motor_cur = (buf_r[4] << 8) | buf_r[3];  //モータ相電流3byteと4byteを結合
      for (int m = 10; m < 16; m++) {
        bitClear(Motor_cur, m);  //LSBから10-15ビット目を「0」ビットにする
      }
      Motor_cur = Motor_cur * 0.5;  //モータ相電流[Arms]，実車の正誤はまだ確認できていない(8/25)
      if (0) {
        Serial.print("Cur:");
        Serial.print(Motor_cur, DEC);
        Serial.print(" [Arms];  ");
      }

      //モータ電圧
      Voltage = (buf_r[5] << 8) | buf_r[4];  //モータ電圧4byteと5byteを結合
      for (int j = 12; j < 16; j++) {
        bitClear(Voltage, j);  //LSBから12-15ビット目を「0」ビットにする
      }
      Voltage = Voltage >> 2;  //モータ電圧[V]，実車の正誤はまだ確認できていない(8/25)
      if (1) {
        Serial.print("Vol:");
        Serial.print(Voltage, DEC);
        Serial.print(" [V];  ");
      }

      //異常状態 信号
      Anomaly_sig = buf_r[7] >> 5;  //異常状態 信号
      if (1) {
        Serial.print("Error: ");
        if (Anomaly_sig == B000) {
          Serial.print("No Error;  ");
        } else if (Anomaly_sig == B001) {
          Serial.print("power limit;  ");  //derating, モータ出力制限
        } else if (Anomaly_sig == B010) {
          Serial.print("Warning;  ");
        } else if (Anomaly_sig == B100) {
          Serial.print("Error;  ");
        } else if (Anomaly_sig == B101) {
          Serial.print("Critical Error;  ");
        } else {
          Serial.print("--;  ");  //--:Reserved
        }
      }
      Serial.println();

    }
    //ID:801
    else if (id == 0x321) {
      if (0) {
        Serial.print("ID:");
        Serial.print(id);
        Serial.print(" => ");
      }
      // 温度
      INV_deg = buf_r[0] - 35;  //インバータ温度[℃],オフセット-40，実車の正誤はまだ確認できていない(8/25)
      Mo_deg = buf_r[4] - 35;   //モータ温度[℃],オフセット-40，実車の正誤はまだ確認できていない(8/25)
      if (0) {
        Serial.print("INV:");
        Serial.print(INV_deg, DEC);
        Serial.print(" [℃];  ");
      }
      if (0) {
        Serial.print("Motor:");
        Serial.print(Mo_deg, DEC);
        Serial.print(" [℃];  ");
      }
      Serial.println();
      //"トルク制限値"は必要ないので処理しない
    }
  }

  //ファンのON,OFF
  /*
  if (Deg_Val < INV_deg || Deg_Val < Mo_deg) {
    digitalWrite(Fan_PIN, HIGH);
  } else {
    digitalWrite(Fan_PIN, LOW);
  }
  */
  if(Op_status == B011) //走るときだけ回す
  {
    digitalWrite(Fan_PIN, HIGH);
  }
  else
  {
    digitalWrite(Fan_PIN, LOW);
  }
  //digitalWrite(Fan_PIN, HIGH);

  //IGNSWの状態分岐，HIGH or LOW
  if (IGNSWsta == LOW) {
    digitalWrite(PreLED_PIN, LOW);                                        //PrechargeLED : OFF (-)
    digitalWrite(RtoDLED_PIN, LOW);                                       //RtoDLED : OFF (-)
    if (ECUsta != 0) {                                                    //MG-ECU"OFF"ではないなら以下を実行
      byte buf_s[] = { 0x00, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };  //"MG-ECU"OFF"送信"
      sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("MG-ECU : OFF");
        ECUsta = 0;
      } else {
        Serial.println("MG-ECU (ON) : Error Sending Message...");
        ECUsta = 1;
      }
    }
    if (Op_status == B111) {  //rapid discharge状態
      if (Dissta_on != 1) {   //Co放電要求Active済みじゃないなら以下を実行
        if (Voltage >= 60) {
          byte buf_s[] = { 0x02, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };  //"Co放電要求 Active送信"
          sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("Discharge Command : ON");
            Dissta_on = 1;
          } else {
            Serial.println("Dis command (ON): Error Sending Message...");
            Dissta_on = 0;
          }
        }
      }
    } else if (Op_status == B010) {  //standby状態
      //"Co放電要求 Inactive送信"
      if (Dissta_off != 1) {
        byte buf_s[] = { 0x00, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("Discharge Command : OFF");
          Dissta_off = 1;
        } else {
          Serial.println("Dis command (OFF): Error Sending Message...");
          Dissta_off = 0;
        }
      }
    }
  } else if (IGNSWsta == HIGH) {
    if (0) {
      Serial.print("ID:301 => ");
    }
    Dissta_on = 0;  //dischargeの指令を初期化
    Dissta_off = 0;
    if (Op_status == B011) {  //torque control状態
      //"トルク値CAN送信プログラム"
      // トルク送信値の範囲⇒DEC:2000～2120, HEX:0x7d0～0x848
      digitalWrite(RtoDLED_PIN, HIGH);                                    //RtoDLED : OFF (グリーン)
      digitalWrite(PreLED_PIN, LOW);                                      //PrechargeLED : OFF (-) ※念のため
      byte buf_s[] = { 0x01, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };  //0byte目はMG-ECU:on, Co放電要求:off 固定
      int T_in = 0;                                                       //トルク入力用の変数

      //各種センサーの値を取得
      APPS_val_1 = analogRead(APPS_PIN_1); //apps_1の値を取得(10bit)
      //Serial.print("APPS_1:");Serial.println(APPS_val_1);Serial.print(" "); //apps_1の値(アナログ値)を表示

      APPS_val_2 = analogRead(APPS_PIN_2); //apps_1の値を取得(10bit)
      //Serial.print("APPS_2:");Serial.print(APPS_val_2);Serial.print(" "); //apps_2の値(アナログ値)を表示

      Brake_val = analogRead(Brake_PIN); //bppsの値を取得(10bit)
      //Serial.print("BPPS:");Serial.print(Brake_val);Serial.print(" "); //bppsの値(アナログ値)を表示

      //各種センサーの値を変換
      int deg_1 = map(APPS_val_1, min_val, max_val, 0, 359);  //アナログ入力値を角度に置き換え(apps_1)
      //Serial.print("deg_1:");Serial.println(deg_1);Serial.print(" "); //apps_1の角度を表示(°)

      int deg_2 = map(APPS_val_2, min_val, max_val, 0, 359);  //アナログ入力値を角度に置き換え(apps_2)
      //Serial.print("deg_2:");Serial.print(deg_2);Serial.print("  "); //apps_2の角度を表示(°)

      int deg_in;                                        //トルクに変換するための角度変数
      float deg_tread;                                   //appsの動作量(APPS25％を判断するための変数)
      float deg_dev = deg_1 - deg_2;                     //2つのセンサーの角度の偏差 
      float deg_dev_val = (abs(deg_dev) / 360.0) * 100;  //偏差を評価 

      //APPSのの小さい値の法を選ぶ
      if (deg_dev > 0) {
        deg_in = constrain(deg_2, deg_0, deg_m);  //角度の範囲を制限
        deg_tread = deg_in - deg_0;               //APPS踏み込み量を計算
        //Serial.print("APPS_2を採用");
      } else {
        deg_in = constrain(deg_1, deg_0, deg_m);  //角度の範囲を制限
        deg_tread = deg_in - deg_0;               //APPS踏み込み量を計算
        //Serial.print("APPS_1を採用");
      }

      //トルク制限(車両速度が40km/hで作動)
      short Tordec=0; //トルク減少量
      short Tordec_lim=TorMax; //トルク減少量限界値(最大に減少してトルク値が20となる)
      short TorMax_val; //使用するトルク量
      if(Motor_lim<Motor_revpo)
      {
        Tordec=map(Motor_revpo,Motor_lim,N_lim,TorMin,Tordec_lim);
        if(Tordec>=0||Tordec<=Tordec_lim)
        {
          TorMax_val=TorMax*exp(Ki*Tordec);
        }
        else
        {
          TorMax_val=0;
        }
      }
      else
      {
        TorMax_val=TorMax;
      }
      Serial.print("Motor_revpo:");Serial.print(Motor_revpo);
      Serial.print(" Tordec_lim:");Serial.print(Tordec_lim);
      Serial.print(" Tordec:");Serial.print(Tordec);
      Serial.print(" TorMax_val:");Serial.print(TorMax_val);

      //APPSの妥当性チェック
      if (deg_dev_val <= 10) {                                                       //偏差10%以下か？
        if (APPS_BK_Check == false) {                                                //前処理で25％条件に該当したか？
          if (deg_tread >= APPS_tread_threshold && Brake_val > Brake_Check_Thold) {  //同時踏みか？
            T_in = 0;                                                             //トルク値は0にする
            APPS_BK_Check = true;
          } else {
            T_in = map(deg_in, deg_0, deg_m, TorMin, TorMax_val);//0~(120)
          }
        } else if (APPS_BK_Check == true) {
          if (deg_tread < APPS_rest_threshold) {  //踏みこみ量が5%以下か？
            T_in = map(deg_in, deg_0, deg_m, TorMin, TorMax_val);//0~(120)
            APPS_BK_Check = false;
          } else {
            T_in = 0;
          }
        }
      } else {
        T_in = 0;
        //Serial.print("  !!偏差が10%を超え,T_in=0");
      }

      /*
      回転数リミッターのテストはモータ暴走に気を付けて，慎重に
      ///////回転数リミッター
      if (T_delta > T_in) {
        T_delta = T_in;
      } else if (Motor_rev > N_lim) {
        if (T_in > T_delta) {
          T_delta++;
        } else if (T_delta > 2000) {
          T_delta--;
        }
      }
      int T_out = T_in - T_delta;　//回転数リミッターあり
      ///////回転数リミッター
      */

      int T_out = 2000 - T_in;  //回転数リミッターなし
      Serial.print(" torque:");Serial.print(T_out);

      buf_s[1] = byte(T_out & 0xFF);
      buf_s[2] = byte((T_out >> 8) & 0xF);
      sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);  //トルク値CAN送信

      if (sndStat == CAN_OK) {
        if (0) {
          Serial.print("Tor:");
          Serial.println(T_out);
          //Serial.println(" [Nm]  ");
        }
      } else {
        Serial.println("Torque : Error Sending Message...");
      }
    } else if (Op_status == B001) {  //Precharge状態
      int Presta = digitalRead(Precharge_PIN);
      if (Presta == LOW) {               //Cpu5:precharge制御は完了か？
        digitalWrite(PreLED_PIN, HIGH);  //PrechargeLED : ON (オレンジ)
        if (ECUsta != 1) {  //MG-ECU"OFF"なら以下を実行
          //Ready to Drive の操作

          int Brake_val_RtoD = analogRead(Brake_PIN); //ReadytoDrive用のブレーキ値を取得
          if (Brake_val_RtoD >= Brake_RtoD_Thold) {
            RtoD_Bra = 1;
          } else {
            RtoD_Bra = 0;
          }
          int R = 0;
          if (RtoD_Bra == 1) {
            
            int RtoDSW = digitalRead(RtoD_PIN);
            
            if (RtoDSW == HIGH) //複数ms押し込み必要？
            {
              //"MG-ECU"ON"送信"
              byte buf_s[] = { 0x01, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };
              sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);  //ID, 標準フレーム:0, データ長:8
              if (sndStat == CAN_OK) 
              {
                Serial.println("MG-ECU : ON");
                ECUsta = 1;
                digitalWrite(PreLED_PIN, LOW);  //PrechargeLED : OFF (-)
              } 
              else 
              {
                Serial.println("MG-ECU (ON) : Error Sending Message...");
                ECUsta = 0;
              }
            }
            else
            {
              ;
            }
          }
          else
          {
            ;
          }
        }
      }
    }
  }
}  //loop終