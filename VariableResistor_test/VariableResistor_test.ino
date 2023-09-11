//ポテションメータのテストコード

const int APPS_PIN_1 = A4;
int val_1 = 0;
const int APPS_PIN_2 = A5;
int val_2 = 0;

const float min_val = 1024 * 0.1;  //APPS,PST360-G2 出力関数：0°＝10%
const float max_val = 1024 * 0.9;  //APPS,PST360-G2 出力関数：360°＝90%
const int deg_0 = 23;              //APPS,作動開始角度
const int deg_m = 38;              //APPS,作動限界角度
const int TorMin = 2000;           //APPS,入力値下限, DEC:2000～2120, HEX:0x7d0～0x848, INV:0～60[Nm]
const int TorMax = 2120;           //APPs,入力値上限
int T_in = 0;

int deg_in = 0;

void setup() {
  Serial.begin(9600);
  pinMode(APPS_PIN_1, INPUT);
  pinMode(APPS_PIN_2, INPUT);
}

void loop() {
  //unsigned long time = 0;
  //time = millis();
  val_1 = analogRead(APPS_PIN_1);
  val_2 = analogRead(APPS_PIN_2);


  int deg_1 = map(val_1, min_val, max_val, 0, 359);  //アナログ入力値を角度に置き換え
  int deg_2 = map(val_2, min_val, max_val, 0, 359);  //アナログ入力値を角度に置き換え

  float deg_dev = deg_1 - deg_2;                     //2つのセンサーの角度の偏差
  float deg_dev_val = (abs(deg_dev) / 360.0) * 100;  //偏差を評価

  if (deg_dev > 0) {
    deg_in = constrain(deg_2, deg_0, deg_m);  //角度の範囲を制限
    Serial.print("APPS_2を採用");
  } else {
    deg_in = constrain(deg_1, deg_0, deg_m);  //角度の範囲を制限
    Serial.print("APPS_1を採用");
  }

  Serial.print("  deg_dev:");
  Serial.print(abs(deg_dev));
  Serial.print("  deg_dev_val:");
  Serial.print(deg_dev_val);
  Serial.print("%");

  if (deg_dev_val <= 10) {
    T_in = map(deg_in, deg_0, deg_m, TorMin, TorMax);  //角度をトルク値に変換
  } else {
    Serial.print("  !!偏差が10%を超え,T_in=0");
    T_in = 2000;
  }

  //float Voltage = (APPS_val * 5) / 1024;
  Serial.print("   val_1:");
  Serial.print(val_1);
  Serial.print("  deg_1:");
  Serial.print(deg_1);

  Serial.print("  val_2:");
  Serial.print(val_2);
  Serial.print("  deg_2:");
  Serial.print(deg_2);


  Serial.print("  deg_in:");
  Serial.print(deg_in);
  //Serial.print("  Voltage:");
  //Serial.print(Voltage);
  Serial.print("  Torque:");
  Serial.print(T_in);

  /*
  time = millis();
  Serial.print("  time:");
  Serial.print(time);
  Serial.print("ms");
  */
  Serial.println();


  //delay(100);
}
