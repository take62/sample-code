//ポテションメータのテストコード

const int Brake_PIN = A6;
int val = 0;


const float min_val = 1024 * 0.1;  //APPS,PST360-G2 出力関数：0°＝10%
const float max_val = 1024 * 0.9;  //APPS,PST360-G2 出力関数：360°＝90%
const int deg_0 = 26;              //APPS,作動開始角度
const int deg_m = 30;              //APPS,作動限界角度

void setup() {
  Serial.begin(9600);
  pinMode(Brake_PIN, INPUT);
  
}

void loop() {
  //unsigned long time = 0;
  //time = millis();
  val = analogRead(Brake_PIN);

  int deg = map(val, min_val, max_val, 0, 359);  //アナログ入力値を角度に置き換え

  int deg_in = constrain(deg, deg_0, deg_m);  //角度の範囲を制限

  float Voltage = (Brake_PIN * 5) / 1024;
  Serial.print("   val:");
  Serial.print(val);
  Serial.print("  deg:");
  Serial.print(deg);

  Serial.print("  deg_in:");
  Serial.print(deg_in);
  Serial.print("  Voltage:");
  Serial.print(Voltage);

  Serial.println();

}
