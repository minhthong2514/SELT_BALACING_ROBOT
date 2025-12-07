#define ena 12
#define in1 14
#define in2 27
#define in3 26
#define in4 25
#define enb 33

const int freq = 1000;
const int resolution = 8;

void stop() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}

void forward() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  ledcWrite(ena, 255);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
  ledcWrite(enb, 255);
  Serial.println("TIEN");
}

void backward() {
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  ledcWrite(ena, 255);
    digitalWrite(in3, 1);
  digitalWrite(in4, 0);
  ledcWrite(enb, 255);
  Serial.println("LUI");
}

void setup() {
  Serial.begin(115200);
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  ledcAttach(ena, freq, resolution);
  ledcAttach(enb, freq, resolution);
  delay(1000);
  stop();
}

void loop() {
  forward();
  delay(3000);
  stop();
  delay(2000);
  backward();
  delay(3000);
}
