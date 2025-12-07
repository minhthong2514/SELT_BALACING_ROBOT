int motor1Pin1 = 12;
int motor1Pin2 = 14;
int motor2Pin1 = 27;
int motor2Pin2 = 26;
const int freq = 100;
const int resolution = 8;
int dutyCycle = 255;

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  ledcAttach(motor1Pin1, freq, resolution);
  ledcAttach(motor1Pin2, freq, resolution);
  ledcAttach(motor2Pin1, freq, resolution);
  ledcAttach(motor2Pin2, freq, resolution);
  stop();
  delay(1000);
  Serial.begin(115200);
}

void tien() {
  ledcWrite(motor1Pin1, 0);
  ledcWrite(motor1Pin2, 255);
  ledcWrite(motor2Pin1, 0);
  ledcWrite(motor2Pin2, 255);
  Serial.println("tien");
}
void lui() {
  ledcWrite(motor1Pin2, 0);
  ledcWrite(motor1Pin1, 255);
  ledcWrite(motor2Pin2, 0);
  ledcWrite(motor2Pin1, 255);
  Serial.println("lui");
}
void stop() {
  ledcWrite(motor1Pin2, 0);
  ledcWrite(motor1Pin1, 0);
  ledcWrite(motor2Pin2, 0);
  ledcWrite(motor2Pin1, 0);
  Serial.println("dung");
}

void loop() {
  tien();
  delay(3000);
  stop();
  delay(3000);
  lui();
  delay(3000);
  //lui();
  //delay(3000);
}
