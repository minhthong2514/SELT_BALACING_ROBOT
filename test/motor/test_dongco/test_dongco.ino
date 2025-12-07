int output = 255;

void setup() {
  Serial.begin(115200);
  // Khởi tạo các chân điều khiển động cơ
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);

  // Tắt động cơ ban đầu
  analogWrite(3, LOW);
  analogWrite(5, LOW);
  analogWrite(6, LOW);
  analogWrite(9, LOW);
}

void loop() {
  moveForward();
  Serial.println("Đi thẳng!");
  delay(4000);
  moveBackward();
  Serial.println("Đi lùi!");
  delay(4000);
  stopMotors();
  Serial.println("Dừng lại!");
  delay(2000);
}
void moveForward() {
  analogWrite(3, 255);
  analogWrite(5, 0);
  analogWrite(6, 255 );
  analogWrite(9, 0);
}

// Hàm điều khiển robot di chuyển về phía sau
void moveBackward() {
  analogWrite(3, 0);
  analogWrite(5, 255);
  analogWrite(6, 0);
  analogWrite(9, 255);
}

// Hàm dừng động cơ
void stopMotors() {
  analogWrite(3, 0);
  analogWrite(5, 0);
  analogWrite(6, 0);
  analogWrite(9, 0);
}