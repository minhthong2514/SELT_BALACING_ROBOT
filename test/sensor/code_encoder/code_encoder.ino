// Khai báo các chân kết nối
#define IN1 26
#define IN2 25
#define ENA 12
#define ENCODER_A 2
#define ENCODER_B 15

volatile int encoderCount = 0;
unsigned long lastTime = 0;
float speed = 0;  // tốc độ động cơ
float rpm;
const int freq = 100;
const int resolution = 8;


// Hàm ngắt để đọc tín hiệu từ Encoder
void IRAM_ATTR readEncoder() {
  // int stateA = digitalRead(ENCODER_A);
  // int stateB = digitalRead(ENCODER_B);
  
  // if (stateA == HIGH) {
  //   if (stateB == LOW) {
  //     encoderCount++;
  //   } else {
  //     encoderCount--;
  //   }
  // } else {
  //   if (stateB == LOW) {
  //     encoderCount--;
  //   } else {
  //     encoderCount++;
  //   }
  // }
  encoderCount++;
}


void setup() {
  // Thiết lập các chân điều khiển động cơ
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  ledcAttach(ENA, freq, resolution);
  // Thiết lập chân đọc encoder
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  
  // Kích hoạt ngắt để đọc encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, RISING);

  Serial.begin(115200);
  
  // // Khởi động động cơ theo hướng nhất định
  // digitalWrite(IN1, 0);
  // digitalWrite(IN2, 1);
  // ledcWrite(ENA, 255);  // điều chỉnh tốc độ PWM từ 0-255
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) {  // cập nhật mỗi giây
    // Tính toán tốc độ dựa trên số xung encoder
    rpm = (encoderCount / 330.0) * 60.0;  // Chuyển đổi thành RPM
    speed = (65 * PI ) / 1000 * (encoderCount / 330);
    Serial.print("Tốc độ động cơ: ");
    Serial.print(rpm);
    Serial.print(" RPM");
    Serial.print(" | Speed: ");
    Serial.println(abs(speed));
    
    encoderCount = 0;  // Reset bộ đếm encoder
    lastTime = currentTime;
  }
}