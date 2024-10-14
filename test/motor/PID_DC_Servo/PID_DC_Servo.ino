// Các hằng số PID
float Kp = 4.45;   // Hệ số P
float Ki = 0.432;  // Hệ số I
float Kd = 0.2;   // Hệ số D
float error1, error2, lastOutput;
// Biến PID
float previous_error = 0;
float integral = 0;
float actual_speed;  // Tốc độ thực tế (RPM)
int setpoint = 333;  // Tốc độ mong muốn (RPM)

// Biến tốc độ thực tế
volatile int encoder_ticks = 0;  // Đếm số xung từ encoder
int previous_ticks = 0;
unsigned long last_time = 0;

// Chân điều khiển cho ESP32
const int IN1 = 14;        // Chân IN1 của L298N
const int IN2 = 27;        // Chân IN2 của L298N
const int ENA = 12;        // Chân ENA (PWM) của L298N
const int encoderPin = 2;  // Chân nối với Encoder

// Thông số PWM cho ESP32
const int Freq = 1000;     // Tần số PWM
const int Resolution = 8;  // Độ phân giải PWM (8 bit)

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(encoderPin, INPUT);  // Cài đặt chân encoder là đầu vào với pull-up

  // Thiết lập kênh PWM cho ESP32
  ledcAttach(ENA, Freq, Resolution);  // Gán chân ENA vào kênh PWM 0
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  ledcWrite(ENA, 0);
  // Gán ngắt cho encoder
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderInterrupt, RISING);

  // Khởi tạo thời gian
  last_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  int dt = current_time - last_time;

  if (dt >= 10) {  // Cập nhật mỗi 10ms (tần số lấy mẫu là 10ms / 1lần = 100hz)
    // Tính toán tốc độ thực tế (RPM)
    actual_speed = (encoder_ticks / 330.0) * 60.0 * (1000.0 / dt);  // 330 pulses per revolution

    // Tính toán lỗi
    float error = setpoint - actual_speed;

    // Tính toán PID
    float alpha = 2 * dt * Kp + Ki * dt * dt + 2 * Kd;
    float beta = dt * dt * Ki - 4 * Kd - 2 * dt * Kp;
    float gamma = 2 * Kd;
    // Tín hiệu điều khiển (PID)
    float control_signal = (alpha * error + beta * error1 + gamma * error2 + 2 * dt * lastOutput) / (2 * dt);
    lastOutput = control_signal;
    error2 = error1;
    error1 = error;
    // Đảm bảo tín hiệu trong khoảng hợp lệ (0-255)
    control_signal = constrain(control_signal, 0, 255);
    // Điều khiển động cơ theo chiều và tốc độ
    if (control_signal > 0) {
      // Quay theo chiều thuận
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, control_signal);  // Ghi tín hiệu PWM
    } else {
      // Quay theo chiều ngược
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      ledcWrite(ENA, -control_signal);  // Ghi tín hiệu PWM
    }

    // Cập nhật các biến cho vòng lặp tiếp theo
    previous_error = error;
    last_time = current_time;
    encoder_ticks = 0;

    // In tốc độ thực tế và setpoint
    // Serial.print("Toc do hien tai: ");
    Serial.print(actual_speed);
    // Serial.print(" RPM, Setpoint: ");
    Serial.print(",");
    Serial.println(setpoint);
  }
}

void encoderInterrupt() {
  encoder_ticks++;  // Tăng biến đếm xung khi có tín hiệu từ encoder
}
