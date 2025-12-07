#define in1 14
#define in2 27
#define ena 12
#define in3 26
#define in4 25
#define enb 33
#define encoderA_m1 2
#define encoderB_m1 15
#define encoderA_m2 34
#define encoderB_m2 35

// Cấu hình cho động cơ 1
float setpoint = 100;  // Giá trị mục tiêu cho động cơ
volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;
float error1_1, error1_2, last_rpm1, output1;
float error2_1, error2_2, last_rpm2, output2;
float rpm1 = 0, rpm2 = 0;
// Cấu hình cho động cơ 2
// float setpoint2 = 200;  // Giá trị mục tiêu cho động cơ
const int freq = 1000;
const int resolution = 8;

unsigned long lastTime1, lastTime2, currentTime;
float rpm1Filt = 0;
float rpm1Prev = 0;
float rpm2Filt = 0;
float rpm2Prev = 0;

void IRAM_ATTR encoder_motor1() {
  int stateA1 = digitalRead(encoderA_m1);
  int stateB1 = digitalRead(encoderB_m1);
  if (stateA1 == HIGH) {
    encoderCount1 += (stateB1 == LOW) ? 1 : -1;
  } else {
    encoderCount1 += (stateB1 == LOW) ? -1 : 1;
  }
}

void IRAM_ATTR encoder_motor2() {
  int stateA2 = digitalRead(encoderA_m2);
  int stateB2 = digitalRead(encoderB_m2);
  if (stateA2 == HIGH) {
    encoderCount2 += (stateB2 == LOW) ? 1 : -1;
  } else {
    encoderCount2 += (stateB2 == LOW) ? -1 : 1;
  }
}

float PID4MOTOR1() {
  float kp1 = 12.85;
  float ki1 = 1.932;
  float kd1 = 0.1;
  currentTime = millis();
  unsigned long dt = currentTime - lastTime1;
  if (dt >= 1) {
    rpm1 = (encoderCount1 / 330.0) * 60.0 * (1000.0 / dt);
    rpm1Filt = 0.854 * rpm1Filt + 0.0728 * rpm1 + 0.0728 * rpm1Prev;
    rpm1Prev = rpm1;
    // Tính toán PID cho động cơ 1
    float error = setpoint - rpm1Filt;
    output1 = kp1 * error + ki1 * error1_1 + kd1 * (error - error1_1);
    output1 = constrain(output1, 0, 255);

    // Đặt lại encoderCount
    encoderCount1 = 0;
    lastTime1 = currentTime;
    error1_1 = error;
  }
  return output1;
}

float PID4MOTOR2() {
  float kp2 = 13.5;
  float ki2 = 2.32;
  float kd2 = 0.1;
  currentTime = millis();
  unsigned long dt = currentTime - lastTime2;
  if (dt >= 1) {
    rpm2 = (encoderCount2 / 330.0) * 60.0 * (1000.0 / dt);
    rpm2Filt = 0.854 * rpm2Filt + 0.0728 * rpm2 + 0.0728 * rpm2Prev;
    rpm2Prev = rpm2;
    // Tính toán PID cho động cơ 2
    float error = setpoint - rpm2Filt;
    output2 = kp2 * error + ki2 * error2_1 + kd2 * (error - error2_1);
    output2 = constrain(output2, 0, 255);

    // Đặt lại encoderCount
    encoderCount2 = 0;
    lastTime2 = currentTime;
    error2_1 = error;
  }
  return output2;
}

void setup() {
  Serial.begin(115200);

  // Cấu hình chân điều khiển động cơ
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);

  ledcAttach(ena, freq, resolution);
  ledcAttach(enb, freq, resolution);

  // Cấu hình encoder
  pinMode(encoderA_m1, INPUT);
  pinMode(encoderB_m1, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA_m1), encoder_motor1, RISING);

  pinMode(encoderA_m2, INPUT);
  pinMode(encoderB_m2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA_m2), encoder_motor2, RISING);

  stop();
}

void forward1(int output1) {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  ledcWrite(ena, abs(output1));
}

void forward2(int output2) {
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
  ledcWrite(enb, abs(output2));
}
void backward() {
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  ledcWrite(ena, output1);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);
  ledcWrite(enb, output2);
}

void stop() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  ledcWrite(ena, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
  ledcWrite(enb, 0);
}

void loop() {
  float pwm1 = PID4MOTOR1();  // Gọi hàm PID cho động cơ 1
  float pwm2 = PID4MOTOR2();  // Gọi hàm PID cho động cơ 2
  forward1(int(pwm1));
  forward2(int(pwm2));
  Serial.print(setpoint);
  Serial.print(" | ");
  Serial.print(rpm1);
  Serial.print(" | ");
  Serial.println(rpm2);
}
