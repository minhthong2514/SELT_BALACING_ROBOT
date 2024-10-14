#define in1 26
#define in2 25
#define ena 12
#define in3 14
#define in4 27
#define enb 33
#define encoderA_m1 2
#define encoderB_m1 15
#define encoderA_m2 34
#define encoderB_m2 35

float setpoint = 200;
float kp = 4.5;
float ki = 0.5;
float kd = 0.0;

const int freq = 1000;
const int resolution = 8;

unsigned long lastTime, prevTime, currentTime;
volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;
float error1, error2, last_rpm, current_rpm;

void IRAM_ATTR encoder_motor1() {
  int stateA1 = digitalRead(encoderA_m1);
  int stateB1 = digitalRead(encoderB_m1);
  if (stateA1 == HIGH) {
    if (stateB1 == LOW) {
      encoderCount1++;
    } else {
      encoderCount1--;
    }
  } else {
    if (stateB1 == LOW) {
      encoderCount1--;
    } else {
      encoderCount1++;
    }
  }
}

void IRAM_ATTR encoder_motor2() {
  int stateA2 = digitalRead(encoderA_m2);
  int stateB2 = digitalRead(encoderB_m2);
  if (stateA2 == HIGH) {
    if (stateB2 == LOW) {
      encoderCount2++;
    } else {
      encoderCount2--;
    }
  } else {
    if (stateB2 == LOW) {
      encoderCount2--;
    } else {
      encoderCount2++;
    }
  }
}

void PID4MOTOR1() {
  currentTime = millis();
  unsigned long dt = currentTime - lastTime;
  if (dt >= 10) {
    float rpm1 = (encoderCount1 / 330.0) * 60.0 * (1000.0 / dt);
    float rpm2 = (encoderCount2 / 330.0) * 60.0 * (1000.0 / dt);
    // float speed = (65 / 1000 * PI) * (encoderCount1 / 330);

    float error = setpoint - current_rpm;
    float alpha = 2 * dt * kp + ki * dt * dt + 2 * kd;
    float beta = dt * dt * ki - 4 * kd - 2 * dt * kp;
    float gamma = 2 * kd;

    current_rpm = (alpha * error + beta * error1 + gamma * error2 + 2 * dt * last_rpm) / (2 * dt);
    error2 = error1;
    error1 = error;
    last_rpm = current_rpm;
  
    current_rpm = constrain(current_rpm, 0 ,255);

    if (current_rpm > 0){
      forward(int(current_rpm));
    }
    else {
      backward(int(-current_rpm));
    }
    // else {
    //   stop();
    // }
    Serial.print(rpm1);
    Serial.print(",");
    Serial.println(rpm2);

    encoderCount1 = 0;
    encoderCount2 = 0;
    lastTime = currentTime;
  }
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


  // Configurating encoder
  pinMode(encoderA_m1, INPUT);
  pinMode(encoderB_m1, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA_m1), encoder_motor1, RISING);
  pinMode(encoderA_m2, INPUT);
  pinMode(encoderB_m2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA_m2), encoder_motor2, RISING);

  stop();
}

void forward(int output) {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  ledcWrite(ena, output);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
  ledcWrite(enb, output);
}

void backward(int output) {
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  ledcWrite(ena, output);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);
  ledcWrite(enb, output);
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
  PID4MOTOR1();
}
