#include "stmpu6050.h"
SMPU6050 mpu6050;


float currentTime, previousTime, currentAngle;
float offset = 0.0;
float alpha = 0.98;

// Thông số PID
float Kp = 28.5, Ki = 12.0, Kd = 0.05;
float setpoint =  -5.0;
float output , error , previousError, integral, derivative;
float outputL, outputR, Motor_L, Motor_R;


void setup() {
  mpu6050.init(0x68);
  Serial.begin(115200);               //Khai báo Serial
  // Cấu hình chân điều khiển động cơ
  pinMode(3, OUTPUT);                                          
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);

  // Tắt động cơ ban đầu
  analogWrite(3, LOW);
  analogWrite(5, LOW);
  analogWrite(6, LOW);
  analogWrite(9, LOW);

  calibrate();
  delay(500);

}

void loop() {
  currentTime = millis();
  float AngleY = mpu6050.getYAngle();

  //Serial.println(AngleY);

  // Tính toán PIDa
  float dt = (currentTime - previousTime) / 1000.0;
  currentAngle = round(AngleY);
  error = currentAngle - setpoint;
  integral += error * dt;
  derivative = (error - previousError) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  previousTime = currentTime;
  previousError = error;

  // Giới hạn giá trị output trong khoảng từ -255 đến 255
  output = constrain(output, -255, 255);
  // Nếu output trong khoảng -11 đến 11 thì cho output bằng 0 vì output lúc này là số âm
  if (output >= -11 && output <= 11) { output = 0; } 
  outputL = output;
  outputR = output;
  if (output > 0) {
    Motor_L = 275 - (1 / (outputL + 9)) * 5500;  // output = 1 ==> Motor_L = -275
    Motor_R = 275 - (1 / (outputR + 9)) * 5500;  // output = 10 ==> Motor_L =  -14.47
                                                 // output = 11 ==> Motor_L = 0
                                                 // output = 255  ==> Motor_L = 254.16
    if (Motor_L > 100) { Motor_L = 255;}
    if (Motor_R > 100) {Motor_R = 255;}
    moveBackward();  
  }                                              


                                                  
  else if (output < 0) {
    Motor_L = -275 - (1 / (outputL - 9)) * 5500;  // output = -1 ==> Motor_L = 275
    Motor_R = -275 - (1 / (outputR - 9)) * 5500;  // output = -10 ==> Motor_L = 14.47
                                                  // output = -11 ==> Motor_L = 0
                                                  // output = -255 ==> Motor_L = -254.16
    if (Motor_L < -100) { Motor_L = -255;}
    if (Motor_R < -100) {Motor_R = -255;}
    moveForward();
  }                                               

  else {
    output = 0;
    Motor_L =0;
    Motor_R =0;
    stopMotors();
  }
  Serial.print("Angle:");
  Serial.print(currentAngle);
  Serial.print(" | Motor_L: ");
  Serial.print(Motor_L);
  Serial.print(" | Motor_R:");
  Serial.println(Motor_R);

}

void moveForward() {
  analogWrite(3, -outputL);  // Đặt PWM cho chân 3
  analogWrite(5, 0);         // Đặt PWM cho chân 5 là 0
  analogWrite(6, -outputR);  // Đặt PWM cho chân 6
  analogWrite(9, 0);         // Đặt PWM cho chân 9 là 0
}

// Hàm điều khiển robot di chuyển về phía sau
void moveBackward() {
  analogWrite(3, 0);        // Đặt PWM cho chân 3 là 0
  analogWrite(5, outputL);  // Đặt PWM cho chân 5
  analogWrite(6, 0);        // Đặt PWM cho chân 6 là 0
  analogWrite(9, outputR);  // Đặt PWM cho chân 9
}

// Hàm dừng động cơ
void stopMotors() {
  analogWrite(3, 0);  // Đặt PWM cho chân 3 là 0
  analogWrite(5, 0);  // Đặt PWM cho chân 5 là 0
  analogWrite(6, 0);  // Đặt PWM cho chân 6 là 0
  analogWrite(9, 0);  // Đặt PWM cho chân 9 là 0
}

void calibrate(){
  int c = 0;
  while (c < 200 ){
    float angle = mpu6050.getYAngle();
    currentAngle += angle;
    c++;
  }
  currentAngle /= 200;
}
