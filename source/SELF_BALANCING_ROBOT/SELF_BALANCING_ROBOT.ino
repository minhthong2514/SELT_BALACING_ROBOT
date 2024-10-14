#include <Wire.h>
#include "Kalman.h"
#define ena 12
#define in1 25
#define in2 26
#define in3 27
#define in4 14
#define enb 33

Kalman kalman;
const int freq = 100;
const int resolution = 8;

const int MPU = 0x68;  //Khai báo địa chỉ cho MPU

// Khai báo biến cho thời gian lấy mẫu
unsigned long currentTime, prevTime;
double dt;

// Khai báo biến cho MPU
int16_t accXRaw, accYRaw, accZRaw;
int16_t gyroXRaw, gyroYRaw, gyroZRaw;
float accX, accY, accZ, accelX, accelY;
float gyroX, gyroY, gyroZ;
float roll, pitch, yaw;

// Khai báo các biến cho tính toán sai số
int c;
float gyroX_Offset, gyroY_Offset, gyroZ_Offset;
float gyroX_Total, gyroY_Total, gyroZ_Total;

// Thông số PID
float Kp = 125.5, Ki = 10.2, Kd = 2.3;  //Kp = 125.5, Ki=10.2, Kd=2.3
float setpoint = -3.5;
float offset = -0.5;
float output, error, prevError, integral, derivative;
float outputL, outputR, Motor_L, Motor_R;


void setup() {
  Serial.begin(115200);
  // Sensor
  Wire.begin();                 // Tạo kết nối I2C
  Wire.setClock(400000);        // Set tốc độ xung clock ic2 là 400KHz
  Wire.beginTransmission(MPU);  // Đọc địa chỉ của MPU
  Wire.write(0x6B);             // Thanh ghi nguồn 6B
  Wire.write(0x00);             // Đặt thanh ghi 6B về 0 để đánh thức cảm biến
  Wire.endTransmission(true);   // Kết thức kết nối I2C

  // Cấu hình chân điều khiển động cơ
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);

  ledcAttach(ena, freq, resolution);
  ledcAttach(enb, freq, resolution);
  kalman.setAngle(0);
  // Tắt động cơ ban đầu
  stopMotors();
  Calibrate_IMU();
  // delay(100);
  currentTime = micros();
}

void loop() {
  prevTime = currentTime;
  currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  accXRaw = (Wire.read() << 8) | Wire.read();
  accYRaw = (Wire.read() << 8) | Wire.read();
  accZRaw = (Wire.read() << 8) | Wire.read();

  accX = accXRaw / 16384.0;
  accY = accYRaw / 16384.0;
  accZ = accZRaw / 16384.0;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  gyroXRaw = (Wire.read() << 8) | Wire.read();
  gyroYRaw = (Wire.read() << 8) | Wire.read();
  gyroZRaw = (Wire.read() << 8) | Wire.read();

  gyroX = gyroXRaw / 131.0;
  gyroY = gyroYRaw / 131.0;
  gyroZ = gyroZRaw / 131.0;

  // Tính toán GÓC cho accel và gyro
  accelX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI;
  accelY = atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI;

  gyroX = (gyroXRaw + gyroX_Offset) / 131.0;
  gyroY = (gyroYRaw + gyroY_Offset) / 131.0;
  gyroZ = (gyroZRaw + gyroZ_Offset) / 131.0;

  roll = 0.98 * (roll + gyroX * dt) + 0.02 * accelX;
  pitch = 0.98 * (pitch + gyroY * dt) + 0.02 * accelY;
  float angle = kalman.getAngle(pitch, gyroY, dt);
  yaw += gyroZ * dt;

  // Tính toán PID
  error = angle - setpoint + offset;  //offset =2.5
  integral += error * dt;
  derivative = (error - prevError) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  prevTime = currentTime;
  prevError = error;


  // Giới hạn giá trị output trong khoảng từ -255 đến 255
  output = constrain(output, -255, 255);

  // Nếu output trong khoảng -11 đến 11 thì cho output bằng 0 vì output lúc này là số âm
  if (output >= -11 && output <= 11) { output = 0; }
  outputL = output;
  outputR = output;
  if (angle > -40 && angle < 40) {
    if (output > 0) {
      Motor_L = 275 - (1 / (outputL + 9)) * 5500;  // output = 1 ==> Motor_L = -275
      Motor_R = 275 - (1 / (outputR + 9)) * 5500;  // output = 10 ==> Motor_L =  -14.47
      //moveBackward();                            // output = 11 ==> Motor_L = 0
      // output = 255  ==> Motor_L = 254.16
      // if (Motor_L > 150) { Motor_L = 255; }
      // if (Motor_R > 150) { Motor_R = 255; }
      moveBackward();
    }

    else if (output < 0) {
      Motor_L = -275 - (1 / (outputL - 9)) * 5500;  // output = -1 ==> Motor_L = 275
      Motor_R = -275 - (1 / (outputR - 9)) * 5500;  // output = -10 ==> Motor_L = 14.47
      //moveForward();                              // output = -11 ==> Motor_L = 0
      // output = -255 ==> Motor_L = -254.16
      // if (Motor_L < -150) { Motor_L = -255; }
      // if (Motor_R < -150) { Motor_R = -255; }
      moveForward();
    }
  } else {
    stopMotors();
  }

  Serial.print("Pitch:");
  Serial.print(pitch);
  Serial.print(" | Angle:");
  Serial.print(angle);
  Serial.print(" | Motor_L: ");
  Serial.print(Motor_L);
  Serial.print(" | Motor_R: ");
  Serial.println(Motor_R);
}

// Hàm điều khiển robot di chuyển về trước
void moveForward() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  ledcWrite(ena, -outputL);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
  ledcWrite(enb, -outputR);
}

// Hàm điều khiển robot di chuyển về phía sau
void moveBackward() {
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  ledcWrite(ena, outputL);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);
  ledcWrite(enb, outputR);
}

// Hàm dừng động cơ
void stopMotors() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}

void Calibrate_IMU() {
  c = 0;
  int count = 2000;
  while (c < count) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gyroXRaw = (Wire.read() << 8) | Wire.read();
    gyroYRaw = (Wire.read() << 8) | Wire.read();
    gyroZRaw = (Wire.read() << 8) | Wire.read();

    gyroX_Total += gyroXRaw;
    gyroY_Total += gyroYRaw;
    gyroZ_Total += gyroZRaw;
    c++;
  }
  gyroX_Offset = -gyroX_Total * 1.0 / count;
  gyroY_Offset = -gyroY_Total * 1.0 / count;
  gyroZ_Offset = -gyroZ_Total * 1.0 / count;
}