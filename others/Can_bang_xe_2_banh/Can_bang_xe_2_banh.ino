#include <Wire.h>
#include "Kalman.h"
#include <LiquidCrystal_I2C.h>

Kalman kalmanX, kalmanY, kalmanZ;

const int MPU = 0x68;                   //Khai báo địa chỉ cho MPU


LiquidCrystal_I2C lcd(0x27, 16, 2);     //Khai báo địa chỉ i2c

// Khai báo biến cho thời gian lấy mẫu
float currentTime, prevTime, dt;
float currentValue= 0.0, prevValue= 0.0, errorValue = 0.0;
float offset = 0.0;


// Khai báo biến cho MPU
int16_t accX, accY, accZ;
float accelX, accelY, accelZ, accelAngleX = 0, accelAngleY = 0;
int16_t gyX, gyY, gyZ;
float gyroX, gyroY, gyroZ, gyroAngleX = 0, gyroAngleY = 0;
float roll = 0, pitch = 0, yaw = 0;
float angleRoll, anglePitch, angleYaw;

// Khai báo các biến cho tính toán sai số
int c;
float accelX_Error = 0, accelY_Error = 0, accelZ_Error = 0;
float gyroX_Error = 0, gyroY_Error = 0, gyroZ_Error = 0;

// Thông số PID
float Kp = , Ki = 12.0, Kd = 0.05;
float setpoint = 0.0;
float output , error , prevError, integral, derivative;
float outputL, outputR, Motor_L, Motor_R;


void setup() {
  Serial.begin(115200);               //Khai báo Serial

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initial MPU6050...");

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

  // Sensor
  Wire.begin();                  // Tạo kết nối I2C
  Wire.setClock(400000);         // Set tốc độ xung clock ic2 là 400KHz
  Wire.beginTransmission(MPU);   // Đọc địa chỉ của MPU
  Wire.write(0x6B);              // Thanh ghi nguồn 6B
  Wire.write(0x00);              // Đặt thanh ghi 6B về 0 để đánh thức cảm biến
  // Wire.write(0x19);           // Thanh ghi SMPLRT_DIV
  // Wire.write(0x00);           // Đặt giá trị 0 để lấy mẫu ở tần số cao nhất
  Wire.endTransmission(true);    // Kết thức kết nối I2C

  Error_IMU();
  // delay(2000);
  lcd.clear();
  currentTime = micros();
}

void loop() {
  prevTime = currentTime;
  currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0;
  Read_accel();
  Read_gyro();

  // Tính toán GÓC cho accel và gyro
  accelAngleX = (atan(accelY / sqrt(pow(accelX,2) + pow(accelZ,2))) * 180 / PI) - accelX_Error;
  accelAngleY = (atan(-1 * accelX / sqrt(pow(accelY,2) + pow(accelZ,2))) * 180 / PI) - accelY_Error;

  gyroX -= gyroX_Error;
  gyroY -= gyroY_Error;
  gyroZ -= gyroZ_Error;

  gyroAngleX = gyroAngleX + gyroX * dt;
  gyroAngleY = gyroAngleY + gyroY * dt;

  // Tính toán góc Roll, Pitch, Yaw
  float alpha = 0.95;
  angleRoll = (1 - alpha) * accelAngleX + (1 * alpha) * gyroAngleX;
  anglePitch = (1 - alpha) * accelAngleY + (1 * alpha) * gyroAngleY;
  angleYaw += gyroZ * dt;
  float rollf = kalmanX.getAngle(angleRoll, gyroX,dt);
  float pitch = kalmanY.getAngle(anglePitch, gyroY,dt);
  roll = round(rollf);
  //pitch = round(pitchf);
  yaw = round(angleYaw);

  // Tính giá trị sai số giữa các lần đọc cảm biến để xử lý driff
  currentValue = pitch;
  errorValue += currentValue - prevValue;

  
  if (pitch - errorValue != 0) {pitch -= errorValue;}         // Điều kiện để xóa bỏ sai số
  // pitch -= errorValue;
  prevValue = currentValue;
  // Tính toán PID
  error = pitch - setpoint + offset;
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

  if (output > 0) {
    Motor_L = 275 - (1 / (outputL + 9)) * 5500;  // output = 1 ==> Motor_L = -275
    Motor_R = 275 - (1 / (outputR + 9)) * 5500;  // output = 10 ==> Motor_L =  -14.47
    //moveBackward();                            // output = 11 ==> Motor_L = 0
                                                 // output = 255  ==> Motor_L = 254.16
    if (Motor_L > 100) { Motor_L = 255;}
    if (Motor_R > 100) { Motor_R = 255;}
    moveBackward();
  }                                              
                                                  
  else if (output < 0) {
    Motor_L = -275 - (1 / (outputL - 9)) * 5500;  // output = -1 ==> Motor_L = 275
    Motor_R = -275 - (1 / (outputR - 9)) * 5500;  // output = -10 ==> Motor_L = 14.47
    //moveForward();                              // output = -11 ==> Motor_L = 0
                                                  // output = -255 ==> Motor_L = -254.16
    if (Motor_L < -100) { Motor_L = -255;}
    if (Motor_R < -100) { Motor_R = -255;}
    moveForward();
  }                                               

  else {
    output = 0;
    Motor_L =0;
    Motor_R =0;
    stopMotors();
  }
  Serial.print("Angle:");
  Serial.print(pitch);
  Serial.print(" | Motor_L: ");
  Serial.print(Motor_L);
  Serial.print(" | Motor_R: ");
  Serial.println(Motor_R);
  lcd.setCursor(0, 0);
  lcd.print("Ngo An Thuyen");
  lcd.setCursor(0, 1);
  lcd.print("Pitch: ");
  lcd.print(pitch);

}
void Read_accel(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);        // 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  accX = (Wire.read() << 8) | Wire.read();
  accY = (Wire.read() << 8) | Wire.read();
  accZ = (Wire.read() << 8) | Wire.read();
  accelX = accX / 16384.0;
  accelY = accY / 16384.0;
  accelZ = accZ / 16384.0;
}

void Read_gyro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  gyX = (Wire.read() << 8) | Wire.read();
  gyY = (Wire.read() << 8) | Wire.read();
  gyZ = (Wire.read() << 8) | Wire.read();
  gyroX = gyX / 131.0;
  gyroY = gyY / 131.0;
  gyroZ = gyZ / 131.0;
}
void Error_IMU(){
  c = 0;
  while (c < 2000) {
    Read_accel();
    accelX_Error += accelX;
    accelY_Error += accelY;
    accelZ_Error += accelZ;
    Read_gyro();
    gyroX_Error += gyroX;
    gyroY_Error += gyroY;
    gyroZ_Error += gyroZ;
    c++;
    //delay(1);
  }
  accelX_Error = accelX_Error / 2000;
  accelY_Error = accelY_Error / 2000;
  accelZ_Error = accelZ_Error / 2000;
  gyroX_Error = gyroX_Error / 2000;
  gyroY_Error = gyroY_Error / 2000;
  gyroZ_Error = gyroZ_Error / 2000;
}

// Hàm điều khiển robot di chuyển về trước
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