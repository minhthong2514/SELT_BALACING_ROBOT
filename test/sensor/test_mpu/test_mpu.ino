#include <Wire.h>


const int MPU = 0x68; //Khai báo địa chỉ cho MPU
// Khai báo biến cho thời gian lấy mẫu
unsigned long currentTime, prevTime; 
double dt;

// Khai báo biến cho MPU
int16_t accX, accY, accZ;
float accelX, accelY, accelZ, accelAngleX = 0, accelAngleY = 0;
int16_t gyX, gyY, gyZ;
float gyroX, gyroY, gyroZ, gyroAngleX = 0, gyroAngleY = 0;
float roll, pitch, yaw;
float angleRoll, anglePitch, angleYaw;
float currentValue, prevValue, errorValue;
// Khai báo các biến cho tính toán sai số
int c;
float accelX_Error = 0, accelY_Error = 0, accelZ_Error = 0;
float gyroX_Error = 0, gyroY_Error = 0, gyroZ_Error = 0;
float accelX_Total, accelY_Total, accelZ_Total;
float gyroX_Total, gyroY_Total, gyroZ_Total;


void Error_IMU(){
  c = 0;
  int count = 2000;
  while (c < count) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gyX = (Wire.read() << 8) | Wire.read();
    gyY = (Wire.read() << 8) | Wire.read();
    gyZ = (Wire.read() << 8) | Wire.read();
    gyroX_Total += gyX;
    gyroY_Total += gyY;
    gyroZ_Total += gyZ;
    c++;
  }
  gyroX_Error = -gyroX_Total * 1.0 / count;
  gyroY_Error = -gyroY_Total * 1.0 / count;
  gyroZ_Error = -gyroZ_Total * 1.0 / count;

}


void setup() {
  Serial.begin(115200);
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
  delay(20);
  currentTime = micros();
}

void loop() {
  prevTime = currentTime;
  currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0;
  
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
  // Tính toán GÓC cho accel và gyro
  accelAngleX = (atan(accelY / sqrt(pow(accelX,2) + pow(accelZ,2))) * 180 / PI);
  accelAngleY = (atan(-1 * accelX / sqrt(pow(accelY,2) + pow(accelZ,2))) * 180 / PI);

  gyroX = (gyX + gyroX_Error) / 131.0;
  gyroY = (gyY + gyroY_Error) / 131.0;
  gyroZ = (gyZ + gyroZ_Error) / 131.0;

  // gyroAngleX = gyroAngleX + gyroX * dt;
  // gyroAngleY = gyroAngleY + gyroY * dt;
  // Tính toán góc Roll, Pitch, Yaw
  // float alpha = 0.98;
  angleRoll = 0.02 * accelAngleX + 0.98 * (angleRoll + gyroX * dt);
  anglePitch = 0.02 * accelAngleY + 0.98 * (anglePitch + gyroY * dt);
  angleYaw += gyroZ * dt;
  // float rollf = kalmanX.getAngle(angleRoll, gyroX,dt);
  // float pitchf = kalmanY.getAngle(anglePitch, gyroY,dt);
    // Đưa lên cổng serial
  Serial.print("Roll: ");
  Serial.print(angleRoll);
  Serial.print("  | Pitch: ");
  Serial.print(anglePitch);
  Serial.print("  | Yaw: ");
  Serial.println(angleYaw);
}
