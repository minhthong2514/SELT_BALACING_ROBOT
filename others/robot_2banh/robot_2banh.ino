#include "MPU6050.h"
#include "Wire.h"
#include <SimpleKalmanFilter.h>

MPU6050 mpu;                // Khởi tạo phương thức cho thư viện MPU6050.h

//***********************   Kalman Filter   *****************
const float measurementNoise = 300;
const float processNoise = 5;
const float initialValuze = 5;
SimpleKalmanFilter AngleY_KalmanFilter(measurementNoise, processNoise, initialValue);

// Tín hiệu ngắt từ MPU6050
volatile bool mpuInterrupt = false;

// Biến lưu trữ dữ liệu đọc từ cảm biến
int16_t accX, accY, accZ;
int16_t gyX, gyY, gyZ;
float gyroAngleX = 0, gyroAngleY = 0;
float currentTime, previousTime;
float offset = 2.0;
float alpha = 0.98;

// Thông số PID
float Kp = 16.2, Ki = 9.22, Kd = 0.0;
float setpoint = 0.0;
float output , error , previousError, integral, derivative;
float outputL, outputR, Motor_L, Motor_R;


// Hàm xử lý ngắt
void DataReady() {
  mpuInterrupt = true;
}


void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Khởi tạo MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  delay(1000);
  // Kiểm tra kết nối cảm biến
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Kiểm tra nếu khởi tạo không thành công
  if (!mpu.testConnection()) {
    while (1) {
      Serial.println("Failed to connect to MPU6050");
      delay(1000);
    }
  }

  // // Cấu hình chân ngắt
  pinMode(2, INPUT); // Thay đổi theo chân ngắt mà bạn sử dụng
  attachInterrupt(0, DataReady, RISING); // Cấu hình ngắt

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
}

void loop() {
    currentTime = millis();

    // Đọc dữ liệu từ MPU6050
    mpu.getMotion6(&accX, &accY, &accZ, &gyX, &gyY, &gyZ);

    float accelX = accX / 16384.0;
    float accelY = accY / 16384.0;
    float accelZ = accZ / 16384.0;
    float gyroX = gyX / 131.0;
    float gyroY = gyY / 131.0;
    float gyroZ = gyZ / 131.0;

    // Tính góc AccelX và AccelY
    //float AccelAngleX = (atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI);
    float AccelAngleY = (atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI);     // Sử dụng accelY để không bị trôi dữ liệu gyro
    float currentAngle = AngleY_KalmanFilter.updateEstimate(AccelAngleY);                           // Áp dụng bộ lọc Kalman cho góc accelY
    // Cập nhật góc nghiêng cho gyro
    float dt = (currentTime - previousTime) / 1000.0;

    //gyroAngleX += gyroX * dt;
    //gyroAngleY += gyroY * dt;

    // Tính toán góc nghiêng kết hợp từ con quay hồi chuyển và gia tốc kế
    //float roll = (1 - alpha) * gyroAngleX + alpha * AccelAngleX;
    //float pitch = (1 - alpha) * gyroAngleY + alpha * AccelAngleY;

    // Tính toán PID
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
      moveBackward();                              // output = 11 ==> Motor_L = 0
    }                                              // output = 255  ==> Motor_L = 254.16
                                                  
    else if (output < 0) {
      Motor_L = -275 - (1 / (outputL - 9)) * 5500;  // output = -1 ==> Motor_L = 275
      Motor_R = -275 - (1 / (outputR - 9)) * 5500;  // output = -10 ==> Motor_L = 14.47
      moveForward();                                // output = -11 ==> Motor_L = 0
    }                                               // output = -255 ==> Motor_L = -254.16

    else {
      output = 0;
      Motor_L =0;
      Motor_R =0;
      stopMotors();
    }

    Serial.print("Pitch:");
    Serial.print(currentAngle);
    Serial.print(" | Motor_L: ");
    Serial.print(Motor_L);
    Serial.print(" | Motor_R:");
    Serial.println(Motor_R);

    // moveForward();
    // moveBackward();

    // Hiển thị kết quả trên Serial Monitor
    // Serial.print("PWM: ");
    // Serial.print(output);
    // Serial.print(" | Angle: ");
    // Serial.println(currentAngle);
    
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
