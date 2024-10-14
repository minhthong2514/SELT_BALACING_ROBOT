#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;

// Biến lưu trữ dữ liệu đọc từ cảm biến
int16_t accX, accY, accZ;
int16_t gyX, gyY, gyZ;
float gyroAngleX = 0, gyroAngleY = 0;
float currentTime, previousTime;
float currentAngle;
float alpha = 0.98;

// Thông số PID
float Kp = 20, Ki = 10.0, Kd = 0.1;
float setpoint = 1.0; 
float output = 0.0, error = 0.0, previousError = 0.0, integral = 0.0, derivative = 0.0;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Khởi tạo MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  // Kiểm tra kết nối cảm biến
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
  // Kiểm tra nếu khởi tạo không thành công
  if (!mpu.testConnection()) {
      while(1) {
          Serial.println("Failed to connect to MPU6050");
          delay(1000);
      }
  }

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
    float AccelAngleX = (atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI );
    float AccelAngleY = (atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI ); 

    // Cập nhật góc nghiêng cho gyro
    float dt = (currentTime - previousTime) / 1000.0; 

    gyroAngleX += gyroX * dt; 
    gyroAngleY += gyroY * dt;  
    
    // Tính toán góc nghiêng kết hợp từ con quay hồi chuyển và gia tốc kế
    float roll = (1 - alpha) * gyroAngleX + alpha * AccelAngleX;
    float pitch = (1 - alpha)  * gyroAngleY + alpha * AccelAngleY;

    // Tính toán PID
    currentAngle = pitch;  // Dùng giá trị pitch để đại diện cho góc hiện tại của xe
    error = currentAngle - setpoint;  // Tính toán lỗi
    integral += error * dt;  // Tính toán tích phân lỗi
    derivative = (error - previousError) / dt;  // Tính toán đạo hàm lỗi
    output = Kp * error + Ki * integral + Kd * derivative;  // Tính toán output PID
    // Giới hạn giá trị output trong khoảng từ -255 đến 255

    //output = constrain(output, -255, 255);
    if (output < -255) {output = -255;}
    if (output > 255) {output = 255;}
    Serial.println(output);


    // Điều khiển động cơ
    if (currentAngle > -60 && currentAngle < 60){
      if (output < 0) {
        
          moveForward(-output);
      } else if (output > 0) {
          
          moveBackward(output);  // Dùng giá trị dương cho backward
      }
    } 
    else {
        stopMotors();
    }

    previousTime = currentTime;
    previousError = error;
    
    // Hiển thị kết quả trên Serial Monitor
    // Serial.print("PWM: ");
    // Serial.print(output);
    // Serial.print(" | Angle: ");
    // Serial.println(currentAngle);
}

void moveForward(int pwm) {
  analogWrite(3, pwm);  // Đặt PWM cho chân 3
  analogWrite(5, 0);    // Đặt PWM cho chân 5 là 0
  analogWrite(6, pwm);  // Đặt PWM cho chân 6
  analogWrite(9, 0);    // Đặt PWM cho chân 9 là 0
}

// Hàm điều khiển robot di chuyển về phía sau
void moveBackward(int pwm) {
  analogWrite(3, 0);    // Đặt PWM cho chân 3 là 0
  analogWrite(5, pwm);  // Đặt PWM cho chân 5
  analogWrite(6, 0);    // Đặt PWM cho chân 6 là 0
  analogWrite(9, pwm);  // Đặt PWM cho chân 9
}

// Hàm dừng động cơ
void stopMotors() {
  analogWrite(3, 0);  // Đặt PWM cho chân 3 là 0
  analogWrite(5, 0);  // Đặt PWM cho chân 5 là 0
  analogWrite(6, 0);  // Đặt PWM cho chân 6 là 0
  analogWrite(9, 0);  // Đặt PWM cho chân 9 là 0
}
