#include <Wire.h>
#include "Kalman.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#define ena 12
#define in1 25
#define in2 26
#define in3 27
#define in4 14
#define enb 33
#define encoderA_m1 2
#define encoderB_m1 15
#define encoderA_m2 34
#define encoderB_m2 35
#define MAX_INTEGRAL 500  // Giá trị giới hạn tích phân

//-----------------------------Wifi-------------------------------------//
// Thông tin WiFi và MQTT Broker
const char* ssid = "";     // Your ssid wifi
const char* password = "";    // Your password wifi
const char* mqtt_server = "";  // Your mqtt server link
const int mqtt_port = 8883;
const char* mqtt_username = "";     // Your mqtt username
const char* mqtt_password = "";     // Your mqtt password

WiFiClientSecure espClient;
PubSubClient client(espClient);
//-----------------------------Wifi-------------------------------------//


//-----------------------------MPU6050----------------------------------//
Kalman kalman;
const int MPU = 0x68;  //Khai báo địa chỉ cho MPU

// Khai báo biến cho MPU
int16_t accXRaw, accYRaw, accZRaw;
int16_t gyroXRaw, gyroYRaw, gyroZRaw;
float accX, accY, accZ, accelX, accelY;
float gyroX, gyroY, gyroZ;
float roll, pitch, yaw;
float angle;

// Khai báo các biến cho tính toán sai số
int c;
float gyroX_Offset, gyroY_Offset, gyroZ_Offset;
float gyroX_Total, gyroY_Total, gyroZ_Total;
//-----------------------------MPU6050-----------------------------------//

//-----------------------------MOTOR AND ENCODER-------------------------//
const int freq = 100;
const int resolution = 8;

// Khai báo biến ngắt ngoài để đọc encoder
volatile int encoderCountLeft = 0;
volatile int encoderCountRight = 0;
unsigned long currentTimeEncoderLeft, previousTimeEncoderLeft;
unsigned long currentTimeEncoderRight, previousTimeEncoderRight;
float RPM_Left, RPM_Right;
//-----------------------------MOTOR AND ENCODER-------------------------//


//------------------------------PID--------------------------------------//
// Thông số PID cho cân bằng
unsigned long currentTime, prevTime;
double dt;
float Kp = 100.5, Ki = 350.0, Kd = 4.2;  //Kp = 125.5, Ki=10.2, Kd=2.3
float setpoint = -3.0;                   // -3.5
float offset = -0.5;                     // -0.5
float errorL, errorR, integralL, integralR, derivativeL, derivativeR;
float prevErrorL, prevErrorR;
float outputL, outputR;

// Thông số cho bộ PID điều khiển động cơ trái
unsigned long currentTimeMotor_L, prevTimeMotor_L;
float setpoint_L, outputPID_L, errorPID_L = 0.0;
float last_error_L = 0.0, integral_L = 0.0;

// Thông số cho bộ PID điều khiển động cơ phải
unsigned long currentTimeMotor_R, prevTimeMotor_R;
float setpoint_R, outputPID_R, errorPID_R = 0.0;
float last_error_R = 0.0, integral_R = 0.0;

// Biến PID cho động cơ 1
float setpoint_motor1 = 100.0;
float kp_motor1 = 10.85, ki_motor1 = 1.932, kd_motor1 = 0.1;
float input_motor1 = 0.0;
float output_motor1 = 0.0;
float last_error_motor1 = 0.0;
float last_time_motor1 = 0.0;
float integral_motor1 = 0.0;

// Biến PID cho động cơ 2
float setpoint_motor2 = 100.0;
float kp_motor2 = 10.5, ki_motor2 = 2.32, kd_motor2 = 0.1;
float input_motor2 = 0.0;
float output_motor2 = 0.0;
float last_error_motor2 = 0.0;
float last_time_motor2 = 0.0;
float integral_motor2 = 0.0;

// Tổng PWM cuối cùng đưa vào hàm di chuyển để điều khiển xe
float totalPWM_Left = 0.0;
float totalPWM_Right = 0.0;
volatile float pwmLeft = 0.0;
volatile float pwmRight = 0.0;

//------------------------------PID--------------------------------------//

// // Định nghĩa Queue để lưu trữ giá trị RPM
QueueHandle_t dataQueue;
QueueHandle_t xQueuePWM;

struct SensorData {
  float angle;      // Giá trị góc
  float RPM_Left;   // RPM của động cơ trái
  float RPM_Right;  // RPM của động cơ phải
};

// Task Balance để giữ cân bằng xe và điều khiển động cơ
void taskBalance(void* pvParameters) {
  float pwmData[2];
  while (true) {
    // Nhận giá trị PWM từ hàng đợi
    if (xQueueReceive(xQueuePWM, &pwmData, portMAX_DELAY) == pdPASS) {
      pwmLeft = pwmData[0];
      pwmRight = pwmData[1];
    }
    ControlBalancingRobot();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// Task PID này chỉ tính toán giá trị PWM cho tốc độ động cơ
void taskPID(void* pvParameters) {
  float pwmData[2];
  while (1) {
    // Tính toán PID điều khiển tốc độ động cơ trái và phải
    pwmData[0] = computePID4MOTOR_LEFT(setpoint_motor1, input_motor1, kp_motor1, ki_motor1, kd_motor1, &last_error_L, &integral_L, &prevTimeMotor_L);
    pwmData[1] = computePID4MOTOR_RIGHT(setpoint_motor2, input_motor2, kp_motor2, ki_motor2, kd_motor2, &last_error_R, &integral_R, &prevTimeMotor_R);

    pwmData[0] = constrain(pwmData[0], -255, 255);
    pwmData[1] = constrain(pwmData[1], -255, 255);

    // Gửi giá trị PWM qua hàng đợi
    xQueueSend(xQueuePWM, &pwmData, portMAX_DELAY);

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay giữa các lần gửi
  }
}

// Task gửi dữ liệu (angle và RPM) vào hàng đợi hoặc gửi trực tiếp lên Node-RED
void taskSendData(void* pvParameters) {
  SensorData data;
  while (true) {
    data.angle = angle;                    // Đọc giá trị góc
    data.RPM_Left = calculateRPMLeft();    // Đọc RPM trái
    data.RPM_Right = calculateRPMRight();  // Đọc RPM phải

    xQueueSend(dataQueue, &data, 0);  // Gửi cấu trúc SensorData vào hàng đợi

    // In thông tin ra Serial Monitor
    Serial.print("Angle sent: ");
    Serial.print(angle);
    Serial.print(" | RPM_Left: ");
    Serial.print(RPM_Left);
    Serial.print(" | RPM_Right: ");
    Serial.println(RPM_Right);

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay giữa các lần gửi
  }
}

void publishMQTT(float angle, float RPM_Left, float RPM_Right) {
  // Chuyển đổi các giá trị float thành chuỗi
  char angleStr[10];
  char rpmLeftStr[10];
  char rpmRightStr[10];

  dtostrf(angle, 1, 2, angleStr);         // Chuyển đổi angle thành chuỗi
  dtostrf(RPM_Left, 1, 2, rpmLeftStr);    // Chuyển đổi rpmLeft thành chuỗi
  dtostrf(RPM_Right, 1, 2, rpmRightStr);  // Chuyển đổi rpmRight thành chuỗi

  // Gửi các giá trị đến các chủ đề khác nhau
  client.publish("3T/MPU6050", angleStr);
  client.publish("3T/RPM_LEFT", rpmLeftStr);
  client.publish("3T/RPM_RIGHT", rpmRightStr);
}
void taskPublishMQTT(void* pvParameters) {
  SensorData data;

  while (true) {
    // Nhận dữ liệu từ hàng đợi
    if (xQueueReceive(dataQueue, &data, portMAX_DELAY) == pdPASS) {
      if (!client.connected()) {
        reconnect();
      }
      client.loop();
      // Gửi dữ liệu đến MQTT broker
      publishMQTT(data.angle, data.RPM_Left, data.RPM_Right);
      vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay giữa các lần gửi
    }
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void IRAM_ATTR ENCODER_LEFT() {
  int stateA1 = digitalRead(encoderA_m1);
  int stateB1 = digitalRead(encoderB_m1);
  if (stateA1 == HIGH) {
    encoderCountLeft += (stateB1 == LOW) ? 1 : -1;
  } else {
    encoderCountLeft += (stateB1 == LOW) ? -1 : 1;
  }
}

void IRAM_ATTR ENCODER_RIGHT() {
  int stateA2 = digitalRead(encoderA_m2);
  int stateB2 = digitalRead(encoderB_m2);
  if (stateA2 == HIGH) {
    encoderCountRight += (stateB2 == LOW) ? 1 : -1;
  } else {
    encoderCountRight += (stateB2 == LOW) ? -1 : 1;
  }
}

// Hàm tính tốc độ từ số xung (RPM)
float calculateRPMLeft() {
  currentTimeEncoderLeft = millis();
  double timeElapsed = currentTimeEncoderLeft - previousTimeEncoderLeft;
  // Nếu đã đủ thời gian để tính toán tốc độ
  if (timeElapsed >= 10) {                                                  // Cập nhật mỗi 1 giây
    RPM_Left = (encoderCountLeft / 330.0) * 60.0 * (1000.0 / timeElapsed);  // Chuyển số xung thành RPM
    encoderCountLeft = 0;                                                   // Reset số xung để đếm lại
    previousTimeEncoderLeft = currentTimeEncoderLeft;                       // Cập nhật thời gian
  }

  return RPM_Left;
}

float calculateRPMRight() {
  currentTimeEncoderRight = millis();
  double timeElapsed = currentTimeEncoderRight - previousTimeEncoderRight;
  // Nếu đã đủ thời gian để tính toán tốc độ
  if (timeElapsed >= 10) {                                                    // Cập nhật mỗi 1 giây
    RPM_Right = (encoderCountRight / 330.0) * 60.0 * (1000.0 / timeElapsed);  // Chuyển số xung thành RPM
    encoderCountRight = 0;                                                    // Reset số xung để đếm lại
    previousTimeEncoderRight = currentTimeEncoderRight;                       // Cập nhật thời gian
  }

  return RPM_Right;
}

// Hàm PID cho động cơ trái
float computePID4MOTOR_LEFT(float setpoint_L, float input_L, float kp, float ki, float kd, float* prevErrorPID_L, float* integralPID_L, unsigned long* prevTimeMotor_L) {
  unsigned long currentTimeMotor_L = millis();

  // // Kiểm tra nếu các con trỏ chưa được khởi tạo
  if (prevTimeMotor_L == nullptr || prevErrorPID_L == nullptr || integralPID_L == nullptr) { return 0.0; }

  // Tính toán thời gian thay đổi
  double time_change = (double)(currentTimeMotor_L - *prevTimeMotor_L) / 1000.0;  // chuyển thành giây
  if (time_change <= 0) time_change = 0.001;                                      // Tránh chia cho 0

  // Tính toán lỗi, tích phân, và đạo hàm
  float errorPID_L = setpoint_L - input_L;
  *integralPID_L += errorPID_L * time_change;
  *integralPID_L = constrain(*integralPID_L, -MAX_INTEGRAL, MAX_INTEGRAL);  // Giới hạn tích phân

  float derivativePID_L = (errorPID_L - *prevErrorPID_L) / time_change;
  float outputPID_L = kp * errorPID_L + ki * (*integralPID_L) + kd * derivativePID_L;

  // Cập nhật giá trị trước
  *prevErrorPID_L = errorPID_L;
  *prevTimeMotor_L = currentTimeMotor_L;

  return outputPID_L;
}

// Hàm PID cho động cơ phải
float computePID4MOTOR_RIGHT(float setpoint_R, float input_R, float kp, float ki, float kd, float* prevErrorPID_R, float* integralPID_R, unsigned long* prevTimeMotor_R) {
  unsigned long currentTimeMotor_R = millis();

  // Kiểm tra nếu các con trỏ chưa được khởi tạo
  if (prevTimeMotor_R == nullptr || prevErrorPID_R == nullptr || integralPID_R == nullptr) { return 0; }

  // Tính toán thời gian thay đổi
  double time_change = (double)(currentTimeMotor_R - *prevTimeMotor_R) / 1000.0;  // chuyển thành giây
  if (time_change <= 0) time_change = 0.001;                                      // Tránh chia cho 0

  // Tính toán lỗi, tích phân, và đạo hàm
  float errorPID_R = setpoint_R - input_R;
  *integralPID_R += errorPID_R * time_change;
  *integralPID_R = constrain(*integralPID_R, -MAX_INTEGRAL, MAX_INTEGRAL);  // Giới hạn tích phân

  float derivativePID_R = (errorPID_R - *prevErrorPID_R) / time_change;
  float outputPID_R = kp * errorPID_R + ki * (*integralPID_R) + kd * derivativePID_R;

  // Cập nhật giá trị trước
  *prevErrorPID_R = errorPID_R;
  *prevTimeMotor_R = currentTimeMotor_R;

  return outputPID_R;
}

// Hàm điều khiển robot di chuyển về trước
void moveForwardLeft(int totalPWM_Left) {
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  ledcWrite(ena, abs(totalPWM_Left));
}

void moveForwardRight(int totalPWM_Right) {
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);
  ledcWrite(enb, abs(totalPWM_Right));
}
// Hàm điều khiển robot di chuyển về phía sau
void moveBackwardLeft(int totalPWM_Left) {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  ledcWrite(ena, abs(totalPWM_Left));
}

void moveBackwardRight(int totalPWM_Right) {
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
  ledcWrite(enb, abs(totalPWM_Right));
}

// Hàm dừng động cơ
void stopMotors() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  ledcWrite(ena, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
  ledcWrite(enb, 0);
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

void ControlBalancingRobot() {
  // Tính toán thời gian giữa các lần lấy dữ liệu
  prevTime = currentTime;
  currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0;

  // Đọc dữ liệu từ cảm biến MPU6050
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

  // Tính toán góc dựa trên dữ liệu accel và gyro
  accelX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI;
  accelY = atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI;

  pitch = 0.98 * (pitch + gyroY * dt) + 0.02 * accelY;
  angle = kalman.getAngle(pitch, gyroY, dt);

  // Tính toán PID để giữ cân bằng
  errorL = setpoint - angle + offset;
  integralL += errorL * dt;
  integralL = constrain(integralL, -MAX_INTEGRAL, MAX_INTEGRAL);
  derivativeL = (errorL - prevErrorL) / dt;
  outputL = Kp * errorL + Ki * integralL + Kd * derivativeL;
  outputL = constrain(outputL, -255, 255);
  prevErrorL = errorL;

  errorR = setpoint - angle + offset;
  integralR += errorR * dt;
  integralR = constrain(integralR, -MAX_INTEGRAL, MAX_INTEGRAL);
  derivativeR = (errorR - prevErrorR) / dt;
  outputR = Kp * errorR + Ki * integralR + Kd * derivativeR;
  outputR = constrain(outputR, -255, 255);
  prevErrorR = errorR;

  // Tổng hợp đầu ra từ PID cân bằng và PID tốc độ động cơ
  totalPWM_Left = outputL;    // Cộng thêm PWM từ taskPID
  totalPWM_Right = outputR;  // Cộng thêm PWM từ taskPID

  // Điều khiển xe dựa trên góc nghiêng và hướng đi
  if (angle < -30.00 || angle > 30.00) {
    stopMotors();
  } else {
    if (totalPWM_Left > 0.0 && totalPWM_Right > 0.0) {
      moveBackwardLeft(totalPWM_Left);
      moveBackwardRight(totalPWM_Right);
    } else if (totalPWM_Left < 0.0 && totalPWM_Right < 0.0) {
      moveForwardLeft(totalPWM_Left);
      moveForwardRight(totalPWM_Right);
    } else {
      stopMotors();
    }
  }

  prevTime = currentTime;
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  espClient.setInsecure();  // Kết nối không kiểm tra chứng chỉ SSL
  client.setServer(mqtt_server, mqtt_port);

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

  // Cấu hình cho encoder trái
  pinMode(encoderA_m1, INPUT);
  pinMode(encoderB_m1, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA_m1), ENCODER_LEFT, RISING);

  // Cấu hình cho encoder phải
  pinMode(encoderA_m2, INPUT);
  pinMode(encoderB_m2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA_m2), ENCODER_RIGHT, RISING);

  ledcAttach(ena, freq, resolution);
  ledcAttach(enb, freq, resolution);
  // Tắt động cơ ban đầu
  stopMotors();
  Calibrate_IMU();
  kalman.setAngle(0);

  // Khởi tạo các hàng đợi
  dataQueue = xQueueCreate(10, sizeof(SensorData));  // Hàng đợi cho SensorData
  xQueuePWM = xQueueCreate(10, sizeof(float[2]));

  // Tạo task cho việc cân bằng và gửi dữ liệu
  xTaskCreatePinnedToCore(taskBalance, "Balance Task", 16384, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskPID, "PID Task", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskSendData, "Send Data Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskPublishMQTT, "Publish Data Task", 4096, NULL, 1, NULL, 0);
  currentTime = micros();
}

void loop() {
  // Do nothing...
}