// Khai báo chân kết nối với các kênh A và B của encoder
#define ena 12
#define in1 25
#define in2 26
#define in3 27
#define in4 14
#define enb 33

const int encoderPinA = 2;  // Kênh A nối với chân số 2
const int encoderPinB = 15;  // Kênh B nối với chân số 3

const int freq = 100;
const int resolution = 8;

volatile long encoderCount = 0;    // Biến lưu số xung
unsigned long previousMillis = 0;  // Thời gian lưu trước đó

const long interval = 1000;  // Khoảng thời gian 1 giây (1000ms)

// Thông số bánh xe và encoder
const float wheelDiameter = 0.065;   // Đường kính bánh xe (65mm = 0.065m)
const int ticksPerRevolution = 330;  // Số xung trên mỗi vòng quay của trục chính (PPR)

void setup() {
  // Thiết lập chân A và B như ngõ vào
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);
  ledcAttach(ena, freq, resolution);
  ledcAttach(enb, freq, resolution);

  // Kích hoạt ngắt ngoài để theo dõi kênh A
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);

  // Thiết lập serial monitor để theo dõi kết quả
  Serial.begin(115200);
}

void loop() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  ledcWrite(ena, 255);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
  ledcWrite(enb, 255);
  // Kiểm tra xem đã trôi qua 1 giây chưa
  unsigned long currentMillis = millis();  // Lấy thời gian hiện tại

  if (currentMillis - previousMillis >= interval) {
    // Nếu đã đủ 1 giây, cập nhật thời gian lưu trước đó
    previousMillis = currentMillis;

    // Tính vận tốc bánh xe
    float speed = calculateSpeed();  // Tính vận tốc (m/s)

    // In ra số xung và vận tốc sau mỗi giây
    Serial.print("Số xung trong 1 giây: ");
    Serial.println(encoderCount);

    Serial.print("Vận tốc bánh xe (m/s): ");
    Serial.println(speed);

    // Đặt lại encoderCount sau khi xuất ra để tính lại cho giây tiếp theo
    encoderCount = 0;
  }
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  ledcWrite(ena, 255);
}

// Hàm ngắt khi có thay đổi trên kênh A
void encoderISR() {
  // Đọc trạng thái hiện tại của kênh B
  int encoderBState = digitalRead(encoderPinB);

  // Nếu trạng thái kênh B khác trạng thái trước đó, trục đang quay theo chiều thuận
  if (encoderBState) {
    encoderCount++;  // Quay theo chiều thuận
  } else {
    encoderCount--;  // Quay theo chiều ngược lại
  }
}

// Hàm tính vận tốc bánh xe dựa trên số xung trong 1 giây
float calculateSpeed() {
  // Tính số vòng quay của trục chính trong 1 giây
  float revolutionsPerSecond = (float)encoderCount / ticksPerRevolution;

  // Tính chu vi bánh xe
  float wheelCircumference = PI * wheelDiameter;

  // Tính vận tốc bánh xe (m/s) = số vòng quay trục chính * chu vi bánh xe
  float speed = revolutionsPerSecond * wheelCircumference;

  return speed;  // Vận tốc tính theo m/s
}
