// Khai báo chân của encoder
const int encoderPinA = 2;  // Chân ngắt của Arduino (kênh A)
const int encoderPinB = 4;  // Chân digital của Arduino (kênh B)

volatile int encoderPos = 0;  // Biến để lưu số xung đã đếm
volatile bool direction = true;  // Hướng quay (true = quay thuận, false = quay ngược)

void setup() {
  // Cấu hình chân của encoder
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  // Gắn ngắt ngoài cho chân A (kênh A)
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  
  // Khởi tạo cổng Serial để hiển thị kết quả
  Serial.begin(9600);
}

void loop() {
  // Hiển thị vị trí encoder và hướng quay
  Serial.print("Position: ");
  Serial.print(encoderPos);
  Serial.print("\tDirection: ");
  if (direction) {
    Serial.println("Clockwise (Forward)");
  } else {
    Serial.println("Counter-Clockwise (Backward)");
  }
  delay(500); // In ra kết quả mỗi nửa giây
}

// Hàm ngắt để xử lý tín hiệu từ encoder
void encoderISR() {
  // Đọc trạng thái kênh A và kênh B
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);
  
  // Kiểm tra hướng dựa trên trạng thái kênh B
  if (stateA == stateB) {
    encoderPos++;  // Quay thuận (clockwise)
    direction = true;
  } else {
    encoderPos--;  // Quay ngược (counter-clockwise)
    direction = false;
  }
}
