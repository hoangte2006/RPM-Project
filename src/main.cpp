#include <Arduino.h>
#include <TM1637Display.h>

// --- Định nghĩa các chân cho ESP32-C3 theo sơ đồ ---
#define ADC_PIN     0  // GPIO 0: Đo điện áp (qua bộ chia áp 47k - 10k)
#define NTC_PIN     1  // GPIO 1: Đo nhiệt độ từ cảm biến NTC
#define CLK_PIN     4  // GPIO 4: Chân CLK của TM1637
#define RPM_PIN     5  // GPIO 5: Đọc xung RPM từ Opto PC817
#define DIO_PIN     6  // GPIO 6: Chân DIO của TM1637
#define TOUCH_PIN   7  // GPIO 7: Chân tín hiệu từ cảm biến chạm TTP223

// --- Cấu hình bộ chia áp ---
// Công thức tính: V_in = V_adc * (R1 + R2) / R2
const float R1 = 47000.0; // Điện trở 47k
const float R2 = 10000.0; // Điện trở 10k
const float DIVIDER_RATIO = (R1 + R2) / R2; 
const float CALIBRATION_FACTOR = 1.0; // Dùng để tinh chỉnh sai số đồng hồ đo (nếu cần)

// --- Cấu hình cảm biến nhiệt độ NTC ---
// Sơ đồ mạch: VCC (3.3V) ---> Trở 10K ---> GPIO 1 (NTC_PIN) ---> NTC ---> GND
const float SERIES_RESISTOR = 10000.0;     // Điện trở mắc nối tiếp kéo lên 3.3V (10K ohms)
const float NOMINAL_RESISTANCE = 10000.0;  // Điện trở của NTC ở 25°C (10K. Nếu NTC 5K thì đổi thành 5000.0)
const float NOMINAL_TEMPERATURE = 25.0;    // Nhiệt độ danh định
const float BCOEFFICIENT = 3950.0;         // Hệ số Beta của NTC (thường là 3950)

// --- Biến cho RPM ---
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0; // Biến ghi nhớ thời điểm xuất hiện xung cuối cùng
unsigned long lastRpmTime = 0;
int currentRpm = 0;
int smoothedRpm = 0; // Biến lưu vòng tua đã được làm mượt

// ISR: Trình phục vụ ngắt cho RPM (Kích hoạt khi có xung từ opto kéo xuống Mass)
void IRAM_ATTR rpmInterrupt() {
  unsigned long currentTime = micros(); // Lấy thời gian thực tính bằng micro-giây
  // Lọc nhiễu (Debounce): Redline của xe Wave tối đa khoảng 10.000 RPM (1 vòng tốn 6000us).
  // Nếu 2 xung đến cách nhau nhỏ hơn 3000us (tương đương tốc độ > 20.000 RPM) thì chắc chắn là nhiễu gai.
  if (currentTime - lastPulseTime > 3000) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}

// --- Biến Trạng Thái Chế Độ Hiển Thị ---
int displayState = 0; // 0 = Tắt, 1 = RPM, 2 = Voltage, 3 = Temperature
int lastTouchState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // Chống dội chạm (50ms)
bool showingModeName = false;
unsigned long modeNameStartTime = 0;
const unsigned long MODE_NAME_DURATION = 1000; // Hiện tên mode 1 giây

// Khởi tạo module màn hình TM1637
TM1637Display display(CLK_PIN, DIO_PIN);

// Xoay 1 byte segment 180°: A↔D, B↔E, C↔F, G giữ nguyên
uint8_t rotSeg(uint8_t s) {
  return ((s & 0x01) << 3) | ((s & 0x08) >> 3)
       | ((s & 0x02) << 3) | ((s & 0x10) >> 3)
       | ((s & 0x04) << 3) | ((s & 0x20) >> 3)
       | (s & 0x40);
}
// Hiển thị 4 segment đã xoay 180° và đảo thứ tự trái↔phải
void displayFlipped(const uint8_t segs[4]) {
  uint8_t f[4] = { rotSeg(segs[3]), rotSeg(segs[2]), rotSeg(segs[1]), rotSeg(segs[0]) };
  display.setSegments(f);
}

uint32_t readADCAvg(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogReadMilliVolts(pin);
    delayMicroseconds(100);
  }
  return sum / 20;
}

// Hàm đọc nhiệt độ từ cảm biến NTC
float getTemperature() {
  float adc_mv = readADCAvg(NTC_PIN);
  
  // Giới hạn để tránh lỗi chia cho 0 hoặc hở mạch
  if (adc_mv <= 0) return 0.0;
  if (adc_mv >= 3300) return 99.0; // Lỗi hở mạch hoặc chưa cắm cảm biến

  // Tính điện trở NTC hiện tại
  float r_ntc = SERIES_RESISTOR * adc_mv / (3300.0 - adc_mv);

  // Tính toán nhiệt độ theo phương trình Steinhart-Hart
  float steinhart = r_ntc / NOMINAL_RESISTANCE; 
  steinhart = log(steinhart); // Cần #include <math.h> nhưng Arduino.h đã bao gồm sẵn
  steinhart /= BCOEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15; // Đổi sang độ C
  
  return steinhart;
}

// Hiệu ứng LED chạy vòng quanh viền màn hình lúc khởi động
void playStartupAnimation() {
  // Mảng chứa các khung hình, mỗi khung bật 1 thanh LED ở viền ngoài cùng của 4 con số
  const uint8_t animFrames[][4] = {
    {SEG_A, 0, 0, 0},        // Viền trên số 1
    {0, SEG_A, 0, 0},        // Viền trên số 2
    {0, 0, SEG_A, 0},        // Viền trên số 3
    {0, 0, 0, SEG_A},        // Viền trên số 4
    {0, 0, 0, SEG_B},        // Viền phải trên số 4
    {0, 0, 0, SEG_C},        // Viền phải dưới số 4
    {0, 0, 0, SEG_D},        // Viền dưới số 4
    {0, 0, SEG_D, 0},        // Viền dưới số 3
    {0, SEG_D, 0, 0},        // Viền dưới số 2
    {SEG_D, 0, 0, 0},        // Viền dưới số 1
    {SEG_E, 0, 0, 0},        // Viền trái dưới số 1
    {SEG_F, 0, 0, 0}         // Viền trái trên số 1
  };

  for(int i = 0; i < 3; i++) { // Lặp vòng tròn 3 lần
    for(int j = 0; j < 12; j++) {
      displayFlipped(animFrames[j]);
      delay(35); // Thay đổi số này để tăng/giảm tốc độ chạy LED
    }
  }
  display.clear(); // Chạy xong thì xóa trắng màn hình
}

void setup() {
  Serial.begin(115200);

  // Mở rộng ngưỡng ADC lên ~3.1V — bắt buộc phải có, mặc định chỉ đo được ~950mV
  analogSetAttenuation(ADC_11db);

  // Khởi tạo các chân tín hiệu
  pinMode(TOUCH_PIN, INPUT_PULLDOWN); // Pull-down để tránh float khi không chạm
  pinMode(ADC_PIN, INPUT);
  pinMode(NTC_PIN, INPUT);
  
  // Chân nhận xung RPM cần Pull-up do Opto PC817 xuất xung kéo xuống GND (chân 4 & 3)
  // Dù sơ đồ đã có trở Pull-up 10k bên ngoài, bật thêm Pull-up nội vẫn an toàn
  pinMode(RPM_PIN, INPUT_PULLUP);
  
  // Cài đặt ngắt: Khi Opto PC817 đóng mạch sẽ tạo một cạnh xuống (FALLING)
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmInterrupt, FALLING);

  // Khởi tạo LED 7 đoạn
  display.setBrightness(0x0f); // Độ sáng tối đa (0x00 đến 0x0f)
  display.clear();

  // Chạy hiệu ứng khởi động
  playStartupAnimation();
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. ĐỌC CẢM BIẾN CHẠM (Có chống dội / Debounce)
  int touchState = digitalRead(TOUCH_PIN);
  if (touchState != lastTouchState) {
    if (touchState == HIGH && (currentMillis - lastDebounceTime) > debounceDelay) {
      
      // Xử lý chuyển đổi các chế độ
      if (displayState == 0) {
        displayState = 1; // Đang tắt -> Bật lên RPM
      } else if (displayState == 1) {
        displayState = 2; // Đang RPM -> Chuyển sang Vol
      } else if (displayState == 2) {
        displayState = 3; // Đang Vol -> Chuyển sang Nhiệt độ
      } else {
        displayState = 1; // Đang Nhiệt độ -> Quay lại RPM
      }

      display.clear();
      showingModeName = true;
      modeNameStartTime = currentMillis;
      lastDebounceTime = currentMillis;
      
      if (displayState == 1) Serial.println("Che do: VONG TUA (RPM)");
      else if (displayState == 2) Serial.println("Che do: DIEN AP (VOLTAGE)");
      else if (displayState == 3) Serial.println("Che do: NHIET DO (TEMP)");
    }
  }
  lastTouchState = touchState;

  // 2. ĐO VÀ TÍNH TOÁN RPM (Cập nhật sau mỗi 500ms)
  if (currentMillis - lastRpmTime >= 500) {
    noInterrupts(); // Tạm tắt ngắt để lấy số đếm chính xác nhất
    unsigned long currentPulses = pulseCount;
    pulseCount = 0; // Đặt lại bộ đếm
    interrupts();   // Bật ngắt trở lại

    // Trigger coil Wave: RPM = pulses * (1000/500) * 60 * PULSES_PER_REV_DIVIDER
    // Nếu RPM hiện = 1/3 thực tế -> đổi * 2 thành * 6. Nếu = 1/2 -> đổi thành * 4.
    int rawRpm = currentPulses * 6 * 60;

    // Lọc làm mượt số hiển thị (Exponential Moving Average)
    if (smoothedRpm == 0) smoothedRpm = rawRpm; 
    else smoothedRpm = (rawRpm * 0.4) + (smoothedRpm * 0.6); // Lấy 40% giá trị mới + 60% giá trị cũ

    // Làm tròn về bội số của 10 để chống chớp nháy hàng đơn vị (Ví dụ: 1453 -> 1450)
    currentRpm = (smoothedRpm / 10) * 10;
    
    lastRpmTime = currentMillis;
  }

  // 3. ĐO ĐIỆN ÁP ACQUY
  // analogReadMilliVolts() của ESP32 tính toán phần cứng rất chuẩn xác so với analogRead()
  uint32_t adc_mv = readADCAvg(ADC_PIN);
  float adc_voltage = adc_mv / 1000.0; 
  float battery_voltage = (adc_voltage * DIVIDER_RATIO) * CALIBRATION_FACTOR;

  // 4. HIỂN THỊ DỮ LIỆU LÊN MÀN HÌNH LED 7 ĐOẠN
  if (displayState != 0) {
    // Hiện tên mode 1 giây khi mới chuyển: "rPH_" / "UoLt" / "tEHP"
    if (showingModeName) {
      if (currentMillis - modeNameStartTime < MODE_NAME_DURATION) {
        static const uint8_t modeNames[3][4] = {
          {0x50, 0x73, 0x76, 0x00}, // r P H _  (RPM)
          {0x3E, 0x5C, 0x38, 0x78}, // U o L t  (Volt)
          {0x78, 0x79, 0x76, 0x73}, // t E H P  (Temp)
        };
        displayFlipped(modeNames[displayState - 1]);
      } else {
        showingModeName = false;
        display.clear();
      }
    }

    if (!showingModeName && displayState == 1) {
      uint8_t rpmSegs[4] = {0, 0, 0, 0};
      int rpm = currentRpm;
      rpmSegs[3] = display.encodeDigit(rpm % 10);
      if (rpm >= 10)   rpmSegs[2] = display.encodeDigit((rpm / 10) % 10);
      if (rpm >= 100)  rpmSegs[1] = display.encodeDigit((rpm / 100) % 10);
      if (rpm >= 1000) rpmSegs[0] = display.encodeDigit(rpm / 1000);
      displayFlipped(rpmSegs);
    } else if (!showingModeName && displayState == 2) {
      // Hiển thị dạng "12.50" — nhân 100 để giữ 2 số lẻ, bật bit 0x80 vào chữ số thứ 2 để ra dấu chấm
      int v = constrain((int)round(battery_voltage * 100), 0, 9999);
      uint8_t voltSegs[4] = {
        display.encodeDigit(v / 1000),
        (uint8_t)(display.encodeDigit((v / 100) % 10) | 0x80), // dấu chấm thập phân
        display.encodeDigit((v / 10) % 10),
        display.encodeDigit(v % 10)
      };
      displayFlipped(voltSegs);
    } else if (!showingModeName && displayState == 3) {
      // Hiển thị nhiệt độ dạng "85°C" — 0x63 = ký hiệu ° trên LED 7 đoạn
      float tempC = getTemperature();
      int t = round(tempC);
      uint8_t tempSegs[4] = {0x00, 0x00, 0x63, 0x39}; // mặc định: [_][_][°][C]

      if (t >= 100) {                          // "100C" — không đủ chỗ cho °
        tempSegs[0] = display.encodeDigit(t / 100);
        tempSegs[1] = display.encodeDigit((t / 10) % 10);
        tempSegs[2] = display.encodeDigit(t % 10);
      } else if (t >= 10) {                    // "85°C"
        tempSegs[0] = display.encodeDigit(t / 10);
        tempSegs[1] = display.encodeDigit(t % 10);
      } else if (t >= 0) {                     // " 5°C"
        tempSegs[1] = display.encodeDigit(t);
      } else if (t >= -9) {                    // "-5°C"
        tempSegs[0] = 0x40;
        tempSegs[1] = display.encodeDigit(-t);
      } else {                                 // "-15C" — không đủ chỗ cho °
        tempSegs[0] = 0x40;
        tempSegs[1] = display.encodeDigit((-t) / 10);
        tempSegs[2] = display.encodeDigit((-t) % 10);
      }
      displayFlipped(tempSegs);
    }
  }

  delay(50); // Delay nhẹ giúp làm dịu tác vụ của vòng lặp
}