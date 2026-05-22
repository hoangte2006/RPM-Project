#include <Arduino.h>
#include <TM1637Display.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

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
volatile unsigned long lastPulseTime = 0;
unsigned long lastRpmTime = 0;
int currentRpm = 0;
int smoothedRpm = 0;
unsigned long rideTimerStart = 0;

void IRAM_ATTR rpmInterrupt() {
  unsigned long currentTime = micros();
  if (currentTime - lastPulseTime > 5000) {
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
      display.setSegments(animFrames[j]);
      delay(35); // Thay đổi số này để tăng/giảm tốc độ chạy LED
    }
  }
  display.clear(); // Chạy xong thì xóa trắng màn hình
}

void setup() {
  // Chỉ tắt bit ENA của brownout, không ghi đè cả register như lần trước
  REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);

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
  rideTimerStart = millis();
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
      } else if (displayState == 3) {
        displayState = 4; // Đang Nhiệt độ -> Chuyển sang Timer
      } else {
        displayState = 1; // Đang Timer -> Quay lại RPM
      }

      display.clear();
      showingModeName = true;
      modeNameStartTime = currentMillis;
      lastDebounceTime = currentMillis;
      
      if (displayState == 1) Serial.println("Che do: VONG TUA (RPM)");
      else if (displayState == 2) Serial.println("Che do: DIEN AP (VOLTAGE)");
      else if (displayState == 3) Serial.println("Che do: NHIET DO (TEMP)");
      else if (displayState == 4) Serial.println("Che do: DONG HO (TIMER)");
    }
  }
  lastTouchState = touchState;

  // 2. ĐO VÀ TÍNH TOÁN RPM (cập nhật mỗi 500ms)
  if (currentMillis - lastRpmTime >= 500) {
    noInterrupts();
    unsigned long currentPulses = pulseCount;
    pulseCount = 0;
    interrupts();

    int rawRpm = currentPulses * 6 * 60;

    static int zeroCount = 0;
    if (smoothedRpm == 0) {
      smoothedRpm = rawRpm;
      zeroCount = 0;
    } else if (rawRpm == 0) {
      // 3 window liên tiếp = 1.5 giây không có xung -> tắt máy thật, về 0
      if (++zeroCount >= 3) { smoothedRpm = 0; zeroCount = 0; }
    } else {
      zeroCount = 0;
      smoothedRpm = (rawRpm * 0.3) + (smoothedRpm * 0.7);
    }

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
        static const uint8_t modeNames[4][4] = {
          {0x50, 0x73, 0x76, 0x00}, // r P H _  (RPM)
          {0x3E, 0x5C, 0x38, 0x78}, // U o L t  (Volt)
          {0x78, 0x79, 0x76, 0x73}, // t E H P  (Temp)
          {0x39, 0x38, 0x5C, 0x58}, // C L o c  (Timer)
        };
        display.setSegments(modeNames[displayState - 1]);
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
      display.setSegments(rpmSegs);
    } else if (!showingModeName && displayState == 2) {
      // Hiển thị dạng "12.50" — nhân 100 để giữ 2 số lẻ, bật bit 0x80 vào chữ số thứ 2 để ra dấu chấm
      int v = constrain((int)round(battery_voltage * 100), 0, 9999);
      uint8_t voltSegs[4] = {
        display.encodeDigit(v / 1000),
        (uint8_t)(display.encodeDigit((v / 100) % 10) | 0x80), // dấu chấm thập phân
        display.encodeDigit((v / 10) % 10),
        display.encodeDigit(v % 10)
      };
      display.setSegments(voltSegs);
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
      display.setSegments(tempSegs);
    } else if (!showingModeName && displayState == 4) {
      // Hiển thị MM:SS (dưới 1 giờ) hoặc HH:MM (trên 1 giờ), dấu : nhấp nháy mỗi giây
      unsigned long elapsed = (millis() - rideTimerStart) / 1000;
      int hi = (elapsed < 3600) ? (elapsed / 60) : (elapsed / 3600);
      int lo = (elapsed < 3600) ? (elapsed % 60) : ((elapsed % 3600) / 60);
      hi = constrain(hi, 0, 99);
      bool colon = (millis() % 1000) < 500;
      display.showNumberDecEx(hi * 100 + lo, colon ? 0b01000000 : 0, true);
    }
  }

  delay(50); // Delay nhẹ giúp làm dịu tác vụ của vòng lặp
}