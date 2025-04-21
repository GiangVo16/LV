// Cấu hình động cơ 0
#define DIR0_PIN 2
#define STEP0_PIN 4
// Cấu hình động cơ 1
#define DIR1_PIN 17
#define STEP1_PIN 5
#define ENA_1 16
// Cấu hình động cơ 2
#define STEP2_PIN 23
#define DIR2_PIN 19
// Cấu hình động cơ 3
#define STEP3_PIN 33
#define DIR3_PIN 25
#define ENA_3 26
// Cấu hình phanh từ
#define PHANH_0 999
#define PHANH_1 12
#define PHANH_2 999
#define PHANH_3 13
// Cấu hình Servo
#define SERVO_1 14
#define SERVO_2 27

// THỜI GIAN DELAY GIỮA CÁC BƯỚC (microsecond)
const int speed0 = 800;
const int speed1 = 1300;
const int speed2 = 30;
const int speed3 = 800;

// GÓC BAN ĐẦU CỦA TAY MÁY (khi khởi động sẽ quay về góc này)
float goc_dat_0 = 0;
float goc_dat_1 = 90;
float goc_dat_2 = 270;
float goc_dat_3 = 270;

// NGƯỠNG SAI SỐ GÓC CHẤP NHẬN
float threshold = 0.2;


// GIỚI HẠN GÓC QUAY CÁC KHỚP
const float min_angle_0 = 35.0;  // Giới hạn góc tối thiểu
const float max_angle_0 = 310.0;  // Giới hạn góc tối đa

const float min_angle_1 = 110.0;  // Giới hạn góc tối thiểu
const float max_angle_1 = 300.0;  // Giới hạn góc tối đa

const float min_angle_2 = 100.0;  // Giới hạn góc tối thiểu
const float max_angle_2 = 200.0;  // Giới hạn góc tối đa

const float min_angle_3 = 100.0;  // Giới hạn góc tối thiểu
const float max_angle_3 = 240.0;  // Giới hạn góc tối đa

// Giá trị góc đã được set làm gốc 0 độ (180.24°)
const float angle_calib_0 = 185.36;
const float angle_calib_1 = 323.17;
const float angle_calib_2 = 26.72;
const float angle_calib_3 = 265.69;

// BIẾN KÍCH THƯỚC VÀ ĐỘNG HỌC TAY MÁY
float a2 = 0.32, a3 = 0.27, a4 = 0.076;
float d1 = 0.406, d2=0.039, d3=0.669;
float phi = -PI/2; // góc gắp = 0 (radian)
float x, y, z;
String input;
enum Direction { CW, CCW };

