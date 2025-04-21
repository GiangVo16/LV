// Định nghĩa các chân IO encoder
#define ENCODER1_A_PIN 25  // Chân A của encoder 1
#define ENCODER1_B_PIN 33  // Chân B của encoder 1
#define ENCODER2_A_PIN 27  // Chân A của encoder 2
#define ENCODER2_B_PIN 26  // Chân B của encoder 2
// Định nghĩa các chân IO driver
#define IN1 16
#define IN2 4
#define IN3 18
#define IN4 19
#define EN1 17  // Chân PWM cho motor 1
#define EN2 5   // Chân PWM cho motor 2

// Thông số mobile robot và động cơ (KHI CÓ CÁNH TAY ROBOT)
// #define WHEEL_RADIUS 0.076   // Bán kính bánh xe 1 (m) -> 100 (cm)
// #define PULSES_PER_REV 500  // Xung encoder trên mỗi vòng động cơ
// #define GEAR_RATIO 19.7     // Tỷ số truyền hộp số
// #define WHEEL_BASE 0.328     // Khoảng cách giữa hai bánh xe (m)

// Thông số mobile robot và động cơ (KHI CÓ CÁNH TAY ROBOT)
#define WHEEL_RADIUS 0.08   // Bán kính bánh xe 1 (m) -> 100 (cm)
#define PULSES_PER_REV 500  // Xung encoder trên mỗi vòng động cơ
#define GEAR_RATIO 19.7     // Tỷ số truyền hộp số
#define WHEEL_BASE 0.325     // Khoảng cách giữa hai bánh xe (m)

// Tọa độ ban đầu và góc quay robot
float x = 1.5, y = 1.1   , theta = 0.0;  

// Khai báo Wifi ESP là station
// const char *ssid = "Phong IOT";          // Thay bằng SSID của bạn
// const char *password = "@thayHieucatroi";  // Thay bằng mật khẩu Wi-Fi của bạn
const char *ssid = "K50U";          // Thay bằng SSID của bạn
const char *password = "19051890";  // Thay bằng mật khẩu Wi-Fi của bạn

// Cấu hình IP tĩnh
IPAddress local_IP(192, 168, 50, 234);      // IP bạn muốn gán cho ESP32
IPAddress gateway(192, 168, 50, 43);         // Thường là địa chỉ router
IPAddress subnet(255, 255, 255, 0);        // Subnet mask
IPAddress primaryDNS(8, 8, 8, 8);          // DNS chính (Google)
IPAddress secondaryDNS(8, 8, 4, 4);        // DNS phụ (Google)


// Khởi tạo UDP
WiFiUDP udp;
// Địa chỉ IP của máy tính chạy MATLAB, chu kỳ truyền (thay đổi cho đúng mạng của bạn)
IPAddress matlabIP(192, 168, 50, 218);
unsigned int localUdpPort = 1234;  // Cổng nhận (nếu cần)
unsigned int matlabPort = 4321;    // Cổng MATLAB lắng nghe

// BIẾN TRẠNG THÁI ROBOT
int STATUS_ROBOT = 1;
int RUN_PATH = 1, FIND_OBJ = 2, WAIT = 3, DONE_PICKUP = 4;


float DISTANCE_OK_TO_OBJ = 25;
float ERROR_ANGLE_OBJ;

int detect_obj = 0;
String inputData = "";
float x_obj, y_obj, angle_obj;