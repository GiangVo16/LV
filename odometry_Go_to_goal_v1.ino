#include <ESP32Encoder.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "data_web.h"
#include <math.h>

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

// Thông số mobile robot và động cơ
#define WHEEL_RADIUS 0.08   // Bán kính bánh xe 1 (m) -> 100 (cm)
#define PULSES_PER_REV 500  // Xung encoder trên mỗi vòng động cơ
#define GEAR_RATIO 19.7     // Tỷ số truyền hộp số
#define WHEEL_BASE 0.32     // Khoảng cách giữa hai bánh xe (m)

int64_t pulseCount_R = 0;  // Số xung encoder bánh phải
int64_t pulseCount_L = 0;  // Số xung encoder bánh trái
long lastPulse_R = 0, lastPulse_L = 0;
float x = 0.0, y = 0.0, theta = 0.0;  // Tọa độ và góc quay robot

// Khai báo 2 bộ encoder
ESP32Encoder encoder1;
ESP32Encoder encoder2;
// Biến thư viện ESP32Encoder
unsigned long encoder2lastToggled;
bool encoder2Paused = false;
long xung1, xung2;
long prev_xung1, prev_xung2;
long delta_xung1, delta_xung2;

// Biến đọc tốc độ
long RPM1, RPM2;
long PULSES_PER_REVOLUTION = 9850;
float sampleTime = 5;  // mili giây
unsigned long prevTime1 = 0, prevTime2 = 0, Ts1 = 5, Ts2 = 5;

// Biến PID
double speed_R = 0;  // Setpoint
double speed_L = 0;
double setSpeed = 0;
double error1 = 0, total_error1 = 0, prev_error1 = 0;  // Sai số hiện tại - tổng sai số - sai số trước đó
double error2 = 0, total_error2 = 0, prev_error2 = 0;
double integral1 = 0, derivative1 = 0, output_pwm1 = 0;
double integral2 = 0, derivative2 = 0, output_pwm2 = 0;
double Kp1 = 5, Ki1 = 0.25, Kd1 = 0.0;
double Kp2 = 5, Ki2 = 0.02, Kd2 = 0.0;


// Tham số PID góc
float Kp_t = 7;
float Ki_t = 0.001;
float Kd_t = 0.01;
float prev_theta_error = 0;
float integral_theta = 0;

// Vị trí đích
float target_x = 0.5;
float target_y = 0.5;


// Khai báo Wifi ESP là station
const char *ssid = "K50U";          // Thay bằng SSID của bạn
const char *password = "19051890";  // Thay bằng mật khẩu Wi-Fi của bạn

// Cấu hình PWM
const int frequency = 1000;  // Tần số PWM 5kHz
const int resolution = 8;    // Độ phân giải 8-bit

// FreeRTOS--------------------------------------------------------
// Tạo các task riêng biệt cho từng động cơ
TaskHandle_t motor1TaskHandle = NULL;
TaskHandle_t motor2TaskHandle = NULL;

// CPU định nghĩa cho FreeRTOS
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

void motor1Task(void *parameter) {
  while (true) {
    // Đọc tốc độ và PID cho động cơ 1
    read_motorSpeed1();
    PID_1();
    vTaskDelay(sampleTime / portTICK_PERIOD_MS);
  }
}

void motor2Task(void *parameter) {
  while (true) {
    // Đọc tốc độ và PID cho động cơ 2
    read_motorSpeed2();
    PID_2();
    vTaskDelay(sampleTime / portTICK_PERIOD_MS);
  }
}

// Khởi tạo Webserver giao thức Websocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup() {
  Serial.begin(115200);
  // Setup thư viện ESP32Encoder-----------------------------------------------------------------
  // Enable the weak pull down resistors
  //ESP32Encoder::useInternalWeakPullResistors = puType::down;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // use 2 pin for the first encoder
  encoder1.attachHalfQuad(ENCODER1_B_PIN, ENCODER1_A_PIN);
  // use 2 pin for the second encoder
  encoder2.attachHalfQuad(ENCODER2_A_PIN, ENCODER2_B_PIN);

  // set starting count value after attaching
  encoder1.setCount(0);
  // clear the encoder's raw count and set the tracked count to zero
  encoder2.clearCount();
  // set the lastToggle
  encoder2lastToggled = millis();
  //---------------------------------------------------------------------------------------------

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
  delay(1000);  // delay để đọc local ip

  // Xử lý các sự kiện WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Cung cấp giao diện web
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", htmlPage);
  });

  server.begin();

  // Thiết lập OUTPUT cho các chân IN1, IN2, IN3, IN4
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  /// demo
  /* Tiến
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  */

  /* Lùi
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  */

  /* quay phải
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  */

  /* quay trái
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  */

  // Thiết lập PWM cho các chân Enable của motor
  ledcAttach(EN1, frequency, resolution);
  ledcAttach(EN2, frequency, resolution);

  // Khởi tạo hai task cho hai động cơ-----------------------------------------
  xTaskCreatePinnedToCore(
    motor1Task,         // Hàm xử lý động cơ 1
    "Motor 1 Control",  // Tên task
    4096,               // Stack size
    NULL,               // Tham số truyền vào
    1,                  // Độ ưu tiên của task
    &motor1TaskHandle,  // Handle của task
    app_cpu             // CPU chạy task
  );

  xTaskCreatePinnedToCore(
    motor2Task,         // Hàm xử lý động cơ 2
    "Motor 2 Control",  // Tên task
    4096,               // Stack size
    NULL,               // Tham số truyền vào
    2,                  // Độ ưu tiên của task
    &motor2TaskHandle,  // Handle của task
    app_cpu             // CPU chạy task
  );
}  // Kết thúc setup


// Khối xử đọc tốc độ và PID -----------------------------------------------------
// Hàm đọc tốc độ cho động cơ 1
void read_motorSpeed1() {
  unsigned long currentTime = millis();
  prevTime1 = currentTime;

  // Lấy số xung hiện tại
  xung1 = encoder1.getCount() / 2;

  // Tính số xung trong khoảng thời gian lấy mẫu
  delta_xung1 = xung1 - prev_xung1;
  prev_xung1 = xung1;

  // Tính tốc độ động cơ (vòng/phút)
  RPM1 = ((float)delta_xung1 / PULSES_PER_REVOLUTION) * (60000.0 / sampleTime);
  RPM1 = abs(RPM1);
}

// Hàm đọc tốc độ cho động cơ 2
void read_motorSpeed2() {
  unsigned long currentTime = millis();
  prevTime2 = currentTime;

  // Lấy số xung hiện tại
  xung2 = encoder2.getCount() / 2;

  // Tính số xung trong khoảng thời gian lấy mẫu
  delta_xung2 = xung2 - prev_xung2;
  prev_xung2 = xung2;

  // Tính tốc độ động cơ (vòng/phút)
  RPM2 = ((float)delta_xung2 / PULSES_PER_REVOLUTION) * (60000.0 / sampleTime);
  RPM2 = abs(RPM2);
}

// Hàm PID cho động cơ 1
void PID_1() {
  error1 = speed_R - RPM1;
  //total_error1 += error1;
  integral1 += error1 * Ts1;
  if (delta_xung1 == 0) integral1 = 0;
  derivative1 = (error1 - prev_error1) / Ts1;
  output_pwm1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;

  // Xuất xung pwm
  output_pwm1 = constrain(output_pwm1, 0, 100);  // Chặn trên và chặn dưới khoảng [0;255]
  ledcWrite(EN1, output_pwm1);
  prev_error1 = error1;
}

// Hàm PID cho động cơ 2
void PID_2() {
  error2 = speed_L - RPM2;
  //total_error2 += error2;
  integral2 += error2 * Ts2;
  if (delta_xung2 == 0) integral2 = 0;
  derivative2 = (error2 - prev_error2) / Ts2;
  output_pwm2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;

  // Xuất xung pwm
  output_pwm2 = constrain(output_pwm2, 0, 100);  // Chặn trên và chặn dưới khoảng [0;255]
  ledcWrite(EN2, output_pwm2);
  prev_error2 = error2;
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    String msg = "";
    for (size_t i = 0; i < len; i++) {
      msg += (char)data[i];
    }
    Serial.println("Message received:" + msg);



    // Xử lý điều khiển motor theo nút nhấn
    if (msg == "forward") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (msg == "backward") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else if (msg == "left") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else if (msg == "right") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (msg == "stop") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      ledcWrite(EN1, 0);
      ledcWrite(EN2, 0);
    } else if (msg.startsWith("speed")) {
      setSpeed = msg.substring(5).toInt();
      speed_R = setSpeed;
      speed_L = setSpeed;
    }
    Serial.println(msg);
  }
}

float PID_AngleControl(float theta_error) {
  integral_theta += theta_error * sampleTime;
  integral_theta = constrain(integral_theta, -10, 10);  // Giới hạn tích phân
  float derivative_theta = (theta_error - prev_theta_error) / sampleTime;
  prev_theta_error = theta_error;
  return Kp_t * theta_error + Ki_t * integral_theta + Kd_t * derivative_theta;
}

void MotorControl(float control_signal) {
  int speed_turn = (int)abs(control_signal) * 10;
  speed_turn = constrain(speed_turn, 10, 30);

  if (control_signal > 0) {  // Quay trái
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {  // Quay phải
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  speed_R = speed_turn;
  speed_L = speed_turn;
}

void loop() {
  //Serial.print(Kp1); Serial.print(" "); Serial.print(Ki1, 3); Serial.print(" "); Serial.print(Kd1, 3); Serial.print(" "); Serial.print(speed_R); Serial.print(" "); Serial.println(RPM1);
  //Serial.print(Kp2); Serial.print(" "); Serial.print(Ki2, 3); Serial.print(" "); Serial.print(Kd2, 3); Serial.print(" "); Serial.print(speed_L); Serial.print(" "); Serial.println(RPM2);
  //Serial.print(error1); Serial.print(" "); Serial.print(integral1); Serial.print(" "); Serial.println(derivative1);
  inputPIDValues();
  Odometry();

  // Điều khiển góc --------------------------------------------------------------------------
  float target_theta = atan2(target_y - y, target_x - y);
  float theta_error = target_theta - theta;
  if (theta_error > PI) theta_error -= 2 * PI;
  if (theta_error < -PI) theta_error += 2 * PI;

  float control_signal = PID_AngleControl(theta_error);

  if (abs(theta_error*180/PI) > 3) {  // Nếu góc lỗi lớn hơn khoảng n độ thì quay robot
    MotorControl(control_signal);
  } else {  // DỪng
    //delay(1000);
    GoToPoint(target_x, target_y);
  }
  //--------------------------------------------------------------------------------------------
  Serial.print("X_target: ");
  Serial.print(target_x * 100);
  Serial.print(" cm, Y_target: ");
  Serial.print(target_y * 100);
  Serial.print(" X: ");
  Serial.print(x * 100);
  Serial.print(" cm, Y: ");
  Serial.print(y * 100);
  Serial.print(" cm, Theta: ");
  Serial.print(theta * 180 / PI);
  Serial.print(" độ, Lỗi góc: ");
  Serial.print(theta_error * 180 / PI);
  Serial.print(" độ, Điều khiển góc: ");
  Serial.println(control_signal);

  // GỬi tọa độ lên web
  sendOdometryData(); // tọa độ cm và độ

  delay(5);
}

// Hàm nhập giá trị PID và trạng thái động cơ
void inputPIDValues() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Đọc chuỗi đến khi gặp newline
    input.trim();                                 // Loại bỏ khoảng trắng đầu cuối

    // Tách chuỗi nhập vào thành hai phần: lệnh và giá trị
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("Invalid command! Use format: kp <value>, Ki <value>, Kd <value>, s1 <value>, s2 <value>, em1 <value>, em2 <value>");
      return;
    }

    String command = input.substring(0, spaceIndex);
    double value = input.substring(spaceIndex + 1).toDouble();

    // Kiểm tra lệnh và cập nhật giá trị tương ứng
    if (command == "kp1") {
      Kp1 = value;
    } else if (command == "ki1") {
      Ki1 = value;
    } else if (command == "kd1") {
      Kd1 = value;
    } else if (command == "kp2") {
      Kp2 = value;
    } else if (command == "ki2") {
      Ki2 = value;
    } else if (command == "kd2") {
      Kd2 = value;
    } else if (command == "kpt") {
      Kp_t = value;
    } else if (command == "kit") {
      Ki_t = value;
    } else if (command == "kdt") {
      Kd_t = value;
    } else if (command == "x") {
      target_x = value;
    } else if (command == "y") {
      target_y = value;
    } else if (command == "s1") {
      speed_R = value;  // Gán giá trị cho tốc độ đặt động cơ 1
    } else if (command == "s2") {
      speed_L = value;  // Gán giá trị cho tốc độ đặt động cơ 2
    } else {
      Serial.println("Invalid command! Use format: kp <value>, Ki <value>, Kd <value>, s1 <value>, s2 <value>, em1 <value>, em2 <value>");
    }
  }
}

// Hàm định vị robot
void Odometry() {
  // Chuyển xung encoder thành quãng đường từng bánh xe
  pulseCount_R = encoder1.getCount() / -2;
  pulseCount_L = encoder2.getCount() / -2;
  long deltaPulse_R = pulseCount_R - lastPulse_R;
  long deltaPulse_L = pulseCount_L - lastPulse_L;

  // Chuyển xung encoder thành quãng đường từng bánh xe
  float D_R = (deltaPulse_R / (PULSES_PER_REV * GEAR_RATIO)) * (2 * PI * WHEEL_RADIUS);
  float D_L = (deltaPulse_L / (PULSES_PER_REV * GEAR_RATIO)) * (2 * PI * WHEEL_RADIUS);

  // Tính quãng đường trung bình
  float D_C = (D_R + D_L) / 2.0;
  float delta_theta = (D_R - D_L) / 0.32;  // Thay đổi góc quay (rad)

  // Cập nhật vị trí (dựa trên phương trình odometry)
  x += D_C * cos(theta);
  y += D_C * sin(theta);
  theta += delta_theta;

  // Giới hạn góc theta trong khoảng 0 - 360
  if (theta > 2 * PI) theta -= 2 * PI;
  if (theta < 0) theta += 2 * PI;

  lastPulse_R = pulseCount_R;
  lastPulse_L = pulseCount_L;
}

void sendOdometryData() {
  String message = "pose:" + String(x*100, 2) + "," + String(y*100, 2) + "," + String(theta* 180 / PI, 2);
  ws.textAll(message);  // Gửi dữ liệu đến tất cả client kết nối
}

void GoToPoint(float goal_x, float goal_y) {
  // Khoảng cách đến point
  float distance_error = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));

  if (distance_error > 0.05) {  // Nếu còn xa đích thì tiến tới
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      speed_R = 30;
      speed_L = 30;
    } else {  // Nếu gần đích thì dừng
        speed_R = 0;
        speed_L = 0;
        ledcWrite(EN1, 0);
        ledcWrite(EN2, 0);
    }
}