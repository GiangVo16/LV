#include <ESP32Encoder.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "header.h"

char incomingPacket[256];  // Bộ đệm nhận dữ liệu
float **path = nullptr;    // Mảng lưu tọa độ (động)
int num_points = 0;        // Số lượng điểm
int queu = 0;              // thứ tự các điểm
unsigned long lastSendTime = 0;
const unsigned long interval = 200;  // ms


// TARGETS (điểm đích) dùng mảng cố định
float target_points[10][2] = {999};
int num_targets = 0;
int vtrig = 0;

// Biến bộ đếm xung trong thư viện ESP32Encoder
int64_t pulseCount_R = 0;  // Số xung encoder bánh phải
int64_t pulseCount_L = 0;  // Số xung encoder bánh trái
long lastPulse_R = 0, lastPulse_L = 0;

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
double Kp1 = 5, Ki1 = 0.25, Kd1 = 0.0;  // Motor bên phải
double Kp2 = 5, Ki2 = 0.02, Kd2 = 0.0;  // Motor bên trái


// Tham số PID góc
float Kp_t = 7;
float Ki_t = 0.001;
float Kd_t = 0.01;
float theta_error;
float prev_theta_error = 0;
float integral_theta = 0;
const float omega = 2.2;  // Lệch 45 độ
const float ref_speed = 18;

// Vị trí đích
float target_x = 0.0;
float target_y = 0.0;
float distance_error;

// Cấu hình PWM
const int frequency = 1000;  // Tần số PWM 5kHz
const int resolution = 8;    // Độ phân giải 8-bit

MPU6050 mpu(Wire);
// Biến lưu góc theta (yaw)
float thetaMPU = 0.0;
unsigned long lastTimeMPU;
unsigned long lastTimeUpdateMPU;

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

void PID1Task(void *parameter) {
  for (;;) {
    // Đọc tốc độ và PID cho động cơ 1
    read_motorSpeed1();
    PID_1();
    vTaskDelay(sampleTime / portTICK_PERIOD_MS);
  }
}

void PID2Task(void *parameter) {
  for (;;) {
    // Đọc tốc độ và PID cho động cơ 2
    read_motorSpeed2();
    PID_2();
    vTaskDelay(sampleTime / portTICK_PERIOD_MS);
  }
}

void send_recv(void *parameter) {
  for (;;) {
    // Gửi dữ liệu lên matlab mỗi interval
    sendpose_matlab(x, y, theta, thetaMPU);

    // Nhận dữ liệu matlab
    int packetSize = udp.parsePacket();
    received_data(packetSize);

    vTaskDelay(interval / portTICK_PERIOD_MS);
  }
}

void Robot_control(void *parameter) {
  for (;;) {
    // Đọc góc theta bằng MPU
    read_thetaMPU();
    // Cập nhật theta dựa trên cảm biến MPU
    update_theta();
    // Đọc tọa độ robot (x, y, theta)
    Odometry();

    // NHẬN DỮ LIỆU TỪ CAMERA (qua serial laptop)
    SERIAL_RECV_CAMERA();


    if (STATUS_ROBOT == RUN_PATH) {
      // Khoảng cách đến point
      distance_error = sqrt(pow(path[queu][0] - x, 2) + pow(path[queu][1] - y, 2));  // Points[i].first là lấy x của phần tử thứ i
      // Điều khiển góc --------------------------------------------------------------------------
      float target_theta = atan2(path[queu][1] - y, path[queu][0] - x);
      theta_error = (target_theta)-radians(thetaMPU);
      if (theta_error > PI) theta_error -= 2 * PI;
      if (theta_error < -PI) theta_error += 2 * PI;
      float control_signal = PID_AngleControl(theta_error);
      // Nếu góc lỗi lớn hơn khoảng n độ và xa điểm đến thì quay robot hoặc đi lệch qua trái/phải
      if (abs(theta_error * 180 / PI) > 3 && distance_error > 0.06) {
        MotorControl(control_signal);
      } else {  // ĐI THẲNG ĐẾN ĐIỂM ĐẾN
        GoToPoint();  // Nếu đến đích sẽ chuyển sang trạng thái tìm vật gắp
      }
    }

    if (STATUS_ROBOT == FIND_OBJ) {
      FIND_OBJ_FUNCTION(); // TÌM KIẾM VÀ TIẾP CẬN ROBOT (nếu vật gần thì vào trạng thái chờ)
    }

    if (STATUS_ROBOT == WAIT) {
      // Không làm gì dừng chờ
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      speed_R = 0;
      speed_L = 0;
    }

    if (STATUS_ROBOT == DONE_PICKUP) {
      STATUS_ROBOT = RUN_PATH;
    }

    //debug
    //Serial.printf("status: %d, target_points[%.2f %.2f], path[%.2f %.2f]\n", STATUS_ROBOT, target_points[queu][0], target_points[queu][1], path[queu][0], path[queu][1]);
    
    // Serial.print("queu: ");
    // Serial.print(queu);
    // Serial.print(" Lỗi góc: ");
    // Serial.print(theta_error * 180 / PI);
    // Serial.print(" độ, Điều khiển góc: ");
    // Serial.println(control_signal);

    // Serial.print("x: ");
    // Serial.print(x * 100);
    // Serial.print(" y: ");
    // Serial.print(y * 100);

    // Serial.print(" theta: ");
    // Serial.print(theta * 180 / PI);
    // Serial.print(" thetaMPU: ");
    // Serial.println(thetaMPU);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void create_path() {
  // Cấp phát bộ nhớ cho path với 1 điểm
  path = (float **)malloc(sizeof(float *));  // Cấp phát bộ nhớ cho con trỏ cấp 2
  if (path == nullptr) {
    //Serial.println("Lỗi cấp phát path");
    return;
  }

  path[0] = (float *)malloc(2 * sizeof(float));  // Cấp phát 2 giá trị (x, y) cho path[0]
  if (path[0] == nullptr) {
    //Serial.println("Lỗi cấp phát path[0]");
    free(path);
    path = nullptr;
    return;
  }

  // Gán tọa độ mặc định (0,0)
  path[0][0] = x;
  path[0][1] = y;
  num_points = 1;

  // Kiểm tra bằng Serial
  // Serial.print("X0: ");
  // Serial.println(path[0][0]);
  // Serial.print("Y0: ");
  // Serial.println(path[0][1]);
  delay(1000);
}


void setup() {
  Serial.begin(115200);
  create_path();
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

  // Thiết lập IP tĩnh
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("⚠️ Cấu hình IP tĩnh thất bại!");
  }
  // Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
  delay(2000);  // delay để đọc local ip

  // Bắt đầu UDP server
  udp.begin(localUdpPort);
  Serial.print("UDP server is listening on port ");
  Serial.println(localUdpPort);

  // Thiết lập OUTPUT cho các chân IN1, IN2, IN3, IN4
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Thiết lập PWM cho các chân Enable của motor
  ledcAttach(EN1, frequency, resolution);
  ledcAttach(EN2, frequency, resolution);

  Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22 (ESP32)
  mpu.begin();
  mpu.calcGyroOffsets(true); // Tự động cân bằng gyro ban đầu
  Serial.println("\nMPU6050 sẵn sàng!");

  lastTimeMPU = millis();
  lastTimeUpdateMPU = millis();

  // Khởi tạo hai task cho hai động cơ-----------------------------------------
  xTaskCreatePinnedToCore(
    PID1Task,         // Hàm xử lý động cơ 1
    "Motor 1 Control",  // Tên task
    4096,               // Stack size
    NULL,               // Tham số truyền vào
    1,                  // Độ ưu tiên của task
    &motor1TaskHandle,  // Handle của task
    app_cpu             // CPU chạy task
  );

  xTaskCreatePinnedToCore(
    PID2Task,         // Hàm xử lý động cơ 2
    "Motor 2 Control",  // Tên task
    4096,               // Stack size
    NULL,               // Tham số truyền vào
    2,                  // Độ ưu tiên của task
    &motor2TaskHandle,  // Handle của task
    app_cpu             // CPU chạy task
  );

  xTaskCreatePinnedToCore(
    send_recv,           // Hàm xử lý động cơ 2
    "truyen_nhan_data",  // Tên task
    4096,                // Stack size
    NULL,                // Tham số truyền vào
    1,                   // Độ ưu tiên của task
    &motor2TaskHandle,   // Handle của task
    app_cpu              // CPU chạy task
  );

  xTaskCreatePinnedToCore(
    Robot_control,       // Hàm xử lý động cơ 2
    "Dieu khien robot",  // Tên task
    8192,                // Stack size
    NULL,                // Tham số truyền vào
    3,                   // Độ ưu tiên của task
    &motor2TaskHandle,   // Handle của task
    app_cpu              // CPU chạy task
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
  output_pwm1 = constrain(output_pwm1, 0, 200);  // Chặn trên và chặn dưới khoảng [0;255]
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
  output_pwm2 = constrain(output_pwm2, 0, 200);  // Chặn trên và chặn dưới khoảng [0;255]
  ledcWrite(EN2, output_pwm2);
  prev_error2 = error2;
}

// Hàm PID góc
float PID_AngleControl(float theta_error) {
  integral_theta += theta_error * sampleTime;
  integral_theta = constrain(integral_theta, -10, 10);  // Giới hạn tích phân
  float derivative_theta = (theta_error - prev_theta_error) / sampleTime;
  prev_theta_error = theta_error;
  return Kp_t * theta_error + Ki_t * integral_theta + Kd_t * derivative_theta;
}

// Hàm điều khiển robot
void MotorControl(float control_signal) {
  int speed_turn = (int)abs(control_signal) * 10;
  speed_turn = constrain(speed_turn, 10, 15);

  // Quay trái
  if (control_signal > omega * 2) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speed_R = speed_turn;
    speed_L = speed_turn;
  }
  // Quay phải
  else if (control_signal < -omega * 2) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_R = speed_turn;
    speed_L = speed_turn;
  }
  // Đi lệch trái Tốc độ bánh phải > bánh trái
  else if (control_signal > 0 && control_signal < omega / 10) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_R = ref_speed;
    speed_L = speed_R * 0.6;
  }
  // Cua góc trái Tốc độ bánh phải >> bánh trái
  else if (control_signal >= omega / 10 && control_signal <= omega * 2) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_R = ref_speed;
    speed_L = speed_R * 0;
  }
  // Đi lệch phải Tốc độ bánh trái > bánh phải
  else if (control_signal > -omega / 10 && control_signal < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_L = ref_speed;
    speed_R = speed_L * 0.6;
  }

  // Cua góc phải Tốc độ bánh trái >> bánh phải
  else if (control_signal >= -omega * 2 && control_signal <= -omega / 10) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_L = ref_speed;
    speed_R = speed_L * 0;
  }
}

void loop() {
  //--------------------------------------------------------------------------------------------
  // Serial.print("X_T: ");
  // Serial.print(path[queu][0] * 100);
  // Serial.print(" cm, Y_T: ");
  // Serial.print(path[queu][1] * 100);
  // Serial.print(" X: ");
  // Serial.print(x * 100);
  // Serial.print(" cm, Y: ");
  // Serial.print(y * 100);
  // Serial.print(" cm, Theta: ");
  // Serial.print(theta * 180 / PI);
  //Serial.print(" độ, Lỗi góc: ");
  //Serial.println(theta_error * 180 / PI);
  // Serial.print(" độ, Điều khiển góc: ");
  // Serial.print(control_signal);
  // Serial.printf(" distance_error: %.4f m", distance_error);
  // Serial.print("size path: ");
  // Serial.println(get_size_path());

  delay(1);
  //Serial.println(queu);
}

// Hàm nhập giá trị PID và trạng thái động cơ
void InputSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Đọc chuỗi đến khi gặp newline
    input.trim();                                 // Loại bỏ khoảng trắng đầu cuối

    // Tách chuỗi nhập vào thành các phần: lệnh và giá trị
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      //Serial.println("Invalid command! Use format: kp <value>, Ki <value>, Kd <value>, s1 <value>, s2 <value>, em1 <value>, em2 <value>");
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
    } else if (command == "s1") {
      speed_R = value;  // Gán giá trị cho tốc độ đặt động cơ 1
    } else if (command == "s2") {
      speed_L = value;  // Gán giá trị cho tốc độ đặt động cơ 2
    } else {
      // Kiểm tra và gán giá trị cho x và y nếu chúng được nhập cùng lúc
      int xIndex = input.indexOf("x");
      int yIndex = input.indexOf("y");

      if (xIndex != -1 && yIndex != -1) {
        // Nếu cả x và y đều có trong chuỗi, trích xuất giá trị
        target_x = input.substring(xIndex + 1, input.indexOf(" ", xIndex)).toDouble();
        target_y = input.substring(yIndex + 1).toDouble();

        // Serial.print("Target X: ");
        // Serial.println(target_x);
        // Serial.print("Target Y: ");
        // Serial.println(target_y);
      } else {
        // Serial.println("Invalid command! Use format: kp <value>, Ki <value>, Kd <value>, s1 <value>, s2 <value>, em1 <value>, em2 <value>");
      }
    }
  }
}

// Hàm định vị robot bằng encoder kết hợp MPU6050
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
  // float delta_theta = (D_R - D_L) / WHEEL_BASE;  // Thay đổi góc quay (rad)

  // Cập nhật vị trí (dựa trên phương trình odometry)
  x += D_C * cos(radians(thetaMPU));
  y += D_C * sin(radians(thetaMPU));
  // theta += delta_theta;

  // Giới hạn góc theta trong khoảng 0 - 360
  // if (theta > 2 * PI) theta -= 2 * PI;
  // if (theta < 0) theta += 2 * PI;

  lastPulse_R = pulseCount_R;
  lastPulse_L = pulseCount_L;
}

// Hàm đi thẳng đến point (nếu đến thì dừng chuyển trạng thái tìm vật)
void GoToPoint() {
  if (distance_error > 0.04) {  // Nếu còn xa đích thì tiến tới
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_R = ref_speed;
    speed_L = ref_speed;

  } 
  else {  // Nếu gần đích thì dừng
    speed_R = 0;
    speed_L = 0;
    ledcWrite(EN1, 0);
    ledcWrite(EN2, 0);

    // NẾU ĐÃ ĐẾN GẦN ĐIỂM ĐÍCH THÌ VÀO TRẠNG THÁI TÌM VẬT CẦN GẮP
    if(path[queu][0] == target_points[vtrig][0] && path[queu][1] == target_points[vtrig][1]) {
      vtrig = vtrig + 1;
      STATUS_ROBOT = FIND_OBJ;
    }


    // tiến vị trí tiếp theo (theo path)
    queu = queu + 1;
    if (queu >= get_size_path() - 1) {
      queu = get_size_path() - 1;
    }
    // if (queu >= 1) {  // VỊ TRÍ 0 KO CÓ VẬT
    //   // VÀO TRẠNG THÁI TÌM VẬT GẮP
    //   STATUS_ROBOT = FIND_OBJ;
    // }
    
  }
}

void sendpose_matlab(float a, float b, float c, float d) {
  // Sinh giá trị ngẫu nhiên cho 3 biến float với 2 chữ số thập phân
  // Tạo chuỗi dữ liệu định dạng: "x 37.50 y 30.30 theta 75.30\n"
  String data = "x " + String(a, 2) + " y " + String(b, 2) + " theta " + String(c, 2) + " thetaMPU " + String(d, 2) + "\n";

  // Gửi dữ liệu qua UDP tới MATLAB
  udp.beginPacket(matlabIP, matlabPort);
  udp.print(data);
  udp.endPacket();

  //Serial.print("Sent: ");
  //Serial.print(data);
}

// Hàm nhận chuỗi Json từ matlab
void received_data(int packetSize) {
  if (packetSize) {  //Nếu có dữ liệu gửi đến
    queu = 0;
    // Nhận dữ liệu JSON từ MATLAB
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';  // Đảm bảo chuỗi kết thúc
      //Serial.printf("Dữ liệu nhận: %s\n", incomingPacket);

      // Kiểm tra nếu nhận được lệnh RESET
      if (strcmp(incomingPacket, "RESET") == 0) {
        Serial.println("Reset ESP32!");
        ESP.restart();
        return;
      }

      // Phân tích json
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, incomingPacket);
      if (error) {
        Serial.println("Lỗi phân tích JSON!");
        return;
      }

      const char* type = doc["TYPE"];
      JsonArray data = doc["DATA"];

      if (strcmp(type, "PATH") == 0) {
        freeMemory();
        parseJsonArray(data);
        printpath();
      } else if (strcmp(type, "TARGETS") == 0) {
        parseTargetPoints(data);
      }
    }
  }
}

// Phân tích chuỗi JSON và lưu vào mảng 2 chiều động
void parseJsonArray(JsonArray arr) {
  num_points = arr.size();
  path = (float **)malloc(num_points * sizeof(float *));
  if (!path) return;

  for (int i = 0; i < num_points; i++) {
    path[i] = (float *)malloc(2 * sizeof(float));
    if (!path[i]) return;

    path[i][0] = arr[i][0];
    path[i][1] = arr[i][1];
  }
}

// Mảng TARGETS - cố định
void parseTargetPoints(JsonArray arr) {
  num_targets = min((int)arr.size(), 10);
  for (int i = 0; i < num_targets; i++) {
    target_points[i][0] = arr[i][0];
    target_points[i][1] = arr[i][1];
  }

  Serial.println("Danh sách điểm đích:");
  for (int i = 0; i < num_targets; i++) {
    Serial.printf("Target %d: x=%.2f, y=%.2f\n", i, target_points[i][0], target_points[i][1]);
  }
}

// Giải phóng bộ nhớ cũ
void freeMemory() {
  if (path) {
    for (int i = 0; i < num_points; i++) {
      free(path[i]);
    }
    free(path);
    path = nullptr;
  }
  num_points = 0;
}

// In danh sách tọa độ sau khi nhận
void printpath() {
  if (!path) {
    Serial.println("Không có dữ liệu path.");
    return;
  }
  Serial.println("Danh sách path mới:");
  for (int i = 0; i < num_points; i++) {
    Serial.printf("Point %d: (x=%.4f, y=%.4f)\n", i, path[i][0], path[i][1]);
  }
}

// Hàm trả về có bao nhiêu điểm
int get_size_path() {
  return num_points;
}

void read_thetaMPU() {
    mpu.update();
  // Tính thời gian giữa 2 lần đọc
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTimeMPU) / 1000.0; // Chuyển millis thành giây
  lastTimeMPU = currentTime;

  // Lấy giá trị gyro yaw (tốc độ góc quanh trục Z)
  float gyroZ = mpu.getGyroZ();

  // Tích phân tốc độ góc để tính theta
  if (fabs(gyroZ) > 0.05) {
    thetaMPU += gyroZ * dt;
  }
  
  // Giới hạn theta từ 0 đến 360 độ
  if (thetaMPU > 360) thetaMPU -= 360;
  if (thetaMPU < 0) thetaMPU += 360;

  // In kết quả
  // Serial.print("Theta (Yaw): ");
  // Serial.println(theta, 2); // 2 chữ số thập phân

  delay(1); // Đọc mỗi 10ms (1000Hz)
}

// Hàm cập nhật theta dựa trên cảm biến MPU mỗi n giây
void update_theta() {
  unsigned long currentTime = millis();
  if(currentTime - lastTimeUpdateMPU > 2000) {
    theta = radians(thetaMPU);
    lastTimeUpdateMPU = currentTime;
  }
}


// ---------------------------------------------------------------------------------------
// ↓ HÀM LIÊN QUAN ĐẾN TAY MÁY ↓

// Hàm chạy tìm và tiếp cận vật gắp
void FIND_OBJ_FUNCTION() {
  // QUAY VÒNG TÌM VẬT ĐẾN KHI THẤY
  if(detect_obj == 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_R = 15;
    speed_L = 15;
  } else {  // detect_obj == 1
    // NẾU PHÁT HIỆN VẬT CẦN GẮP THÌ TIẾP CẬN NÓ
    // Nếu khoảng cách đến vật gắp >= 0.5m và góc lệch < 3 độ thì tiến đến vật
    if(x_obj >= DISTANCE_OK_TO_OBJ && abs(angle_obj) <= 2) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_R = ref_speed;
    speed_L = ref_speed;
    } 
    // Nếu kc đến vật gắp >= 0.5m và nằm bến trái camera thì đi lệch trái (speed bánh phải > bánh trái)
    else if(x_obj >= DISTANCE_OK_TO_OBJ && angle_obj < -2) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      speed_R = ref_speed;
      speed_L = 0.5*speed_R;
    }
    // Nếu kc đến vật gắp >= 0.5m và nằm bến phải camera thì đi lệch trái (speed bánh phải < bánh trái)
    else if(x_obj >= DISTANCE_OK_TO_OBJ && angle_obj > 2) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      speed_L = ref_speed;
      speed_R = 0.5* speed_L;
    }
    // NẾU kc đến vật gắp < 0.5m thì dừng xe vào trạng thái chờ gắp vật
    else if (x_obj < DISTANCE_OK_TO_OBJ) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      speed_L = 0;
      speed_R = 0;
      STATUS_ROBOT = WAIT;
    }
  }
}

void SERIAL_RECV_CAMERA() {
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      inputData.trim();

      //Serial.print("Dữ liệu nhận: ");
      //Serial.println(inputData);

    // Nếu là lệnh DONE_PICKUP
    if (inputData == "DONE_PICKUP") {
      Serial.println("ĐÃ NHẬN DONE_PICKUP");
      STATUS_ROBOT = DONE_PICKUP;
      Serial.println(STATUS_ROBOT);
      inputData = "";
      return;
    }

      // Tách chuỗi
      int firstComma = inputData.indexOf(',');
      int secondComma = inputData.indexOf(',', firstComma + 1);
      int thirdComma = inputData.indexOf(',', secondComma + 1);

      if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
        String sx = inputData.substring(0, firstComma);
        String sy = inputData.substring(firstComma + 1, secondComma);
        String sangle = inputData.substring(secondComma + 1, thirdComma);
        String sdetected = inputData.substring(thirdComma + 1);

        if (sx == "-1" && sy == "-1" && sangle == "None" && sdetected == "-1") {
          //Serial.println("Không phát hiện vật thể nào.");
          x_obj = y_obj = angle_obj = 0;
          detect_obj = 0;
        } else {
          // Gán giá trị vào biến
          x_obj = sx.toFloat();
          y_obj = sy.toFloat();
          angle_obj = sangle.toFloat();
          detect_obj = sdetected.toInt();

          // In ra để kiểm tra
          // Serial.println("Đã phát hiện vật thể:");
          // Serial.print("x_obj = "); Serial.println(x_obj);
          // Serial.print("y_obj = "); Serial.println(y_obj);
          // Serial.print("angle_obj = "); Serial.println(angle_obj);
          // Serial.print("detect_obj = "); Serial.println(detect_obj);
        }
      } else {
        //Serial.println("Chuỗi dữ liệu không hợp lệ!");
      }

      inputData = ""; // Reset lại
    } else {
      inputData += inChar;
    }
  }
}