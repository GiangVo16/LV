// CHO ĐỘNG CHO QUAY BAO NHIÊU ĐỘ
#include <Wire.h>
#include <TCA9548.h>
#include <AS5600.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include "header.h"

// const char* ssid = "Phong IOT";
// const char* password = "@thayHieucatroi";

// // Cấu hình IP tĩnh
// IPAddress local_IP(192, 168, 1, 179);      // IP bạn muốn gán cho ESP32
// IPAddress gateway(192, 168, 1, 1);         // Thường là địa chỉ router
// IPAddress subnet(255, 255, 255, 0);        // Subnet mask
// IPAddress primaryDNS(8, 8, 8, 8);          // DNS chính (Google)
// IPAddress secondaryDNS(8, 8, 4, 4);        // DNS phụ (Google)

// // Địa chỉ IP của máy tính đang chạy MATLAB (cần cố định IP hoặc dùng IP tĩnh)
// const char* host = "192.168.1.177";  // <-- thay bằng IP máy tính
// const uint16_t port = 1234;          // Port tuỳ chọn (phải khớp với MATLAB)

// Dùng WiFi điện thoại
const char* ssid = "K50U";
const char* password = "19051890";

IPAddress local_IP(192, 168, 50, 179);      // IP bạn muốn gán cho ESP32
IPAddress gateway(192, 168, 50, 43);         // Thường là địa chỉ router
IPAddress subnet(255, 255, 255, 0);        // Subnet mask
IPAddress primaryDNS(8, 8, 8, 8);          // DNS chính (Google)
IPAddress secondaryDNS(8, 8, 4, 4);        // DNS phụ (Google)

// Địa chỉ IP của máy tính đang chạy MATLAB (cần cố định IP hoặc dùng IP tĩnh)
const char* host = "192.168.50.218";  // <-- thay bằng IP máy tính
const uint16_t port = 1234;          // Port tuỳ chọn (phải khớp với MATLAB)


WiFiServer server(8888);
WiFiClient client;

Servo servo2;

// Khai báo địa chỉ - đối tượng as5600 và module TCA9548
TCA9548 MP(0x70);        // Địa chỉ của TCA9548
AMS_5600 ams5600;        // Cảm biến AS5600
uint8_t Address = 0x36;  // Địa chỉ của AS5600
const int kenh_0 = 0;
const int kenh_1 = 1;
const int kenh_2 = 2;  // Kênh I2C mà AS5600 kết nối
const int kenh_3 = 3;

// Biến góc hiện tại
float curr_angle_0 = 0;
float curr_angle_1 = 0;
float curr_angle_2 = 0;
float curr_angle_3 = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Thiết lập IP tĩnh
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("⚠️ Cấu hình IP tĩnh thất bại!");
  }

  // Khởi động Wifi
  WiFi.begin(ssid, password);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi đã kết nối!");
  Serial.print("Địa chỉ IP: ");
  Serial.println(WiFi.localIP());
  server.begin();
  delay(1000);

  // Khởi động TCA9548
  Serial.println("Khởi động TCA9548...");
  if (!MP.begin()) {
    Serial.println("Không tìm thấy TCA9548!");
    while (1)
      ;
  }

  // CẤU HÌNH OUTPUT CHO CÁC CHÂN ĐIỀU KHIỂN DRIVER
  pinMode(STEP0_PIN, OUTPUT);
  pinMode(DIR0_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(STEP3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);

  pinMode(ENA_1, OUTPUT);
  pinMode(ENA_3, OUTPUT);

  pinMode(PHANH_1, OUTPUT);
  pinMode(PHANH_3, OUTPUT);
  digitalWrite(PHANH_1, LOW);  // BẬT PHANH 1
  digitalWrite(PHANH_3, LOW);  // BẬT PHANH 1

  servo2.attach(SERVO_2);
  servo2.write(0);
}

// Hàm lấy góc hiệu chỉnh
float getCalibratedAngle(int chanel, float angle_calib) {
  // Lấy góc đã hiệu chỉnh
  if (MP.isConnected(Address) && ams5600.detectMagnet() == 1) {
    int raw_angle = ams5600.getRawAngle();  // góc thô (giá trị từ 0 - 4095)
    float angle = (raw_angle / 4096.0) * 360.0;
    // Điều chỉnh về góc gốc (180.24° là 0° mới)
    float calibratedAngle = angle - angle_calib;
    if (calibratedAngle < 0) {
      calibratedAngle += 360.0;
    }
    return calibratedAngle;
  } else {
    return -1;
  }
}

// HÀM CHẠY ĐỘNG CƠ BƯỚC
void runMotor(int dir, int stepSpeed, int dirPin, int stepPin) {
  digitalWrite(dirPin, dir);  // Enables the motor to move in a particular direction
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepSpeed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepSpeed);
}


// Các hàm chạy khớp version 1-----------------------------------------------------------------------------------------------------------------------------------------------------
// Hàm chọn chiều quay  (CW: cùng chiều kim đồng hồ)
Direction getTurnDirection(float curr_angle, float target_angle) {
  float delta = fmod((target_angle - curr_angle + 360), 360);
  return (delta <= 180) ? CCW : CW;
}

// Hàm kiểm tra có trong vùng hoạt động không
bool isInAllowedZone(float angle, float min_angle, float max_angle) {
  angle = fmod(angle + 360, 360);
  return (angle <= min_angle || angle >= max_angle);
}

void RUN_KHOP_V1(int kenh, int dirPin, int stepPin, int refspeed, float curr_angle, float targetAngle, float min_angle, float max_angle, int PHANH, float angle_calib) {
  Direction direction = getTurnDirection(curr_angle, targetAngle);
  if (isInAllowedZone(targetAngle, min_angle, max_angle)) {
    if (direction == CCW) {
      // Quay ngược chiều kim đồng hồ
      while (abs(targetAngle - curr_angle) >= threshold) {
        // Liên tục đọc góc hiện tại
        if (MP.isConnected(Address) && ams5600.detectMagnet() == 1) {
          curr_angle = getCalibratedAngle(kenh, angle_calib);
        }
        // Kiểm tra giới hạn góc quay
        if (curr_angle >= min_angle && curr_angle <= max_angle) {
          Serial.printf("Vượt giới hạn! Dừng động cơ %d\n", kenh);
          return;
        }
        
        // ENABLE DRIVER ĐỐI VỚI STEP 1 và 3
        if (kenh == 1) digitalWrite(ENA_1, LOW);  // CHO PHÉP ĐỘNG CƠ CHẠY
        if (kenh == 3) digitalWrite(ENA_3, LOW);  // CHO PHÉP ĐỘNG CƠ CHẠY
        // TẮT PHANH CHẠY ĐỘNG CƠ
        digitalWrite(PHANH, HIGH);
        runMotor(1, refspeed, dirPin, stepPin);

        // DEBUG
        // Serial.printf("kenh %d, DK2 ", kenh);
        // Serial.print(refspeed);
        // Serial.print(" ");
        // Serial.println(curr_angle);
      }
      // ĐÃ ĐẾN TARGET ANGLE -> DỪNG ĐỘNG CƠ (KHÔNG GỌI HÀM runMotor nửa)
      // BẬT PHANH
      digitalWrite(PHANH, LOW);
      delay(500);
      // DISABLE DRIVER DỪNG BẰNG PHANH ĐỐI VỚI STEP 1 và 3
      //if (kenh == 1) digitalWrite(ENA_1, HIGH);
      if (kenh == 3) digitalWrite(ENA_3, HIGH);
    } else {
      // Quay cùng chiều kim đồng hồ
      while (abs(targetAngle - curr_angle) >= threshold) {
        // Liên tục đọc góc hiện tại
        if (MP.isConnected(Address) && ams5600.detectMagnet() == 1) {
          curr_angle = getCalibratedAngle(kenh, angle_calib);
        }

        // Kiểm tra giới hạn góc quay
        if (curr_angle >= min_angle && curr_angle <= max_angle) {
          Serial.printf("Vượt giới hạn! Dừng động cơ %d\n", kenh);
          return;
        }

        if (kenh == 1) digitalWrite(ENA_1, LOW);  // CHO PHÉP CHẠY
        if (kenh == 3) digitalWrite(ENA_3, LOW);
        // TẮT PHANH CHẠY ĐỘNG CƠ
        digitalWrite(PHANH, HIGH);

        runMotor(0, refspeed, dirPin, stepPin);

        // DEBUG
        // Serial.printf("kenh %d, DK2 ", kenh);
        // Serial.print(refspeed);
        // Serial.print(" ");
        // Serial.println(curr_angle);
      }
      // ĐÃ ĐẾN TARGET ANGLE -> DỪNG ĐỘNG CƠ
      // BẬT PHANH
      digitalWrite(PHANH, LOW);
      delay(500);
      //if (kenh == 1) digitalWrite(ENA_1, HIGH);
      if (kenh == 3) digitalWrite(ENA_3, HIGH);
    }
  } else {
    Serial.printf("TARGET ANGLE %d (%.2f) NGOÀI VÙNG HOẠT ĐỘNG!", kenh, targetAngle);
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void RUN_ROBOT(float targetAngle_0, float targetAngle_1, float targetAngle_2, float targetAngle_3) {
  //RUN KHỚP 0----------------------------------------------------------------------------------------------------------------------
  //CHỌN KÊNH 0
  MP.selectChannel(kenh_0);
  delay(5);
  // LẤY GÓC 0 HIỆN TẠI
  curr_angle_0 = getCalibratedAngle(kenh_0, angle_calib_0);
  //Serial.printf("Angle 0: %.2f\n", curr_angle_0);
  // QUAY ĐỘNG CƠ 0
  RUN_KHOP_V1(kenh_0, DIR0_PIN, STEP0_PIN, speed0, curr_angle_0, targetAngle_0, min_angle_0, max_angle_0, PHANH_0, angle_calib_0);


  // RUN KHỚP 2----------------------------------------------------------------------------------------------------------------------
  // ĐỌC GÓC THỨ 2
  MP.selectChannel(kenh_2);
  delay(5);
  // LẤY GÓC 1 HIỆN TẠI
  curr_angle_2 = getCalibratedAngle(kenh_2, angle_calib_2);
  //Serial.printf("Angle 2: %.2f\n", curr_angle_2);
  // QUAY ĐỘNG CƠ 2
  RUN_KHOP_V1(kenh_2, DIR2_PIN, STEP2_PIN, speed2, curr_angle_2, targetAngle_2, min_angle_2, max_angle_2, PHANH_2, angle_calib_2);

  // RUN KHỚP 3----------------------------------------------------------------------------------------------------------------------
  // ĐỌC GÓC THỨ 3
  MP.selectChannel(kenh_3);
  delay(5);
  // LẤY GÓC 1 HIỆN TẠI
  curr_angle_3 = getCalibratedAngle(kenh_3, angle_calib_3);
  //Serial.printf("Angle 3: %.2f\n", curr_angle_3);
  // QUAY ĐỘNG CƠ 1
  RUN_KHOP_V1(kenh_3, DIR3_PIN, STEP3_PIN, speed3, curr_angle_3, targetAngle_3, min_angle_3, max_angle_3, PHANH_3, angle_calib_3);

  
  // RUN KHỚP 1----------------------------------------------------------------------------------------------------------------------
  // ĐỌC GÓC THỨ 1
  MP.selectChannel(kenh_1);
  delay(5);
  // LẤY GÓC 1 HIỆN TẠI
  curr_angle_1 = getCalibratedAngle(kenh_1, angle_calib_1);
  //Serial.printf("Angle 1: %.2f\n", curr_angle_1);
  // QUAY ĐỘNG CƠ 1
  RUN_KHOP_V1(kenh_1, DIR1_PIN, STEP1_PIN, speed1, curr_angle_1, targetAngle_1, min_angle_1, max_angle_1, PHANH_1, angle_calib_1);

}

void RUN_ROBOT_HOME(float targetAngle_0, float targetAngle_1, float targetAngle_2) {
  //RUN KHỚP 0----------------------------------------------------------------------------------------------------------------------
  //CHỌN KÊNH 0
  MP.selectChannel(kenh_0);
  delay(5);
  // LẤY GÓC 0 HIỆN TẠI
  curr_angle_0 = getCalibratedAngle(kenh_0, angle_calib_0);
  //Serial.printf("Angle 0: %.2f\n", curr_angle_0);
  // QUAY ĐỘNG CƠ 0
  RUN_KHOP_V1(kenh_0, DIR0_PIN, STEP0_PIN, speed0, curr_angle_0, targetAngle_0, min_angle_0, max_angle_0, PHANH_0, angle_calib_0);


  // RUN KHỚP 2----------------------------------------------------------------------------------------------------------------------
  // ĐỌC GÓC THỨ 2
  MP.selectChannel(kenh_2);
  delay(5);
  // LẤY GÓC 1 HIỆN TẠI
  curr_angle_2 = getCalibratedAngle(kenh_2, angle_calib_2);
  //Serial.printf("Angle 2: %.2f\n", curr_angle_2);
  // QUAY ĐỘNG CƠ 2
  RUN_KHOP_V1(kenh_2, DIR2_PIN, STEP2_PIN, speed2, curr_angle_2, targetAngle_2, min_angle_2, max_angle_2, PHANH_2, angle_calib_2);


  // RUN KHỚP 1----------------------------------------------------------------------------------------------------------------------
  // ĐỌC GÓC THỨ 1
  MP.selectChannel(kenh_1);
  delay(5);
  // LẤY GÓC 1 HIỆN TẠI
  curr_angle_1 = getCalibratedAngle(kenh_1, angle_calib_1);
  //Serial.printf("Angle 1: %.2f\n", curr_angle_1);
  // QUAY ĐỘNG CƠ 1
  RUN_KHOP_V1(kenh_1, DIR1_PIN, STEP1_PIN, speed1, curr_angle_1, targetAngle_1, min_angle_1, max_angle_1, PHANH_1, angle_calib_1);
}

// Hàm đọc các góc chưa offset về 0
void read_raw_angle() {
  // DEBUG đọc góc chưa calib (đọc kênh nào thay vào) ------------------------------------------------------------------------------------------------------------------------------
  MP.selectChannel(kenh_0);
  delay(5);
  if (MP.isConnected(Address) && ams5600.detectMagnet() == 1) {
    int raw_angle = ams5600.getRawAngle();  // góc thô (giá trị từ 0 - 4095)
    curr_angle_0 = (raw_angle / 4096.0) * 360.0;
  } else curr_angle_0 = -1;
  MP.selectChannel(kenh_1);
  delay(5);
  if (MP.isConnected(Address) && ams5600.detectMagnet() == 1) {
    int raw_angle = ams5600.getRawAngle();  // góc thô (giá trị từ 0 - 4095)
    curr_angle_1 = (raw_angle / 4096.0) * 360.0;
  } else curr_angle_1 = -1;
  MP.selectChannel(kenh_2);
  delay(5);
  if (MP.isConnected(Address) && ams5600.detectMagnet() == 1) {
    int raw_angle = ams5600.getRawAngle();  // góc thô (giá trị từ 0 - 4095)
    curr_angle_2 = (raw_angle / 4096.0) * 360.0;
  } else curr_angle_2 = -1;
  MP.selectChannel(kenh_3);
  delay(5);
  if (MP.isConnected(Address) && ams5600.detectMagnet() == 1) {
    int raw_angle = ams5600.getRawAngle();  // góc thô (giá trị từ 0 - 4095)
    curr_angle_3 = (raw_angle / 4096.0) * 360.0;
  } else curr_angle_3 = -1;
  Serial.printf("%.2f | %.2f | %.2f | %.2f\n", curr_angle_0, curr_angle_1, curr_angle_2, curr_angle_3);
  // -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
}

void read_angle_offseted() {
  MP.selectChannel(kenh_0);
  delay(10);
  // LẤY GÓC 0 HIỆN TẠI
  curr_angle_0 = getCalibratedAngle(kenh_0, angle_calib_0);

  MP.selectChannel(kenh_1);
  delay(10);
  // LẤY GÓC 1 HIỆN TẠI
  curr_angle_1 = getCalibratedAngle(kenh_1, angle_calib_1);

  MP.selectChannel(kenh_2);
  delay(10);
  // LẤY GÓC 2 HIỆN TẠI
  curr_angle_2 = getCalibratedAngle(kenh_2, angle_calib_2);

  MP.selectChannel(kenh_3);
  delay(10);
  // LẤY GÓC 3 HIỆN TẠI
  curr_angle_3 = getCalibratedAngle(kenh_3, angle_calib_3);

  Serial.printf("%.2f | %.2f | %.2f | %.2f\n", curr_angle_0, curr_angle_1, curr_angle_2, curr_angle_3);
}

// Hàm tính toán động học thuận
void Dong_hoc_thuan(float t0, float t1, float t2, float t3) {
  if ((t0 > min_angle_0 && t0 < max_angle_0) && (t1 > min_angle_1 && t1 < max_angle_1) && (t2 > min_angle_2 && t2 < max_angle_2) && (t3 > min_angle_3 && t3 < max_angle_3)) {
    Serial.println("NGOÀI VÙNG LÀM VIỆC!!!");
  } else {
    RUN_ROBOT(t0, t1, t2, t3);
    t0 = t0 * PI / 180.0;
    t1 = t1 * PI / 180.0;
    t2 = t2 * PI / 180.0;
    t3 = t3 * PI / 180.0;
    x = a2*cos(t0)*cos(t1) + a3*cos(t0)*cos(t1)*cos(t2) - a3*cos(t0)*sin(t1)*sin(t2);
    y = a2*cos(t1)*sin(t0) + a3*cos(t1)*cos(t2)*sin(t0) - a3*sin(t0)*sin(t1)*sin(t2);
    z = d1 + a2*sin(t1) + a3*cos(t1)*sin(t2) + a3*cos(t2)*sin(t1);
    Serial.printf("x: %.3f m | y: %.3f m | z: %.3f m\n", x, y, z);
  }
}

// Hàm tính toán động học nghịch
void Dong_hoc_nghich(float x, float y, float z) {
  // Kiểm tra trong vùng làm việc !!! (CHƯA CODE)
  if (false) {  // VÍ DỤ TRONG VÙNG LÀM VIỆC
    return;
  }

  z = z + a4;
  goc_dat_0 = atan2(y, x);

  float s = z - d1;
  float r = sqrt(pow(x, 2) + pow(y, 2));
  float r1 = r - a4*cos(phi);
  float m = sqrt(pow(s, 2) + pow(r1, 2));

  float alpha1 = atan2(s, r1);
  float alpha2 = acos((pow(m, 2) + pow(a2, 2) - pow(a3, 2)) / (2 * m * a2));

  goc_dat_1 = alpha1 + alpha2;

  float beta = acos((pow(a2, 2) + pow(a3, 2) - pow(m, 2)) / (2 * a2 * a3));

  goc_dat_2 = -(PI - beta);

  goc_dat_3 = phi - (goc_dat_1 + goc_dat_2);  // CHƯA TÍNH ĐẾN KHỚP NÀY

  // ĐỔI RADIAN SANG ĐỘ và góc âm thành (0 - 360)
  goc_dat_0 = fmod((goc_dat_0 * 180.0 / PI + 360.0), 360.0);
  goc_dat_1 = fmod((goc_dat_1 * 180.0 / PI + 360.0), 360.0);
  goc_dat_2 = fmod((goc_dat_2 * 180.0 / PI + 360.0), 360.0);
  goc_dat_3 = fmod((goc_dat_3 * 180.0 / PI + 360.0), 360.0);

  RUN_ROBOT(goc_dat_0, goc_dat_1, goc_dat_2, goc_dat_3);
  Serial.printf("theta 0: %.2f° | theta 1: %.2f° | theta 2: %.2f° | theta 3: %.2f°\n", goc_dat_0, goc_dat_1, goc_dat_2, goc_dat_3);
}

// Hàm chọn mode chạy DHT hoặc DHN bằng serial (dht t0 15 t1 15 t2 15 t3 15)
void ROBOT_SERIAL() {
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if (input.startsWith("dht")) {
      Serial.println("CHỌN ĐỘNG HỌC THUẬN");
      if (input.indexOf("t0") != -1) {
        goc_dat_0 = input.substring(input.indexOf("t0") + 3).toFloat();
      }
      if (input.indexOf("t1") != -1) {
        goc_dat_1 = input.substring(input.indexOf("t1") + 3).toFloat();
      }
      if (input.indexOf("t2") != -1) {
        goc_dat_2 = input.substring(input.indexOf("t2") + 3).toFloat();
      }
      if (input.indexOf("t3") != -1) {
        goc_dat_3 = input.substring(input.indexOf("t3") + 3).toFloat();
      }

      Serial.print("theta 0: ");
      Serial.println(goc_dat_0);
      Serial.print("theta 1: ");
      Serial.println(goc_dat_1);
      Serial.print("theta 2: ");
      Serial.println(goc_dat_2);
      Serial.print("theta: ");
      Serial.println(goc_dat_3);

      Dong_hoc_thuan(goc_dat_0, goc_dat_1, goc_dat_2, goc_dat_3);  // Gọi hàm động học thuận
    } else if (input.startsWith("dhn")) {
      Serial.println("CHỌN ĐỘNG HỌC NGHỊCH");
      if (input.indexOf("x") != -1) {
        x = input.substring(input.indexOf("x") + 2).toFloat();
      }
      if (input.indexOf("y") != -1) {
        y = input.substring(input.indexOf("y") + 2).toFloat();
      }
      if (input.indexOf("z") != -1) {
        z = input.substring(input.indexOf("z") + 2).toFloat();
      }

      Serial.print("x = ");
      Serial.println(x);
      Serial.print("y = ");
      Serial.println(y);
      Serial.print("z = ");
      Serial.println(z);

      Dong_hoc_nghich(x, y, z);  // Gọi hàm động học nghịch
      delay(500);
      servo2.write(40);
      // IN RA TRẠNG THÁI CHẠY XONG ĐỘNG HỌC CHO PC
      Serial.println("KINEMATIC_DONE");

      // TỰ ĐỘNG GẤP VỀ
      delay(250);
      Serial.println("GIVE OBJ...");
      GIVE_OBJ();
      delay(250);
      GO_HOME();
      Serial.println("ARRIVED HOME");
    } else if(input == "h") {
      GO_HOME();
      Serial.println("ARRIVED HOME");
    } else if(input == "g") {
      Serial.println("GIVE OBJ...");
      GIVE_OBJ();
    } else {
      // Serial.println("Sai cú pháp! Ví dụ:");
      // Serial.println("  dht t0 30 t1 40");
      // Serial.println("  dhn x 1.2 y 3.4 z 2.5");
    }
  }
}

// Hàm chọn mode chạy DH bằng WiFi (matlab)
void ROBOT_WIFI() {
  if (!client || !client.connected()) {
    client = server.available();  // chờ client kết nối
    return;
  }

  if (client.available()) {
    input = client.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if (input.startsWith("dht")) {
      client.println("CHỌN ĐỘNG HỌC THUẬN");
      if (input.indexOf("t0") != -1) goc_dat_0 = input.substring(input.indexOf("t0") + 3).toFloat();
      if (input.indexOf("t1") != -1) goc_dat_1 = input.substring(input.indexOf("t1") + 3).toFloat();
      if (input.indexOf("t2") != -1) goc_dat_2 = input.substring(input.indexOf("t2") + 3).toFloat();
      if (input.indexOf("t3") != -1) goc_dat_3 = input.substring(input.indexOf("t3") + 3).toFloat();

      client.printf("Góc đặt 0: %.2f\n", goc_dat_0);
      client.printf("Góc đặt 1: %.2f\n", goc_dat_1);
      client.printf("Góc đặt 2: %.2f\n", goc_dat_2);
      client.printf("Góc đặt 3: %.2f\n", goc_dat_3);

      Dong_hoc_thuan(goc_dat_0, goc_dat_1, goc_dat_2, goc_dat_3);

    } else if (input.startsWith("dhn")) {
      client.println("CHỌN ĐỘNG HỌC NGHỊCH");
      if (input.indexOf("x") != -1) x = input.substring(input.indexOf("x") + 2).toFloat();
      if (input.indexOf("y") != -1) y = input.substring(input.indexOf("y") + 2).toFloat();
      if (input.indexOf("z") != -1) z = input.substring(input.indexOf("z") + 2).toFloat();

      client.printf("x = %.2f\ny = %.2f\nz = %.2f\n", x, y, z);
      Dong_hoc_nghich(x, y, z);
  
      servo2.write(40);
      // IN RA TRẠNG THÁI CHẠY XONG ĐỘNG HỌC CHO PC
      Serial.println("KINEMATIC_DONE");

      // TỰ ĐỘNG GẤP VỀ
      delay(10);
      Serial.println("GIVE OBJ...");
      GIVE_OBJ();
      delay(10);
      GO_HOME();
      Serial.println("ARRIVED HOME");
    } else if(input == "h") {
      GO_HOME();
      Serial.println("ARRIVED HOME");
    } else if(input == "g") {
      Serial.println("GIVE OBJ...");
      GIVE_OBJ();
    } else {
      // client.println("Sai cú pháp! Ví dụ:");
      // client.println("  dht t0 30 t1 40 t2 50 t3 60");
      // client.println("  dhn x 1.2 y 3.4 z 2.5");
    }
  }
}

void GIVE_OBJ() {
  digitalWrite(ENA_3, HIGH);  // DISABLE ĐỘNG CƠ 3
  // TẮT PHANH ĐỘNG CƠ 3
  digitalWrite(PHANH_3, HIGH);

  goc_dat_1 = 60;
  RUN_ROBOT_HOME(goc_dat_0, goc_dat_1, goc_dat_2);
  goc_dat_2 = 240;
  RUN_ROBOT_HOME(goc_dat_0, goc_dat_1, goc_dat_2);
  goc_dat_0 = 0; goc_dat_1 = 90;
  RUN_ROBOT_HOME(goc_dat_0, goc_dat_1, goc_dat_2);
  goc_dat_2 = 203;
  RUN_ROBOT_HOME(goc_dat_0, goc_dat_1, goc_dat_2);
  goc_dat_3 = 325;
  RUN_ROBOT_HOME(goc_dat_0, goc_dat_1, goc_dat_2);
  servo2.write(0);
}

// Hàm chạy về điểm home
void GO_HOME() {
  digitalWrite(ENA_3, HIGH);  // DISABLE ĐỘNG CƠ 3
  // TẮT PHANH ĐỘNG CƠ 3
  digitalWrite(PHANH_3, HIGH);

  Serial.println("GO HOME....");
  goc_dat_1 = 100;
  RUN_ROBOT_HOME(goc_dat_0, goc_dat_1, goc_dat_2);
  goc_dat_2 = 210; goc_dat_0 = 0;
  RUN_ROBOT_HOME(goc_dat_0, goc_dat_1, goc_dat_2);
  goc_dat_3 = 325;
  digitalWrite(ENA_3, LOW);
  RUN_ROBOT(goc_dat_0, goc_dat_1, goc_dat_2, goc_dat_3);

  servo2.write(0);
}

// Hàm gửi góc hiện tại cho matlab (nếu dùng wifi)
void send_angle_matlab() {
  if (client.connected()) {
    // Gửi dữ liệu theo định dạng CSV
    client.printf("%.2f,%.2f,%.2f,%.2f\n", curr_angle_0, curr_angle_1, curr_angle_2, curr_angle_3);
    delay(10); // gửi mỗi 100ms
  } else {
    Serial.println("⚠️ Mất kết nối với MATLAB, thử lại...");
    client.connect(host, port);
  }
}


void loop() {
  // Chạy về home 1 lần
  static int flag = 0;
  if (!flag) {
    Serial.println("GO HOME...");
    GO_HOME();
    Serial.println("SẴN SÀNG");
    flag = 1;
  }

  // Nhập và chọn mode động học
  ROBOT_SERIAL();
  ROBOT_WIFI();

  // Đọc góc hiện tại
  //read_angle_offseted();
  //send_angle_matlab();

  // Hàm đọc 4 góc chưa và đã offset
  //read_raw_angle();
  //read_angle_offseted();
}
