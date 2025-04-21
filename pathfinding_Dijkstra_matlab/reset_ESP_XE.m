clear;
%% Truyền x,y,z = 0 0 0
% Cấu hình udpport: Chọn LocalHost là IP của máy tính chạy MATLAB
esp = udpport("datagram", "IPV4", LocalHost="192.168.50.218", LocalPort=4321);
esp_ip = "192.168.50.176";  % Địa chỉ ESP32
esp_port = 1234;

% Gửi lệnh "RESET" đến ESP32
reset_cmd = "RESET";
write(esp, reset_cmd, "string", esp_ip, esp_port);

% Hiển thị thông báo trên MATLAB
disp("Đã reset ESP32.");