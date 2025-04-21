% Cấu hình địa chỉ IP và cổng của ESP32 (lấy từ Serial Monitor của ESP32)
remoteIP = '192.168.1.12';  % Thay bằng địa chỉ IP của ESP32
remotePort = 1234;

% Tạo đối tượng UDP (MATLAB R2020b trở lên sử dụng udpport)
u = udpport("datagram", "IPV4");

% Gửi dữ liệu đến ESP32
dataToSend = "Hello ESP32";
write(u, dataToSend, "string", remoteIP, remotePort);
disp("Đã gửi dữ liệu đến ESP32");

% Chờ ESP32 phản hồi
pause(1);  % Tạm dừng 1 giây, có thể điều chỉnh nếu cần

if u.NumDatagramsAvailable > 0
    dataReceived = read(u, u.NumDatagramsAvailable, "string");
    disp("Phản hồi từ ESP32:");
    disp(dataReceived)
else
    disp("Không nhận được phản hồi từ ESP32");
end


% Đóng kết nối UDP (đối tượng udpport sẽ tự giải phóng tài nguyên khi clear)
clear u;
