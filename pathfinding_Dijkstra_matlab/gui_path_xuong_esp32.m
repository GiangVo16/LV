% ✅ Cấu hình Serial Port
serialPort = "COM8";  % ⚠️ Thay bằng cổng thực tế của ESP32
baudRate = 115200;
s = serialport(serialPort, baudRate);

% ✅ Mảng tọa độ 13x2
path = [1.0 2.0; 2.1 3.2; 3.2 4.3; 4.3 5.4; 5.4 6.5;
        6.5 7.6; 7.6 8.7; 8.7 9.8; 9.8 10.9; 10.9 11.0;
        11.0 12.1; 12.1 13.2; 13.2 14.3];

% ✅ Gửi từng tọa độ & chờ ESP32 phản hồi trước khi gửi tiếp
disp("Sending coordinates...");
for i = 1:size(path,1)
    coordStr = sprintf("X%.2f,Y%.2f\n", path(i,1), path(i,2));
    writeline(s, coordStr); % Gửi dữ liệu
    
    % ✅ Đợi ESP32 phản hồi, tránh tràn buffer
    response = readline(s);
    disp("ESP32 Response: " + response);

    pause(0.2);  % ✅ Thêm delay 200ms để ESP32 kịp xử lý
end

disp("All coordinates sent.");
clear s; % Đóng cổng Serial
