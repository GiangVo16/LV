clear;
% Cấu hình udpport: Chọn LocalHost là IP của máy tính chạy MATLAB
esp = udpport("datagram", "IPV4", LocalHost="192.168.1.8", LocalPort=111);

disp("Đang lắng nghe dữ liệu từ ESP32...");

while true
    received = read(esp, 1, "string");
    dataStr = received.Data;

    % Tách x, y, theta
    tokens = split(dataStr);  % Kết quả: ["x"; "37.50"; "y"; "30.30"; "theta"; "75.30"]
    
    % Giả sử định dạng luôn cố định:
    x = str2double(tokens(2));    % "37.50" -> 37.50
    y = str2double(tokens(4));    % "30.30" -> 30.30
    theta = str2double(tokens(6));% "75.30" -> 75.30
    
    % In ra các giá trị:
    fprintf('x = %.2f y = %.2f theta = %.2f\n', x, y, theta);
    %disp(pose)
    
    pause(0.1);
end
