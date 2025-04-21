clear
%% Truyền Path cho ESP32
% Cấu hình udpport: Chọn LocalHost là IP của máy tính chạy MATLAB
esp = udpport("datagram", "IPV4", LocalHost="192.168.50.218", LocalPort=4321);
esp_ip = "192.168.50.176";  % Địa chỉ ESP32
esp_port = 1234;

targets = [2.1 1.2; -0.9 -0.9];

% Gửi JSON target points đến ESP32
jsonTargets = jsonencode(struct('TYPE', 'TARGETS', 'DATA', targets));
write(esp, jsonTargets, "char", esp_ip, 1234);
disp("Sent to ESP32: " + jsonTargets);
pause(0.5);


path = [0.9 0; 
        0.9 1.2;
        1.5 1.5;
        2.1 1.2;
        2.1 0;
        0 0];

% Gửi JSON path đến ESP32
jsonPath = jsonencode(struct('TYPE', 'PATH', 'DATA', path));
write(esp, jsonPath, "char", esp_ip, 1234);
disp("Sent to ESP32: " + jsonPath);




%% Nhận tọa độ x, y và theta vẽ robot di chuyển
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
pose = [];
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
    thetaMPU = str2double(tokens(8));

    
    newPose = [x y theta];
    pose = [pose; newPose];
    viz(newPose, path);
    
    % In ra các giá trị:
    fprintf('x = %.2f y = %.2f theta = %.2f thetaMPU = %.2f\n', x, y, theta*180/pi, thetaMPU);
    %disp(pose)
end