% Ví dụ ma trận điểm (n x 2)
path = [1.5 0; 0.9 0.9; 1.5 1.2; 0 1.5];  % Thay đổi theo dữ liệu thực tế

% Chuyển ma trận thành chuỗi định dạng CSV: mỗi dòng "x,y\n"
numPoints = size(path, 1);
lines = strings(numPoints, 1);
for i = 1:numPoints
    % Định dạng số với 2 chữ số thập phân
    lines(i) = sprintf('%.2f,%.2f', path(i,1), path(i,2));
end
dataStr = join(lines, "\n");  % Nối các dòng với ký tự xuống dòng
dataStr = dataStr + "\n";      % Đảm bảo có ký tự xuống dòng cuối cùng

% Cấu hình UDP Client (MATLAB R2020b trở lên sử dụng udpport)
remoteIP = '192.168.50.128';  % Thay bằng địa chỉ IP của ESP32
remotePort = 1234;          % Cổng mà ESP32 lắng nghe

u = udpport("datagram", "IPV4");
write(u, dataStr, "string", remoteIP, remotePort);

disp("Đã gửi dữ liệu:");
disp(dataStr);
