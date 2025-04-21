ipAddress = '192.168.1.12';
port = 1234;

% Tạo đối tượng TCP Client
client = tcpclient(ipAddress, port);

% Gửi dữ liệu đến ESP32
data = 'Hello ESP32\n';

% THêm ký tự xuống dòng để server ESP32 dùng readString
write(client, data, 'char');

% Chờ ESP32 phản hồi
pause(2);
if client.NumBytesAvailable > 0
    dataReceived = char(read(client, client.NumBytesAvailable, 'char'));
    disp('Phản hồi từ ESP32: ');
    disp(dataReceived)

end

% Đóng kết nối
clear client;