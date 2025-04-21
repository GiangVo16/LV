[rows, cols] = size(MAP);
image_height = rows;  % Chiều cao ảnh
x = break_points(:,2);  % Cột giữ nguyên
y = image_height - break_points(:,1);  % Hàng đổi theo công thức

converted_points = [x, y];  % Ma trận mới chứa tọa độ (x, y)

disp(converted_points);  % Hiển thị tọa độ mới
