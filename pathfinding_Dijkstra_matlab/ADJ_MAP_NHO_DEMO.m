clear;
clc;
%% Tải map từ ma trận nhị phân
A = load('map_data.mat');   % Đọc ma trận
binMap = A.map;
rate_map = 100;  % 1m = 100px
safe_distance = 0.3;    % 0.3m = 30cm
safe_distance = safe_distance*rate_map;  %

%% Phồng map
OccMap = binaryOccupancyMap(binMap);  % Tạo occupancy map từ binMap
OccMapFL = copy(OccMap);
inflate(OccMapFL, safe_distance);           % Inflate map tránh va chạm
threshold = 0.5;                % Ngưỡng cho chướng ngại vật
MAP = getOccupancy(OccMapFL) > threshold;   % Lấy binMap sau khi inflate

%% Xác định các điểm đến
% Hiển thị map chọn điểm đến
dis_map = binaryOccupancyMap(binMap, rate_map);
dis_map_fl = copy(dis_map);
inflate(dis_map_fl, safe_distance/rate_map);
dis_map_fl.show
xlabel('x [Met]');
ylabel('y [Met]');
grid on;

%[x, y] = ginput(4); % Lấy 4 điểm lần lược start -> A -> B -> goal

% Làm tròn tọa độ đến chữ số thập phân thứ nhất
%x = round(x, 1);
%y = round(y, 1);

% Gán vào 4 biến tạo độ
%start_oxy = [x(1) y(1)];
%A_oxy = [x(2) y(2)];
%B_oxy = [x(3) y(3)];
%goal_oxy = [x(4) y(4)];

start_oxy = [1.5 0];
A_oxy = [0.9 0.9];
B_oxy = [1.5 1.2];
goal_oxy = [0 1.5];


[r_max, c_max] = size(binMap);

% Chuyển về tọa độ ảnh (matrix) để chạy thuật toán Dijsktra
start = round([r_max - start_oxy(2)*rate_map start_oxy(1)*rate_map]);
A = round([r_max - A_oxy(2)*rate_map A_oxy(1)*rate_map]);
B = round([r_max - B_oxy(2)*rate_map B_oxy(1)*rate_map]);
goal = round([r_max - goal_oxy(2)*rate_map goal_oxy(1)*rate_map]);
a = 10;  % Chi phí dọc/ngang

%% Tìm đường đi qua các điểm trung gian
[path1, visited1] = dijkstraOptimized(MAP, start, A, a);
[path2, visited2] = dijkstraOptimized(MAP, A, B, a);
[path3, visited3] = dijkstraOptimized(MAP, B, goal, a);

%% Ghép các đoạn đường
full_path = [path1; path2(2:end,:); path3(2:end,:)];
disp('Đường đi qua các điểm:');
%disp(full_path);

%% Tìm điểm gãy của các đoạn thẳng
break_points = Break_points(full_path);

%% Chuyển điểm gãy sang tọa độ XY
[rows, cols] = size(MAP);
image_height = rows;  % Chiều cao ảnh
x = break_points(:,2);  % Cột giữ nguyên
y = image_height - break_points(:,1);  % Hàng đổi theo công thức
converted_points = [x, y];  % Ma trận mới chứa tọa độ (x, y)
disp('Các điểm gãy các đường thẳng (tọa độ xy');
disp(converted_points/rate_map);  % Hiển thị tọa độ mới

%% Lọc path
path = converted_points / rate_map;
points = [start_oxy(1) start_oxy(2); A_oxy(1) A_oxy(2); B_oxy(1) B_oxy(2); goal_oxy(1) goal_oxy(2)];
min_distance = 0.35;  % Kc lọc 10cm
final_path = filter_coordinates(path, min_distance, points);
disp('path cuối cùng');
disp(final_path);

%% Hiển thị kết quả
visualizePath(binMap, full_path, visited1 | visited2 | visited3, [A; B], break_points);

% Tọa độ oxy
visualizePath_oxy(dis_map, final_path, points);

%% Truyền Path cho ESP32
% Cấu hình udpport: Chọn LocalHost là IP của máy tính chạy MATLAB
esp = udpport("datagram", "IPV4", LocalHost="192.168.50.218", LocalPort=4321);
esp_ip = "192.168.50.151";  % Địa chỉ ESP32
esp_port = 1234;

% Gửi JSON tọa độ đến ESP32
json_data = jsonencode(final_path);
write(esp, json_data, "string", esp_ip, esp_port);
disp("Sent to ESP32: " + json_data);


%% Nhận tọa độ x, y và theta vẽ robot di chuyển
bando = occupancyMap(binMap, rate_map);

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'bando';
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

    
    newPose = [x y theta];
    pose = [pose; newPose];
    viz(newPose, points);
    
    % In ra các giá trị:
    fprintf('x = %.2f y = %.2f theta = %.2f\n', x, y, theta*180/pi);
    %disp(pose)
    
    pause(0.1);
end



%% --------------------- HÀM CON ---------------------
function [path, visited] = dijkstraOptimized(MAP, start, goal, a)
    [rows, cols] = size(MAP);
    directions = [
        0,  1,  a;        % Đi phải
        0, -1,  a;        % Đi trái
        1,  0,  a;        % Đi xuống
       -1,  0,  a;        % Đi lên
        1,  1,  a*sqrt(2); % Chéo phải xuống
        1, -1,  a*sqrt(2); % Chéo trái xuống
       -1,  1,  a*sqrt(2); % Chéo phải lên
       -1, -1,  a*sqrt(2)  % Chéo trái lên
    ];
    
    % Khởi tạo ma trận khoảng cách
    dist = inf(rows, cols);
    dist(start(1), start(2)) = 0;
    visited = zeros(rows, cols);
    parent = zeros(rows, cols, 2);
    direction_map = zeros(rows, cols, 2);

    % Hàng đợi ưu tiên (min-heap)
    pq = [0, start, 0, 0];  % [cost, row, col, prev_dir_x, prev_dir_y]

    while ~isempty(pq)
        % Lấy phần tử có cost nhỏ nhất
        pq = sortrows(pq, 1);
        current = pq(1, 2:3);
        cost = pq(1, 1);
        prev_dir = pq(1, 4:5);
        pq(1, :) = [];

        if isequal(current, goal)
            break;
        end
        
        if visited(current(1), current(2))
            continue;
        end
        visited(current(1), current(2)) = 1;

        % Duyệt qua tất cả hướng
        for i = 1:size(directions, 1)
            new_r = current(1) + directions(i, 1);
            new_c = current(2) + directions(i, 2);
            new_cost = cost + directions(i, 3);
            new_dir = directions(i, 1:2);
            
            % Kiểm tra đổi hướng (thêm hình phạt khi rẽ)
            if any(prev_dir ~= [0, 0])  
                if any(prev_dir ~= new_dir)  % Đổi hướng nhẹ
                    new_cost = new_cost + 0.2;
                end
                if abs(atan2d(new_dir(2), new_dir(1)) - atan2d(prev_dir(2), prev_dir(1))) >= 90  % Đổi hướng mạnh
                    new_cost = new_cost + 0.5;
                end
            else
                new_cost = new_cost - 0.1; % Ưu tiên đi thẳng
            end
            
            % Kiểm tra trong biên và đường hợp lệ
            if new_r > 0 && new_r <= rows && new_c > 0 && new_c <= cols
                if MAP(new_r, new_c) == 0 && new_cost < dist(new_r, new_c)
                    dist(new_r, new_c) = new_cost;
                    parent(new_r, new_c, :) = current;
                    direction_map(new_r, new_c, :) = new_dir;
                    pq = [pq; new_cost, new_r, new_c, new_dir];
                end
            end
        end
    end

    % Truy vết đường đi tối ưu
    path = [];
    current = goal;
    while ~isequal(current, start)
        path = [current; path];
        current = squeeze(parent(current(1), current(2), :))';
        if all(current == 0)
            path = [];
            break;
        end
    end
    path = [start; path];

    % Hậu xử lý: Xóa điểm dư thừa
    path = simplifyPath(path);
end

% 🛠 Hàm xóa điểm dư thừa trên đường đi
function path = simplifyPath(path)
    if isempty(path)
        return;
    end
    new_path = path(1, :);
    for i = 2:size(path, 1) - 1
        prev_dir = path(i, :) - path(i-1, :);
        next_dir = path(i+1, :) - path(i, :);
        if any(prev_dir ~= next_dir)  % Nếu đổi hướng, giữ lại điểm này
            new_path = [new_path; path(i, :)];
        end
    end
    new_path = [new_path; path(end, :)];  % Thêm điểm cuối
    path = new_path;
end


%% Hàm vẽ path lên bản đồ và các điểm gãy-----------------------------------
function visualizePath(MAP, path, visited, waypoints, break_points)
    figure;
    imagesc(~MAP);  % Hiển thị bản đồ với các chướng ngại vật
    colormap(gray);  % Đặt màu cho bản đồ
    hold on;
    
    % Vẽ các ô đã duyệt
    [r, c] = find(visited);
    plot(c, r, 'g.', 'MarkerSize', 10);
    
    % Vẽ đường đi
    if ~isempty(path)
        plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);  % Đoạn đường đi
        plot(path(1,2), path(1,1), 'bx', 'MarkerSize', 10, 'LineWidth', 2);  % Điểm bắt đầu
        plot(path(end,2), path(end,1), 'bx', 'MarkerSize', 10, 'LineWidth', 2);  % Điểm kết thúc
        
        % Vẽ các điểm trung gian
        for i = 1:size(waypoints, 1)
            plot(waypoints(i, 2), waypoints(i, 1), 'bx', 'MarkerSize', 10, 'LineWidth', 2); % Đánh dấu điểm trung gian
        end
    end
    
    % Vẽ các điểm gãy
    %if ~isempty(break_points)
    %    plot(break_points(:,2), break_points(:,1), 'ms', 'MarkerSize', 8, 'LineWidth', 2); % Các điểm gãy màu tím
    %end
    
    title('Dijkstra Pathfinding with Break Points');
    xlabel('Cột');
    ylabel('Hàng');
    axis equal;
    grid on;
    hold off;
end

%% Hàm vẽ đường đi theo hệ oxy
function visualizePath_oxy(map, path, points)
    figure
    map.show
    xlabel('x [Met]');
    ylabel('y [Met]');
    grid on;
    hold on
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);  % Đoạn đường đi
    % Vẽ các điểm
    for i = 1:size(points, 1)
        plot(points(i, 1), points(i, 2), 'bx', 'MarkerSize', 10, 'LineWidth', 2); % Đánh dấu điểm trung gian
    end
end
%% Hàm tìm điểm gãy của các đoạn thẳng--------------------------------------
function break_points = Break_points(path)
    coordinates = path;    % path
    
    % Tính toán sự chênh lệch giữa các điểm liên tiếp
    dx = diff(coordinates(:,1));  % Δx
    dy = diff(coordinates(:,2));  % Δy
    
    % Khởi tạo biến lưu trữ điểm đầu và cuối các đoạn
    start_points = coordinates(1, :);  % Điểm đầu tiên
    end_points = [];
    end_points = [start_points, end_points];
    
    % Duyệt qua các điểm để tìm điểm gãy
    for i = 2:length(dx)
        if dx(i) ~= dx(i-1) || dy(i) ~= dy(i-1)
            end_points = [end_points; coordinates(i, :)];  % Lưu điểm gãy
            start_points = [start_points; coordinates(i, :)];  % Điểm bắt đầu đoạn mới
        end
    end
    
    % Lưu điểm cuối cùng
    end_points = [end_points; coordinates(end, :)];
    
    % Trả về điểm gãy
    break_points = end_points;
end


%% Hàm tính độ dài và góc giữa các điểm gãy---------------------------------
function [segment_lengths, angles] = length_angle_d(break_points)
    % Tính độ dài và góc các đoạn thẳng
    num_points = size(break_points, 1);
    segment_lengths = zeros(num_points - 1, 1);  % Mảng lưu độ dài các đoạn
    angles = zeros(num_points - 1, 1);           % Mảng lưu góc các đoạn
    
    for i = 1:num_points-1
        x1 = break_points(i, 2);  % Cột của điểm i
        y1 = break_points(i, 1);  % Hàng của điểm i
        x2 = break_points(i+1, 2);  % Cột của điểm i+1
        y2 = break_points(i+1, 1);  % Hàng của điểm i+1
        
        % Tính khoảng cách Euclid giữa hai điểm
        segment_lengths(i) = sqrt((x2 - x1)^2 + (y2 - y1)^2);
        
        % Tính góc giữa hai điểm (theo độ)
        dx = x2 - x1;  % Chênh lệch cột
        dy = y2 - y1;  % Chênh lệch hàng
        
        % Tính góc theo công thức atan2 (góc tính theo độ)
        angle_rad = atan2(dy, dx);  % Góc theo radian
        angle_deg = rad2deg(angle_rad);  % Chuyển sang độ
        
        angles(i) = -angle_deg;
    end
end

%% Hàm lọc các điểm gãy
function filtered_coords = filter_coordinates(coords, min_distance, fixed_points)
    % Hàm lọc tọa độ sao cho khoảng cách giữa các điểm liên tiếp >= min_distance,
    % nhưng vẫn giữ lại các tọa độ trong danh sách fixed_points.
    %
    % Inputs:
    %   - coords: Ma trận Nx2 chứa danh sách tọa độ (x, y)
    %   - min_distance: Khoảng cách tối thiểu giữa các điểm
    %   - fixed_points: Ma trận Mx2 chứa danh sách tọa độ không được lọc
    %
    % Output:
    %   - filtered_coords: Ma trận chứa tọa độ sau khi lọc
    
    % Kiểm tra đầu vào hợp lệ
    if isempty(coords) || size(coords, 2) ~= 2
        error('Dữ liệu đầu vào phải là ma trận Nx2 chứa tọa độ (x, y).');
    end
    if ~isempty(fixed_points) && size(fixed_points, 2) ~= 2
        error('fixed_points phải là ma trận Mx2 chứa tọa độ (x, y).');
    end

    % Sắp xếp danh sách fixed_points theo thứ tự xuất hiện trong coords
    fixed_points = intersect(coords, fixed_points, 'rows', 'stable');
    
    % Khởi tạo danh sách đã lọc với điểm đầu tiên
    filtered_coords = coords(1, :);

    % Duyệt qua các điểm còn lại
    for i = 2:size(coords, 1)
        current_point = coords(i, :); % Lấy điểm hiện tại

        % Nếu điểm này nằm trong danh sách fixed_points, luôn thêm vào danh sách
        if ismember(current_point, fixed_points, 'rows')
            filtered_coords = [filtered_coords; current_point]; %#ok<AGROW>
            continue;
        end

        % Lấy điểm cuối trong danh sách đã lọc
        prev_point = filtered_coords(end, :);

        % Tính khoảng cách giữa hai điểm
        distance = sqrt((current_point(1) - prev_point(1))^2 + (current_point(2) - prev_point(2))^2);

        % Nếu khoảng cách >= min_distance, thêm điểm đó vào danh sách
        if distance >= min_distance
            filtered_coords = [filtered_coords; current_point]; 
        end
    end
end

