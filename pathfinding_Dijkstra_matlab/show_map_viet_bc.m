clc
clear
%% Tải map từ ma trận nhị phân
A = readmatrix("grid_map_300x450_new.txt");   % Đọc ma trận
binMap = A;       % Lấy bản đồ nhị phân từ map2.mat
rate_map = 50;  % 1m = 50px
safe_distance = 0.4;    % 0.3m = 30cm
safe_distance = safe_distance*rate_map;

figure;
imagesc(~binMap);  % Hiển thị bản đồ với các chướng ngại vật
colormap(gray);  % Đặt màu cho bản đồ
title('Grid map');
grid on;
hold on;

waypoints = [150 490; 268 46; 400 200; 828 306];
% Vẽ các điểm
for i = 1:size(waypoints, 1)
    plot(waypoints(i, 1)/2, waypoints(i, 2)/2, 'bx', 'MarkerSize', 10, 'LineWidth', 2); % Đánh dấu điểm trung gian
end
xlabel('Cột');           % Đặt tên trục X
ylabel('Hàng');           % Đặt tên trục Y

%% Các điểm đến
start_oxy = [1.5 1.1];
A_oxy = [2.68 5.54];
B_oxy = [5 4];
goal_oxy = [8.28 2.94];

% chuyen ve xy
points = [start_oxy(1) start_oxy(2); A_oxy(1) A_oxy(2); B_oxy(1) B_oxy(2); goal_oxy(1) goal_oxy(2)];

%% Phồng map
my_occupancy_map = binaryOccupancyMap(binMap, 50);  % Tạo occupancy map từ binMap
OccMapFL = copy(my_occupancy_map);
inflate(OccMapFL, 0.4);           % Inflate map tránh va chạm
threshold = 0.5;                % Ngưỡng cho chướng ngại vật
MAP = getOccupancy(OccMapFL) > threshold;   % Lấy binMap sau khi inflate

figure
OccMapFL.show
hold on
% Vẽ các điểm
for i = 1:size(points, 1)
    plot(points(i, 1), points(i, 2), 'bx', 'MarkerSize', 10, 'LineWidth', 2); % Đánh dấu điểm trung gian
end
grid on
hold off

%% Chuyển về tọa độ ảnh (matrix) để chạy thuật toán Dijsktra
[r_max, c_max] = size(binMap);
start = round([r_max - start_oxy(2)*rate_map start_oxy(1)*rate_map]);
A = round([r_max - A_oxy(2)*rate_map A_oxy(1)*rate_map]);
B = round([r_max - B_oxy(2)*rate_map B_oxy(1)*rate_map]);
goal = round([r_max - goal_oxy(2)*rate_map goal_oxy(1)*rate_map]);
a = 10;  % Chi phí dọc/ngang

%% Tìm đường đi qua các điểm trung gian
[path1, visited1] = dijkstraWithTraceback(MAP, start, A, a);
[path2, visited2] = dijkstraWithTraceback(MAP, A, B, a);
[path3, visited3] = dijkstraWithTraceback(MAP, B, goal, a);

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

min_distance = 0.35;  % Kc lọc 10cm
final_path = filter_coordinates(path, min_distance, points);
disp('path cuối cùng');
disp(final_path);

%% 
figure
my_occupancy_map.show
hold on
% Vẽ các điểm
for i = 1:size(points, 1)
    plot(points(i, 1), points(i, 2), 'bx', 'MarkerSize', 10, 'LineWidth', 2); % Đánh dấu điểm trung gian
end
grid on
hold off


% Tọa độ oxy
visualizePath_oxy(my_occupancy_map, final_path, points);

%% --------------------- HÀM CON ---------------------
function [path, visited] = dijkstraWithTraceback(MAP, start, goal, a)
    [rows, cols] = size(MAP);
    directions = [
        0,  1,  a;       % Phải
        0, -1,  a;       % Trái
        1,  0,  a;       % Xuống
       -1,  0,  a;       % Lên
        1,  1,  a*sqrt(2); % Chéo phải xuống
        1, -1,  a*sqrt(2); % Chéo trái xuống
       -1,  1,  a*sqrt(2); % Chéo phải lên
       -1, -1,  a*sqrt(2)  % Chéo trái lên
    ];
    
    dist = inf(rows, cols);  % Khởi tạo khoảng cách vô hạn
    dist(start(1), start(2)) = 0;
    visited = zeros(rows, cols);
    parent = zeros(rows, cols, 2);
    pq = [0, start];  % Hàng đợi ưu tiên
    
    while ~isempty(pq)
        pq = sortrows(pq, 1);  % Sắp xếp pq theo chi phí
        current = pq(1, 2:3);
        cost = pq(1, 1);
        pq(1, :) = [];
        
        if isequal(current, goal)
            break;
        end
        
        if visited(current(1), current(2))
            continue;
        end
        visited(current(1), current(2)) = 1;
        
        for i = 1:size(directions, 1)
            new_r = current(1) + directions(i, 1);
            new_c = current(2) + directions(i, 2);
            new_cost = cost + directions(i, 3);
            
            if new_r > 0 && new_r <= rows && new_c > 0 && new_c <= cols
                if MAP(new_r, new_c) == 0 && new_cost < dist(new_r, new_c)
                    dist(new_r, new_c) = new_cost;
                    parent(new_r, new_c, :) = current;
                    pq = [pq; new_cost, new_r, new_c];
                end
            end
        end
    end
    
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

