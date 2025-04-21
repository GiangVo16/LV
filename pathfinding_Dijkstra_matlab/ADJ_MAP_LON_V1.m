clear;
%% Tải map từ ma trận nhị phân
A = readmatrix("grid_map.txt");   % Đọc ma trận
binMap = A;       % Lấy bản đồ nhị phân từ map2.mat
rate_map = 10;  % 1m = 10px
safe_distance = 3;  %

%% Phồng map
OccMap = occupancyMap(binMap);  % Tạo occupancy map từ binMap
OccMapFL = copy(OccMap);
inflate(OccMapFL, safe_distance);           % Inflate map tránh va chạm
threshold = 0.5;                % Ngưỡng cho chướng ngại vật
MAP = getOccupancy(OccMapFL) > threshold;   % Lấy binMap sau khi inflate

% Cấu hình UDP (chỉ chạy 1 lần)
esp = udpport("datagram", "IPV4", LocalHost="192.168.1.3", LocalPort=4321);
esp_ip = "192.168.1.12";  % Địa chỉ ESP32
esp_port = 1234;

%% Bat dau chuong trinh
disp("Chọn điểm để bắt đầu...");
while true
    % Tạo Visualizer
    bando = occupancyMap(binMap, rate_map);
    viz = Visualizer2D;
    viz.hasWaypoints = true;
    viz.mapName = 'bando';

    %% Xác định các điểm đến
    % Hiển thị map để chọn điểm đến
    dis_map = occupancyMap(binMap, rate_map);
    dis_map_fl = copy(dis_map);
    inflate(dis_map_fl, safe_distance/rate_map);
    dis_map_fl.show
    xlabel('x [Met]');
    ylabel('y [Met]');
    grid on;

    [x, y] = ginput(4); % Lấy 4 điểm lần lược start -> A -> B -> goal
    x = round(x, 1); y = round(y, 1);

    % Gán tọa độ OXY
    start_oxy = [x(1) y(1)];
    A_oxy = [x(2) y(2)];
    B_oxy = [x(3) y(3)];
    goal_oxy = [x(4) y(4)];

    [r_max, c_max] = size(binMap);

    % Chuyển về tọa độ ảnh
    start = [r_max - start_oxy(2)*rate_map start_oxy(1)*rate_map];
    A = [r_max - A_oxy(2)*rate_map A_oxy(1)*rate_map];
    B = [r_max - B_oxy(2)*rate_map B_oxy(1)*rate_map];
    goal = [r_max - goal_oxy(2)*rate_map goal_oxy(1)*rate_map];

    %% Tìm đường đi bằng thuật toán Dijsktra
    a = 10;
    [path1, visited1] = dijkstraWithTraceback(MAP, start, A, a);
    [path2, visited2] = dijkstraWithTraceback(MAP, A, B, a);
    [path3, visited3] = dijkstraWithTraceback(MAP, B, goal, a);
    full_path = [path1; path2(2:end,:); path3(2:end,:)];

    %% Tìm điểm gãy & lọc path
    break_points = Break_points(full_path);
    x = break_points(:,2);
    y = r_max - break_points(:,1);
    converted_points = [x, y] / rate_map;  % Chuyển sang mét

    points = [start_oxy; A_oxy; B_oxy; goal_oxy];
    min_distance = 0.4; % 40cm
    final_path = filter_coordinates(converted_points, min_distance, points);

    %% Hiển thị kết quả đường đi
    visualizePath(binMap, full_path, visited1 | visited2 | visited3, [A, B], break_points);

    %% Gửi path cho ESP32
    json_data = jsonencode(final_path);
    write(esp, json_data, "string", esp_ip, esp_port);
    disp("Sent to ESP32: " + json_data);

    %% Nhận vị trí robot từ ESP32 và visualizer
    pose = [];
    disp("Đang lắng nghe dữ liệu từ ESP32...");
    while true
        received = read(esp, 1, "string");
        dataStr = received.Data;
        tokens = split(dataStr);
        
        % Xử lý dữ liệu vị trí
        x = str2double(tokens(2));    
        y = str2double(tokens(4));    
        theta = str2double(tokens(6));  
        newPose = [x y theta];
        pose = [pose; newPose];

        % Cập nhật visualizer
        viz(newPose, points);
        fprintf('x = %.2f y = %.2f theta = %.2f\n', x, y, theta);
        
        % Kiểm tra nếu robot đến goal
        if norm([x y] - goal_oxy) < 0.03  % Nếu khoảng cách đến goal < 3cm
            disp("Robot đã đến goal, sẵn sàng chọn điểm tiếp theo!");
            break; % Thoát khỏi vòng lặp theo dõi vị trí
        end
        pause(0.1);
    end
end  
    


%% --------------------- HÀM CON ------------------------------------------
% Hàm Tìm đường đi ngắn nhất DIJSKTRA trên bản đồ nhị phân
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

%% Hàm vẽ path lên bản đồ và các điểm gãy----------------------------------
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
        plot(path(1,2), path(1,1), 'bo', 'MarkerSize', 10, 'LineWidth', 2);  % Điểm bắt đầu
        plot(path(end,2), path(end,1), 'bx', 'MarkerSize', 10, 'LineWidth', 2);  % Điểm kết thúc
        
        % Vẽ các điểm trung gian
        for i = 1:size(waypoints, 1)
            plot(waypoints(i, 2), waypoints(i, 1), 'bo', 'MarkerSize', 10, 'LineWidth', 2); % Đánh dấu điểm trung gian
        end
    end
    
    % Vẽ các điểm gãy
    if ~isempty(break_points)
        plot(break_points(:,2), break_points(:,1), 'ms', 'MarkerSize', 8, 'LineWidth', 2); % Các điểm gãy màu tím
    end
    
    title('Dijkstra Pathfinding with Break Points');
    xlabel('Cột');
    ylabel('Hàng');
    axis equal;
    grid on;
    hold off;
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
            filtered_coords = [filtered_coords; current_point]; %#ok<AGROW>
        end
    end
end
