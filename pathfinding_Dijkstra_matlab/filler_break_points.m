%%
% Dữ liệu đầu vào
coords = [0 0; 5 5; 10 10; 12 12; 15 15; 20 20; 25 25; 30 30];

% Khoảng cách tối thiểu giữa các điểm
min_distance = 10;

% Các tọa độ không được lọc bỏ
fixed_points = [5 5; 15 15];

% Gọi hàm lọc tọa độ
filtered_coords = filter_coordinates(coords, min_distance, fixed_points);

% Hiển thị kết quả
disp('Tọa độ sau khi lọc:')
disp(filtered_coords)



%%
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
