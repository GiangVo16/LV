clear

giao_dien_dong_hoc_pro



%% Giao diện truyền động học
function giao_dien_dong_hoc_pro()
    % Tạo UI Figure
    fig = uifigure('Name', 'Giao Diện Điều Khiển Động Học', 'Position', [100 100 450 300]);

    % Kết nối TCP
    tcpObj = tcpclient('192.168.1.5', 8888);  % Đổi IP của ESP32

    %% VÙNG: ĐỘNG HỌC THUẬN
    lbl_dht = uilabel(fig, 'Position', [30 250 200 22], 'Text', 'ĐỘNG HỌC THUẬN', 'FontWeight', 'bold');

    uilabel(fig, 'Position', [30 220 30 22], 'Text', 'T0');
    t0_edit = uieditfield(fig, 'numeric', 'Position', [60 220 60 22]);

    uilabel(fig, 'Position', [130 220 30 22], 'Text', 'T1');
    t1_edit = uieditfield(fig, 'numeric', 'Position', [160 220 60 22]);

    uilabel(fig, 'Position', [230 220 30 22], 'Text', 'T2');
    t2_edit = uieditfield(fig, 'numeric', 'Position', [260 220 60 22]);

    uilabel(fig, 'Position', [330 220 30 22], 'Text', 'T3');
    t3_edit = uieditfield(fig, 'numeric', 'Position', [360 220 60 22]);

    btn_dht = uibutton(fig, 'Text', 'Gửi DHT', 'Position', [180 190 100 25], ...
        'ButtonPushedFcn', @(btn, event) gui_dht(tcpObj, t0_edit.Value, t1_edit.Value, t2_edit.Value, t3_edit.Value));

    %% VÙNG: ĐỘNG HỌC NGHỊCH
    lbl_dhn = uilabel(fig, 'Position', [30 140 200 22], 'Text', 'ĐỘNG HỌC NGHỊCH', 'FontWeight', 'bold');

    uilabel(fig, 'Position', [30 110 30 22], 'Text', 'X');
    x_edit = uieditfield(fig, 'numeric', 'Position', [60 110 60 22]);

    uilabel(fig, 'Position', [130 110 30 22], 'Text', 'Y');
    y_edit = uieditfield(fig, 'numeric', 'Position', [160 110 60 22]);

    uilabel(fig, 'Position', [230 110 30 22], 'Text', 'Z');
    z_edit = uieditfield(fig, 'numeric', 'Position', [260 110 60 22]);

    btn_dhn = uibutton(fig, 'Text', 'Gửi DHN', 'Position', [180 80 100 25], ...
        'ButtonPushedFcn', @(btn, event) gui_dhn(tcpObj, x_edit.Value, y_edit.Value, z_edit.Value));
end

%% Các hàm gửi lệnh
function gui_dht(tcpObj, t0, t1, t2, t3)
    cmd = sprintf('dht t0 %.1f t1 %.1f t2 %.1f t3 %.1f\n', t0, t1, t2, t3);
    writeline(tcpObj, cmd);
    disp(['Đã gửi: ' cmd]);
end

function gui_dhn(tcpObj, x, y, z)
    cmd = sprintf('dhn x %.2f y %.2f z %.2f\n', x, y, z);
    writeline(tcpObj, cmd);
    disp(['Đã gửi: ' cmd]);
end
