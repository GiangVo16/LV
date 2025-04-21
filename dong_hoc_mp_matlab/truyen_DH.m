clear
giao_dien_dong_hoc_pro

function giao_dien_dong_hoc_pro()

    %% Kết nối TCP
    tcpObj = tcpclient('192.168.1.179', 8888);  % IP ESP32
    pause(1);  % Đợi kết nối ổn định

    global isReading;
    isReading = true;

    %% Tạo UI Figure
    fig = uifigure('Name', 'Giao Diện Điều Khiển Động Học', 'Position', [100 100 500 350]);

    %% Hiển thị góc nhận được
    txtStatus = uitextarea(fig, 'Position', [30 30 430 70], 'Editable', 'off');

    %% VÙNG: ĐỘNG HỌC THUẬN
    uilabel(fig, 'Position', [30 290 200 22], 'Text', 'ĐỘNG HỌC THUẬN', 'FontWeight', 'bold');

    uilabel(fig, 'Position', [30 260 30 22], 'Text', 'T0');
    t0_edit = uieditfield(fig, 'numeric', 'Position', [60 260 60 22]);

    uilabel(fig, 'Position', [130 260 30 22], 'Text', 'T1');
    t1_edit = uieditfield(fig, 'numeric', 'Position', [160 260 60 22]);

    uilabel(fig, 'Position', [230 260 30 22], 'Text', 'T2');
    t2_edit = uieditfield(fig, 'numeric', 'Position', [260 260 60 22]);

    uilabel(fig, 'Position', [330 260 30 22], 'Text', 'T3');
    t3_edit = uieditfield(fig, 'numeric', 'Position', [360 260 60 22]);

    btn_dht = uibutton(fig, 'Text', 'Gửi DHT', 'Position', [180 230 100 25], ...
        'ButtonPushedFcn', @(btn, event) gui_dht(tcpObj, t0_edit.Value, t1_edit.Value, t2_edit.Value, t3_edit.Value, txtStatus));

    %% VÙNG: ĐỘNG HỌC NGHỊCH
    uilabel(fig, 'Position', [30 180 200 22], 'Text', 'ĐỘNG HỌC NGHỊCH', 'FontWeight', 'bold');

    uilabel(fig, 'Position', [30 150 30 22], 'Text', 'X');
    x_edit = uieditfield(fig, 'numeric', 'Position', [60 150 60 22]);

    uilabel(fig, 'Position', [130 150 30 22], 'Text', 'Y');
    y_edit = uieditfield(fig, 'numeric', 'Position', [160 150 60 22]);

    uilabel(fig, 'Position', [230 150 30 22], 'Text', 'Z');
    z_edit = uieditfield(fig, 'numeric', 'Position', [260 150 60 22]);

    btn_dhn = uibutton(fig, 'Text', 'Gửi DHN', 'Position', [180 120 100 25], ...
        'ButtonPushedFcn', @(btn, event) gui_dhn(tcpObj, x_edit.Value, y_edit.Value, z_edit.Value, txtStatus));

    %% Tạo Timer để đọc dữ liệu liên tục
    t = timer('ExecutionMode', 'fixedSpacing', ...
              'Period', 0.1, ...
              'TimerFcn', @(~,~) doc_goc(tcpObj, txtStatus));
    start(t);

    %% Đóng kết nối khi đóng UI
    fig.CloseRequestFcn = @(src, event) dong_ung_dung(src, t, tcpObj);
end

%% Hàm gửi động học thuận
function gui_dht(tcpObj, t0, t1, t2, t3, txt)
    global isReading;
    isReading = false;
    txt.Value = "⏳ Đang thực hiện động học thuận...";

    cmd = sprintf('dht t0 %.1f t1 %.1f t2 %.1f t3 %.1f\n', t0, t1, t2, t3);
    writeline(tcpObj, cmd);
    disp("📤 Gửi DHT: " + cmd);

    pause(5);  % Thời gian chờ ESP32 xử lý (có thể điều chỉnh)
    isReading = true;
end

%% Hàm gửi động học nghịch
function gui_dhn(tcpObj, x, y, z, txt)
    global isReading;
    isReading = false;
    txt.Value = "⏳ Đang thực hiện động học nghịch...";

    cmd = sprintf('dhn x %.2f y %.2f z %.2f\n', x, y, z);
    writeline(tcpObj, cmd);
    disp("📤 Gửi DHN: " + cmd);

    pause(5);  % Thời gian chờ ESP32 xử lý
    isReading = true;
end

%% Hàm đọc góc từ ESP32
function doc_goc(tcpObj, txt)
    global isReading;
    if isReading
        try
            while tcpObj.NumBytesAvailable > 0
                data = strtrim(readline(tcpObj));
                %disp("📥 Nhận: " + data);

                % Kiểm tra dữ liệu hợp lệ (CSV: 4 giá trị phân tách bằng dấu phẩy)
                parts = split(data, ",");
                if numel(parts) == 4
                    angles = str2double(parts);
                    if all(~isnan(angles))
                        txt.Value = sprintf("Góc hiện tại:\nT0: %.2f°\nT1: %.2f°\nT2: %.2f°\nT3: %.2f°", angles);
                    end
                end
            end
        catch
            txt.Value = "⚠️ Lỗi đọc dữ liệu từ ESP32.";
        end
    end
end

%% Đóng ứng dụng
function dong_ung_dung(fig, t, tcpObj)
    stop(t);
    delete(t);
    clear tcpObj;
    delete(fig);
end
