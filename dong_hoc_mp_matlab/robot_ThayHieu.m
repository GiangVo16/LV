clc; clear; close all;

% === Thông số Robot ===
a2 = 32; % Chiều dài liên kết 2
a3 = 27; % Chiều dài liên kết 3
d1 = 12.6; % Chiều dài khớp 1
a4 = 7.5;
% Góc khớp ban đầu (radians)
theta1 = 0; % Góc khớp 1 (quay quanh trục Z của L1)
theta2 = 0; % Góc khớp 2 (điều chỉnh hướng lên/xuống của L2)
theta3 = 0; % Góc khớp 3
theta4 = 0; % Góc khớp 3
% === Jacobian ===
s1 = sin(theta1); c1 = cos(theta1);
s2 = sin(theta2); c2 = cos(theta2);
s3 = sin(theta3 ); c3 = cos(theta3 );
s4 = sin(theta4 ); c4 = cos(theta4 );

% === Tính toán vị trí các điểm ===
O0 = [0, 0, 0]; % Gốc tọa độ
O1 = [0, 0, d1]; % Vị trí khớp 1
O2 = [a2*c1*c2, a2*c2*s1, d1 + a2*s2]; % Vị trí khớp 2
O3 = [a2*c1*c2 - a3*c1*s2*s3 + a3*c1*c2*c3, ...
      a2*c2*s1 - a3*s1*s2*s3 + a3*c2*c3*s1, ...
         d1 + a2*s2 + a3*c2*s3 + a3*c3*s2]; % Vị trí khớp 3
O4 = [a2*c1*c2 + a4*c4*(c1*c2*c3 - c1*s2*s3) - a4*s4*(c1*c2*s3 + c1*c3*s2) - a3*c1*s2*s3 + a3*c1*c2*c3, ...
    a2*c2*s1 + a4*c4*(c2*c3*s1 - s1*s2*s3) - a4*s4*(c2*s1*s3 + c3*s1*s2) - a3*s1*s2*s3 + a3*c2*c3*s1, ...
                  d1 + a2*s2 + a3*c2*s3 + a3*c3*s2 + a4*c4*(c2*s3 + c3*s2) + a4*s4*(c2*c3 - s2*s3)]; % Vị trí khớp 4

% Hiển thị vị trí điểm cuối
disp('=== Vị trí điểm cuối (O3) ===');
fprintf('X = %.3f\n', O4(1));
fprintf('Y = %.3f\n', O4(2));
fprintf('Z = %.3f\n', O4(3));

% === Vẽ Robot trong 3D ===
figure;
hold on; grid on; axis equal;

% Cài đặt trục
xlim([-50, 50]); ylim([-50, 50]); zlim([0, 50]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); % Góc nhìn 3D


% Vẽ các liên kết
plot3([O0(1), O1(1)], [O0(2), O1(2)], [O0(3), O1(3)], 'r-', 'LineWidth', 2);
plot3([O1(1), O2(1)], [O1(2), O2(2)], [O1(3), O2(3)], 'g-', 'LineWidth', 2);
plot3([O2(1), O3(1)], [O2(2), O3(2)], [O2(3), O3(3)], 'b-', 'LineWidth', 2);
plot3([O3(1), O4(1)], [O3(2), O4(2)], [O3(3), O4(3)], 'b-', 'LineWidth', 2);

% Vẽ các khớp
scatter3(O0(1), O0(2), O0(3), 40, 'k', 'filled'); % Gốc tọa độ
scatter3(O1(1), O1(2), O1(3), 40, 'r', 'filled'); % Khớp 1
scatter3(O2(1), O2(2), O2(3), 40, 'g', 'filled'); % Khớp 2
scatter3(O3(1), O3(2), O3(3), 40, 'b', 'filled'); % Khớp 3
scatter3(O4(1), O4(2), O4(3), 40, 'y', 'filled'); % Khớp 3
% Hiển thị thông tin
title('ROBOT ARM');
