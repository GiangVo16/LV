a2 = 0.32; a3 = 0.27; a4 = 0.076; d1 = 0.406;

phi = -pi/2;    % Góc gắp
x = 0.418;
y = 0.01;
z = 0.12;

z = z + a4;
theta1 = atan2(y,x);

s = z - d1;
r = sqrt(x^2 + y^2);
r1 = r - a4*cos(phi);

m = sqrt(s^2 + r1^2);

beta = acos((a2^2 + a3^2 - m^2) / (2*a2*a3));

theta3 = -(pi - beta);

alpha1 = atan2(s, r1);
alpha2 = acos((a2^2 + m^2 - a3^2)/ (2*a2*m));

theta2 = alpha1 + alpha2;

theta4 = phi - (theta2 + theta3);

theta1
theta2
theta3
theta4

% Đổi radian sang độ
% Đổi radian sang độ và giới hạn từ 0 đến 360
theta1_a = mod(rad2deg(theta1), 360);
theta2_a = mod(rad2deg(theta2), 360);
theta3_a = mod(rad2deg(theta3), 360);
theta4_a = mod(rad2deg(theta4), 360);

% -180 đến 180 độ
theta1_b = rad2deg(theta1);
theta2_b = rad2deg(theta2);
theta3_b = rad2deg(theta3);
theta4_b = rad2deg(theta4);

% Hiển thị góc ko âm và có âm
theta1_c = [theta1_a theta1_b]
theta2_c = [theta2_a theta2_b]
theta3_c = [theta3_a theta3_b]
theta4_c = [theta4_a theta4_b]
