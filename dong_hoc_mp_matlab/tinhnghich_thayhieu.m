% === Thông số Robot ===
a2 = 32; % Chiều dài liên kết 2
a3 = 27; % Chiều dài liên kết 3
d1 = 12.6; % Chiều dài khớp 1

% Động học nghịch
x3= 44.3;
y3= 28.1;
z3= 31.8;

s = z3-d1;
m = x3;
r = sqrt(s^2+m^2);
%tìm theta1
theta1 = atan(y3/x3);
%tìm theta2
anpha1 = atan(s/m);
anpha2 = -acos((a3*a3 -a2*a2 - r*r) / (2 * a2 *r));
theta2 = anpha1 + anpha2 + pi;
%tìm theta3
beta1 = -acos((r*r -a2*a2 -a3*a3)/(2*a2*a3));
beta2 = pi - theta2  ;
theta3 = beta1 - beta2 + pi;
% tìm theta4
theta4 = -theta2-theta3;
%doi radian sang do
theta1_do = rad2deg(theta1);
theta2_do = rad2deg(theta2);
theta3_do = rad2deg(theta3);
theta4_do = rad2deg(theta4);
%hienthi ket qua
disp('khop theta1 theo do');
disp(theta1_do);
disp('khop theta2 theo do');
disp(theta2_do);
disp('khop theta3 theo do');
disp(theta3_do);
disp('khop theta4 theo do');
disp(theta4_do);

disp('khop theta1 theo rad');
disp(theta1);
disp('khop theta2 theo rad');
disp(theta2);
disp('khop theta3 theo rad');
disp(theta3);
disp('khop theta4 theo rad');
disp(theta4);
