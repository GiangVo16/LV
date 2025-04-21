% Start up trong file D:\Thu vien\RVC1\RVC1\rvctools
close all

% Các khâu robot
L1 = Revolute('d', 0.406, 'a', 0, 'alpha', pi/2, 'offset', 0);
L2 = Revolute('d', 0, 'a', 0.32, 'alpha', 0, 'offset', 0);
L3 = Revolute('d', 0, 'a', 0.27, 'alpha', 0, 'offset', 0);
L4 = Revolute('d', 0, 'a', 0.076, 'alpha', 0, 'offset', 0);


% Tạo robot 3 khớp quay
robot = SerialLink([L1 L2 L3 L4], 'name', '3DOF robot');

% Hiển thị robot với góc quay của từng khớp
q = [0, 0, 0, 0];  % θ₁, θ₂ , θ₃ 
robot.teach(q)
