clear, clc;

% Robotics Toolbox for MATLAB
L1 = RevoluteMDH('alpha', 0,     'a', 0,  'd', 72);  %  加入沿 Z0 轴方向 72 mm 的偏移，为底座高度
L2 = RevoluteMDH('alpha', -pi/2, 'a', 3,  'd', 0);
L3 = RevoluteMDH('alpha', 0,     'a', 96, 'd', 0);
L4 = RevoluteMDH('alpha', 0,     'a', 96, 'd', 0);
L5 = RevoluteMDH('alpha', -pi/2, 'a', 0,  'd', 0);

arm = SerialLink([L1 L2 L3 L4 L5], 'name', 'xArm');
% 初始姿态
% init = [0 0 0 -pi/2 0];
% arm.plot(init, 'workspace', [-400 400 -400 400 0 400]);
% 逆解姿态
theta = solve_ik(120, 0, 20, 0, pi/2.5);
arm.plot(theta, 'workspace', [-400 400 -400 400 0 400]);
