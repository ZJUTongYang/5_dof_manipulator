% 笛卡尔空间中的轨迹规划

close all;
clear, clc;

% Robotics Toolbox for MATLAB
L1 = RevoluteMDH('alpha', 0,     'a', 0,  'd', 72);  %  加入沿 Z0 轴方向 72 mm 的偏移，为底座高度
L2 = RevoluteMDH('alpha', -pi/2, 'a', 3,  'd', 0);
L3 = RevoluteMDH('alpha', 0,     'a', 96, 'd', 0);
L4 = RevoluteMDH('alpha', 0,     'a', 96, 'd', 0);
L5 = RevoluteMDH('alpha', -pi/2, 'a', 0,  'd', 0);

arm = SerialLink([L1 L2 L3 L4 L5], 'name', 'xArm');

step = 50;

% 起点位姿
start_pose = rpy2tr(0, pi/6, atan(50/120));
start_pose(1, 4) = 120;
start_pose(2, 4) = 50;
start_pose(3, 4) = 120;
% 终点位姿
end_pose = rpy2tr(0, pi/2.5, atan(0/100));
end_pose(1, 4) = 150;
end_pose(2, 4) = 0;
end_pose(3, 4) = 10;

TC = ctraj(start_pose, end_pose, step);
T = transl(TC);

figure;
plot2(T);
title('末端执行器的轨迹');
xlim([-200 200]);
ylim([-200 200]);
zlim([0 200]);
grid on;

Q = zeros(step, 5);
for i = 1:step
    Q(i, :) = solve_ik(TC(:, :, i));
end
QD = zeros(step, 5);
QD(2:step, :) = diff(Q);
QDD = diff(QD);

figure;
subplot(3,1,1);
plot(Q);
title('位置');
xlabel('t');
ylabel('Q');
grid on;
subplot(3,1,2);
plot(QD);
title('速度');
xlabel('t');
ylabel('QD');
grid on;
subplot(3,1,3);
plot(QDD);
title('加速度');
xlabel('t');
ylabel('QDD');
grid on;

figure;
arm.plot(Q, 'workspace', [-300 300 -300 300 0 300]);
