% 关节空间中的轨迹规划

close, clear, clc;

% Robotics Toolbox for MATLAB
L1 = RevoluteMDH('alpha', 0,     'a', 0,  'd', 72);  %  加入沿 Z0 轴方向 72 mm 的偏移，为底座高度
L2 = RevoluteMDH('alpha', -pi/2, 'a', 3,  'd', 0);
L3 = RevoluteMDH('alpha', 0,     'a', 96, 'd', 0);
L4 = RevoluteMDH('alpha', 0,     'a', 96, 'd', 0);
L5 = RevoluteMDH('alpha', -pi/2, 'a', 0,  'd', 0);

arm = SerialLink([L1 L2 L3 L4 L5], 'name', 'xArm');

step = 50;

begin_angle = solve_ik(160, 120, 160);
end_angle = solve_ik(120, 0 ,20, 0, pi/3);
[Q, QD, QDD] = jtraj(begin_angle, end_angle, step);
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
