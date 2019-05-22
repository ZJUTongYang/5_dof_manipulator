% 正运动学方程

clear, clc;

syms pi theta1 theta2 theta3 theta4 theta5 a1 a2 a3;

% T1
alpha = 0;
a = 0;
theta = theta1;
d = 0;
T1 = [cos(theta)            -sin(theta)           0           a            ;
      sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha);
      sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha)  d*cos(alpha) ;
      0                     0                     0           1           ];

% T2
alpha = -pi/2;
a = a1;
theta = theta2;
d = 0;
T2 = [cos(theta)            -sin(theta)           0           a            ;
      sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha);
      sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha)  d*cos(alpha) ;
      0                     0                     0           1           ];

% T3
alpha = 0;
a = a2;
theta = theta3;
d = 0;
T3 = [cos(theta)            -sin(theta)           0           a            ;
      sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha);
      sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha)  d*cos(alpha) ;
      0                     0                     0           1           ];

% T4
alpha = 0;
a = a3;
theta = theta4;
d = 0;
T4 = [cos(theta)            -sin(theta)           0           a            ;
      sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha);
      sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha)  d*cos(alpha) ;
      0                     0                     0           1           ];

% T5
alpha = -pi/2;
a = 0;
theta = theta5;
d = 0;
T5 = [cos(theta)            -sin(theta)           0           a            ;
      sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha);
      sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha)  d*cos(alpha) ;
      0                     0                     0           1           ];

T = expand(T1*T2*T3*T4*T5);
