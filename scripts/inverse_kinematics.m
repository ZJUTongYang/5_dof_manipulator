% 逆运动学方程

clear, clc;

% 末端位姿
syms nx ny nz ox oy oz ax ay az px py pz;
T = [nx ox ax px ;
     ny oy ay py ;
     nz oz az pz ;
     0  0  0  1 ];

% 连杆参数
syms pi s1 c1 s2 c2 s3 c3 s4 c4 s5 c5 a1 a2 a3;
alpha1 = -pi/2;
d2 = 0;

% 齐次变换矩阵
T1 = [c1 -s1 0 0 ;
      s1 c1  0 0 ;
      0  0   1 0 ;
      0  0   0 1];

T2 = [c2  -s2 0 a1;
      0   0   1 0 ;
      -s2 -c2 0 0 ;
      0   0   0 1];

T3 = [c3 -s3 0 a2;
      s3 c3  0 0 ;
      0  0   1 0 ;
      0  0   0 1];

T4 = [c4 -s4 0 a3;
      s4 c4  0 0 ;
      0  0   1 0 ;
      0  0   0 1];

T5 = [c5  -s5 0 0 ;
      0   0   1 0 ;
      -s5 -c5 0 0 ;
      0   0   0 1];

f = T3*T4(:, 4);
f1 = f(1); f2 = f(2); f3 = f(3);

g = T2*T3*T4(:, 4);
g1 = g(1); g2 = g(2); g3 = g(3);

k1 = f1;
k2 = -f2;
k3 = f1^2 + f2^2 + f3^2 + a1^2 + d2^2 + 2*d2*f3;
k4 = f3*cos(alpha1) + d2*cos(alpha1);

r = px^2 + py^2 + pz^2;

% 关于 theta3 的方程，代入万能公式，解 theta3
syms u;
eq = (r - k3)^2 / (4*a1^2) + (pz - k4)^2 / sin(alpha1)^2 == k1^2 + k2^2;
eq = subs(eq, s3, 2*u / (1 + u^2));
eq = subs(eq, c3, (1 - u^2) / (1 + u^2));
u = solve(eq, u);
theta3 = simplify(2*atan(u));

% 解 theta2
syms u;
eq = r == 2*a1*(k1*c2 + k2*s2) + k3;
eq = subs(eq, s2, 2*u / (1 + u^2));
eq = subs(eq, c2, (1 - u^2) / (1 + u^2));
u = solve(eq, u);
theta2 = simplify(2*atan(u));

% 解 theta1
syms u;
eq = px == g1*c1 - g2*s1;
eq = subs(eq, s1, 2*u / (1 + u^2));
eq = subs(eq, c1, (1 - u^2) / (1 + u^2));
u = solve(eq, u);
theta1 = simplify(2*atan(u));

% 解 theta4
iT13 = (T1*T2*T3)^(-1);
left = iT13*T;
theta4 = atan2(-left(1, 3), left(2, 3));

% 解 theta5
iT14 = (T1*T2*T3*T4)^(-1);
left = iT14*T;
theta5 = atan2(-left(1, 2), left(1, 1));
