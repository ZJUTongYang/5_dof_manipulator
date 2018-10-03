clear, clc;

syms pi x y z s1 c1 s2 c2 s3 c3 s4 c4 s5 c5 a1 a2 a3;
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

r = x^2 + y^2 + z^2;

% 关于 theta3 的方程
eq = (r - k3)^2 / (4*a1^2) + (z - k4)^2 / sin(alpha1)^2 == k1^2 + k2^2;

% 代入万能公式，解 theta3
syms u;
eq = subs(eq, s3, 2*u / (1 + u^2));
eq = subs(eq, c3, (1 - u^2) / (1 + u^2));
u = solve(eq, u);
theta3 = 2*atan(u);
simplify(theta3)
