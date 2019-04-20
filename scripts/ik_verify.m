clear, clc;

% Position
px_g = 100;
py_g = 50;  
pz_g = 150;

% RPY
a = 0;
b = pi/4;
c = atan2(py_g, px_g);  % 5 自由度限制

% RPY 角转四元数 (Quaternion)
w = cos(a/2)*cos(b/2)*cos(c/2) + sin(a/2)*sin(b/2)*sin(c/2);
x = cos(b/2)*cos(c/2)*sin(a/2) - cos(a/2)*sin(b/2)*sin(c/2);
y = cos(a/2)*cos(c/2)*sin(b/2) + cos(b/2)*sin(a/2)*sin(c/2);
z = cos(a/2)*cos(b/2)*sin(c/2) - cos(c/2)*sin(a/2)*sin(b/2);

% 四元数 (Quaternion) 转旋转矩阵
nx_g = w^2 + x^2 - y^2 - z^2;
ox_g = 2*x*y - 2*w*z;
ax_g = 2*x*z + 2*w*y;
ny_g = 2*x*y + 2*w*z;
oy_g = w^2 - x^2 + y^2 - z^2;
ay_g = 2*y*z - 2*w*x;
nz_g = 2*x*z - 2*w*y;
oz_g = 2*y*z + 2*w*x;
az_g = w^2 - x^2 - y^2 + z^2;

% 连杆参数，单位：mm
a1 = 3;
a2 = 96;
a3 = 96;

% 目标物体的位姿
% nx_g = 0; ox_g = 1; ax_g = 0; px_g = 0;
% ny_g = 1; oy_g = 0; ay_g = 0; py_g = 200;
% nz_g = 0; oz_g = 0; az_g = 1; pz_g = 100;
G = [nx_g ox_g ax_g px_g;
     ny_g oy_g ay_g py_g;
     nz_g oz_g az_g pz_g;
     0    0    0    1  ];

% 世界坐标系到基坐标系的变换
T0 = [1 0 0 0 ;
      0 1 0 0 ;
      0 0 1 72;
      0 0 0 1];
% 坐标系 5 到末端执行器坐标系的变换
Tt = [0 0  1 0  ;
      0 -1 0 0  ;
      1 0  0 120;
      0 0  0 1 ];
% 基坐标系到坐标系 5 的变换
T = T0^(-1)*G*Tt^(-1);
nx = T(1, 1); ox = T(1, 2); ax = T(1, 3); px = T(1, 4);
ny = T(2, 1); oy = T(2, 2); ay = T(2, 3); py = T(2, 4);
nz = T(3, 1); oz = T(3, 2); az = T(3, 3); pz = T(3, 4);

% 逆运动学的解
theta3 = [
    2*atan(((2*a1^2*a2^2 - a2^4 - a3^4 - px^4 - py^4 - pz^4 - a1^4 + 2*a1^2*a3^2 + 2*a2^2*a3^2 + 2*a1^2*px^2 + 2*a2^2*px^2 + 2*a3^2*px^2 + 2*a1^2*py^2 + 2*a2^2*py^2 + 2*a3^2*py^2 - 2*a1^2*pz^2 + 2*a2^2*pz^2 + 2*a3^2*pz^2 - 2*px^2*py^2 - 2*px^2*pz^2 - 2*py^2*pz^2 + 8*a1*a2*a3*(px^2 + py^2)^(1/2))/((px^2 + py^2)*(- 2*a1^2 - 2*a2^2 + 4*a2*a3 - 2*a3^2 + px^2 + py^2 + 2*pz^2) - 4*a2*a3^3 - 4*a2^3*a3 + a1^4 + a2^4 + a3^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 + 6*a2^2*a3^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 4*a1^2*a2*a3 + 4*a2*a3*pz^2))^(1/2))

    -2*atan((-(a1^4 + a2^4 + a3^4 + px^4 + py^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 - 2*a2^2*a3^2 - 2*a1^2*px^2 - 2*a2^2*px^2 - 2*a3^2*px^2 - 2*a1^2*py^2 - 2*a2^2*py^2 - 2*a3^2*py^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 2*px^2*py^2 + 2*px^2*pz^2 + 2*py^2*pz^2 + 8*a1*a2*a3*(px^2 + py^2)^(1/2))/((px^2 + py^2)*(- 2*a1^2 - 2*a2^2 + 4*a2*a3 - 2*a3^2 + px^2 + py^2 + 2*pz^2) - 4*a2*a3^3 - 4*a2^3*a3 + a1^4 + a2^4 + a3^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 + 6*a2^2*a3^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 4*a1^2*a2*a3 + 4*a2*a3*pz^2))^(1/2))

    -2*atan(((2*a1^2*a2^2 - a2^4 - a3^4 - px^4 - py^4 - pz^4 - a1^4 + 2*a1^2*a3^2 + 2*a2^2*a3^2 + 2*a1^2*px^2 + 2*a2^2*px^2 + 2*a3^2*px^2 + 2*a1^2*py^2 + 2*a2^2*py^2 + 2*a3^2*py^2 - 2*a1^2*pz^2 + 2*a2^2*pz^2 + 2*a3^2*pz^2 - 2*px^2*py^2 - 2*px^2*pz^2 - 2*py^2*pz^2 + 8*a1*a2*a3*(px^2 + py^2)^(1/2))/((px^2 + py^2)*(- 2*a1^2 - 2*a2^2 + 4*a2*a3 - 2*a3^2 + px^2 + py^2 + 2*pz^2) - 4*a2*a3^3 - 4*a2^3*a3 + a1^4 + a2^4 + a3^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 + 6*a2^2*a3^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 4*a1^2*a2*a3 + 4*a2*a3*pz^2))^(1/2))

    2*atan((-(a1^4 + a2^4 + a3^4 + px^4 + py^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 - 2*a2^2*a3^2 - 2*a1^2*px^2 - 2*a2^2*px^2 - 2*a3^2*px^2 - 2*a1^2*py^2 - 2*a2^2*py^2 - 2*a3^2*py^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 2*px^2*py^2 + 2*px^2*pz^2 + 2*py^2*pz^2 + 8*a1*a2*a3*(px^2 + py^2)^(1/2))/((px^2 + py^2)*(- 2*a1^2 - 2*a2^2 + 4*a2*a3 - 2*a3^2 + px^2 + py^2 + 2*pz^2) - 4*a2*a3^3 - 4*a2^3*a3 + a1^4 + a2^4 + a3^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 + 6*a2^2*a3^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 4*a1^2*a2*a3 + 4*a2*a3*pz^2))^(1/2))
];
if py_g >= 0
    t3 = real(theta3(1));
else
    t3 = real(theta3(2));
end
s3 = sin(t3);
c3 = cos(t3);

theta2 = [
    2*atan(((- a1^4 + 2*a1^2*a2^2 + 4*a1^2*a2*a3*c3 + 2*a1^2*a3^2*c3^2 + 2*a1^2*a3^2*s3^2 + 2*a1^2*px^2 + 2*a1^2*py^2 + 2*a1^2*pz^2 - a2^4 - 4*a2^3*a3*c3 - 6*a2^2*a3^2*c3^2 - 2*a2^2*a3^2*s3^2 + 2*a2^2*px^2 + 2*a2^2*py^2 + 2*a2^2*pz^2 - 4*a2*a3^3*c3^3 - 4*a2*a3^3*c3*s3^2 + 4*a2*a3*c3*px^2 + 4*a2*a3*c3*py^2 + 4*a2*a3*c3*pz^2 - a3^4*c3^4 - 2*a3^4*c3^2*s3^2 - a3^4*s3^4 + 2*a3^2*c3^2*px^2 + 2*a3^2*c3^2*py^2 + 2*a3^2*c3^2*pz^2 + 2*a3^2*px^2*s3^2 + 2*a3^2*py^2*s3^2 + 2*a3^2*pz^2*s3^2 - px^4 - 2*px^2*py^2 - 2*px^2*pz^2 - py^4 - 2*py^2*pz^2 - pz^4)^(1/2) - 2*a1*a3*s3)/(- a1^2 + 2*a1*a2 + 2*a1*a3*c3 - a2^2 - 2*a2*a3*c3 - a3^2*c3^2 - a3^2*s3^2 + px^2 + py^2 + pz^2))

    -2*atan(((- a1^4 + 2*a1^2*a2^2 + 4*a1^2*a2*a3*c3 + 2*a1^2*a3^2*c3^2 + 2*a1^2*a3^2*s3^2 + 2*a1^2*px^2 + 2*a1^2*py^2 + 2*a1^2*pz^2 - a2^4 - 4*a2^3*a3*c3 - 6*a2^2*a3^2*c3^2 - 2*a2^2*a3^2*s3^2 + 2*a2^2*px^2 + 2*a2^2*py^2 + 2*a2^2*pz^2 - 4*a2*a3^3*c3^3 - 4*a2*a3^3*c3*s3^2 + 4*a2*a3*c3*px^2 + 4*a2*a3*c3*py^2 + 4*a2*a3*c3*pz^2 - a3^4*c3^4 - 2*a3^4*c3^2*s3^2 - a3^4*s3^4 + 2*a3^2*c3^2*px^2 + 2*a3^2*c3^2*py^2 + 2*a3^2*c3^2*pz^2 + 2*a3^2*px^2*s3^2 + 2*a3^2*py^2*s3^2 + 2*a3^2*pz^2*s3^2 - px^4 - 2*px^2*py^2 - 2*px^2*pz^2 - py^4 - 2*py^2*pz^2 - pz^4)^(1/2) + 2*a1*a3*s3)/(- a1^2 + 2*a1*a2 + 2*a1*a3*c3 - a2^2 - 2*a2*a3*c3 - a3^2*c3^2 - a3^2*s3^2 + px^2 + py^2 + pz^2))
];
t2 = real(theta2(2));
s2 = sin(t2);
c2 = cos(t2);

theta1 = [
    2*atan(((a1 - px + a2*c2 + a3*c2*c3 - a3*s2*s3)/(a1 + px + a2*c2 + a3*c2*c3 - a3*s2*s3))^(1/2))
    
    -2*atan(((a1 - px + a2*c2 + a3*c2*c3 - a3*s2*s3)/(a1 + px + a2*c2 + a3*c2*c3 - a3*s2*s3))^(1/2))
];
t1 = real(theta1(1));
s1 = sin(t1);
c1 = cos(t1);

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
iT13 = (T1*T2*T3)^(-1);
left = iT13*T;
t4 = atan2(-left(1, 3), left(2, 3));
s4 = sin(t4);
c4 = cos(t4);

T4 = [c4 -s4 0 a3;
      s4 c4  0 0 ;
      0  0   1 0 ;
      0  0   0 1];
iT14 = (T1*T2*T3*T4)^(-1);
left = iT14*T;
t5 = atan2(-left(1, 2), left(1, 1));
s5 = sin(t5);
c5 = cos(t5);

% Robotics Toolbox for MATLAB
L1 = RevoluteMDH('alpha', 0, 'a', 0, 'd', 72);  %  加入沿 Z0 轴方向 72 mm 的偏移，为底座高度
L2 = RevoluteMDH('alpha', -pi/2, 'a', a1, 'd', 0);
L3 = RevoluteMDH('alpha', 0, 'a', a2, 'd', 0);
L4 = RevoluteMDH('alpha', 0, 'a', a3, 'd', 0);
L5 = RevoluteMDH('alpha', -pi/2, 'a', 0, 'd', 0);

arm = SerialLink([L1 L2 L3 L4 L5], 'name', 'Manipulator');
theta = [t1 t2 t3 t4 t5];
% theta = [0 0 0 0 0];
arm.plot(theta, 'workspace', [-400 400 -400 400 0 400]);
