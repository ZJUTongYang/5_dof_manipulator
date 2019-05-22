function positions = solve_ik(x, y, z, roll, pitch)
%SOLVE_IK    求解五自由度机械臂逆运动学方程
%   solve_ik(x, y, z, roll, pitch) 求解给定世界坐标系下位置 (x, y, z) 和 RPY
%   角表示的姿态时，五自由度机械臂各个关节的关节变量。位置的单位为毫米 (mm)，姿态的单
%   位为弧度 (rad)。
%
%   由于机械臂只有五个自由度，因此受其结构限制，偏航角 (yaw) 通过位置计算得到。
%
%   示例
%      theta = solve_ik(160, 0, 160);
%      theta = solve_ik(120, 0, 20, 0, pi/2.5);


% 检查参数数量，必须输入位置，姿态默认值为 0
if nargin < 3
    error('参数不足！');
elseif nargin == 3
    roll = 0;
    pitch = 0;
elseif nargin == 4
    pitch = 0;
end

% 五自由度的限制
yaw = atan(y/x);

% RPY 角 -> 四元数 (Quaternion)
qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
qx = cos(pitch/2)*cos(yaw/2)*sin(roll/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
qy = cos(roll/2)*cos(yaw/2)*sin(pitch/2) + cos(pitch/2)*sin(roll/2)*sin(yaw/2);
qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - cos(yaw/2)*sin(roll/2)*sin(pitch/2);

% 四元数 -> 旋转矩阵
nx_g = qw^2 + qx^2 - qy^2 - qz^2;
ox_g = 2*qx*qy - 2*qw*qz;
ax_g = 2*qx*qz + 2*qw*qy;
ny_g = 2*qx*qy + 2*qw*qz;
oy_g = qw^2 - qx^2 + qy^2 - qz^2;
ay_g = 2*qy*qz - 2*qw*qx;
nz_g = 2*qx*qz - 2*qw*qy;
oz_g = 2*qy*qz + 2*qw*qx;
az_g = qw^2 - qx^2 - qy^2 + qz^2;

% 连杆参数，单位：mm
a1 = 3;
a2 = 96;
a3 = 96;
base_height = 72;   % 基座高度
tool_length = 120;  % 末端执行器长度

% 末端执行器位姿矩阵
G = [nx_g ox_g ax_g x ;
     ny_g oy_g ay_g y ;
     nz_g oz_g az_g z ;
     0    0    0    1];

% 世界坐标系到基坐标系的变换
T0 = [1 0 0 0          ;
      0 1 0 0          ;
      0 0 1 base_height;
      0 0 0 1         ];
% 坐标系 5 到末端执行器坐标系的变换
Tt = [0 0  1 0          ;
      0 -1 0 0          ;
      1 0  0 tool_length;
      0 0  0 1         ];
% 基坐标系到坐标系 5 的变换
T = T0^(-1)*G*Tt^(-1);
px = T(1, 4);
py = T(2, 4);
pz = T(3, 4);

% 求解逆运动学方程
theta3 = [
    2*atan(((2*a1^2*a2^2 - a2^4 - a3^4 - px^4 - py^4 - pz^4 - a1^4 + 2*a1^2*a3^2 + 2*a2^2*a3^2 + 2*a1^2*px^2 + 2*a2^2*px^2 + 2*a3^2*px^2 + 2*a1^2*py^2 + 2*a2^2*py^2 + 2*a3^2*py^2 - 2*a1^2*pz^2 + 2*a2^2*pz^2 + 2*a3^2*pz^2 - 2*px^2*py^2 - 2*px^2*pz^2 - 2*py^2*pz^2 + 8*a1*a2*a3*(px^2 + py^2)^(1/2))/((px^2 + py^2)*(- 2*a1^2 - 2*a2^2 + 4*a2*a3 - 2*a3^2 + px^2 + py^2 + 2*pz^2) - 4*a2*a3^3 - 4*a2^3*a3 + a1^4 + a2^4 + a3^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 + 6*a2^2*a3^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 4*a1^2*a2*a3 + 4*a2*a3*pz^2))^(1/2))

    -2*atan((-(a1^4 + a2^4 + a3^4 + px^4 + py^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 - 2*a2^2*a3^2 - 2*a1^2*px^2 - 2*a2^2*px^2 - 2*a3^2*px^2 - 2*a1^2*py^2 - 2*a2^2*py^2 - 2*a3^2*py^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 2*px^2*py^2 + 2*px^2*pz^2 + 2*py^2*pz^2 + 8*a1*a2*a3*(px^2 + py^2)^(1/2))/((px^2 + py^2)*(- 2*a1^2 - 2*a2^2 + 4*a2*a3 - 2*a3^2 + px^2 + py^2 + 2*pz^2) - 4*a2*a3^3 - 4*a2^3*a3 + a1^4 + a2^4 + a3^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 + 6*a2^2*a3^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 4*a1^2*a2*a3 + 4*a2*a3*pz^2))^(1/2))

    -2*atan(((2*a1^2*a2^2 - a2^4 - a3^4 - px^4 - py^4 - pz^4 - a1^4 + 2*a1^2*a3^2 + 2*a2^2*a3^2 + 2*a1^2*px^2 + 2*a2^2*px^2 + 2*a3^2*px^2 + 2*a1^2*py^2 + 2*a2^2*py^2 + 2*a3^2*py^2 - 2*a1^2*pz^2 + 2*a2^2*pz^2 + 2*a3^2*pz^2 - 2*px^2*py^2 - 2*px^2*pz^2 - 2*py^2*pz^2 + 8*a1*a2*a3*(px^2 + py^2)^(1/2))/((px^2 + py^2)*(- 2*a1^2 - 2*a2^2 + 4*a2*a3 - 2*a3^2 + px^2 + py^2 + 2*pz^2) - 4*a2*a3^3 - 4*a2^3*a3 + a1^4 + a2^4 + a3^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 + 6*a2^2*a3^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 4*a1^2*a2*a3 + 4*a2*a3*pz^2))^(1/2))

    2*atan((-(a1^4 + a2^4 + a3^4 + px^4 + py^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 - 2*a2^2*a3^2 - 2*a1^2*px^2 - 2*a2^2*px^2 - 2*a3^2*px^2 - 2*a1^2*py^2 - 2*a2^2*py^2 - 2*a3^2*py^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 2*px^2*py^2 + 2*px^2*pz^2 + 2*py^2*pz^2 + 8*a1*a2*a3*(px^2 + py^2)^(1/2))/((px^2 + py^2)*(- 2*a1^2 - 2*a2^2 + 4*a2*a3 - 2*a3^2 + px^2 + py^2 + 2*pz^2) - 4*a2*a3^3 - 4*a2^3*a3 + a1^4 + a2^4 + a3^4 + pz^4 - 2*a1^2*a2^2 - 2*a1^2*a3^2 + 6*a2^2*a3^2 + 2*a1^2*pz^2 - 2*a2^2*pz^2 - 2*a3^2*pz^2 + 4*a1^2*a2*a3 + 4*a2*a3*pz^2))^(1/2))
];
if y >= 0
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

positions = [t1, t2, t3, t4, t5];

end
