function [w, x, y, z] = rpy2q(roll, pitch, yaw)
%RPY2Q 将 RPY 角转换为四元数 (Quaternion)
%   RPY 角的单位为弧度 (rad)。


% 检查参数数量
if nargin < 3
    error('参数不足！');
end

w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
x = cos(pitch/2)*cos(yaw/2)*sin(roll/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
y = cos(roll/2)*cos(yaw/2)*sin(pitch/2) + cos(pitch/2)*sin(roll/2)*sin(yaw/2);
z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - cos(yaw/2)*sin(roll/2)*sin(pitch/2);

end
