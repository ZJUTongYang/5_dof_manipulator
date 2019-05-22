function R = q2r(w, x, y, z)
%Q2R 将四元数 (Quaternion) 转换为旋转矩阵 (Rotation matrix)


R = zeros(3, 3);
R(1, 1) = w^2 + x^2 - y^2 - z^2;
R(1, 2) = 2*x*y - 2*w*z;
R(1, 3) = 2*x*z + 2*w*y;
R(2, 1) = 2*x*y + 2*w*z;
R(2, 2) = w^2 - x^2 + y^2 - z^2;
R(2, 3) = 2*y*z - 2*w*x;
R(3, 1) = 2*x*z - 2*w*y;
R(3, 2) = 2*y*z + 2*w*x;
R(3, 3) = w^2 - x^2 - y^2 + z^2;

end
