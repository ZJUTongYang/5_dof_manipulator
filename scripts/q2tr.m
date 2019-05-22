function T = q2tr(w, x, y, z)
%Q2TT 将四元数 (Quaternion) 转换为齐次变换 (Homogeneous transform)


T = zeros(4, 4);
T(1, 1) = w^2 + x^2 - y^2 - z^2;
T(1, 2) = 2*x*y - 2*w*z;
T(1, 3) = 2*x*z + 2*w*y;
T(2, 1) = 2*x*y + 2*w*z;
T(2, 2) = w^2 - x^2 + y^2 - z^2;
T(2, 3) = 2*y*z - 2*w*x;
T(3, 1) = 2*x*z - 2*w*y;
T(3, 2) = 2*y*z + 2*w*x;
T(3, 3) = w^2 - x^2 - y^2 + z^2;
T(4, 4) = 1;

end
