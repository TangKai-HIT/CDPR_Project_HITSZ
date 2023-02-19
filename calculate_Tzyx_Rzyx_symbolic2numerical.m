%% calculate_Tzyx_Rzyx_symbolic2numerical程序说明：
% 计算符号变量的齐次变换矩阵，并转为数值函数句柄
clc; clear;
syms x y z a b r
%% Z-Y-X齐次变换矩阵符号计算
R_z = [cos(a)  -sin(a) 0;
            sin(a)  cos(a) 0;
            0          0         1];
        
R_y = [cos(b) 0 sin(b);
               0      1     0;
         -sin(b)   0   cos(b)];
     
 R_x = [ 1   0           0;
             0  cos(r)  -sin(r);
            0  sin(r)   cos(r)];
        
 R_zyx = R_z*R_y*R_x;
 T_zyx = cat(1, R_zyx, [0 0 0]);
  T_zyx(x, y, z, a, b, r) = cat(2, T_zyx, [x y z 1]');
 %% X-Y-Z(RPY)
 R_z = [cos(r)  -sin(r) 0;
            sin(r)  cos(r) 0;
            0          0         1];
        
R_y = [cos(b) 0 sin(b);
               0      1     0;
         -sin(b)   0   cos(b)];
     
 R_x = [ 1   0           0;
             0  cos(a)  -sin(a);
            0  sin(a)   cos(a)];
R_xyz = R_x*R_y*R_z;
 T_xyz = cat(1, R_xyz, [0 0 0]);
  T_xyz(x, y, z, a, b, r) = cat(2, T_xyz, [x y z 1]');
 %% 转为数值函数句柄
Rzyx = matlabFunction(R_zyx);
Tzyx = matlabFunction(T_zyx);
%% Z-Y-X输出为matlab函数m文件
matlabFunction(R_zyx, 'File', 'Rzyx');
matlabFunction(T_zyx, 'File', 'Tzyx');
%% X-Y-Z输出为matlab函数m文件
matlabFunction(R_xyz, 'File', 'Rxyz');
matlabFunction(T_xyz, 'File', 'Txyz');