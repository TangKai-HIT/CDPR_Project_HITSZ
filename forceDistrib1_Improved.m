function [tension, isFeasible]=forceDistrib1_Improved(A_T, wrenchSetting)
%% forceDistrib1_Improved 力分配算法1，采用improved closed-form solution
% 说明：1.力分配算法1，采用improved closed-form solution
%            2.输入： A_T——CDPR结构矩阵， wrenchSetting——外力与范围条件对像（WrenchSetting类）
%            3.输出：tension——分配的拉力(列向量), isFeasible——该拉力是否可行，1可行，0不可行

%reference：Pott A. Cable-driven Parallel Robots: Theory and Application[M].Switzerland: Springer International Publishing AG, 2018: 94-95
%% 定义索引
m =size(A_T, 2) ; %绳索数
r = m - 6; %冗余度
f = zeros(m, 1); %张力
freeIndex = 1:m; %自由项索引
constIndex = []; %被限制项的索引
%% 迭代计算张力
w_p = wrenchSetting.wrench;
f_min = repmat(wrenchSetting.feaspTensionRange(1), 8 ,1);
f_max = repmat(wrenchSetting.feaspTensionRange(2), 8 ,1);

while 1
    if size(constIndex, 2)==0
        w_p_ = w_p;
    else
        w_p_ = w_p + A_T(:, constIndex) * f(constIndex);
    end
    %按自由项索引赋值
    A_T_ = A_T(:, freeIndex);
    f_min_ = f_min(freeIndex);
    f_max_ = f_max(freeIndex);
    f_M_ = 0.5 * (f_max_ + f_min_);
    %计算张力
    A_T_pseInv_ = psedoInverse_svd(A_T_); %伪逆
    %A_T_pseInv_ = (A_T_') * inv((A_T_) * A_T_'); %伪逆
    f_V_ = -A_T_pseInv_ * (w_p_ + A_T_ * f_M_);
    f_ = f_V_ + f_M_;
    f(freeIndex) = f_;
    %判断是否在规定范围
    norm_f_V_ = norm(f_V_);
    lowerBound = 0.5 * (max(f_max) - min(f_min)); %超立方体的二分之一边长
    
    if norm_f_V_ <= lowerBound %在超立方体内切圆(最安全情况)
        break;
    else %不在超立方体内切圆
        count=0;
        for i=1:m
            if f(i) < f_min(i) %小于规定范围
                f(i) = f_min(i);
                constIndex(end + 1)=i; %更新约束项
                r = r - 1; 
            elseif f(i) > f_max(i) %大于规定范围
                f(i) = f_max(i);
                constIndex(end + 1)=i; %更新约束项
                r = r - 1; 
            else %在范围内
                count = count + 1;
            end
        end
        
        if count == m %不在内切圆，但在超立方体内
            break;
        else
            freeIndex = setdiff(freeIndex, constIndex);  %自由项索引集中减掉约束项
        end
        
        if r < 0 %构型无法达到要求，没有合适的解
            break;
        end
    end
end

%% 判断构型是否满足要求
tension = f;
if r >= 0 %找到满足要求的解
    isFeasible = 1;
else %构型不满足要求
    isFeasible = 0;
end