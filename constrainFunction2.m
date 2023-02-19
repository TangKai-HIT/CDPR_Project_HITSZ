function [C, Ceq]=constrainFunction2(X, Platform_Next, PlatAnchorPosition_Now, MovAnchor, Cable, ...
                                                                     obsBB, obsNumber, vertex, face, r, wrenchSetting)
%% constrainFunction2 计算约束函数（优化算法, 变量：步长或步长+拉力）
%说明：计算绳索间距，绳索与障碍物间距，绳索与动平台间距的不等式约束（小于等于）
%   以及当绳索间距小于一定距离时的干涉情况（用0和1表示的等式约束）
%输入：下一点的优化参量X（仅步长x_step或步长x_step+绳拉力F），上一点的CdprPlatform对象Platform_Next，
%           当前动平台锚点座位置PlatAnchorPosition，当前CdprMovAnchor对象MovAnchor，当前CdprCable对象Cable，ObsBB边界框对象obsBB，
%           边界框数量obsNumber，动平台包络多面体的几何信息vertex和face，滑轮半径r， 平台力分布参数设置对象wrenchSetting（WrenchSetting类）
%输出：不等式约束C，等式约束Ceq
%% 初始化并记录当前点的绳长向量、绳间距离
x_step = X(1:8); %8个锚点座的步长
tension = []; %绳拉力
if (size(X,2)==16) || (size(X,1)==16)
    tension = X(9:16); %绳拉力
end
TanLinVector_Now = Cable.TanLinVector;
ccDistance_Now = Cable.ccDistance;
%% 更新动锚点座位置
MovAnchor=MovAnchor.setPosL(MovAnchor.positon_L + x_step);
%% 计算绳长向量
% 拷贝并修改自CDPRInvKine.m（只计算绳长向量TanLinVector）
RotZ = @(a) [cos(a) -sin(a) 0;
                        sin(a)  cos(a) 0;
                         0          0        1];
                     
RotX = @(a)  [1     0            0;
                         0 cos(a) -sin(a);
                         0 sin(a)  cos(a)]; 
for i=1:8
    Rotzi = MovAnchor.AnRotz_G(i);
    Ai = MovAnchor.positon_G(:,i);
    Bi = Platform_Next.anchorPosition_G(:, i); 
    %基本参数
    vec_BiAi = Ai - Bi;
    %ai = [0 0 1]'; 
    %求滑轮转角
    n1i = RotZ(Rotzi) * [1 0 0]';
    n2i = RotZ(Rotzi) * [0 1 0]';
    pulRot = atan2(vec_BiAi' * n1i, abs(vec_BiAi' * n2i));
    %求解切线绳长
    vec_AiCi_Ai = [0 r 0]'; % Ai坐标系下向量AiCi，Ci为滑轮圆心
    vec_BiAi_Ai = RotZ(Rotzi + pulRot)' * vec_BiAi; % 将向量BiAi变换到动锚点Ai坐标系
    vec_BiCi_Ai = vec_BiAi_Ai + vec_AiCi_Ai ;
    L_BiCi = norm(vec_BiCi_Ai);
    L_BiMi = sqrt(L_BiCi * L_BiCi - r * r); %切线绳长BiMi
    %求解绳索单位向量、切线向量
    theta1 = asin(r / L_BiCi);
    ui_Ai = RotX(theta1)' * vec_BiCi_Ai / L_BiCi; %Ai坐标系下绳索单位向量ui
    vec_BiMi_Ai = L_BiMi * ui_Ai; %Ai系下的绳长向量BiMi
    ui = RotZ(Rotzi + pulRot) * ui_Ai; %全局G坐标系下绳索单位向量ui
    vec_BiMi = RotZ(Rotzi + pulRot) * vec_BiMi_Ai; %全局G系下的绳长向量BiMi
    Cable.UniVector(:, i) = ui;
    Cable.TanLinVector(:, i) = vec_BiMi;
end

%% 计算绳索到障碍物的距离(openGJK)(正常值)
for i=1:obsNumber
    for j=1:8
        cable = [Cable.TanLinVector(:, j) + Platform_Next.anchorPosition_G(:, j), Platform_Next.anchorPosition_G(:, j)];
        Cable.coDistance(i, j) = openGJK(cable, obsBB(i).vertices); %openGJK
    end
end

%% 计算绳索与动平台之间的距离（正弦平方值）
[Cable.cpDistance, Cable.voronoi]=updateVoronoi(Cable.UniVector, vertex, face);

%% 计算绳索之间的距离（平方值）
Cable.ccDistance=updateCableDis(Platform_Next, Cable, vertex);
% for i=1:8
%         for j=(i+1):8
%             index = j-i+((8-1)+(8-i+1))*(i-1)/2; %索引
%             % 调用CGAL库计算最短距离平方
%             cable_i = [Cable.TanLinVector(:, i) + Platform_Next.anchorPosition_G(:, i), Platform_Next.anchorPosition_G(:, i)];
%             cable_j = [Cable.TanLinVector(:, j) + Platform_Next.anchorPosition_G(:, j), Platform_Next.anchorPosition_G(:, j)];
%            Cable.ccDistance(index) = calCableSqrDis_CGAL(cable_i, cable_j);
%         end
% end

% %% 计算动平台到障碍物距离(openGJK)(正常值)
% for i=1:obsNumber
%     Platform_Next.poDistance(i) = openGJK(Platform_Next.anchorPosition_G, obsBB(i).vertices);
% end

%% 等式约束
Ceq = zeros(1, 28 + 1); %等式约束：28个绳间干涉检查 + 结构矩阵满秩
%绳穿越干涉检查(当前点绳间距离小于一定值时检查)
%遍历两两绳之间
sqr_ccDis_Intersect = Cable.ccDis_Intersect * Cable.ccDis_Intersect;
for i=1:8
	for j=(i+1):8
		index = (j-i) + (7+(9-i))*(i-1)/2;
        if ccDistance_Now(index) < sqr_ccDis_Intersect %绳间距离小于最短检测距离时
            test1 = [(TanLinVector_Now(:, i)+PlatAnchorPosition_Now(:, i)), PlatAnchorPosition_Now(:, i), ...
                (Cable.TanLinVector(:, i)+Platform_Next.anchorPosition_G(:, i)), Platform_Next.anchorPosition_G(:, i)];
            
            test2 = [(TanLinVector_Now(:, j)+PlatAnchorPosition_Now(:, j)), PlatAnchorPosition_Now(:, j), ...
                (Cable.TanLinVector(:, j)+Platform_Next.anchorPosition_G(:, j)), Platform_Next.anchorPosition_G(:, j)];
            
            Ceq(index) = cableIntersectTest(test1, test2);
        end
	end
end

%结构矩阵满秩
 %构造结构矩阵（structure matrix）
A_T = zeros(6,8);
for i=1:8
    bi = Platform_Next.anchorPosition_G(:, i) - Platform_Next.pose_G(1:3);
    A_T(:, i) = [Cable.UniVector(:, i); cross(bi,Cable.UniVector(:, i))];
end
Ceq(29)=rank(A_T) - 6;

% %力平衡等式（外力在力可行域立方体的线性映射范围内）
% %判断外力是否在映射后的立方体中
% w_p = wrenchSetting.wrench;
% Ceq(30)= inhull(-w_p', (A_T*wrenchSetting.rangeCube)') - 1; %运算太慢

if size(tension,2)>0 %当拉力作为优化变量时
    if size(tension, 2)~=1
    tension = tension';
    end
    Ceq(30:37) = A_T * tension + wrenchSetting.wrench;
end
%% 计算不等式约束中不等式左边的值
C = zeros(1, (8*obsNumber + 8+ 28 + 16 + 1)); %预分配内存加速运算：绳索到障碍物距离+绳索与动平台间的距离+绳索之间的距离+动锚点座位置限制 + 拉力解平面约束
index = 0; %索引初始化，根据条件数叠加
% 绳索到障碍物距离
for i=1:obsNumber
    for j=1:8
        index = index + 1;
        C(index) = (obsBB(i).d_min - Cable.coDistance(i, j)) * 1e3; %<=0, , 乘1000提高精度
    end
end

% 绳索与动平台间的距离(平方)
sqr_cpDis_min = Cable.cpDis_min * Cable.cpDis_min; %平方最小允许值
for i=1:8
    index = index + 1;
    C(index) = sqr_cpDis_min - Cable.cpDistance(i); %<=0
end

% 绳索之间的距离(平方)
sqr_ccDis_min = Cable.ccDis_min * Cable.ccDis_min; %平方最小允许值
count = 0;
for i=1:8
	for j=(i+1):8
		count = count + 1;
        index = index + 1;
        C(index) = sqr_ccDis_min - Cable.ccDistance(count); %<=0
	end
end

% % 动平台到障碍物的距离
% for i=1:obsNumber
%     index = index + 1;
%     C(index) = obsBB(i).d_min - Platform_Next.poDistance(i); %<=0
% end

%动锚点座位置限制
for i=1:8
     for j=1:2
        index = index + 1;
        if j==1 %动锚点座局部位置下限
            C(index) = (MovAnchor.position_LB(i,j) - MovAnchor.positon_L(i)) * 1e3; %<=0, 乘1000提高精度
        else %动锚点座局部位置上限
            C(index) = (MovAnchor.positon_L(i) - MovAnchor.position_LB(i,j)) * 1e3; %<=0, 乘1000提高精度
        end
    end
end

if size(tension,2)==0 %当不用拉力作为优化变量时
    % %%构造结构矩阵（structure matrix）
    % A_T = zeros(6,8);
    % for i=1:8
    %     bi = Platform_Next.anchorPosition_G(:, i) - Platform_Next.pose_G(1:3);
    %     A_T(:, i) = [Cable.UniVector(:, i); cross(bi,Cable.UniVector(:, i))];
    % end
    
    f_max= wrenchSetting.feaspTensionRange(2); %最大张力
    f_min= wrenchSetting.feaspTensionRange(1); %最小张力
    %%力分配算法求解拉力
    [f,  ~] = forceDistrib1_Improved(A_T, wrenchSetting); 
    for i=1:8
        for j=1:2
            index = index + 1;
            if j==1
                C(index) = f(i) - f_max; %<=0
            else
                C(index) = f_min - f(i); %<=0
            end
        end
    end
end