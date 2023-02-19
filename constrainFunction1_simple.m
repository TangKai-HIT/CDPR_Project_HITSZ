function [C, Ceq]=constrainFunction1_simple(X, Platform_Next, MovAnchor, Cable, ...
                                                                                    obsBB, obsNumber, r, wrenchSetting)
%% 计算约束函数constrainFunction1_simple（优化算法, 步长+拉力）
%说明：constrainFunction1.m的简单版，只计算绳索间距，绳索与障碍物间距的不等式约束（小于等于）
%           力平衡方程（用0和1表示的等式约束）
%输入：下一点的优化参量X（包含步长x_step，绳拉力F），上一点的CdprPlatform对象Platform_Next，
%           当前CdprMovAnchor对象MovAnchor，当前CdprCable对象Cable，ObsBB边界框对象obsBB，
%           边界框数量obsNumber，滑轮半径r， 平台力分布参数设置对象wrenchSetting（WrenchSetting类）
%输出：不等式约束C，等式约束Ceq
%% 初始化并记录当前点的绳长向量、绳间距离
x_step = X(1:8); %8个锚点座的步长
tension = X(9:16); %绳拉力
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

%% 计算绳索之间的距离（平方值）
%Cable.ccDistance=updateCableDis(Platform_Next, Cable, vertex);
for i=1:8
        for j=(i+1):8
            index = j-i+((8-1)+(8-i+1))*(i-1)/2; %索引
            % 调用CGAL库计算最短距离平方
            cable_i = [Cable.TanLinVector(:, i) + Platform_Next.anchorPosition_G(:, i), Platform_Next.anchorPosition_G(:, i)];
            cable_j = [Cable.TanLinVector(:, j) + Platform_Next.anchorPosition_G(:, j), Platform_Next.anchorPosition_G(:, j)];
           Cable.ccDistance(index) = calCableSqrDis_CGAL(cable_i, cable_j);
        end
end

%% 等式约束
Ceq = zeros(1, 6); %等式约束：6个力平衡分量
%力平衡等式
%构造结构矩阵（structure matrix）
A_T = zeros(6,8);
for i=1:8
    bi = Platform_Next.anchorPosition_G(:, i) - Platform_Next.pose_G(1:3);
    A_T(:, i) = [Cable.UniVector(:, i); cross(bi,Cable.UniVector(:, i))];
end

if size(tension, 2)~=1
    tension = tension';
end
Ceq(1:end) = A_T * tension + wrenchSetting.wrench;

%% 计算不等式约束中不等式左边的值
C = zeros(1, (8*obsNumber + 28 + 16)); %预分配内存加速运算：绳索到障碍物距离约束+绳索之间的距离约束+动锚点座位置限制
index = 0; %索引初始化，根据条件数叠加
% 绳索到障碍物距离
for i=1:obsNumber
    %sqr_ObsBB_dmin = obsBB(i).d_min * obsBB(i).d_min;
    for j=1:8
        index = index + 1;
        %C(index) = sqr_ObsBB_dmin - Cable.coDistance(i, j); %<=0
        C(index) = obsBB(i).d_min - Cable.coDistance(i, j); %<=0
    end
end

% 绳索之间的距离
sqr_ccDis_min = Cable.ccDis_min * Cable.ccDis_min; %平方最小允许值
count = 0;
for i=1:8
	for j=(i+1):8
		count = count + 1;
        index = index + 1;
        C(index) = sqr_ccDis_min - Cable.ccDistance(count); %<=0
	end
end

%动锚点座位置限制
for i=1:8
     for j=1:2
        index = index + 1;
        if j==1 %动锚点座局部位置下限
            C(index) = MovAnchor.position_LB(i,j) - MovAnchor.positon_L(i); %<=0
        else %动锚点座局部位置上限
            C(index) = MovAnchor.positon_L(i) - MovAnchor.position_LB(i,j); %<=0
        end
    end
end