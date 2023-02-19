%% 说明：inKineSolve_CablePlatformJacobian_symbolic2numeric程序
%先得出符号计算绳长以及绳长相对6个自由坐标的雅可比矩阵块的符号函数，再将其转为数值函数文件
clc; clear; 
%% 求绳长的符号函数（逆运动学解）计算
syms Xc Yc Zc a b y Xbi Ybi Zbi Xai Yai Zai r Rotzi %动平台中心位姿[Xc Yc Zc a b y]；动平台上锚点bi相对动坐标系坐标[Xbi Ybi Zbi]
                                                                                       %动锚点Ai全局坐标[Xai Yai Zai]；滑轮半径：r；动锚点座i坐标系相对G坐标系（Z轴）转角Rotzi
RotZ = @(a) [cos(a) -sin(a) 0;
                        sin(a)  cos(a) 0;
                         0          0        1];
                     
RotX = @(a)  [1     0            0;
                         0 cos(a) -sin(a);
                         0 sin(a)  cos(a)];                   
%基本参数
Ai = [Xai Yai Zai]';
Bi = Tzyx([Xc, Yc, Zc, a, b, y])*([Xbi Ybi Zbi 1]');    %将平台动坐标变到全局坐标系
Bi = Bi(1:3);
vec_BiAi = Ai - Bi;
ai = [0 0 1]'; 
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
vec_CiMi_Ai = vec_BiMi_Ai - vec_BiCi_Ai; % Ai坐标系下向量AiMi，Mi为绳在滑轮上的中心圆切点
vec_CiAi_Ai = - vec_AiCi_Ai;
theta = acos(vec_CiMi_Ai' * vec_CiAi_Ai / (r * r)); % 绳索在滑轮上绕过的弧长对应圆心角
ui = RotZ(Rotzi + pulRot) * ui_Ai; %全局G坐标系下绳索单位向量ui
vec_BiMi = RotZ(Rotzi + pulRot) * vec_BiMi_Ai; %全局G系下的绳长向量BiMi
%求解绳长
Li = L_BiMi + r * theta; %远锚点Bi到近锚点Ai的总绳长

%% 求绳长-六自由坐标的雅克比矩阵块（1X6）
Ji = [diff(Li,'Xc') diff(Li,'Yc') diff(Li,'Zc')  diff(Li,'a') diff(Li,'b') diff(Li,'y')];

%% 转化上述所求符号函数为数值函数
%matlabFunction(Li, vec_BiCi,  pulRot, 'File', 'CDPRInvKine_MF'); % 转化运动学逆解函数，输出绳长,滑轮转角,绳长向量
matlabFunction(Ji, 'File', 'CablePlatJacob_MF'); % 绳长-六自由坐标的雅克比矩阵块求解
