function [Li, ui, vec_BiMi, pulCenter, pulRot] = CDPRInvKine(P, Bi_P, Ai, r, Rotzi)
%CDPRINVKINE 计算CDPR逆运动学，算出绳长，出绳滑轮转角，绳长向量
%    input describtion:
%    P: pose and position of the platform; Bi_P: coordinate of the platform anchor
%    in P-coordinate; Ai: coordinate of the moving anchor in G-coordinate;
%    r: radius of pully; Rotzi: moving anchor Z-rotation angle relative to G-coordinate

Xc = P(1);
Yc = P(2);
Zc = P(3);
a = P(4);
b = P(5);
y = P(6);
Xai = Ai(1);
Yai = Ai(2);
Zai = Ai(3);
Xbi = Bi_P(1);
Ybi = Bi_P(2);
Zbi = Bi_P(3);

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
vec_CiMi_Ai = vec_BiMi_Ai - vec_BiCi_Ai; % Ai坐标系下向量AiMi，Mi为绳在滑轮上的中心圆切点
vec_CiAi_Ai = - vec_AiCi_Ai;
theta = acos(vec_CiMi_Ai' * vec_CiAi_Ai / (r * r)); % 绳索在滑轮上绕过的弧长对应圆心角
ui = RotZ(Rotzi + pulRot) * ui_Ai; %全局G坐标系下绳索单位向量ui
vec_BiMi = RotZ(Rotzi + pulRot) * vec_BiMi_Ai; %全局G系下的绳长向量BiMi
%求解绳长
Li = L_BiMi + r * theta; %远锚点Bi到近锚点Ai的总绳长
%求解滑轮中心点在G系下坐标
pulCenter = Ai + RotZ(Rotzi + pulRot) * vec_AiCi_Ai; 