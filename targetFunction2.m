function f=targetFunction2(X, Platform_Next, MovAnchor, Cable, r)
%% 目标函数1：targetFunction2
% 说明：优化目标函数2，使等效刚度矩阵K_e达到优化（只计算与绳刚度相关的对称部分）
%输入：下一点的优化参量X（步长x_step），下一点的CdprPlatform，当前的CdprMovAnchor对象, 当前的CdprCable对象以及滑轮半径
%输出：计算绳长单位向量之和的模（实际计算输出为模的平方）
%% 更新动锚点座位置
x_step = X(1:8); %8个锚点座的步长
MovAnchor = MovAnchor.setPosL(MovAnchor.positon_L + x_step);
%% 计算绳长单位向量与绳长
for i=1:8
    Rotzi = MovAnchor.AnRotz_G(i);
    Ai = MovAnchor.positon_G(:,i);   
   [Cable.Length(i), Cable.UniVector(:, i), ~, ~, ~] = CDPRInvKine(Platform_Next.pose_G, Platform_Next.anchorPosition_P(:,i), Ai, r, Rotzi);
end
%% 构造结构矩阵（structure matrix）
A_T = zeros(6,8);
for i=1:8
    bi = Platform_Next.anchorPosition_G(:, i) - Platform_Next.pose_G(1:3);
    A_T(:, i) = [Cable.UniVector(:, i); cross(bi, Cable.UniVector(:, i))];
end
%% 计算等效刚度矩阵
for i =1:8
    Cable.K(i) = Cable.E * Cable.A / (Cable.Length(i) + Cable.Fixed_Length(i) + MovAnchor.positon_L(i));
end
K_e = A_T * diag(Cable.K) * A_T';
%% 求目标函数
f = -trace(K_e); %计算等效刚度矩阵的迹取负（各主轴刚度平均值最大） 