function f=targetFunction3(X)
%% 目标函数1：targetFunction3
% 说明：优化目标函数3，使每一步步长向量最小
%输入：下一点的优化参量X（包含步长x_step，绳拉力F）
%输出：计算绳长单位向量之和的模（实际计算输出为模的平方）

x_step = X(1:8); %动锚点步长向量
%% 求目标函数
f = norm(x_step); %计算动锚点步长向量范数