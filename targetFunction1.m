function f=targetFunction1(X)
%% 目标函数1：targetFunction1
% 说明：优化目标函数1，使力均匀分布(计算力的方差)
%输入：下一点的优化参量X（包含步长x_step，绳拉力F）
%输出：计算绳长单位向量之和的模（实际计算输出为模的平方）
%% 更新动锚点座位置
tension = X(9:16); %绳拉力
%% 求目标函数
f = var(tension, 1); %计算张力方差