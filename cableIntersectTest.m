function intersectFlag=cableIntersectTest(cable1, cable2)
%% 两绳索从一点到下一邻近点过程中的干涉检查函数cableIntersectTest
% 说明：两个小步长迭代点间绳索扫掠图形的干涉检查(扫掠图形共三种情况：线段、三角形、四面体)
% 输入：cable1--存放绳索1在两个位置时端点的坐标（1、2列存放前一点时动锚点、平台锚点端点坐标，3、4列存放下一点时动锚点、平台锚点端点坐标）
%            cable2--存放绳索2在两个位置时端点的坐标（1、2列存放前一点时动锚点、平台锚点端点坐标，3、4列存放下一点时动锚点、平台锚点端点坐标）
% 输出：1--干涉，0--不干涉
%% 计算端点是否重合
inputShape1 = [];
inputShape2 = [];
% 距离小于1mm则认为重合（单位m）
% 绳1扫掠的图形inputShape1
if abs(norm(cable1(:, 1) - cable1(:, 3)))<1e-3 
    inputShape1(:, 1) = cable1(:, 1);
else
    inputShape1(:, 1:2) = [cable1(:, 1), cable1(:, 3)];
end

if abs(norm(cable1(:, 2) - cable1(:, 4)))<1e-3 
    inputShape1(:, end+1) = cable1(:, 2);
else
    inputShape1(:, (end+1):(end+2)) = [cable1(:, 2), cable1(:, 4)];
end
% 绳2扫掠的图形inputShape2
if abs(norm(cable2(:, 1) - cable2(:, 3)))<1e-3 
    inputShape2(:, 1) = cable2(:, 1);
else
    inputShape2(:, 1:2) = [cable2(:, 1), cable2(:, 3)];
end

if abs(norm(cable2(:, 2) - cable2(:, 4)))<1e-3 
    inputShape2(:, end+1) = cable2(:, 2);
else
    inputShape2(:, (end+1):(end+2)) = [cable2(:, 2), cable2(:, 4)];
end

%% 调用CGAL库写的函数检查两个扫掠图形是否干涉
intersectFlag = doIntersect_CGAL(inputShape1, inputShape2);