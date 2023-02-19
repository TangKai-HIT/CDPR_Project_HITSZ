function coDistance=updateCableObsDis(Cable, Platform, obsBB)
%% 该算法已废弃，可删除，不要运行！！！！！
%% 计算绳索与障碍物间的距离函数updateCableObsDis(暂时不采用该算法计算,会导致约束不连续)
%说明：计算绳索与障碍物的距离
numbObs = size(obsBB, 2);
numCable = 8;
coDistance = zeros(numbObs, numCable);
for i=1:numCable
    for j=1:numbObs
        vecObs = obsBB(j).origin - Platform.anchorPosition_G(:, i);
        a = dot(vecObs, Cable.UniVector(:, i));
        vecObs_t = a * Cable.UniVector(:, i);
        vecObs_n = vecObs - vecObs_t;
        dis = dot(vecObs, vecObs);
        if dis>=(obsBB(j).Rs)^2 %未进入检测球
            if a<=0 %绳索与到检测球球心向量成钝角
                coDistance(j, i) = sqrt(dis); %距离
                continue; %进入下一轮j
            end

            dis = dot(vecObs_n, vecObs_n);

            if dis>=(obsBB(j).Rs)^2 %绳索未进入检测球
                coDistance(j, i) = sqrt(dis); %距离
                continue; %进入下一轮j
            end
        else %进入检测球，调用openGJK算
            cable = [Cable.TanLinVector(:, i) + Platform.anchorPosition_G(:, i), Platform.anchorPosition_G(:, i)];
            coDistance(j, i) = openGJK(cable, obsBB(j).vertices);
            coDistance(j, i) = coDistance(j, i); %距离
        end
    end
end