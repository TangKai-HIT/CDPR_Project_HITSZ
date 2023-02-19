function [angles, voronoi]=updateVoronoi(univector, vertex, face)
%% 计算绳索所在的voronoi region的函数
% 注：app中有与本函数相同的成员函数
%说明：计算各绳索向量所在的voronoi region，并计算离区域元素的夹角，返回包含相应区域类型和索引的元胞数组、夹角数组
%输入：绳索单位向量，点、面的拓扑信息表
%输出：绳索向量与对应元素正弦值的平方，以及包含相应区域类型和索引的元胞数组，第一行为区域类型，第二行为对应的索引
voronoi=cell(2, size(univector, 2));
angles = zeros(1, size(univector, 2)); %初始设正弦值为0，干涉
for i=1:size(univector, 2)
    findFlag = 0; %找到区域标志
    judgeVertex2 =0 ; %判断点域的判据2：满足判据1的次数
    numEdge =  size(vertex(i).localEdgeVector, 2);
    for j=1:numEdge
        %计算点域判据
        judgeVertex1 = dot(vertex(i).localEdgeVector(:,j), univector(:,i)); %判断点域的判据1
        if  judgeVertex1>0 %不在点域
            %记录索引
            nFaceId_M = vertex(i).n_facesId(j); %middle, nij 对应面的ID
            if j==1
                nFaceId_N = vertex(i).n_facesId(numEdge); %negative, nij-1 对应面的ID
            else
                nFaceId_N = vertex(i).n_facesId(j-1); 
            end
            if j== numEdge
                nEdge_P = 1; %positive, eij+1 对应的ID
            else
                nEdge_P = j+1;
            end
            %计算边判据
            judgeEdge1 = dot(univector(:,i), cross(vertex(i).localEdgeVector(:,j), face(nFaceId_M).normal_vec)); %边判据1
            judgeEdge2 = dot(univector(:,i), cross(vertex(i).localEdgeVector(:,j), face(nFaceId_N).normal_vec)); %边判据2
            %判断是否为边域
            if  judgeEdge1>=0 &&  judgeEdge2<=0%在边域
                voronoi{1, i} = 'e';
                voronoi{2, i} = vertex(i).n_edgesId(j);
                angles(i) = 1-judgeVertex1*judgeVertex1; %与边夹角的正弦值平方
                findFlag =1;
                break;
            else
                %计算面判据
                judgeFace1 = dot(univector(:,i), face(nFaceId_M).normal_vec); %面判据1
                judgeFace2 = judgeEdge1; %面判据2
                judgeFace3 = dot(univector(:,i), cross(vertex(i).localEdgeVector(:, nEdge_P), face(nFaceId_M).normal_vec));
                %判断是否为面域
                if judgeFace1>0 && judgeFace2<0 && judgeFace3>0
                voronoi{1, i} = 'f';
                voronoi{2, i} = vertex(i).n_facesId(j);
                angles(i) = judgeFace1 * judgeFace1; %与面夹角的正弦值平方（=与面法向量夹角的余弦值）
                findFlag =1;
                break;
                end
            end
        else %可能在点域
            judgeVertex2 = judgeVertex2 +1; %满足在点域的条件次数加1
        end
    end
    if ~findFlag 
        if  judgeVertex2==numEdge  %在点域
        voronoi{1, i} = 'v';
        voronoi{2, i} = i;
        angles(i) = 1; %安全区设置正弦值为1，夹角90°，不干涉
        else  %还没找到，说明干涉
        voronoi{1, i} = 'C'; %碰撞
        voronoi{2, i} = 0;
        angles(i) = 0; %碰撞设置正弦值为0，夹角0°，干涉
        end
    end
end