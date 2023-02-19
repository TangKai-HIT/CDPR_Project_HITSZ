%% topology_Information程序
%说明：计算并储存动平台凸包络的拓扑信息
clc;clear;
%预定义结构体与数据
vertex = struct('n_verticesId',[], 'n_edgesId',[], 'n_facesId',[], 'localEdgeVector',[] ,'coordinate',[]);
edge = struct('n_verticesId',[], 'n_edgesId',[], 'n_facesId',[], 'v_facesId',[]);
face = struct('n_verticesId',[], 'n_edgesId',[], 'n_facesId',[], 'v_edgesId',[], 'v_facesId',[], 'normal_vec',[]);
anchorPosition_P = [45 -45 -21.5;
                                             45  45 -21.5;
                                            -45  45 -21.5;
                                            -45 -45 -21.5;
                                          63.64   0  42.5;
                                             0 63.64 42.5;
                                         -63.64   0  42.5;
                                            0 -63.64 42.5]'*1e-3; % 3X8, P-coordinate anchor positions
%% 定义点的拓扑信息(手动添加邻点)
for i=1:8
    vertex(i).coordinate = anchorPosition_P(:,i); %点坐标
        if i>=1 && i<=4  %定义邻接点
            if i == 1 
                vertex(i).n_verticesId = [2 4 5 8];
            elseif i == 4
                vertex(i).n_verticesId = [1 3 7 8];
            else
                vertex(i).n_verticesId =  [i-1 i+1 i+3 i+4];
            end
        else 
            if i==8
                vertex(i).n_verticesId = [1 4 5 7];
            elseif i==5
                vertex(i).n_verticesId = [1 2 6 8];
            else
                vertex(i).n_verticesId =  [i-1 i+1 i-3 i-4];
            end
        end
end
%% 定义边的拓扑信息
%创建点与点相邻表，相邻为1，不相邻为0
isVerNeighbor = zeros(8,8); 
numEdge = 0; %边的数量
for i=1:8
    for j=1:8
        if ismember(i, vertex(j).n_verticesId)
        isVerNeighbor(i,j) = 1;
        end
    end
end
%按点相邻表索引边与点的关系(两点创建一边)
for i=1:8
    for j=i:8
        if isVerNeighbor(i,j) == 1
            numEdge = numEdge + 1;
            edge(numEdge).n_verticesId = [i j];  %边的邻接点索引
            vertex(i).n_edgesId(end+1) = numEdge;  %点的邻接边索引
            vertex(j).n_edgesId(end+1) = numEdge;
        end
    end
end
%边与边的邻接关系
for i=1:numEdge
    for j=1:2 %每边连两点
        for k = 1:4 %每点连4边
            if ~ismember(vertex(edge(i).n_verticesId(j)).n_edgesId(k), edge(i).n_edgesId) && vertex(edge(i).n_verticesId(j)).n_edgesId(k)~=i
                edge(i).n_edgesId(end+1) = vertex(edge(i).n_verticesId(j)).n_edgesId(k);
            end
        end
    end
end
%% 定义面的拓扑信息
numFace = 0; %面的数量
%点的相邻点索引按照连接顺序重新排序（若两个相邻点间也相邻则排在一起）
for i=1:8
    j=1;
    while(1)
        if j==size(vertex(i).n_verticesId,2)
            break;
        end
        if ~ismember(vertex(i).n_verticesId(j),  vertex(vertex(i).n_verticesId(j+1)).n_verticesId)  %当第j个点不与第j+1个相邻时
            if (j+1)<size(vertex(i).n_verticesId,2)   %j+1<最大值，将j+1放在数组最后
                vertex(i).n_verticesId = cat(2, vertex(i).n_verticesId(1:j), vertex(i).n_verticesId(j+2:end), vertex(i).n_verticesId(j+1));
            else  %j+1=最大值，将j+1放在数组最前
                vertex(i).n_verticesId = cat(2, vertex(i).n_verticesId(j+1), vertex(i).n_verticesId(1:j), vertex(i).n_verticesId(j+2:end));
            end
        else
            j=j+1;
        end
    end
end
%按点相邻关系创建面
for i=1:8
    vecBi = vertex(i).coordinate; %原点到点Bi的向量（用于判断外法向量方向）
    for j=1:size(vertex(i).n_verticesId, 2) %遍历点i的邻接点
        temp_nVer1 =  vertex(i).n_verticesId(j); %temp neighbour vertex1
        if j == size(vertex(i).n_verticesId, 2)
            temp_nVer2 = vertex(i).n_verticesId(1); %temp neighbour vertex2
        else
            temp_nVer2 = vertex(i).n_verticesId(j+1); %temp neighbour vertex2
        end
        tempNew = [i temp_nVer1 temp_nVer2];  %待审查面邻接关系的点
        tempVec1 = vertex(temp_nVer1).coordinate - vecBi; %temp vector1 (Bi to vertex1)
        tempVec2 = vertex(temp_nVer2).coordinate - vecBi; %temp vector2 (Bi to vertex2)
        temp_normal = cross(tempVec1, tempVec2); %temp normal vector of the face
        temp_normal = temp_normal/norm(temp_normal); %归一化
        if dot(temp_normal, vecBi)<0  %判断法向量是否朝外
            temp_normal = -temp_normal;
        end
        if numFace>0  %若之前已创建了面
            for k=1:numFace
                overlap = dot(temp_normal, face(k).normal_vec);
                if  overlap>0 && abs(overlap-1)<1e-5  %判断是否与之前的面重叠
                    memberShip = ismember(tempNew, face(k).n_verticesId);
                    for l=1:size(memberShip, 2)
                        if ~memberShip(l)  %若三个点不是该面已登记的邻接点则登记
                            face(k).n_verticesId(end+1) = tempNew(l); %添加面的邻接点索引到面
                            vertex(tempNew(l)).n_facesId(end+1) = k; %添加点的邻接面索引到点
                        end
                    end
                    break; %结束k的循环查找
                else  %本次创建的是新的面
                    if k==numFace
                        numFace = numFace + 1;
                        face(numFace).normal_vec = temp_normal;
                        vertex(i).n_facesId(end+1) = numFace;  %添加点的邻接面索引
                        vertex(temp_nVer1).n_facesId(end+1) = numFace;  %添加点的邻接面索引
                        vertex(temp_nVer2).n_facesId(end+1) = numFace;  %添加点的邻接面索引
                        face(numFace).n_verticesId = cat(2, face(numFace).n_verticesId, tempNew); %添加三个邻接点的索引到面
                    end
                 end
            end
        else %若之前没有创建过面
            numFace = numFace + 1;
            face(numFace).normal_vec = temp_normal;
            vertex(i).n_facesId(end+1) = numFace;  %添加点的邻接面索引
            vertex(temp_nVer1).n_facesId(end+1) = numFace;  %添加点的邻接面索引
            vertex(temp_nVer2).n_facesId(end+1) = numFace;  %添加点的邻接面索引
            face(numFace).n_verticesId = tempNew; %添加三个邻接点的索引到面
        end
    end
end
%边与面的邻接关系
for i=1:size(edge,2)
    for j=1:size(face,2)
        memberShip=ismember(edge(i).n_verticesId, face(j).n_verticesId);
        if sum(memberShip) ==2 %有两个公共点：边相邻
            edge(i).n_facesId(end+1) = j;
            face(j).n_edgesId(end+1) = i;
        elseif sum(memberShip) ==1 %有1个公共点：点相邻
            edge(i).v_facesId(end+1) = j;
            face(j).v_edgesId(end+1) = i;
        end
    end
end
%面与面的邻接关系
for i=1:size(face,2)
    for j=(i+1):size(face,2)
        memberShip=ismember(face(i).n_verticesId, face(j).n_verticesId);
        if sum(memberShip) ==2 %有两个公共点：边相邻
            face(i).n_facesId(end+1) = j;
            face(j).n_facesId(end+1) = i;
        elseif sum(memberShip) ==1 %有1个公共点：点相邻
            face(i).v_facesId(end+1) = j;
            face(j).v_facesId(end+1) = i;
        end
    end
end
%% 定义各点所在的局部单元信息（使邻点、邻边、邻面均按绕该点的逆时针排列）
%由于之前已经排列好点（逆时针或顺时针），通过判断相邻两边叉乘方向判断是否逆时针排列
for i=1:8
    vecBi = vertex(i).coordinate; %原点到点Bi的向量（用于判断外法向量方向）
    Ver1 =  vertex(i).n_verticesId(1); %temp neighbour vertex1
    Ver2 = vertex(i).n_verticesId(2); %temp neighbour vertex2
    Vec1 = vertex(Ver1).coordinate - vecBi; %temp vector1 (Bi to vertex1)
    Vec2 = vertex(Ver2).coordinate - vecBi; %temp vector2 (Bi to vertex2)
    tempCross = cross(Vec1, Vec2);
    if dot(tempCross, vecBi)<0  %顺时针排列
       vertex(i).n_verticesId = flip(vertex(i).n_verticesId);  %则翻转数组
    end
end
%对应点的邻点数组，重新排列点的邻边数组（邻边与邻点相互对应），并填充对应的邻边单位向量（从该点指向其邻点）
for i=1:8
    temp=zeros(1, size(vertex(i).n_edgesId, 2));
    vertex(i).localEdgeVector = zeros(3, size(vertex(i).n_edgesId, 2));
    for j= 1:size(vertex(i).n_edgesId, 2) %第i点的第j个邻边
        nEdgeId = vertex(i).n_edgesId(j);
        for k= 1:size(vertex(i).n_verticesId, 2) %第i点的第k个邻点
            nVertexId = vertex(i).n_verticesId(k); 
            if ismember(nVertexId, edge(nEdgeId).n_verticesId)
                temp(k) = nEdgeId; %说明第k个邻点对应第j个邻边
                vertex(i).localEdgeVector(:, k) = vertex(nVertexId).coordinate - vertex(i).coordinate; %填充对应的邻边向量
                vertex(i).localEdgeVector(:, k) = vertex(i).localEdgeVector(:, k)/norm(vertex(i).localEdgeVector(:, k)); %单位归一化
            end
        end
    end
    vertex(i).n_edgesId = temp;
end
%对应点的邻点数组，重新排列点的邻面数组（邻面与两个邻点相互对应）
for i=1:8
    temp=zeros(1, size(vertex(i).n_facesId, 2));
    for j= 1:size(vertex(i).n_facesId, 2) %第i点的第j个邻面
        nFaceId = vertex(i).n_facesId(j);
        for k= 1:size(vertex(i).n_verticesId, 2) %第i点的第k个邻点
            nVertexId1 = vertex(i).n_verticesId(k); 
            if k<size(vertex(i).n_verticesId, 2)
                nVertexId2 = vertex(i).n_verticesId(k+1); 
            else
                nVertexId2 = vertex(i).n_verticesId(1);
            end
            if sum(ismember([nVertexId1 nVertexId2], face(nFaceId).n_verticesId)) == 2
                temp(k) = nFaceId; %说明第k个邻点对应第j个邻边
            end
        end
    end
    vertex(i).n_facesId = temp;
end
%% 保存生成的几何拓扑信息数据
save('vertex', 'vertex');
save('face', 'face');
save('edge', 'edge');