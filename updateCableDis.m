function ccDistance=updateCableDis(Platform, Cable, vertex)
%% 计算各绳索之间的距离函数updateCableDis
    %说明：计算各绳索之间的距离平方值
    ccDistance = zeros(1,28);
    for i=1:8
        for j=(i+1):8
            index = j-i+((8-1)+(8-i+1))*(i-1)/2; %索引
            l_ij = Platform.anchorDis(index); 
            uniVec_ij = Platform.anchorUniVec_P(:,index);
            uniVec_ji = -uniVec_ij;
            % region判断(一级)
            % i，j均在点域？
            cond11 = (Cable.voronoi{1,i}=='v'); 
            cond21 = (Cable.voronoi{1,j}=='v');
            if cond11 && cond21
                ccDistance(index) = l_ij*l_ij;
                continue;
            end
            % i在点域且j为i的邻点且u_j在i不相邻面域或边域
            % 或j在点域且j为i的邻点且u_i在j不相邻面域或边域
            cond12 = ismember(j, vertex(i).n_verticesId); cond22 = cond12;
            cond13 = (Cable.voronoi{1,j}=='e' && ~ismember(Cable.voronoi{2,j}, vertex(i).n_edgesId))...
                    || (Cable.voronoi{1,j}=='f' && ~ismember(Cable.voronoi{2,j}, vertex(i).n_facesId));
            cond23 = (Cable.voronoi{1,i}=='e' && ~ismember(Cable.voronoi{2,i}, vertex(j).n_edgesId))...
                    || (Cable.voronoi{1,i}=='f' && ~ismember(Cable.voronoi{2,i}, vertex(j).n_facesId));
            if (cond11 && cond12 && cond13) || (cond21 && cond22 && cond23)
                if cond11 % i在点域
                    a = dot(uniVec_ji, Cable.UniVector_P(:, j));
                else % j在点域
                    a = dot(uniVec_ij, Cable.UniVector_P(:, i));
                end
                if a>=0
                    ccDistance(index) = l_ij*l_ij*(1-a*a);
                    continue;
                else
                    ccDistance(index) = l_ij*l_ij;
                    continue;
                end
            end

            % 方位判断(二级)
            a1 = dot(Cable.UniVector_P(:, i), uniVec_ij);
            a2 = dot(Cable.UniVector_P(:, j), uniVec_ji);
            uniVec_n1 = Cable.UniVector_P(:, i) - a1*uniVec_ij;
            uniVec_n2 = Cable.UniVector_P(:, j) - a2*uniVec_ji;
            cond3 = dot(uniVec_n1, uniVec_n2);
            if a1<=0 && a2<=0
                ccDistance(index) = l_ij*l_ij; continue;
            elseif (a1<=0 && abs(a1)>=abs(a2)) || (cond3<=0 && a2>=a1)
                ccDistance(index) = l_ij*l_ij*dot(uniVec_n2, uniVec_n2); continue;
            elseif (a2<=0 && abs(a2)>=abs(a1)) || (cond3<=0 && a1>a2)
                ccDistance(index) = l_ij*l_ij*dot(uniVec_n1, uniVec_n1); continue;
            end

            % 调用CGAL库计算最短距离平方
            cable_i = [Cable.TanLinVector(:, i) + Platform.anchorPosition_G(:, i), Platform.anchorPosition_G(:, i)];
            cable_j = [Cable.TanLinVector(:, j) + Platform.anchorPosition_G(:, j), Platform.anchorPosition_G(:, j)];
            ccDistance(index) = calCableSqrDis_CGAL(cable_i, cable_j);
        end
    end
  end