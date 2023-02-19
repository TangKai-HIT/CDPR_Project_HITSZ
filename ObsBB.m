classdef ObsBB
    % obstacle boundind box 障碍物边界框类
    %   Detailed explanation goes here
    
    properties (Access = public)
        ds;  %安全距离
        d_min; %最小允许距离
        
        velocity=zeros(3,1); %3X1,速度向量, m/s
        acceleration = 0; %加速度大小, m/(s^2)
    end
    
    properties (SetAccess=private, GetAccess=public)
        length; %X轴向长度
        width; %Y轴向宽度
        height; %Z轴向高度
        vertices;  %3X8, 存储顶点位置向量
        verticesDir; % 顶点坐标相对原点的方向向量（各元素绝对值为1，非单位向量，用于更新顶点坐标）
        origin=zeros(3,1);  %3X1, 原点位置
    end
    
    methods (Access = public)
        function obj = ObsBB(length, width, height, origin, velocity, acceleration, d_min, ds)
            %Construct an instance of this class
            if nargin==8
                obj.length = length;
                obj.width = width;
                obj.height = height;
                
                % 按规则计算各顶点坐标相对原点的方向向量
                index=0;
                dir = [1; -1];
                vertices = repmat(origin, 1 , 8);
                obj.verticesDir = zeros(3,8);
                for i=1:2
                    for j=1:2
                        for k=1:2
                            index = index+1;
                            if dir(j)>0
                                obj.verticesDir(:, index) = [dir(k); dir(j); dir(i)];
                            else
                                obj.verticesDir(:, index) = [-dir(k); dir(j); dir(i)];
                            end
                        end
                    end
                end
                
                obj.vertices = vertices + obj.verticesDir .* [length/2; width/2; height/2]; 
                obj.origin = origin;
                obj.acceleration = acceleration;
                obj.velocity = velocity;
                
                if ds<=d_min
                    ds = 2*d_min;
                end
                
                obj.ds = ds;
                obj.d_min = d_min;
            end            
        end 
        
        function obj=setEdgeLength(obj, Length, edgeType)
            %设置边长
            % 输入：边长、边的类型（长？宽？高？）
            switch edgeType
                case 'length'
                    obj.length = Length;
                case 'width'
                    obj.width = Length;
                case 'height'
                    obj.height = Length;
            end
            
            vertices = repmat(obj.origin, 1 , 8);
           
            obj.vertices = vertices + obj.verticesDir .* [obj.length/2; obj.width/2; obj.height/2];
        end
        
        function obj=setOriginPosition(obj, position)
            %设置中心点坐标
            if size(position, 2)~=1
                position = position';
            end
               
            differ = position - obj.origin; %偏移量
            obj.origin = position;
            differ = repmat(differ, 1, 8);
            obj.vertices = obj.vertices + differ;
        end
        
        function obj=setMinDis(obj, d_min)
            %设置最小允许距离
            %   Detailed explanation goes here
            obj.d_min = d_min;
            if obj.ds<=d_min
                    obj.ds = 2*d_min;
            end
        end
        
        function obj=setSafeDis(obj, ds)
            %设置最小允许距离
            %   Detailed explanation goes here
            obj.ds = ds;
            if obj.ds<=obj.d_min
                    obj.ds = 2*obj.d_min;
            end
        end
        
        function showVelDir(obj, ax, linspec)
            % 在ax图中画出bounding box 速度方向图
            % 输入：图的ax句柄， 线型与颜色linspec
            if norm(obj.velocity)>0
                dirVector = obj.velocity/norm(obj.velocity);
                showVector = dirVector * sqrt(obj.length^2 + obj.width^2 + obj.height^2)/2 * 1.5; %缩放矢量长度以超出边界框
                X=obj.origin(1); Y=obj.origin(2); Z=obj.origin(3);
                U=showVector(1); V=showVector(2); W=showVector(3);
                quiver3(ax, X, Y, Z, U, V, W, linspec); %矢量图
            end
        end
            
        function plot(obj, ax)
            % 在ax图中画出bounding box 3D线框图
            %   Detailed explanation goes here
            X0=[obj.vertices(1, 1:4) obj.vertices(1, 1)];
            Y0=[obj.vertices(2, 1:4) obj.vertices(2, 1)];
            Z0=[obj.vertices(3, 1:4) obj.vertices(3, 1)];
            
            X1=[obj.vertices(1, 5:8) obj.vertices(1, 5)];
            Y1=[obj.vertices(2, 5:8) obj.vertices(2, 5)];
            Z1=[obj.vertices(3, 5:8) obj.vertices(3, 5)];
            
            if nargin==2
                hold(ax, 'on');                
                plot3(ax, X0, Y0, Z0);                
                plot3(ax, X1, Y1, Z1);
                for i=1:4
                    X=[obj.vertices(1, i) obj.vertices(1, i+4)];
                    Y=[obj.vertices(2, i), obj.vertices(2, i+4)];
                    Z=[obj.vertices(3, i) obj.vertices(3, i+4)];
                    plot3(ax, X, Y, Z);
                end
            else
               hold on;
                plot3(X0, Y0, Z0);                
                plot3(X1, Y1, Z1);
               for i=1:4
                    X=[obj.vertices(1, i) obj.vertices(1, i+4)];
                    Y=[obj.vertices(2, i), obj.vertices(2, i+4)];
                    Z=[obj.vertices(3, i) obj.vertices(3, i+4)];
                    plot3(X, Y, Z);
               end
            end
        end
        
        function fill3(obj, ax, color)
            % 在ax图中画出bounding box 3D填充图
            %   Detailed explanation goes here
            index=zeros(4,6); %每列为一组矩形的点的索引，共6个面（6个矩形）
            index(:,1)=[1;2;3;4];
            index(:,2)=[5;6;7;8];
            index(:,3)=[1;5;8;4];
            index(:,4)=[1;2;6;5];
            index(:,5)=[2;3;7;6];
            index(:,6)=[3;4;8;7];
            X = zeros(4,6); Y=zeros(4,6); Z=zeros(4,6);
            for i=1:6
                for j=1:4
                    X(j,i) = obj.vertices(1,index(j,i));
                    Y(j,i) = obj.vertices(2,index(j,i));
                    Z(j,i) = obj.vertices(3,index(j,i));
                end
            end
            fill3(ax, X, Y, Z, color);
        end

    end
end

