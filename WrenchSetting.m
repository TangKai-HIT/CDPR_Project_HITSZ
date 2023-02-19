classdef WrenchSetting
    %设置外力条件与可行力范围的类
    %   Detailed explanation goes here
    
    properties (Access = public)
        wrench; %6X1，外力与外力矩
        feaspTensionRange; %1X2, 张力上下限，(1,1)为绳张力下限，(1,2)为绳张力上限
        rangeCube; %由拉力可行域构建的8维超立方体
    end
    
    methods (Access = public)
        %constructor
        function obj = WrenchSetting(wrench, feaspTensionRange)
            %Construct an instance of this class
            %   初始化赋值外力、可行力区间
            if nargin == 2
                obj.wrench = wrench;
                obj.feaspTensionRange = feaspTensionRange;
            end
        end
        
        function obj=createRangeCube(obj)
            %构建由拉力可行域构建的8维超立方体
            total = 2^8;
            sum = total;
            cube = zeros(8, total);
            row = 1; %行索引
            
            while sum~=1 %迭代构造出立方体所有顶点
            sum = sum/2;
            count = 1;
            valueID = 1;

            for col=1:total %列索引
                cube(row, col) = obj.feaspTensionRange(valueID);
                count = count + 1;

                if count>sum
                    count = 1;
                    valueID = valueID + 1;
                    if valueID>size(obj.feaspTensionRange, 2)
                        valueID = 1;
                    end
                end
            end
            row = row +1;
            end
            
            obj.rangeCube = cube;
        end
        
    end
end

