classdef CableTraj
    % 保存绳索轨迹规划点上信息的类
    %   Detailed explanation goes here
    
    properties (Access = public)
        cableLength; %8xn, 绳长
        cableTension; %8xn,绳拉力
    end
    
    methods (Access = public)
        %constructor
        function obj = CableTraj()
            %init trajInfo class
            obj.cableLength = zeros(8,1);
            obj.cableTension = zeros(8,1);
        end
        
    end
end

