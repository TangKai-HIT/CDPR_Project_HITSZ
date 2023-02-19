classdef AnchorTraj
    % 保存动锚点单次轨迹规划信息的类
    %   Detailed explanation goes here
    
    properties (Access = public)
        optiFreq; %执行优化算法的频率
        maxAverVelBound; %单步最大允许频率
        maxStepBound; %单步最大允许长度 
        maxStep; %由允许量综合计算得到的最大单步长度
        maxAverVel; %由允许量综合计算得到的最大单步平均速度
        pos; %8Xn, 轨迹上各锚点的局部位置,n:轨迹点数（时间序列点数）
        averVel; %8Xn,轨迹上各锚点平均速度（相对局部正方向）,n:轨迹点数（时间序列点数）
    end
    
    methods (Access = public)
        %constructor
        function obj = AnchorTraj(maxVelBound, maxStepBound, optiFreq)
            %init trajInfo class
            if nargin==3
                obj.maxAverVelBound = maxVelBound; % default allowed maximum average speed in single step
                obj.maxStepBound = maxStepBound; % default allowed maximum step length
                obj.optiFreq = optiFreq; % default frequency to run optimizion
            end
        end
        
    end
end

