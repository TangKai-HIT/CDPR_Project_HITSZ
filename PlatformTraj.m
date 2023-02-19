classdef PlatformTraj
    % 保存动平台单次轨迹规划信息的类
    %   Detailed explanation goes here
    
    properties (Access = public)
        maxVelBound;
        maxAccelBound;
        maxVel;
        maxAccel;
        pos;
        eular;
        vel;
        accel;
        magVel;
        magAccel;
    end
    
    methods (Access = public)
        %constructor
        function obj = PlatformTraj(maxVelBound, maxAccelBound)
            %init trajInfo class
            if nargin==2
                obj.maxVelBound = maxVelBound; % default upper boundary of maximum speed
                obj.maxAccelBound = maxAccelBound; % default upper boundary of maximum acceleration
            end
        end
        
    end
end

