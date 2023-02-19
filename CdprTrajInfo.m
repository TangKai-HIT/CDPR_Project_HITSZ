classdef CdprTrajInfo
    % 保存单次轨迹规划信息的类
    %   Detailed explanation goes here
    
    properties (Access = public)
        platformTraj PlatformTraj %末端平台轨迹规划信息对象
        anchorTraj AnchorTraj %动锚点轨迹规划信息对象
        cableTraj CableTraj %绳索在轨迹规划点上信息的对象
        Stiffness; %6XN, 末端刚度信息（x向，y向，z向，x转轴，y转轴，z转轴）
        wrenchSetting WrenchSetting %外力与绳索拉力范围对象
        
        winchMotorsPos; %绞盘电机绝对转角轨迹，3XN
        railMotorsPos; %轨道电机绝对转角轨迹，3XN
        winchMotorsSpeed; %绞盘电机速度轨迹，3XN
        railMotorsSpeed; %轨道电机速度轨迹，3XN
        
        obsBBTraj={}; %障碍物原点轨迹元胞数组；
                            %按障碍物索引存储各障碍物原点在其梯形-
                            %加减速轨迹点(K个,若障碍物静止则K=1)上的位置向量（3XK矩阵）
                                                            
        time; %1XN，时间点序列
        Number=0; %规划点的数量
        timeStep=0.1; %规划时间步长
        
        PlanMode; %记录在APP中规划的模式
        PlanAlgorithm; %记录在APP中规划的算法
    end
    
    methods (Access = public)
        %constructor
        function obj = CdprTrajInfo(maxVelBound_Platfrom, maxAccelBound_Platfrom, ...
                maxVelBound_Anchor, maxStepBound_Anchor, optiFreq, timeStep)
            %init PlatformTraj class
            if nargin==6
                obj.platformTraj = PlatformTraj(maxVelBound_Platfrom, maxAccelBound_Platfrom);
                obj.anchorTraj = AnchorTraj(maxVelBound_Anchor, maxStepBound_Anchor, optiFreq);
                obj.cableTraj = CableTraj();
                obj.wrenchSetting = WrenchSetting();
                obj.timeStep = timeStep;
                obj.Number = 0;
                obj = updateAnchorCon(obj);
            end
        end
        
        function obj = setAnchorMaxStep(obj, maxStepBound_Anchor)
             %set allowed maximum step length of movable anchors 
            obj.anchorTraj.maxStepBound = maxStepBound_Anchor;
            obj = updateAnchorCon(obj);
        end
   
        function obj = setAnchorMaxVel(obj, maxVelBound_Anchor)
        %set allowed maximum average velocity of movable anchors
            obj.anchorTraj.maxAverVelBound = maxVelBound_Anchor;
            obj = updateAnchorCon(obj);
        end
        
    end
    
    methods (Access = private)
        %update step length and max velocity constrain of movable anchors   
        function obj = updateAnchorCon(obj)
            %计算由允许的最大步长决定的最大速度velBound
            velBound = obj.anchorTraj.maxStepBound / (obj.timeStep * obj.anchorTraj.optiFreq);
            if velBound<obj.anchorTraj.maxAverVelBound %当velBound小于允许的最大平均速度
                obj.anchorTraj.maxAverVel = velBound;
                obj.anchorTraj.maxStep = obj.anchorTraj.maxStepBound;
            else
                obj.anchorTraj.maxAverVel = obj.anchorTraj.maxAverVelBound;
                obj.anchorTraj.maxStep = obj.anchorTraj.maxAverVelBound * obj.timeStep;
            end
        end
        
    end
    
end

