classdef CdprPlatform 
    %末端动平台类
    %   Detailed explanation goes here
    
    properties (Access = public)
        anchorPosition_P;
        anchorPosition_G;
        anchorDis;
        anchorUniVec_P;
        pose_G;
        originPose_G;
        boundary;
        poDistance;
    end
    
    methods (Access = public)
        function obj = CdprPlatform()
            obj.anchorPosition_P = [45 -45 -21.5;
                                             45  45 -21.5;
                                            -45  45 -21.5;
                                            -45 -45 -21.5;
                                          63.64   0  42.5;
                                             0 63.64 42.5;
                                         -63.64   0  42.5;
                                            0 -63.64 42.5]'*1e-3; % 3X8, P-coordinate anchor positions
            obj.anchorPosition_G = zeros(3,8); % 3X8, predefine G-coordinate anchor positions 
            load("anchorDistance.mat");
            obj.anchorDis = anchorDistance; % 1X28, load pre-calculated distance between each anchor points
            load("anchorUniVec_P.mat");
            obj.anchorUniVec_P=anchorUniVec_P; % 3X28, load pre-calculated unit vector between each anchor points 
            obj.originPose_G = [0 0 37 0 0 0]'*1e-3; % 6X1, original global center position and Z-X-Y eular angle [0 0 37 0 0 0]'*1e-3
            obj.pose_G = obj.originPose_G; % 6X1, real time global center position and Z-X-Y eular angle
            obj.boundary = [-300 300;
                                    -250 250;
                                     37  500] * 1e-3; % 3X2, platform position boundary, m
            obj.poDistance = [ ];
        end
        
        function obj=setPoseG(obj, pose_G)
            if size(pose_G, 1)~=6
                pose_G = pose_G';
            end
            obj.pose_G = pose_G;
            %update Platform anchors' G-coordinate
            T_GP = Tzyx(obj.pose_G);
            temPosition_P = cat(1, obj.anchorPosition_P, ones(1,8));
            temPosition_G = T_GP * temPosition_P;
            obj.anchorPosition_G = temPosition_G(1:3,:);
        end
        
        function obj=resetPoseG(obj)
            %reset platform pose to the original one
            obj = obj.setPoseG(obj.originPose_G);
        end
    end
    
end

