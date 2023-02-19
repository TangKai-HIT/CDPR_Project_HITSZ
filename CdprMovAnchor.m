classdef CdprMovAnchor
    %动锚点座类
    %   Detailed explanation goes here
    
    properties (Access = public)
        positon_origin_G = [468.5 -334 135;
                                           468.5  334 135;
                                          -468.5  334 135;
                                          -468.5 -334 135;
                                           415    0   608;
                                           0    378.5 608;
                                          -415    0   608;
                                           0   -378.5 608]'*1e-3;  % 3X8,global origin position,unit:m 
                                       
        positon_G ; % 3X8,global realtime position,unit:m
        positon_L = zeros(1,8); % 1X8,local realtime position,unit:m
        position_LB = zeros(8,2); % 8X2,local position boundary ,unit:m
        dir_G = [0 0 1;
                      0 0 1;
                      0 0 1;
                      0 0 1;
                      0 1 0;
                     -1 0 0;
                     0 -1 0;
                      1 0 0]';  % 3X8, positive moving direction relative to G-coordinate，unit:mm
        AnRotz_G = pi/180.*[0 180 180 0 90 180 -90 0]; % 1X8, anchor Z-rotation angle relative to G-coordinate，unit:rad
        PulRotz_L = zeros(1,8); % 1X8, pully Z-rotation angle relative to local coordinate，unit:rad
        PulCenter = zeros(3,8); % 3X8, predefine pully center G-coordinate

        caliPosition_L = [121.5 121.5 121.5 121.5 -267.5 -314 -267.5 -314]*1e-3;   % 1X8, local linear rail calibration position，unit:m
        endPosition_L = [450 450 450 450 300 310 300 310]*1e-3;   % 1X8, local linear rail end position，unit:m
        caliPosition_G;
        endPosition_G;
    end
    
    methods (Access = public)
        function  obj=CdprMovAnchor()
            %constructor
            obj.positon_G = obj.positon_origin_G;
            
            fixInd = ones(3,8) - abs(obj.dir_G); % fixed coordinate index
            fixCoord = fixInd .* obj.positon_G; % fixed coordinates
            caliCoord = obj.dir_G .* obj.caliPosition_L; % calibration position coordinates
            endCoord = obj.dir_G .* obj.endPosition_L; % endPosition coordinates
            obj.caliPosition_G = caliCoord + fixCoord; % 1X8, global linear rail calibration position，unit:m
            obj.endPosition_G = endCoord + fixCoord; % 1X8, global linear rail end position，unit:m
            
            posL = obj.positon_G(find(obj.dir_G~=0));
            obj.position_LB(:, 2) = obj.endPosition_L' - posL; %upper boundary of local position 
            obj.position_LB(:, 1) = (obj.caliPosition_L' - posL) + repmat(5*1e-3, 8, 1); %lower boundary of local position=calibration position + 5mm
        end
        
        function obj=setPosL(obj, posL)
            %set local position 
            if size(posL, 1)~=1
                posL = posL';
            end
            obj.positon_L = posL;
            %update global position of movable anchors by local position
            obj.positon_G = obj.positon_origin_G + obj.dir_G .* obj.positon_L;
        end
        
         function obj=resetAllPos(obj)
             %reset all anchor position to origin
             obj.positon_G = obj.positon_origin_G;
             obj.positon_L = zeros(1,8);
         end
         
    end
end

