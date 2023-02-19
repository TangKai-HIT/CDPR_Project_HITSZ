classdef UR5Interface < handle
    % interface to control UR5 arm（句柄类）
    % using URscript command and real-time interface (port 30003)
    
    properties (Access = public)
        IP="192.168.1.10"; %IP Adress(string)
    end
    
    properties (SetAccess=private, GetAccess=public)
        Port=30003; % port
        Client; %TCP Client
        Messages; %Messages received from UR5
        MessageNum; %number of Messages received from UR5
        JointPos_Current; % current joint position of UR5
        ToolPose_Current; % current tool pose of UR5
    end
    
    methods
        function obj = UR5Interface(IP)
            %UR5Interface Construct an instance of this class
            %   Detailed explanation goes here
            if nargin == 1
                obj.IP = IP ;
                obj.Client = tcpip(obj.IP, obj.Port, 'NetworkRole', 'client');
                obj.Client.InputBufferSize = 1200; %total number of data: 1116
                fclose(obj.Client);
            end
        end
        
        function ConnectUR5(obj)
            %start connection with UR5
            %   Detailed explanation goes here
             fopen(obj.Client);
             readUR5Data(obj);
        end
        
        function DisconnectUR5(obj)
            %Disconnect UR5
            %   Detailed explanation goes here
             fclose(obj.Client);
        end
        
        function updateUR5Info(obj)
            %update information of UR5
            fclose(obj.Client);
            fopen(obj.Client);
            readUR5Data(obj);
        end
        
        function moveL_Trapez(obj, Tool_Pose, velocity, acceleration)
            %move tool in linear trajectory using trapezoidal acceleration and deceleration
            %%Tool_Pose: 6X1, m, rad(rotational vector) ；velocity :m/s；acceleration : m/(s^2)
            if size(Tool_Pose, 2)==1
                pose = Tool_Pose';
            else
                pose = Tool_Pose;
            end
            
            sendScript = sprintf("movel(p[%f, %f, %f, %f, %f ,%f], a=%f, v=%f, t=0, r=0)\n", ...
                                            [pose, acceleration, velocity]);
            fprintf(obj.Client, sendScript);
        end
        
    end
    
    methods(Access = private)
        function readUR5Data(obj)
            % read robot state data  from UR5
            obj.MessageNum=fread(obj.Client, 1, 'int');
            obj.Messages = fread(obj.Client, 139, 'double');
            obj.JointPos_Current = obj.Messages(32:37); %Actual joint positions(rad)
            obj.ToolPose_Current = obj.Messages(56:61); %Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), 
                                                                                        %where rx, ry and rz is a rotation vector representation of the tool orientation
        end
        
    end
end

