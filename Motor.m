classdef Motor < handle
    %MaxonRE35 电机CAN总线控制的类（句柄类） 
    %   Detailed explanation goes here
    
    properties (SetAccess=private, GetAccess=public)
        GroupID = 0; %组号
        MemberID = 1; %成员号
        CANID_Hex = '01' %CAN消息帧中驱动器编组ID号（16进制）
        CAN CdprCAN %连接的USB-CAN
        
        TransRatio = 13.76; %传动比
        PulsePR = 2000; %转一圈编码器脉冲数
        
        SetPosition = 0; %设定要达到的绝对位置（rad）
        SetSpeed = 0; %设定要限制的绝对速度（RPM）
        MaxPWM = 5000; %设定最大的PWM值
        RealPosition = 0; %编码器读取的实时绝对位置（rad）
        RealSpeed = 0; %编码器读取的实时绝对速度（RPM）
        
        DriverMode = 1; % position mode:0   speed&position mode:1
    end
    
    methods
        function obj = Motor(GroupID, MemberID, CAN)
            %Construct an instance of this class
            %   Detailed explanation goes here
            if nargin == 3
                obj.GroupID = GroupID;
                obj.MemberID = MemberID;
                obj.CAN = CAN;
                obj.CANID_Hex = strcat(dec2hex(GroupID,1), dec2hex(MemberID,1));
            end
        end
        
        function setMaxPWM(obj, MaxPWM)
            %setMaxPWM set PWM of the driver
            obj.MaxPWM = MaxPWM;
        end
        
        function setMotorID(obj, GroupID, MemberID)
            %setMotorID set group ID and member ID of the driver
            %   Detailed explanation goes here
            obj.GroupID = GroupID;
            obj.MemberID = MemberID;
            obj.CANID_Hex = strcat(dec2hex(GroupID,1), dec2hex(MemberID,1));
        end
        
        function setDriverMode(obj, Mode, push2queue_Flag)
            %setDriverMode set working mode of the driver
            %   position mode:0   speed&position mode:1
            %   push2queue_Flag: true->push the message to transmit queue(not send the meassage directly)
            %   false->send the meassage directly  
            obj.DriverMode = Mode;
            obj.CAN.setDriverMode(obj.CANID_Hex, Mode, push2queue_Flag);
        end
        
        function resetDriver(obj, push2queue_Flag)
            %reset Robomodule Driver, clear up
            %   push2queue_Flag: true->push the message to transmit queue(not send the meassage directly)
            %   false->send the meassage directly 
            obj.CAN.resetDriver(obj.CANID_Hex, push2queue_Flag);
        end
        
        function setMotorMotion(obj, SetPosition, SetSpeed, push2queue_Flag)
            %set target Motor motion parameters
            % SetPosition-rad, SetSpeed-rpm
            %   push2queue_Flag: true->push the message to transmit queue(not send the meassage directly)
            %   false->send the meassage directly 
            obj.SetPosition = SetPosition;
            obj.SetSpeed = SetSpeed;
            rawSpeed = SetSpeed * obj.TransRatio; %unit:rpm
            rawPosition = SetPosition / (2*pi) * obj.TransRatio * obj.PulsePR; %unit:qc
            obj.CAN.setMotorMotion(obj.CANID_Hex, obj.MaxPWM, rawSpeed, rawPosition, obj.DriverMode, push2queue_Flag);
        end
        
        function requestMotorInfo(obj, sendPeriod1, sendPeriod2)
            %request motor driver to send information
            % sendPeriod1: time period of sending information includeing current, speed and position
            % sendPeriod2: time period of sending information includeing voltage of CTL1/CTL2/DSIN/CHZ ,PWM output value
            functionID = 'A';
            ID = {strcat(obj.CANID_Hex, functionID)};
            Data={{dec2hex(sendPeriod1, 2)}, {dec2hex(sendPeriod2, 2)}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}};
            obj.CAN.sendData2Driver(ID, Data);
        end
        
    end
end

