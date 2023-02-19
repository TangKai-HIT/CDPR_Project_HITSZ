classdef CdprCAN < handle
    %与RoBoModule驱动器CAN通信的类(句柄类)
    %在App中通过USBCAN的VCI函数与CAN总线通讯
    
    properties (SetAccess=private, GetAccess=public)
        CONNECTED = 0; % unconnected
        DEVICETYPE = 4; % CANalyst-II
        DEVICEINDEX = 0; % only one USB-CAN device
        CANINDEX = 0; % CAN channel 1
        SENDTYPE = 0; % send type: normal
        FRAMETYPE = 0; % frame type: 
        FRAMEFORMAT = 0; % frame format: 
        RECVFRAMENUM = 0; % number of received frames 
        TIM0 = '00'; % register0 (baud rate setting)
        TIM1 = '14'; % register1 (baud rate setting)
        
        TransmitQueue = {};
    end
    
    methods
        function obj = CdprCAN()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            
        end
        
        function success=openDevice(obj)
            %open usb-can device
            success=VCI_OpenDevice(obj.DEVICETYPE, obj.DEVICEINDEX, obj.CANINDEX);
            if success
                obj.CONNECTED=1;
            end
        end
        
        function success=closeDevice(obj)
            %open usb-can device
            success=VCI_CloseDevice(obj.DEVICETYPE, obj.DEVICEINDEX);
            if success
                obj.CONNECTED=0;
            end
        end
        
        function success=initCAN(obj)
            %initiate can bus
            AccCode = hex2dec("00000000");
            AccMask =hex2dec("FFFFFFFF");
            Reserved = 0;
            Filter=0;
            Timing0 =hex2dec(obj.TIM0);
            Timing1 =hex2dec(obj.TIM1); 
            Mode = 0;
            InitConfig=[AccCode AccMask Reserved Filter Timing0 Timing1 Mode];
            
            success=VCI_InitCAN(obj.DEVICETYPE, obj.DEVICEINDEX, obj.CANINDEX, InitConfig);
        end
        
        function success=startCAN(obj)
            %start can communication
            success=VCI_StartCAN(obj.DEVICETYPE, obj.DEVICEINDEX, obj.CANINDEX);
        end
        
        function success=resetCAN(obj)
            %reset can communication
            success=VCI_ResetCAN(obj.DEVICETYPE, obj.DEVICEINDEX, obj.CANINDEX);
        end
        
        function sendData2Driver(obj, ID, Data)
            %send instructions and data to RoboModule Drivers
            %   ID: cell array of hex string(different row->different frame ID);   
            %   Data:cell array of hex string(different row->different frame Data)
            SendNums=size(Data, 1); %发送帧数
            
            TimeStamp=0;
            TimeFlag=0;
            SendType=obj.SENDTYPE;
            RemoteFlag=0;
            ExternFlag=0;
            DataLen=8;
            
            Frames=[];
            for i=1:SendNums
                ID_Now=hex2dec(ID{i, 1});
                Data1=hex2dec(Data{i,1});
                Data2=hex2dec(Data{i,2});
                Data3=hex2dec(Data{i,3});
                Data4=hex2dec(Data{i,4});
                Data5=hex2dec(Data{i,5});
                Data6=hex2dec(Data{i,6});
                Data7=hex2dec(Data{i,7}); 
                Data8=hex2dec(Data{i,8});
                Data_Now=[Data1 Data2 Data3 Data4 Data5 Data6 Data7 Data8];
                Reserved=[0 0 0];
                Frame=[ID_Now TimeStamp TimeFlag SendType RemoteFlag ExternFlag DataLen Data_Now Reserved];
                Frames=[Frames;Frame];
            end
           
            Succeed = VCI_Transmit(obj.DEVICETYPE, obj.DEVICEINDEX, obj.CANINDEX, Frames,SendNums);%发送成功的帧数
        end
        
        function add2TransQueue(obj, Message)
            %add new message to the end of transmit queue
            %   Message: 1X9 cell array of hex string (ID+Data)
            obj.TransmitQueue(end+1, :)=Message;
        end
        
        function clearTransQueue(obj)
            %clear up all messages in transmit queue
            obj.TransmitQueue={};
        end
        
        function sendTransQueue(obj)
            %send all messages saved in transmit queue in once and clear up
            %the queue
            obj.sendData2Driver(obj.TransmitQueue(: , 1), obj.TransmitQueue(: , 2:end));
            obj.TransmitQueue={};
        end
        
         function resetDriver(obj, DriverID, push2queue_Flag)
            %reset Robomodule Driver, clear up
            %   DriverID: hex string;  
            %   push2queue_Flag: true->push the message to transmit queue(not send the meassage directly)
            %   false->send the meassage directly
            functionID = '0';
            ID = {strcat(DriverID, functionID)};
            Data={{'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}};
            
            if push2queue_Flag
                obj.add2TransQueue([ID, Data]);
            else
                obj.sendData2Driver(ID, Data);
            end
         end
        
         function setDriverMode(obj, DriverID, Mode, push2queue_Flag)
            %reset Robomodule Driver, clear up
            %   DriverID: hex string; Mode: position mode:0   speed&position mode:1
            %   push2queue_Flag: true->push the message to transmit queue(not send the meassage directly)
            %   false->send the meassage directly
            functionID = '1';
            ID = {strcat(DriverID, functionID)};
            switch Mode
                case 0 %position mode
                    Data={{'04'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}};
                case 1 %speed&position mode
                    Data={{'05'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}};
            end
            
            if push2queue_Flag
                obj.add2TransQueue([ID, Data]);
            else
                obj.sendData2Driver(ID, Data);
            end
         end
         
         function setMotorMotion(obj, DriverID, PWM, rawSpeed, rawPosition, Mode, push2queue_Flag)
            %send PWM limit(0~5000), raw speed(unit: rpm) and raw position(unit: qc) to RoboModule Drivers
            %   DriverID: hex string;   PWM:(0~5000)(16bits); raw speed(unit: rpm)(16bits); raw position(unit: qc)(32bits)
            %   push2queue_Flag: true->push the message to transmit queue(not send the meassage directly)
            %   false->send the meassage directly            
             Data={{'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}, {'55'}};
             functionID = '0';
             
             PWM=dec2hex(PWM, 4); %convert to hexstr(16bits)
             Data{1,1} = PWM(1:2); %upper 8 bits of PWM
             Data{1,2} = PWM(3:4); %lower 8 bits of PWM
             
             if rawPosition>=0  
                 position = dec2hex(abs(round(rawPosition)), 8); %Positive number directly convert to hexstr(32bits)
             else
                  position = dec2hex(hex2dec('100000000') - abs(round(rawPosition))); %Negative number take its binary complement(32bits)
             end
             
             switch Mode
                case 0 %position mode
                    functionID = '5';
                    Data{1,3} = '55';
                    Data{1,4} = '55';
                    Data{1,5} = position(1:2);    Data{1,6} = position(3:4);
                    Data{1,7} = position(5:6);    Data{1,8} = position(7:8);
                case 1 %speed&position mode
                    functionID = '6';
                    velocity = dec2hex(round(rawSpeed), 4); %convert to hexstr(16bits)
                    Data{1,3} = velocity(1:2);
                    Data{1,4} = velocity(3:4);
                    Data{1,5} = position(1:2);    Data{1,6} = position(3:4);
                    Data{1,7} = position(5:6);    Data{1,8} = position(7:8);
             end
             ID = {strcat(DriverID, functionID)};
             
             if push2queue_Flag
                obj.add2TransQueue([ID, Data]);
            else
                obj.sendData2Driver(ID, Data);
            end
         end
         
        function success = setBaudRate(obj, BaudRate)
            %set CAN BUS Baud Rate by modify register0 and register1
            %   BaudRate unit: Kbps, type: string
            switch BaudRate
                case '1000'
                    obj.TIM0='00';
                    obj.TIM1='14';
                    success = 1;
                case '800'     
                    obj.TIM0='00';
                    obj.TIM1='16';
                    success = 1;
                case '500'
                    obj.TIM0='00';
                    obj.TIM1='1C';
                    success = 1;
                case '250'
                    obj.TIM0='01';
                    obj.TIM1='1C';
                    success = 1;
                case '125'
                    obj.TIM0='03';
                    obj.TIM1='1C';
                    success = 1;
                case '100'
                    obj.TIM0='04';
                    obj.TIM1='1C';
                    success = 1;
                case '50'
                    obj.TIM0='09';
                    obj.TIM1='1C';
                    success = 1;
                case '20'
                    obj.TIM0='18';
                    obj.TIM1='1C';
                    success = 1;
                case '10'
                    obj.TIM0='31';
                    obj.TIM1='1C';
                    success = 1;
                case '5'
                    obj.TIM0='BF';
                    obj.TIM1='FF';
                    success = 1;
                otherwise
                    success = 0;
            end
        end
        
    end
end

