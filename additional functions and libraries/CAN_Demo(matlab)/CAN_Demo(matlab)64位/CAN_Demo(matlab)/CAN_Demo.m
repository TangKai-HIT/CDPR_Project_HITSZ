function varargout = CAN_Demo(varargin)
% Edit the above text to modify the response to help CAN_Demo

% Last Modified by GUIDE v2.5 10-Dec-2014 09:58:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CAN_Demo_OpeningFcn, ...
                   'gui_OutputFcn',  @CAN_Demo_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end


if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before CAN_Demo is made visible.
function CAN_Demo_OpeningFcn(hObject, eventdata, handles, varargin)
global CONNECTED;
global LISTBOXNUM;
LISTBOXNUM = 0;
CONNECTED = 0;%--开始时为没连接状态
set(handles.edit_Timing0,'Enable','off');
set(handles.edit_Timing1,'Enable','off');
if libisloaded('ControlCAN')~=1
end

%初始化
global DEVICETYPE;
set(handles.popupmenu_DeviceName,'value',3);
DEVICETYPE=4;

global DEVICEINDEX;
set(handles.popupmenu_DeviceIndex,'value',1);
DEVICEINDEX=0;

global CANINDEX;
set(handles.popupmenu_CANIndex,'value',1);
CANINDEX=0;

global SENDTYPE;
SENDTYPE=0;

global FRAMETYPE;
FRAMETYPE=0;

global FRAMEFORMAT;
FRAMEFORMAT=0;

% Choose default command line output for CAN_Demo
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);



% UIWAIT makes CAN_Demo wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CAN_Demo_OutputFcn(hObject, eventdata, handles) 
% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu_DeviceName.
function popupmenu_DeviceName_Callback(hObject, eventdata, handles)
global DEVICETYPE;
dev=get(hObject,'value');
switch dev
    case 1
        DEVICETYPE=2;
    case 2     
        DEVICETYPE=3;
    case 3
        DEVICETYPE=4;
    case 4
        DEVICETYPE=5;
    case 5
        DEVICETYPE=12;
    case 6
        DEVICETYPE=14;
    case 7
        DEVICETYPE=15;
    case 8
        DEVICETYPE=16;
    case 9
        DEVICETYPE=17;
    case 10
        DEVICETYPE=24;
    otherwise
        disp('Unknown DeviceType.');
end


% --- Executes on selection change in popupmenu_DeviceIndex.
function popupmenu_DeviceIndex_Callback(hObject, eventdata, handles)
global DEVICEINDEX;
DEVICEINDEX=get(handles.popupmenu_DeviceIndex,'value')-1;




% --- Executes on selection change in popupmenu_CANIndex.
function popupmenu_CANIndex_Callback(hObject, eventdata, handles)
global CANINDEX;
CANINDEX=get(handles.popupmenu_CANIndex,'value')-1;



% --- Executes on button press in pushbutton_Connect.
function pushbutton_Connect_Callback(hObject, eventdata, handles)
global DEVICETYPE;
global DEVICEINDEX;
global CANINDEX;
global CONNECTED;
if CONNECTED~=1
    if VCI_OpenDevice(DEVICETYPE,DEVICEINDEX,CANINDEX)    
        CONNECTED=1;
        set(hObject,'String','断开');
        AccCode = hex2dec(get(handles.edit_ACCCode,'String'));
        AccMask =hex2dec(get(handles.edit_ACCMask,'String'));
        Reserved = 0;
        Filter=get(handles.popupmenu_FilterMode,'value')-1;
        Timing0 =hex2dec(get(handles.edit_Timing0,'String'));
        Timing1 =hex2dec(get(handles.edit_Timing1,'String'));
        Mode =get(handles.popupmenu_Mode,'value')-1;
        InitConfig=[AccCode AccMask Reserved Filter Timing0 Timing1 Mode];
        if  VCI_InitCAN(DEVICETYPE,DEVICEINDEX,CANINDEX,InitConfig)~=1                
            msgbox('Init CAN失败！');
        end
    else
        msgbox('OPen CAN失败！');
    end
else
    if VCI_CloseDevice(DEVICETYPE,DEVICEINDEX)~= 1
        msgbox('关闭CAN失败！');
    else
        CONNECTED=0;
        set(hObject,'String','连接');
    end
end



function timerCallback(obj, event,listbox_Information,edit_Timeout)
global CONNECTED;
global DEVICETYPE;
global DEVICEINDEX;
global CANINDEX;
global LISTBOXNUM;
ID=0;
TimeStamp=0;
TimeFlag=0;
SendType=0;
RemoteFlag=0;
ExternFlag=0;
DataLen=8;
if CONNECTED~=1
    stop(obj);
    delete(obj);
    clear obj;
else
    len = VCI_GetReceiveNum(DEVICETYPE,DEVICEINDEX,CANINDEX);
    if len
         [GetLength,Frames]=VCI_Receive(DEVICETYPE,DEVICEINDEX,CANINDEX,50,400);
         if GetLength
             for i=1:GetLength
                 ID=Frames(i,1);
                 strID=strcat('ID：',sprintf('%08x',ID));
                 TimeStamp=Frames(i,2);
                 strTimeStamp=strcat('       时间标识：',num2str(TimeStamp));
    
                 strDir='       接收';
        
                 RemoteFlag = Frames(i,4);
                 if RemoteFlag
                     strRemoteFlag = '      远程帧';
                 else
                     strRemoteFlag = '      数据帧';
                 end

                 ExternFlag = Frames(i,5);
                 if ExternFlag
                     strExternFlag = '      扩展帧';
                 else
                     strExternFlag = '      标准帧';
                 end
                 
                 DataLen = Frames(i,7);
                 strDataLen = strcat('       长度：',num2str(DataLen));
                 strData='';
                 data1=num2str(dec2hex(Frames(i,8)));
                 data2=num2str(dec2hex(Frames(i,9)));
                 data3=num2str(dec2hex(Frames(i,10)));
                 data4=num2str(dec2hex(Frames(i,11)));
                 data5=num2str(dec2hex(Frames(i,12)));
                 data6=num2str(dec2hex(Frames(i,13)));
                 data7=num2str(dec2hex(Frames(i,14)));
                 data8=num2str(dec2hex(Frames(i,15)));
                 
                 strData=strcat(data1,' -',data2,' -',data3,' -',data4,' -',data5,' -',data6,' -',data7,' -',data8);
                 strr=get(listbox_Information,'String');

                 strr{i+LISTBOXNUM}=strcat(strID,strTimeStamp,strDir,strRemoteFlag,strExternFlag,strDataLen,'           Data：',strData);
                 set(listbox_Information,'String',strr);
             end
              LISTBOXNUM = LISTBOXNUM + GetLength;%listbox里的帧数加1
         end
    else
        
    end
end

% --- Executes on button press in pushbutton_StartCAN.
function pushbutton_StartCAN_Callback(hObject, eventdata, handles)
global DEVICETYPE;
global DEVICEINDEX;
global CANINDEX;
if VCI_StartCAN(DEVICETYPE,DEVICEINDEX,CANINDEX)~= 1
    msgbox('启动CAN失败！');
else
    t = timer('TimerFcn',{@timerCallback,handles.listbox_Information}, 'ExecutionMode', 'fixedDelay', 'Period', 0.5);
    start(t);
end


% --- Executes on button press in pushbutton_ResetCAN.
function pushbutton_ResetCAN_Callback(hObject, eventdata, handles)
global DEVICETYPE;
global DEVICEINDEX;
global CANINDEX;
if VCI_ResetCAN(DEVICETYPE,DEVICEINDEX,CANINDEX)~= 1
    msgbox('复位CAN失败！');
end


% --- Executes on button press in popupmenu_Baud.
function popupmenu_Baud_Callback(hObject, eventdata, handles)
global TIM0;
global TIM1;
select=get(hObject,'value');
switch select
    case 1
        TIM0='00';
        TIM1='14';
        set(handles.edit_Timing0,'String','00');
        set(handles.edit_Timing1,'String','14');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 2     
        TIM0='00';
        TIM1='16';
        set(handles.edit_Timing0,'String','00');
        set(handles.edit_Timing1,'String','16');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 3
        TIM0='00';
        TIM1='1C';
        set(handles.edit_Timing0,'String','00');
        set(handles.edit_Timing1,'String','1C');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 4
        TIM0='01';
        TIM1='1C';
         set(handles.edit_Timing0,'String','01');
        set(handles.edit_Timing1,'String','1C');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 5
        TIM0='03';
        TIM1='1C';
        set(handles.edit_Timing0,'String','03');
        set(handles.edit_Timing1,'String','1C');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 6
        TIM0='04';
        TIM1='1C';
        set(handles.edit_Timing0,'String','04');
        set(handles.edit_Timing1,'String','1C');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 7
        TIM0='09';
        TIM1='1C';
        set(handles.edit_Timing0,'String','09');
        set(handles.edit_Timing1,'String','1C');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 8
        TIM0='18';
        TIM1='1C';
        set(handles.edit_Timing0,'String','18');
        set(handles.edit_Timing1,'String','1C');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 9
        TIM0='31';
        TIM1='1C';
        set(handles.edit_Timing0,'String','31');
        set(handles.edit_Timing1,'String','1C');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 10
        TIM0='BF';
        TIM1='FF';
        set(handles.edit_Timing0,'String','BF');
        set(handles.edit_Timing1,'String','FF');
        set(handles.edit_Timing0,'Enable','off');
        set(handles.edit_Timing1,'Enable','off');
    case 11
        set(handles.edit_Timing0,'Enable','on');
        set(handles.edit_Timing1,'Enable','on');
    otherwise
        disp('Unknown BAUD.');
end




function popupmenu_FilterMode_Callback(hObject, eventdata, handles)
global FILTERMODE;
select=get(hObject,'value');
switch select
    case 1
        FILTERMODE=0;
    case 2
        FILTERMODE=1;
end
        


function popupmenu_Mode_Callback(hObject, eventdata, handles)
global MODE;
select=get(hObject,'value');
switch select
    case 1
        FILTERMODE=0;
    case 2
        FILTERMODE=1;
end




% --- Executes on selection change in popupmenu_SendType.
function popupmenu_SendType_Callback(hObject, eventdata, handles)
global SENDTYPE;
select=get(hObject,'value');
switch select
    case 1
        SENDTYPE=0;
    case 2
        SENDTYPE=1;
    case 3
        SENDTYPE=2;
    case 4
        SENDTYPE=3;
end



% --- Executes on selection change in popupmenu_FrameType.
function popupmenu_FrameType_Callback(hObject, eventdata, handles)
global FRAMETYPE;
select=get(hObject,'value');
switch select
    case 1
       FRAMETYPE=0;
    case 2
       FRAMETYPE=1;
end




% --- Executes on selection change in popupmenu_FrameFormat.
function popupmenu_FrameFormat_Callback(hObject, eventdata, handles)
global FRAMEFORMAT;
select=get(hObject,'value');
switch select
    case 1
       FRAMEFORMAT=0;
    case 2
       FRAMEFORMAT=1;
end



% --- Executes on button press in pushbutton_Send.
function pushbutton_Send_Callback(hObject, eventdata, handles)
global DEVICETYPE;
global DEVICEINDEX;
global CANINDEX;
global LISTBOXNUM;
ID=hex2dec(get(handles.edit_FrameID,'String'));
TimeStamp=0;
TimeFlag=0;
SendType=get(handles.popupmenu_SendType,'value')-1;
RemoteFlag=get(handles.popupmenu_FrameFormat,'value')-1;
ExternFlag=get(handles.popupmenu_FrameType,'value')-1;
DataLen=8;
SendNums=str2double(get(handles.edit_SendNums,'String'));%发送帧数
Data1=hex2dec(get(handles.edit_Data1,'String'));
Data2=hex2dec(get(handles.edit_Data2,'String'));
Data3=hex2dec(get(handles.edit_Data3,'String'));
Data4=hex2dec(get(handles.edit_Data4,'String'));
Data5=hex2dec(get(handles.edit_Data5,'String'));
Data6=hex2dec(get(handles.edit_Data6,'String'));
Data7=hex2dec(get(handles.edit_Data7,'String'));
Data8=hex2dec(get(handles.edit_Data8,'String'));
Data=[Data1 Data2 Data3 Data4 Data5 Data6 Data7 Data8];
Reserved=[0 0 0];
Frame=[ID TimeStamp TimeFlag SendType RemoteFlag ExternFlag DataLen Data Reserved];
Frames=Frame;
for i=1:SendNums-1
    Frames=[Frames;Frame];
end
Succeed = VCI_Transmit(DEVICETYPE,DEVICEINDEX,CANINDEX, Frames,SendNums);%发送成功的帧数
if Succeed
    for i=1:Succeed
        ID=Frames(i,1);
        strID=strcat('ID：',sprintf('%08x',ID));

        TimeStamp=Frames(i,2);
        strTimeStamp=strcat('       时间标识：',sprintf('%08x',TimeStamp));

        strDir='       发送';

        RemoteFlag = Frames(i,4);
        if RemoteFlag
            strRemoteFlag = '      远程帧';
        else
            strRemoteFlag = '      数据帧';
        end 

        ExternFlag = Frames(i,5);
        if ExternFlag
            strExternFlag = '      扩展帧';
        else
            strExternFlag = '      标准帧';
        end

        DataLen = Frames(i,7);
        strDataLen = strcat('       长度：',num2str(DataLen));
        strData='';
        data1=num2str(dec2hex(Frames(i,8)));
        data2=num2str(dec2hex(Frames(i,9)));
        data3=num2str(dec2hex(Frames(i,10)));
        data4=num2str(dec2hex(Frames(i,11)));
        data5=num2str(dec2hex(Frames(i,12)));
        data6=num2str(dec2hex(Frames(i,13)));
        data7=num2str(dec2hex(Frames(i,14)));
        data8=num2str(dec2hex(Frames(i,15)));
        strData=strcat(data1,' -',data2,' -',data3,' -',data4,' -',data5,' -',data6,' -',data7,' -',data8);
        strr=get(handles.listbox_Information,'String');
        strr{i+LISTBOXNUM}=strcat(strID,strTimeStamp,strDir,strRemoteFlag,strExternFlag,strDataLen,'           Data：',strData);
        set(handles.listbox_Information,'String',strr);
    end
    LISTBOXNUM = LISTBOXNUM + Succeed;%listbox里的帧数加接收成功的帧
else

    msgbox('发送数据失败');
end


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
global DEVICETYPE;
global DEVICEINDEX;
global CONNECTED;
if CONNECTED
    if VCI_CloseDevice(DEVICETYPE,DEVICEINDEX)~= 1
    msgbox('关闭CAN失败！');
    end
end
delete(hObject);



% --- Executes on button press in pushbutton_Clear.
function pushbutton_Clear_Callback(hObject, eventdata, handles)
global DEVICETYPE;
global DEVICEINDEX;
global CANINDEX;
global LISTBOXNUM;
VCI_ClearBuffer(DEVICETYPE, DEVICEINDEX, CANINDEX);
LISTBOXNUM = 0;
strr='';
set(handles.listbox_Information,'String',strr);


% --- Executes on button press in pushbutton_DeviceInfo.
function pushbutton_DeviceInfo_Callback(hObject, eventdata, handles)
global DEVICETYPE;
global DEVICEINDEX;
[result,info] = VCI_ReadBoardInfo(DEVICETYPE,DEVICEINDEX);
hw_Version = info{1};
fw_Version = info{2};
dr_Version = info{3};
in_Version = info{4};
irq_Num    = info{5};
can_Num    = info{6};
str_Serial_Num = info{7};
str_hw_Type= info{8};
Reserved   = info{9};
str0=sprintf('硬件版本号:%x\n固件版本号:%x\n驱动程序版本号:%x\n接口库版本号:%x\n板卡所使用的中断号:%x\nCAN通道:%x\n',hw_Version,fw_Version,dr_Version,in_Version,irq_Num,can_Num);
str=strcat(str0,'     序列号:',str_Serial_Num,'     硬件类型:',str_hw_Type,'     系统保留',Reserved);
msgbox(str);

    


% --- Executes on button press in pushbutton_Status.
%function pushbutton_Status_Callback(hObject, eventdata, handles)
%global DEVICETYPE;
%global DEVICEINDEX;
%global CANINDEX;
%[result,info] = VCI_ReadCanStatus(DEVICETYPE,DEVICEINDEX,CANINDEX)
%ErrInterrupt = info(1,1)
%regMode      = info(2);
%regStatus    = info(3);
%regALCapture = info(4);
%regECCapture = info(5);
%regEWLimit   = info(6);
%regRECounter = info(7);
%regTECounter = info(8);
%Reserved     = info(9)



