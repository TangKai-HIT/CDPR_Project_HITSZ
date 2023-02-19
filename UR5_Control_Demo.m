% UR5 matlab control demo
% using URscript command and real-time interface (port 30003)
clear; clc;
%% Connect to robot
Robot_IP = "192.168.1.10";
port = 30003;
PC_Client = tcpip(Robot_IP, port, 'NetworkRole', 'client');
PC_Client.InputBufferSize = 1200; %total number of data: 1116
fclose(PC_Client);
pause(0.1);
fopen(PC_Client);
%% read robot state data data from UR5
MessageLength=fread(PC_Client, 1, 'int');
message = fread(PC_Client, 139, 'double');
JointPosition = message(32:37); %Actual joint positions(rad)
Tool_Pose = message(56:61); %Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
%% move arm
velocity = 0.1; %m/s
acceleration = 1.0; %m/(s^2)
sendScript = sprintf("movel(p[%f, %f, %f, %f, %f ,%f], a=%f, v=%f, t=0, r=0)\n", ...
                                [Tool_Pose(1)+0.1, Tool_Pose(2)+0.1, Tool_Pose(3), Tool_Pose(4:6)', acceleration, velocity]);
fprintf(PC_Client, sendScript);
%% terminate connection
% fclose(PC_Client);
% delete(PC_Client);