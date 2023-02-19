% Code template for UR5 by Fereshteh Feb. 2015
% This demo reads current robot pose, then translates and rotates
% endeffector

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear all

% Connect to robot
Robot_IP = '192.168.1.10';
Socket_conn = tcpip(Robot_IP,30000,'NetworkRole','server');
fclose(Socket_conn);
disp('Press Play on Robot...')
fopen(Socket_conn);
disp('Connected!');

%% Read robot pose
Robot_Pose = readrobotpose(Socket_conn)
Translation = Robot_Pose(1:3); % in mm
Orientation = Robot_Pose(4:6);

%% %%%%%%%%%%% Example 1: translation
% Translate robot
Translation = Translation - 20; % 20 mm in all axes
moverobot(Socket_conn,Translation,Orientation);

%% %%%%%%%%%%%% Example 2: Rotation
% Converting axis-angle to rotation matrix
orientation_mat = vrrotvec2mat([Orientation,norm(Orientation)]);
% 20 degrees rotation about endeffector z axis
z_axis = [1,0,0];
Rot_mag = 20*pi/180;
Rot_z = vrrotvec2mat([z_axis,Rot_mag]);
Goal_orient = orientation_mat *Rot_z;
% Convert back to axis-angel (Rotation Vector representation)
Goal_v = vrrotmat2vec(Goal_orient(1:3,1:3));
Goal_ori = Goal_v(4)*Goal_v(1:3);
% Rotate:
moverobot(Socket_conn,Translation,Goal_ori);

%% Coop mode
ActivateCoopMode(Socket_conn);
pause(2);
DeActivateCoopMode(Socket_conn);