% Control UR5 by 3d Connexion mouse:
% http://www.3dconnexion.com/products/spacemouse/spacenavigator-for-notebooks.html
% Mouse3D.m from: Gus Brown @
% http://www.mathworks.com/matlabcentral/fileexchange/18266-3d-mouse-support
% by Fereshteh May 2015

% Note: Matlab should be the active window for 3d mouse data to be received
% by Matlab
clear all
% Parameters:
Scale = 200;% max robot speed in mm/s
alpha = 0.75; % Filter coef.
ToolSpace = 1; %1: tool space, 0:base space

% Connect to robot
Robot_IP = '0.0.0.0';
Socket_conn = tcpip(Robot_IP,30000,'NetworkRole','server');
fclose(Socket_conn);
disp('Press Play on Robot...')
fopen(Socket_conn);
disp('Connected! You can now control the robot with 3d mouse');

% Start 3d mouse
Mouse3D('start');
out = Mouse3D('get');
SpeedT = Scale*out.pos/2500;
SpeedR = Scale*out.ang*out.rot/2500;
Prev_Speed = [SpeedT,SpeedR];

if ToolSpace
    while 1
        out = Mouse3D('get');
        Speed = alpha*[Scale*out.pos/2500,Scale*out.ang*out.rot/2500]+...
            (1-alpha)*Prev_Speed;
        Prev_Speed = Speed;

        % Convert to tool coordinate:
        B = readrobotpose(Socket_conn);
        R_B = vrrotvec2mat([B(4:6)/norm(B(4:6)),norm(B(4:6))]);
        Speed = [(R_B*Speed(1:3)')',(R_B*Speed(4:6)')']
        
        speedrobot(Socket_conn,Speed);
                
    end
else
    while 1
        out = Mouse3D('get');
        Speed = alpha*[Scale*out.pos/2500,Scale*out.ang*out.rot/2500]+...
            (1-alpha)*Prev_Speed;
        Prev_Speed = Speed;
        
        % Calibrate to base coordinate (make it more intuitive):
        Speed = [Speed(3),Speed(1),Speed(2),Speed(6),Speed(4),Speed(5)]
        speedrobot(Socket_conn,Speed);
        
    end
end
