% Goal_Pose should be in mm and Orientation the rotation vector
function P_new = moverobot(t,Goal_Pose,Orientation)
if nargin == 1
    error('error; not enough input arguments')
elseif nargin == 2
    P = Goal_Pose;
elseif nargin == 3
    P = [Goal_Pose,Orientation];
end
P(1:3) = P(1:3) * 0.001; % Converting to meter
P_char = ['(',num2str(P(1)),',',...
    num2str(P(2)),',',...
    num2str(P(3)),',',...
    num2str(P(4)),',',...
    num2str(P(5)),',',...
    num2str(P(6)),...
    ')'];
success = '0';
while strcmp(success,'0')
    fprintf(t,'(1)'); % task = 1 : moving task
    pause(0.01);% Tune this to meet your system
    fprintf(t,P_char);
    while t.BytesAvailable==0
        %t.BytesAvailable
    end
    success  = readrobotMsg(t);
end
if ~strcmp(success,'1')
    error('error sending robot pose')
end
%pause(0.5)
P_new = readrobotpose(t);