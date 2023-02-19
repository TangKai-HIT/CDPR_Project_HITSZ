% Speed 1x6 vector in mm/s 
function P_new = speedrobot(t,Speed)
if nargin == 1
    error('error; not enough input arguments')
elseif nargin == 2
    P = Speed;
elseif nargin == 3
    error('too many inputs')
end
P = P * 0.001; % Converting to meter
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
    pause(0.02)
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