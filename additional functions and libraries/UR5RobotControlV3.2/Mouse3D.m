function out = Mouse3D(varargin)
% function Mouse3D
%
% Activates 3D mouse navigation on all 3D figures.
% Allows the use of a 3Dconnexion 'magellan' 3Dmouse in 
% Matlab figure windows. Uses a com interface to the 
% 3Dmouse service (TDxInput.Device), and uses a timer 
% object (Mouse3dtimer) to poll the 3Dmouse.
%
% Use, Mouse3D to start and stop the service.
%
% Alternate syntax
% out = Mouse3D(cmd)
% where cmd can be ;
%   'start' : start 3D mouse navigation, returns com handle
%   'stop'  : stop 3D mouse navigation, returns 0
%   'get'   : returns 3dmouse position and rotation structure
%
% Example:
%    logo; 
%    Mouse3D;
%
% Gus - 2008

persistent srv busycntr 

period = 0.01;  % Timer frequency, should match frame rate on your graphics card
slowdown = 25;  % slowdown factor for inactive mouse
timertag = 'Mouse3dtimer';  % Unique tag for timer object

% check inputs
if nargin==0, 
  cmd = 0; 
else
  % convert string inputs to numeric eqiv
  if ischar(varargin{1}),
    switch lower(varargin{1}),
    case('view');
      cmd = 1;
    case('busy')
      cmd = 2;
    case('rotate')
      cmd = 3;
    case('start'),
      cmd = 4;
    case('stop'),   
      cmd = 5;
    case('gettimer');
      cmd = 6;
    case('get'),   
      cmd = 7;
    otherwise
      cmd = 0;
    end;
  else
    cmd = varargin{1};
  end;
end;

%% #Activities#
tim = timerfind('tag',timertag);
switch lower(cmd),
  case(1), % #view#
    % starts timer and returns a handle to the timer
    disp('start')
    start(tim);
    out = 1;
  case(2), % #busy#
    % counts each time the timer calls and the function is still busy
    busycntr = busycntr + 1;
    out = busycntr;
    if  busycntr<10, 
      % restarts the timer as it stops by default
      start(tim); 
    else
      % causes the timer to stop if too busy 
      warning('Mouse3D:stop','Mouse3D stopped');
      mouse3D_stop(srv,tim);
    end;
  case(3), % #rotate#
    % polls the mouse and calls the relevant camera operations
    busycntr = 0;
    dt = 10*(1+busycntr)*period;
    out = mouse3D_rotate(srv,dt,period,slowdown,tim);
  case(4), % #start#
    % connect to com object and setup timer object
    disp('start');
    srv = mouse3D_start(period,timertag);
    busycntr = 0;
    out = srv;
  case(5), % #stop#
    % stop and cleanup
    srv = mouse3D_stop(srv,tim);
    out = 0;
  case(6), % #gettimer#
    % return handle to timer
    out = timerfind('tag',timertag);
  case(7), % #get#
    % get position from the mouse
    out = mouse3D_get(srv);

  otherwise
    if isempty(srv),
      [srv,tim] = mouse3D_start(period,timertag);
      out = srv;
      disp('Mouse3D started!');
      start(tim);
    elseif strcmpi(get(timerfind('tag',timertag),'Running'),'off'), 
      out = mouse3D_view;
    else
      srv = mouse3D_stop(srv,tim);
      out = 0;
      disp('Mouse3D stopped!');
    end;
end;
    


%% ---------------------------------------------------------
% polls the mouse and callls the relavent camera operations
function out = mouse3D_rotate(srv,dt,period,slowdown,tim)
  m3d = mouse3D_get(srv);
  % checks for mouse activity and slows down the time if there is none
  % needed to prevent lockup
  if (m3d.len==0 && m3d.ang==0),
    if (tim.Period == period),
      stop(tim);  % must stop timer before changing period
      tim.Period = period*slowdown;
      start(tim);
    end;
  else
    % speedup timer if it was resting
    if (tim.Period ~= period),
      stop(tim);
      tim.Period = period;
      start(tim);
    end;
    %pos = sign(m3d.pos).*(m3d.pos/1600).^2;
    %rot = sign(m3d.rot).*(m3d.rot*m3d.ang/1600).^2;
    pos = (m3d.pos/1600);
    rot = (m3d.rot*m3d.ang/1600);
    camorbit(30*dt*rot(3),30*dt*rot(1),'camera'); 
    VA = camva;
    campan(-VA*dt*pos(1),VA*dt*pos(3),'camera');  
    camroll(30*dt*rot(2)); 
    VA = max(1e-6,VA+5*dt*pos(2)); 
    camva(VA); 
    drawnow; 
  end;
  out = 0;


%% ---------------------------------------------------------
% get position from the mouse
function out = mouse3D_get(srv)
  out.period = srv.get('Sensor').get('Period');

  opos = srv.get('Sensor').get('Translation');
  out.pos = [get(opos,'X') get(opos,'Y') get(opos,'Z')];
  out.len = get(opos,'Length');

  orot = srv.get('Sensor').get('Rotation');
  out.rot = [get(orot,'X') get(orot,'Y') get(orot,'Z')];
  out.ang = get(orot,'Angle');


%% ---------------------------------------------------------
% connect to com object and setup timer object
function [srv tim] = mouse3D_start(period,timertag)
  srv = actxserver('TDxInput.Device');
  tim = timerfind('tag',timertag);
  if isempty(tim),
    tim = timer;
    % set(tim, 'TimerFcn','Mouse3D(3);', 'ErrorFcn', 'Mouse3D(2);', 'BusyMode','error', 'ExecutionMode','FixedRate', 'StartDelay',period, 'period',period, 'tag',timertag); 
    set(tim, 'TimerFcn','Mouse3D(3);', 'BusyMode','drop', 'ExecutionMode','FixedRate', 'StartDelay',period, 'period',period, 'tag',timertag); 
  else
    stop(tim);
  end;
  % srv.invoke('Connect');  % This line does not seem to be needed


%% ---------------------------------------------------------
% Stop timer and release com object
function srv = mouse3D_stop(srv,tim)
  % srv.invoke('Disconnect'); % This line does not seem to be needed
  if ~isempty(tim),
    stop(tim);
    delete(tim);
  end;
  release(srv);
  srv = [];
