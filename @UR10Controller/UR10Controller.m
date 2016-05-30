classdef UR10Controller < handle
%UR10Controller Adds commands for steering the end effector of a UR10
%robot based on a remote TCP/IP connection.
%
% Usage example:
%  h = UR10Controller('192.168.1.2',30002),
%  initialize(h)
%  speedl(h)
%  speedj(h)
%  terminate(h)
%  delete(h)
%
% TODO:
%  right now, all commands are sent blindly. We need controller feedback
%% Public Properties
properties
    inc = zeros(6,1)    % Speed vector cartesian space
    q = zeros(6,1)      % Speed vector, joint space (rad/s)
    a = 0.5             % tool acceleration (m/s^2)
    a_r = 0.1           % joint acceleration (rad/s^2)
    t_min = 3.0         % minimal time before function return
    t_max = 1.0
end

properties (SetAccess = private)
    isConnected = false     % Robot Connected, aka TCP/IP object connected
    autoMode = false        % If automatic mode
    tcpObj                  % TCP/IP Object
    timerObj                % Timer Object for sending commands
    timerStop               % Timer to send stop command
    currentCommand          % String representing a command to be sent
end

%% Hidden Properties
properties (Access = private) % Default 
   defaultHost = '192.168.1.2'
   defaultPort = 30002
   refreshRate = 1/125  % seconds. Default is 0.008 (1/125).
   timestampBegin
end
properties (Dependent)
    speedLimitCart
    speedLimitJoint
    t_factorCart
    t_factorJoint
    timerStopDelay
    timestamp
end
%% Events
events
end %events

methods
%% CONSTRUCTORS
function hObj = UR10Controller( varargin )
    switch nargin

% default settings:
        case 0 % nargin == 0
            hObj.tcpObj = tcpip(hObj.defaultHost,hObj.defaultPort);

% given 'tcpip'  object:
        case 1 % nargin == 1
            if isa( varargin{1}, 'tcpip')
                hObj.tcpObj = varargin{1};
            else
                error( 'UR10Controller:constructor:nargin1', ...
                           'Invalid input. Singleton must be a TCP/IP object.' );
            end

% given address and port
        case 2
            if or( ~ischar(varargin{1} ), ~isnumeric( varargin{2} )  )
                error( 'UR10Controller:constructor:nargin2', ...
                           'Invalid inputs. Indicate first the remote host as a string, then the remote port and a numeric value.' );
            else
                hObj.tcpObj = tcpip(varargin{1},varargin{2});
            end
% other cases
        otherwise
                error( 'UR10Controller:constructor:nargin3', ...
                           'Too many inputs.' );
                return

    end % switch:nargin

% define timer
    hObj.timerStop = timer(...
        'Name','Stop-Timer',...
        'StartDelay',1,...
        'TimerFcn',@(src,evt)hObj.timerStopFcn(hObj,evt),...
        'UserData',hObj);
    
% setting up the tcp/ip object for receiving comms
    set(hObj.tcpObj,...
        'BytesAvailableFcn','fprintf(''%s'',fgetl(h.tcpObj));',...
        'InputBufferSize',1024);
    
    
% print result
    fprintf('Remote TCP/IP server configured @%s:%i\n',hObj.tcpObj.RemoteHost,hObj.tcpObj.RemotePort);

end % controller (constructor)

%% PROPERTY ACCESS METHODS
function set.inc( obj, value )
    
    if obj.autoMode
        error( 'UR10Controller:setproperty:increment:mode',...
                    'You cannot set the increment during automatic mode.');
    elseif ~isnumeric(value)
        error( 'UR10Controller:setproperty:increment:numeric',...
                    'Value must be numeric.');
    else
        sz = size(value);
        if ( min(sz)~=1 || max(sz)~=6 )
            error( 'UR10Controller:setproperty:increment:size',...
                        'Increment must be set to a vector of length 6.');
        else
            obj.inc = value;
            if norm(obj.inc(1:3)) > obj.speedLimitCart
                r = obj.speedLimitCart / norm(obj.inc(1:3));
                obj.inc(1:3) = r * obj.inc(1:3);
                fprintf(['The specified speed cannot be attained under these conditions.\n'...
                         'Speed Rescale factor: %.2f\n'], r);
            end
            speedl(obj);
        end
    end
end %set.inc
function set.tcpObj( obj, value )
    if ~isa(value,'tcpip')
        error( 'UR10Controller:setproperty:tcpip',...
                    'This property must be a object of class %s.','tcpip');
    else
        obj.tcpObj = value;
    end
end
function value = get.t_factorCart(obj)
    value = 1 - norm(obj.inc(1:3)) / obj.a / obj.t_min;
end
function value = get.t_factorJoint(obj)
    value = 1 - sum(obj.inc(4:6).^2) / obj.a_r / obj.t_min;
end
function value = get.timerStopDelay(obj)
    switch true
        case strcmp(obj.currentCommand(1:6),'speedl')
            value = obj.t_factorCart * obj.t_min;
        case strcmp(obj.currentCommand(1:6),'speedj')
            value = obj.t_factorJoint * obj.t_min;
        otherwise
            value = 0.95;
    end%switch
end
function value = get.speedLimitCart(obj)
    value = 0.5 * obj.a * obj.t_min;
    value = floor(value * 1000) / 1000;
end
function value = get.timestamp(obj)
    value = datestr(toc(obj.timestampBegin)/3600/24, 'HH:MM:SS.FFF');
end
%% BASIC METHODS
function initialize ( obj )
    if obj.isConnected
        disp('Already initialized.');
        return
    end
    fopen(obj.tcpObj);

    % check connection
    % send robot home
    % wait
    disp('Connection initialized.');
    obj.isConnected = true;
    obj.timestampBegin = tic;
end 

function terminate ( obj )
    if ~obj.isConnected
        disp('Already terminated.');
        return
    end
    % send stop command
    % close connection
    fclose(obj.tcpObj);
    obj.isConnected = false;
    disp('I''m no longer active.');
end

% function startAuto ( obj )
%     % Starts timer that sends commands to the robot
%     if ( strcmp(obj.timerObj.Running,'on') || obj.autoMode == true )
%         disp('I believe the...');
%     end
% end

% Delete method
function delete (obj)
    if obj.isConnected
        terminate(obj); end
    delete(obj.timerStop);
%     if strcmp(obj.timerObj.Running,'on')
%         stop(obj.timerObj); end
end

%% CONTROL METHODS: Rotation
function rotDir ( obj, value )
% function rotDir(value)
% Inverts rotation direction based on 'value'.
% 'value' can be any numeric value. The sign of 'value'
% determines the rotation direction according to the
% right-hand rule.

    if ( value > 0 && obj.inc < 0 )
        obj.inc(4) = - obj.inc(4);
    elseif ( value < 0 && obj.inc > 0 )
        obj.inc(4) = - obj.inc(4);
    else
        %Do nothing
    end
end

function rotAdd ( obj )
% function rotAdd
% Sets the rotation term to 1.

    if obj.inc(4) == 0
        obj.inc(4) = 1;
    end
end

function rotSpeed ( obj , value )
% function rotSpeed
% Sets the rotation speed.

    obj.inc(4) = value;
end

function rotStop (obj)
% function rotStop
% Annuls rotation speed.
    obj.rotSpeed(0);
end

%% CONTROL METHODS: Translation
function transDir ( obj )
% function transDir
% Inverts translation direction.
obj.inc(1:3) = obj.inc(1:3) * rotz(pi);
end

function transAdd ( obj, value )
% function transAdd
% Sets increment to the translation direction.
value = value / norm(value);
obj.inc(1:3) = value;

end

function transSpeed ( obj , value )
% function transSpeed(value)
% Sets the translation speed to 'value'.

obj.inc(1:3) = obj.inc(1:3) / norm( obj.inc(1:3) ) * value;
end

function transStop (obj)
    obj.transSpeed(0);
end

% Stop all movements
function stop (obj)
    obj.inc = zeros(6,1);
    % TODO: possibly have to had an abort command
end

% TODO:
% Set home
% Send home
% Set vertical translation direction
% Add vertical translation
% Set vertical translation speed
% Remove vertical translation

%% COMMAND METHODS
% TODO:
% movel
% movej
% speedl
% speedj
% stopj

function sendCommand( obj )
    if ~obj.isConnected
        error( 'UR10Controller:methods:sendCommand:notconnected',...
                    'Not connected to the host.');
        return %#ok<UNRCH>
    end
    fprintf('%s: %s',obj.timestamp,obj.currentCommand);
    fprintf(obj.tcpObj,obj.currentCommand);
    
end

function speedl(obj)
% Function speedl - Accelerate to specified tool speed
% Parameters:
% Input  xd:    tool speed (m/s) (spatial vector)
%         a:    tool acceleration (m/s^2)
%     t_min:    minimal time before function return
% Output  s:    string to be sent to the UR10
    
    in = [obj.inc; obj.a; obj.t_min];
    in = cellstr(num2str(in,'%.2f'));
    in = strtrim(in);
    
    s = sprintf(['speedl([%s,%s,%s,%s,%s,%s],' ...
                 '%s,'...
                 '%s)\n'],in{1:8});
    obj.currentCommand = s;
    startTimer(obj);
    sendCommand(obj);
end
function speedj(obj)
% Function speedj - Accelerate joints to target speed.
    s = sprintf('speedj([%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],%.2f,%.2f)\n',obj.q,obj.a_r,obj.t_min);
    obj.currentCommand = s;
    start(obj.timerStop);
    sendCommand(obj);
end
function stopl(obj)
    a_max = obj.a;
    obj.currentCommand = sprintf('stopl(%.1f)\n',a_max);
    sendCommand(obj);
end

end %methods
%% PRIVATE METHODS
methods ( Access = private )
    function timerStopFcn(src,~,~,~)
        stopl(src);
    end
    function startTimer(obj)
        t = obj.timerStop;
        set(t, 'StartDelay', floor(obj.timerStopDelay*1000)/1000);
        start(t);
    end
end

%% STATIC METHODS
methods ( Static = true )
function R = rotz(angle)
    R = [cos(angle),-sin(angle),0;...
         sin(angle),cos(angle) ,0;...
         0,         0,          1];
end

end %methods:static


end %classdef