function varargout = imugui(varargin)
% IMUGUI MATLAB code for imugui.fig
%      IMUGUI, by itself, creates a new IMUGUI or raises the existing
%      singleton*.
%
%      H = IMUGUI returns the handle to a new IMUGUI or the handle to
%      the existing singleton*.
%
%      IMUGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IMUGUI.M with the given input arguments.
%
%      IMUGUI('Property','Value',...) creates a new IMUGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before imugui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to imugui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help imugui

% Last Modified by GUIDE v2.5 17-May-2016 10:32:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @imugui_OpeningFcn, ...
                   'gui_OutputFcn',  @imugui_OutputFcn, ...
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


% --- Executes just before imugui is made visible.
function imugui_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to imugui (see VARARGIN)

addpath(genpath('MyoMex'));

% Constants:
handles.const.STRIP_TIME = 10; 
handles.const.UPDATE_RATE = 15; % Hz | Default: 25
handles.const.UPDATE_START_DELAY = 1; % s 
handles.const.STREAMING_DATA_TIME = 0.020; % [s] (50 Hz) 
handles.const.STREAMING_FRAME_TIME = 0.040; % [s] (25 Hz)
handles.const.QUAT_STRIP_YLIM = [-1,1]; 
handles.const.GYRO_STRIP_YLIM = 2000*[-1,1]; 
handles.const.ACCEL_STRIP_YLIM = 2*[-1,1]; 
handles.const.EMG_STRIP_YLIM = [-1,1.2];

handles.const.POSE_YDATA_VALUE = 1.1;
handles.const.NUM_POSES = 5;

handles.const.ROTATE_OFFSET = -135; %-45

handles.const.countMyos = 1;
handles.const.LINK1_SIZE = 1.0;
handles.const.LINK2_SIZE = 1.0;

% Load pictures
handles.image_pose_enabled = true;
handles.image_pose_fist             = imread('image_pose_fist.png');
handles.image_pose_wave_in          = imread('image_pose_wave_in.png');
handles.image_pose_wave_out         = imread('image_pose_wave_out.png');
handles.image_pose_fingers_spread   = imread('image_pose_fingers_spread.png');
handles.image_pose_double_tap       = imread('image_pose_double_tap.png');
handles.image_pose_rest             = imread('image_pose_rest.png');

% process inputs 
handles.myoData = []; 
handles.closeFunc = [];

handles.data.imu1 = zeros(handles.const.STRIP_TIME * 50,11 * 1);
handles.data.imu2 = zeros(handles.const.STRIP_TIME * 50,11 * 1);
handles.data.R1 = eye(3);
handles.data.R2 = eye(3);
handles.data.P0 = zeros(3,1);
handles.data.P1 = zeros(3,1);
handles.data.P2 = zeros(3,1);

handles.cal.R1 = eye(3);
handles.cal.q1 = quaternion([1;0;0;0]); %w-x-y-z
handles.cal.R2 = eye(3);

handles.status.isConnected = false;
handles.status.isStreaming = false;

handles.update_timer = timer(...
  'name','GUI_update_timer',...
  'busymode','drop',...
  'executionmode','fixedrate',...
  'period',round(1/handles.const.UPDATE_RATE,3),...
  'startdelay',handles.const.UPDATE_START_DELAY,...
  'startfcn',{@clearLogsFcn,handles.figure1},...
  'timerfcn',{@updateFigureCallback,handles.figure1});

set(handles.axGraphics,...
    'nextplot','add',...
    'xlim',[-2,2],'ylim',[-2,2],'zlim',[-2,2],...
    'xtick',[],'ytick',[],'ztick',[],...
    'dataaspectratio',[1,1,1],...
    'color',hObject.Color);
view(handles.axGraphics,...
  -handles.const.ROTATE_OFFSET,30);

hp.hx_global= hgtransform('parent',handles.axGraphics);
hp.hx_cal1= hgtransform('parent',handles.axGraphics);

hp.hx_link1 = hgtransform('parent',hp.hx_global);

hp.hx_axis1 = hgtransform('parent',hp.hx_link1);

hp.hx_link2 = hgtransform('parent',hp.hx_axis1);

hp.hx_axis2 = hgtransform('parent',hp.hx_link2);

% Coordinate Axes: Global
plot3(handles.axGraphics,...
  [0,0.5],[0,0],[0,0],'r','linewidth',1.5,'parent',hp.hx_global);
plot3(handles.axGraphics,...
  [0,0],[0,0.5],[0,0],'g','linewidth',1.5,'parent',hp.hx_global);
plot3(handles.axGraphics,...
  [0,0],[0,0],[0,0.5],'b','linewidth',1.5,'parent',hp.hx_global);

% Coordinate Axes: Calibration 1
plot3(handles.axGraphics,...
  [0,0.5],[0,0],[0,0],'r','linewidth',0.5,'parent',hp.hx_cal1);
plot3(handles.axGraphics,...
  [0,0],[0,0.5],[0,0],'g','linewidth',0.5,'parent',hp.hx_cal1);
plot3(handles.axGraphics,...
  [0,0],[0,0],[0,0.5],'b','linewidth',0.5,'parent',hp.hx_cal1);

% Graphics: Link 1
[xL1,yL1,zL1] = cylinder(0.06,4);
zL1 = zL1 * handles.const.LINK1_SIZE;

hLink1 = surf(xL1,yL1,zL1,'parent',hp.hx_link1,...
    'facecolor','r',...
    'edgecolor','k');
rotate(hLink1,[0,1,0],90);

% Coordinate Axes: Link 1
set(hp.hx_axis1,'Matrix',[eye(3), [handles.const.LINK1_SIZE;0;0];0,0,0,1]);

plot3(handles.axGraphics,...
  [0,0+0.2],[0,0],[0,0],...
  'r','linewidth',1,'parent',hp.hx_axis1);
plot3(handles.axGraphics,...
  [0,0],[0,0+0.2],[0,0],...
  'g','linewidth',1,'parent',hp.hx_axis1);
plot3(handles.axGraphics,...
  [0,0],[0,0],[0,0+0.2],...
  'b','linewidth',1,'parent',hp.hx_axis1);

% Graphics: Link 2
[xL2,yL2,zL2] = cylinder(0.05,4);
zL2 = zL2 * handles.const.LINK2_SIZE;

hLink2 = surf(xL2,yL2,zL2,'parent',hp.hx_link2,...
    'facecolor',[0.8,0,0],...
    'edgecolor','k');
rotate(hLink2,[0,1,0],90);

% Coordinate Axis: Link 2
set(hp.hx_axis2,'matrix',[eye(3), [handles.const.LINK2_SIZE;0;0];0,0,0,1]);

plot3(handles.axGraphics,...
  [0,0+0.2],[0,0],[0,0],...
  'r','linewidth',1,'parent',hp.hx_axis2);
plot3(handles.axGraphics,...
  [0,0],[0,0+0.2],[0,0],...
  'g','linewidth',1,'parent',hp.hx_axis2);
plot3(handles.axGraphics,...
  [0,0],[0,0],[0,0+0.2],...
  'b','linewidth',1,'parent',hp.hx_axis2);

% Pose Images
im = handles.image_pose_rest;
[M,N,~] = size(im);
[X,Y] = meshgrid(1:M,1:N);
A = sqrt( (X-floor(M/2)).^2 + (Y-floor(N/2)).^2 )<(floor(min([M,N])/2)-5);

hp.image_pose = imshow(ones(225,225,3)*0.94,'parent',handles.axPose);
set(hp.image_pose,'cdata',im,...
    'parent',handles.axPose,...
    'alphadata',A,'visible','on');
set(handles.axPose,...
    'xtick',[],'ytick',[],...
    'box','on',...
    'color',hObject.Color);

handles.hp = hp;
assignin('base','hp',hp);
drawnow

% Choose default command line output for imugui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes imugui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = imugui_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function clearLogsFcn(~,~,fig)
handles = guidata(fig);
handles.m1.clearLogs();

function updateFigureCallback(hObj,~,fig)

handles = guidata(fig);
hp = handles.hp;

m1 = handles.m1;
% m2 = handles.m2;

data1 = handles.data.imu1;
% data2 = handles.data.imu2;

newData1  = [m1.timeIMU_log m1.accel_log m1.gyro_log m1.quat_log];

%newData1 = [t=1------------a=2:4--------g=5:7-------q=8:11-----]; 
m1.clearLogs();

%newData2  = [m2.timeIMU_log m2.accel_log m2.gyro_log m2.quat_log];
%m2.clearLogs();

if size(newData1,2)~=11
    return; end
% if size(newData2,2)~=11
%     return; end

nSamples1 = size(newData1,1);

%nSamples2 = size(newData2,1);

newData1 = newData1(end:-1:1,:);
%newData2 = newData2(end:-1:1,:);

data1 = data1(1:end-nSamples1,:);
data1 = [newData1; data1];

     
R1c = handles.cal.R1;
R2c = handles.cal.R2;
%set(hp.hx_cal1,'matrix',[R1c, [0;0;0];0,0,0,1]);

q1 = quaternion(data1(1,8:11));
q1c = handles.cal.q1;
q1 = product(q1,q1c);

R1 = RotationMatrix(q1);
R1 = L2R(R1);
% Calibration:
% R1 = R1c' * R1;

%R2 = m1.q2r(data2(1,8:11))*R2c';
R2 = eye(3);

%%%%%%%%%%%%%%%%%%%%%%%%
% Rotation simulation:
% R1 = rotz(hObj.TasksExecuted * 90/ 30 *0); %hObj.TasksExecuted
% R2 = R1c' * m1.q2r(data(1,8:11)); %R1'*R1c'*m1.q2r(data(1,8:11));
%%%%%%%%%%%%%%%%%%%%%%%%


H1 = [R1, [0;0;0];...
     0,0,0,1];
set(hp.hx_link1,'matrix',H1);

H2 = [R2, [0;0;0];...
     0,0,0,1];
set(hp.hx_link2,'matrix',H2);

% Position:
H2 = hp.hx_global.Matrix * hp.hx_link1.Matrix * hp.hx_axis1.Matrix *...
    hp.hx_link2.Matrix * hp.hx_axis2.Matrix;
P2 = H2(1:3,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pose image
if handles.image_pose_enabled
    im = [];
    switch true
        case m1.pose_fist
            im = handles.image_pose_fist;
        case m1.pose_wave_in
            im = handles.image_pose_wave_in;
        case m1.pose_wave_out
            im = handles.image_pose_wave_out;
        case m1.pose_fingers_spread
            im = handles.image_pose_fingers_spread;
        case m1.pose_double_tap
            im = handles.image_pose_double_tap;
        case m1.pose_unknown
            im = handles.image_pose_rest;
    end
    % update image_pose
      if ~isempty(im)
        [M,N,~] = size(im);
        [X,Y] = meshgrid(1:M,1:N);
        A = sqrt( (X-floor(M/2)).^2 + (Y-floor(N/2)).^2 )<(floor(min([M,N])/2)-5);
        set(hp.image_pose,'cdata',im,'alphadata',A,'visible','on');
      else
        set(hp.image_pose,'visible','off');
      end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

angles = R2YPR(R1)*180/3.14;
%newData = [t=1------------a=2:4--------g=5:7-------q=8:11-----]; 
handles.txtStatus.String = sprintf('Pos: X:% 1.2f, Y:% 1.2f, Z:% 1.2f',P2(1),P2(2),P2(3));
handles.txtSensors.String = sprintf('Sen: Ax:% 1.2f, Ay:% 1.2f, Az:% 1.2f',data1(1,2),data1(1,3),data1(1,4));
handles.txtEuler.String = sprintf('Eul: Y:% 1.1f, P:% 1.1f, R:% 1.1f',angles(1),angles(2),angles(3));

handles.data.imu1 = data1;
%handles.data.imu2 = data2;

handles.hp = hp;

drawnow;
guidata(fig,handles);
assignin('base','data',handles.data.imu1);

function pbReset_Callback(hObject, ~, handles)
% Calibration Matrix
R = q2R(handles.m1.quat);
handles.cal.R1 = R;
q1 = quaternion(handles.m1.quat);
handles.cal.q1 = inverse(q1);

guidata(hObject, handles);

% --- Executes on button press in tbConnect.
function tbConnect_Callback(hObject, ~, handles)

countMyos = handles.const.countMyos;

if handles.status.isConnected == false
    disp('Trying to connect...');
    try
        mm = MyoMex(countMyos);
        m1 = mm.myoData(countMyos);
        handles.mm = mm;
        handles.m1 = m1;
        handles.status.isConnected = true;
        disp('Successful!');
    catch err
        disp('Unsuccessful.');
        throw(err);
    end
else
    disp('Already connected.');
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in rbStreamY.
function rbStreamY_Callback(hObject, eventdata, handles)
% hObject    handle to rbMyo1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbMyo1

disp('Starting stream...');

if handles.status.isConnected == false
    disp('Device not connected.');
    hObject.Value = 0; handles.rbStreamN.Value = 1;
else
    handles.m1.startStreaming();
    pause(0.5);
    if handles.m1.isStreaming == 1;
        disp('Streaming.');
        handles.status.isStreaming = true;
    end
end

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in rbStreamN.
function rbStreamN_Callback(hObject, eventdata, handles)
% hObject    handle to rbStreamN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbStreamN

disp('Stopping stream...');

if handles.status.isConnected == false
    disp('Device not connected.');
    handles.rbStreamY.Value = 0; handles.rbStreamN.Value = 1;
elseif handles.status.isStreaming == true
    handles.m1.stopStreaming();
    pause(0.5);
    if handles.m1.isStreaming == 0;
        disp('Stopped.');
    end
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in sqPlot.
function sqPlot_Callback(hObject, eventdata, handles)
% hObject    handle to sqPlot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in sqStream.
function sqStream_Callback(hObject, eventdata, handles)
% hObject    handle to sqStream (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% % --- Executes on button press in pbConnect.
% function pbConnect_Callback(hObject, eventdata, handles)
% % hObject    handle to pbConnect (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hint: get(hObject,'Value') returns toggle state of pbConnect


% --- Executes on button press in rbMyo1.
function rbMyo1_Callback(hObject, eventdata, handles)
% hObject    handle to rbMyo1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbMyo1


% --- Executes on button press in rbMyo2.
function rbMyo2_Callback(hObject, eventdata, handles)
% hObject    handle to rbMyo2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbMyo2


% --- Executes on button press in sqOnMyo.
function sqOnMyo_Callback(hObject, eventdata, handles)
% hObject    handle to sqOnMyo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in sqLog.
function sqLog_Callback(hObject, eventdata, handles)
% hObject    handle to sqLog (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on button press in sqConnect.
function sqConnect_Callback(hObject, eventdata, handles)
% hObject    handle to sqConnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
stop(handles.update_timer);
try
    handles.m1.stopStreaming();
    handles.mm.delete;
% catch err
%     disp(err);
end


delete(hObject);
disp('GUI terminated.');

% --- Executes on button press in rbLogY.
function rbLogY_Callback(hObject, eventdata, handles)
% q1 = quaternion(handles.m1.quat);
% handles.cal.q1 = inverse(q1);
start(handles.update_timer);


% --- Executes on button press in rbLogN.
function rbLogN_Callback(hObject, eventdata, handles)
stop(handles.update_timer);

function R = rotx(angle)
angle = angle * pi / 180;
R = [1,0,0;...
     0,cos(angle),-cos(angle);...
     0,sin(angle),cos(angle)];
 
function R = roty(angle)

angle = angle * pi / 180;
R = [cos(angle),0,sin(angle);...
     0          1,0;...
     -sin(angle),0,cos(angle)];
 
function R = rotz(angle)
angle = angle * pi / 180;
R = [cos(angle),-sin(angle),0;...
     sin(angle),cos(angle) ,0;...
     0,       0,            1];

function R = q2R( Qrotation )
% q2R: get a 3x3 rotation matrix
% R = q2R( Qrotation )
% IN: 
%     Qrotation - quaternion describing rotation
% 
% OUT:
%     R - rotation matrix 
%     
% VERSION: 03.03.2012

w = Qrotation( 1 );
x = Qrotation( 2 );
y = Qrotation( 3 );
z = Qrotation( 4 );

Rxx = 1 - 2*(y^2 + z^2);
Rxy = 2*(x*y - z*w);
Rxz = 2*(x*z + y*w);

Ryx = 2*(x*y + z*w);
Ryy = 1 - 2*(x^2 + z^2);
Ryz = 2*(y*z - x*w );

Rzx = 2*(x*z - y*w );
Rzy = 2*(y*z + x*w );
Rzz = 1 - 2 *(x^2 + y^2);

R = [ 
    Rxx,    Rxy,    Rxz;
    Ryx,    Ryy,    Ryz;
    Rzx,    Rzy,    Rzz];


function R = L2R(R)

R(1,3) = - R(1,3);
R(2,3) = - R(2,3);
R(3,1) = - R(3,1);
R(3,2) = - R(3,2);

function angles = R2YPR(R)
% this conversion uses conventions as described on page:
% http://www.euclideanspace.com/maths/geometry/rotations/euler/index.htm
% Coordinate System: right hand
% Positive angle: right hand
% Order of euler angles: heading first, then attitude, then bank
% matrix row column ordering:
% [m00 m01 m02] % [m11 m12 m13]
% [m10 m11 m12] % [m21 m22 m23]
% [m20 m21 m22] % [m31 m32 m33]

if R(2,1)>0.998 % singularity north pole
    y = atan2(R(1,3),R(3,3));
    p = pi/2;
    r = 0;
elseif R(2,1)<-0.998 % singularity south pole
    y = atan2(R(1,3),R(3,3));
    p = -pi/2;
    r = 0;
else
    y = atan2(-R(3,1),R(1,1));
    p = atan2(-R(2,3),R(2,2));
    r = asin(R(2,1));
end
angles = [y p r];


function angles = q2angle(q)
q = q/norm(q);
qx = q(1);
qy = q(2);
qz = q(3);
qw = q(4);

% switch round(qx*qy+qz*qw,3) %check for singularities
%     case 0.50 % north pole
%         y = 2*atan2(qx,qw);
%         p = pi/2;
%         r = 0;
%     case -0.50 % south pole
%         y = -2*atan2(qx,qw);
%         p = pi/2;
%         r = 0;
%     otherwise %others
%         y = atan2(2*qy*qw-2*qx*qz, 1-2*qy^2-2^qz^2);
%         p = asin(2*qx*qy+2*qz*qw);
%         r = atan2(2*qx*qw-2*qy*qz, 1-2*qx^2-2^qz^2);
% end
% angles = [y p r];

r = atan2(2*qw*qx+2*qy*qz, 1-2*qx^2-2^qy^2);
p = asin(max(-1,min(1,2*qw*qy-2*qz*qx)));
y = atan2(2*qw*qz+2*qx*qy, 1-2*qy^2-2^qz^2);
angles = [y p r];


