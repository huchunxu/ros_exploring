function varargout = myTeleop(varargin)
% MYTELEOP MATLAB code for myTeleop.fig
%      MYTELEOP, by itself, creates a new MYTELEOP or raises the existing
%      singleton*.
%
%      H = MYTELEOP returns the handle to a new MYTELEOP or the handle to
%      the existing singleton*.
%
%      MYTELEOP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYTELEOP.M with the given input arguments.
%
%      MYTELEOP('Property','Value',...) creates a new MYTELEOP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before myTeleop_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to myTeleop_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help myTeleop

% Last Modified by GUIDE v2.5 20-Jun-2017 15:24:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @myTeleop_OpeningFcn, ...
                   'gui_OutputFcn',  @myTeleop_OutputFcn, ...
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


% --- Executes just before myTeleop is made visible.
function myTeleop_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to myTeleop (see VARARGIN)

% Choose default command line output for myTeleop
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes myTeleop wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = myTeleop_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% 声明一些全局变量
% ROS Master URI和Topic name
global rosMasterUri
global teleopTopicName

rosMasterUri = 'http://192.168.1.202:11311';
teleopTopicName = '/cmd_vel';

% 机器人的运行速度
global leftVelocity
global rightVelocity
global forwardVelocity
global backwardVelocity

leftVelocity = 2;        % 角速度 (rad/s)
rightVelocity = -2;      % 角速度 (rad/s)
forwardVelocity = 2;     % 线速度 (m/s)
backwardVelocity = -2;   % 线速度 (m/s)


% 设置ROS Master URI
function URIEdit_Callback(hObject, eventdata, handles)
% hObject    handle to URIEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of URIEdit as text
%        str2double(get(hObject,'String')) returns contents of URIEdit as a double
global rosMasterUri
rosMasterUri = get(hObject,'String')

% --- Executes during object creation, after setting all properties.
function URIEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to URIEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% 设置Topic name
function TopicEdit_Callback(hObject, eventdata, handles)
% hObject    handle to TopicEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TopicEdit as text
%        str2double(get(hObject,'String')) returns contents of TopicEdit as a double
global teleopTopicName
teleopTopicName = get(hObject,'String')

% --- Executes during object creation, after setting all properties.
function TopicEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TopicEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% 建立连接并初始化ROS publisher
% --- Executes on button press in ConnectButton.
function ConnectButton_Callback(hObject, eventdata, handles)
% hObject    handle to ConnectButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global rosMasterUri
global teleopTopicName
global robot
global velmsg

setenv('ROS_MASTER_URI',rosMasterUri)
rosinit
robot = rospublisher(teleopTopicName,'geometry_msgs/Twist');
velmsg = rosmessage(robot);


% 断开连接，关闭ROS
% --- Executes on button press in DisconnectButton.
function DisconnectButton_Callback(hObject, eventdata, handles)
% hObject    handle to DisconnectButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rosshutdown


% 向前
% --- Executes on button press in ForwardButton.
function ForwardButton_Callback(hObject, eventdata, handles)
% hObject    handle to ForwardButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global velmsg
global robot
global teleopTopicName
global forwardVelocity

velmsg.Angular.Z = 0;
velmsg.Linear.X = forwardVelocity;
send(robot,velmsg);
latchpub = rospublisher(teleopTopicName, 'IsLatching', true);


%向左
% --- Executes on button press in LeftButton.
function LeftButton_Callback(hObject, eventdata, handles)
% hObject    handle to LeftButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global velmsg
global robot
global teleopTopicName
global leftVelocity

velmsg.Angular.Z = leftVelocity;
velmsg.Linear.X = 0;
send(robot,velmsg);
latchpub = rospublisher(teleopTopicName, 'IsLatching', true);


% 向右
% --- Executes on button press in RightButton.
function RightButton_Callback(hObject, eventdata, handles)
% hObject    handle to RightButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global velmsg
global robot
global teleopTopicName
global rightVelocity

velmsg.Angular.Z = rightVelocity;
velmsg.Linear.X = 0;
send(robot,velmsg);
latchpub = rospublisher(teleopTopicName, 'IsLatching', true);


% 向后
% --- Executes on button press in BackwardButton.
function BackwardButton_Callback(hObject, eventdata, handles)
% hObject    handle to BackwardButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global velmsg
global robot
global teleopTopicName
global backwardVelocity

velmsg.Angular.Z = 0;
velmsg.Linear.X = backwardVelocity;
send(robot,velmsg);
latchpub = rospublisher(teleopTopicName, 'IsLatching', true);
