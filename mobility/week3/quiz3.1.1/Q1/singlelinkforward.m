function varargout = singlelinkforward(varargin)
% SINGLELINKFORWARD MATLAB code for singlelinkforward.fig
%      SINGLELINKFORWARD, by itself, creates a new SINGLELINKFORWARD or raises the existing
%      singleton*.
%
%      H = SINGLELINKFORWARD returns the handle to a new SINGLELINKFORWARD or the handle to
%      the existing singleton*.
%
%      SINGLELINKFORWARD('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SINGLELINKFORWARD.M with the given input arguments.
%
%      SINGLELINKFORWARD('Property','Value',...) creates a new SINGLELINKFORWARD or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before singlelinkforward_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to singlelinkforward_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help singlelinkforward

% Last Modified by GUIDE v2.5 23-Feb-2016 13:52:49


% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @singlelinkforward_OpeningFcn, ...
                   'gui_OutputFcn',  @singlelinkforward_OutputFcn, ...
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


% --- Executes just before singlelinkforward is made visible.
function singlelinkforward_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no edittext2 args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to singlelinkforward (see VARARGIN)

%Create slider listener to respond to slider motion in real-time
if ~isfield(handles,'hListener')
    handles.hListener = ...
        addlistener(handles.slider1,'ContinuousValueChange',@respondToContSlideCallback);
end

%Plot initial position of link
org = [0,0];
rads = get(handles.slider1,'Value')*2*pi;
%x = cos(sldval);
%y = sin(sldval);
%endeff = [x,y];
endeff = computeForwardKinematics(rads);

cla();
axis equal;
xlim([-2,2]);
line([org(1),endeff(1)],[org(2),endeff(2)],'LineWidth',1.5,'Color','black');
hold on;
scatter(endeff(1),endeff(2),'LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue');
hold off;

%Initialize Edittexts to input and edittext2
%set(handles.edittext2,'string',strcat('(',num2str(endeff(1),'%+.2f'),',',num2str(endeff(2),'%+.2f'),')'));
%set(handles.input, 'string', strcat('Input Angle:  ',num2str(rads,' %.3f'),' rads'));

% Choose default command line output for singlelinkforward
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
% UIWAIT makes singlelinkforward wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function respondToContSlideCallback(hObject,eventdata)
%establish handles object as belonging to Gui object
handles = guidata(hObject);

%Calculate forward Kinematics
org = [0,0];
%sldval = get(hObject,'Value')*2*pi;
rads = get(hObject,'Value')*2*pi;
endeff = computeForwardKinematics(rads);

%Establish axes1 as axes to plot to
axes(handles.axes1);
%Plot link 
cla();
axis equal;
xlim([-2,2]);
line([org(1),endeff(1)],[org(2),endeff(2)],'LineWidth',1.5,'Color','black');
hold on;
ang(org,0.5,[0,rads],'g');
line([org(1),1],[org(2),0],'LineWidth',1,'Color','green','LineStyle','--');
scatter(endeff(1),endeff(2),'LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue');
text(.6*cos(rads/2),.6*sin(rads/2),strcat('\theta = ',num2str(rads,'%.3f')));
text(1.2*endeff(1),1.2*endeff(2),strcat('[',num2str(endeff(1),'%.2f'),',',num2str(endeff(2),'%.2f'),']'));
hold off;

%Update Edittexts to input and edittext2
%set(handles.edittext2,'string',strcat('Output Coordinates: ','(',num2str(endeff(1),'%+.2f'),',',num2str(endeff(2),'%+.2f'),')'));
%set(handles.input, 'string', strcat('Input Angle:  ',num2str(rads,'% .3f'),' rads'));


% --- Outputs from this function are returned to the command line.
function varargout = singlelinkforward_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning edittext2 args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line edittext2 from handles structure
%varargout{1} = handles.edittext2;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function input_Callback(hObject, eventdata, handles)
% hObject    handle to input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of input as text
%        str2double(get(hObject,'String')) returns contents of input as a double


% --- Executes during object creation, after setting all properties.
function input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edittext2_Callback(hObject, eventdata, handles)
% hObject    handle to edittext2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edittext2 as text
%        str2double(get(hObject,'String')) returns contents of edittext2 as a double


% --- Executes during object creation, after setting all properties.
function edittext2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edittext2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
