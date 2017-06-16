function varargout = miniForwardNoToe(varargin)
% MINIFORWARDNOTOE MATLAB code for miniForwardNoToe.fig
%      MINIFORWARDNOTOE, by itself, creates a new MINIFORWARDNOTOE or raises the existing
%      singleton*.
%
%      H = MINIFORWARDNOTOE returns the handle to a new MINIFORWARDNOTOE or the handle to
%      the existing singleton*.
%
%      MINIFORWARDNOTOE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MINIFORWARDNOTOE.M with the given input arguments.
%
%      MINIFORWARDNOTOE('Property','Value',...) creates a new MINIFORWARDNOTOE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before miniForwardNoToe_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to miniForwardNoToe_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help miniForwardNoToe

% Last Modified by GUIDE v2.5 19-Feb-2016 13:39:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @miniForwardNoToe_OpeningFcn, ...
                   'gui_OutputFcn',  @miniForwardNoToe_OutputFcn, ...
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


% --- Executes just before miniForwardNoToe is made visible.
function miniForwardNoToe_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to miniForwardNoToe (see VARARGIN)

%Create slider listener for slider 1
if ~isfield(handles,'hListener')
    handles.hListener = ...
        addlistener(handles.slider1,'ContinuousValueChange',@respondToContSlide1Callback);
        addlistener(handles.slider2,'ContinuousValueChange',@respondToContSlide2Callback);
end

set(handles.slider1,'Value',3/8);
set(handles.slider2,'Value',1/8);

%Create slider listener for slider 2
% if ~isfield(handles,'hListener')
%     handles.hListener = ...
%         addlistener(handles.slider2,'ContinuousValueChange',@respondToContSlide2Callback);
% end

%Compute kinematics

org = [0,0];
rads1 = get(handles.slider1,'Value')*2*pi;
rads2 = get(handles.slider2,'Value')*2*pi;
el1 = [cos(rads1),sin(rads1)];
el2 = [cos(rads2),sin(rads2)];

[endeff] = computeMiniForwardKinematics(rads1,rads2);


%Initial Plot
cla();
axis equal;
xlim([-3,3]);
line([org(1),el1(1)],[org(2),el1(2)],'LineWidth',1.5,'Color','black');
hold on;
line([org(1),el2(1)],[org(2),el2(2)],'LineWidth',1.5,'Color','black');
line([el1(1),endeff(1)],[el1(2),endeff(2)],'LineWidth',1.5,'Color','black');
line([el2(1),endeff(1)],[el2(2),endeff(2)],'LineWidth',1.5,'Color','black');
scatter(endeff(1),endeff(2),'LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue');
hold off;
%Initalize EditTexts
%set(handles.edit1,'string',strcat('(',num2str(endeff(1)),',',num2str(endeff(2)),')'));
%set(handles.linkonerads, 'string', strcat(num2str(rads1),' radians'));
%set(handles.linktworads, 'string', strcat(num2str(rads2),' radians'));

% Choose default command line output for miniForwardNoToe
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes miniForwardNoToe wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function respondToContSlide1Callback(hObject,eventdata)
%establish handles object as belonging to Gui object
handles = guidata(hObject);

%Calculate forward Kinematics
org = [0,0];

%sldval = get(hObject,'Value')*2*pi;
rads1 = get(hObject,'Value')*2*pi;
rads2 = get(handles.slider2,'Value')*2*pi;
el1 = [cos(rads1),sin(rads1)];
el2 = [cos(rads2),sin(rads2)];
[endeff] = computeMiniForwardKinematics(rads1,rads2);

%Establish axes1 as axes to plot to
axes(handles.axes1);
%Plot link 
cla();
axis equal;
xlim([-3,3]);
line([org(1),el1(1)],[org(2),el1(2)],'LineWidth',1.5,'Color','green');
hold on;
line([org(1),el2(1)],[org(2),el2(2)],'LineWidth',1.5,'Color','black');
line([el1(1),endeff(1)],[el1(2),endeff(2)],'LineWidth',1.5,'Color','black');
line([el2(1),endeff(1)],[el2(2),endeff(2)],'LineWidth',1.5,'Color','black');
scatter(endeff(1),endeff(2),'LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue');
ang(org,0.5,[0,rads1],'g');
line([org(1),1],[org(2),0],'LineWidth',1,'Color','green','LineStyle','--');
text(-2.5,2.5,strcat('\theta_1 = ',num2str(rads1,'%.3f')),'Color','green');
text(-2.5,2.2,strcat('\theta_2 = ',num2str(rads2,'%.3f')),'Color',[.5,.5,.5]);
text(.6*cos(rads1/2),.6*sin(rads1/2),'\theta_1');
text(1.5*endeff(1),1.5*endeff(2),strcat('[',num2str(endeff(1),'%.2f'),',',num2str(endeff(2),'%.2f'),']'));
hold off;

%Update Edittexts to input and edittext2
%set(handles.edit1,'string',strcat('(',num2str(endeff(1)),',',num2str(endeff(2)),')'));
%set(handles.linkonerads, 'string', strcat(num2str(rads1),' radians'));
%set(handles.linktworads, 'string', strcat(num2str(rads2),' radians'));

function respondToContSlide2Callback(hObject,eventdata)
%establish handles object as belonging to Gui object
handles = guidata(hObject);

%Calculate forward Kinematics
org = [0,0];


%sldval = get(hObject,'Value')*2*pi;
rads2 = get(hObject,'Value')*2*pi;
rads1 = get(handles.slider1,'Value')*2*pi;
el1 = [cos(rads1),sin(rads1)];
el2 = [cos(rads2),sin(rads2)];
[endeff] = computeMiniForwardKinematics(rads1,rads2);
%Establish axes1 as axes to plot to
axes(handles.axes1);
%Plot link 
cla();
axis equal;
xlim([-3,3]);
line([org(1),el1(1)],[org(2),el1(2)],'LineWidth',1.5,'Color','black');
hold on;
line([org(1),el2(1)],[org(2),el2(2)],'LineWidth',1.5,'Color','blue');
line([el1(1),endeff(1)],[el1(2),endeff(2)],'LineWidth',1.5,'Color','black');
line([el2(1),endeff(1)],[el2(2),endeff(2)],'LineWidth',1.5,'Color','black');
scatter(endeff(1),endeff(2),'LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue');
ang(org,0.5,[0,rads2],'b');
line([org(1),1],[org(2),0],'LineWidth',1,'Color','blue','LineStyle','--');

text(-2.5,2.5,strcat('\theta_1 = ',num2str(rads1,'%.3f')),'Color',[.5,.5,.5]);
text(-2.5,2.2,strcat('\theta_2 = ',num2str(rads2,'%.3f')),'Color','blue');
text(.6*cos(rads2/2),.6*sin(rads2/2),'\theta_2');
text(1.5*endeff(1),1.5*endeff(2),strcat('[',num2str(endeff(1),'%.2f'),',',num2str(endeff(2),'%.2f'),']'));
hold off;

%Update Edittexts to input and edittext2
%set(handles.edit1,'string',strcat('(',num2str(endeff(1)),',',num2str(endeff(2)),')'));
%set(handles.linkonerads, 'string', strcat(num2str(rads1),' radians'));
%set(handles.linktworads, 'string', strcat(num2str(rads2),' radians'));



% --- Outputs from this function are returned to the command line.
function varargout = miniForwardNoToe_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


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


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function linkonerads_Callback(hObject, eventdata, handles)
% hObject    handle to linkonerads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linkonerads as text
%        str2double(get(hObject,'String')) returns contents of linkonerads as a double


% --- Executes during object creation, after setting all properties.
function linkonerads_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linkonerads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function linktworads_Callback(hObject, eventdata, handles)
% hObject    handle to linktworads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linktworads as text
%        str2double(get(hObject,'String')) returns contents of linktworads as a double


% --- Executes during object creation, after setting all properties.
function linktworads_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linktworads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
