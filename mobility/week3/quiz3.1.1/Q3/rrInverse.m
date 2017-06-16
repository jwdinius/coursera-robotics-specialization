function varargout = rrInverse(varargin)
% RRINVERSE MATLAB code for rrInverse.fig
%      RRINVERSE, by itself, creates a new RRINVERSE or raises the existing
%      singleton*.
%
%      H = RRINVERSE returns the handle to a new RRINVERSE or the handle to
%      the existing singleton*.
%
%      RRINVERSE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RRINVERSE.M with the given input arguments.
%
%      RRINVERSE('Property','Value',...) creates a new RRINVERSE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before rrInverse_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to rrInverse_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help rrInverse

% Last Modified by GUIDE v2.5 18-Feb-2016 18:30:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @rrInverse_OpeningFcn, ...
                   'gui_OutputFcn',  @rrInverse_OutputFcn, ...
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


% --- Executes just before rrInverse is made visible.
function rrInverse_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to rrInverse (see VARARGIN)

%Create slider listener for slider 1

if ~isfield(handles,'hListener')
    handles.hListener = ...
        addlistener(handles.slider1,'ContinuousValueChange',@respondToContSlide1Callback);
        addlistener(handles.slider2,'ContinuousValueChange',@respondToContSlide2Callback);
end

set(handles.slider1,'Value',2/3);
set(handles.slider2,'Value',5/6);
%Create slider listener for slider 2
% if ~isfield(handles,'hListener')
%     handles.hListener = ...
%         addlistener(handles.slider2,'ContinuousValueChange',@respondToContSlide2Callback);
% end

%Compute kinematics
org = [0,0];
X = (get(handles.slider1,'Value')-.5)*4;
Y = (get(handles.slider2,'Value')-.5)*4;
[rads1,rads2] = computeRrInverseKinematics(X,Y);

%Initial Plot
cla();
axis equal;
xlim([-3,3]);
if isreal(rads1)
    elbow = [cos(rads1),sin(rads1)];
    endeff = [elbow(1)+cos(rads2+rads1),elbow(2)+sin(rads2+rads1)];
    line([org(1),elbow(1)],[org(2),elbow(2)],'LineWidth',1.5,'Color','black');
    hold on;
    line([elbow(1),endeff(1)],[elbow(2),endeff(2)],'LineWidth',1.5,'Color','black');
    scatter(endeff(1),endeff(2),'LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue');
    hold off;
end

%Initalize EditTexts
%if valid
    %set(handles.firstlinkrads,'string',strcat(num2str(rads1),' radians'));
    %set(handles.secondlinkrads, 'string', strcat(num2str(rads2),' radians'));
%else
    %set(handles.firstlinkrads,'string','Position Not Feasible!');
    %set(handles.secondlinkrads,'string','Position Not Feasible!');
%end
%set(handles.inputPosition, 'string',strcat('(',num2str(X),',',num2str(Y),')'));

% Choose default command line output for rrInverse
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes rrInverse wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function respondToContSlide1Callback(hObject,eventdata)
%establish handles object as belonging to Gui object
handles = guidata(hObject);

%Calculate Inverse Kinematics
org = [0,0];

%sldval = get(hObject,'Value')*2*pi;
X = (get(hObject,'Value')-.5)*4;
Y = (get(handles.slider2,'Value')-.5)*4;
[rads1,rads2] = computeRrInverseKinematics(X,Y);

%Establish axes1 as axes to plot to
axes(handles.axes1);
%Plot link 
cla();
axis equal;
xlim([-3,3]);
if isreal(rads1)
    elbow = [cos(rads1),sin(rads1)];
    endeff = [elbow(1)+cos(rads2+rads1),elbow(2)+sin(rads2+rads1)];
    line([org(1),elbow(1)],[org(2),elbow(2)],'LineWidth',1.5,'Color','black');
    hold on;
    line([elbow(1),endeff(1)],[elbow(2),endeff(2)],'LineWidth',1.5,'Color','black');
    scatter(endeff(1),endeff(2),'LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue');
    ang(org,0.5,[0,rads1],'g');
    line([org(1),1],[org(2),0],'LineWidth',1,'Color','green','LineStyle','--');
    ang(elbow,0.5,[rads1,rads2+rads1],'b');
    line([elbow(1),elbow(1)+cos(rads1)],[elbow(2),elbow(2)+sin(rads1)],'LineWidth',1,'Color','blue','LineStyle','--');
    
    
    text(-2.5,2.5,strcat('\theta_1 = ',num2str(rads1,'%.3f')),'Color','green');
    text(-2.5,2.2,strcat('\theta_2 = ',num2str(rads2,'%.3f')),'Color','blue');
 

    text(.6*cos(rads1/2),.6*sin(rads1/2),'\theta_1');
    text(.6*cos(rads1+rads2/2)+elbow(1),.6*sin(rads1+rads2/2)+elbow(2),'\theta_2');
    text(1.2*endeff(1),1.2*endeff(2),strcat('[',num2str(endeff(1),'%.2f'),',',num2str(endeff(2),'%.2f'),']'));
    hold off;
end

%Update EditTexts
%if valid
    %set(handles.firstlinkrads,'string',strcat(num2str(rads1),' radians'));
    %set(handles.secondlinkrads, 'string', strcat(num2str(rads2),' radians'));
%else
    %set(handles.firstlinkrads,'string','Position Not Feasible!');
    %set(handles.secondlinkrads,'string','Position Not Feasible!');
%end
%set(handles.inputPosition, 'string',strcat('(',num2str(X),',',num2str(Y),')'));

function respondToContSlide2Callback(hObject,eventdata)
%establish handles object as belonging to Gui object
handles = guidata(hObject);

%Calculate Inverse Kinmeatics
org = [0,0];

%sldval = get(hObject,'Value')*2*pi;
Y = (get(hObject,'Value')-.5)*4;
X = (get(handles.slider1,'Value')-.5)*4;
[rads1,rads2] = computeRrInverseKinematics(X,Y);

%Establish axes1 as axes to plot to
axes(handles.axes1);
%Plot link 
cla();
axis equal;
xlim([-3,3]);
if isreal(rads1)
    elbow = [cos(rads1),sin(rads1)];
    endeff = [elbow(1)+cos(rads2+rads1),elbow(2)+sin(rads2+rads1)];
    line([org(1),elbow(1)],[org(2),elbow(2)],'LineWidth',1.5,'Color','black');
    hold on;
    line([elbow(1),endeff(1)],[elbow(2),endeff(2)],'LineWidth',1.5,'Color','black');
    scatter(endeff(1),endeff(2),'LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue');
    ang(org,0.5,[0,rads1],'g');
    line([org(1),1],[org(2),0],'LineWidth',1,'Color','green','LineStyle','--');
    ang(elbow,0.5,[rads1,rads2+rads1],'b');
    line([elbow(1),elbow(1)+cos(rads1)],[elbow(2),elbow(2)+sin(rads1)],'LineWidth',1,'Color','blue','LineStyle','--');
    
    
    text(-2.5,2.5,strcat('\theta_1 = ',num2str(rads1,'%.3f')),'Color','green');
    text(-2.5,2.2,strcat('\theta_2 = ',num2str(rads2,'%.3f')),'Color','blue');
    text(.6*cos(rads1/2),.6*sin(rads1/2),'\theta_1');
    text(.6*cos(rads1+rads2/2)+elbow(1),.6*sin(rads1+rads2/2)+elbow(2),'\theta_2');
    text(1.2*endeff(1),1.2*endeff(2),strcat('[',num2str(endeff(1),'%.2f'),',',num2str(endeff(2),'%.2f'),']'));
    hold off;
end

%Update EditTexts
%if valid
    %set(handles.firstlinkrads,'string',strcat(num2str(rads1),' radians'));
    %set(handles.secondlinkrads, 'string', strcat(num2str(rads2),' radians'));
%else
    %set(handles.firstlinkrads,'string','Position Not Feasible!');
    %set(handles.secondlinkrads,'string','Position Not Feasible!');
%end
%set(handles.inputPosition, 'string',strcat('(',num2str(X),',',num2str(Y),')'));



% --- Outputs from this function are returned to the command line.
function varargout = rrInverse_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function firstlinkrads_Callback(hObject, eventdata, handles)
% hObject    handle to firstlinkrads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of firstlinkrads as text
%        str2double(get(hObject,'String')) returns contents of firstlinkrads as a double


% --- Executes during object creation, after setting all properties.
function firstlinkrads_CreateFcn(hObject, eventdata, handles)
% hObject    handle to firstlinkrads (see GCBO)
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
set(hObject,'Value',.5);

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
set(hObject,'Value',.5);


function secondlinkrads_Callback(hObject, eventdata, handles)
% hObject    handle to secondlinkrads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of secondlinkrads as text
%        str2double(get(hObject,'String')) returns contents of secondlinkrads as a double


% --- Executes during object creation, after setting all properties.
function secondlinkrads_CreateFcn(hObject, eventdata, handles)
% hObject    handle to secondlinkrads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function inputPosition_Callback(hObject, eventdata, handles)
% hObject    handle to inputPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of inputPosition as text
%        str2double(get(hObject,'String')) returns contents of inputPosition as a double


% --- Executes during object creation, after setting all properties.
function inputPosition_CreateFcn(hObject, eventdata, handles)
% hObject    handle to inputPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
