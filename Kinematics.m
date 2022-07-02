function varargout = Kinematics(varargin)
% KINEMATICS MATLAB code for Kinematics.fig
%      KINEMATICS, by itself, creates a new KINEMATICS or raises the existing
%      singleton*.
%
%      H = KINEMATICS returns the handle to a new KINEMATICS or the handle to
%      the existing singleton*.
%
%      KINEMATICS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KINEMATICS.M with the given input arguments.
%
%      KINEMATICS('Property','Value',...) creates a new KINEMATICS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Kinematics_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Kinematics_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Kinematics

% Last Modified by GUIDE v2.5 02-Jul-2022 14:51:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Kinematics_OpeningFcn, ...
                   'gui_OutputFcn',  @Kinematics_OutputFcn, ...
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


% --- Executes just before Kinematics is made visible.
function Kinematics_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Kinematics (see VARARGIN)

% Choose default command line output for Kinematics
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
clear all
clc

% axes(handles.axes1)%AXES1 İN ÖZELLİKLERİNE ERİŞ
axis([-3,3,-3,3,0,3])
xlabel('X EKSENİ');
ylabel('Y EKSENİ');
zlabel('Z EKSENİ');
title('MAKİNE');

global h1 l2 d2 q1 q2 d3 q1s q2s d3s
h1=1;
l2=1;
d2=1;
q1=0;
q2=0;
d3=0;

%MAKİNEMİZİN SON KONUMUNU TUTAN DEGİŞKENLER
q1s=0;
q2s=0;
d3s=0;

t01=[cosd(q1) -sind(q1) 0 0;sind(q1) cosd(q1) 0 0;0 0 1 h1;0 0 0 1];
t12=[cosd(q2) -sind(q2) 0 0;0 0 -1 -d2;sind(q2) cosd(q2) 0 0;0 0 0 1];
t23=[1 0 0 0;0 0 1 (l2+d3);0 -1 0 0;0 0 0 1];

T01=t01;
T02=t01*t12;
T03=t01*t12*t23;

x=[0 T01(1,4)];
y=[0 T01(2,4)];
z=[0 T01(3,4)];

x01_02=[T01(1,4) T02(1,4)];
y01_02=[T01(2,4) T02(2,4)];
z01_02=[T01(3,4) T02(3,4)];

x02_03=[T02(1,4) T03(1,4)];
y02_03=[T02(2,4) T03(2,4)];
z02_03=[T02(3,4) T03(3,4)];


line(x,y,z,'LineWidth',5,'Color','b');
line(x01_02,y01_02,z01_02,'LineWidth',5,'Color','r');
line(x02_03,y02_03,z02_03,'LineWidth',5,'Color','g');



% UIWAIT makes Kinematics wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Kinematics_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in ileri_kinematik.
function ileri_kinematik_Callback(hObject, eventdata, handles)
% hObject    handle to ileri_kinematik (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global h1 l2 d2 q1s q2s d3s

q1=str2double(get(handles.edit1,'String'));
q2=str2double(get(handles.edit2,'String'));
d3=str2double(get(handles.edit3,'String'));

t01=[cosd(q1) -sind(q1) 0 0;sind(q1) cosd(q1) 0 0;0 0 1 h1;0 0 0 1];
t12=[cosd(q2) -sind(q2) 0 0;0 0 -1 -d2;sind(q2) cosd(q2) 0 0;0 0 0 1];
t23=[1 0 0 0;0 0 1 (l2+d3);0 -1 0 0;0 0 0 1];

t03=t01*t12*t23;

set(handles.edit13,'String',t03(1,4));
set(handles.edit14,'String',t03(2,4));
set(handles.edit15,'String',t03(3,4));

set(handles.edit4,'String',t03(1,1));
set(handles.edit5,'String',t03(2,1));
set(handles.edit6,'String',t03(3,1));

set(handles.edit7,'String',t03(1,2));
set(handles.edit8,'String',t03(2,2));
set(handles.edit9,'String',t03(3,2));

set(handles.edit10,'String',t03(1,3));
set(handles.edit11,'String',t03(2,3));
set(handles.edit12,'String',t03(3,3));

% time=str2double(get(handles.edit16,'String'));
% %animasyon kodları
% %1. bag için kodlar
% t10=q1s;
% t1f=q1;
% 
% s10=q1s;
% s12=(3/time^2)*(t1f-t10);
% s13=(-2/time^3)*(t1f-t10);
% %2.bag için kodlar
% t20=q2s;
% t2f=q2;
% 
% s20=q2s;
% s22=(3/time^2)*(t2f-t10);
% s23=(-2/time^3)*(t2f-t10);
% %3. bag için kodlar
% t10=q1s;
% t1f=q1;
% 
% s10=q1s;
% s12=(3/time^2)*(t1f-t10);
% s13=(-2/time^3)*(t1f-t10);
% %s0 s2 s3 lerin hesplamasının sonu

Q1=(q1-q1s)/360;
Q2=(q2-q2s)/360;
D3=(d3-d3s)/360;

for i=1:360
    
    q1s=q1s+Q1;
    q2s=q2s+Q2;
    d3s=d3s+D3;

    t01=[cosd(q1s) -sind(q1s) 0 0;sind(q1s) cosd(q1s) 0 0;0 0 1 h1;0 0 0 1];
    t12=[cosd(q2s) -sind(q2s) 0 0;0 0 -1 -d2;sind(q2s) cosd(q2s) 0 0;0 0 0 1];
    t23=[1 0 0 0;0 0 1 (l2+d3s);0 -1 0 0;0 0 0 1];

    T01=t01;
    T02=t01*t12;
    T03=t01*t12*t23;

    x=[0 T01(1,4)];
    y=[0 T01(2,4)];
    z=[0 T01(3,4)];

    x01_02=[T01(1,4) T02(1,4)];
    y01_02=[T01(2,4) T02(2,4)];
    z01_02=[T01(3,4) T02(3,4)];

    x02_03=[T02(1,4) T03(1,4)];
    y02_03=[T02(2,4) T03(2,4)];
    z02_03=[T02(3,4) T03(3,4)];
   
    line(x,y,z,'LineWidth',5,'Color','b');
    line(x01_02,y01_02,z01_02,'LineWidth',5,'Color','r');
    line(x02_03,y02_03,z02_03,'LineWidth',5,'Color','g');
    
    pause(0.001)
    if i<360
        cla
    end
end


% --- Executes on button press in ters_kinematik.
function ters_kinematik_Callback(hObject, eventdata, handles)
% hObject    handle to ters_kinematik (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global h1 l2 d2 q1s q2s d3s

px=str2double(get(handles.edit13,'String'));
py=str2double(get(handles.edit14,'String'));
pz=str2double(get(handles.edit15,'String'));

q1(1)=atan2d(px,-py)+atan2d(sqrt(px^2+py^2-d2^2),d2);
q1(2)=atan2d(px,-py)+atan2d(-sqrt(px^2+py^2-d2^2),d2);

q2(1)=atan2d(-cosd(q1(1))*px-sind(q1(1))*py,pz-h1);
q2(2)=atan2d(-cosd(q1(1))*px-sind(q1(1))*py,-pz+h1);

q2(3)=atan2d(-cosd(q1(2))*px-sind(q1(2))*py,pz-h1);
q2(4)=atan2d(-cosd(q1(2))*px-sind(q1(2))*py,-pz+h1);

d3(1)=-sind(q2(1))*(cosd(q1(1))*px+sind(q1(1))*py)+cosd(q2(1))*(pz-h1)-l2;
d3(2)=-sind(q2(2))*(cosd(q1(1))*px+sind(q1(1))*py)+cosd(q2(2))*(pz-h1)-l2;

% d3(3)=-sind(q2(3))*(cosd(q1(2))*px+sind(q1(2))*py)+cosd(q2(3))*(pz-h1)-l2;
% d3(4)=-sind(q2(4))*(cosd(q1(2))*px+sind(q1(2))*py)+cosd(q2(4))*(pz-h1)-l2

t=1;

for i=1:2
    for j=1:4
        for k=1:2
            sonuc=Control(q1(i),q2(j),d3(k));
            if (px<=sonuc(1)+0.01 && px>=sonuc(1)-0.01) && (py<=sonuc(2)+0.01 && py>=sonuc(2)-0.01) && (pz<=sonuc(3)+0.01 && pz>=sonuc(3)-0.01)
               aci(t,1)=q1(i);
               aci(t,2)=q2(j);
               aci(t,3)=d3(k);
               t=t+1;
            end
        end
    end
end
aci;
set(handles.edit1,'String',aci(1,1));
set(handles.edit2,'String',aci(1,2));
set(handles.edit3,'String',aci(1,3));


%animasyon kodları
q1=aci(1,1);
q2=aci(1,2);
d3=aci(1,3);

Q1=(q1-q1s)/360;
Q2=(q2-q2s)/360;
D3=(d3-d3s)/360;


for i=1:360
    
    q1s=q1s+Q1;
    q2s=q2s+Q2;
    d3s=d3s+D3;
    
    t01=[cosd(q1s) -sind(q1s) 0 0;sind(q1s) cosd(q1s) 0 0;0 0 1 h1;0 0 0 1];
    t12=[cosd(q2s) -sind(q2s) 0 0;0 0 -1 -d2;sind(q2s) cosd(q2s) 0 0;0 0 0 1];
    t23=[1 0 0 0;0 0 1 (l2+d3s);0 -1 0 0;0 0 0 1];

    T01=t01;
    T02=t01*t12;
    T03=t01*t12*t23;

    x=[0 T01(1,4)];
    y=[0 T01(2,4)];
    z=[0 T01(3,4)];

    x01_02=[T01(1,4) T02(1,4)];
    y01_02=[T01(2,4) T02(2,4)];
    z01_02=[T01(3,4) T02(3,4)];

    x02_03=[T02(1,4) T03(1,4)];
    y02_03=[T02(2,4) T03(2,4)];
    z02_03=[T02(3,4) T03(3,4)];


    line(x,y,z,'LineWidth',5,'Color','b');
    line(x01_02,y01_02,z01_02,'LineWidth',5,'Color','r');
    line(x02_03,y02_03,z02_03,'LineWidth',5,'Color','g');
    
    pause(0.001)
    if i<360
        cla
    end
end


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



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in baslangic_konumunu_kaydet.
function baslangic_konumunu_kaydet_Callback(hObject, eventdata, handles)
% hObject    handle to baslangic_konumunu_kaydet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global q1s q2s d3s baslangic

baslangic(1)=q1s;
baslangic(2)=q2s;
baslangic(3)=d3s;
set(handles.baslangic_konumunu_kaydet,'backgroundcolor','green')


% --- Executes on button press in final_konumunu_kaydet.
function final_konumunu_kaydet_Callback(hObject, eventdata, handles)
% hObject    handle to final_konumunu_kaydet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global h1 l2 d2 q1s q2s d3s baslangic son
global x y z x01_02 y01_02 z01_02 x02_03 y02_03 z02_03

son(1)=q1s;
son(2)=q2s;
son(3)=d3s;

% Q1=(q1-q1s)/360;
% Q2=(q2-q2s)/360;
% D3=(d3-d3s)/360;

Q1=(baslangic(1)-son(1))/30;
Q2=(baslangic(2)-son(2))/30;
D3=(baslangic(3)-son(3))/30;

for i=1:30
    
    son(1)=son(1)+Q1;
    son(2)=son(2)+Q2;
    son(3)=son(3)+D3;

    q1s=son(1);
    q2s=son(2);
    d3s=son(3);
    
    t01=[cosd(q1s) -sind(q1s) 0 0;sind(q1s) cosd(q1s) 0 0;0 0 1 h1;0 0 0 1];
    t12=[cosd(q2s) -sind(q2s) 0 0;0 0 -1 -d2;sind(q2s) cosd(q2s) 0 0;0 0 0 1];
    t23=[1 0 0 0;0 0 1 (l2+d3s);0 -1 0 0;0 0 0 1];

    T01=t01;
    T02=t01*t12;
    T03=t01*t12*t23;

%     x(i)=[0 T01(1,4)];
%     y(i)=[0 T01(2,4)];
%     z(i)=[0 T01(3,4)];
    x(i,1)=0;
    x(i,2)=T01(1,4);

    y(i,1)=0;
    y(i,2)=T01(2,4);

    z(i,1)=0;
    z(i,2)=T01(3,4);

%     x01_02(i)=[T01(1,4) T02(1,4)];
%     y01_02(i)=[T01(2,4) T02(2,4)];
%     z01_02(i)=[T01(3,4) T02(3,4)];
    x01_02(i,1)=T01(1,4);
    x01_02(i,2)=T02(1,4);

    y01_02(i,1)=T01(2,4);
    y01_02(i,2)=T02(2,4);

    z01_02(i,1)=T01(3,4);
    z01_02(i,2)=T02(3,4);

%     x02_03(i)=[T02(1,4) T03(1,4)];
%     y02_03(i)=[T02(2,4) T03(2,4)];
%     z02_03(i)=[T02(3,4) T03(3,4)];
    x02_03(i,1)=T02(1,4);
    x02_03(i,2)=T03(1,4);

    y02_03(i,1)=T02(2,4);
    y02_03(i,2)=T03(2,4);

    z02_03(i,1)=T02(3,4);
    z02_03(i,2)=T03(3,4);


%     line(x,y,z,'LineWidth',5,'Color','b');
%     line(x01_02,y01_02,z01_02,'LineWidth',5,'Color','r');
%     line(x02_03,y02_03,z02_03,'LineWidth',5,'Color','g');
    
%     pause(0.001)
%     if i<360
%         cla
%     end
end

set(handles.final_konumunu_kaydet,'backgroundcolor','green')

function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in animasyonu_baslat.
function animasyonu_baslat_Callback(hObject, eventdata, handles)
% hObject    handle to animasyonu_baslat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x y z x01_02 y01_02 z01_02 x02_03 y02_03 z02_03
time=str2double(get(handles.edit16,'String'));
t=(time/30)*0.5;
for i=1:30
    
    line(x(i,:),y(i,:),z(i,:),'LineWidth',5,'Color','b');
    line(x01_02(i,:),y01_02(i,:),z01_02(i,:),'LineWidth',5,'Color','r');
    line(x02_03(i,:),y02_03(i,:),z02_03(i,:),'LineWidth',5,'Color','g');
    
    pause(t)
    if i<30
        cla
    end
end

set(handles.final_konumunu_kaydet,'backgroundcolor','red')
set(handles.baslangic_konumunu_kaydet,'backgroundcolor','red')


% --- Executes during object creation, after setting all properties.
function baslangic_konumunu_kaydet_CreateFcn(hObject, eventdata, handles)
% hObject    handle to baslangic_konumunu_kaydet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function final_konumunu_kaydet_CreateFcn(hObject, eventdata, handles)
% hObject    handle to final_konumunu_kaydet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
