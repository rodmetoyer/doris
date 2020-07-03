function varargout = visualizeAngles(varargin)
% VISUALIZEANGLES MATLAB code for visualizeAngles.fig
%      VISUALIZEANGLES, by itself, creates a new VISUALIZEANGLES or raises the existing
%      singleton*.
%
%      H = VISUALIZEANGLES returns the handle to a new VISUALIZEANGLES or the handle to
%      the existing singleton*.
%
%      VISUALIZEANGLES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VISUALIZEANGLES.M with the given input arguments.
%
%      VISUALIZEANGLES('Property','Value',...) creates a new VISUALIZEANGLES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before visualizeAngles_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to visualizeAngles_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help visualizeAngles

% Last Modified by GUIDE v2.5 26-Sep-2018 10:20:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @visualizeAngles_OpeningFcn, ...
                   'gui_OutputFcn',  @visualizeAngles_OutputFcn, ...
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


% --- Executes just before visualizeAngles is made visible.
function visualizeAngles_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to visualizeAngles (see VARARGIN)

handles.U_m_s = 0; % 0 to 10.0
handles.a = 0; % 0 to 1
handles.r_m = 0; % 0 to 10.0
handles.omega_rpm = 0; % 0 to 500
handles.twistAngle_deg = 0.0;

handles.omega_rad = handles.omega_rpm*2*pi/60;
handles.vax_m = (1-handles.a)*handles.U_m_s;  % (1-a)*U
handles.vtan_m = handles.omega_rad*handles.r_m; % omega*r
handles.chord = 0.0254; % TODO(rodney) lets make this an input
handles.length = 0.1905;
handles.centerLoc = 0.25; % Typically quarter chord
handles.chordLinePts = [-handles.centerLoc*cosd(handles.twistAngle_deg)*handles.chord -handles.centerLoc*sind(handles.twistAngle_deg)*handles.chord;...
    (handles.chord-handles.centerLoc)*cosd(handles.twistAngle_deg)*handles.chord (handles.chord-handles.centerLoc)*sind(handles.twistAngle_deg)*handles.chord];
handles.phi = atan2(handles.vax_m,handles.vtan_m); handles.phi_deg = handles.phi*180/pi;
handles.aoa_deg = handles.phi_deg - handles.twistAngle_deg;
handles.aoaLinePts = [-(handles.centerLoc)*cosd(handles.aoa_deg+handles.twistAngle_deg) -(handles.centerLoc)*sind(handles.aoa_deg+handles.twistAngle_deg); 0 0];
if handles.aoa_deg < 0
    handles.aoaLineColor = 'r';
else
    handles.aoaLineColor = 'g';
end

% get the points for the airfoil outline
% basefile = [pwd '\..\airfoilData\S814_base.txt'];
% A = importdata(basefile);
A = getS814Coords;
% Rotate to the correct AoA
handles.basex = A(:,1);
handles.basey = A(:,2);
handles.x = (handles.basex-handles.centerLoc)*handles.chord*cosd(handles.twistAngle_deg)-handles.basey*handles.chord*sind(handles.twistAngle_deg);
handles.y = (handles.basex-handles.centerLoc)*handles.chord*sind(handles.twistAngle_deg)+handles.basey*handles.chord*cosd(handles.twistAngle_deg);
 
makeplot(handles);

% Choose default command line output for visualizeAngles
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes visualizeAngles wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = visualizeAngles_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function freestreamvel_Callback(hObject, eventdata, handles)
% hObject    handle to freestreamvel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.U_m_s = get(hObject,'Value');
set(hObject,'Value',round(handles.U_m_s,1));
set(handles.text6,'String',num2str(get(hObject,'Value')));
makeplot(handles);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function freestreamvel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freestreamvel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function inductionfactor_Callback(hObject, eventdata, handles)
% hObject    handle to inductionfactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.a = get(hObject,'Value');
set(hObject,'Value',round(handles.a,3));
set(handles.text7,'String',num2str(get(hObject,'Value')));
makeplot(handles);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function inductionfactor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to inductionfactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function radiallocation_Callback(hObject, eventdata, handles)
% hObject    handle to radiallocation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.r_m = get(hObject,'Value');
set(hObject,'Value',round(handles.r_m,2));
set(handles.text8,'String',num2str(get(hObject,'Value')));
makeplot(handles);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function radiallocation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to radiallocation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function angularspeed_Callback(hObject, eventdata, handles)
% hObject    handle to angularspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.omega_rpm = get(hObject,'Value');
set(hObject,'Value',round(handles.omega_rpm,0));
set(handles.text9,'String',num2str(get(hObject,'Value')));
makeplot(handles);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function angularspeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angularspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function makeplot(handles)
handles.omega_rad = handles.omega_rpm*2*pi/60;
handles.vax_m = (1-handles.a)*handles.U_m_s;  % (1-a)*U
handles.vtan_m = handles.omega_rad*handles.r_m; % omega*r
%aoa_deg = -20.0;
%handles.chord = 1.0;
handles.centerLoc = 0.25; % Typically quarter chord

% Compute
handles.chordLinePts = [-handles.centerLoc*cosd(handles.twistAngle_deg)*handles.chord -handles.centerLoc*sind(handles.twistAngle_deg)*handles.chord;...
    (1-handles.centerLoc)*cosd(handles.twistAngle_deg)*handles.chord (1-handles.centerLoc)*sind(handles.twistAngle_deg)*handles.chord];
handles.phi = atan2(handles.vax_m,handles.vtan_m); handles.phi_deg = handles.phi*180/pi;
handles.aoa_deg = handles.phi_deg - handles.twistAngle_deg;
handles.aoaLinePts = [-handles.chord*(handles.centerLoc)*cosd(handles.aoa_deg+handles.twistAngle_deg) -handles.chord*(handles.centerLoc)*sind(handles.aoa_deg+handles.twistAngle_deg); 0 0];
sp1 = summer(4);
sp2 = autumn(4);
c = [sp2;sp1(4,:);sp1(3,:);sp1(2,:);sp1(1,:);sp1;sp2(4,:);sp2(3,:);sp2(2,:);sp2(1,:)];
%c = [sp2;sp1;sp1(4,:);sp1(3,:);sp1(2,:);sp1(1,:);sp2(4,:);sp2(3,:);sp2(2,:);sp2(1,:)];
if handles.aoa_deg <= 0.5
    handles.aoaLineColor = [1 0 0];
elseif handles.aoa_deg > 15.5
    handles.aoaLineColor = [1 0 0];
else
    handles.aoaLineColor = c(round(handles.aoa_deg),:);
end

handles.x = (handles.basex-handles.centerLoc)*handles.chord*cosd(handles.twistAngle_deg)-handles.basey*handles.chord*sind(handles.twistAngle_deg);
handles.y = (handles.basex-handles.centerLoc)*handles.chord*sind(handles.twistAngle_deg)+handles.basey*handles.chord*cosd(handles.twistAngle_deg);

%s = handles.chord*handles.length;
[l,d] = getLiftDrag(handles.aoa_deg,handles.chord,handles.U_m_s);
% This will give us forces per meter. Scaling down to per length of chord
% just so the plot is prettier. 
l = l*handles.chord; d = d*handles.chord;
bravo = atan2(handles.vax_m,handles.vtan_m);
totalColor = 'g';
if d*cos(bravo)-l*sin(bravo) > 0
    totalColor = 'r';
end

plot(handles.x,handles.y,'ob','MarkerSize',2.0,'MarkerFaceColor','b')
hold on
axis equal
%sp1 = max([handles.vax_m,handles.vtan_m,handles.chord]);
axis([min(-1.25*l*sin(bravo),-1.25*handles.centerLoc*handles.chord) 1.25*max([handles.vtan_m,(handles.chord-handles.centerLoc*handles.chord)]) -1.25*max([handles.centerLoc*handles.chord,-min(handles.y)]) 1.25*max([handles.U_m_s,max(handles.y),handles.centerLoc*handles.chord])])
plot([0 0],[0 handles.vax_m],'--k','LineWidth',1.0); % Axial velocity
plot([0 0],[handles.vax_m handles.U_m_s],'--r','LineWidth',1.0); % Axial velocity
plot([0 handles.vtan_m],[0 0],'--k','LineWidth',1.0);% Tangential velocity
plot([0 handles.vtan_m],[0 handles.vax_m],'-k','LineWidth',2.0) % Relative velocity
plot(handles.chordLinePts(:,1),handles.chordLinePts(:,2),':k','LineWidth',2.0);
plot(handles.aoaLinePts(:,1),handles.aoaLinePts(:,2),'Color',handles.aoaLineColor,'LineWidth',2.0);
plot([0 -l*sin(bravo)],[0 l*cos(bravo)],':c','LineWidth',4.0); %Lift
plot([0 d*cos(bravo)],[0 d*sin(bravo)],':m','LineWidth',4.0); %Drag
plot([0 d*cos(bravo)-l*sin(bravo)],[0 d*sin(bravo)+l*cos(bravo)],['-' totalColor],'LineWidth',2.0); %Total
annotation(gcf,'textarrow',[0.1 0.065],[0.05 0.05],'String','Direction of blade section travel');
annotation(gcf,'textarrow',[0.2 0.2],[0.1 0.15],'String','Direction of freestream flow');
s = sprintf('Angle of attack %2.1f (deg)', handles.aoa_deg);
title(s);
hold off

% --- Executes on slider movement.
function twist_Callback(hObject, eventdata, handles)
% hObject    handle to twist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.twistAngle_deg = get(hObject,'Value');
set(hObject,'Value',round(handles.twistAngle_deg,1));
set(handles.text11,'String',num2str(get(hObject,'Value')));
makeplot(handles);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function twist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to twist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.U_m_s = 0.8; % 0 to 10.0
handles.a = 1/3; % 0 to 1
handles.r_m = 0.1; % 0 to 10.0
handles.omega_rpm = 210; % 0 to 500
handles.twistAngle_deg = 4.80;
set(handles.text6,'String',num2str(handles.U_m_s));
set(handles.text7,'String',num2str(handles.a));
set(handles.text8,'String',num2str(handles.r_m));
set(handles.text9,'String',num2str(handles.omega_rpm));
set(handles.text11,'String',num2str(handles.twistAngle_deg));
makeplot(handles);
guidata(hObject, handles);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.U_m_s = 0; % 0 to 10.0
handles.a = 0; % 0 to 1
handles.r_m = 0; % 0 to 10.0
handles.omega_rpm = 0; % 0 to 500
handles.twistAngle_deg = 0;
set(handles.text6,'String',num2str(handles.U_m_s));
set(handles.text7,'String',num2str(handles.a));
set(handles.text8,'String',num2str(handles.r_m));
set(handles.text9,'String',num2str(handles.omega_rpm));
set(handles.text11,'String',num2str(handles.twistAngle_deg));
makeplot(handles);
guidata(hObject, handles);


% --- Executes on slider movement.
function chordtag_Callback(hObject, eventdata, handles)
% hObject    handle to chordtag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.chord = get(hObject,'Value');
set(hObject,'Value',round(handles.chord,4));
s = sprintf('%4.3f %s %4.3f %s',handles.chord," (m) or ",handles.chord*39.3701," (in)");
set(handles.text13,'String',s);
makeplot(handles);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function chordtag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to chordtag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .6 .9]);
end


function coords = getS814Coords()
    coords = [1	0	0;...
    0.996277	0.001079	0;...
    0.985681	0.004644	0;...
    0.969429	0.010691	0;...
    0.948574	0.018525	0;...
    0.923625	0.027157	0;...
    0.894505	0.035738	0;...
    0.86085	0.044178	0;...
    0.823023	0.052748	0;...
    0.781586	0.061424	0;...
    0.73713	0.070108	0;...
    0.690273	0.078659	0;...
    0.641651	0.086901	0;...
    0.59191	0.094633	0;...
    0.541692	0.101631	0;...
    0.491625	0.107648	0;...
    0.442317	0.112418	0;...
    0.394345	0.115645	0;...
    0.348257	0.116962	0;...
    0.304384	0.115627	0;...
    0.261983	0.111612	0;...
    0.221337	0.105629	0;...
    0.182903	0.097997	0;...
    0.147112	0.088966	0;...
    0.114367	0.078763	0;...
    0.085039	0.067619	0;...
    0.059481	0.055766	0;...
    0.038007	0.043458	0;...
    0.020952	0.030954	0;...
    0.008577	0.018556	0;...
    0.001431	0.006672	0;...
    0.001119	0.005797	0;...
    0.000338	0.002959	0;...
    0.000002	0.000223	0;...
    0.000000	0.000000	0;...
    0.000245	-0.002549	0;...
    0.000678	-0.004584	0;...
    0.000925	-0.005491	0;...
    0.006381	-0.01743	0;...
    0.016792	-0.031576	0;...
    0.031367	-0.046492	0;...
    0.049641	-0.061655	0;...
    0.07124	-0.076713	0;...
    0.09561	-0.091093	0;...
    0.122438	-0.104309	0;...
    0.151203	-0.115726	0;...
    0.181669	-0.124578	0;...
    0.213672	-0.130235	0;...
    0.247139	-0.13131	0;...
    0.283942	-0.127757	0;...
    0.323782	-0.120293	0;...
    0.367326	-0.109093	0;...
    0.414593	-0.095249	0;...
    0.465255	-0.07966	0;...
    0.518814	-0.063236	0;...
    0.574574	-0.046894	0;...
    0.631638	-0.03152	0;...
    0.688908	-0.017904	0;...
    0.745125	-0.006688	0;...
    0.798908	0.001698	0;...
    0.84883	0.00707	0;...
    0.893492	0.009517	0;...
    0.931609	0.009395	0;...
    0.962086	0.007279	0;...
    0.983819	0.003889	0;...
    0.996132	0.001014	0;...
    1	-0	0];

function [l,d] = getLiftDrag(aa,s,u)
% INPUT
%    aa = angle of attack
%     s = planform area
% OUTPUT
%    [l,d] = lift and drag
% assuming density of water
rho = 997.0; %kg/m^3

d = getClCdDat;
cl = interp1(d.alpha,d.cl,aa);
cd = interp1(d.alpha,d.cd,aa);

l = cl*rho*u^2*s/2;
d = cd*rho*u^2*s/2;


function d = getClCdDat(af)
% INPUT
%   af = airfoil
% OUTPUT
%   d = data

% TODO make a way to switch between airfoils
if nargin < 1
    af = 1; % TODO switch from int to something meaningful
end

if af == 1
A = [-1.8000000E+02 -1.4678165E-01 7.9652736E-02;
-1.7800000E+02 -1.4678165E-01 7.9652736E-02;
-1.7600000E+02 4.2439552E-02 8.2793070E-02;
-1.7400000E+02 2.4275832E-01 8.8976768E-02;
-1.7200000E+02 3.9394938E-01 1.0581558E-01;
-1.7000000E+02 4.9571323E-01 1.2996463E-01;
-1.6800000E+02 5.6390860E-01 1.6076760E-01;
-1.6600000E+02 6.4632530E-01 2.0485765E-01;
-1.6400000E+02 6.7087362E-01 2.4082366E-01;
-1.6200000E+02 6.5643342E-01 2.8418120E-01;
-1.6000000E+02 5.9521264E-01 3.0863577E-01;
-1.5800000E+02 5.8043315E-01 3.4821616E-01;
-1.5600000E+02 5.3544644E-01 3.9164799E-01;
-1.5400000E+02 5.4748586E-01 4.3447292E-01;
-1.5200000E+02 5.5205761E-01 4.8317035E-01;
-1.5000000E+02 5.5093583E-01 5.2936715E-01;
-1.4800000E+02 5.5278581E-01 5.7404744E-01;
-1.4600000E+02 5.6657920E-01 6.4320655E-01;
-1.4400000E+02 5.6592855E-01 6.9444101E-01;
-1.4200000E+02 5.6414662E-01 7.5517833E-01;
-1.4000000E+02 5.7009252E-01 8.1571652E-01;
-1.3800000E+02 5.6628620E-01 8.7400326E-01;
-1.3600000E+02 5.8062706E-01 9.5679163E-01;
-1.3400000E+02 5.7344682E-01 1.0286178E+00;
-1.3200000E+02 5.7829101E-01 1.1107014E+00;
-1.3000000E+02 5.8294445E-01 1.2064844E+00;
-1.2800000E+02 5.7324546E-01 1.2832871E+00;
-1.2600000E+02 5.6397963E-01 1.3464883E+00;
-1.2400000E+02 5.5328297E-01 1.4426537E+00;
-1.2200000E+02 5.3554332E-01 1.5212004E+00;
-1.2000000E+02 5.1191078E-01 1.5994275E+00;
-1.1800000E+02 4.9172629E-01 1.6806169E+00;
-1.1600000E+02 4.6410281E-01 1.7705554E+00;
-1.1400000E+02 4.6332771E-01 1.8520105E+00;
-1.1200000E+02 4.1502448E-01 1.9308838E+00;
-1.1000000E+02 3.6179145E-01 1.9931776E+00;
-1.0800000E+02 3.0599231E-01 2.0766381E+00;
-1.0600000E+02 2.4974337E-01 2.1194934E+00;
-1.0400000E+02 1.8614411E-01 2.1581279E+00;
-1.0200000E+02 1.1873808E-01 2.2033037E+00;
-1.0000000E+02 4.0836532E-02 2.2569109E+00;
-9.8000000E+01 -9.1398440E-03 2.2689651E+00;
-9.6000000E+01 -8.7741095E-02 2.2834486E+00;
-9.4000000E+01 -1.4860230E-01 2.2943541E+00;
-9.2000000E+01 -2.3637472E-01 2.3250047E+00;
-9.0000000E+01 -2.9828385E-01 2.3161557E+00;
-8.8000000E+01 -3.7334264E-01 2.3180988E+00;
-8.6000000E+01 -4.3551879E-01 2.2930785E+00;
-8.4000000E+01 -5.0179640E-01 2.2766917E+00;
-8.2000000E+01 -5.6883933E-01 2.2372158E+00;
-8.0000000E+01 -6.2097681E-01 2.2126886E+00;
-7.8000000E+01 -6.7664195E-01 2.1731955E+00;
-7.6000000E+01 -7.2996889E-01 2.1188444E+00;
-7.4000000E+01 -7.7919644E-01 2.0768038E+00;
-7.2000000E+01 -7.9543428E-01 1.9471083E+00;
-7.0000000E+01 -8.4157373E-01 1.8819063E+00;
-6.8000000E+01 -8.6930279E-01 1.8275860E+00;
-6.6000000E+01 -8.9300581E-01 1.7554288E+00;
-6.4000000E+01 -9.1206868E-01 1.6773555E+00;
-6.2000000E+01 -9.2522533E-01 1.6008923E+00;
-6.0000000E+01 -9.3526782E-01 1.5122398E+00;
-5.8000000E+01 -9.3946911E-01 1.4372348E+00;
-5.6000000E+01 -9.4204335E-01 1.3770612E+00;
-5.4000000E+01 -9.2989255E-01 1.2766271E+00;
-5.2000000E+01 -9.1951298E-01 1.1915662E+00;
-5.0000000E+01 -9.0197537E-01 1.1051001E+00;
-4.8000000E+01 -8.7364620E-01 1.0113402E+00;
-4.6000000E+01 -8.6366461E-01 9.5139970E-01;
-4.4000000E+01 -8.3215118E-01 8.7237532E-01;
-4.2000000E+01 -8.0777110E-01 8.0875349E-01;
-4.0000000E+01 -7.7947855E-01 7.4209008E-01;
-3.8000000E+01 -7.5362796E-01 6.7387896E-01;
-3.6000000E+01 -7.2818506E-01 6.0687274E-01;
-3.4000000E+01 -6.9439873E-01 5.5712948E-01;
-3.2000000E+01 -6.5805798E-01 4.8143120E-01;
-3.0000000E+01 -6.5208471E-01 4.3283550E-01;
-2.8000000E+01 -8.9511188E-01 2.6957546E-01;
-2.6000000E+01 -8.9268967E-01 2.5791083E-01;
-2.4000000E+01 -8.9132356E-01 2.7994374E-01;
-2.2000000E+01 -5.2786403E-01 2.5924553E-01;
-2.0000000E+01 -3.8960682E-01 2.1179976E-01;
-1.8000000E+01 -3.4672572E-01 1.8863874E-01;
-1.6000000E+01 -3.1005111E-01 1.7451368E-01;
-1.4000000E+01 -2.5915946E-01 1.5534300E-01;
-1.2000000E+01 -2.0989142E-01 1.3435624E-01;
-1.0000000E+01 -1.5659827E-01 1.2041299E-01;
-8.0000000E+00 -9.0359045E-02 1.0436785E-01;
-6.0000000E+00 -7.5461000E-04 8.9467358E-02;
-4.0000000E+00 8.3748656E-02 7.9409778E-02;
-2.0000000E+00 1.5938250E-01 6.9510535E-02;
0.0000000E+00 2.8483032E-01 5.4680579E-02;
2.0000000E+00 3.6721298E-01 4.6730593E-02;
4.0000000E+00 5.4610368E-01 4.6258157E-02;
6.0000000E+00 7.1317161E-01 4.9106498E-02;
8.0000000E+00 8.6748101E-01 5.8452407E-02;
1.0000000E+01 9.7559449E-01 7.3114744E-02;
1.1000000E+01 1.0245737E+00 7.8725412E-02;
1.2000000E+01 1.0687444E+00 8.7588077E-02;
1.3000000E+01 1.0903345E+00 1.0237681E-01;
1.4000000E+01 1.0942533E+00 1.1068988E-01;
1.5000000E+01 1.1182471E+00 1.3034797E-01;
1.6000000E+01 1.1171826E+00 1.3846723E-01;
1.7000000E+01 1.1299015E+00 1.5399270E-01;
1.8000000E+01 1.1346799E+00 1.7678675E-01;
1.9000000E+01 1.1240267E+00 1.8731181E-01;
2.0000000E+01 8.3870787E-01 3.5183902E-01;
2.1000000E+01 8.2200952E-01 3.8311359E-01;
2.2000000E+01 8.1963896E-01 4.2169716E-01;
2.3000000E+01 8.3778452E-01 4.4617267E-01;
2.4000000E+01 8.2457043E-01 4.6924189E-01;
2.5000000E+01 8.0796567E-01 4.9151684E-01;
2.6000000E+01 8.3030682E-01 5.2116348E-01;
2.7000000E+01 8.2161039E-01 5.4338335E-01;
2.8000000E+01 8.2363227E-01 5.6152142E-01;
2.9000000E+01 8.3968586E-01 5.9700405E-01;
3.0000000E+01 8.4695099E-01 6.2866381E-01;
3.2000000E+01 8.7778501E-01 6.8634344E-01;
3.4000000E+01 8.9509575E-01 7.3832419E-01;
3.6000000E+01 9.2043030E-01 8.0847903E-01;
3.8000000E+01 9.4794556E-01 8.8343989E-01;
4.0000000E+01 9.6991706E-01 9.5402125E-01;
4.2000000E+01 9.8924615E-01 1.0293072E+00;
4.4000000E+01 1.0600551E+00 1.1377978E+00;
4.6000000E+01 1.0738051E+00 1.2170893E+00;
4.8000000E+01 1.0779319E+00 1.3116774E+00;
5.0000000E+01 1.0796554E+00 1.3974054E+00;
5.2000000E+01 1.0772726E+00 1.4913190E+00;
5.4000000E+01 1.0745405E+00 1.5698589E+00;
5.6000000E+01 1.0540412E+00 1.6585597E+00;
5.8000000E+01 1.0368980E+00 1.7425444E+00;
6.0000000E+01 1.0148406E+00 1.8318252E+00;
6.2000000E+01 9.9329589E-01 1.8783527E+00;
6.4000000E+01 9.5233313E-01 1.9404593E+00;
6.6000000E+01 9.2198302E-01 2.0252622E+00;
6.8000000E+01 8.5458786E-01 2.0894644E+00;
7.0000000E+01 8.0390739E-01 2.1581727E+00;
7.2000000E+01 7.4743893E-01 2.1972970E+00;
7.4000000E+01 6.9500248E-01 2.2301234E+00;
7.6000000E+01 6.3502763E-01 2.2810448E+00;
7.8000000E+01 5.7125655E-01 2.3200536E+00;
8.0000000E+01 5.0520070E-01 2.3441257E+00;
8.2000000E+01 4.3801559E-01 2.3816275E+00;
8.4000000E+01 3.6310531E-01 2.3802463E+00;
8.6000000E+01 3.0695368E-01 2.3920529E+00;
8.8000000E+01 2.2804942E-01 2.3654239E+00;
9.0000000E+01 1.6559225E-01 2.3866186E+00;
9.2000000E+01 9.6618277E-02 2.3565418E+00;
9.4000000E+01 1.4717392E-02 2.3525100E+00;
9.6000000E+01 -6.0374912E-02 2.3322160E+00;
9.8000000E+01 -1.2593631E-01 2.3127150E+00;
1.0000000E+02 -1.9321642E-01 2.2394812E+00;
1.0200000E+02 -2.7510879E-01 2.2041182E+00;
1.0400000E+02 -3.1931813E-01 2.1916608E+00;
1.0600000E+02 -3.6899825E-01 2.1292157E+00;
1.0800000E+02 -4.3003735E-01 2.0672758E+00;
1.1000000E+02 -4.7961145E-01 2.0125077E+00;
1.1200000E+02 -5.1733060E-01 1.9393598E+00;
1.1400000E+02 -5.5482107E-01 1.8655790E+00;
1.1600000E+02 -5.8946702E-01 1.7852677E+00;
1.1800000E+02 -6.2649119E-01 1.7110824E+00;
1.2000000E+02 -6.5205738E-01 1.6351848E+00;
1.2200000E+02 -6.7640349E-01 1.5510658E+00;
1.2400000E+02 -7.0350847E-01 1.4755193E+00;
1.2600000E+02 -7.0065573E-01 1.3408718E+00;
1.2800000E+02 -7.1113415E-01 1.2507437E+00;
1.3000000E+02 -7.3117364E-01 1.2093073E+00;
1.3200000E+02 -7.5362286E-01 1.1477435E+00;
1.3400000E+02 -7.6060654E-01 1.0727883E+00;
1.3600000E+02 -7.7687181E-01 1.0074310E+00;
1.3800000E+02 -7.8651908E-01 9.5732897E-01;
1.4000000E+02 -8.0361985E-01 9.0723552E-01;
1.4200000E+02 -8.0350376E-01 8.4444524E-01;
1.4400000E+02 -8.1482889E-01 7.9424923E-01;
1.4600000E+02 -8.1935203E-01 7.4199877E-01;
1.4800000E+02 -8.3770181E-01 6.9231601E-01;
1.5000000E+02 -8.4569752E-01 6.3962994E-01;
1.5200000E+02 -8.6799302E-01 6.0181080E-01;
1.5400000E+02 -8.7962766E-01 5.4526209E-01;
1.5600000E+02 -8.9993908E-01 5.0672720E-01;
1.5800000E+02 -9.6228809E-01 4.6733074E-01;
1.6000000E+02 -1.0127464E+00 4.1942439E-01;
1.6200000E+02 -1.1221912E+00 3.5078506E-01;
1.6400000E+02 -1.1749273E+00 2.8659366E-01;
1.6600000E+02 -1.1923990E+00 2.2761724E-01;
1.6800000E+02 -1.1342718E+00 1.6702122E-01;
1.7000000E+02 -1.0522687E+00 1.3191151E-01;
1.7200000E+02 -9.6392648E-01 1.1157525E-01;
1.7400000E+02 -7.6486426E-01 9.7915887E-02;
1.7600000E+02 -4.7377607E-01 9.8448001E-02;
1.7800000E+02 -3.7960950E-01 9.0072990E-02;
1.8000000E+02 -1.4678165E-01 7.9652736E-02;];
end
d.alpha = A(:,1);
d.cl = A(:,2);
d.cd = A(:,3);
