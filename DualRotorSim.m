% DualRotorSim.m
% Simualtion script for a dual-rotor simulation
% todo(rodney) does it matter if it is parallel or coaxial? I don't think
% so. This script should let you setup either.

% Clear the workspace
clearvars; close all; clc;
% Tell matlab to look in the src folder for the class files
addpath('src')

% Input file
% todo(rodney) need to make this bulletproof. You can't do some whit from
% file and adjust other shit in the script. It needs to be all from file or
% nothing from file. Maybe with a gui that starts with choose input or
% enter manually.
inputfile = 'dualRotorBaseline.txt';
fid = fopen(['input\' inputfile]);
while true
    tline = fgetl(fid);            
    if isnumeric(tline)
        break;
    end
    eval(tline);
end
fclose(fid);
moviefile = ['products\videos\' runname '.avi'];

%% Make objects
% fluid
water = fluid(fluidtype); % No arguments to fluid gives the obj water properties
% airfoils - same for the entire rotor so we just need one
af = airfoil(airfoiltype);
% blade sections
bs = bladesection(secChord,secWidth,af);
% Make a blade comprised of the same section.
% Rotor 1 blades
for i=1:1:numSections
    section(i) = bs;
end
for i=1:1:numBlades
    bld1(i) = blade(section,bladeMass,twist);
end
% Rotor 2 blades need to twist in the opposite direction
for i=1:1:numBlades
    bld2(i) = blade(section,bladeMass,twist);
    bld2(i).reverseTwist;
end
% Make a set of rotors
r1 = rotor(bld1);
r1.setID(1);
% Need to make rotor 2 have blades with twist 180-
r2 = rotor(bld2);
r2.setID(2);
% Make a vehicle body
vbod = vehiclebody(vbmass,I);
vbod.setRelativeDensity(0.5);
% Make a vehicle
rotPoints = [rot1point,rot2point];
v = vehicle;
v.init(vbod,[r1,r2],rotPoints,vbcentermass,vbtetherpoint,vbbuoypoint);
% Associate rotor objects with vehicle object
r1.connectVehicle(v);
r2.connectVehicle(v);

%% Make sure the vehicle we just built is what we were trying to build.
% The easiest way to do this is visually. WE can also play around here and
% then use this to set initial values.
water.velocity = [0.5;0;0];
pitch = 90; yaw = 0;
theta0 = pitch*pi/180; gamma0 = yaw*pi/180; beta0 = 0;
v.orientation = [theta0;gamma0;beta0]; % (theta, gamma, beta for 2-1-3) rotation
v.position = [2;0;1];
v.velocity = [0;0;0];
dgamma0 = 0*2*pi; % rad/s
w10 = cosd(beta0)*dgamma0;
w20 = -sin(beta0)*dgamma0;
v.angvel = [w10;w20;0];
v.rotors(1).orientation = [0;0;0];
v.rotors(2).orientation = [0;0;0];
rpm = 0;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];
hfig = vehicleVisualCheck(water,v);
disp('Close figure to continue...');
waitfor(hfig)
disp('Figure closed, continuing');


%% Set-up simulation
%tspan = 0:tstep:totalSimTime;
% temporary control for debugging
tspan = 0:0.005:20;

% Initial states
% State vector
% x = [x1; x2; x3; theta; gamma; beta; w1; w2; w3; u1; u2; u3; p3; fi3; q3; sy3];
x0 = [v.position; v.orientation; v.angvel; v.velocity;...
    v.rotors(1).angvel(3); v.rotors(1).orientation(3); v.rotors(2).angvel(3); v.rotors(2).orientation(3)];
% We are going to add a tether with two links and one central node

disp('Running the simulation');
%[t, y] = ode45(@(t,y) rotorState(t,y,rotor,water),tspan,x0);
opts = odeset('RelTol',1e-6,'AbsTol',1e-6,'Stats','on','OutputFcn',@odeplot);
%opts = odeset('RelTol',1e-5,'AbsTol',1e-6);
[t, y] = ode45(@(t,y) vehicleState( t,y,v,water),tspan,x0,opts);

%% Write simulation results to file
simCaseID = 0;
% todo make a simulation class that can make a number of simualtion cases
% or inport simulation cases from input files.
if ~exist('data','dir')
    mkdir('data');
end
flnm = ['data\simCase' num2str(simCaseID) '.txt'];
writeToFile(t,y,flnm);

%% Make a video of the simulation results


%% Plots
plotlowx = 50; plotlowy = 50; plotw = 600; ploth = 400;
figure('Position',[plotlowx plotlowy plotw ploth])
plot(t,y(:,1),'r',t,y(:,2),'b',t,y(:,3),'g');
xlabel('Time (s)'); ylabel('Position (m)');
legend({'x1','x2','x3'},'Location','Best');

plotlowy = plotlowy+ploth; % Move up
figure('Position',[plotlowx plotlowy plotw ploth])
plot(t,y(:,4)*180/pi,'r',t,y(:,5)*180/pi,'b',t,y(:,6)*180/pi,'g');
xlabel('Time (s)'); ylabel('Angle (deg)');
legend({'theta','gamma','beta'},'Location','Best');

plotlowx = plotlowx+plotw; plotlowy = 50; % Move over
figure('Position',[plotlowx plotlowy plotw ploth])
plot(t,y(:,7)*180/pi,'r',t,y(:,8)*180/pi,'b',t,y(:,9)*180/pi,'g');
xlabel('Time (s)'); ylabel('Angular Rate (deg/s)');
legend({'w1','w2','w3'},'Location','Best');

plotlowy = plotlowy+ploth; % Move up
figure('Position',[plotlowx plotlowy plotw ploth])
plot(t,y(:,10),'r',t,y(:,11),'b',t,y(:,12),'g');
xlabel('Time (s)'); ylabel('Speed (m/s)');
legend({'u1','u2','u3'},'Location','Best');

plotlowx = plotlowx+plotw; plotlowy = 50; % Move over
figure('Position',[plotlowx plotlowy plotw ploth])
plot(t,y(:,14)*180/pi,'r',t,y(:,16)*180/pi,'b');
xlabel('Time (s)'); ylabel('Angle (deg)');
legend({'fi3','sy3'},'Location','Best');

plotlowy = plotlowy+ploth; % Move up
figure('Position',[plotlowx plotlowy plotw ploth])
plot(t,y(:,13)*180/pi,'r',t,y(:,15)*180/pi,'b');
xlabel('Time (s)'); ylabel('Angular Rate (deg/s)');
legend({'p3','q3'},'Location','Best');