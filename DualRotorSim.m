% DualRotorSim.m
% Simualtion script for a dual-rotor simulation
% todo(rodney) does it matter if it is parallel or coaxial? I don't think
% so. This script should let you setup either.

% Clear the workspace
clear all; close all; clc;
% Tell matlab to look in the src folder for the class files
addpath('src')

inputfile = 'dualRotorBaseline.txt';
sim = simulation(inputfile);

%% Make sure the vehicle we just built is what we were trying to build.
% The easiest way to do this is visually. WE can also play around here and
% then use this to set initial values.
%sim.showme(fvel,vornt,vpos,vvel,vangvel,rornt,rangvel)
% sim.fld.velocity = [0.5;0;0];
% pitch = 90; yaw = 0;
% theta0 = pitch*pi/180; gamma0 = yaw*pi/180; beta0 = 0;
% sim.vhcl.orientation = [theta0;gamma0;beta0]; % (theta, gamma, beta for 2-1-3) rotation
% sim.vhcl.position = [2;0;1];
% sim.vhcl.velocity = [0;0;0];
% dgamma0 = 0*2*pi; % rad/s
% w10 = cosd(0)*dgamma0;
% w20 = -sin(0)*dgamma0;
% sim.vhcl.angvel = [w10;w20;0];
sim.vhcl.rotors(1).orientation = [0;0;0];
sim.vhcl.rotors(2).orientation = [0;0;0];
% Compute the mass matrix - todo add to sim class and add an initial
% orientation to the rotors.
sim.vhcl.computeMstar;
rpm = 0;
sim.vhcl.rotors(1).angvel = [0;0;rpm/60*2*pi];
sim.vhcl.rotors(2).angvel = [0;0;-rpm/60*2*pi];
% Add a generator to the vehicle
gen = generator(0.19,1,0.1,1.0e-4); % todo size generator constant so that torque is reasonable
sim.vhcl.addGenerator(gen);
% hfig = vehicleVisualCheck(sim.fld,sim.vhcl);
% disp('Close figure to continue...');
% uiwait(hfig,5)
% disp('Figure closed, continuing');


%% Simulate
% No argument to the simulate method will default to simulation parameters
% specified in the input file and ode45 as the solver.
tspan = 0:0.005:1;
sim.simulate('tspan',tspan);
%sim.simulate('tspan',tspan,'stats','off','output',[]);

%% Write simulation results to file
sim.write2file('test_relaxedAssumptions'); % The method will append the .txt extension

%% Make a video of the simulation results


%% Plots
t = sim.times;
y = sim.states;
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
plot(t,y(:,13)*30/pi,'r',t,y(:,15)*30/pi,'b'); %rad/s*180/pi*60/360 = 30/pi
xlabel('Time (s)'); ylabel('Angular Rate (RPM - in Body Frame)');
legend({'p3','q3'},'Location','Best');