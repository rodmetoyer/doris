% DualRotorSim.m
% Simualtion script for a dual-rotor simulation
% todo(rodney) does it matter if it is parallel or coaxial? I don't think
% so. This script should let you setup either.

% Clear the workspace
clear all; close all; clc;
% Tell matlab to look in the src folder for the class files
addpath('src')

inputfile = 'tacticalScale23.m';
sim = simulation(inputfile);

%% Make sure the vehicle we just built is what we were trying to build.
% The easiest way to do this is visually. You can also play around here and
% then use this to set/reset initial values for one-off simulations.
% hfig = sim.showme;
% disp('Close figure to continue...');
% uiwait(hfig,5)
% disp('Figure closed, continuing');

%% Simulate
% No argument to the simulate method will default to simulation parameters
% specified in the input file and ode45 as the solver.
tspan = 0:0.005:10;
sim.simulate; % ('output',[])
%sim.simulate('tspan',tspan,'stats','off','output',[]);

%% Write simulation results to file
% If you want to name the results file something other than the simulation
% name just pass the filename as an argument with no extension (the method
% will append .txt).
sim.write2file;

%% Make a video of the simulation results
% The makeMovie method is static. The first argument is the name of the
% data and input file combo to use. The second argument is the name of the
% movie file. If you only pass one name the movie file gets that name.
simulation.makeMovie(sim.name);

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