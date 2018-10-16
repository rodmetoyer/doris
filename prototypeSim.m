% prototypeSim.m
% Simualtion script for the protptype single-rotor simulation

clearvars; close all; clc;

%% Make objects
% fluid
water = fluid; % No arguments makes to fluid give the obj water props

% airfoils - same for the entire rotor so we just need one
af = airfoil(1,'S814');

% blade sections
aspectRatio = 7;
bladeLength = 4*0.0254; % inch*m/inch
numSections = 20;
bs = bladesection(bladeLength/aspectRatio,bladeLength/numSections,af);
for i=1:1:numSections
    section(i) = bs;
end

% blade
bladeMass = 0.02;
numBlades = 3;
twist = -[64.6,49.9,38.7,30.5,24.5,19.95,16.45,13.7,11.5,9.7,8.1,6.9,5.76,4.81,3.98,3.25,2.61,2.03,1.51,1.04];
b = blade(section,bladeMass,twist);
for i=1:1:numBlades
    blade(i) = b; % Note that this is a vector of the same handle
end

% rotor
rotor = rotor(blade);

%% Set-up simulation

tspan = 0:0.1:5;

% Initial states
% State vector
%    1      2     3    4  5  6     7     
% [theta, gamma, beta, x, y, z, theta_dot,
%     8          9      10      11     12
% gamma_dot, beta_dot, x_dot, y_dot, z_dot]
x0 = [90;0;0;0;0;0;0;0;0;0;0;0];
water.velocity = [0.1;0;0];

% Need to set rotor position. This will happen automatically in the state
% file from now on. todo(rodney) fix this when you move ot vehicle.
rotor.position = [x0(4);x0(5);x0(6)];
rotor.orientation = [x0(1);x0(2);x0(3)];
rotor.velocity = [x0(10);x0(11);x0(12)];
rotor.angvel = [x0(7);x0(8);x0(9)];

% Function Handle
fnhndl = @rotorState;

%[t, y] = ode45(@(t,y) rotorState(t,y,rotor,water),tspan,x0);
y = ode4(@(t,y) rotorState(t,y,rotor,water),tspan,x0);
