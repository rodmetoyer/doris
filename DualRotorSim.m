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
% Make a vehicle
rotPoints = [rot1point,rot2point];
v = vehicle;
v.init(vbod,[r1,r2],rotPoints,vbcentermass,vbtetherpoint,vbbuoypoint);
% Associate rotor objects with vehicle object
r1.connectVehicle(v);
r2.connectVehicle(v);

%% Make sure the vehicle we just built is what we were trying to build.
% The easiest way to do this is visually.
water.velocity = [0.5;0;0];
pitch = 90; yaw = 0;
v.orientation = [pitch*pi/180;yaw*pi/180;0];
v.position = [0;0;1];
v.velocity = [0;0;0];
v.angvel = [0;0;0];
v.rotors(1).orientation = [0;0;0];
v.rotors(2).orientation = [0;0;0];
rpm = 60;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];
vehicleVisualCheck(water,v)


%% Set-up simulation
tspan = 0:tstep:totalSimTime;

% Initial states
% State vector