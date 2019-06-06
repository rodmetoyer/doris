%function pf = vehicleUnitTest(opts)
% Vehicle unit test.
% INPUTS:
   % opts = struct. containing options
        % opts.verbose = true
        % opts.resultsfile = 'airfoilUnitTest_results.txt'
        % opts.plots = false
        % opts.movie = false
% OUTPUTS
    % pf = pass/fail bool
    
%% delete when convert to function
clearvars; close all; clc;
addpath('..\src');
opts.verbose = true;
opts.resultsfile = '';
opts.plot = true;
opts.movie = false;
% delete above here
    
% Don't need to see these warnings. We know they are going to happen.
warning('off','BLADE:construction');
warning('off','VEHICLE:construction');

% Build a vehicle operating in a fluid
water = fluid('water');
af = airfoil('SG6040');
bladeChord = 0.1; % Meters
sectionWidth  = 0.1; % Meters - making blade one meter long 10 sections
numSections = 10;
bs = bladesection(bladeChord,sectionWidth,af);
clear bladeChord sectionWidth

% Make a blade comprised of the same section.
bladeMass = 0.25;
AoAopt_deg = 8.0;
bladeDZfrac = 0.0;
numBlades = 3;
twist.AoAopt_deg = AoAopt_deg;
twist.numBlades = numBlades;
twist.bladeDZfrac = bladeDZfrac;
for i=1:1:numSections
    section(i) = bs;
end
for i=1:1:numBlades
    b1(i) = blade(section,bladeMass,twist);
end
warning('off','BLADE:construction');
for i=1:1:numBlades
    b2(i) = blade(section,bladeMass,twist);
    b2(i).reverseTwist;
end
clear twist bladeMass AoAopt_deg bladeDZfrac numBlades numSections i

% Make a set of rotors
r1 = rotor(b1);
r1.setID(1);
% Need to make rotor 2 have blades with twist 180-
r2 = rotor(b2);
r2.setID(2);
% Make a vehicle
vbcentermass = [0;0;0];
vbtetherpoint = [-0.01;0;0];
vbbuoypoint = [0;0;0];
r1point = [0;0;0.65];
r2point = [0;0;-0.65];
rotPoints = [r1point,r2point];
vbmass = 0.8;
vbod = vehiclebody(vbmass);
v = vehicle;
v.init(vbod,[r1,r2],rotPoints,vbcentermass,vbtetherpoint,vbbuoypoint);
% Associate rotors with vehicle
r1.connectVehicle(v);
r2.connectVehicle(v);

% This is how you would get the rotor position in the vehicle from the
% vehicle object
for i=1:1:numel(v.rotors)
    position(:,i) = v.rotors(i).vehicle.rotorLocs(:,i);
end
if norm(r1point-position(:,1)) > 0 || norm(r2point-position(:,2))
    warning('Rotor positions do not match');
    pf = false;
    return
end

% Alternatively, since you already know which rotor is first and which is
% second, you can get the locations directly like this
position1 = v.rotorLocs(:,1);
position2 = v.rotorLocs(:,2);
% or like this
position1 = v.rotorLocs(:,v.rotors(1).ID);
position2 = v.rotorLocs(:,v.rotors(2).ID);

% clear out all the non-object variables so I don't accidentally use them
% and get an erroneous pass.
clear position position1 position2 i r1point r2point rotPoints vbbuoypoint vbtetherpoint vbmass vbtetherpoint vbcentermass

%% Ok, now we have a vehicle with a couple of rotors. Lets set some states and compute some loads
% First give the water some velocity in the x direction
water.velocity = [1;0;0];
% Now orient the vehicle - think of this as pitch-yaw-roll where 0 pitch
% points the axis of rotation of a coaxial vehicle towards the ground
v.orientation = [90*pi/180;15*pi/180;0];
% Initial posotion of the center of mass in the inertial frame
v.position = [1;0;1];
% Initial velocity of the center of mass in the inertial frame
v.velocity = [0;0;0];
% Initial rotational velocity of the vehicle frame w.r.t. the inertial frame
v.angvel = [0;0;0];

% Now set the orientation of the rotors. From vehicle frame to rotor frame
% For a coaxial system, only the 3rd component will change during sim
v.rotors(1).orientation = [0;0;0];
v.rotors(2).orientation = [0;0;0];

% Set the angular velocity of the rotors. All 3rd for coax.
rpm = 60;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];

% I think we have enough information to compute the relative velocities at
% the blade sections and thus the aero loads.
% Make a method in vehicle that computes all aero loads
% Starts by getting forces from the rotors
Urel_P1 = v.rotors(1).computeHydroLoads(water);
Urel_P2 = v.rotors(2).computeHydroLoads(water);
% To visulaize the relative velocity vectors and the forces at teh sections
% we need to get the positions of the sections in the inertial frame.

% Now get the forces from the rotor objects and show the forces and/or
% velocity vectors as quivers applied at the section locations.

