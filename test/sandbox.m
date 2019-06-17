% sandbox.m
% This is just a sandbox

clearvars; close all; clc;
addpath('..\src');


saveplots = true;
plotsavepath = [pwd '\tempfigs'];
% delete above here

if saveplots
    if ~exist(plotsavepath,'dir')
        mkdir(plotsavepath)
    end
end

% Make a rotor in a fluid
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
    b(i) = blade(section,bladeMass,twist);
    b2(i) = blade(section,bladeMass,twist);
end
clear twist bladeMass AoAopt_deg bladeDZfrac numBlades numSections i
% Make a set of rotors
r = rotor(b);
r.setID(1);
% Does the rotor visualization method work?
r.orientation = [0;0;0]; % Rotor orientation is w.r.t. the vehicle frame
rpm = 0;
r.angvel = [0;0;rpm/60*2*pi]; % Also w.r.t. the vehicle frame
% Need a vehicle to relate the rotor frame to inertial frame
vb = vehiclebody;
v = vehicle;
v.init(vb,r,[0;0;0],[0;0;0],[0;0;0],[0;0;0]);
v.orientation = [pi/2;0;0];
v.velocity = [0;0;0];
v.angvel = [0/60*2*pi;0/60*2*pi;60/60*2*pi];
r.connectVehicle(v);
water.velocity = [0.1;0;0];
v.computeHydroLoads(water);
r.visualizeSectionLoads;
% I'd like to visualize the rotor on a body
% init(hobj,bod,rot,rotLocs,cm,tp,bp)
b2.reverseTwist;
r2 = rotor(b2);
r2.setID(2);
r2.orientation = [0;0;0]; % Rotor orientation is w.r.t. the vehicle frame
vRPM = [0;0;0];
v.angvel = vRPM/60*2*pi;
%v.angvel = [0;1;0];
rpm = 10;
rpm2 = -10;
r.angvel = [0;0;rpm/60*2*pi];
r2.angvel = [0;0;rpm2/60*2*pi]; % Also w.r.t. the vehicle frame
r2.connectVehicle(v);
v.init(vb,[r r2],[0 0;0 0;1 -1],[0;0;0],[0;0;0],[0;0;0]);
v.computeHydroLoads(water);
v.visualizeSectionLoads(false,0.7);
v.visualizeRelativeVelocities(water,false,0);

%% Let's try to visualize all the frames in the vehicle
v.visualizeSectionFrames;
