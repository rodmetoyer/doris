% testMomentCoefficient.m
% script to test the simulation class makeMovie method

clearvars; close all; clc;
addpath('..\src');
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
% Make a vehicle - all points in A frame
vbcentermass = [0;0;0];
vbtetherpoint = [-1.25;0;0];
vbbuoypoint = [0;0;0];
r1point = [0;0;1.25];
r2point = [0;0;-1.25];
rotPoints = [r1point,r2point];
vbmass = 0.8;
vblength = 1.0;
vbradius = 0.1;
I = [1/12*vbmass*(3*vbradius^2+vblength^2),0,0;...
    0,1/12*vbmass*(3*vbradius^2+vblength^2),0;...
    0,0,1/2*vbmass*vbradius^2];
vbod = vehiclebody(vbmass,I);
vbod.setRadius(vbradius);
vbod.setLength(vblength);
v = vehicle;
v.init(vbod,[r1,r2],rotPoints,vbcentermass,vbtetherpoint,vbbuoypoint);
% Associate rotors with vehicle
r1.connectVehicle(v);
r2.connectVehicle(v);
% clear out all the non-object variables so I don't accidentally use them
% and get an erroneous pass.
clear position position1 position2 i r1point r2point rotPoints vbbuoypoint vbtetherpoint vbmass vbtetherpoint vbcentermass

%% Ok, now we have a vehicle with a couple of rotors. Lets set some states and compute some loads
% First give the water some velocity in the x direction
water.velocity = [1;0;0];
% Now orient the vehicle - think of this as pitch-yaw-roll where 0 pitch
% points the axis of rotation of a coaxial vehicle towards the ground
pitch = 90; yaw = 0;
v.orientation = [pitch*pi/180;yaw*pi/180;0];
% Initial posotion of the center of mass in the inertial frame
v.position = [3;0;1];
% Initial velocity of the center of mass in the inertial frame
v.velocity = [0;0;0];
% Initial rotational velocity of the vehicle frame w.r.t. the inertial frame
v.angvel = [0;0;0];

% Now set the orientation of the rotors. From vehicle frame to rotor frame
% For a coaxial system, only the 3rd component will change during sim
v.rotors(1).orientation = [0;0;0];
v.rotors(2).orientation = [0;0;0];

rpm = 0:0.001:0.2;
for i=1:1:length(rpm)
    omg = rpm(i)/30*pi;
    v.angvel = [0;0;omg];
    re(i) = water.getRotationalRe(v.angvel(3),v.body.radius);
    cmc(i) = v.getCylinderMomentCoefficient(water,true);
    Tq(i) = 0.5*water.density*pi*v.angvel(3)^2*v.body.radius^4*v.body.length*cmc(i);
end
figure('Color','none');
plot(rpm,cmc);
xloc = 60*1/(2*pi)*60*water.dynVisc/(water.density*v.body.radius^2);
xline(xloc);
text(xloc,0.45,'Laminar-Turbulent Transition');
xlabel('Rotation Rate (RPM)'); 
%ylabel('Hydrodynamic Torque about Body Axis (N)');
ylabel('C_m_c');
% hfig = gcf;
% hfig.CurrentAxes.Color = 'none';
% plotsavepath = [pwd '\tempfigs'];
% export_fig([plotsavepath '\bodyCmc.png'],'-png','-transparent','-m3');
% figure
% plot(rpm,re);
