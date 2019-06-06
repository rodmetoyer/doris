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
% Make a vehicle - all points in A frame
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
pitch = 90; yaw = 0;
v.orientation = [90*pi/180;yaw*pi/180;0];
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
rpm = 0;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];

% I think we have enough information to compute the relative velocities at
% the blade sections and thus the aero loads.
% Make a method in vehicle that computes all aero loads
% Starts by getting forces from the rotors
Urel1_P1 = v.rotors(1).computeHydroLoads(water);
Urel2_P2 = v.rotors(2).computeHydroLoads(water);
% To visulaize the relative velocity vectors and the forces at the sections
% we need to get the positions of the sections in the inertial frame as
% well as the forces from the rotor objects. 
temp = size(v.rotors(1).sectPos);
O_C_A = transpose(v.A_C_O);
O_C_P1 = O_C_A*transpose(v.rotors(1).P_C_A);
O_C_P2 = O_C_A*transpose(v.rotors(2).P_C_A);
for i=1:1:temp(2)
    for j=1:1:temp(3)
        rap1_O(:,i,j) = O_C_P1*v.rotors(1).sectPos(:,i,j);
        rap2_O(:,i,j) = O_C_P2*v.rotors(2).sectPos(:,i,j);
        Urel1_O(:,i,j) = O_C_P1*Urel1_P1(:,i,j);
        Urel2_O(:,i,j) = O_C_P2*Urel2_P2(:,i,j);
    end
end
% Show the forces and/or velocity vectors as quivers applied at the section
% locations. For that we need rp1o_O and rp2o_O
rco_O = v.position;
rp1c_O = O_C_A*v.rotorLocs(:,1);
rp2c_O = O_C_A*v.rotorLocs(:,2);
rp1o_O = rco_O + rp1c_O;
rp2o_O = rco_O + rp2c_O;
%% First check is to plot the relative velocity vectors in the O frame
% No motion of the rotors or body means that we should see relative
% velocity vectors that are exactly the freestream velocity.
figure
scale = 0;
quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),scale,'b');
hold on
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c')
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g')
axis equal
xlabel('x'); ylabel('y');
title('Velocity Vectors');
legend('Freestream Velocity','Relative Velocity','Location','East');
view(-123,22)
hold off