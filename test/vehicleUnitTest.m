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
opts.verbose = true;
opts.resultsfile = '';
opts.plot = true;
opts.movie = false;
% delete above here
    
    
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
    b(i) = blade(section,bladeMass,twist);
end
clear twist bladeMass AoAopt_deg bladeDZfrac numBlades numSections i

% Make a set of rotors
r1 = rotor(b);
r2 = rotor(b);
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
    position = v.rotors(i).vehicle.rotorLocs(:,i)
end
% Alternatively, since you already know which rotor is first and which is
% second, you can get the locations directly like this
position1 = v.rotorLocs(:,1)
position2 = v.rotorLocs(:,2)

%% Ok, now we have a vehicle with a couple of rotors. Lets set some states and compute some loads
