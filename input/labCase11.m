% Simulation input file for a dual rotor simualtion
runname = 'labCase11';

% Environment
fluidtype = 'air';
fluidBaseVelocity = [3.5;0.0;0]; % Approximately river velocity
%flowtype = 'steady';
%flowparms = [];
% ramped - rampspeed(1 to inf), starttime
flowtype = 'ramped';
flowparms = [2,2.5];
% disturbed - rampspeed(1 to inf), starttime, duration, maximum_yvel
%flowtype = 'disturbed';
%flowparms = [5,2,4,0.5];
% sinusoidal - frequency, amplitude, phase
%flowtype = 'sinusoidal';
%flowparms = [0.5,0.5,0];

%% Rotor 1
bladeMass1 = 0.02; % kg
airfoiltype1 = 'SG6040';
aspectRatio1 = 7;
bladeLength1 = 0.1016;
secChord1 = bladeLength1/aspectRatio1;
numSections1 = 6;       % Number of sections (whole number)
secWidth1 = bladeLength1/numSections1;
numBlades1 = 2;
bladeDZfrac1 = 0.0; 
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
twist1.AoAopt_deg = 8.0;
twist1.numBlades = numBlades1;
twist1.bladeDZfrac = bladeDZfrac1;

%% Rotor 2
bladeMass2 = 0.02; % kg
airfoiltype2 = 'SG6040';
aspectRatio2 = 7;
bladeLength2 = 0.1016;
secChord2 = bladeLength2/aspectRatio2;
numSections2 = 6;       % Number of sections (whole number)
secWidth2 = bladeLength2/numSections2;
numBlades2 = 2;
bladeDZfrac2 = 0.0; 
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
twist2.AoAopt_deg = 8.0;
twist2.numBlades = numBlades2;
twist2.bladeDZfrac = bladeDZfrac2;

% vehicle
vblength = 0.1016;
vbradius = 0.075;
vbmass = 0.1;
I = [1/12*vbmass*(3*vbradius^2+vblength^2),0,0;0,1/12*vbmass*(3*vbradius^2+vblength^2),0;0,0,1/2*vbmass*vbradius^2];
vbcentermass = [0.01;0;0]; % This is center mass of the vehicle body
vcentermass = []; % This is center mass of the vehicle - leave empty to compute
vbtetherpoint = [0;0;-vblength/2];
vbbuoypoint = [0;0;0.0]; % Center of buoyancy
vreldensity = 1000.0;    % Density of the vehilce body relative to water
rot1point = [0;0;-vblength/2]; % Point where the 1st rotor is located [g1,g2,g3]
rot2point = [0;0;vblength/2]; % Point where the 2nd rotor is located [h1,h2,h3]
rot1rad = bladeLength1;
rot2rad = bladeLength2;
rot1ornt = [0;0;0];
rot2ornt = [0;0;0];
rot1initRPM = 0;
rot2initRPM = 0; % sign for directionality

% tether
tspring = 1200;
tdamp = 300;
tunstrch = 1;
tnnodes = 0;
tnodlocs = []; % one column per node

% generator
gmconst = 0.19;
gflux = 2;
grarm = 0.1;
gkvisc = 1.0e-12;
gmass = 0.02; % mass in kg
grload = inf; % inf for no load, 0 for closed circuit
gpoint = [0;0;0];

% Simulation
totalSimTime = 10;
tstep = 0.01;

% Initial Conditions
initialYaw = 0*pi/180;
initialPitch = 170*pi/180;
initialRoll = 0*pi/180;
initialLateral = 0;
initialLongitudinal = 0;
initialVertical = -1.05;
initialSway = 0;
initialSurge = 0;
initialHeave = 0;
initialYawRate = 0;
initialPitchRate = 0;
initialRollRate = 0;

% Products
makeplots = true;
makemovie = true;
speedfactor = 1; % How much to slow the video by. e.g. 1/8;