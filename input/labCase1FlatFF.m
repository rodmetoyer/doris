% Simulation input file for a dual rotor simualtion
runname = 'labCase1FlatFF';

% Environment
fluidtype = 'air';
fluidBaseVelocity = [6.9;0.0;0]; % Approximately river velocity
%flowtype = 'steady';
%flowparms = [];
% ramped - rampspeed(1 to inf), starttime
flowtype = 'ramped';
flowparms = [5,1];
% disturbed - rampspeed(1 to inf), starttime, duration, maximum_yvel
%flowtype = 'disturbed';
%flowparms = [5,2,4,0.5];
% sinusoidal - frequency, amplitude, phase
%flowtype = 'sinusoidal';
%flowparms = [0.5,0.5,0];

%% Rotor 1
bladeMass1 = 0.005; % kg
airfoiltype1 = 'FLAT';
aspectRatio1 = 7;
bladeLength1 = 0.08;
secChord1 = bladeLength1/aspectRatio1;
numSections1 = 8;       % Number of sections (whole number)
%secWidth1 = bladeLength1/numSections1;
numBlades1 = 2;
bladeDZfrac1 = 0.05; 
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
twist1.AoAopt_deg = 6.0;
twist1.numBlades = numBlades1;
twist1.bladeDZfrac = bladeDZfrac1;
axflowfactor1 = 0.8;

%% Rotor 2
bladeMass2 = 0.005; % kg
airfoiltype2 = 'FLAT';
aspectRatio2 = 7;
bladeLength2 = 0.08;
secChord2 = bladeLength2/aspectRatio2;
numSections2 = 8;       % Number of sections (whole number)
%secWidth2 = bladeLength2/numSections2;
numBlades2 = 2;
bladeDZfrac2 = 0.05; 
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
twist2.AoAopt_deg = 6.0;
twist2.numBlades = numBlades2;
twist2.bladeDZfrac = bladeDZfrac2;
axflowfactor2 = 0.4;

% vehicle
vblength = 0.1;
vbradius = 0.025;
vbmass = 0.025;
I = [1/12*vbmass*(3*vbradius^2+vblength^2),0,0;0,1/12*vbmass*(3*vbradius^2+vblength^2),0;0,0,1/2*vbmass*vbradius^2];
vbcentermass = [0;0;0]; % This is center mass of the vehicle body
vcentermass = []; % This is center mass of the vehicle - leave empty to compute
vbtetherpoint = [0;0;-vblength/2];
vbbuoypoint = [0;0;0.0]; % Center of buoyancy
vbnorm = 1.2;
vbax = 0.1;
vreldensity = 8000.0;    % Density of the vehilce body relative to the fluid
rot1point = [0;0;-vblength/8]; % Point where the 1st rotor is located [g1,g2,g3]
rot2point = [0;0;vblength/8]; % Point where the 2nd rotor is located [h1,h2,h3]
rot1rad = bladeLength1;
rot2rad = bladeLength2;
rot1ornt = [0;0;0];
rot2ornt = [0;0;0];
rot1initRPM = 0;
rot2initRPM = 0; % sign for directionality
addedMass = [];
rotorVisc = 0.0; % Viscosity between rotor and body - 0.02 is reasonable for Re > 100
vtMod = 100;
hifiTors = false;

% tether
tspring = 10000;
tdamp = [];
tdampfac = 1.0;
tunstrch = 0.3;
tnnodes = 0;
tnodlocs = []; % one column per node

% generator
gmconst = 0.19;
gflux = 0;
grarm = inf;
gkvisc = 0;
gmass = 0; % mass in kg
grload = inf; % inf for no load, 0 for closed circuit
gpoint = [0;0;0];

% Simulation
totalSimTime = 10;
tstep = 0.01;

% Initial Conditions
initialYaw = 0*pi/180;
initialPitch = 180*pi/180;
initialRoll = 0*pi/180;
initialLateral = 0;
initialLongitudinal = 0;
initialVertical = -0.35;
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