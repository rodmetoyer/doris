% Simulation input file for a dual rotor simualtion
runname = 'depBaseline';

% Environment
fluidtype = 'water';
fluidBaseVelocity = [1.5;0.0;0]; % Ocean current
flowtype = 'steady';
flowparms = [];
% ramped - rampspeed(1 to inf), starttime
%flowtype = 'ramped';
%flowparms = [2,5];
% disturbed - rampspeed(1 to inf), starttime, duration, maximum_yvel
%flowtype = 'disturbed';
%flowparms = [5,2,4,0.5];
% sinusoidal - frequency, amplitude, phase
%flowtype = 'sinusoidal';
%flowparms = [0.5,0.5,0];

% Vehicle body
vblength = 12.0;
vbradius = 0.6;
vbmass = 11200.0;
vbnorm = 1.2;
vbax = 0.062;

% BEMT properties may be set for each rotor individually, but in the setup we assume that we want the same choice for both
useBEMT = true;
usePrandtl = true; % Yes, you can do tip loss without BEMT if you want, though I don't know why you would

%% Rotor 1
bladeMass1 = 1000.0; % kg
airfoiltype1 = 'S814';
aspectRatio1 = 10;
bladeLength1 = 10.0;
secChord1 = bladeLength1/aspectRatio1;
numSections1 = 12;       % Number of sections (whole number)
%secWidth1 = bladeLength1/numSections1;
numBlades1 = 3;
bladeDZfrac1 = 0.1; 
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
twist1.AoAopt_deg = 8.0;
twist1.numBlades = numBlades1;
twist1.bladeDZfrac = bladeDZfrac1;
axflowfactor1 = 0.8;
tnflowfactor1 = 1.0;

%% Rotor 2
bladeMass2 = 1000.0; % kg
airfoiltype2 = 'S814';
aspectRatio2 = 10;
bladeLength2 = 10.0;
secChord2 = bladeLength2/aspectRatio2;
numSections2 = 12;       % Number of sections (whole number)
numBlades2 = 3;
%secWidth2 = bladeLength2/numSections2;
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
bladeDZfrac2 = 0.1;
twist2.AoAopt_deg = 8.0;
twist2.numBlades = numBlades2;
twist2.bladeDZfrac = bladeDZfrac2;
axflowfactor2 = 0.4;
tnflowfactor2 = 1.0;

% vehicle body again
I = [1/12*vbmass*(3*vbradius^2+vblength^2),0,0;0,1/12*vbmass*(3*vbradius^2+vblength^2),0;0,0,1/2*vbmass*vbradius^2];
vbcentermass = [0.0;0;0.0*vblength/2]; % This is center mass of the vehicle body
vcentermass = []; % This is center mass of the vehicle - leave empty to compute
vbtetherpoint = [0;0;-vblength/2];
vbbuoypoint = [0;0;0.0*vblength/2]; % Center of buoyancy
vreldensity = 1.0;    % Density of the vehilce body relative to water
rot1point = [0;0;-vblength/2]; % Point where the 1st rotor is located [g1,g2,g3]
rot2point = [0;0;vblength/2]; % Point where the 2nd rotor is located [h1,h2,h3]
rot1rad = bladeLength1;
rot2rad = bladeLength2;
rot1ornt = [0;0;0];
rot2ornt = [0;0;0];
rot1initRPM = 2;
rot2initRPM = -1.5; % sign for directionality
addedMass = [];
rotorVisc = 0.01; % viscous friction coeffiecient between rotors and body
vtMod = 1.0;      % to modify the torsional viscosity constant (cm - cylinder moment coefficient) 
hifiTors = false; % Use the high-fidelity torsion coefficient model - better model for low Re

% tether
tmod = 100.0e9;     % Youngs modulus
tunstrch = 100;    % Tether relaxed length
tradius = 0.05;  % Tether radius in m
tspring = tmod*pi*tradius^2/tunstrch;  % Whole tether spring constant (EA/L)
tdamp = [];
tdampfac = 1.0;
tnnodes = 0;
tnodlocs = [];   % one column per node

% generator
gmconst = 0.19;
gflux = 10;
grarm = 0.1;
gkvisc = 1.0e-8;
gmass = 350.0; % mass in kg
grload = inf; % inf for no load, 0 for closed circuit
gpoint = [0;0;0];

% Simulation
totalSimTime = 600;
tstep = 0.1;

% Initial Conditions
initialYaw = 0*pi/180;
initialPitch = 90*pi/180;
initialRoll = 0*pi/180;
initialLateral = 0;
initialLongitudinal = 106.0;
initialVertical = 0;
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