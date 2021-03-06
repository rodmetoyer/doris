% Simulation input file for a dual rotor simualtion
runname = 'sinusoidDemo';

% Environment
fluidtype = 'water';
fluidBaseVelocity = [1.6;0.0;0]; % Approximately river velocity
%flowtype = 'steady';
%flowparms = [];
% ramped - rampspeed(1 to inf), starttime
%flowtype = 'ramped';
%flowparms = [2,4];
% disturbed - rampspeed(1 to inf), starttime, duration, maximum_yvel
%flowtype = 'disturbed';
%flowparms = [5,5,3,1.0];
% sinusoidal - amplitude, frequency, phase
flowtype = 'sinusoidal';
flowparms = [0.5,0.5,0];

% Vehicle body
vblength = 1.0;
vbradius = 0.05;
vbmass = 15.5;
vbnorm = 1.2;
vbax = 0.1;

%% Rotor 1
bladeMass1 = 0.43; % kg
airfoiltype1 = 'NACA0015';
aspectRatio1 = 10;
bladeLength1 = 0.5;
secChord1 = bladeLength1/aspectRatio1;
numSections1 = 10;       % Number of sections (whole number)
%secWidth1 = bladeLength1/numSections1;
numBlades1 = 3;
bladeDZfrac1 = 0.1; 
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
twist1.AoAopt_deg = 8.0;
twist1.numBlades = numBlades1;
twist1.bladeDZfrac = bladeDZfrac1;
axflowfactor1 = 1.0;
tnflowfactor1 = 1.0;

%% Rotor 2
bladeMass2 = 0.3225; % kg
airfoiltype2 = 'NACA0015';
aspectRatio2 = 10;
bladeLength2 = 0.5;
secChord2 = bladeLength2/aspectRatio2;
numSections2 = 10;       % Number of sections (whole number)
secWidth2 = bladeLength2/numSections2;
numBlades2 = 4;
%secWidth2 = bladeLength2/numSections2;
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
bladeDZfrac2 = 0.1;
twist2.AoAopt_deg = 8.0;
twist2.numBlades = numBlades2;
twist2.bladeDZfrac = bladeDZfrac2;
axflowfactor2 = 1.0;
tnflowfactor2 = 1.0;

% vehicle body again
I = [1/12*vbmass*(3*vbradius^2+vblength^2),0,0;0,1/12*vbmass*(3*vbradius^2+vblength^2),0;0,0,1/2*vbmass*vbradius^2];
vbcentermass = [0.0;0;0.0*vblength/2]; % This is center mass of the vehicle body
vcentermass = []; % This is center mass of the vehicle - leave empty to compute
vbtetherpoint = [0;0;-vblength/2];
vbbuoypoint = [0;0;-0.0*vblength/2]; % Center of buoyancy
vreldensity = 1.0;    % Density of the vehilce body relative to water
rot1point = [0;0;-vblength/2]; % Point where the 1st rotor is located [g1,g2,g3]
rot2point = [0;0;vblength/2]; % Point where the 2nd rotor is located [h1,h2,h3]
rot1rad = bladeLength1;
rot2rad = bladeLength2;
rot1ornt = [0;0;0];
rot2ornt = [0;0;0];
rot1initRPM = 0;
rot2initRPM = 0; % sign for directionality
addedMass = [];
rotorVisc = 0.0; % viscous friction coeffiecient between rotors and body
vtMod = 1.0;      % to modify the torsional viscosity constant (cm - cylinder moment coefficient) 
hifiTors = false; % Use the high-fidelity torsion coefficient model - better model for low Re

% tether
tmod = 1.0e9;     % Youngs modulus
tunstrch = 10;    % Tether relaxed length
tradius = 0.007;  % Tether radius in mm
tspring = tmod*pi*tradius^2/tunstrch;  % Whole tether spring constant (EA/L)
tdamp = [];
tdampfac = 1.0;
tnnodes = 0;
tnodlocs = [];   % one column per node

% generator
gmconst = 0.19;  % motor/generator constant
gflux = 10;      % Flux in Wb
grarm = 0.1;     % Armature Resistance in Ohms
gkvisc = 1.0e-1; % Additional resistance modeled as viscous friction
gmass = 3.0;     % mass in kg2;
grload = inf;    % inf for no load, 0 for closed circuitmass in kg
gpoint = [0;0;0];% Where it is located in the body (adjusts center of mass) inf for no load, 0 for closed circuit

% Simulation
totalSimTime = 20;
tstep = 0.01;

% Initial Conditions
initialYaw = 0*pi/180;
initialPitch = 90*pi/180;
initialRoll = 0*pi/180;
initialLateral = 0;
initialLongitudinal = 10.5;
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