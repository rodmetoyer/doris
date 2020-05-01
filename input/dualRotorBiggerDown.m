%% Simulation input file for a dual rotor simualtion
runname = 'dualRotorBaseline';

%% Environment
fluidtype = 'water';
fluidVelocity = [0.5;0.0;0];

%% Rotor 1
bladeMass1 = 0.5;
airfoiltype1 = 'SG6040';
aspectRatio1 = 7;
bladeLength1 = 1.0; %12*0.0254; % Inches times meters per inch
secChord1 = bladeLength1/aspectRatio1;
numSections1 = 18;       % Number of sections (whole number)
secWidth1 = bladeLength1/numSections1;
numBlades1 = 3;
bladeDZfrac1 = 0.0; % Blade dead zone fraction (Ro/R - see Spera 2009) %%%NOTE%%% Only used to compute twist right now. Forces are computed the entire length of the blade.
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
twist1.AoAopt_deg = 8.0;
twist1.numBlades = numBlades1;
twist1.bladeDZfrac = bladeDZfrac1;

%% Rotor 2
bladeMass2 = 0.5;
airfoiltype2 = 'SG6040';
aspectRatio2 = 7;
bladeLength2 = 2.0; %12*0.0254; % Inches times meters per inch
secChord2 = bladeLength2/aspectRatio2;
numSections2 = 18;       % Number of sections (whole number)
secWidth2 = bladeLength2/numSections2;
numBlades2 = 3;
bladeDZfrac2 = 0.0; % Blade dead zone fraction (Ro/R - see Spera 2009) %%%NOTE%%% Only used to compute twist right now. Forces are computed the entire length of the blade.
% twist = []; % To prescribe a twist make a 1 X numSections array, otherwise use the struct format and twist will be computed.
twist2.AoAopt_deg = 8.0;
twist2.numBlades = numBlades2;
twist2.bladeDZfrac = bladeDZfrac2;

%% vehicle body
vblength = 1.0;
vbradius = 0.1;
vbmass = 5.0;
I = [1/12*vbmass*(3*vbradius^2+vblength^2),0,0;0,1/12*vbmass*(3*vbradius^2+vblength^2),0;0,0,1/2*vbmass*vbradius^2];
vbcentermass = [0;0;0];
vbtetherpoint = [0;0;-0.5];
vbbuoypoint = [0;0;0.0]; % Center of buoyancy
vbreldensity = 1.0;    % Density of the vehilce body relative to water
rot1point = [0;0;-0.5]; % Point where the 1st rotor is located [g1,g2,g3]
rot2point = [0;0;0.5]; % Point where the 2nd rotor is located [h1,h2,h3]
rot1rad = bladeLength1;
rot2rad = bladeLength2;
rot1ornt = [0;0;0];
rot2ornt = [0;0;0];
rot1initRPM = 0;
rot2initRPM = 0; % sign for directionality

%% tether
tspring = 100;
tdamp = 25;
tunstrch = 1;
tnnodes = 0;
tnodlocs = []; % one column per node

%% generator
gmconst = 0.19;
gflux = 1;
grarm = 0.1;
gkvisc = 1.0e-4;
gmass = 0;

%% Simulation
totalSimTime = 10;
tstep = 0.01;

%% Initial Conditions
initialYaw = 0*pi/180;
initialPitch = 90*pi/180;
initialRoll = 0*pi/180;
initialLateral = 0;
initialLongitudinal = 2;
initialVertical = 1;
initialSway = 0;
initialSurge = 0;
initialHeave = 0;
initialYawRate = 0;
initialPitchRate = 0;
initialRollRate = 0;

%% Products
makeplots = true;
makemovie = true;
speedfactor = 1; % How much to slow the video by. e.g. 1/8;