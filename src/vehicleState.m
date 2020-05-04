function xdot = vehicleState(t,x,v,f,thr)
% State derivative function for the vehicle prototype code
% INPUTS:
    % t = time (s)
    % x = state vector
    % v = vehicle object
    % f = fluid object representing the freestream
    % thr = tether object

xdot = NaN(size(x));    
%% Get useful values
% todo(rodney) make sure we're only using resources when necessary. Mass
% never changes and is not computed on demand so there is no need to save
% it to a local variable.
mt = v.mass; % Total mass. Mass of just body would be v.body.mass
%g = 9.81; % todo this needs to come from an environment object
cosbeta = cos(x(6)); sinbeta = sin(x(6));
cosgamma = cos(x(5)); singamma = sin(x(5));
costheta = cos(x(4)); sintheta = sin(x(4));
% rotor angles
fi1 = v.rotors(1).orientation(1);
fi2 = v.rotors(1).orientation(2);
sy1 = v.rotors(2).orientation(1);
sy2 = v.rotors(2).orientation(2);
cosfi1 = cos(fi1); sinfi1 = sin(fi1);
cossy1 = cos(sy1); sinsy1 = sin(sy1);
cosfi2 = cos(fi2); sinfi2 = sin(fi2);
cossy2 = cos(sy2); sinsy2 = sin(sy2);
fi3 = x(14); sy3 = x(16);
cosfi3 = cos(fi3); sinfi3 = sin(fi3);
cossy3 = cos(sy3); sinsy3 = sin(sy3);

%% Modify fluid as a function of time

%% Update object states from current state vector
v.position = [x(1);x(2);x(3)];   % Expressed in O frame
v.orientation = [x(4);x(5);x(6)]; % theta, gamma, beta
v.angvel = [x(7);x(8);x(9)];
v.velocity = [x(10);x(11);x(12)];
v.rotors(1).angvel(3) = x(13);
v.rotors(1).orientation(3) = x(14);
v.rotors(2).angvel(3) = x(15);
v.rotors(2).orientation(3) = x(16);

%% Update fluid velocity 
f.updateVelocity(t);

%% Compute hydrodynamic loads
v.computeHydroLoads(f);

%% Compute tether loads and add them to the vehicle loads
% For now, single-element elastic tether fixed at the origin of length 1m
% todo(rodney) make a tether class. Number nodes = number links-1. End
% nodes are part of the signal. Output is loads on end links and position
% of internal nodes
if thr.numnodes > 0
    error('oops - no tethers allowed (yet)');
else % no internal nodes
    O_C_A = transpose(v.A_C_O);
    r_to_O = [x(1); x(2); x(3)] + O_C_A*v.tetherpoint;
    Ov_to_O = O_C_A*v.velocity + O_C_A*cross(v.angvel,v.tetherpoint); % CAUTION assumes one tether. todo(rodney) generalize14
    tetherforce = thr.computeTension(r_to_O,Ov_to_O);
end
v.addTetherLoads(v.A_C_O*tetherforce);

%% Compute weight and buoyancy loads
buoyForceA = transpose(O_C_A)*[0;0;(1/v.body.relDensity)*f.gravity*v.mass];
weightA = transpose(O_C_A)*[0;0;-v.mass*f.gravity];
buoyTorqueA = cross(v.buoypoint,buoyForceA);
weightTorqueA = cross(v.syscm,weightA);
v.force = v.force + buoyForceA + weightA;
v.torque = v.torque + buoyTorqueA + weightTorqueA;

%% prepare to compute state derivatives
ta1 = v.torque(1); ta2 = v.torque(2); ta3 = v.torque(3);
f1 = v.force(1); f2 = v.force(2); f3 = v.force(3);
m1 = v.rotors(1).mass; m2 = v.rotors(2).mass; mv = v.body.mass;
c1 = v.centermass(1); c2 = v.centermass(2);
c3 = v.centermass(3);  % Only this will be non-zero for a coaxial turbine
g1 = v.rotorLocs(1,1); g2 = v.rotorLocs(2,1);
g3 = v.rotorLocs(3,1); % Only this will be non-zero for a coaxial turbine
h1 = v.rotorLocs(1,2); h2 = v.rotorLocs(2,2);
h3 = v.rotorLocs(3,2); % Only this will be non-zero for a coaxial turbine
% Position from point A to CM of system
s1 = v.syscm(1); s2 = v.syscm(2); s3 = v.syscm(3);
Icv11 = v.body.inertia(1,1);
Icv22 = v.body.inertia(2,2); 
Icv33 = v.body.inertia(3,3);
Ip11 = v.rotors(1).inertia(1,1);
Ip22 = v.rotors(1).inertia(2,2);
Ip33 = v.rotors(1).inertia(3,3);
Iq11 = v.rotors(2).inertia(1,1);
Iq22 = v.rotors(2).inertia(2,2);
Iq33 = v.rotors(2).inertia(3,3);
tp3 = v.rotors(1).torqueCM(3);
tq3 = v.rotors(2).torqueCM(3);

%% Compute state derivatives
tauMinusCvec = NaN(8,1);
tauMinusCvec(1) =  f1 + (x(8)*(s1*x(8) - s2*x(7)) + x(9)*(s1*x(9) - s3*x(7)))*mt + (x(11)*x(9) - x(12)*x(8))*mt;
tauMinusCvec(2) =  f2 - (x(7)*(s1*x(8) - s2*x(7)) - x(9)*(s2*x(9) - s3*x(8)))*mt - (x(10)*x(9) - x(12)*x(7))*mt;
tauMinusCvec(3) =  f3 - (x(7)*(s1*x(9) - s3*x(7)) + x(8)*(s2*x(9) - s3*x(8)))*mt + (x(10)*x(8) - x(11)*x(7))*mt;
tauMinusCvec(4) =  ta1 - (x(13)*x(7)*cosfi1*cosfi2 - x(13)*x(9)*cosfi1*sinfi2)*(Ip33*cosfi1*sinfi1*sinfi2 + Ip22*cosfi1*cosfi3*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip11*cosfi1*sinfi3*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)) + (x(15)*x(9)*cossy1*sinsy2 - x(15)*x(7)*cossy1*cossy2)*(Iq33*cossy1*sinsy1*sinsy2 + Iq22*cossy1*cossy3*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq11*cossy1*sinsy3*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)) - m1*(g2*(x(7)*(g1*x(9) - g3*x(7)) + x(8)*(g2*x(9) - g3*x(8))) - g3*(x(7)*(g1*x(8) - g2*x(7)) - x(9)*(g2*x(9) - g3*x(8)))) - (x(15)*x(7)*sinsy1 + x(15)*x(8)*cossy1*sinsy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3) + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq33*cossy1^2*cossy2*sinsy2) - m2*(h2*(x(7)*(h1*x(9) - h3*x(7)) + x(8)*(h2*x(9) - h3*x(8))) - h3*(x(7)*(h1*x(8) - h2*x(7)) - x(9)*(h2*x(9) - h3*x(8)))) - (x(9) + x(13)*cosfi1*cosfi2)*((x(9) + x(13)*cosfi1*cosfi2)*(Ip33*cosfi1*cosfi2*sinfi1 - Ip22*cosfi1*cosfi3*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1) + Ip11*cosfi1*sinfi3*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)) - (x(8) - x(13)*sinfi1)*(Ip33*sinfi1^2 + Ip11*cosfi1^2*sinfi3^2 + Ip22*cosfi1^2*cosfi3^2) + (x(7) + x(13)*cosfi1*sinfi2)*(Ip33*cosfi1*sinfi1*sinfi2 + Ip22*cosfi1*cosfi3*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip11*cosfi1*sinfi3*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3))) - mv*(x(8)*(c1*x(11) - c2*x(10)) + x(9)*(c1*x(12) - c3*x(10))) - (x(15)*x(9)*sinsy1 + x(15)*x(8)*cossy1*cossy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)^2 + Iq22*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2)^2 + Iq33*cossy1^2*sinsy2^2) - m1*(x(8)*(g1*x(11) - g2*x(10)) + x(9)*(g1*x(12) - g3*x(10))) - m2*(x(8)*(h1*x(11) - h2*x(10)) + x(9)*(h1*x(12) - h3*x(10))) - (x(13)*x(9)*sinfi1 + x(13)*x(8)*cosfi1*cosfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)^2 + Ip22*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2)^2 + Ip33*cosfi1^2*sinfi2^2) - (x(9) + x(15)*cossy1*cossy2)*((x(9) + x(15)*cossy1*cossy2)*(Iq33*cossy1*cossy2*sinsy1 - Iq22*cossy1*cossy3*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1) + Iq11*cossy1*sinsy3*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3)) + (x(7) + x(15)*cossy1*sinsy2)*(Iq33*cossy1*sinsy1*sinsy2 + Iq22*cossy1*cossy3*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq11*cossy1*sinsy3*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)) - (x(8) - x(15)*sinsy1)*(Iq33*sinsy1^2 + Iq22*cossy1^2*cossy3^2 + Iq11*cossy1^2*sinsy3^2)) + (x(8) - x(15)*sinsy1)*((x(7) + x(15)*cossy1*sinsy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3) + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq33*cossy1^2*cossy2*sinsy2) - (x(9) + x(15)*cossy1*cossy2)*(Iq11*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3)^2 + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)^2 + Iq33*cossy1^2*cossy2^2) + (x(8) - x(15)*sinsy1)*(Iq33*cossy1*cossy2*sinsy1 - Iq22*cossy1*cossy3*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1) + Iq11*cossy1*sinsy3*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3))) - x(9)*(c1*c2*mv*x(7) - x(8)*(Icv22 + c1^2*mv + c3^2*mv) + c2*c3*mv*x(9)) + x(8)*(c1*c3*mv*x(7) - x(9)*(Icv33 + c1^2*mv + c2^2*mv) + c2*c3*mv*x(8)) + (x(8) - x(13)*sinfi1)*((x(8) - x(13)*sinfi1)*(Ip33*cosfi1*cosfi2*sinfi1 - Ip22*cosfi1*cosfi3*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1) + Ip11*cosfi1*sinfi3*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)) - (x(9) + x(13)*cosfi1*cosfi2)*(Ip11*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)^2 + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)^2 + Ip33*cosfi1^2*cosfi2^2) + (x(7) + x(13)*cosfi1*sinfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3) + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip33*cosfi1^2*cosfi2*sinfi2)) - (x(13)*x(7)*sinfi1 + x(13)*x(8)*cosfi1*sinfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3) + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip33*cosfi1^2*cosfi2*sinfi2);
tauMinusCvec(5) =  ta2 + m1*(g1*(x(7)*(g1*x(9) - g3*x(7)) + x(8)*(g2*x(9) - g3*x(8))) + g3*(x(8)*(g1*x(8) - g2*x(7)) + x(9)*(g1*x(9) - g3*x(7)))) - (x(15)*x(9)*cossy1*sinsy2 - x(15)*x(7)*cossy1*cossy2)*(Iq33*sinsy1^2 + Iq22*cossy1^2*cossy3^2 + Iq11*cossy1^2*sinsy3^2) + m2*(h1*(x(7)*(h1*x(9) - h3*x(7)) + x(8)*(h2*x(9) - h3*x(8))) + h3*(x(8)*(h1*x(8) - h2*x(7)) + x(9)*(h1*x(9) - h3*x(7)))) - (x(13)*x(7)*sinfi1 + x(13)*x(8)*cosfi1*sinfi2)*(Ip33*cosfi1*cosfi2*sinfi1 - Ip22*cosfi1*cosfi3*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1) + Ip11*cosfi1*sinfi3*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)) + (x(13)*x(9)*sinfi1 + x(13)*x(8)*cosfi1*cosfi2)*(Ip33*cosfi1*sinfi1*sinfi2 + Ip22*cosfi1*cosfi3*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip11*cosfi1*sinfi3*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)) - (x(15)*x(7)*sinsy1 + x(15)*x(8)*cossy1*sinsy2)*(Iq33*cossy1*cossy2*sinsy1 - Iq22*cossy1*cossy3*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1) + Iq11*cossy1*sinsy3*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3)) + (x(15)*x(9)*sinsy1 + x(15)*x(8)*cossy1*cossy2)*(Iq33*cossy1*sinsy1*sinsy2 + Iq22*cossy1*cossy3*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq11*cossy1*sinsy3*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)) + mv*(x(7)*(c1*x(11) - c2*x(10)) - x(9)*(c2*x(12) - c3*x(11))) + m1*(x(7)*(g1*x(11) - g2*x(10)) - x(9)*(g2*x(12) - g3*x(11))) + m2*(x(7)*(h1*x(11) - h2*x(10)) - x(9)*(h2*x(12) - h3*x(11))) - (x(7) + x(13)*cosfi1*sinfi2)*((x(8) - x(13)*sinfi1)*(Ip33*cosfi1*cosfi2*sinfi1 - Ip22*cosfi1*cosfi3*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1) + Ip11*cosfi1*sinfi3*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)) - (x(9) + x(13)*cosfi1*cosfi2)*(Ip11*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)^2 + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)^2 + Ip33*cosfi1^2*cosfi2^2) + (x(7) + x(13)*cosfi1*sinfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3) + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip33*cosfi1^2*cosfi2*sinfi2)) - (x(7) + x(15)*cossy1*sinsy2)*((x(7) + x(15)*cossy1*sinsy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3) + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq33*cossy1^2*cossy2*sinsy2) - (x(9) + x(15)*cossy1*cossy2)*(Iq11*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3)^2 + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)^2 + Iq33*cossy1^2*cossy2^2) + (x(8) - x(15)*sinsy1)*(Iq33*cossy1*cossy2*sinsy1 - Iq22*cossy1*cossy3*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1) + Iq11*cossy1*sinsy3*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3))) + (x(9) + x(13)*cosfi1*cosfi2)*((x(8) - x(13)*sinfi1)*(Ip33*cosfi1*sinfi1*sinfi2 + Ip22*cosfi1*cosfi3*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip11*cosfi1*sinfi3*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)) - (x(7) + x(13)*cosfi1*sinfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)^2 + Ip22*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2)^2 + Ip33*cosfi1^2*sinfi2^2) + (x(9) + x(13)*cosfi1*cosfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3) + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip33*cosfi1^2*cosfi2*sinfi2)) + (x(13)*x(7)*cosfi1*cosfi2 - x(13)*x(9)*cosfi1*sinfi2)*(Ip33*sinfi1^2 + Ip11*cosfi1^2*sinfi3^2 + Ip22*cosfi1^2*cosfi3^2) + (x(9) + x(15)*cossy1*cossy2)*((x(9) + x(15)*cossy1*cossy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3) + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq33*cossy1^2*cossy2*sinsy2) + (x(8) - x(15)*sinsy1)*(Iq33*cossy1*sinsy1*sinsy2 + Iq22*cossy1*cossy3*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq11*cossy1*sinsy3*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)) - (x(7) + x(15)*cossy1*sinsy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)^2 + Iq22*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2)^2 + Iq33*cossy1^2*sinsy2^2)) + x(9)*(c1*c2*mv*x(8) - x(7)*(Icv11 + c2^2*mv + c3^2*mv) + c1*c3*mv*x(9)) - x(7)*(c1*c3*mv*x(7) - x(9)*(Icv33 + c1^2*mv + c2^2*mv) + c2*c3*mv*x(8));
tauMinusCvec(6) =  ta3 - (x(13)*x(7)*cosfi1*cosfi2 - x(13)*x(9)*cosfi1*sinfi2)*(Ip33*cosfi1*cosfi2*sinfi1 - Ip22*cosfi1*cosfi3*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1) + Ip11*cosfi1*sinfi3*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)) - (x(8) - x(15)*sinsy1)*((x(9) + x(15)*cossy1*cossy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3) + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq33*cossy1^2*cossy2*sinsy2) + (x(8) - x(15)*sinsy1)*(Iq33*cossy1*sinsy1*sinsy2 + Iq22*cossy1*cossy3*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq11*cossy1*sinsy3*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)) - (x(7) + x(15)*cossy1*sinsy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)^2 + Iq22*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2)^2 + Iq33*cossy1^2*sinsy2^2)) - (x(8) - x(13)*sinfi1)*((x(8) - x(13)*sinfi1)*(Ip33*cosfi1*sinfi1*sinfi2 + Ip22*cosfi1*cosfi3*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip11*cosfi1*sinfi3*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)) - (x(7) + x(13)*cosfi1*sinfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)^2 + Ip22*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2)^2 + Ip33*cosfi1^2*sinfi2^2) + (x(9) + x(13)*cosfi1*cosfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3) + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip33*cosfi1^2*cosfi2*sinfi2)) + (x(15)*x(9)*cossy1*sinsy2 - x(15)*x(7)*cossy1*cossy2)*(Iq33*cossy1*cossy2*sinsy1 - Iq22*cossy1*cossy3*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1) + Iq11*cossy1*sinsy3*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3)) + (x(15)*x(9)*sinsy1 + x(15)*x(8)*cossy1*cossy2)*(Iq11*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3) + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq33*cossy1^2*cossy2*sinsy2) - m1*(g2*(x(8)*(g1*x(8) - g2*x(7)) + x(9)*(g1*x(9) - g3*x(7))) + g1*(x(7)*(g1*x(8) - g2*x(7)) - x(9)*(g2*x(9) - g3*x(8)))) - m2*(h2*(x(8)*(h1*x(8) - h2*x(7)) + x(9)*(h1*x(9) - h3*x(7))) + h1*(x(7)*(h1*x(8) - h2*x(7)) - x(9)*(h2*x(9) - h3*x(8)))) + mv*(x(7)*(c1*x(12) - c3*x(10)) + x(8)*(c2*x(12) - c3*x(11))) + (x(15)*x(7)*sinsy1 + x(15)*x(8)*cossy1*sinsy2)*(Iq11*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3)^2 + Iq22*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1)^2 + Iq33*cossy1^2*cossy2^2) + (x(7) + x(13)*cosfi1*sinfi2)*((x(9) + x(13)*cosfi1*cosfi2)*(Ip33*cosfi1*cosfi2*sinfi1 - Ip22*cosfi1*cosfi3*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1) + Ip11*cosfi1*sinfi3*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)) - (x(8) - x(13)*sinfi1)*(Ip33*sinfi1^2 + Ip11*cosfi1^2*sinfi3^2 + Ip22*cosfi1^2*cosfi3^2) + (x(7) + x(13)*cosfi1*sinfi2)*(Ip33*cosfi1*sinfi1*sinfi2 + Ip22*cosfi1*cosfi3*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip11*cosfi1*sinfi3*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3))) + m1*(x(7)*(g1*x(12) - g3*x(10)) + x(8)*(g2*x(12) - g3*x(11))) + m2*(x(7)*(h1*x(12) - h3*x(10)) + x(8)*(h2*x(12) - h3*x(11))) + (x(13)*x(7)*sinfi1 + x(13)*x(8)*cosfi1*sinfi2)*(Ip11*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3)^2 + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)^2 + Ip33*cosfi1^2*cosfi2^2) + (x(7) + x(15)*cossy1*sinsy2)*((x(9) + x(15)*cossy1*cossy2)*(Iq33*cossy1*cossy2*sinsy1 - Iq22*cossy1*cossy3*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1) + Iq11*cossy1*sinsy3*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3)) + (x(7) + x(15)*cossy1*sinsy2)*(Iq33*cossy1*sinsy1*sinsy2 + Iq22*cossy1*cossy3*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) - Iq11*cossy1*sinsy3*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3)) - (x(8) - x(15)*sinsy1)*(Iq33*sinsy1^2 + Iq22*cossy1^2*cossy3^2 + Iq11*cossy1^2*sinsy3^2)) + (x(13)*x(9)*sinfi1 + x(13)*x(8)*cosfi1*cosfi2)*(Ip11*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3)*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3) + Ip22*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1)*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) - Ip33*cosfi1^2*cosfi2*sinfi2) - x(8)*(c1*c2*mv*x(8) - x(7)*(Icv11 + c2^2*mv + c3^2*mv) + c1*c3*mv*x(9)) + x(7)*(c1*c2*mv*x(7) - x(8)*(Icv22 + c1^2*mv + c3^2*mv) + c2*c3*mv*x(9));
tauMinusCvec(7) =  tp3 + Ip11*(x(7)*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3) - x(9)*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3) + x(8)*cosfi1*sinfi3)*(x(9)*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1) - x(7)*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) + x(8)*cosfi1*cosfi3) - Ip22*(x(7)*(cosfi2*cosfi3 + sinfi1*sinfi2*sinfi3) - x(9)*(cosfi3*sinfi2 - cosfi2*sinfi1*sinfi3) + x(8)*cosfi1*sinfi3)*(x(9)*(sinfi2*sinfi3 + cosfi2*cosfi3*sinfi1) - x(7)*(cosfi2*sinfi3 - cosfi3*sinfi1*sinfi2) + x(8)*cosfi1*cosfi3);
tauMinusCvec(8) =  tq3 + Iq11*(x(9)*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1) - x(7)*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) + x(8)*cossy1*cossy3)*(x(7)*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3) - x(9)*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3) + x(8)*cossy1*sinsy3) - Iq22*(x(9)*(sinsy2*sinsy3 + cossy2*cossy3*sinsy1) - x(7)*(cossy2*sinsy3 - cossy3*sinsy1*sinsy2) + x(8)*cossy1*cossy3)*(x(7)*(cossy2*cossy3 + sinsy1*sinsy2*sinsy3) - x(9)*(cossy3*sinsy2 - cossy2*sinsy1*sinsy3) + x(8)*cossy1*sinsy3);

dtheta =  (x(8)*cosbeta + x(7)*sinbeta)/cosgamma;
dgamma =  x(7)*cosbeta - x(8)*sinbeta;
dbeta =  (x(9)*cosgamma - x(8)*cosbeta*singamma + x(7)*sinbeta*singamma)/cosgamma;
dx1 =  x(10)*(cosbeta*costheta + sinbeta*singamma*sintheta) - x(11)*(sinbeta*costheta - cosbeta*singamma*sintheta) + x(12)*cosgamma*sintheta;
dx2 =  x(11)*cosbeta*cosgamma - x(12)*singamma + x(10)*cosgamma*sinbeta;
dx3 =  x(11)*(sinbeta*sintheta + cosbeta*costheta*singamma) - x(10)*(cosbeta*sintheta - sinbeta*costheta*singamma) + x(12)*cosgamma*costheta;
dfi3 =  x(13);
dsy3 =  x(15);
betavec = v.MstarInv*tauMinusCvec;

xdot(1) = dx1;
xdot(2) = dx2;
xdot(3) = dx3;
xdot(4) = dtheta;
xdot(5) = dgamma;
xdot(6) = dbeta;
xdot(7) = betavec(4);  % betavec(4)
xdot(8) = betavec(5);  % betavec(5)
xdot(9) = betavec(6);  % betavec(6)
xdot(10) = betavec(1); % betavec(1)
xdot(11) = betavec(2); % betavec(2)
xdot(12) = betavec(3); % betavec(3)
xdot(13) = betavec(7); % betavec(7)
xdot(14) = dfi3;
xdot(15) = betavec(8); % betavec(8)
xdot(16) = dsy3;
% Not currently using the following, but may be usefull in the future.
% One version of the "long expression" can be found in equations_assumptions_dual.txt.
% See appendix A for a better way to compute these reaction torques.
%tp11 = [long expression] - v.rotor(1).torqueCM(1);
%tp12 = [long expression] - v.rotor(1).torqueCM(2);
%tp21 = [long expression] - v.rotor(2).torqueCM(1);
%tp22 = [long expression] - v.rotor(2).torqueCM(2);

