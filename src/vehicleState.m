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

%% Update fluid velocity and generator load resistance
f.updateVelocity(t);
v.generator.updateLoadResistance(t);

%% Compute hydrodynamic loads
v.computeHydroLoads(f);

%% Compute tether loads and add them to the vehicle loads
% For now, single-element elastic tether fixed at the origin of length 1m
% todo(rodney) make a tether class. Number nodes = number links-1. End
% nodes are part of the signal. Output is loads on end links and position
% of internal nodes
O_C_A = transpose(v.A_C_O);
r_to_O = [x(1); x(2); x(3)] + O_C_A*v.tetherpoint;
Ov_to_O = O_C_A*v.velocity + O_C_A*cross(v.angvel,v.tetherpoint);
[A,Ad] = thr.getAnchorPoint(t);
thr.setLocationA(A);
thr.setVelocityA(Ad);
thr.setLocationB(r_to_O);
thr.setVelocityB(Ov_to_O);
if thr.numnodes > 0
    % States for two rotors 16+1=17 to 16+3N are tether node positions and states 16+3N+1 to
    % 16+3N+3N=16+6N are tether node velocities
    numstates = numel(x);
    % First update the tether states with the current information x(numstates+1:numstates+6*thr.numnodes)
    thr.nodelocs = x(numstates+1:numstates+3*thr.numnodes);
    thr.nodevels = x(numstates+3*thr.numnodes+1:numstates+6*thr.numnodes);
    % now compute the state derivatives
    [tetherforce,xdot(numstates+1+3*thr.numnodes:numstates+6*thr.numnodes)] = thr.computeTension(A,B,r_to_O,Ov_to_O,f);
    xdot(numstates+1:numstates+3*thr.numnodes) = x(numstates+1+3*thr.numnodes:numstates+6*thr.numnodes);
else % no internal nodes
    thr.computeTension(f);
    tetherforce = thr.tension(:,2);
end
v.addTetherLoads(v.A_C_O*tetherforce);

%% Compute hydrostatic loads
buoyForceA = transpose(O_C_A)*[0;0;(1/v.relDensity)*f.gravity*v.mass];
weightA = transpose(O_C_A)*[0;0;-v.mass*f.gravity];
buoyTorqueA = cross(v.buoypoint,buoyForceA);
weightTorqueA = cross(v.centermass,weightA);
v.force = v.force + buoyForceA + weightA;
v.torque = v.torque + buoyTorqueA + weightTorqueA;

%% Add generator loads
% All of the hydrodynamic loads are computed and updated in the
% computeHydroLoads method, but we need to add the generator loads to the
% rotor torques.
relRotSpeed = x(13)-x(15);
gtq = v.generator.getTorque(relRotSpeed);
v.rotors(1).addTorque(gtq);
v.rotors(2).addTorque(-gtq);

%% prepare to compute state derivatives
ta1 = v.torque(1); ta2 = v.torque(2); ta3 = v.torque(3);
f1 = v.force(1); f2 = v.force(2); f3 = v.force(3);
m1 = v.rotors(1).mass; m2 = v.rotors(2).mass; mv = v.body.mass;
c1 = v.body.centermass(1); % c2 = v.body.centermass(2);
c3 = v.body.centermass(3);  % Only this will be non-zero for a coaxial turbine
%g1 = v.rotorLocs(1,1); g2 = v.rotorLocs(2,1);
g3 = v.rotorLocs(3,1); % Only this will be non-zero for a coaxial turbine
%h1 = v.rotorLocs(1,2); h2 = v.rotorLocs(2,2);
h3 = v.rotorLocs(3,2); % Only this will be non-zero for a coaxial turbine
% Position from point A to CM of system
s1 = v.centermass(1); % s2 = v.centermass(2); 
s3 = v.centermass(3);
Icv11 = v.body.inertia(1,1);
Icv22 = v.body.inertia(2,2); 
Icv33 = v.body.inertia(3,3);
Icv13 = v.body.inertia(1,3);
Ip11 = v.rotors(1).inertia(1,1);
Ip22 = v.rotors(1).inertia(2,2);
Ip33 = v.rotors(1).inertia(3,3);
Iq11 = v.rotors(2).inertia(1,1);
Iq22 = v.rotors(2).inertia(2,2);
Iq33 = v.rotors(2).inertia(3,3);
tp3 = v.rotors(1).torqueCM(3);
tq3 = v.rotors(2).torqueCM(3);

%% Compute state derivatives
cvecstar = NaN(8,1);
cvecstar(1) =  - (s1*x(8)^2 + x(9)*(s1*x(9) - s3*x(7)))*(m1 + m2 + mv) - (x(11)*x(9) - x(12)*x(8))*(m1 + m2 + mv);
cvecstar(2) =  (s1*x(7)*x(8) + s3*x(8)*x(9))*(m1 + m2 + mv) + (x(10)*x(9) - x(12)*x(7))*(m1 + m2 + mv);
cvecstar(3) =  - (s3*x(8)^2 - x(7)*(s1*x(9) - s3*x(7)))*(m1 + m2 + mv) - (x(10)*x(8) - x(11)*x(7))*(m1 + m2 + mv);
cvecstar(4) =  x(8)*(x(9)*(Icv33 + c1^2*mv) - x(7)*(Icv13 + c1*c3*mv)) - (x(15) + x(9))*(x(7)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(8)*(Iq22*cossy3^2 + Iq11*sinsy3^2)) + mv*(x(9)*(c1*x(12) - c3*x(10)) + c1*x(11)*x(8)) - (x(7)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(8)*(Ip22*cosfi3^2 + Ip11*sinfi3^2))*(x(13) + x(9)) - x(8)*x(9)*(Icv22 + c1^2*mv + c3^2*mv) - x(13)*x(7)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(13)*x(8)*(Ip11*cosfi3^2 + Ip22*sinfi3^2) + Ip33*x(8)*(x(13) + x(9)) + Iq33*x(8)*(x(15) + x(9)) - x(15)*x(7)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(15)*x(8)*(Iq11*cossy3^2 + Iq22*sinsy3^2) - g3*m1*x(10)*x(9) - h3*m2*x(10)*x(9) - g3^2*m1*x(8)*x(9) - h3^2*m2*x(8)*x(9);
cvecstar(5) =  (x(15) + x(9))*(x(8)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(7)*(Iq11*cossy3^2 + Iq22*sinsy3^2)) + x(9)*(x(7)*(Icv11 + c3^2*mv) - x(9)*(Icv13 + c1*c3*mv)) - x(7)*(x(9)*(Icv33 + c1^2*mv) - x(7)*(Icv13 + c1*c3*mv)) - mv*(c1*x(11)*x(7) + c3*x(11)*x(9)) + (x(8)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(7)*(Ip11*cosfi3^2 + Ip22*sinfi3^2))*(x(13) + x(9)) + x(13)*x(8)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) - x(13)*x(7)*(Ip22*cosfi3^2 + Ip11*sinfi3^2) - Ip33*x(7)*(x(13) + x(9)) - Iq33*x(7)*(x(15) + x(9)) + x(15)*x(8)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) - x(15)*x(7)*(Iq22*cossy3^2 + Iq11*sinsy3^2) - g3*m1*x(11)*x(9) - h3*m2*x(11)*x(9) + g3^2*m1*x(7)*x(9) + h3^2*m2*x(7)*x(9);
cvecstar(6) =  x(7)*(x(7)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(8)*(Iq22*cossy3^2 + Iq11*sinsy3^2)) - x(8)*(x(8)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(7)*(Iq11*cossy3^2 + Iq22*sinsy3^2)) - x(8)*(x(7)*(Icv11 + c3^2*mv) - x(9)*(Icv13 + c1*c3*mv)) - mv*(x(7)*(c1*x(12) - c3*x(10)) - c3*x(11)*x(8)) + m1*(g3*x(10)*x(7) + g3*x(11)*x(8)) + m2*(h3*x(10)*x(7) + h3*x(11)*x(8)) + x(7)*(x(7)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(8)*(Ip22*cosfi3^2 + Ip11*sinfi3^2)) - x(8)*(x(8)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(7)*(Ip11*cosfi3^2 + Ip22*sinfi3^2)) + x(7)*x(8)*(Icv22 + c1^2*mv + c3^2*mv);
cvecstar(7) =  Ip22*(x(7)*cosfi3 + x(8)*sinfi3)*(x(8)*cosfi3 - x(7)*sinfi3) - Ip11*(x(7)*cosfi3 + x(8)*sinfi3)*(x(8)*cosfi3 - x(7)*sinfi3);
cvecstar(8) =  Iq22*(x(7)*cossy3 + x(8)*sinsy3)*(x(8)*cossy3 - x(7)*sinsy3) - Iq11*(x(7)*cossy3 + x(8)*sinsy3)*(x(8)*cossy3 - x(7)*sinsy3);

tau = [f1;f2;f3;ta1;ta2;ta3;tp3;tq3];

dtheta =  (x(8)*cosbeta + x(7)*sinbeta)/cosgamma;
dgamma =  x(7)*cosbeta - x(8)*sinbeta;
dbeta =  (x(9)*cosgamma - x(8)*cosbeta*singamma + x(7)*sinbeta*singamma)/cosgamma;
dx1 =  x(10)*(cosbeta*costheta + sinbeta*singamma*sintheta) - x(11)*(sinbeta*costheta - cosbeta*singamma*sintheta) + x(12)*cosgamma*sintheta;
dx2 =  x(11)*cosbeta*cosgamma - x(12)*singamma + x(10)*cosgamma*sinbeta;
dx3 =  x(11)*(sinbeta*sintheta + cosbeta*costheta*singamma) - x(10)*(cosbeta*sintheta - sinbeta*costheta*singamma) + x(12)*cosgamma*costheta;
dfi3 =  x(13);
dsy3 =  x(15);
%betavec = v.MstarInv*(tau-cvec);
betavec = v.MtotInv*(tau-v.Mtot*v.MstarInv*cvecstar);

xdot(1) = dx1;
xdot(2) = dx2;
xdot(3) = dx3;
xdot(4) = dtheta;
xdot(5) = dgamma;
xdot(6) = dbeta;
xdot(7) = betavec(4);  % omg_1
xdot(8) = betavec(5);  % omg_2
xdot(9) = betavec(6);  % omg_3
xdot(10) = betavec(1); % u_1
xdot(11) = betavec(2); % u_2
xdot(12) = betavec(3); % u_3
xdot(13) = betavec(7); % p_3
xdot(14) = dfi3;
xdot(15) = betavec(8); % q_3
xdot(16) = dsy3;
% Not currently using the following, but may be usefull in the future.
% One version of the "long expression" can be found in equations_assumptions_dual.txt.
% See appendix A for a better way to compute these reaction torques.
%tp11 = [long expression] - v.rotor(1).torqueCM(1);
%tp12 = [long expression] - v.rotor(1).torqueCM(2);
%tp21 = [long expression] - v.rotor(2).torqueCM(1);
%tp22 = [long expression] - v.rotor(2).torqueCM(2);

