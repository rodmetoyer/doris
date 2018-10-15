function xdot = rotorState(t,x,rotor,fluid)
% State derivative function for the single-rotor prototype code
% INPUTS:
    % t = time (s)
    % x = state vector
    % rotor = rotor object
    % fluid = fluid object representing the freestream
mass = rotor.mass;
I = rotor.inertia;
Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);
theta_dot = x(7);
gamma_dot = x(8);
beta_dot = x(9);
cosbeta = cosd(x(3)); sinbeta = sind(x(3)); sin2beta = sind(2*x(3));
cosgamma = cosd(x(2)); singamma = sind(x(2)); sin2gamma = sind(2*x(2));
costheta = cosd(x(1)); sintheta = sind(x(1));

B_C_O = [cosbeta*costheta + sinbeta*singamma*sintheta, cosgamma*sinbeta, sinbeta*costheta*singamma - cosbeta*sintheta;cosbeta*singamma*sintheta - sinbeta*costheta, cosbeta*cosgamma, sinbeta*sintheta + cosbeta*costheta*singamma;cosgamma*sintheta,-singamma,cosgamma*costheta];
O_C_B = B_C_O.';


% Modify fluid as function of time if you want time-varying flow

% Update object states as needed
rotor.position = [x(4);x(5);x(6)];
rotor.orientation = [x(1);x(2);x(3)];

% Compute loads on blade sections
[rotorforce, rotortorque] = rotor.computeAeroLoadsBasic(fluid); % Rotor is responsible for computing his near-field
% Move to center mass if necessary. By symmetry, loads will be about rotor
% center mass for any rotor of n>1 blades when the blades are equal. (This
% should be done in a rotor class method, not here)

% Rotate to the inertial frame
rotorforce = O_C_B*rotorforce;
rotortorque = O_C_B*rotortorque;

fx = rotorforce(1);
fy = rotorforce(2);
fz = rotorforce(3);
taux = rotortorque(1);
tauy = rotortorque(2);
tauz = rotortorque(3);

% For now, single-element elastic tether fixed at the origin of length 1m
k = 100.0;
c = 25.0;
vec = [x(4); x(5); x(6)];
mag = norm(vec);
uvec = vec/mag;
if mag < 1*10^-13
    uvec = [0;0;0];
end
stretch = mag-1;
stretchd = (x(4)*x(10)+x(5)*x(11)+x(6)*x(12))/mag;
Fmag = 0;
if stretch > 1
    Fmag = -(stretch-1)*k;
end
if stretchd > 0
    Fmag = Fmag -stretchd*c;
end
fx = fx + Fmag*uvec(1);
fy = fy + Fmag*uvec(2);
fz = fz + Fmag*uvec(3);

% Equations of motion
% fx,y,x = forces in x,y,z direction
% taux,y,z = torque about x,y,z axis
xdot(1) = x(7);
xdot(2) = x(8);
xdot(3) = x(9);
xdot(4) = x(10);
xdot(5) = x(11);
xdot(6) = x(12);
xdot(7) = (Ixx*tauy*cosbeta - Iyy^2*beta_dot*gamma_dot + Iyy*taux*sinbeta + Ixx*Iyy*beta_dot*gamma_dot + Iyy*Izz*beta_dot*gamma_dot + Iyy^2*gamma_dot*theta_dot*singamma - Ixx^2*beta_dot*gamma_dot*cosbeta^2 + Iyy^2*beta_dot*gamma_dot*cosbeta^2 + Ixx*Iyy*gamma_dot*theta_dot*singamma - Iyy*Izz*gamma_dot*theta_dot*singamma + Ixx*Izz*beta_dot*gamma_dot*cosbeta^2 - Iyy*Izz*beta_dot*gamma_dot*cosbeta^2 + Ixx^2*gamma_dot*theta_dot*cosbeta^2*singamma - Iyy^2*gamma_dot*theta_dot*cosbeta^2*singamma + Ixx^2*theta_dot^2*cosbeta*cosgamma*sinbeta*singamma - Iyy^2*theta_dot^2*cosbeta*cosgamma*sinbeta*singamma - Ixx^2*beta_dot*theta_dot*cosbeta*cosgamma*sinbeta + Iyy^2*beta_dot*theta_dot*cosbeta*cosgamma*sinbeta - Ixx*Izz*gamma_dot*theta_dot*cosbeta^2*singamma + Iyy*Izz*gamma_dot*theta_dot*cosbeta^2*singamma - Ixx*Izz*theta_dot^2*cosbeta*cosgamma*sinbeta*singamma + Iyy*Izz*theta_dot^2*cosbeta*cosgamma*sinbeta*singamma + Ixx*Izz*beta_dot*theta_dot*cosbeta*cosgamma*sinbeta - Iyy*Izz*beta_dot*theta_dot*cosbeta*cosgamma*sinbeta)/(Ixx*Iyy*cosgamma);
xdot(8) = -(Ixx^2*theta_dot^2*sin2gamma - 2*Iyy*taux*cosbeta + 2*Ixx*tauy*sinbeta - Ixx*Izz*theta_dot^2*sin2gamma - Ixx^2*beta_dot*gamma_dot*sin2beta + Iyy^2*beta_dot*gamma_dot*sin2beta - 2*Ixx^2*beta_dot*theta_dot*cosgamma + 2*Ixx*Iyy*beta_dot*theta_dot*cosgamma + 2*Ixx*Izz*beta_dot*theta_dot*cosgamma - 2*Ixx^2*theta_dot^2*cosbeta^2*cosgamma*singamma + 2*Iyy^2*theta_dot^2*cosbeta^2*cosgamma*singamma + 2*Ixx^2*beta_dot*theta_dot*cosbeta^2*cosgamma - 2*Iyy^2*beta_dot*theta_dot*cosbeta^2*cosgamma + Ixx*Izz*beta_dot*gamma_dot*sin2beta - Iyy*Izz*beta_dot*gamma_dot*sin2beta + 2*Ixx^2*gamma_dot*theta_dot*cosbeta*sinbeta*singamma - 2*Iyy^2*gamma_dot*theta_dot*cosbeta*sinbeta*singamma + 2*Ixx*Izz*theta_dot^2*cosbeta^2*cosgamma*singamma - 2*Iyy*Izz*theta_dot^2*cosbeta^2*cosgamma*singamma - 2*Ixx*Izz*beta_dot*theta_dot*cosbeta^2*cosgamma + 2*Iyy*Izz*beta_dot*theta_dot*cosbeta^2*cosgamma - 2*Ixx*Izz*gamma_dot*theta_dot*cosbeta*sinbeta*singamma + 2*Iyy*Izz*gamma_dot*theta_dot*cosbeta*sinbeta*singamma)/(2*Ixx*Iyy);
xdot(9) = (Iyy^2*Izz*gamma_dot*theta_dot - Iyy*Izz^2*gamma_dot*theta_dot + Ixx*Iyy*tauz*cosgamma + Ixx*Izz*tauy*cosbeta*singamma + Iyy*Izz*taux*sinbeta*singamma + Ixx*Iyy*Izz*gamma_dot*theta_dot - Ixx*Izz^2*gamma_dot*theta_dot*cosbeta^2 + Ixx^2*Izz*gamma_dot*theta_dot*cosbeta^2 + Iyy*Izz^2*gamma_dot*theta_dot*cosbeta^2 - Iyy^2*Izz*gamma_dot*theta_dot*cosbeta^2 + Ixx*Iyy^2*gamma_dot*theta_dot*cosgamma^2 - Ixx^2*Iyy*gamma_dot*theta_dot*cosgamma^2 + Iyy*Izz^2*gamma_dot*theta_dot*cosgamma^2 - Iyy^2*Izz*gamma_dot*theta_dot*cosgamma^2 + Iyy*Izz^2*beta_dot*gamma_dot*singamma - Iyy^2*Izz*beta_dot*gamma_dot*singamma + Ixx*Izz^2*beta_dot*gamma_dot*cosbeta^2*singamma - Ixx^2*Izz*beta_dot*gamma_dot*cosbeta^2*singamma - Iyy*Izz^2*beta_dot*gamma_dot*cosbeta^2*singamma + Iyy^2*Izz*beta_dot*gamma_dot*cosbeta^2*singamma - 2*Ixx*Iyy^2*gamma_dot*theta_dot*cosbeta^2*cosgamma^2 + 2*Ixx^2*Iyy*gamma_dot*theta_dot*cosbeta^2*cosgamma^2 + Ixx*Izz^2*gamma_dot*theta_dot*cosbeta^2*cosgamma^2 - Ixx^2*Izz*gamma_dot*theta_dot*cosbeta^2*cosgamma^2 - Iyy*Izz^2*gamma_dot*theta_dot*cosbeta^2*cosgamma^2 + Iyy^2*Izz*gamma_dot*theta_dot*cosbeta^2*cosgamma^2 + Ixx*Iyy^2*gamma_dot^2*cosbeta*cosgamma*sinbeta - Ixx^2*Iyy*gamma_dot^2*cosbeta*cosgamma*sinbeta - Ixx*Izz^2*theta_dot^2*cosbeta*cosgamma*sinbeta + Ixx^2*Izz*theta_dot^2*cosbeta*cosgamma*sinbeta + Iyy*Izz^2*theta_dot^2*cosbeta*cosgamma*sinbeta - Iyy^2*Izz*theta_dot^2*cosbeta*cosgamma*sinbeta + Ixx*Iyy*Izz*beta_dot*gamma_dot*singamma - Ixx*Iyy^2*theta_dot^2*cosbeta*cosgamma^3*sinbeta + Ixx^2*Iyy*theta_dot^2*cosbeta*cosgamma^3*sinbeta + Ixx*Izz^2*theta_dot^2*cosbeta*cosgamma^3*sinbeta - Ixx^2*Izz*theta_dot^2*cosbeta*cosgamma^3*sinbeta - Iyy*Izz^2*theta_dot^2*cosbeta*cosgamma^3*sinbeta + Iyy^2*Izz*theta_dot^2*cosbeta*cosgamma^3*sinbeta + Ixx*Izz^2*beta_dot*theta_dot*cosbeta*cosgamma*sinbeta*singamma - Ixx^2*Izz*beta_dot*theta_dot*cosbeta*cosgamma*sinbeta*singamma - Iyy*Izz^2*beta_dot*theta_dot*cosbeta*cosgamma*sinbeta*singamma + Iyy^2*Izz*beta_dot*theta_dot*cosbeta*cosgamma*sinbeta*singamma)/(Ixx*Iyy*Izz*cosgamma);
xdot(10) = 0;%fx/mass;
xdot(11) = 0;%fy/mass;
xdot(12) = 0;%fz/mass;
xdot = xdot.';