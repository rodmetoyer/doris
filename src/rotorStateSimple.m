function xdot = rotorStateSimple(t,x,rotor,fluid)
% State derivative function for the single-rotor prototype code
% INPUTS:
    % t = time (s)
    % x = state vector
    % rotor = rotor object
    % fluid = fluid object representing the freestream

%% Get useful values
% todo(rodney) look into the wisdom of this type of implementation. Sure it
% makes the equations easier to type and read, but are we sacrificing
% computation economy? Am I forcing matlab to allocate and deallocate
% memory to make a copy of rotor mass? Maybe faster to just use the object
% property.
m = rotor.mass;
g = 9.81;
I = rotor.inertia;
Ixx = I(1,1); Iyy = I(2,2); Izz = I(3,3);
cosbeta = cos(x(3)); sinbeta = sin(x(3));
cosgamma = cos(x(2)); singamma = sin(x(2));
costheta = cos(x(1)); sintheta = sin(x(1));

%% Modify fluid as function of time if you want time-varying flow
%fluid.velocity = [0.5*tanh(t/3);0;0];
%fluid.velocity = [0.5/(1+exp(-0.5*(t-10)));0;0]; 

%% Update rotor object properites with current state info
rotor.position = [x(4);x(5);x(6)];   % Expressed in O frame
rotor.orientation = [x(1);x(2);x(3)];
% Compute
% $\phantom{}^B[C]^O$
% and
% $\phantom{}^O[C]^B$
% matrices.
B_C_O = rotor.B_C_O;%[cosbeta*costheta + sinbeta*singamma*sintheta, cosgamma*sinbeta, sinbeta*costheta*singamma - cosbeta*sintheta;cosbeta*singamma*sintheta - sinbeta*costheta, cosbeta*cosgamma, sinbeta*sintheta + cosbeta*costheta*singamma;cosgamma*sintheta,-singamma,cosgamma*costheta];
O_C_B = B_C_O.';
OV_AO_O = O_C_B*[x(10);x(11);x(12)];
rotor.velocity = OV_AO_O;            % Expressed in O frame
rotor.angvel = [x(7);x(8);x(9)];     % Expressed in rotor frame

%% Compute loads on rotor
[rotorforce, rotortorque] = rotor.computeAeroLoadsBasic(fluid); % Rotor is responsible for computing his near-field
% Move to center mass if necessary. By symmetry, loads will be about rotor
% center mass for any rotor of n>1 blades when the blades are equal. (This
% should be done in a rotor class method, not here)
% Torque and angular rates are expressed in the rotor frame.
taux = rotortorque(1);
tauy = rotortorque(2);
tauz = rotortorque(3);

% Forces and positions are expressed in the inertial frame
rotorforce = O_C_B*rotorforce;
Fx = rotorforce(1);
Fy = rotorforce(2);
Fz = rotorforce(3);

% For now, single-element elastic tether fixed at the origin of length 1m
k = 10.0;
c = 5.0;
vec = [x(4); x(5); x(6)];
mag = norm(vec);
unitvec = vec/mag;
if mag < 1*10^-13
    unitvec = [0;0;0];
end
stretch = mag; % If it moves from the origin there is a restoring force
stretchd = (x(4)*OV_AO_O(1)+x(5)*OV_AO_O(2)+x(6)*OV_AO_O(3))/mag;
Fmag = 0;
if stretch > 0
    Fmag = -stretch*k;
end
if stretchd > 0
    Fmag = Fmag - stretchd*c;
end
Fx = Fx + Fmag*unitvec(1);
Fy = Fy + Fmag*unitvec(2);
Fz = Fz + Fmag*unitvec(3)-m*g;

%% Update states
% fx,y,x = forces in x,y,z direction
% taux,y,z = torque about x,y,z axis
xdot(1) = (x(8)*cosbeta + x(7)*sinbeta)/cosgamma;
xdot(2) = x(7)*cosbeta - x(8)*sinbeta;
xdot(3) = (x(9)*cosgamma + x(8)*cosbeta*singamma + x(7)*sinbeta*singamma)/cosgamma;
xdot(4) = OV_AO_O(1);
xdot(5) = OV_AO_O(2);
xdot(6) = OV_AO_O(3);
xdot(7) = taux/Ixx+(Iyy-Izz)/Ixx*x(8)*x(9);
xdot(8) = tauy/Iyy+(Izz-Ixx)/Iyy*x(9)*x(7);
xdot(9) = tauz/Izz+(Ixx-Iyy)/Izz*x(7)*x(8);
xdot(10) = x(11)*x(9) - x(12)*x(8) + (Fx*cosbeta*costheta)/m + (Fy*cosgamma*sinbeta)/m - (Fz*cosbeta*sintheta)/m + (Fx*sinbeta*singamma*sintheta)/m + (Fz*sinbeta*costheta*singamma)/m;
xdot(11) = x(12)*x(7) - x(10)*x(9) + (Fy*cosbeta*cosgamma)/m - (Fx*sinbeta*costheta)/m + (Fz*sinbeta*sintheta)/m + (Fz*cosbeta*costheta*singamma)/m + (Fx*cosbeta*singamma*sintheta)/m;
xdot(12) = x(10)*x(8) - x(11)*x(7) - (Fy*singamma)/m + (Fz*cosgamma*costheta)/m + (Fx*cosgamma*sintheta)/m;
xdot = xdot.';