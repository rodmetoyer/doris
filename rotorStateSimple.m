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
I = rotor.inertia;
Ixx = I(1,1); Iyy = I(2,2); Izz = I(3,3);
cosbeta = cos(x(3)); sinbeta = sin(x(3)); %sin2beta = sind(2*x(3));
cosgamma = cos(x(2)); singamma = sin(x(2)); %sin2gamma = sind(2*x(2));
costheta = cos(x(1)); sintheta = sin(x(1));

%%
% Compute
% $\phantom{}^B[C]^O$
% and
% $\phantom{}^O[C]^B$
% matrices.
B_C_O = [cosbeta*costheta + sinbeta*singamma*sintheta, cosgamma*sinbeta, sinbeta*costheta*singamma - cosbeta*sintheta;cosbeta*singamma*sintheta - sinbeta*costheta, cosbeta*cosgamma, sinbeta*sintheta + cosbeta*costheta*singamma;cosgamma*sintheta,-singamma,cosgamma*costheta];
O_C_B = B_C_O.';


%% Modify fluid as function of time if you want time-varying flow

%% Update rotor object properites with current state info
rotor.position = [x(4);x(5);x(6)];
rotor.orientation = [x(1);x(2);x(3)];
temp = O_C_B*[x(10);x(11);x(12)];
rotor.velocity = temp;
rotor.angvel = [x(7);x(8);x(9)];

%% Compute loads on rotor
[rotorforce, rotortorque] = rotor.computeAeroLoadsBasic(fluid); % Rotor is responsible for computing his near-field
% Move to center mass if necessary. By symmetry, loads will be about rotor
% center mass for any rotor of n>1 blades when the blades are equal. (This
% should be done in a rotor class method, not here)
taux = rotortorque(1);
tauy = rotortorque(2);
tauz = rotortorque(3);

% Rotate force to the inertial frame to make it easier to add tether loads
rotorforce = O_C_B*rotorforce;
Fx = rotorforce(1);
Fy = rotorforce(2);
Fz = rotorforce(3);

% For now, single-element elastic tether fixed at the origin of length 1m
k = 100.0;
c = 25.0;
vec = [x(4); x(5); x(6)];
mag = norm(vec);
uvec = vec/mag;
if mag < 1*10^-13
    uvec = [0;0;0];
end
stretch = mag; % If it moves from the origin there is a restoring force
stretchd = (x(4)*temp(1)+x(5)*temp(2)+x(6)*temp(3))/mag;
Fmag = 0;
if stretch > 0
    Fmag = -stretch*k;
end
if stretchd > 0
    Fmag = Fmag -stretchd*c;
end
Fx = Fx + Fmag*uvec(1);
Fy = Fy + Fmag*uvec(2);
Fz = Fz + Fmag*uvec(3);

%% Update states
% fx,y,x = forces in x,y,z direction
% taux,y,z = torque about x,y,z axis
xdot(1) = (x(8)*cosbeta + x(7)*sinbeta)/cosgamma;
xdot(2) = x(7)*cosbeta - x(8)*sinbeta;
xdot(3) = (x(9)*cosgamma + x(8)*cosbeta*singamma + x(7)*sinbeta*singamma)/cosgamma;
xdot(4) = temp(1);
xdot(5) = temp(2);
xdot(6) = temp(3);
xdot(7) = taux/Ixx+(Iyy-Izz)/Ixx*x(8)*x(9);
xdot(8) = tauy/Iyy+(Izz-Ixx)/Iyy*x(9)*x(7);
xdot(9) = tauz/Izz+(Ixx-Iyy)/Izz*x(7)*x(8);
xdot(10) = x(11)*x(9) - x(12)*x(8) + (Fx*cosbeta*costheta)/m + (Fy*cosgamma*sinbeta)/m - (Fz*cosbeta*sintheta)/m + (Fx*sinbeta*singamma*sintheta)/m + (Fz*sinbeta*costheta*singamma)/m;
xdot(11) = x(12)*x(7) - x(10)*x(9) + (Fy*cosbeta*cosgamma)/m - (Fx*sinbeta*costheta)/m + (Fz*sinbeta*sintheta)/m + (Fz*cosbeta*costheta*singamma)/m + (Fx*cosbeta*singamma*sintheta)/m;
xdot(12) = x(10)*x(8) - x(11)*x(7) - (Fy*singamma)/m + (Fz*cosgamma*costheta)/m + (Fx*cosgamma*sintheta)/m;
xdot = xdot.';