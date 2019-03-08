% prototypeSim.m
% Simualtion script for the protptype single-rotor simulation

clearvars; close all; clc;

%% Make objects
% fluid
water = fluid; % No arguments makes to fluid give the obj water props

% airfoils - same for the entire rotor so we just need one
af = airfoil(2,'SG6040');

% blade sections
aspectRatio = 7;
bladeLength = 4*0.0254; % inch*m/inch
numSections = 20;
bs = bladesection(bladeLength/aspectRatio,bladeLength/numSections,af);
for i=1:1:numSections
    section(i) = bs;
end

% blade
bladeMass = 0.02;
numBlades = 3;
twist = -[64.6,49.9,38.7,30.5,24.5,19.95,16.45,13.7,11.5,9.7,8.1,6.9,5.76,4.81,3.98,3.25,2.61,2.03,1.51,1.04];
b = blade(section,bladeMass,twist);
for i=1:1:numBlades
    blade(i) = b; % Note that this is a vector of the same handle
end

% rotor
rotor = rotor(blade);

%% Set-up simulation
tstep = 0.05;
ttot = 10;
tspan = 0:tstep:ttot;

% Initial states
% State vector
%    1      2     3    4  5  6     7     
% [theta, gamma, beta, x, y, z, theta_dot,
%     8          9      10      11     12
% gamma_dot, beta_dot, x_dot, y_dot, z_dot]
x0 = [90*pi/180;0;0;0;0;0;0;0;0;0;0;0];
water.velocity = [0.1;0;0];

% Need to set rotor position. This will happen automatically in the state
% file from now on. todo(rodney) fix this when you move ot vehicle.
rotor.position = [x0(4);x0(5);x0(6)];
rotor.orientation = [x0(1);x0(2);x0(3)];
rotor.velocity = [x0(10);x0(11);x0(12)];

% todo(rodney) change next line to computed angular velocity using initial
% states rather than hardcode of zeros.
rotor.angvel = [0;0;0];

% Function Handle
fnhndl = @rotorStateSimple;

disp('Running the simulation');
%[t, y] = ode45(@(t,y) rotorState(t,y,rotor,water),tspan,x0);
[t, y] = ode45(@(t,y) rotorStateSimple( t,y,rotor,water),tspan,x0);
figure
plot(t,y(:,4))
close gcf

%% Make a movie
disp('Making a movie of the motion');
r_Ocm_O = [y(:,4),y(:,5),y(:,6)];
f1 = figure;
f1.Position = [100 100 900 550];
f1.Color = [1 1 1];
for i = 1:1:length(t)
    cthe = cos(y(i,1)); sthe = sin(y(i,1));
    cgam = cos(y(i,2)); sgam = sin(y(i,2));
    cbet = cos(y(i,3)); sbet = sin(y(i,3));
    B_C_O = [cbet*cthe + sbet*sgam*sthe, cgam*sbet, sbet*cthe*sgam - cbet*sthe;...
        cbet*sgam*sthe - sbet*cthe, cbet*cgam, sbet*sthe + cbet*cthe*sgam;...
        cgam*sthe,-sgam,cgam*cthe];
    O_C_B = transpose(B_C_O);
    r_b1cm_O = O_C_B*rotor.sectPos(:,20,1);
    r_b2cm_O = O_C_B*rotor.sectPos(:,20,2);
    r_b3cm_O = O_C_B*rotor.sectPos(:,20,3);
    figure(f1);
    plot3([r_Ocm_O(i,1) r_Ocm_O(i,1)+r_b1cm_O(1)],[r_Ocm_O(i,2) r_Ocm_O(i,2)+r_b1cm_O(2)],[r_Ocm_O(i,3) r_Ocm_O(i,3)+r_b1cm_O(3)],'r');
    hold on
    plot3([r_Ocm_O(i,1) r_Ocm_O(i,1)+r_b2cm_O(1)],[r_Ocm_O(i,2) r_Ocm_O(i,2)+r_b2cm_O(2)],[r_Ocm_O(i,3) r_Ocm_O(i,3)+r_b2cm_O(3)],'b');
    plot3([r_Ocm_O(i,1) r_Ocm_O(i,1)+r_b3cm_O(1)],[r_Ocm_O(i,2) r_Ocm_O(i,2)+r_b3cm_O(2)],[r_Ocm_O(i,3) r_Ocm_O(i,3)+r_b3cm_O(3)],'k');
    axis equal
    axis([-0.25 0.25 -0.25 0.25 -0.25 0.25]);
    
    %view(0,0)
    xlabel('x'); ylabel('y');
    F(i) = getframe(f1);    
    hold off
end
sm = input('Do you want to save the movie file? Y/N [Y]: ', 's');
if isempty(sm)
    sm = 'Y';
end
% todo make an interface for v props
if strcmp(sm,'Y')
    v = VideoWriter('figures\generatorviz2.avi');
    v.FrameRate = round(1/tstep);
    %v.Quality = 100;% v.Width = 800; w.Height = 450;
    open(v);
    writeVideo(v,F); close(v);
end