% rotorSim.m
% Simualtion script for a single-rotor simulation

clearvars; close all; clc;
runname = 'sg6040skew30lowerSpringk';
makeplots = true;
makemovie = true; moviefile = ['figures\' runname '.avi'];


%% Make objects
% fluid
water = fluid; % No arguments to fluid gives the obj water properties

% airfoils - same for the entire rotor so we just need one
% 0 = SG6040 Improved force coefficient curves
% 2 = SG6040 Old Force coefficent curves
% 1 = S814
af = airfoil(0,'SG6040');

% blade sections
aspectRatio = 7;
bladeLength = 4*0.0254; % Inches times meters per inch
bladeChord = bladeLength/aspectRatio;
numSections = 20;       % Number of sections (whole number)
bs = bladesection(bladeLength/aspectRatio,bladeLength/numSections,af);
for i=1:1:numSections
    section(i) = bs;
end

% blade
bladeMass = 0.25; % kg
numBlades = 3;
bladeDZfrac = 0.05; % Blade dead zone fraction (Ro/R - see Spera 2009) %%%NOTE%%% Only used to compute twist right now. Forces are computed the entire length of the blade.
rtos = 2.0;    % This is the ratio of rotor radius to disturbed fluid
% stream length for computing optimal TSR. A good rule-of-thumb value is
% 2.0 (Ragheb and Ragheb 2011).
AoAopt_deg = 8.0; % Optimal angle of attack for the airfoil
% Compute optimal TSR using method described in (Ragheb and Ragheb 2011)
TSRopt = 2*pi/numBlades*rtos;
% Compute Twist using method described in Ch. 5 of (Gasch and Twele 2012)
locs = linspace(bladeLength*bladeDZfrac,bladeLength,numSections);
twist = atand(2/3*bladeLength/TSRopt*1./locs)-AoAopt_deg;
% twist = -[64.6,49.9,38.7,30.5,24.5,19.95,16.45,13.7,11.5,9.7,8.1,6.9,5.76,4.81,3.98,3.25,2.61,2.03,1.51,1.04];
b = blade(section,bladeMass,twist);
for i=1:1:numBlades
    blade(i) = b; % Note that this is a vector of the same handle
end

% rotor
rotor = rotor(blade);

%% Set-up simulation
tstep = 0.05;
ttot = 60;
tspan = 0:tstep:ttot;

% Initial states
% State vector
%    1      2     3    4  5  6     7     
% [theta, gamma, beta, x, y, z, theta_dot,
%     8          9      10      11     12
% gamma_dot, beta_dot, x_dot, y_dot, z_dot]
x0 = [60*pi/180;0.0*pi/180;0;0;0;0;0;0;0;0;0;0];
water.velocity = [0.1;0.0;0];

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
opts = odeset('RelTol',1e-3,'AbsTol',1e-5,'Stats','on','OutputFcn',@odeplot);
[t, y] = ode45(@(t,y) rotorStateSimple( t,y,rotor,water),tspan,x0,opts);
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
    r_b1cm_O = O_C_B*rotor.sectPos(:,numSections,1);
    r_b2cm_O = O_C_B*rotor.sectPos(:,numSections,2);
    r_b3cm_O = O_C_B*rotor.sectPos(:,numSections,3);
    figure(f1);
    plot3([r_Ocm_O(i,1) r_Ocm_O(i,1)+r_b1cm_O(1)],[r_Ocm_O(i,2) r_Ocm_O(i,2)+r_b1cm_O(2)],[r_Ocm_O(i,3) r_Ocm_O(i,3)+r_b1cm_O(3)],'r');
    hold on
    plot3([r_Ocm_O(i,1) r_Ocm_O(i,1)+r_b2cm_O(1)],[r_Ocm_O(i,2) r_Ocm_O(i,2)+r_b2cm_O(2)],[r_Ocm_O(i,3) r_Ocm_O(i,3)+r_b2cm_O(3)],'b');
    plot3([r_Ocm_O(i,1) r_Ocm_O(i,1)+r_b3cm_O(1)],[r_Ocm_O(i,2) r_Ocm_O(i,2)+r_b3cm_O(2)],[r_Ocm_O(i,3) r_Ocm_O(i,3)+r_b3cm_O(3)],'k');
    axis equal
    axis([-0.25 0.25 -0.25 0.25 -0.25 0.25]);
    
    view(-215,32)
    xlabel('x'); ylabel('y');
    F(i) = getframe(f1);    
    hold off
end

% sm = input('Do you want to save the movie file? Y/N [Y]: ', 's');
% if isempty(sm)
%     sm = 'Y';
% end
% todo make an interface for v props
% if strcmp(sm,'Y')
if makemovie
    v = VideoWriter(moviefile);
    v.FrameRate = round(1/tstep)*0.125;
    %v.Quality = 100;% v.Width = 800; w.Height = 450;
    open(v);
    writeVideo(v,F); close(v);
end

%% Make some plots
if makeplots
    figure
    plot(t,y(:,1)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\theta (Yaw angle, degrees)')
    saveas(gcf,['figures\' runname '_theta.png'])
    
    figure
    plot(t,y(:,2)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\gamma (Pitch angle, degrees)')
    saveas(gcf,['figures\' runname '_gamma.png'])
    
    figure
    plot(t,y(:,3)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\beta (Azimuth angle, degrees)')
    saveas(gcf,['figures\' runname '_beta.png'])
    
    figure
    plot(t,y(:,4),'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('x Position (m)')
    saveas(gcf,['figures\' runname '_x.png'])
    
    figure
    plot(t,y(:,5),'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('y Position (m)')
    saveas(gcf,['figures\' runname '_y.png'])
        
    figure
    plot(t,y(:,6),'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('z Position (m)')
    saveas(gcf,['figures\' runname '_z.png'])
    
    figure
    plot(t,y(:,7)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\thetaDot (Yaw rate, deg/s)')
    saveas(gcf,['figures\' runname '_thetadot.png'])
    
    figure
    plot(t,y(:,8)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\gammaDot (Pitch rate, deg/s)')
    saveas(gcf,['figures\' runname '_gammadot.png'])
    
    figure
    plot(t,y(:,9)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\betaDot (Spin rate, deg/s)')
    saveas(gcf,['figures\' runname '_betadot.png'])
end