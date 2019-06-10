% rotorSim.m
% Simualtion script for a single-rotor simulation

clearvars; close all; clc;

% Tell matlab to look in the src folder for the class files
addpath('src')

% Input file
inputfile = 'singleRotorBaseline.txt';
fid = fopen(['input\' inputfile]);
% Inputs to workspace
while true
    tline = fgetl(fid);            
    if isnumeric(tline)
        break;
    end
    eval(tline);
end
fclose(fid);

runname = ['Yaw' num2str(initialYaw) '_Pitch' num2str(initialPitch) '_yOff' num2str(initialLateral) '_vbttest1'];
moviefile = ['products\videos\' runname '.avi'];

%% Make objects
% fluid
water = fluid; % No arguments to fluid gives the obj water properties
water.init(fluidtype);

% airfoils - same for the entire rotor so we just need one
af = airfoil(airfoiltype);

% blade sections
bs = bladesection(secChord,secWidth,af);

% blade - trying to give the option to specify the twist in the input file.
% If no twist is specified then compute it using idealized assumptions.
if isempty(twist)
    AoAopt_deg = 8.0;
    % TODO put this on the blade.
    % blade.maketwist(AoAOpt_deg,bladeDZfrac,numBlades) - Or maybe the
    % rotor? What makes sense?
    twist = computeTwist(AoAopt_deg,bladeLength,bladeDZfrac,numSections,numBlades);
    twist_deg = twist*180/pi; % In case I want to plot later.
end
% Make a blade comprised of the same section.
for i=1:1:numSections
    section(i) = bs;
end
b1 = blade(section,bladeMass,twist);
b2 = blade(section,bladeMass,twist);
b3 = blade(section,bladeMass,twist);

% rotor
rotor = rotor([b1,b2,b3]);
rotor.setID(1); % Says this is the first rotor in the vehicle.

% vehicle body - Has no mass or inertia in single-rotor case, but may have
% geometry (i.e. tether point off of center mass)
vbod = vehiclebody(vbmass,I);

% vehicle
%v = vehicle(rotor,vbod,vbcentermass,vbtetherpoint,vbbuoypoint,vbbuoyforce);
v = vehicle;
v.init(vbod,rotor,rotpoint,vbcentermass,vbtetherpoint,vbbuoypoint);
rotor.connectVehicle(v);

%% Set-up simulation
tspan = 0:tstep:totalSimTime;

% Initial states
% State vector
%    1      2     3    4  5  6     7     
% [theta, gamma, beta, x, y, z, theta_dot,
%     8          9      10      11     12
% gamma_dot, beta_dot, x_dot, y_dot, z_dot]
x0 = [(90-initialYaw)*pi/180;initialPitch*pi/180;0;-v.tetherpoint(1);-v.tetherpoint(2)+initialLateral;-v.tetherpoint(3)-0.02;0;0;0;0;0;0];
water.velocity = fluidVelocity;

% Need to set rotor position. This will happen automatically in the state
% file from now on. 
% todo(rodney) fix this when you move ot vehicle. Specifically need to set
% the position of the rotor in the vehicle frame.
v.position = [x0(4);x0(5);x0(6)];
v.orientation = [x0(1);x0(2);x0(3)];
v.velocity = [x0(10);x0(11);x0(12)];
v.angvel = [cos(x0(3)) cos(x0(2))*sin(x0(3)) 0; -sin(x0(3)) cos(x0(2))*cos(x0(3)) 0; 0 -sin(x0(2)) 1]*[x0(7);x0(8);x0(9)];

% Function Handle
fnhndl = @rotorStateSimple;

disp('Running the simulation');
opts = odeset('RelTol',1e-5,'AbsTol',1e-6,'Stats','on','OutputFcn',@odeplot);
[t, y] = ode45(@(t,y) singleRotorVehicleState( t,y,v,water),tspan,x0,opts);

% Parse results for processing and saving
eulerAngs = [y(:,1),y(:,2),y(:,3)];
r_cmO_O = [y(:,4),y(:,5),y(:,6)];
eulerAngs_dot = [y(:,7),y(:,8),y(:,9)];
Ov_cmO_O = [y(:,10),y(:,11),y(:,12)];

%% Send results to file
if ~exist('products\data\','dir')
    mkdir('products\data\');
end
resultsfile = [pwd '\products\data\' inputfile(1:end-4) '_results.txt'];
fid = fopen(resultsfile,'w');
fprintf(fid, 'time theta gamma beta x1 x2 x3 dtheta dgamma dbeta u1 u2 u3\r\n');
dat = [t y].';
fprintf(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f\r\n',dat);
fclose(fid);

%% Make a movie
if makemovie
disp('Making a movie of the motion');
f1 = figure;
f1.Position = [0 0 0.9*1920 0.9*1080];
f1.Color = [1 1 1];
for i = 1:1:length(t)
    cthe = cos(y(i,1)); sthe = sin(y(i,1));
    cgam = cos(y(i,2)); sgam = sin(y(i,2));
    cbet = cos(y(i,3)); sbet = sin(y(i,3));
    B_C_O = [cbet*cthe + sbet*sgam*sthe, cgam*sbet, sbet*cthe*sgam - cbet*sthe;...
        cbet*sgam*sthe - sbet*cthe, cbet*cgam, sbet*sthe + cbet*cthe*sgam;...
        cgam*sthe,-sgam,cgam*cthe];
    O_C_B = transpose(B_C_O);
    r_b1cm_O = O_C_B*v.rotors.sectPos(:,numSections,1);
    r_b2cm_O = O_C_B*v.rotors.sectPos(:,numSections,2);
    r_b3cm_O = O_C_B*v.rotors.sectPos(:,numSections,3);
    figure(f1);
    plot3([r_cmO_O(i,1) r_cmO_O(i,1)+r_b1cm_O(1)],[r_cmO_O(i,2) r_cmO_O(i,2)+r_b1cm_O(2)],[r_cmO_O(i,3) r_cmO_O(i,3)+r_b1cm_O(3)],'r');
    hold on
    plot3([r_cmO_O(i,1) r_cmO_O(i,1)+r_b2cm_O(1)],[r_cmO_O(i,2) r_cmO_O(i,2)+r_b2cm_O(2)],[r_cmO_O(i,3) r_cmO_O(i,3)+r_b2cm_O(3)],'b');
    plot3([r_cmO_O(i,1) r_cmO_O(i,1)+r_b3cm_O(1)],[r_cmO_O(i,2) r_cmO_O(i,2)+r_b3cm_O(2)],[r_cmO_O(i,3) r_cmO_O(i,3)+r_b3cm_O(3)],'k');
    axis equal
    axis([-0.25 0.25 -0.25 0.25 -0.5 0.25]);
    
    view(-80,15)
    xlabel('x'); ylabel('y');
    %title(['\fontsize{20}U_\infty = ' num2str(0.5/(1+exp(-0.5*(t(i)-10))),2)]);
    %water.rampvelocity(t(i));
    title(['\fontsize{20}RPM = ' num2str(abs(y(i,9)/(2*pi)*60),'%5.2f'), ' U_\infty = ' num2str(water.velocity(1),'%5.2f')]);
    F(i) = getframe(f1);    
    hold off
end

% sm = input('Do you want to save the movie file? Y/N [Y]: ', 's');
% if isempty(sm)
%     sm = 'Y';
% end
% todo make an interface for v props
% if strcmp(sm,'Y')

    vw = VideoWriter(moviefile);
    vw.FrameRate = round(1/tstep)*speedfactor;
    %v.Quality = 100;% v.Width = 800; w.Height = 450;
    open(vw);
    writeVideo(vw,F); close(vw);
end

%% Make some plots
if makeplots
    imgfldr = 'products\images\';
    figure
    plot(t,y(:,1)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\theta (Yaw angle, degrees)')
    saveas(gcf,[imgfldr runname '_theta.png'])
    
    figure
    plot(t,y(:,2)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\gamma (Pitch angle, degrees)')
    saveas(gcf,[imgfldr runname '_gamma.png'])
    
    figure
    plot(t,y(:,3)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\beta (Azimuth angle, degrees)')
    saveas(gcf,[imgfldr runname '_beta.png'])
    
    figure
    plot(t,y(:,4),'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('x Position (m)')
    saveas(gcf,[imgfldr runname '_x.png'])
    
    figure
    plot(t,y(:,5),'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('y Position (m)')
    saveas(gcf,[imgfldr runname '_y.png'])
        
    figure
    plot(t,y(:,6),'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('z Position (m)')
    saveas(gcf,[imgfldr runname '_z.png'])
    
    figure
    plot(t,y(:,7)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\thetaDot (Yaw rate, deg/s)')
    saveas(gcf,[imgfldr runname '_thetadot.png'])
    
    figure
    plot(t,y(:,8)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\gammaDot (Pitch rate, deg/s)')
    saveas(gcf,[imgfldr runname '_gammadot.png'])
    
    figure
    plot(t,y(:,9)*180/pi,'LineWidth',2.0)
    %title('SG6040 Axis Parallel to Flow')
    xlabel('Time (s)')
    ylabel('\betaDot (Spin rate, deg/s)')
    saveas(gcf,[imgfldr runname '_betadot.png'])
end
