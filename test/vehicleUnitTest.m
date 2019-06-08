%function pf = vehicleUnitTest(opts)
% Vehicle unit test.
% INPUTS:
   % opts = struct. containing options
        % opts.verbose = true
        % opts.resultsfile = 'airfoilUnitTest_results.txt'
        % opts.plots = false
        % opts.movie = false
% OUTPUTS
    % pf = pass/fail bool
    
%% delete when convert to function
clearvars; close all; clc;
addpath('..\src');
opts.verbose = true;
opts.resultsfile = '';
opts.plot = true;
opts.movie = false;
% todo add these fields to the opts struct
saveplots = true;
plotsavepath = [pwd '\tempfigs'];
% delete above here

if saveplots
    if ~exist(plotsavepath,'dir')
        mkdir(plotsavepath)
    end
end
    
% Don't need to see these warnings. We know they are going to happen.
warning('off','BLADE:construction');
warning('off','VEHICLE:construction');

% Build a vehicle operating in a fluid
water = fluid('water');
af = airfoil('SG6040');
bladeChord = 0.1; % Meters
sectionWidth  = 0.1; % Meters - making blade one meter long 10 sections
numSections = 10;
bs = bladesection(bladeChord,sectionWidth,af);
clear bladeChord sectionWidth

% Make a blade comprised of the same section.
bladeMass = 0.25;
AoAopt_deg = 8.0;
bladeDZfrac = 0.0;
numBlades = 3;
twist.AoAopt_deg = AoAopt_deg;
twist.numBlades = numBlades;
twist.bladeDZfrac = bladeDZfrac;
for i=1:1:numSections
    section(i) = bs;
end
for i=1:1:numBlades
    b1(i) = blade(section,bladeMass,twist);
end
warning('off','BLADE:construction');
for i=1:1:numBlades
    b2(i) = blade(section,bladeMass,twist);
    b2(i).reverseTwist;
end
clear twist bladeMass AoAopt_deg bladeDZfrac numBlades numSections i

% Make a set of rotors
r1 = rotor(b1);
r1.setID(1);
% Need to make rotor 2 have blades with twist 180-
r2 = rotor(b2);
r2.setID(2);
% Make a vehicle - all points in A frame
vbcentermass = [0;0;0];
vbtetherpoint = [-1.25;0;0];
vbbuoypoint = [0;0;0];
r1point = [0;0;1.25];
r2point = [0;0;-1.25];
rotPoints = [r1point,r2point];
vbmass = 0.8;
vbod = vehiclebody(vbmass);
v = vehicle;
v.init(vbod,[r1,r2],rotPoints,vbcentermass,vbtetherpoint,vbbuoypoint);
% Associate rotors with vehicle
r1.connectVehicle(v);
r2.connectVehicle(v);

% This is how you would get the rotor position in the vehicle from the
% vehicle object
for i=1:1:numel(v.rotors)
    position(:,i) = v.rotors(i).vehicle.rotorLocs(:,i);
end
if norm(r1point-position(:,1)) > 0 || norm(r2point-position(:,2))
    warning('Rotor positions do not match');
    pf = false;
    return
end

% Alternatively, since you already know which rotor is first and which is
% second, you can get the locations directly like this
position1 = v.rotorLocs(:,1);
position2 = v.rotorLocs(:,2);
% or like this
position1 = v.rotorLocs(:,v.rotors(1).ID);
position2 = v.rotorLocs(:,v.rotors(2).ID);

% clear out all the non-object variables so I don't accidentally use them
% and get an erroneous pass.
clear position position1 position2 i r1point r2point rotPoints vbbuoypoint vbtetherpoint vbmass vbtetherpoint vbcentermass

%% Ok, now we have a vehicle with a couple of rotors. Lets set some states and compute some loads
% First give the water some velocity in the x direction
water.velocity = [1;0;0];
% Now orient the vehicle - think of this as pitch-yaw-roll where 0 pitch
% points the axis of rotation of a coaxial vehicle towards the ground
pitch = 90; yaw = 0;
v.orientation = [90*pi/180;yaw*pi/180;0];
% Initial posotion of the center of mass in the inertial frame
v.position = [3;0;1];
% Initial velocity of the center of mass in the inertial frame
v.velocity = [0;0;0];
% Initial rotational velocity of the vehicle frame w.r.t. the inertial frame
v.angvel = [0;0;0];

% Now set the orientation of the rotors. From vehicle frame to rotor frame
% For a coaxial system, only the 3rd component will change during sim
v.rotors(1).orientation = [0;0;0];
v.rotors(2).orientation = [0;0;0];

% Set the angular velocity of the rotors. All 3rd for coax.
rpm = 0;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];

% I think we have enough information to compute the relative velocities at
% the blade sections and thus the aero loads.
% Make a method in vehicle that computes all aero loads
% Starts by getting forces from the rotors
% To visulaize the relative velocity vectors and the forces at the sections
% we need to get the positions of the sections in the inertial frame as
% well as the forces from the rotor objects. 
[rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,water);
%% First check is to plot the relative velocity vectors in the O frame
% No motion of the rotors or body means that we should see relative
% velocity vectors that are exactly the freestream velocity.


figure
scale = 0;
y = -1:0.5:1; z = 0:0.5:2;
[Y,Z] = meshgrid(y, z);
X = zeros(size(Y));
U = ones(size(X))*water.velocity(1); V = ones(size(Y))*water.velocity(2); W = ones(size(Z))*water.velocity(3);
%quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),scale,'b');
quiver3(X,Y,Z,U,V,W,scale,'b');
hold on
vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c')
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g')
axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','best','color','none');
view(-130,20)
hold off
ax = gca;
ax.FontSize = 10;
set(ax,'color','none');
if saveplots
    saveto = [plotsavepath '\still'];    
    export_fig(saveto, '-png', '-transparent');
end

% now lets give a little rotor rotation
rpm = 10;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;0/60*2*pi];
[rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,water);
figure
scale = 0;
y = -1:0.5:1; z = 0:0.5:2;
[Y,Z] = meshgrid(y, z);
X = zeros(size(Y));
U = ones(size(X))*water.velocity(1); V = ones(size(Y))*water.velocity(2); W = ones(size(Z))*water.velocity(3);
%quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),scale,'b');
quiver3(X,Y,Z,U,V,W,scale,'b');
hold on
vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c')
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g')
axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','best','color','none');
view(-130,20)
hold off
ax = gca;
ax.FontSize = 10;
set(ax,'color','none');
if saveplots
    saveto = [plotsavepath '\R1rotating'];
    export_fig(saveto, '-png', '-transparent');
end

rpm = 10;
v.rotors(1).angvel = [0;0;0/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];
[rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,water);
figure
scale = 0;
y = -1:0.5:1; z = 0:0.5:2;
[Y,Z] = meshgrid(y, z);
X = zeros(size(Y));
U = ones(size(X))*water.velocity(1); V = ones(size(Y))*water.velocity(2); W = ones(size(Z))*water.velocity(3);
%quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),scale,'b');
quiver3(X,Y,Z,U,V,W,scale,'b');
hold on
vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c')
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g')
axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','best','color','none');
view(-130,20)
hold off
ax = gca;
ax.FontSize = 10;
set(ax,'color','none');
if saveplots
    saveto = [plotsavepath '\R2rotating'];
    export_fig(saveto, '-png', '-transparent');
end

% Change the position of the second rotor in the vehicle frame and make
% both spin
rpm = 20;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];
v.rotors(2).orientation = [10;-20;60]*pi/180;
[rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,water);
figure('Position',[100 100 1000 700]);
scale = 0;
y = -1:0.5:1; z = 0:0.5:2;
[Y,Z] = meshgrid(y, z);
X = zeros(size(Y));
U = ones(size(X))*water.velocity(1); V = ones(size(Y))*water.velocity(2); W = ones(size(Z))*water.velocity(3);
%quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),scale,'b');
quiver3(X,Y,Z,U,V,W,scale,'b','LineWidth',2);
hold on
vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c','LineWidth',2)
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g','LineWidth',2)
axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','northeast','color','none');
view(-176,25)
hold off
ax = gca;
ax.FontSize = 10;
set(ax,'color','none');
if saveplots
    saveto = [plotsavepath '\BothRotating_R2Skewed'];
    export_fig(saveto, '-png', '-transparent');
end

% Now put rotor rotation back to zero and skew the vehicle a litte 
rpm = 0;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];
v.rotors(2).orientation = [0;0;60]*pi/180;
v.orientation = [90;20;0]*pi/180;
[rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,water);
figure('Position',[100 100 1000 700]);
scale = 0;
y = -1:0.5:1; z = 0:0.5:2;
[Y,Z] = meshgrid(y, z);
X = zeros(size(Y));
U = ones(size(X))*water.velocity(1); V = ones(size(Y))*water.velocity(2); W = ones(size(Z))*water.velocity(3);
%quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),scale,'b');
quiver3(X,Y,Z,U,V,W,scale,'b','LineWidth',2);
hold on
vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c','LineWidth',2)
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g','LineWidth',2)
axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','northeast','color','none');
view(-176,25)
hold off
ax = gca;
ax.FontSize = 10;
set(ax,'color','none');
if saveplots
    saveto = [plotsavepath '\skewVehicle_R2Skewed'];
    export_fig(saveto, '-png', '-transparent');
end

%% Parallel example
% Vehicle is rotated 90 degrees
rpm = 20;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];
v.orientation = [0;90;0]*pi/180;
v.rotors(1).orientation = [0;80;0]*pi/180;
v.rotors(2).orientation = [0;100;0]*pi/180;
[rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,water);
figure('Position',[100 100 1000 700]);
scale = 0;
y = -1:0.5:1; z = 0:0.5:2;
[Y,Z] = meshgrid(y, z);
X = zeros(size(Y));
U = ones(size(X))*water.velocity(1); V = ones(size(Y))*water.velocity(2); W = ones(size(Z))*water.velocity(3);
%quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),scale,'b');
quiver3(X,Y,Z,U,V,W,scale,'b','LineWidth',2);
hold on
vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c','LineWidth',2)
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g','LineWidth',2)
axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','northeast','color','none');
view(-140,17)
hold off
ax = gca;
ax.FontSize = 10;
set(ax,'color','none');
if saveplots
    saveto = [plotsavepath '\parallelaxis2'];
    export_fig(saveto, '-png', '-transparent');
end

%% Ok now we can look at some forces
% Vehicle is rotated 90 degrees
rpm = 0;
v.rotors(1).angvel = [0;0;rpm/60*2*pi];
v.rotors(2).angvel = [0;0;-rpm/60*2*pi];
v.orientation = [90;0;0]*pi/180;
v.rotors(1).orientation = [0;0;0]*pi/180;
v.rotors(2).orientation = [0;0;0]*pi/180;
[rp1o_O,rp2o_O,rap1_O,rap2_O,~,~,L1_O,D1_O,L2_O,D2_O] = getPlotArrays(v,water);

figure('Position',[100 100 1000 700]);
scale = 1.0;
%ratDL1 = max(D1_O,[],'all')/max(L1_O,[],'all');
%ratDL2 = max(D2_O,[],'all')/max(L2_O,[],'all');
y = -1:0.5:1; z = 0:0.5:2;
[Y,Z] = meshgrid(y, z);
X = zeros(size(Y));
U = ones(size(X))*water.velocity(1); V = ones(size(Y))*water.velocity(2); W = ones(size(Z))*water.velocity(3);
%quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),scale,'b');
quiver3(X,Y,Z,U,V,W,1.0*scale,'b','LineWidth',2);
hold on
vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),L1_O(1,:,:),L1_O(2,:,:),L1_O(3,:,:),scale,':','color',[0 1 0.3],'LineWidth',2)
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),L2_O(1,:,:),L2_O(2,:,:),L2_O(3,:,:),scale,'color',[0.3 1 0],'LineWidth',2)
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),D1_O(1,:,:),D1_O(2,:,:),D1_O(3,:,:),scale,':','color',[1 0.1 0.3],'LineWidth',2)
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),D2_O(1,:,:),D2_O(2,:,:),D2_O(3,:,:),scale,'color',[1 0.2 0.1],'LineWidth',2)
axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
legend({'Freestream Velocity','Vehicle Body','Lift Rotor 1','Lift Rotor 2','Drag Rotor 1','Drag Rotor 2'},'Location','northeast','color','none');
view(-140,17)
hold off
ax = gca;
ax.FontSize = 10;
set(ax,'color','none');
if saveplots
    saveto = [plotsavepath '\loads1'];
    export_fig(saveto, '-png', '-transparent');
end

%% Lets try to make an animation



%% functions
function [rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,L1_O,D1_O,L2_O,D2_O] = getPlotArrays(v,f)
    Urel1_P1 = v.rotors(1).computeHydroLoads(f);
    Urel2_P2 = v.rotors(2).computeHydroLoads(f);
    temp = size(v.rotors(1).sectPos);
    O_C_A = transpose(v.A_C_O);
    O_C_P1 = O_C_A*transpose(v.rotors(1).P_C_A);
    O_C_P2 = O_C_A*transpose(v.rotors(2).P_C_A);
    for i=1:1:temp(2)
        for j=1:1:temp(3)
            rap1_O(:,i,j) = O_C_P1*v.rotors(1).sectPos(:,i,j);
            rap2_O(:,i,j) = O_C_P2*v.rotors(2).sectPos(:,i,j);
            Urel1_O(:,i,j) = O_C_P1*Urel1_P1(:,i,j);
            Urel2_O(:,i,j) = O_C_P2*Urel2_P2(:,i,j);
            L1_O(:,i,j) = O_C_P1*v.rotors(1).sectLift(:,i,j);
            D1_O(:,i,j) = O_C_P1*v.rotors(1).sectDrag(:,i,j);
            L2_O(:,i,j) = O_C_P2*v.rotors(2).sectLift(:,i,j);
            D2_O(:,i,j) = O_C_P2*v.rotors(2).sectDrag(:,i,j);
        end
    end
    % Show the forces and/or velocity vectors as quivers applied at the section
    % locations. For that we need rp1o_O and rp2o_O
    rco_O = v.position;
    rp1c_O = O_C_A*v.rotorLocs(:,1);
    rp2c_O = O_C_A*v.rotorLocs(:,2);
    rp1o_O = rco_O + rp1c_O;
    rp2o_O = rco_O + rp2c_O;
end