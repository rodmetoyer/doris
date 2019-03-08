%function passed = rotorUnitTest(fluid,test)

clearvars; close all; clc;

% runall = false; runperp = false; run45deg = false;
% if nargin < 2
%     answer = questdlg('Which rotor unit tests would you like to run?','Rotor Unit Tests','All','Perpendicular','45deg','All');
%     switch answer
%         case 'All'
%             runall = true;
%         case 'Perpendicular'
%             runperp = true;
%         case '45deg'
%             run45deg = true;
%         otherwise
%             error('Should never hit this. Time to panic.');
%     end
% else
%     switch test
%         case 'All'
%             runall = true;
%         case 'Perpendicular'
%             runperp = true;
%         case '45deg'
%             run45deg = true;
%         otherwise
%             error('Should never hit this. Time to panic.');
%     end
% end

% Build a rotor that is very much like the one we will use in the water
% tunnel
water = fluid;
water.velocity = [0.8;0;0];
% airfoils - same for the entire rotor so we just need one
af = airfoil(1,'S814');

% blade sections
aspectRatio = 7;
bladeLength = 4*0.0254; % inch*m/inch - 4 inch blades
numSections = 20;
bs = bladesection(bladeLength/aspectRatio,bladeLength/numSections,af);
for i=1:1:numSections
    section(i) = bs;
end

% blade
bladeMass = 0.02;
numBlades = 3;
twist = -[64.6,49.9,38.7,30.5,24.5,19.95,16.45,13.7,11.5,9.7,8.1,6.9,5.76,4.81,3.98,3.25,2.61,2.03,1.51,1.04];
twsit = -[82.0000   55.6167   37.2277   25.9006   18.7477   13.9589   10.5720    8.0661    6.1438    4.6256    3.3978    2.3853    1.5365    0.8149    0.1941   -0.3455   -0.8187   -1.2371   -1.6096   -1.9434];
b = blade(section,bladeMass,twist);
for i=1:1:numBlades
    blade(i) = b; % Note that this is a vector of the same handle
end

% rotor
rotor = rotor(blade);

% Set the rotor states such that it is not translating or rotating w.r.t.
% inertial space and is perpenducular to the flow.
rotor.position = [0;0;0];
rotor.velocity = [0;0;0];
rotor.orientation = [90;0;0];
rotor.angvel = [0;0;0];

% Compute the loads on the rotor at each blade section
[force, torque, vrels, lift, drag] = rotor.computeAeroLoadArrays(water);

% Plot:
    % Velocity vectors in the rotor frame
    % Force vectors in the rotor frame

cosbeta = cosd(rotor.orientation(3)); sinbeta = sind(rotor.orientation(3));
cosgamma = cosd(rotor.orientation(2)); singamma = sind(rotor.orientation(2));
costheta = cosd(rotor.orientation(1)); sintheta = sind(rotor.orientation(1));
B_C_O = [cosbeta*costheta + sinbeta*singamma*sintheta, cosgamma*sinbeta, sinbeta*costheta*singamma - cosbeta*sintheta;cosbeta*singamma*sintheta - sinbeta*costheta, cosbeta*cosgamma, sinbeta*sintheta + cosbeta*costheta*singamma;cosgamma*sintheta,-singamma,cosgamma*costheta];
Uinf_R = B_C_O*[water.velocity(1);water.velocity(2);water.velocity(3)];
% Relative velocities
figure
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),vrels(1,:,:),vrels(2,:,:),vrels(3,:,:),0,'c')
axis equal
title('Velocity Vectors');
legend('Freestream Velocity','Relative Velocity','Location','East');
view(-100,80)
hold off
% Lift and drag
figure
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0.1,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),lift(1,:,:),lift(2,:,:),lift(3,:,:),0,'g')
axis equal
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),drag(1,:,:),drag(2,:,:),drag(3,:,:),0,'r')
title('Lift and Drag Vectors');
legend('Freestream Velocity','Lift','Drag','Location','East');
view(-20,35)
hold off
% Total force
figure
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0.1,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),force(1,:,:),force(2,:,:),force(3,:,:),0,'k')
axis equal
title('Force Vectors');
legend('Freestream Velocity','Total Force','Location','East');
view(-20,35)
hold off

% Total Torque
figure
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0.1,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),torque(1,:,:),torque(2,:,:),torque(3,:,:),0,'k')
axis equal
title('Torque Vectors');
legend('Freestream Velocity','Total Force','Location','East');
view(-20,35)
hold off

% Net Force
[netforce1, nettorque1] = rotor.computeAeroLoadsBasic(water);
disp(['Force x = ' num2str(netforce1(1)) ' Force y = ' num2str(netforce1(2)) ' Force z = ' num2str(netforce1(3))]);
disp(['Torque x = ' num2str(nettorque1(1)) ' Torque y = ' num2str(nettorque1(2)) ' Torque z = ' num2str(nettorque1(3))]);


    
%% Now give the rotor some rotation
RPM = 150;
rad_per_s = RPM*1/60*2*pi;
rotor.angvel = [0;0;rad_per_s];
[force, torque, vrels, lift, drag] = rotor.computeAeroLoadArrays(water);

cosbeta = cosd(rotor.orientation(3)); sinbeta = sind(rotor.orientation(3));
cosgamma = cosd(rotor.orientation(2)); singamma = sind(rotor.orientation(2));
costheta = cosd(rotor.orientation(1)); sintheta = sind(rotor.orientation(1));
B_C_O = [cosbeta*costheta + sinbeta*singamma*sintheta, cosgamma*sinbeta, sinbeta*costheta*singamma - cosbeta*sintheta;cosbeta*singamma*sintheta - sinbeta*costheta, cosbeta*cosgamma, sinbeta*sintheta + cosbeta*costheta*singamma;cosgamma*sintheta,-singamma,cosgamma*costheta];
Uinf_R = B_C_O*[water.velocity(1);water.velocity(2);water.velocity(3)];
% Relative velocities
figure
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),vrels(1,:,:),vrels(2,:,:),vrels(3,:,:),0,'c')
axis equal
title('Velocity Vectors - rotation');
legend('Freestream Velocity','Relative Velocity','Location','East');
view(-100,80)
hold off
% Lift and drag
figure
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0.1,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),lift(1,:,:),lift(2,:,:),lift(3,:,:),0,'g')
axis equal
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),drag(1,:,:),drag(2,:,:),drag(3,:,:),0,'r')
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),force(1,:,:),force(2,:,:),force(3,:,:),0,'k')
title('Lift, Drag, and Total Force Vectors - rotation');
legend('Freestream Velocity','Lift','Drag','Total Force','Location','East');
view(-20,35)
hold off
% Total force
figure
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0.1,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),force(1,:,:),force(2,:,:),force(3,:,:),0,'k')
axis equal
title('Force Vectors - rotation');
legend('Freestream Velocity','Total Force','Location','East');
view(-20,35)
hold off
% Plot in the intertial frame too?
% Relative velocities
for i=1:1:rotor.numblades
    for j=1:1:rotor.blades(i).numsects
        vrels_O(:,j,i) = transpose(B_C_O)*vrels(:,j,i);
        pos_O(:,j,i) = transpose(B_C_O)*rotor.sectPos(:,j,i);
    end
end
figure
quiver3(0,0,0,water.velocity(1),water.velocity(2),water.velocity(3),0,'b');
hold on
quiver3(pos_O(1,:,:),pos_O(2,:,:),pos_O(3,:,:),vrels_O(1,:,:),vrels_O(2,:,:),vrels_O(3,:,:),0,'c')
axis equal
title('Velocity Vectors - rotation - Interial Frame');
legend('Freestream Velocity','Relative Velocity','Location','East');
view(-90,0)
hold off

% Rotational Torque
figure
tz = zeros(size(torque(1,:,:)));
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0.1,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),tz,tz,torque(3,:,:),0,'k')
axis equal
title('Torque Vectors - Rotation');
legend('Freestream Velocity','Rotational Torque Contributions','Location','East');
view(-20,35)
hold off

% Total Torque
figure
tz = zeros(size(torque(1,:,:)));
quiver3(0,0,0,Uinf_R(1),Uinf_R(2),Uinf_R(3),0.1,'b');
hold on
quiver3(rotor.sectPos(1,:,:),rotor.sectPos(2,:,:),rotor.sectPos(3,:,:),torque(1,:,:),torque(2,:,:),torque(3,:,:),0,'k')
axis equal
title('Torque Vectors');
legend('Freestream Velocity','Rotational Torque Contributions','Location','East');
view(-20,35)
hold off

[netforce2, nettorque2] = rotor.computeAeroLoadsBasic(water);
disp(['Force x = ' num2str(netforce2(1)) ' Force y = ' num2str(netforce2(2)) ' Force z = ' num2str(netforce2(3))]);
disp(['Torque x = ' num2str(nettorque2(1)) ' Torque y = ' num2str(nettorque2(2)) ' Torque z = ' num2str(nettorque2(3))]);
% figure
% quiver3(0,0,0,netforce2(1),netforce2(2),netforce2(3),0,'b');
% hold on
% quiver3(0,0,0,nettorque2(1),nettorque2(2),nettorque2(3),0,'r');
% axis equal
% title('Force Vectors - rotation - Rotor Frame');
% legend('Force','Moment','Location','East');
% hold off

%% Now lets look at some yaw



%% Movie of how the net force varies with rotational velocity