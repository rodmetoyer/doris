% unitTests.m
% Runs unit tests on classes and modules

clearvars; close all; clc;
p = 0; % How much time to pause and look at figures

%% Make sure intantiation gives you the expected objects
pause on % Turn off if you don't want to pause for figures

% Fluid
f = fluid;
f.velocity = [1.0,0.0,0.0];
passed = fluidUnitTest(f);
if ~passed
    error('Fluid failed fluid unit test');
end

% Airfoil
af = airfoil(1,'S814');
figure
plot(af.clcurve(1,:),af.clcurve(2,:));
hold on
plot(af.cdcurve(1,:),af.cdcurve(2,:));
af2 = airfoil(2,'SG6040');
hold off
movegui(gcf,'northwest');
figure
plot(af2.clcurve(1,:),af2.clcurve(2,:));
hold on
plot(af2.cdcurve(1,:),af2.cdcurve(2,:));
hold off
movegui(gcf,'northeast')
afoops = airfoil(1000,'SG6040_35');
pause(p)
% close all;

% bladesection
% Building on the previous test. We have airfoil objects.
bs = bladesection(0.1,0.1,af);
bs1 = bladesection(0.1,0.1,af);
bs2 = bladesection(0.1,0.1,2,'SG6040');
passed = bladesectionUnitTest(bs,f,false);
if ~passed
    error('Bladesection failed unit test');
end

% blade
% Make a blade out of blade sections
% (root)<-bs->-<-bs1->-<-bs2->-<--bs3-->-<---bs4--->-<bs5>(tip)
bs3 = bladesection(0.1,0.2,af);
bs4 = bladesection(0.1,0.3,af);
bs5 = bladesection(0.1,0.05,af);
bsv = [bs, bs1, bs2, bs3, bs4, bs5];
expectedlength = 3*0.1 + 0.2 + 0.3 + 0.05;
twist = -[64.6,49.9,38.7,30.5,24.5,19.95,16.45,13.7,11.5,9.7,8.1,6.9,5.76,4.81,3.98,3.25,2.61,2.03,1.51,1.04];
blade1 = blade(bsv,1.1,twist);
if blade1.length ~= expectedlength
    error('Blade length error');
end
mag = 1.0;
angs = 0:10:180;
vx = cosd(angs);
vz = sind(angs);
vy = zeros(size(vx));
vr = [vx;vy;vz];
for i=1:1:numel(blade1.sections)
    [L(:,i),D(:,i),M(:,i)] = computeLoads(blade1.sections(i),vr(:,i),f); % You know this doesn't use twist right.
end
Lmag = vecnorm(L);
Dmag = vecnorm(D);
Mmag = vecnorm(M);
figure
plot(Lmag,'*-r');
hold on;
plot(Dmag,'*-b');
plot(Mmag,'*-c');
hold off
pause(p)
%close gcf

% Rotor
% first need to make a couple more blades (remember that these are handle
% classes, so copying does not create a new object).
% Actually, the rotor only needs one blade object. He'll just keep going to
% that same object to get preperties. Only need multiple objects if blade
% is different.
%blade2 = blade(bsv,1.1);
%blade3 = blade(bsv,1.1);
r = rotor([blade1,blade1,blade1]);
% Put him in a stationary fluid, fixed in space, perpendicular to flow at zero
% rotation
f.velocity = [0.0;0.0;0.0];
r.position = [0,0,0]; % Should get a warning for using a row vector
r.velocity = [0;0;0];
r.orientation = [90;0;0];
r.angvel = [0;0;0];
% Now see what the loads are. Should be nada.
%[force, torque] = r.computeAeroLoadsBasic(f);
% Now let's see what happens when we put it into slow flow. Should have
% most torque in the negative i_O direction and most force in the positive
% i_O direction.
f.velocity = [0.5;0.0;0.0];
[force, torque] = r.computeAeroLoadsBasic(f);


