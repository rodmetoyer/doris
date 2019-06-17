% BEMloadsTest.m
% Compares loads to aerodyn

clearvars; close all; clc;
addpath('..\src');

saveplots = true;
plotsavepath = [pwd '\tempfigs'];
% delete above here

if saveplots
    if ~exist(plotsavepath,'dir')
        mkdir(plotsavepath)
    end
end

% Make a rotor in a fluid
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
    b(i) = blade(section,bladeMass,twist);
end
clear twist bladeMass AoAopt_deg bladeDZfrac numBlades numSections i
% Make a set of rotors
r = rotor(b);
r.setID(1);
% Does the rotor visualization method work?
r.orientation = [0;0;0]; % Rotor orientation is w.r.t. the vehicle frame
rpm = 0;
r.angvel = [0;0;rpm/60*2*pi]; % Also w.r.t. the vehicle frame
% Need a vehicle to relate the rotor frame to inertial frame
vb = vehiclebody;
v = vehicle;
v.init(vb,r,[0;0;0],[0;0;0],[0;0;0],[0;0;0]);
v.orientation = [pi/2;0;0];
v.velocity = [0;0;0];
v.angvel = [0/60*2*pi;0/60*2*pi;0/60*2*pi];
r.connectVehicle(v);
% Once a vehicle is connected you want to access rotor attributes through
% him.
water.velocity = [0.5;0;0];
v.computeHydroLoads(water);
itr = 1;
for TSR=0:0.5:18
    omega(itr) = TSR*norm(water.velocity)/v.rotors(1).blades(1).length;
    v.rotors(1).angvel = [0;0;omega(itr)];
    v.computeHydroLoads(water);
    forceout(:,itr) = v.rotors(1).force;
    TQout(:,itr) = v.rotors(1).torqueCM;
    TSRout(itr) = TSR;
    %figs(itr) = v.rotors(1).visualizeSectionLoads(true,0);
    itr = itr + 1;
end
figure
plot(TSRout,forceout(3,:),'o');

figure
plot(TSRout,TQout(3,:),'o');

%% Movie

writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = 1;
open(writerObj);
% write the frames to the video
for i=1:numel(figs)
    % convert the image to a frame
    frame = getframe(figs(i));
    drawnow
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
