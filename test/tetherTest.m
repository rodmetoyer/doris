% tetherTest.m
% Performs tether simulations using the tether class

clear all; close all; clc;
addpath('..\src');

%% Simulation 1 - basic
% This is a basic simualtion
tottime = 30;
timestep = 1/60;
%nnodes=[0 1 2 5 10];
nnodes = 0;
amp = 2.0;
omg = 2.1;
for i=1:1:length(nnodes)
    thr = runSim(nnodes(i),tottime,timestep,'air',amp,omg);
end

function thr = runSim(nnodes,tottime,timestep,ft,amp,omg)
% get objects
fld = fluid(ft);
thr = tether;
numnodes = nnodes;
thr.name = ['circTestFast' num2str(numnodes) fld.typeName];
length = 10;
radius = 0.01; % meters
theta = 90;
startA = [amp*sin(omg*0);amp*cos(omg*0);0];
startB = [length*cosd(theta)+amp*sin(omg*0);amp*cos(omg*0);-length*sind(theta)];
pts = [startA,startB];
modulus = 1.0e9;
stiffness = modulus*pi*radius^2/length;
dampfactor = 1.0;
relativeDensity = 997/fld.density;
endMass = 100;
%init(hobj,nnodes,endpoints,f,reldens,radius,stf,dmp,endmass)
thr.setVelocityA([0;0;0]);
thr.setVelocityB([0;0;0]);
thr.init(numnodes,pts,fld,relativeDensity,radius,stiffness,dampfactor,endMass);
thr.setLocationB(pts(:,2));
thr.setLocationA(pts(:,1));
hfig = thr.showme;
thr.computeTension(fld);
thr.tension
thr.setLocationB([0;0;-11]);
thr.computeTension(fld);
thr.tension
thr.setLocationB(startB); 
thr.setLocationA([0;0;1]);
thr.computeTension(fld);
thr.tension
thr.setLocationA(startA);

% ok looks good. setup the test sim
% first 12 states are positions and velocities of point masses A and B
% next 6n states are the tether node position and velocity
startAd = [amp*omg*cos(omg*0);amp*omg*sin(omg*0);0];
startBd = [0;0;0];
thr.setVelocityA(startAd);
thr.setVelocityB(startBd);
x0(1:3) = thr.endpnts(:,1); x0(4:6) = thr.endvels(:,1);
x0(7:9) = thr.endpnts(:,2); x0(10:12) = thr.endvels(:,2);
x0(13:12+3*numnodes) = thr.nodelocs;
x0(13+3*numnodes:12+6*numnodes) = thr.nodevels;
tspan = 0:timestep:tottime;
opts = odeset('OutputFcn',@odeplot); %'RelTol',1e-6,'AbsTol',1e-6,'Stats',p.Results.stats,
[t,y] = ode45(@(t,y) stateder(t,y,thr,fld,endMass,amp,omg),tspan,x0,opts);
% plot end point motion
figure
plot(t,y(:,9),'g');
% plot(t,y(:,7),'r',t,y(:,8),'b',t,y(:,9),'g');
%return
%thr.makeMovie('Adata',y(:,1:6),'Bdata',y(:,7:12),'tetherdata',y(:,13:end),'times',t,'framerate',24);
save(['output\tetherTestData\' thr.name],'t','y','thr');
end

function xdot = stateder(t,x,thr,fld,endmass,amp,omg)
    xdot = NaN(size(x));
    A = [x(1);x(2);x(3)];    
    Ad = [x(4);x(5);x(6)];
    B = [x(7);x(8);x(9)];
    Bd = [x(10);x(11);x(12)];
    viscDampEnd = 0.5*fld.density*norm(Bd)*Bd*pi/4*(thr.radius)^2;
    %viscDampEnd = [0;0;0];
    % update tether position and locations
    thr.setNodePosition(x(13:12+3*thr.numnodes));
    thr.setNodeVelocity(x(13+3*thr.numnodes:12+6*thr.numnodes));
    thr.setLocationA(A);
    thr.setVelocityA(Ad);
    thr.setLocationB(B);
    thr.setVelocityB(Bd);
    txdot = thr.computeTension(fld); % compute tension returns accelerations of nodes and populates tension property
    xdot(1) = Ad(1);
    xdot(2) = Ad(2);
    xdot(3) = Ad(3);
    xdot(4) = -amp*omg^2*sin(omg*t);
    xdot(5) = -amp*omg^2*cos(omg*t);
    xdot(6) = 0;
    xdot(7) = Bd(1);
    xdot(8) = Bd(2);
    xdot(9) = Bd(3);
    xdot(10) = 1/endmass*thr.tension(1,2)-viscDampEnd(1);
    xdot(11) = 1/endmass*thr.tension(2,2)-viscDampEnd(2);
    xdot(12) = 1/endmass*thr.tension(3,2)-viscDampEnd(3)-9.81;
    xdot(13:12+6*thr.numnodes) = txdot;
end