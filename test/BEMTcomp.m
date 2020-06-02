% BEMTcomp.m

clear all; close all; clc;

addpath('..\src');
cd ..\
% set-up - load depBaseline to compare to Aerodyn
inputfile = 'depBaseline.m';
sim = simulation(inputfile);
cd test

% Get torque and power with and without BEMT
% For without, use axial factor 1 and 2/3 (induction factor 0 and 1/3)
speed = 0:0.01:1.0;
for i=1:1:length(speed)    
    sim.vhcl.rotors(1).angvel = [0;0;speed(i)];
    sim.vhcl.rotors(1).computeHydroLoads(sim.fld);
    torqueBEMT(:,i) = sim.vhcl.rotors(1).torqueCM;
end
sim.vhcl.rotors(1).setBEMT([false,false]);
sim.vhcl.rotors(1).setAxialFlowFactor(1.0);
for i=1:1:length(speed)    
    sim.vhcl.rotors(1).angvel = [0;0;speed(i)];
    sim.vhcl.rotors(1).computeHydroLoads(sim.fld);
    torque(:,i) = sim.vhcl.rotors(1).torqueCM;
end
sim.vhcl.rotors(1).setAxialFlowFactor(0.667);
sim.vhcl.rotors(1).setTangentialFlowFactor(1.002);
for i=1:1:length(speed)    
    sim.vhcl.rotors(1).angvel = [0;0;speed(i)];
    sim.vhcl.rotors(1).computeHydroLoads(sim.fld);
    torqueAxInd(:,i) = sim.vhcl.rotors(1).torqueCM;
end
sim.vhcl.rotors(1).setAxialFlowFactor(0.5);
sim.vhcl.rotors(1).setTangentialFlowFactor(1.0);
for i=1:1:length(speed)    
    sim.vhcl.rotors(1).angvel = [0;0;speed(i)];
    sim.vhcl.rotors(1).computeHydroLoads(sim.fld);
    torqueAxSub(:,i) = sim.vhcl.rotors(1).torqueCM;
end
hfig = figure('Color','w');
temp1 = sim.vhcl.rotors(1).blades(1).length/norm(sim.fld.velocity);
temp2 = 0.5*sim.fld.density*pi*sim.vhcl.rotors(1).blades(1).length^2*norm(sim.fld.velocity);
plot(speed*temp1,torqueBEMT(3,:).*speed/temp2,'r');
hold on
plot(speed*temp1,torque(3,:).*speed/temp2,'b');
plot(speed*temp1,torqueAxInd(3,:).*speed/temp2,'g');
plot(speed*temp1,torqueAxSub(3,:).*speed/temp2,'k');
hold off
xlabel('TSR'); ylabel('C_P')
legend('BEMT','No Induction','Const. Optimal','Const. 0.5','Location','Best');
% Torque curves
hfig = figure('Color','w');
plot(speed*temp1,torqueBEMT(3,:)/temp2,'r');
hold on
plot(speed*temp1,torque(3,:)/temp2,'b');
plot(speed*temp1,torqueAxInd(3,:)/temp2,'g');
plot(speed*temp1,torqueAxSub(3,:)/temp2,'k');
hold off
xlabel('TSR'); ylabel('C_T_Q')
legend('BEMT','No Induction','Const. Optimal','Const. 0.5','Location','Best');

sim.vhcl.rotors(1).setAxialFlowFactor(0.667);
sim.vhcl.rotors(1).setTangentialFlowFactor(1.002);
sim.vhcl.rotors(1).setBladePitch(3.5*pi/180);
for i=1:1:length(speed)    
    sim.vhcl.rotors(1).angvel = [0;0;speed(i)];
    sim.vhcl.rotors(1).computeHydroLoads(sim.fld);
    torquePitch(:,i) = sim.vhcl.rotors(1).torqueCM;
end
% Torque curvesptich
hfig = figure('Color','w');
plot(speed*temp1,torqueBEMT(3,:)/temp2,'r');
hold on
plot(speed*temp1,torquePitch(3,:)/temp2,'b');
plot(speed*temp1,torqueAxInd(3,:)/temp2,'g');
plot(speed*temp1,torqueAxSub(3,:)/temp2,'k');
hold off
title('With pitch');
xlabel('TSR'); ylabel('C_T_Q')
legend('BEMT','Const. Opt. Pitch','Const. Optimal','Const. 0.5','Location','Best');