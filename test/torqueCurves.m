% torqueCurves.m

clearvars; close all; clc;

addpath('..\src');
[aoas814, cls814, cds814] = getcoeffs;
cd ..\
% set-up - load depBaseline to compare to Aerodyn
inputfile = 's814Compare.m';
sim = simulation(inputfile);
cd test
savefigs = true;
whichrotor = 1;

sim.vhcl.rotors(whichrotor).setBEMT([false,false]);
sim.vhcl.rotors(whichrotor).setAxialFlowFactor(0.82);
sim.vhcl.rotors(whichrotor).orientation = [0*pi/180;0;0]; % to check skew
sim.vhcl.rotors(whichrotor).setBladePitch(-3.5*pi/180);
sim.vhcl.angvel = [0;0;0];
sim.vhcl.velocity = [0;0;0];
sim.vhcl.orientation = [pi/2;0;0];
sim.fld.velocity = [1.5;0;0];
sim.vhcl.generator.setLoadResistance(0);
sim.vhcl.generator.setFlux(500);
aoa = -180:1:180;
for i=1:1:length(aoa)
    Cl(i) = ppval(sim.vhcl.rotors(1).blades(1).sections(1).airfoil.clpp,aoa(i));
    Cd(i) = ppval(sim.vhcl.rotors(1).blades(1).sections(1).airfoil.cdpp,aoa(i));
end
figure
plot(aoa,Cl,'b',aoa,Cd,'r',aoas814,cls814,':b',aoas814,cds814,':r');

%sim.vhcl.rotors(1).setAxialFlowFactor(1);
% Sweep from 0 to 18 RPM and see what the torque is with and without BEMT
itr = 1;
for RPM = 0:0.1:13.0    
    if whichrotor == 2
        RPM = -RPM;
    end
    speed(itr) = RPM*pi/30;
    speedRPM(itr) = RPM;
    sim.vhcl.rotors(whichrotor).angvel = [0;0;speed(itr)];
    sim.vhcl.rotors(whichrotor).computeHydroLoads(sim.fld);
    gtq = sim.vhcl.generator.getTorque(speed(itr));
    sim.vhcl.rotors(whichrotor).addTorque(gtq);
    torque(itr) = sim.vhcl.rotors(whichrotor).torqueCM(3);
    torque1(itr) = sim.vhcl.rotors(whichrotor).torqueCM(1);
    torque2(itr) = sim.vhcl.rotors(whichrotor).torqueCM(2);
    itr = itr + 1;
end
figure
if whichrotor == 2
    speedRPM = -speedRPM;
    torque = -torque;
end
plot(speedRPM,torque);
load('AerodynNoCorrections.mat')
figure
plot(results.RtSpeed,results.meanRtAeroMxh)
figure
toNumber = length(results.RtSpeed);
plot(speedRPM,torque,'r',results.RtSpeed(1:toNumber),results.meanRtAeroMxh(1:toNumber),'b');
xlabel('Rotor Speed (RPM)'); ylabel('Rotor Torque (Nm)');
legend('Our Model','Aerodyn','color','none');
set(gca,'color','none');
%export_fig('CompAerodynInduction02pitch7gen.png','-transparent');
figure
temp1 = pi/30*sim.vhcl.rotors(1).blades(1).length/sim.fld.velocity(1);
temp2 = 0.5*sim.fld.density*pi*sim.vhcl.rotors(1).blades(1).length^2*sim.fld.velocity(1);
plot(speedRPM*temp1,torque/temp2,'r',results.RtSpeed(1:toNumber)*temp1,results.meanRtAeroMxh(1:toNumber)/temp2,'b');
xlabel('TSR'); ylabel('C_T_R_Q')

function [a,l,d] = getcoeffs
    load('S814coeffs.mat');
    a = aoa;
    l = CL;
    d = CD;
end