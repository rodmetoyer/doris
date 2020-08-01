% extendSimulations.m
% Extends the simulation duration of exsiting cases

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder

sweep = "TCS";
itr = 1;
for i=34:1:44
    inputfiles(itr) = strcat("case",num2str(i));
    itr = itr + 1;
end
inputfiles = strcat(sweep,inputfiles,"fromUnder.m");

%inputfiles = ["BLUcase10Extended.m","BLUcase11Extended.m","BLLcase11.m","BLLcase22.m","DBBcase11Extended.m","DBBcase18Extended.m","DBBcase22Extended.m"];
%inputfiles = "EFScase11Extended.m";
lasthalf = true;

extendedTspan = 0:0.2:3600;
for i=1:1:numel(inputfiles)
    sim = simulation.loadsim(inputfiles(i));
    startpt = 1;
    if lasthalf
        numpts = length(sim.states(:,1));
        startpt = round(numpts/2);
    end
    % make the position and orinetation of the vehicle the mean over the last half of the simualtion.  
    sim.vhcl.orientation = [mean(sim.states(startpt:end,4));mean(sim.states(startpt:end,5));mean(sim.states(startpt:end,6))];
    sim.vhcl.position = [mean(sim.states(startpt:end,1));mean(sim.states(startpt:end,2));mean(sim.states(startpt:end,3))];
    sim.vhcl.rotors(1).angvel = [0;0;mean(sim.states(startpt:end,13))];
    sim.vhcl.rotors(2).angvel = [0;0;mean(sim.states(startpt:end,15))];
    sim.changeName([sim.name 'Extended'],'Sure');
    sim.simulate('tspan',extendedTspan,'stats','on','output',[]);
    sim.write2file;
end
 
% Go back to the tools folder
cd tools