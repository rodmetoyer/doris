% extendSimulations.m
% Extends the simulation duration of exsiting cases

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder

%% Make an input file string array with cases that you want to extend
%%% Fast way is to do it in a loop
% sweep = "XB3"; % extending to see if we have another eq point
% itr = 1;
% for i=22:1:22
%     inputfiles(itr) = strcat("case",num2str(i));
%     itr = itr + 1;
% end
%%% and the append anything, like if this is the third time extending then
%%% you need to append ExtendedExtended and after the extension simulations
%%% you have 3 Extended's
% inputfiles = strcat(sweep,inputfiles,"ExtendedExtended.m");

%%% Or you can do them one at a time
inputfiles = strcat("BLBcase50Extended.m");
% Convergence typically happens faster is you use the mean of the states
% from teh last half of the simulation as initial conditions for the
% extension. But if you want you can make lasthalf false and then you'll
% use the final state from the last sim as the ICs for the extension.
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