% extendSimulations.m
% Extends the simulation duration of exsiting cases

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder

sweep = "RDE";
% inputfiles = ["case1","case2","case3","case4","case5","case6","case7","case8","case9","case10","case11",...
%     "case12","case13","case14","case15","case16","case17","case18","case19","case20","case21","case22",...
%     "case23","case24","case25","case26","case27","case28","case29","case30","case31","case32","case33",...
%     "case34","case35","case36","case37","case38","case39","case40","case41","case42","case43","case44",...
%     "case45","case46","case47","case48","case49","case50","case51","case52","case53","case54","case55",...
%     "case56","case57","case58","case59","case60","case61","case62","case63","case64","case65","case66"];
inputfiles = ["case1","case2","case3","case4","case5","case6","case7","case8","case9","case10","case11",...
    "case12","case13","case14","case15","case16","case17","case18","case19"];
inputfiles = strcat(sweep,inputfiles,"Extended.m");
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