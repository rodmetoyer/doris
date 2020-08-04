% extendSimulations.m
% Extends the simulation duration of exsiting cases

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder

sweep = "FBL";
% itr = 1;
% for i=34:1:44
%     inputfiles(itr) = strcat("case",num2str(i));
%     itr = itr + 1;
% end
% inputfiles = strcat(sweep,inputfiles,"fromUnder.m");

inputfiles = ["case67","case68","case69","case70","case71","case72","case73","case1","case2",...
            "case74","case75","case76","case77","case78","case79","case80","case12","case13",...
            "case81","case82","case83","case84","case85","case86","case87","case23","case24",...
            "case88","case89","case90","case91","case92","case93","case94","case34","case35",...
            "case95","case96","case97","case98","case99","case100","case101","case45","case46",...
            "case102","case103","case104","case105","case106","case107","case108","case56","case57"];
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