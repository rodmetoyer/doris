% DualRotorSim.m
% Simualtion script for a dual-rotor simulation

% Clear the workspace
clear all; close all; clc;
% Tell matlab to look in the src folder for the class files
addpath('src')

% If you want to batch process use this block. Otherwise, set to false and
% process input files one at a time following the code that is after this
% block.
if true
    sweep = "FBL";
%     inputfiles = ["case1","case2","case3","case4","case5","case6","case7","case8","case9","case10","case11",...
%         "case12","case13","case14","case15","case16","case17","case18","case19","case20","case21","case22",...
%         "case23","case24","case25","case26","case27","case28","case29","case30","case31","case32","case33",...
%         "case34","case35","case36","case37","case38","case39","case40","case41","case42","case43","case44",...
%         "case45","case46","case47","case48","case49","case50","case51","case52","case53","case54","case55",...
%         "case56","case57","case58","case59","case60","case61","case62","case63","case64","case65","case66"];
    inputfiles = ["case95","case96","case97","case98","case99","case100","case101","case102","case103","case104",...
        "case105","case106","case107","case108"];
    inputfiles = strcat(sweep,inputfiles,".m");
    ssitr = 1;
    
    for i=1:1:numel(inputfiles)
        timerval = tic;
        sim = simulation(inputfiles(i));
        sim.simulate('output',[]);
        Tsim(i) = toc(timerval);
        if ~sim.write2file
            disp('I didn"t write to file. I" assuming you don"t want to make plots or movies either.');
            return;
        end
        if sim.visuals.makeplots
            sim.makePlots(sim.name,'savefigs',true);
        end
        if sim.visuals.makemovie
            simulation.makeMovie(sim.name,'framerate',24,'speedfactor',sim.visuals.speedfactor);
        end
        clear sim;
    end
    return;
end

% The input file controls the simulation. Easiest thing to do is copy an
% exsiting file and rename it, then change the parameter values to make
% your simulation.
% inputfile = 'utilityBaseline.m';
% inputfile = 'ballastTest.m';
% inputfile = 'rampedDemo.m';
% inputfile = 'sinusoidDemo.m';
% inputfile = 'disturbedDemo.m';
inputfiles = 'case15b.m';
sim = simulation(inputfiles);

%% Make sure the vehicle we just built is what we were trying to build.
% The showme method let's you visualize the simulation in its current
% state. Note that the state changes during simulation (obviously), so
% showme after a sim will look different than showme before a sim.
%hfig = sim.showme;
% There are also showmelift, showmedrag, and showmerotor methods
%uiwait(hfig) % wait until you close it to continue if you want a close look
%export_fig([plotsavepath '\figurename.png'],'-png','-transparent','-m3'); % save the figure if you want

%% Simulate
% No argument to the simulate method will default to simulation parameters
% specified in the input file and ode45 as the solver.
%tspan = 0:0.005:1;
sim.simulate('output',[]);
%sim.simulate('tspan',tspan,'stats','on');

%% Write simulation results to file
% If you want to name the results file something other than the simulation
% name just pass the filename as an argument with no extension (the method
% will append .txt).
if ~sim.write2file
    disp('I didn"t write to file. I" assuming you don"t want to make plots or movies either.');
    return;
end

%% Plots
simulation.makePlots(sim.name,'axcolor','w','figcolor','w');
%simulation.makePlots(sim.name,'savefigs',true);

%% Make a video of the simulation results
% The makeMovie method is static. The first argument is the name of the
% data and input file combo to use. The second argument is the name of the
% movie file. If you only pass one name the movie file gets that name.
if sim.makeMovie
    simulation.makeMovie(sim.name,'framerate',60,'speedfactor',10);
end
