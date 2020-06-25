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
    inputfile = ["utilityBaseline.m"];
    %inputfile = ["tacticalCase5","tacticalCase6","tacticalCase7"];
    for i=1:1:numel(inputfile)
        sim = simulation(inputfile(i));
        sim.simulate('output',[]);
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
        % save data in a workspace var for plotting
        
        % and we're done
        %close all;
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
sim = simulation(inputfile);

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
sim.simulate; % ('output',[])
%sim.simulate('tspan',tspan,'stats','on');

%% Write simulation results to file
% If you want to name the results file something other than the simulation
% name just pass the filename as an argument with no extension (the method
% will append .txt).
sim.write2file;

%% Plots
%simulation.makePlots(sim.name,'axcolor','w','figcolor','w');
simulation.makePlots(sim.name,'savefigs',true);

%% Make a video of the simulation results
% The makeMovie method is static. The first argument is the name of the
% data and input file combo to use. The second argument is the name of the
% movie file. If you only pass one name the movie file gets that name.
simulation.makeMovie(sim.name,'framerate',60,'speedfactor',10);
