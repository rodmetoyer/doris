% DualRotorSim.m
% Simualtion script for a dual-rotor simulation

% Clear the workspace and add src to the Matlab search path
clear all; close all; clc;
addpath('src')

%% Run simulations in batches
% If you want to batch process use this block. Otherwise, set to false and
% process input files one at a time following the code that is after this
% block.
if false
    sweep = "ESS";
    modifyCases = true;
    itr = 1;
    for i=1:1:1
            inputfiles(itr) = strcat("case",num2str(i));
            itr = itr + 1;
    end
%     inputfiles = ["case1","case2","case3"];
    inputfiles = strcat(sweep,inputfiles,".m");
    meansimduration = 0;
    
    for i=1:1:numel(inputfiles)
        timerval = tic;
        sim = simulation(inputfiles(i));
        meansimduration = meansimduration + sim.duration;
        if modifyCases
            sim.setStaticICs; % 
            % perturbation for hydrostatic analysis
            sim.changeName([sim.name 'fromUnder'],'Sure');
            sim.vhcl.orientation(1) = sim.vhcl.orientation(1) - 3*pi/180;
        end
        
        %%
        sim.simulate('output',[]);
        %sim.simulate();
        Tsim(i) = toc(timerval);
        if ~sim.write2file            
            answr = questdlg('I didn''t write to file. Stop or keep going?','No Write','Stop','Keep Going','Stop');
            if strcmp(answr,'Stop')
                return;
            else
                disp('OK, moving on.');
            end
        end
        if sim.visuals.makeplots
            sim.makePlots(sim.name,'savefigs',true);
        end
        if sim.visuals.makemovie
            simulation.makeMovie(sim.name,'framerate',24,'speedfactor',sim.visuals.speedfactor);
        end
        clear sim;
    end
    meansimduration = meansimduration/i;
    hfig = bar(Tsim/meansimduration);
    return;
end

%% Run a single simulation
% The input file controls the simulation. Easiest thing to do is copy an
% exsiting file and rename it, then change the parameter values to make
% your simulation.
inputfiles = 'BLBcase1.m';
sim = simulation(inputfiles);

% For static equilibrium analysis you can use the setStaticICs method on a
% simulation object to put it into exactly the theoretical position and
% orientation for static equilibrium.
%sim.setStaticICs;

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
    disp('I didn''t write to file. I''m assuming you don''t want to make plots or movies either. Ending session.');
    return;
end

%% Plots
% There is a flag in the input file for plots and movies. It's there mainly
% for batch processing.
if sim.visuals.makeplots
    simulation.makePlots(sim.name,'axcolor','w','figcolor','w');
end
%simulation.makePlots(sim.name,'savefigs',true);

%% Make a video of the simulation results
% The makeMovie method is static. The first argument is the name of the
% data and input file combo to use. The second argument is the name of the
% movie file. If you only pass one name the movie file gets that name.
if sim.visuals.makemovie
    simulation.makeMovie(sim.name,'framerate',60,'speedfactor',sim.visuals.speedfactor);
end
