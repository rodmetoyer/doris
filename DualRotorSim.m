% DualRotorSim.m
% Simualtion script for a dual-rotor simulation

% Clear the workspace
clear all; close all; clc;
% Tell matlab to look in the src folder for the class files
addpath('src')

% If you want to batch process use this block. Otherwise, set to false and
% process input files one at a time following the code that is after this
% block.
if false
    sweep = "EFT";
    modifyCases = false;
    itr = 1;
    for i=61:1:66
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
            sim.vhcl.orientation(1) = sim.vhcl.orientation(1)*0.95;
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

% The input file controls the simulation. Easiest thing to do is copy an
% exsiting file and rename it, then change the parameter values to make
% your simulation.
% inputfile = 'utilityBaseline.m';
% inputfile = 'ballastTest.m';
% inputfile = 'rampedDemo.m';
% inputfile = 'sinusoidDemo.m';
% inputfile = 'disturbedDemo.m';
inputfiles = 'EFScase58Long.m';
sim = simulation(inputfiles);
sim.setStaticICs;

sim.changeName('EFScase58LongUnsteady','Sure');
unsteadyTheta = sim.vhcl.orientation(1) + pi;
weight = sim.fld.gravity*sim.vhcl.mass;
tension = weight*((1-sim.vhcl.relDensity)/sim.vhcl.relDensity);
stretch = tension/sim.thr.stiffness;
gam = unsteadyTheta - pi;
sim.vhcl.position = [-sim.vhcl.body.length/2*sin(gam);0;sim.thr.uslength+stretch-sim.vhcl.body.length/2*cos(gam)];
sim.vhcl.orientation(1) = unsteadyTheta;
sim.vhcl.orientation(1) = sim.vhcl.orientation(1) - 3*pi/180; % 3 degree perturbation
sim.changeName([sim.name 'fromUnder'],'Sure');

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
