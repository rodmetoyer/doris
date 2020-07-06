% analyzeSteadyState.m
% tool to analyze the steady-state skew angle and other steady stuff

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder
ballastPlots = false;
reldensPlots = true;

%% Ballast Only
if ballastPlots
% loop through the simulations of interest, load them up, get the numbers
inputfiles = ["caseBO1.m","caseBO2.m","caseBO3.m","caseBO4.m","caseBO5.m",...
        "caseBO6.m","caseBO7.m","caseBO8.m","caseBO9.m","caseBO10.m",...
        "case1.m","case2.m","case3.m","case4.m","case5.m",...
        "case6.m","case7.m","case8.m","case9.m","case10.m","case11.m"];
ssitr = 1;
steadyTolTime_s = 10;
steadyTolDeg = 1/10;
 for i=1:1:numel(inputfiles)
     sim = simulation.loadsim(inputfiles(i));
     theta = sim.states(:,4);
     gamma = sim.states(:,5);
     dtheta_1sec = theta(end) - theta(end-ceil(steadyTolTime_s/sim.timestep));
     dgamma_1sec = theta(end) - theta(end-ceil(steadyTolTime_s/sim.timestep));
        changemetric = sqrt(dtheta_1sec^2+dgamma_1sec^2);
        if changemetric > steadyTolDeg*pi/180 % if it has changed by 100th of a degree over the last steadyTolTime_s seconds
            disp('Steady state not reached for skew');
            nosteady(ssitr) = i;
            ssitr = ssitr + 1;
        end
        meantheta(i) = mean(theta(end-ceil(1/sim.timestep):end));
        meangamma(i) = mean(gamma(end-ceil(1/sim.timestep):end));
        skew(i) = acos(cos(meangamma(i))*sin(meantheta(i)));
        
        cm(:,i) = sim.vhcl.centermass/sim.vhcl.body.length;
        numBladesUpstream(i) = sim.vhcl.rotors(1).numblades;
        numBladesDownstream(i) = sim.vhcl.rotors(2).numblades;
        radUpsream(i) = sim.vhcl.rotors(1).blades(1).length;
        radDownsream(i) = sim.vhcl.rotors(2).blades(1).length;
        ffUpstream(i) = sim.vhcl.rotors(1).axflowfac;
        ffDownstream(i) = sim.vhcl.rotors(2).axflowfac;
        relativeDensity(i) = sim.vhcl.relDensity;
 end
skewfig = figure('Position',[100 100 400 400]);
skewax = axes('Parent',skewfig);
scatter(skewax,cm(3,:)*100,skew*180/pi,'*r');
c = skewax.Children.CData;
c = repmat(c,[length(skew) 1]);
try
c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
catch
    disp('Steady state reached for all case');
end
skewax.Children.CData = c;
xlabel('Center mass axial loction (%Body Length)');
ylabel('Skew angle (deg)');
skewax.Color = 'none';
skewax.FontSize = 12;
export_fig(skewfig,['products\analysis\ballastOnly\skew.png'],'-png','-transparent','-m3');

thetafig = figure('Position',[100 100 400 400]);
thetaax = axes('Parent',thetafig);
scatter(thetaax,cm(3,:)*100,meantheta*180/pi-90,'*r');
c = thetaax.Children.CData;
c = repmat(c,[length(meantheta) 1]);
try
c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
catch
    disp('Steady state reached for all case');
end
thetaax.Children.CData = c;
xlabel('Center mass axial loction (%Body Length)');
ylabel('Pitch angle (deg)');
thetaax.Color = 'none';
thetaax.FontSize = 12;
export_fig(thetafig,['products\analysis\ballastOnly\pitch.png'],'-png','-transparent','-m3');

gammafig = figure('Position',[100 100 400 400]);
gammaax = axes('Parent',gammafig);
scatter(gammaax,cm(3,:)*100,meangamma*180/pi,'*r');
c = gammaax.Children.CData;
c = repmat(c,[length(meangamma) 1]);
try
c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
catch
    disp('Steady state reached for all case');
end
gammaax.Children.CData = c;
xlabel('Center mass axial loction (%Body Length)');
ylabel('Yaw angle (deg)');
gammaax.Color = 'none';
gammaax.FontSize = 12;
export_fig(gammafig,['products\analysis\ballastOnly\yaw.png'],'-png','-transparent','-m3');

%  save(['products\data\' blockname '.mat'],'skew','nosteady','cm',...
%         'numBladesUpstream','numBladesDownstream','radUpsream','radDownsream',...
%         'ffUpstream','ffDownstream','relativeDensity');
end

%% Relative Density Only
if reldensPlots
% loop through the simulations of interest, load them up, get the numbers
clearvars
inputfiles = ["case1.m","case14.m","case15.m","case16.m","case17.m","case18.m",...
    "case19.m","case20.m","case21.m","case22.m","case23.m"];
ssitr = 1;
steadyTolTime_s = 10;
steadyTolDeg = 1/10;
 for i=1:1:numel(inputfiles)
     sim = simulation.loadsim(inputfiles(i));
     theta = sim.states(:,4);
     gamma = sim.states(:,5);
     dtheta_1sec = theta(end) - theta(end-ceil(steadyTolTime_s/sim.timestep));
     dgamma_1sec = theta(end) - theta(end-ceil(steadyTolTime_s/sim.timestep));
        changemetric = sqrt(dtheta_1sec^2+dgamma_1sec^2);
        if changemetric > steadyTolDeg*pi/180 % if it has changed by 10th of a degree over the last steadyTolTime_s seconds
            disp('Steady state not reached for skew');
            nosteady(ssitr) = i;
            ssitr = ssitr + 1;
        end
        meantheta(i) = mean(theta(end-ceil(1/sim.timestep):end));
        meangamma(i) = mean(gamma(end-ceil(1/sim.timestep):end));
        skew(i) = acos(cos(meangamma(i))*sin(meantheta(i)));
        
        cm(:,i) = sim.vhcl.centermass/sim.vhcl.body.length;
        numBladesUpstream(i) = sim.vhcl.rotors(1).numblades;
        numBladesDownstream(i) = sim.vhcl.rotors(2).numblades;
        radUpsream(i) = sim.vhcl.rotors(1).blades(1).length;
        radDownsream(i) = sim.vhcl.rotors(2).blades(1).length;
        ffUpstream(i) = sim.vhcl.rotors(1).axflowfac;
        ffDownstream(i) = sim.vhcl.rotors(2).axflowfac;
        relativeDensity(i) = sim.vhcl.relDensity;
 end
skewfig = figure('Position',[100 100 400 400]);
skewax = axes('Parent',skewfig);
scatter(skewax,relativeDensity,skew*180/pi,'*r');
c = skewax.Children.CData;
c = repmat(c,[length(skew) 1]);
try
c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
catch
    disp('Steady state reached for all case');
end
skewax.Children.CData = c;
xlabel('Relative Density');
ylabel('Skew angle (deg)');
skewax.Color = 'none';
skewax.FontSize = 12;
export_fig(skewfig,['products\analysis\relDensOnly\skew.png'],'-png','-transparent','-m3');

thetafig = figure('Position',[100 100 400 400]);
thetaax = axes('Parent',thetafig);
scatter(thetaax,relativeDensity,meantheta*180/pi-90,'*r');
c = thetaax.Children.CData;
c = repmat(c,[length(meantheta) 1]);
try
c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
catch
    disp('Steady state reached for all case');
end
thetaax.Children.CData = c;
xlabel('Relative Density');
ylabel('Pitch angle (deg)');
thetaax.Color = 'none';
thetaax.FontSize = 12;
export_fig(thetafig,['products\analysis\relDensOnly\pitch.png'],'-png','-transparent','-m3');


gammafig = figure('Position',[100 100 400 400]);
gammaax = axes('Parent',gammafig);
scatter(gammaax,relativeDensity,meangamma*180/pi,'*r');
c = gammaax.Children.CData;
c = repmat(c,[length(meangamma) 1]);
try
c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
catch
    disp('Steady state reached for all case');
end
gammaax.Children.CData = c;
xlabel('Relative Density');
ylabel('Yaw angle (deg)');
gammaax.Color = 'none';
gammaax.FontSize = 12;
export_fig(gammafig,['products\analysis\relDensOnly\yaw.png'],'-png','-transparent','-m3');


end %reldensplots

%  save(['products\data\' blockname '.mat'],'skew','nosteady','cm',...
%         'numBladesUpstream','numBladesDownstream','radUpsream','radDownsream',...
%         'ffUpstream','ffDownstream','relativeDensity');

% Go back to the tools folder
cd tools