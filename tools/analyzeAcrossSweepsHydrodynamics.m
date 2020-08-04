% analyzeAcrossSweeps.m

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder

imagedir = 'products\analysis\hydrodynamicOnly\';
color = [0.8510 0.8510 0.8510];
baseloc = 50;
moveright = 200;

reldenscols = 6;
sweeps = ["EAA","EFF","EBB","ECC"];
% cases = [1 12 23 34 45 56 2 13 24 35 46 57 3 14 25 36 47 58 4 15 26 37 48 59 5 16 27 38 49 60 6 17 28 39 50 61 10 21 32 43 54 65 11 22 33 44 55 66];
cases = [1 12 23 34 45 56 3 14 25 36 47 58 5 16 27 38 49 60 7 18 29 40 51 62 9 20 31 42 53 64 11 22 33 44 55 66];
numCases = length(cases);
itr = 1;
for j=1:1:numel(sweeps)
    for i=1:1:numCases
            inputfiles(itr) = strcat(sweeps(j),"case",num2str(cases(i)));
            itr = itr + 1;
    end
end
dp = getEqOrient(inputfiles,reldenscols);

% get data for plot pitch vs. wff for 6 relative densities
for i=1:1:numCases
    flowfactor(i,:) = [dp.wff(0*length(cases) + i) dp.wff(1*length(cases) + i) dp.wff(2*length(cases) + i) dp.wff(3*length(cases) + i)];
    pitch(i,:) = [dp.meanpitch(0*length(cases) + i) dp.meanpitch(1*length(cases) + i) dp.meanpitch(2*length(cases) + i) dp.meanpitch(3*length(cases) + i)];
    dPitchdFF(i) = (dp.meanpitch(3*length(cases) + i)-dp.meanpitch(0*length(cases) + i))/(dp.wff(3*length(cases) + i)-dp.wff(0*length(cases) + i));
end
 % make plots
for i=1:1:numCases/reldenscols
    hfig(i) = figure('Position',[baseloc+(i-1)*moveright 100 600 400],'Color',color);
    ax(i) = axes('Parent',hfig(i),'Color','none');
    hold(ax(i),'on');
    for j=(i-1)*reldenscols+1:1:i*reldenscols
        plot(ax(i),flowfactor(j,:),pitch(j,:),'-o','DisplayName',num2str(dp.relativeDensity(j)),'LineWidth',2.0);
    end
    hold(ax(i),'off');
    ax(i).XLim = [0.6 0.8];
    xlabel('Axial flow factor');
    ylabel('Pitch (deg)');
    title(['Axial Postion of CM = ' num2str(dp.cmaxial(i*reldenscols),'%3.2f')]);
    hleg = legend('Location','Best','Color',color);
    hleg.Title.String = 'Relative Density';
    export_fig(hfig(i),[imagedir 'pitchVff' num2str(i) '.png'],'-png','-transparent','-m3');
end

% Now we need the mean sensitivity of pitch to flow factor over the full range
% This is (pitch_last - pitch_first)/(ff_last - ff_first) dPitchdFF
hfig4 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',color);
ax4 = axes('Parent',hfig4,'Color','none');
hold(ax4,'on');
for i=1:1:numCases/reldenscols
    plot(ax4,dp.relativeDensity(1:6),dPitchdFF((i-1)*reldenscols+1:i*reldenscols),'-o','DisplayName',num2str(dp.cmaxial((i-1)*reldenscols+1),'%3.2f'),'LineWidth',2.0);
end
hold(ax4,'off');
ax4.XDir = 'reverse';
xlabel('Relative Density');
ylabel('dPitch/dFlowFactor (deg)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',color);
hleg.Title.String = 'Center Mass Location (%L_B)';
export_fig(hfig4,[imagedir 'dPitchdFFvRelDens.png'],'-png','-transparent','-m3');

cd tools
function datOut = getEqOrient(casenames,reldensCols)
    inputfiles = strcat(casenames,".m");    
    ballastRows = numel(inputfiles)/reldensCols;    
    ssitr = 1;
    steadyTolTime_s = 10;
    steadyTolDeg = 1/10;
    for i=1:1:numel(inputfiles)
        sim = simulation.loadsim(inputfiles(i));
        a1 = sim.vhcl.centermass(1)/sim.vhcl.body.length;
        depth = sim.states(:,3);
        drift = sim.states(:,2);
        streamwise = sim.states(:,1);
        theta = sim.states(:,4);
        gamma = sim.states(:,5);
        dtheta_1sec = theta(end) - theta(end-ceil(steadyTolTime_s/sim.timestep));
        dgamma_1sec = theta(end) - theta(end-ceil(steadyTolTime_s/sim.timestep));
           changemetric = sqrt(dtheta_1sec^2+dgamma_1sec^2);
           if changemetric > steadyTolDeg*pi/180 % if it has changed by 100th of a degree over the last steadyTolTime_s seconds
               disp(['Steady state not reached for skew case: ' char(inputfiles(i))]);
               nosteady(ssitr) = i;
               ssitr = ssitr + 1;
           end
           meanpitch(i) = mean(theta(end-ceil(1/sim.timestep):end));
           meanyaw(i) = mean(gamma(end-ceil(1/sim.timestep):end));
           meanskew(i) = acos(cos(meanyaw(i))*sin(meanpitch(i)));
           meandepth(i) = mean(depth(end-ceil(1/sim.timestep):end));
           meandrift(i) = mean(drift(end-ceil(1/sim.timestep):end));
           meanstreamwise(i) = mean(streamwise(end-ceil(1/sim.timestep):end));
           cm(:,i) = sim.vhcl.centermass/sim.vhcl.body.length;
           numBladesUpstream(i) = sim.vhcl.rotors(1).numblades;
           numBladesDownstream(i) = sim.vhcl.rotors(2).numblades;
           radUpsream(i) = sim.vhcl.rotors(1).blades(1).length;
           radDownsream(i) = sim.vhcl.rotors(2).blades(1).length;
           ffUpstream(i) = sim.vhcl.rotors(1).axflowfac;
           ffDownstream(i) = sim.vhcl.rotors(2).axflowfac;
           relativeDensity(i) = sim.vhcl.relDensity;
           wff(i) = sim.vhcl.rotors(1).axflowfac;
           lff(i) = sim.vhcl.rotors(2).axflowfac;
    end
    meanpitch = meanpitch*180/pi - 90;
    meanpitch(meanpitch > 180) = mod(meanpitch(meanpitch > 180),360)-360;
    meanpitch(meanpitch < -180) = mod(meanpitch(meanpitch < -180),360)-360;
    meanyaw = meanyaw*180/pi;

    datOut.meanpitch = meanpitch;
    datOut.meanyaw = meanyaw;
    datOut.meanskew = meanskew;
    datOut.cmaxial = cm(3,:)*100;
    datOut.cmradial = cm(1,:)*100;
    datOut.wff = wff;
    datOut.lff = lff;
    datOut.relativeDensity = relativeDensity;
end