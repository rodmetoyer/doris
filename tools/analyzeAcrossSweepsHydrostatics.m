% analyzeAcrossSweepsHydrostatics.m

clearvars; close all; clc;

imagedir = '..\products\analysis\hydrostaticOnly\';
imgNameAppnd = 'gray';
color = [0.8510 0.8510 0.8510];
baseloc = 50;
moveright = 200;

reldenscols = 6;
sweeps = ["EF3","EFF","EF2","EFT"];
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
    radLoc(i,:) = [dp.cmradial(0*length(cases) + i) dp.cmradial(1*length(cases) + i) dp.cmradial(2*length(cases) + i) dp.cmradial(3*length(cases) + i)];
    pitch(i,:) = [dp.meanpitch(0*length(cases) + i) dp.meanpitch(1*length(cases) + i) dp.meanpitch(2*length(cases) + i) dp.meanpitch(3*length(cases) + i)];
    dPitchdRadLoc(i) = (dp.meanpitch(3*length(cases) + i)-dp.meanpitch(0*length(cases) + i))/(dp.cmradial(3*length(cases) + i)-dp.cmradial(0*length(cases) + i));
end
 % make plots
for i=1:1:numCases/reldenscols
    hfig(i) = figure('Position',[baseloc+(i-1)*moveright 100 600 400],'Color',color);
    ax(i) = axes('Parent',hfig(i),'Color','none');
    hold(ax(i),'on');
    for j=(i-1)*reldenscols+1:1:i*reldenscols
        plot(ax(i),radLoc(j,:),pitch(j,:),'-o','DisplayName',num2str(dp.relativeDensity(j)),'LineWidth',2.0);
    end
    hold(ax(i),'off');
    %ax(i).XLim = [0.6 0.8];
    xlabel('Radial Location of CM (%L_B)');
    ylabel('Pitch (deg)');
    title(['Axial Ballast Location ' num2str(dp.ballLocAx(1,i*reldenscols)*100,3) '% L_B (a_3 = ' num2str(dp.cmaxial(i*reldenscols),3) '%)']);
    hleg = legend('Location','Best','Color',color);
    hleg.Title.String = 'Relative Density';
    export_fig(hfig(i),[imagedir 'pitchVradLoc' num2str(i) imgNameAppnd '.png'],'-png','-transparent','-m3');
end

% Now we need the mean sensitivity of pitch to flow factor over the full range
% This is (pitch_last - pitch_first)/(ff_last - ff_first) dPitchdFF
hfig4 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',color);
ax4 = axes('Parent',hfig4,'Color','none');
hold(ax4,'on');
for i=1:1:numCases/reldenscols
    plot(ax4,dp.relativeDensity(1:6),dPitchdRadLoc((i-1)*reldenscols+1:i*reldenscols),'-o','DisplayName',[num2str(dp.ballLocAx((i-1)*reldenscols+1)*100,3) '% L_B'],'LineWidth',2.0);
end
hold(ax4,'off');
ax4.XDir = 'reverse';
xlabel('Relative Density');
ylabel('\Delta\phi/\Delta\it{a_3} (deg)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','NorthWest','Color',color);
hleg.Title.String = 'Axial Ballast Location';
export_fig(hfig4,[imagedir 'dPitchdRadLocvRelDens' imgNameAppnd '.png'],'-png','-transparent','-m3');
