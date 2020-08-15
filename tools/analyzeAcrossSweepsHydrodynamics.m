% analyzeAcrossSweeps.m

clearvars; close all; clc;

imagedir = '..\products\analysis\hydrodynamicOnly\allcases\';
color = [0.8510 0.8510 0.8510];
imgNameAppnd = 'gray';
baseloc = 50;
moveright = 200;

reldenscols = 6;
sweeps = ["EAA","EFF","EBB","ECC"];
casesPerSweep = [1 12 23 34 45 56 2 13 24 35 46 57 3 14 25 36 47 58 4 15 26 37 48 59 5 16 27 38 49 60 6 17 28 39 50 61 7 18 29 40 51 62 8 19 30 41 52 63 9 20 31 42 53 64 10 21 32 43 54 65 11 22 33 44 55 66];
%casesPerSweep = [1 12 23 34 45 56 3 14 25 36 47 58 5 16 27 38 49 60 7 18 29 40 51 62 9 20 31 42 53 64 11 22 33 44 55 66];
numCases = length(casesPerSweep);
mkrclrs = ["b","g","k","c","m","m","c","k","g","b","b","g","k","c","m","m","c","k","g","b"];
styls = ["-o","-+","-s","-d","-^",":o",":+",":s",":d",":^","-.o","-.+","-.s","-.d","-.^","--o","--+","--s","--d","--^"];
lnstyls = ["-","--",":","-."];
mrkrs = ["o","+","s","d","^","<","*",">","v","p","h","."];
itr = 1;
for j=1:1:numel(sweeps)
    for i=1:1:numCases
            inputfiles(itr) = strcat(sweeps(j),"case",num2str(casesPerSweep(i)));
            itr = itr + 1;
    end
end
dp = getEqOrient(inputfiles,reldenscols);

% get data for plot pitch vs. wff for 6 relative densities
for i=1:1:numCases
    flowfactor(i,:) = [dp.wff(0*numCases + i) dp.wff(1*numCases + i) dp.wff(2*numCases + i) dp.wff(3*numCases + i)];
    pitch(i,:) = [dp.meanpitch(0*numCases + i) dp.meanpitch(1*numCases + i) dp.meanpitch(2*numCases + i) dp.meanpitch(3*numCases + i)];
    % (4th sweep - 1st sweep)/flowfactor is delta pitch w.r.t. delta flow factor
    dPitchdFF(i) = (dp.meanpitch(3*numCases + i)-dp.meanpitch(0*numCases + i))/(dp.wff(3*numCases + i)-dp.wff(0*numCases + i));
end
 % make plots
 
for i=1:1:numCases/reldenscols
    hfig(i) = figure('Position',[baseloc+(i-1)*moveright 100 600 400],'Color',color);
    ax(i) = axes('Parent',hfig(i),'Color','none');
    hold(ax(i),'on');
    itr = 1;
    for j=(i-1)*reldenscols+1:1:i*reldenscols
        plot(ax(i),flowfactor(j,:),pitch(j,:),char(strcat(lnstyls(1),mrkrs(itr))),'DisplayName',num2str(dp.relativeDensity(j)),'LineWidth',2.0);
        itr = itr + 1;
    end
    hold(ax(i),'off');
    ax(i).XLim = [0.6 0.8];
    xlabel('Axial flow factor');
    ylabel('Pitch (deg)');
    %title(['Axial Postion of CM = ' num2str(dp.cmaxial(i*reldenscols),'%3.2f')]);
    title(['Axial Ballast Location ' num2str(dp.ballLocAx(1,i*reldenscols)*100,3) '% L_B (a_3 = ' num2str(dp.cmaxial(i*reldenscols),3) '%)']);
    hleg = legend('Location','Best','Color',color);
    hleg.Title.String = 'Relative Density';
    export_fig(hfig(i),[imagedir 'pitchVff' num2str(i) imgNameAppnd '.png'],'-png','-transparent','-m3');
end

%% With this one we are trying to see if the change in pitch w.r.t. flow
% factor is a function of just the pitch angle. Do this by comparing the 4
% sweeps.
hfig1 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',color);
ax1 = axes('Parent',hfig1,'Color','none');
hold(ax1,'on');
for i=1:1:length(sweeps)
    plot(ax1,pitch(:,i).',dPitchdFF,'*','DisplayName',num2str(dp.wff(i*numCases),2),'LineWidth',2.0);
end
hold(ax1,'off');
%ax1.XDir = 'reverse';
xlabel('Pitch (deg)');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('\partial\phi/\partial\it{FlowFactor} (deg)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',color);
hleg.Title.String = 'Axial Flow Factor';
export_fig(hfig1,[imagedir 'dPitchdFFvPitch' imgNameAppnd '.png'],'-png','-transparent','-m3');

%% Now we need the mean sensitivity of pitch to flow factor over the full range
% This is (pitch_last - pitch_first)/(ff_last - ff_first) dPitchdFF
hfig4 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',color);
ax4 = axes('Parent',hfig4,'Color','none');
hold(ax4,'on');
standout = "-";
for i=1:1:numCases/reldenscols
    standout = "-";
    if i == 4
        standout = "--";
    end
    plot(ax4,dp.relativeDensity(1:6),dPitchdFF((i-1)*reldenscols+1:i*reldenscols),char(strcat(standout,mrkrs(i))),'DisplayName',[num2str(dp.ballLocAx((i-1)*reldenscols+1)*100,3) '% L_B (a_3 = ' num2str(dp.cmaxial(i*reldenscols),3) '%)'],'LineWidth',2.0);
end
hold(ax4,'off');
ax4.XDir = 'reverse';
ax4.XLim = [0.95 1];
grid(ax4,'on');
xlabel('Relative Density');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('\Delta\phi/\Delta\it{FlowFactor} (deg)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','NorthWest','Color',color);
hleg.Title.String = 'Axial Ballast Location';
export_fig(hfig4,[imagedir 'dPitchdFFvRelDens' imgNameAppnd '.png'],'-png','-transparent','-m3');
