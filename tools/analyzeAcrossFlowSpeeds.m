% analyzeAcrossFlowSpeeds.m

clearvars; close all; clc;

imagedir = '..\products\analysis\flowspeed\';
% lgdcolor = [0.8510 0.8510 0.8510];
lgdcolor = 'w';
imgNameAppnd = 'whiteWide';
baseloc = 50;
moveright = 200;

%sweeps = ["BAL","BLE","BAH"];
sweeps = ["BVL","BLE","BVH"];

% assuming all sweeps have the same number of cases
numReldensCols = 6;
axialCMrows = 11;
numCases = numReldensCols*axialCMrows;
numSweeps = numel(sweeps);
mkrclrs = ["r","b","g","k","c","m","m","c","k","g","b","b","g","k","c","m","m","c","k","g","b"];
styls = ["-o","-+","-s","-d","-^",":o",":+",":s",":d",":^","-.o","-.+","-.s","-.d","-.^","--o","--+","--s","--d","--^"];
lnstyls = ["-","--",":","-."];
mrkrs = ["o","+","s","d","^","<","*",">","v","p","h","."];
itr = 1;
for i=1:1:numSweeps
    caseitr = 1;
    for j=1:1:numReldensCols
        for k=1:1:axialCMrows
%             inputfiles(itr) = strcat(sweeps(j),"case",num2str(casesPerSweep(i)));
            inputfiles(itr) = strcat(sweeps(i),"case",num2str(caseitr));
            itr = itr + 1;
            caseitr = caseitr + 1;
        end
    end
end
dp = getEqOrient(inputfiles,numReldensCols);
% Now we have the data from the cases in order from sweep 1 to nsweeps

% reshape data. I'm sure there is a Matlab way to do this using reshape.
for i=1:1:numSweeps
    for j=1:1:numReldensCols
        for k=1:1:axialCMrows
            indx = (i-1)*numReldensCols*axialCMrows+(j-1)*axialCMrows+k;
            flowSpeed(k,j,i) = dp.flowspeed(indx);
            skew(k,j,i) = dp.meanskew(indx);
            pitch(k,j,i) = dp.meanpitch(indx);
            axialLocCm(k,j,i) = dp.cmaxial(indx);
            relativeDensity(k,j,i) = dp.relativeDensity(indx);
            ballLocAx(k,j,i) = dp.ballLocAx(indx);
            meandepth(k,j,i) = dp.meandepth(indx);
            meandrift(k,j,i) = dp.meandrift(indx);
        end
    end
end
% change in skew w.r.t flowspeed
dPitchdFlowspeed = (pitch(:,:,1)-pitch(:,:,end))./(flowSpeed(:,:,1)-flowSpeed(:,:,end));
dSkewdFlowspeed = (skew(:,:,1)-skew(:,:,end))./(flowSpeed(:,:,1)-flowSpeed(:,:,end));

% loop over sweeps and make plots 
% tight_subplot(Nh, Nw, gap, marg_h, marg_w)
[hax,pos] = tight_subplot(2,numSweeps,[.025 .02],[0.1 .05],[.05 .05]);
hfig = hax(1).Parent;
hfig.Position = [100 100 900 500];
for i=1:1:numel(hax); hold(hax(i),'on'); end
indx = 1;
for i=1:1:numSweeps
    for j=1:1:numReldensCols
        meandepth2plot = meandepth(:,j,i);
        meandrift2plot = meandrift(:,j,i);
        skewdpth2plot = skew(:,j,i);
        skewdrft2plot = skew(:,j,i);
        [clinds1, ~] = find(meandepth2plot <= 0);
        [clinds2, ~] = find(meandrift2plot <= 0);
        meandepth2plot = meandepth2plot(clinds1);  
        meandrift2plot = meandrift2plot(clinds2);
        skewdpth2plot = skewdpth2plot(clinds1);
        skewdrft2plot = skewdrft2plot(clinds2);
        plot(hax(i),skewdpth2plot,meandepth2plot,char(strcat("-",mrkrs(j),mkrclrs(j))),'MarkerFaceColor',mkrclrs(j),'MarkerSize',6,'DisplayName',num2str(relativeDensity(1,j,i)));
        h(j) = plot(hax(numSweeps+i),skewdrft2plot,meandrift2plot,char(strcat("-",mrkrs(j),mkrclrs(j))),'MarkerFaceColor',mkrclrs(j),'MarkerSize',6,'DisplayName',num2str(relativeDensity(1,j,i)));
        indx = indx +1;
    end
    hax(i).XLim = [5 25]; hax(numSweeps+i).XLim = [5 25];
    hax(i).YLim = [-80 -10];
    hax(numSweeps+i).YLim = [-25 -5];
    grid(hax(i),'on'); grid(hax(numSweeps+i),'on');
    xticklabels(hax(numSweeps+i),'auto');    
    title(hax(i),['Flowspeed = ' num2str(flowSpeed(1,1,i)) ' (m/s)']);    
end
for i=1:1:numel(hax)
    hold(hax(i),'off');
    hax(i).Color = 'none';
    xl = xline(hax(i),15);
end
% set(hax(1:3),'XTickLabel','');
yticklabels(hax(1),'auto');
yticklabels(hax(4),'auto');
hax(1).YLabel.String = 'Depth (m)';
hax(4).XLabel.String = 'Skew Angle (deg)';
hax(5).XLabel.String = 'Skew Angle (deg)';
hax(6).XLabel.String = 'Skew Angle (deg)';
% set(hax([2 3 5 6]),'YTickLabel','');
hax(4).YLabel.String = 'Drift (m)';
hleg = legend(h,'Location','Best','Color',lgdcolor);
hleg.Title.String = 'Relative Density';
export_fig(hfig,[imagedir 'depthDriftSkewArrayImproved' imgNameAppnd '.png'],'-png','-transparent','-m3');

mkrclrs = ["b","g","k","c","m","m","c","k","g","b","b","g","k","c","m","m","c","k","g","b"];
%% Change in skew w.r.t. flow speed for different relative densities ploted against the axial location of center of mass
hfig1 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',lgdcolor);
ax1 = axes('Parent',hfig1,'Color','none');
hold(ax1,'on');
standout = "-";
% dSkewsFlowspeed(cmlocs,reldens)
% axialLocCm(cmlocs,reldens,sweep)
for i=1:1:numReldensCols
    plot(ax1,axialLocCm(:,i,1),dSkewdFlowspeed(:,i),char(strcat(standout,mrkrs(i))),...
        'LineWidth',2.0,'DisplayName',num2str(relativeDensity(1,i,1)));
end
hold(ax1,'off');
% ax1.XDir = 'reverse';
% ax1.XLim = [0.95 1];
grid(ax1,'on');
xlabel('Normalized CM Axial Location');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('{\Delta}Skew/\Delta\it{Flowspeed} (deg*s/m)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',lgdcolor);
hleg.Title.String = 'Relative Density';
export_fig(hfig1,[imagedir 'dSkewdFlowspeedvAxialLocCM' imgNameAppnd '.png'],'-png','-transparent','-m3');

hfig2 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',lgdcolor);
ax2 = axes('Parent',hfig2,'Color','none');
hold(ax2,'on');
standout = "-";
% dSkewsFlowspeed(cmlocs,reldens)
% axialLocCm(cmlocs,reldens,sweep)
for i=1:1:axialCMrows
    plot(ax2,relativeDensity(i,:,1),dPitchdFlowspeed(i,:),char(strcat(standout,mrkrs(i))),'LineWidth',2.0,'DisplayName',[num2str(ballLocAx(i,1,1)) ' %L_B']);
end
hold(ax2,'off');
% ax1.XDir = 'reverse';
% ax1.XLim = [0.95 1];
grid(ax2,'on');
xlabel('Relative Density');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('{\Delta}\phi/\Delta\it{Flowspeed} (deg*s/m)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',lgdcolor);
hleg.Title.String = 'Axial Ballast Location';
export_fig(hfig2,[imagedir 'dPitchdFlowspeedvReldens' imgNameAppnd '.png'],'-png','-transparent','-m3');

hfig3 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',lgdcolor);
ax3 = axes('Parent',hfig3,'Color','none');
hold(ax3,'on');
standout = "-";
% dSkewsFlowspeed(cmlocs,reldens)
% axialLocCm(cmlocs,reldens,sweep)
for i=1:1:axialCMrows
    plot(ax3,pitch(i,:,1),dPitchdFlowspeed(i,:),char(strcat(standout,mrkrs(i))),'LineWidth',2.0,'DisplayName',[num2str(ballLocAx(i,1,1)) ' %L_B']);
end
hold(ax3,'off');
% ax1.XDir = 'reverse';
% ax1.XLim = [0.95 1];
grid(ax3,'on');
xlabel('Pitch (deg)');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('{\Delta}\phi/\Delta\it{Flowspeed} (deg*s/m)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',lgdcolor);
hleg.Title.String = 'Axial Ballast Location';
export_fig(hfig3,[imagedir 'dPitchdFlowspeedvPitchCMs' imgNameAppnd '.png'],'-png','-transparent','-m3');

hfig4 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',lgdcolor);
ax4 = axes('Parent',hfig4,'Color','none');
hold(ax4,'on');
standout = "-";
% dSkewsFlowspeed(cmlocs,reldens)
% axialLocCm(cmlocs,reldens,sweep)
for i=1:1:numReldensCols
    plot(ax4,pitch(:,i,1),dPitchdFlowspeed(:,i),char(strcat(standout,mrkrs(i))),'LineWidth',2.0,'DisplayName',num2str(relativeDensity(1,i,1)));
end
hold(ax4,'off');
% ax1.XDir = 'reverse';
% ax1.XLim = [0.95 1];
grid(ax4,'on');
xlabel('Pitch (deg)');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('{\Delta}\phi/\Delta\it{Flowspeed} (deg*s/m)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',lgdcolor);
hleg.Title.String = 'Relative Density';
export_fig(hfig4,[imagedir 'dPitchdFlowspeedvPitchRelDens' imgNameAppnd '.png'],'-png','-transparent','-m3');

hfig5 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',lgdcolor);
ax5 = axes('Parent',hfig5,'Color','none');
hold(ax5,'on');
standout = "-";
% dSkewsFlowspeed(cmlocs,reldens)
% axialLocCm(cmlocs,reldens,sweep)
for i=1:1:axialCMrows
    plot(ax5,skew(i,:,1),dSkewdFlowspeed(i,:),char(strcat(standout,mrkrs(i))),'LineWidth',2.0,'DisplayName',[num2str(ballLocAx(i,1,1)) ' %L_B']);
end
hold(ax5,'off');
% ax1.XDir = 'reverse';
% ax1.XLim = [0.95 1];
grid(ax5,'on');
xlabel('Skew (deg)');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('{\Delta}Skew/\Delta\it{Flowspeed} (deg*s/m)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',lgdcolor);
hleg.Title.String = 'Axial Ballast Location';
export_fig(hfig5,[imagedir 'dSkewdFlowspeedvPitchCMs' imgNameAppnd '.png'],'-png','-transparent','-m3');

hfig6 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',lgdcolor);
ax6 = axes('Parent',hfig6,'Color','none');
hold(ax6,'on');
standout = "-";
% dSkewsFlowspeed(cmlocs,reldens)
% axialLocCm(cmlocs,reldens,sweep)
for i=1:1:numReldensCols
    plot(ax6,skew(:,i,1),dSkewdFlowspeed(:,i),char(strcat(standout,mrkrs(i))),'LineWidth',2.0,'DisplayName',num2str(relativeDensity(1,i,1)));
end
hold(ax6,'off');
% ax1.XDir = 'reverse';
% ax1.XLim = [0.95 1];
grid(ax6,'on');
xlabel('Skew (deg)');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('{\Delta}Skew/\Delta\it{Flowspeed} (deg*s/m)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',lgdcolor);
hleg.Title.String = 'Relative Density';
export_fig(hfig6,[imagedir 'dSkewdFlowspeedvPitchRelDens' imgNameAppnd '.png'],'-png','-transparent','-m3');

hfig7 = figure('Position',[baseloc+3*moveright 100 600 400],'Color',lgdcolor);
ax7 = axes('Parent',hfig7,'Color','none');
hold(ax7,'on');
standout = "-";
% dSkewsFlowspeed(cmlocs,reldens)
% axialLocCm(cmlocs,reldens,sweep)
for i=1:1:axialCMrows
    plot(ax7,relativeDensity(i,:,1),dSkewdFlowspeed(i,:),char(strcat(standout,mrkrs(i))),'LineWidth',2.0,'DisplayName',[num2str(ballLocAx(i,1,1)) ' %L_B']);
end
hold(ax7,'off');
% ax1.XDir = 'reverse';
% ax1.XLim = [0.95 1];
grid(ax7,'on');
xlabel('Relative Density');
%ylabel('dPitch/dFlowFactor (deg)');
ylabel('{\Delta}Skew/\Delta\it{Flowspeed} (deg*s/m)');
%title(['Axial Postion of CM = ' num2str(dp.cmaxial(2*reldenscols+1),'%3.2f')]);
hleg = legend('Location','Best','Color',lgdcolor);
hleg.Title.String = 'Axial Ballast Location';
export_fig(hfig7,[imagedir 'dSkewdFlowspeedvReldens' imgNameAppnd '.png'],'-png','-transparent','-m3');