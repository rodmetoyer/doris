% analyzeSteadyState.m
% tool to analyze the steady-state skew angle and other steady stuff
% pitch convention is postivie nose up

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder
ballastPlots = false;
reldensPlots = false;
% BLUplots = true;
% EFFplots = false;
sweep = "EFF";
%makeplots("BLU");
%makeplots("EFF");
makeplots("FBL");


%% Ballast Only
if ballastPlots
    % loop through the simulations of interest, load them up, get the numbers
    imagedir = ['products\analysis\' char(sweep) '\'];
    inputfiles = ["caseBO1Extended.m","caseBO2Extended.m","caseBO3Extended.m","caseBO4Extended.m","caseBO5Extended.m",...
            "case7Extended.m","case8Extended.m","case9Extended.m","case10Extended.m","case11Extended.m",...
            "BLUcase1.m","BLUcase2.m","BLUcase3.m","BLUcase4.m","BLUcase5.m",...
            "BLUcase6.m","BLUcase7.m","BLUcase8.m","BLUcase9.m","BLUcase10.m","BLUcase11.m"];
    ssitr = 1;
    steadyTolTime_s = 10;
    steadyTolDeg = 1/10;
     for i=1:1:numel(inputfiles)
         sim = simulation.loadsim(inputfiles(i));
         theta = sim.states(:,4);
         gamma = sim.states(:,5);
         p3 = sim.states(:,13);
         q3 = sim.states(:,15);
         omega3 = sim.states(:,9);
         p3int = p3 + omega3;
         q3int = q3 + omega3;
         relrotspeed = p3 - q3;
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
            meanwindrotspdint = mean(p3int(end-ceil(1/sim.timestep):end));
            meanleerotspdint = mean(q3int(end-ceil(1/sim.timestep):end));
            meanrelrotspd = mean(relrotspeed(end-ceil(1/sim.timestep):end));
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
    export_fig(skewfig,[imagedir 'ballastOnly\skew.png'],'-png','-transparent','-m3');

    pitchfig = figure('Position',[100 100 400 400]);
    pitchax = axes('Parent',pitchfig);
    scatter(pitchax,cm(3,:)*100,meantheta*180/pi-90,'*r','DisplayName','Sim Data');
    c = pitchax.Children.CData;
    c = repmat(c,[length(meantheta) 1]);
    try
    c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
    catch
        disp('Steady state reached for all case');
    end
    pitchax.Children.CData = c;
    % add a trend line
    [f,gof] = fit((cm(3,:)*100).',(meantheta*180/pi-90).','poly1');
    hline = refline(pitchax);
    hline.DisplayName = ['Linear Fit (R^2 = ' num2str(gof.rsquare,'%4.3f') ')'];
    hline.Color = 'b';
%     x = (cm(3,round(length(cm)/2))*100);
%     y = f.p1*x+f.p2;
%     str = {'Linear Fit',['R^2 = ' num2str(gof.rsquare,'%4.3f') '\rightarrow']};
%     text(thetaax,0.9*x,y,str);
    legend('Location','Best','Color','none');
    xlabel('Center mass axial loction (%Body Length)');
    ylabel('Pitch angle (deg)');
    pitchax.Color = 'none';
    pitchax.FontSize = 12;
    export_fig(pitchfig,[imagedir 'ballastOnly\pitch.png'],'-png','-transparent','-m3');

    yawfig = figure('Position',[100 100 400 400]);
    yawax = axes('Parent',yawfig);
    scatter(yawax,cm(3,:)*100,meangamma*180/pi,'*r','DisplayName','Sim Data');
    c = yawax.Children.CData;
    c = repmat(c,[length(meangamma) 1]);
    try
    c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
    catch
        disp('Steady state reached for all case');
    end
    yawax.Children.CData = c;
    hold(yawax,'on');
    x = cm(3,1):0.001:cm(3,end); x = x*100;
    y = f.p2 + f.p1*x;
    ppconst = 0.45;
    plot(yawax,x,ppconst*y.*cosd(y),'DisplayName','k*pitch*cos(pitch)','Color','b');
    hold(yawax,'off');
    legend('Location','Best','Color','none');
    xlabel('Center mass axial loction (%Body Length)');
    ylabel('Yaw angle (deg)');
    yawax.Color = 'none';
    yawax.FontSize = 12;
    export_fig(yawfig,[imagedir 'ballastOnly\yaw.png'],'-png','-transparent','-m3');

    %  save(['products\data\' blockname '.mat'],'skew','nosteady','cm',...
    %         'numBladesUpstream','numBladesDownstream','radUpsream','radDownsream',...
    %         'ffUpstream','ffDownstream','relativeDensity');
end

%% Relative Density Only
if reldensPlots
    clearvars -except ballastPlots reldensPlots
    imagedir = ['products\analysis\BLU\'];
% loop through the simulations of interest, load them up, get the numbers 
% inputfiles = ["RDEcase1ExtendedExtended.m","RDEcase2ExtendedExtended.m","RDEcase3ExtendedExtended.m",...
%     "RDEcase4ExtendedExtended.m","RDEcase5ExtendedExtended.m","RDEcase6ExtendedExtended.m","RDEcase7ExtendedExtended.m","RDEcase8ExtendedExtended.m",...
%     "BLUcase1.m","BLUcase12.m","BLUcase23.m","case16Extended.m","case17Extended.m","BLUcase56.m",...
%     "BLUcase45.m","BLUcase34.m","case21Extended.m","case22Extended.m","case23Extended.m",...
%     "RDEcase9ExtendedExtended.m","RDEcase10ExtendedExtended.m","RDEcase11ExtendedExtended.m","RDEcase12ExtendedExtended.m",...
%     "RDEcase13ExtendedExtended.m","RDEcase14ExtendedExtended.m","RDEcase15ExtendedExtended.m","RDEcase16ExtendedExtended.m",...
%     "RDEcase17ExtendedExtended.m","RDEcase18ExtendedExtended.m","RDEcase19ExtendedExtended.m"];

inputfiles = ["BLUcase1.m","BLUcase12.m","BLUcase23.m","case16Extended.m","case17Extended.m","BLUcase56.m",...
    "BLUcase45.m","BLUcase34.m","case21Extended.m","case22Extended.m","case23Extended.m"];

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
export_fig(skewfig,[imagedir 'relDensOnly\skew.png'],'-png','-transparent','-m3');

pitchfig = figure('Position',[100 100 400 400]);
pitchax = axes('Parent',pitchfig);
scatter(pitchax,relativeDensity,meantheta*180/pi-90,'*r');
c = pitchax.Children.CData;
c = repmat(c,[length(meantheta) 1]);
try
c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
catch
    disp('Steady state reached for all case');
end
pitchax.Children.CData = c;
xlabel('Relative Density');
ylabel('Pitch angle (deg)');
pitchax.Color = 'none';
pitchax.FontSize = 12;
export_fig(pitchfig,[imagedir 'relDensOnly\pitch.png'],'-png','-transparent','-m3');


yawfig = figure('Position',[100 100 400 400]);
yawax = axes('Parent',yawfig);
scatter(yawax,relativeDensity,meangamma*180/pi,'*r');
c = yawax.Children.CData;
c = repmat(c,[length(meangamma) 1]);
try
c(nosteady,:) = repmat([0 0 1],[length(nosteady) 1]);
catch
    disp('Steady state reached for all case');
end
yawax.Children.CData = c;
xlabel('Relative Density');
ylabel('Yaw angle (deg)');
yawax.Color = 'none';
yawax.FontSize = 12;
export_fig(yawfig,[imagedir 'relDensOnly\yaw.png'],'-png','-transparent','-m3');
end %reldensplots


%% Go back to the tools folder
cd tools

%% make plots function
function hfigs = makeplots(sweep)
    imagedir = ['products\analysis\' char(sweep) '\'];
    if ~exist(imagedir,'dir')
        mkdir(imagedir);
    end
    inputfiles = ["case1","case2","case3","case4","case5","case6","case7","case8","case9","case10","case11",...
        "case12","case13","case14","case15","case16","case17","case18","case19","case20","case21","case22",...
        "case23","case24","case25","case26","case27","case28","case29","case30","case31","case32","case33",...
        "case34","case35","case36","case37","case38","case39","case40","case41","case42","case43","case44",...
        "case45","case46","case47","case48","case49","case50","case51","case52","case53","case54","case55",...
        "case56","case57","case58","case59","case60","case61","case62","case63","case64","case65","case66"];
    inputfiles = strcat(sweep,inputfiles,".m");
    ballastRows = 11;
    reldensCols = 6;
    ssitr = 1;
    steadyTolTime_s = 10;
    steadyTolDeg = 1/10;
     for i=1:1:numel(inputfiles)
         sim = simulation.loadsim(inputfiles(i));
         depth = sim.states(:,3);
         drift = sim.states(:,2);
         streamwise = sim.states(:,1);
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
            meanpitch(i) = mean(theta(end-ceil(1/sim.timestep):end));
            meanyaw(i) = mean(gamma(end-ceil(1/sim.timestep):end));
            skew(i) = acos(cos(meanyaw(i))*sin(meanpitch(i)));
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
     end
    skewfig = figure('Position',[100 100 600 400]);
    skewax = axes('Parent',skewfig);
    % reshape does col1 row1:n ... colm row1:n where n is num rows m is num cols 
    cmn = reshape(cm(3,:),ballastRows,reldensCols);
    rdn = reshape(relativeDensity,ballastRows,reldensCols);
    skewn = reshape(skew,ballastRows,reldensCols);
    skewn_deg = skewn*180/pi;
    surf(skewax,cmn*100,rdn,skewn_deg,'FaceColor','interp','LineStyle','-');%,'Marker','o','MarkerFaceColor','r');
%     scatter3(skewax,cm(3,:)*100,relativeDensity,skew*180/pi);%,'Marker','o','MarkerFaceColor','r');
    view(skewax,-20,35);   
    xlabel('Center mass axial loction (%Body Length)');
    ylabel('Relative Density');
    zlabel('Skew angle (deg)');
    skewax.Color = 'none';
    skewax.FontSize = 12;
    export_fig(skewfig,[imagedir 'skewSurface.png'],'-png','-transparent','-m3');

    balfig = figure('Position',[100 100 600 400]);
    balax = axes('Parent',balfig);
    hold(balax,'on');
    plot(balax,cm(3,1:11)*100,skew(1:11)*180/pi,'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(balax,cm(3,12:22)*100,skew(12:22)*180/pi,'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(balax,cm(3,23:33)*100,skew(23:33)*180/pi,'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(balax,cm(3,34:44)*100,skew(34:44)*180/pi,'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(balax,cm(3,45:55)*100,skew(45:55)*180/pi,'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(balax,cm(3,56:66)*100,skew(56:66)*180/pi,'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    xlabel('Center mass axial loction (%Body Length)');
    ylabel('Skew angle (deg)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    balax.Color = 'none';
    balax.FontSize = 12;
    export_fig(balfig,[imagedir 'skewLine.png'],'-png','-transparent','-m3');
    
    rdfig = figure('Position',[100 100 600 400]);
    rdax = axes('Parent',rdfig);
    hold(rdax,'on');
    plot(rdax,relativeDensity([1,12,23,34,45,56]),skew([1,12,23,34,45,56])*180/pi,'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(cm(3,1)*100,'%2.1f'));
    plot(rdax,relativeDensity([2,13,24,35,46,57]),skew([2,13,24,35,46,57])*180/pi,'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(cm(3,2)*100,'%2.1f'));
    plot(rdax,relativeDensity([3,14,25,36,47,58]),skew([3,14,25,36,47,58])*180/pi,'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(cm(3,3)*100,'%2.1f'));
    plot(rdax,relativeDensity([4,15,26,37,48,59]),skew([4,15,26,37,48,59])*180/pi,'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(cm(3,4)*100,'%2.1f'));
    plot(rdax,relativeDensity([5,16,27,38,49,60]),skew([5,16,27,38,49,60])*180/pi,'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(cm(3,5)*100,'%2.1f'));
    plot(rdax,relativeDensity([6,17,28,39,50,61]),skew([6,17,28,39,50,61])*180/pi,'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(cm(3,6)*100,'%2.1f'));
    plot(rdax,relativeDensity([7,18,29,40,51,62]),skew([7,18,29,40,51,62])*180/pi,'-*m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(cm(3,7)*100,'%2.1f'));
    plot(rdax,relativeDensity([8,19,30,41,52,63]),skew([8,19,30,41,52,63])*180/pi,'-oc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(cm(3,8)*100,'%2.1f'));
    plot(rdax,relativeDensity([9,20,31,42,53,64]),skew([9,20,31,42,53,64])*180/pi,'-+k','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(cm(3,9)*100,'%2.1f'));
    plot(rdax,relativeDensity([10,21,32,43,54,65]),skew([10,21,32,43,54,65])*180/pi,'-sg','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(cm(3,10)*100,'%2.1f'));
    plot(rdax,relativeDensity([11,22,33,44,55,66]),skew([11,22,33,44,55,66])*180/pi,'-db','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(cm(3,11)*100,'%2.1f'));    
    xlabel('Relative Density');
    ylabel('Skew angle (deg)');
    hleg = legend('Location','North','Color','w');
    hleg.Title.String = 'CM Loc. (%L_B)';
    hleg.NumColumns = 6;
    hleg.FontSize = 10;
    rdax.Color = 'none';
    rdax.XDir = 'reverse';
    rdax.FontSize = 12;
    export_fig(rdfig,[imagedir 'skewLineRD.png'],'-png','-transparent','-m3');
    
    yawfig = figure('Position',[100 100 600 400]);
    yawax = axes('Parent',yawfig);
    hold(yawax,'on');
    plot(yawax,cm(3,1:11)*100,meanyaw(1:11)*180/pi,'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(yawax,cm(3,12:22)*100,meanyaw(12:22)*180/pi,'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(yawax,cm(3,23:33)*100,meanyaw(23:33)*180/pi,'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(yawax,cm(3,34:44)*100,meanyaw(34:44)*180/pi,'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(yawax,cm(3,45:55)*100,meanyaw(45:55)*180/pi,'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(yawax,cm(3,56:66)*100,meanyaw(56:66)*180/pi,'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    xlabel('Center mass axial loction (%Body Length)');
    ylabel('Yaw angle (deg)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    yawax.Color = 'none';
    yawax.FontSize = 12;
    export_fig(yawfig,[imagedir 'yawLine.png'],'-png','-transparent','-m3');
    
    ptchfig = figure('Position',[100 100 600 400]);
    ptchax = axes('Parent',ptchfig);
    hold(ptchax,'on');
    plot(ptchax,cm(3,1:11)*100,meanpitch(1:11)*180/pi-90,'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(ptchax,cm(3,12:22)*100,meanpitch(12:22)*180/pi-90,'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(ptchax,cm(3,23:33)*100,meanpitch(23:33)*180/pi-90,'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(ptchax,cm(3,34:44)*100,meanpitch(34:44)*180/pi-90,'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(ptchax,cm(3,45:55)*100,meanpitch(45:55)*180/pi-90,'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(ptchax,cm(3,56:66)*100,meanpitch(56:66)*180/pi-90,'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    xlabel('Center mass axial loction (%Body Length)');
    ylabel('Pitch angle (deg)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    ptchax.Color = 'none';
    ptchax.FontSize = 12;
    export_fig(ptchfig,[imagedir 'pitchLine.png'],'-png','-transparent','-m3');
    
    depthfig = figure('Position',[100 100 600 400]);
    depthax = axes('Parent',depthfig);
    cmn = reshape(cm(3,:),ballastRows,reldensCols);
    rdn = reshape(relativeDensity,ballastRows,reldensCols);
    meandepthn = reshape(meandepth,ballastRows,reldensCols);
    surf(depthax,cmn*100,rdn,meandepthn,'FaceColor','interp','LineStyle','-');%,'Marker','o','MarkerFaceColor','r');
    view(depthax,-20,35);   
    xlabel('CM_a_x_i_a_l (%Body Length)');
    ylabel('Relative Density');
    zlabel('Depth (m)');
    depthax.Color = 'none';
    depthax.FontSize = 12;
    export_fig(depthfig,[imagedir 'depthSurface.png'],'-png','-transparent','-m3');
    
    dlinefig = figure('Position',[100 100 600 400]);
    dlineax = axes('Parent',dlinefig);
    hold(dlineax,'on');
    plot(dlineax,cm(3,1:11)*100,meandepth(1:11),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(dlineax,cm(3,12:22)*100,meandepth(12:22),'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(dlineax,cm(3,23:33)*100,meandepth(23:33),'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(dlineax,cm(3,34:44)*100,meandepth(34:44),'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(dlineax,cm(3,45:55)*100,meandepth(45:55),'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(dlineax,cm(3,56:66)*100,meandepth(56:66),'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    xlabel('Center mass axial loction (%Body Length)');
    ylabel('Depth (m)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dlineax.Color = 'none';
    dlineax.FontSize = 12;
    export_fig(dlinefig,[imagedir 'depthLine.png'],'-png','-transparent','-m3');
    
    dpitchfig = figure('Position',[100 100 600 400]);
    dpitchax = axes('Parent',dpitchfig);
    hold(dpitchax,'on');
    plot(dpitchax,meanpitch(1:11)*100,meandepth(1:11),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(dpitchax,meanpitch(12:22)*100,meandepth(12:22),'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(dpitchax,meanpitch(23:33)*100,meandepth(23:33),'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(dpitchax,meanpitch(34:44)*100,meandepth(34:44),'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(dpitchax,meanpitch(45:55)*100,meandepth(45:55),'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(dpitchax,meanpitch(56:66)*100,meandepth(56:66),'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    xlabel('Pitch Angle (Deg)');
    ylabel('Depth (m)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dpitchax.Color = 'none';
    dpitchax.FontSize = 12;
    export_fig(dpitchfig,[imagedir 'depthPitchLine.png'],'-png','-transparent','-m3');
    
    dyawfig = figure('Position',[100 100 600 400]);
    dyawax = axes('Parent',dyawfig);
    hold(dyawax,'on');
    plot(dyawax,meanyaw(1:11)*100,meandepth(1:11),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(dyawax,meanyaw(12:22)*100,meandepth(12:22),'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(dyawax,meanyaw(23:33)*100,meandepth(23:33),'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(dyawax,meanyaw(34:44)*100,meandepth(34:44),'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(dyawax,meanyaw(45:55)*100,meandepth(45:55),'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(dyawax,meanyaw(56:66)*100,meandepth(56:66),'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    xlabel('Yaw Angle (Deg)');
    ylabel('Depth (m)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dyawax.Color = 'none';
    dyawax.FontSize = 12;
    export_fig(dyawfig,[imagedir 'depthYawLine.png'],'-png','-transparent','-m3');
    
    dskewfig = figure('Position',[100 100 600 400]);
    dskewax = axes('Parent',dskewfig);
    hold(dskewax,'on');
    plot(dskewax,skew(1:11)*100,meandepth(1:11),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(dskewax,skew(12:22)*100,meandepth(12:22),'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(dskewax,skew(23:33)*100,meandepth(23:33),'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(dskewax,skew(34:44)*100,meandepth(34:44),'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(dskewax,skew(45:55)*100,meandepth(45:55),'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(dskewax,skew(56:66)*100,meandepth(56:66),'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    xlabel('Skew Angle (Deg)');
    ylabel('Depth (m)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dskewax.Color = 'none';
    dskewax.FontSize = 12;
    export_fig(dskewfig,[imagedir 'depthSkewLine.png'],'-png','-transparent','-m3');    
    % Zoom to region of interest
    dskewax.XLim = [0 35];
    dskewax.YLim = [-60 0];
    dskewax.XGrid = 'on';
    dskewax.YGrid = 'on';
    export_fig(dskewfig,[imagedir 'depthSkewLineZoom.png'],'-png','-transparent','-m3'); 
    
    driftfig = figure('Position',[100 100 600 400]);
    driftax = axes('Parent',driftfig);
    cmn = reshape(cm(3,:),ballastRows,reldensCols);
    rdn = reshape(relativeDensity,ballastRows,reldensCols);
    meandriftn = reshape(meandrift,ballastRows,reldensCols);
    surf(driftax,cmn*100,rdn,meandriftn,'FaceColor','interp','LineStyle','-');%,'Marker','o','MarkerFaceColor','r');
    view(driftax,-20,35);   
    xlabel('CM_a_x_i_a_l (%Body Length)');
    ylabel('Relative Density');
    zlabel('Drift (m)');
    driftax.Color = 'none';
    driftax.FontSize = 12;
    export_fig(driftfig,[imagedir 'driftSurface.png'],'-png','-transparent','-m3');
    
    dftlinefig = figure('Position',[100 100 600 400]);
    dftlineax = axes('Parent',dftlinefig);
    hold(dftlineax,'on');
    plot(dftlineax,cm(3,1:11)*100,meandrift(1:11),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(dftlineax,cm(3,12:22)*100,meandrift(12:22),'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(dftlineax,cm(3,23:33)*100,meandrift(23:33),'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(dftlineax,cm(3,34:44)*100,meandrift(34:44),'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(dftlineax,cm(3,45:55)*100,meandrift(45:55),'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(dftlineax,cm(3,56:66)*100,meandrift(56:66),'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    xlabel('Center mass axial loction (%Body Length)');
    ylabel('Drift (m)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dftlineax.Color = 'none';
    dftlineax.FontSize = 12;
    export_fig(dftlinefig,[imagedir 'driftLine.png'],'-png','-transparent','-m3');
    
    ddfig = figure('Position',[100 100 600 400]);
    ddax = axes('Parent',ddfig);
    hold(ddax,'on');
    plot(ddax,meandepth(1:11),meandrift(1:11),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(ddax,meandepth(12:22),meandrift(12:22),'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(ddax,meandepth(23:33),meandrift(23:33),'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(ddax,meandepth(34:44),meandrift(34:44),'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(ddax,meandepth(45:55),meandrift(45:55),'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(ddax,meandepth(56:66),meandrift(56:66),'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
    plot(ddax,[min(min(meandepth),min(meandrift)) max(max(meandepth),max(meandrift))],[min(min(meandepth),min(meandrift)) max(max(meandepth),max(meandrift))],'-.k','DisplayName','Unity');
    xlabel('Depth (m)');
    ylabel('Drift (m)');
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    ddax.Color = 'none';
    ddax.FontSize = 12;
    export_fig(ddfig,[imagedir 'driftDepth.png'],'-png','-transparent','-m3');    
end