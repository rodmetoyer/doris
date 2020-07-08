% analyzeSteadyState.m
% tool to analyze the steady-state skew angle and other steady stuff

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder
ballastPlots = false;
reldensPlots = false;
ballastAndReldensPlots = true;

%% Ballast Only
if ballastPlots
% loop through the simulations of interest, load them up, get the numbers
inputfiles = ["caseBO1Extended.m","caseBO2Extended.m","caseBO3Extended.m","caseBO4Extended.m","caseBO5Extended.m",...
        "caseBO6Extended.m","caseBO7Extended.m","caseBO8Extended.m","caseBO9Extended.m","caseBO10Extended.m",...
        "case1Extended.m","case2Extended.m","case3Extended.m","case4Extended.m","case5Extended.m",...
        "case6Extended.m","case7Extended.m","case8Extended.m","case9Extended.m","case10Extended.m","case11Extended.m"];
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
    clearvars -except ballastPlots reldensPlots ballastAndReldensPlots
% loop through the simulations of interest, load them up, get the numbers 
inputfiles = ["case1Extended.m","case14Extended.m","case15Extended.m","case16Extended.m","case17Extended.m","case18Extended.m",...
    "case19Extended.m","case20Extended.m","case21Extended.m","case22Extended.m","case23Extended.m"];

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

%% ballast and relative density plots
if ballastAndReldensPlots
    clearvars -except ballastPlots reldensPlots ballastAndReldensPlots
    % loop through the simulations of interest, load them up, get the numbers
    sweep = "BLU";
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
            meantheta(i) = mean(theta(end-ceil(1/sim.timestep):end));
            meangamma(i) = mean(gamma(end-ceil(1/sim.timestep):end));
            skew(i) = acos(cos(meangamma(i))*sin(meantheta(i)));
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
    export_fig(balfig,[imagedir 'ballast.png'],'-png','-transparent','-m3');
    
    yawfig = figure('Position',[100 100 600 400]);
    yawax = axes('Parent',yawfig);
    hold(yawax,'on');
    plot(yawax,cm(3,1:11)*100,meangamma(1:11)*180/pi,'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(yawax,cm(3,12:22)*100,meangamma(12:22)*180/pi,'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(yawax,cm(3,23:33)*100,meangamma(23:33)*180/pi,'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(yawax,cm(3,34:44)*100,meangamma(34:44)*180/pi,'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(yawax,cm(3,45:55)*100,meangamma(45:55)*180/pi,'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(yawax,cm(3,56:66)*100,meangamma(56:66)*180/pi,'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
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
    plot(ptchax,cm(3,1:11)*100,90-meantheta(1:11)*180/pi,'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    plot(ptchax,cm(3,12:22)*100,90-meantheta(12:22)*180/pi,'-ob','MarkerEdgeColor','b','MarkerFaceColor','b','DisplayName',num2str(relativeDensity(12)));
    plot(ptchax,cm(3,23:33)*100,90-meantheta(23:33)*180/pi,'-+g','MarkerEdgeColor','g','MarkerFaceColor','g','DisplayName',num2str(relativeDensity(23)));
    plot(ptchax,cm(3,34:44)*100,90-meantheta(34:44)*180/pi,'-sk','MarkerEdgeColor','k','MarkerFaceColor','k','DisplayName',num2str(relativeDensity(34)));
    plot(ptchax,cm(3,45:55)*100,90-meantheta(45:55)*180/pi,'-dc','MarkerEdgeColor','c','MarkerFaceColor','c','DisplayName',num2str(relativeDensity(45)));
    plot(ptchax,cm(3,56:66)*100,90-meantheta(56:66)*180/pi,'-^m','MarkerEdgeColor','m','MarkerFaceColor','m','DisplayName',num2str(relativeDensity(56)));
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
end % ballastAndReldensPlots

%% Save and go back to tools folder
%  save(['products\data\' blockname '.mat'],'skew','nosteady','cm',...
%         'numBladesUpstream','numBladesDownstream','radUpsream','radDownsream',...
%         'ffUpstream','ffDownstream','relativeDensity');

% Go back to the tools folder
cd tools