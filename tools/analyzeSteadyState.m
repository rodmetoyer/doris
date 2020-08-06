% analyzeSteadyState.m
% tool to analyze the steady-state skew angle and other steady stuff
% pitch convention is postivie nose up

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder
ballastPlots = false;
reldensPlots = false;

%sweep = "EFF";

%%%%%% Ready to go sweeps %%%%%%%%
% makeplots("FBL","Extended");  % Four blades on leeward rotor
% makeplots("BLL","Extended");  % Baseline with lower induction (higher flow factor) on rear rotor
% makeplots("DBB","Extended");  % Radial distance of ballast is doubled
% makeplots("DB2","ExtendedExtended");  % Radial distance of ballast is half 
% makeplots("DB3","Extended");  % Radial distance of ballast is trippled
% makeplots("BLB","Extended");  % The baseline sweep
% makeplots("BL2");             % Windward flow factor is 0.6 instead of 0.8
% makeplots("BAH");             % BLL with higher flow speed
% makeplots("BAL");             % BLL with lower flow speed

%%% Equal flow factor sweeps
% makeplots("EFF");             % Equal flow factors  
% makeplots("EF2");             % Ballast location double
% makeplots("EF3");             % Ballast location half
% makeplots("EFT");             % Ballast location three times
% makeplots("EAA");             % Flow factors 0.6 instead of 0.667
% makeplots("EBB");             % Flow factors 0.7 instead of 0.667
% makeplots("ECC");             % Flow factors 0.8

%%% Still water (hydrostatics) cases
%%% Utility scale %%%
% makeplots("EFS","",true)
%%% Tactical scale %%%
% makeplots("TCS","",true);
% makeplots("TCS","fromUnder",true);
% makeplots("TCS","fromOver",true);

% function makeIsoPlots(sweep)
%     imagedir = ['products\analysis\' char(sweep) '\'];
%     if strcmp(sweep,"BLU")
%         inputfiles = ["caseBO1Extended.m","caseBO2Extended.m","caseBO3Extended.m","caseBO4Extended.m","caseBO5Extended.m",...
%             "case7Extended.m","case8Extended.m","case9Extended.m","case10Extended.m","case11Extended.m",...
%             "BLUcase1.m","BLUcase2.m","BLUcase3.m","BLUcase4.m","BLUcase5.m",...
%             "BLUcase6.m","BLUcase7.m","BLUcase8.m","BLUcase9.m","BLUcase10.m","BLUcase11.m"];
%     else
%         
%     end
% end

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
    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
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
    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
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
    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
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
function hfigs = makeplots(sweep,appnd,hydrostatic)
    if nargin < 2
        appnd = "";
        hydrostatic = false;
    end
    if nargin < 3
        hydrostatic = false;
    end

    imagedir = ['products\analysis\' char(sweep) '\'];
    if ~exist(imagedir,'dir')
        mkdir(imagedir);
    end
    
    reldensCols = 6;
    itr = 1;
    for i=1:1:66
        inputfiles(itr) = strcat("case",num2str(i),appnd);
        itr = itr + 1;
    end
    
    if strcmp(sweep,"BLU")
        %reldensCols = 10;
        reldensCols = 6;
        itr = 1;
        for i=1:1:11
            inputfiles(itr) = strcat("case",num2str(i),"Extended");
            itr = itr + 1;
        end
%         for i=67:1:110
%             inputfiles(itr) = strcat("case",num2str(i),"Extended");
%             itr = itr + 1;
%         end
        for i=12:1:66
            inputfiles(itr) = strcat("case",num2str(i));
            itr = itr + 1;
        end
    end
    
    if (strcmp(sweep,"TCS") && (strcmp(appnd,"fromUnder") || strcmp(appnd,"fromOver")))
        reldensCols = 1;
        clear inputfiles
        itr = 1; 
        for i=34:1:44
            inputfiles(itr) = strcat("case",num2str(i),appnd);
            itr = itr + 1;
        end
    end
%     if strcmp(sweep,'FBL')
%         numinfls = 108;
%     else
%         numinfls = 66;
%     end
%     inputfiles = "case1";
%     for i=2:1:numinfls
%         inputfiles = [inputfiles,strcat("case",num2str(i))];
%     end
    % todo just get whatever files in the data folder are prepended with
    % the sweep id instead of this hacky way
    if strcmp(sweep,'FBL')
%         inputfiles = ["case67","case68","case69","case70","case71","case72","case73","case1","case2","case3","case4","case5","case6","case7","case8","case9","case10","case11",...
%             "case74","case75","case76","case77","case78","case79","case80","case12","case13","case14","case15","case16","case17","case18","case19","case20","case21","case22",...
%             "case81","case82","case83","case84","case85","case86","case87","case23","case24","case25","case26","case27","case28","case29","case30","case31","case32","case33",...
%             "case88","case89","case90","case91","case92","case93","case94","case34","case35","case36","case37","case38","case39","case40","case41","case42","case43","case44",...
%             "case95","case96","case97","case98","case99","case100","case101","case45","case46","case47","case48","case49","case50","case51","case52","case53","case54","case55",...
%             "case102","case103","case104","case105","case106","case107","case108","case56","case57","case58","case59","case60","case61","case62","case63","case64","case65","case66"];
        inputfiles = ["case67","case68","case69","case70","case71","case72","case73","case1","case2",...
            "case74","case75","case76","case77","case78","case79","case80","case12","case13",...
            "case81","case82","case83","case84","case85","case86","case87","case23","case24",...
            "case88","case89","case90","case91","case92","case93","case94","case34","case35",...
            "case95","case96","case97","case98","case99","case100","case101","case45","case46",...
            "case102","case103","case104","case105","case106","case107","case108","case56","case57"];
    end
    
%     if strcmp(sweep,'EFS')
%         % Skip the relative density = 1 cases because it drifts too much
%         reldensCols = 5;
%         inputfiles = inputfiles(12:end);
%     end
    
%     if strcmp(sweep,'EF3')
%         % 5 cases need extension - todo run extensions
%         inputfiles(10) = strcat("case",num2str(i),"Extended");
%         inputfiles(11) = strcat("case",num2str(i),"Extended");
%         inputfiles(22) = strcat("case",num2str(i),"Extended");
%         inputfiles(33) = strcat("case",num2str(i),"Extended");
%         inputfiles(44) = strcat("case",num2str(i),"Extended");
%     end

    mkrclrs = ["b","g","k","c","m","m","c","k","g","b","b","g","k","c","m","m","c","k","g","b"];
    styls = ["-o","-+","-s","-d","-^",":o",":+",":s",":d",":^","-.o","-.+","-.s","-.d","-.^","--o","--+","--s","--d","--^"];
    mrkrs = ["+","s","d","^","<","*",">","v","p","h"];
    
    
    inputfiles = strcat(sweep,inputfiles,".m");    
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
    meanpitch = meanpitch*180/pi - 90;
    meanpitch(meanpitch > 180) = mod(meanpitch(meanpitch > 180),360)-360;
    meanpitch(meanpitch < -180) = mod(meanpitch(meanpitch < -180),360)-360;
    meanyaw = meanyaw*180/pi;
    baseloc = 50;
    moveright = 150;
    
    if ~hydrostatic
    %% Figure 1
    skewfig = figure('Position',[baseloc 100 600 400]);
    skewax = axes('Parent',skewfig);
    % reshape does col1 row1:n ... colm row1:n where n is num rows m is num cols 
    cmn = reshape(cm(3,:),ballastRows,reldensCols);
    rdn = reshape(relativeDensity,ballastRows,reldensCols);
    skewn = reshape(skew,ballastRows,reldensCols);
    skewn_deg = skewn*180/pi;
    surf(skewax,cmn*100,rdn,skewn_deg,'FaceColor','interp','LineStyle','-');%,'Marker','o','MarkerFaceColor','r');
%     scatter3(skewax,cm(3,:)*100,relativeDensity,skew*180/pi);%,'Marker','o','MarkerFaceColor','r');
    view(skewax,-20,35);   
    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
    ylabel('Relative Density');
    zlabel('Skew angle (deg)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    skewax.Color = 'none';
    skewax.FontSize = 12;
    grid(skewax,'on');
    export_fig(skewfig,[imagedir 'skewSurface.png'],'-png','-transparent','-m3');

    
    %% FIgure 2
    
    balfig = figure('Position',[baseloc+moveright 100 600 400]);
    balax = axes('Parent',balfig);
    hold(balax,'on');
    plot(balax,cm(3,1:ballastRows)*100,skew(1:ballastRows)*180/pi,'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(balax,cm(3,i*ballastRows+1:(i+1)*ballastRows)*100,skew(i*ballastRows+1:(i+1)*ballastRows)*180/pi,char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end
    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
    ylabel('Skew angle (deg)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    balax.Color = 'none';
    balax.FontSize = 12;
    grid(balax,'on');
    export_fig(balfig,[imagedir 'skewLine.png'],'-png','-transparent','-m3');
    
    %% figure 3
    rdfig = figure('Position',[baseloc+2*moveright 100 600 400]);
    rdax = axes('Parent',rdfig);
    hold(rdax,'on');
    %inds = [1,ballastRows+1,2*ballastRows+1,3*ballastRows+1,4*ballastRows+1,5*ballastRows+1];
    inds = [];
    for i=1:1:reldensCols
        inds = [inds,(i-1)*ballastRows+1];
    end
    plot(rdax,relativeDensity(inds),skew(inds)*180/pi,'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(cm(3,1)*100,'%2.1f'));
    for i=1:1:ballastRows-1
        inds = [];
        for j=1:1:reldensCols
            inds = [inds,(j-1)*ballastRows+1+i];
        end
        plot(rdax,relativeDensity(inds),...
            skew(inds)*180/pi,...
            char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(cm(3,(i+1))*100,'%2.1f'));
    end

    xlabel('Relative Density');
    ylabel('Skew angle (deg)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','North','Color','w');
    hleg.Title.String = 'CM Loc. (%L_B)';
    hleg.NumColumns = 6;
    hleg.FontSize = 10;
    rdax.Color = 'none';
    rdax.XDir = 'reverse';
    rdax.FontSize = 12;
    grid(rdax,'on');
    export_fig(rdfig,[imagedir 'skewLineRD.png'],'-png','-transparent','-m3');
    
    %% Figure 4
    yawfig = figure('Position',[baseloc+3*moveright 100 600 400]);
    yawax = axes('Parent',yawfig);
    hold(yawax,'on');
    plot(yawax,cm(3,1:ballastRows)*100,meanyaw(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(yawax,cm(3,i*ballastRows+1:(i+1)*ballastRows)*100,meanyaw(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end

    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
    ylabel('Yaw angle (deg)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    yawax.Color = 'none';
    yawax.FontSize = 12;
    grid(yawax,'on');
    export_fig(yawfig,[imagedir 'yawLine.png'],'-png','-transparent','-m3');
    
    %% figure 5
    ptchfig = figure('Position',[baseloc+4*moveright 100 600 400]);
    ptchax = axes('Parent',ptchfig);
    hold(ptchax,'on');
    plot(ptchax,cm(3,1:ballastRows)*100,meanpitch(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(ptchax,cm(3,i*ballastRows+1:(i+1)*ballastRows)*100,meanpitch(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end

    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
    ylabel('Pitch angle (deg)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    ptchax.Color = 'none';
    ptchax.FontSize = 12;
    ptchax.YLim = [-20 60];
    grid(ptchax,'on');
    export_fig(ptchfig,[imagedir 'pitchLine.png'],'-png','-transparent','-m3');
    
    %% figure 6
    depthfig = figure('Position',[baseloc+5*moveright 100 600 400]);
    depthax = axes('Parent',depthfig);
    cmn = reshape(cm(3,:),ballastRows,reldensCols);
    rdn = reshape(relativeDensity,ballastRows,reldensCols);
    meandepthn = reshape(meandepth,ballastRows,reldensCols);
    surf(depthax,cmn*100,rdn,meandepthn,'FaceColor','interp','LineStyle','-');%,'Marker','o','MarkerFaceColor','r');
    view(depthax,-145,40);   
    xlabel('CM_a_x_i_a_l (%Body Length)');
    ylabel('Relative Density');
    zlabel('Depth (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    depthax.Color = 'none';
    depthax.FontSize = 12;
    grid(depthax,'on');
    export_fig(depthfig,[imagedir 'depthSurface.png'],'-png','-transparent','-m3');
    
    %% fig 7
    dlinefig = figure('Position',[baseloc+6*moveright 100 600 400]);
    dlineax = axes('Parent',dlinefig);
    hold(dlineax,'on');
    plot(dlineax,cm(3,1:ballastRows)*100,meandepth(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(dlineax,cm(3,i*ballastRows+1:(i+1)*ballastRows)*100,meandepth(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end

    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
    ylabel('Depth (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dlineax.Color = 'none';
    dlineax.FontSize = 12;
    grid(dlineax,'on');
    export_fig(dlinefig,[imagedir 'depthLine.png'],'-png','-transparent','-m3');
    
    % figure 8
    dpitchfig = figure('Position',[baseloc 600 600 400]);
    dpitchax = axes('Parent',dpitchfig);
    hold(dpitchax,'on');
    plot(dpitchax,meanpitch(1:ballastRows),meandepth(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(dpitchax,meanpitch(i*ballastRows+1:(i+1)*ballastRows),meandepth(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end

    xlabel('Pitch Angle (Deg)');
    ylabel('Depth (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dpitchax.Color = 'none';
    dpitchax.FontSize = 12;
    grid(dpitchax,'on');
    export_fig(dpitchfig,[imagedir 'depthPitchLine.png'],'-png','-transparent','-m3');
    
    %% figure 9
    dyawfig = figure('Position',[baseloc+moveright 600 600 400]);
    dyawax = axes('Parent',dyawfig);
    hold(dyawax,'on');
    plot(dyawax,meanyaw(1:ballastRows),meandepth(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(dyawax,meanyaw(i*ballastRows+1:(i+1)*ballastRows),meandepth(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end

    xlabel('Yaw Angle (Deg)');
    ylabel('Depth (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dyawax.Color = 'none';
    dyawax.FontSize = 12;
    grid(dyawax,'on');
    export_fig(dyawfig,[imagedir 'depthYawLine.png'],'-png','-transparent','-m3');
    
    
    %% fig 10
    dskewfig = figure('Position',[baseloc+2*moveright 600 600 400]);
    dskewax = axes('Parent',dskewfig);
    hold(dskewax,'on');
    h(1) = plot(dskewax,skew(1:ballastRows)*180/pi,meandepth(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        h(i+1) = plot(dskewax,skew(i*ballastRows+1:(i+1)*ballastRows)*180/pi,meandepth(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end
    xline(dskewax,15);
    xlabel('Skew Angle (Deg)');
    ylabel('Depth (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend(h,'Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dskewax.Color = 'none';
    dskewax.FontSize = 12;
    %dskewax.XLim = [0 35];
    %dskewax.YLim = [-60 0];    
    dskewax.XGrid = 'on';
    dskewax.YGrid = 'on';
    grid(dskewax,'on');
    export_fig(dskewfig,[imagedir 'depthSkewLine.png'],'-png','-transparent','-m3');    
    saveas(dskewfig,[imagedir 'depthSkewLine']);
    % Zoom to region of interest    
    dskewax.XLim = [0 30];
    dskewax.YLim = [-80 0];
    export_fig(dskewfig,[imagedir 'depthSkewLineZoom.png'],'-png','-transparent','-m3');
    clear h
    
    %% fig 11
    driftfig = figure('Position',[baseloc+3*moveright 600 600 400]);
    driftax = axes('Parent',driftfig);
    cmn = reshape(cm(3,:),ballastRows,reldensCols);
    rdn = reshape(relativeDensity,ballastRows,reldensCols);
    meandriftn = reshape(meandrift,ballastRows,reldensCols);
    surf(driftax,cmn*100,rdn,meandriftn,'FaceColor','interp','LineStyle','-');%,'Marker','o','MarkerFaceColor','r');
    view(driftax,-145,40);   
    xlabel('CM_a_x_i_a_l (%Body Length)');
    ylabel('Relative Density');
    zlabel('Drift (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    driftax.Color = 'none';
    driftax.FontSize = 12;
    grid(driftax,'on');
    export_fig(driftfig,[imagedir 'driftSurface.png'],'-png','-transparent','-m3');
    
    %% fig 12
    dftlinefig = figure('Position',[baseloc+4*moveright 600 600 400]);
    dftlineax = axes('Parent',dftlinefig);
    hold(dftlineax,'on');
    plot(dftlineax,cm(3,1:ballastRows)*100,meandrift(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(dftlineax,cm(3,i*ballastRows+1:(i+1)*ballastRows)*100,meandrift(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end

    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
    ylabel('Drift (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dftlineax.Color = 'none';
    dftlineax.FontSize = 12;
    grid(dftlineax,'on');
    export_fig(dftlinefig,[imagedir 'driftLine.png'],'-png','-transparent','-m3');
    
    %% fig 13
    ddfig = figure('Position',[baseloc+5*moveright 600 600 400]);
    ddax = axes('Parent',ddfig);
    hold(ddax,'on');
    plot(ddax,meandepth(1:ballastRows),meandrift(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(ddax,meandepth(i*ballastRows+1:(i+1)*ballastRows),meandrift(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end

    plot(ddax,[max(min(meandepth),min(meandrift)) min(max(meandepth),max(meandrift))],[max(min(meandepth),min(meandrift)) min(max(meandepth),max(meandrift))],'-.k','DisplayName','Unity');
    xlabel('Depth (m)');
    ylabel('Drift (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    ddax.Color = 'none';
    ddax.FontSize = 12;
    grid(ddax,'on');
    export_fig(ddfig,[imagedir 'driftDepth.png'],'-png','-transparent','-m3');    
    
    %% fig 14
    pyfig = figure('Position',[baseloc+6*moveright 600 600 400]);
    pyax = axes('Parent',pyfig);
    hold(pyax,'on');
    %meanpitch = meanpitch*180/pi-90;
%     meanyaw = meanyaw*180/pi;
    plot(pyax,meanpitch(1:ballastRows),meanyaw(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(pyax,meanpitch(i*ballastRows+1:(i+1)*ballastRows),meanyaw(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end    

    xlabel('Pitch (deg)');
    ylabel('Yaw (deg)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    pyax.Color = 'none';
    pyax.FontSize = 12;
    grid(pyax,'on');
    export_fig(pyfig,[imagedir 'pitchyaw.png'],'-png','-transparent','-m3');
    
    %% fig 15
    drskewfig = figure('Position',[baseloc+7*moveright 600 600 400]);
    drskewax = axes('Parent',drskewfig);
    hold(drskewax,'on');
    h(1) = plot(drskewax,skew(1:ballastRows)*180/pi,meandrift(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        h(i+1) = plot(drskewax,skew(i*ballastRows+1:(i+1)*ballastRows)*180/pi,meandrift(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end
    xline(drskewax,15);

    xlabel('Skew Angle (Deg)');
    ylabel('Drift (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend(h,'Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    drskewax.Color = 'none';
    drskewax.FontSize = 12;
    %dskewax.XLim = [0 35];
    %dskewax.YLim = [-60 0];
    drskewax.XGrid = 'on';
    drskewax.YGrid = 'on';
    
    grid(drskewax,'on');
    export_fig(drskewfig,[imagedir 'driftSkewLine.png'],'-png','-transparent','-m3');
    saveas(drskewfig,[imagedir 'driftSkewLine']);
    % Zoom to region of interest   
    drskewax.XLim = [0 30];
    drskewax.YLim = [-80 0];
    export_fig(drskewfig,[imagedir 'driftSkewLineZoom.png'],'-png','-transparent','-m3'); 
    clear h
    
    %% figure 16
    dftyawfig = figure('Position',[baseloc+moveright 600 600 400]);
    dftyawax = axes('Parent',dftyawfig);
    hold(dftyawax,'on');
    plot(dftyawax,meanyaw(1:ballastRows),meandrift(1:ballastRows),'-*r','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',num2str(relativeDensity(1)));
    for i=1:1:reldensCols-1
        plot(dftyawax,meanyaw(i*ballastRows+1:(i+1)*ballastRows),meandrift(i*ballastRows+1:(i+1)*ballastRows),char(strcat(styls(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',num2str(relativeDensity(i*ballastRows+1)));
    end

    xlabel('Yaw Angle (Deg)');
    ylabel('Drift (m)');
    title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend('Location','Best','Color','none');
    hleg.Title.String = 'Relative Density';
    dftyawax.Color = 'none';
    dftyawax.FontSize = 12;
    grid(dftyawax,'on');
    export_fig(dftyawfig,[imagedir 'driftYawLine.png'],'-png','-transparent','-m3');
    
    end % end if not hydrostatic
    
    %% figure 16 - Only for hydrostatic
    if hydrostatic
        a3 = 0:0.001:0.1;
        for i=1:1:reldensCols
            for j=1:1:length(a3)
                thetatheory(i,j) = atan2d(2*relativeDensity((i-1)*ballastRows+1)*a1,-2*relativeDensity((i-1)*ballastRows+1)*a3(j)+(1-relativeDensity((i-1)*ballastRows+1)));
            end
        end
            
    ptchfig = figure('Position',[baseloc+4*moveright 100 600 400]);
    ptchax = axes('Parent',ptchfig);
    hold(ptchax,'on');
    hs(1) = plot(ptchax,cm(3,1:ballastRows)*100,meanpitch(1:ballastRows),'o','MarkerEdgeColor','r','MarkerFaceColor','r','DisplayName',...
        [num2str(relativeDensity(1),'%3.2f') ' (Simulation)']);
    ht(1) = plot(ptchax,a3*100,thetatheory(1,:)-90,'-r','DisplayName',[num2str(relativeDensity(1),'%3.2f') ' (Theory)' ]);
    for i=1:1:reldensCols-1
        hs(i+1) = plot(ptchax,cm(3,i*ballastRows+1:(i+1)*ballastRows)*100,meanpitch(i*ballastRows+1:(i+1)*ballastRows),...
            char(strcat(mrkrs(i),mkrclrs(i))),'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),...
            'DisplayName',[num2str(relativeDensity(i*ballastRows+1)) ' (Simulation)']);
        %plot(ptchax,a3*100,theta(i+1,:)-90,['-' char(mkrclrs(i))],'MarkerEdgeColor',char(mkrclrs(i)),'MarkerFaceColor',char(mkrclrs(i)),'DisplayName',[num2str(relativeDensity(i*ballastRows+1)) ' (Theory)' ]);
        ht(i+1) = plot(ptchax,a3*100,thetatheory(i+1,:)-90,['-' char(mkrclrs(i))],'DisplayName',[num2str(relativeDensity(i*ballastRows+1)) ' (Theory)' ],'LineWidth',1.0);
    end

    xlabel('Normalized Center Mass Axial Location (%L_B_o_d_y)');
    ylabel('Pitch angle (deg)');
    %title(['Sweep: ' char(sweep) ' (a_1 = ' num2str(a1,2) ')']);
    hleg = legend([hs ht],'Location','Best','Color','none','NumColumns',2);
    hleg.Title.String = 'Relative Density';
    ptchax.Color = 'none';
    ptchax.FontSize = 12;
    %ptchax.YLim = [-20 80];
    grid(ptchax,'on');
    hold(ptchax,'off');
    export_fig(ptchfig,[imagedir 'theoryHydrostaticCompv2.png'],'-png','-transparent','-m3');
    end
end