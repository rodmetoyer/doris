% unitTests.m
% Runs unit tests on classes and modules

clearvars; close all; clc;
addpath('..\src');

% Global options
opts.interactive = false; % Make this true to turn on all the dialogs, make it false to manually run unit tests
opts.verbose = true;
opts.resultsfile = '';
opts.plot = true;
opts.movie = false;
testToRun = 'Generator';
% These are all of the unit tests available
unittests = {'All','Airfoil','Fluid','Rotor','Blade','BladeSection','Vehicle','Generator'};


%% Run Unit Tests
run.airfoil = false; run.fluid = false; run.rotor = false; run.blade = false;
run.baldesection = false; run.vehicle = false; run.generator = false;

if opts.interactive
    [indx, tf] = listdlg('PromptString','Select a test to run?','ListString',unittests);
else
    tf = opts.interactive;
    ii = 1;
    for i=1:1:numel(unittests)
        if strcmpi(testToRun,unittests(i))
            indx(ii) = i;
            ii = ii + 1;
        end
    end
    clear ii;
end
if (~tf && opts.interactive)
    error('Error determining which tests to run. Test canceled.');
else
    for i=1:1:numel(indx)        
        switch char(unittests(indx(i)))
            case 'All'
                run.airfoil = true; run.fluid = true; run.rotor = true; run.blade = true; run.baldesection = true;
                break;
            case 'Fluid'
                run.fluid = true;
            case 'Rotor'
                run.rotor = true;
            case 'Airfoil'
                run.airfoil = true;
            case 'Blade'
                run.blade = true;
            case 'BladeSection'
                run.baldesection = true;
            case 'Vehicle'
                run.vehicle = true;
            case 'Generator'
                run.generator = true;
            otherwise
                errstr = ['Something strange happened. I was trying to run "' char(unittests(indx(i))) '". What do I do with this?'];
                error(errstr);
        end
    end
end

% Fluid
if run.fluid
    f = fluid('water'); % This object gets used later in bladesection unit test
    % todo make all these stand alone. If you need objects for the unit test
    % make them there then destroy them before moving on.
    f.velocity = [1.0,0.0,0.0];
    fltest = fluidUnitTest(f);
    if ~fltest
        error('Fluid unit test failed.');
    end
    clear f fltest;
end

% Airfoil
if run.airfoil
    aftest = airfoilUnitTest(opts);
    if ~aftest
        error('Airfoil unit test failed.');
    end
end

% bladesection
% Make new airfoil object
if run.baldesection
    f = fluid('water');
    f.velocity = [1.0,0.0,0.0];
    af = airfoil('SG6040');
    bs = bladesection(0.1,0.1,af);
    passed = bladesectionUnitTest(bs,f,true);
    if ~passed
        error('Bladesection failed unit test');
    end
    clear f bs af passed
end

% blade
% Make a blade out of blade sections
% (root)<-bs->-<-bs1->-<-bs2->-<--bs3-->-<---bs4--->-<bs5>(tip)
if run.blade
    error('blade unit test not currently functional');
    
    bs3 = bladesection(0.1,0.2,af);
    bs4 = bladesection(0.1,0.3,af);
    bs5 = bladesection(0.1,0.05,af);
    bsv = [bs, bs1, bs2, bs3, bs4, bs5];
    expectedlength = 3*0.1 + 0.2 + 0.3 + 0.05;
    twist = -[64.6,49.9,38.7,30.5,24.5,19.95,16.45,13.7,11.5,9.7,8.1,6.9,5.76,4.81,3.98,3.25,2.61,2.03,1.51,1.04];
    blade1 = blade(bsv,1.1,twist);
    if blade1.length ~= expectedlength
        error('Blade length error');
    end
    mag = 1.0;
    angs = 0:10:180;
    vx = cosd(angs);
    vz = sind(angs);
    vy = zeros(size(vx));
    vr = [vx;vy;vz];
    for i=1:1:numel(blade1.sections)
        [L(:,i),D(:,i),M(:,i)] = computeLoads(blade1.sections(i),vr(:,i),f); % You know this doesn't use twist right.
    end
    Lmag = vecnorm(L);
    Dmag = vecnorm(D);
    Mmag = vecnorm(M);
    figure
    plot(Lmag,'*-r');
    hold on;
    plot(Dmag,'*-b');
    plot(Mmag,'*-c');
    hold off
end

if run.rotor
    error('rotor unit test not currently funcitonal');
    % Rotor
    % first need to make a couple more blades (remember that these are handle
    % classes, so copying does not create a new object).
    % Actually, the rotor only needs one blade object. He'll just keep going to
    % that same object to get preperties. Only need multiple objects if blade
    % is different.
    %blade2 = blade(bsv,1.1);
    %blade3 = blade(bsv,1.1);
    r = rotor([blade1,blade1,blade1]);
    % Put him in a stationary fluid, fixed in space, perpendicular to flow at zero
    % rotation
    f.velocity = [0.0;0.0;0.0];
    r.position = [0,0,0]; % Should get a warning for using a row vector
    disp('Row vector warning is expected');
    r.velocity = [0;0;0];
    r.orientation = [90;0;0];
    r.angvel = [0;0;0];
    % Now see what the loads are. Should be nada.
    %[force, torque] = r.computeAeroLoadsBasic(f);
    % Now let's see what happens when we put it into slow flow. Should have
    % most torque in the negative i_O direction and most force in the positive
    % i_O direction.
    f.velocity = [0.5;0.0;0.0];
    [force, torque] = r.computeAeroLoadsBasic(f);

    %rotorUnitTest(f,test);
end

if run.vehicle
    vtest = vehicleUnitTest(opts);    
end

% generator
if run.generator
    % Make a generator object
    % generator(machineConstant,magFlux,ArmResistance,viscCoeff)
    gen = generator(0.19,1,0.1,1.0e-4);
    % Get the torque for whatever rate it is currently spinning. Should be
    % minimum right now because there is no load.
    speed = 0:1:100;
%     for i=1:1:length(speed)
%         tq(1,i) = -gen.getTorque(speed(i));
%         pwr(1,i) = gen.getPower(abs(speed(i)));
%         curr(1,i) = gen.getArmatureCurrent(abs(speed(i)));
%     end
    % set the load. Torque increases as load decreases
    load = [inf logspace(0,-2,3) 0];
    load = [inf logspace(2,1,3) 0];
    load = [inf 100 5 0];
    for j = 1:1:length(load)
        gen.setLoadResistance(load(j));
        for i=1:1:length(speed)
            tq(j,i) = -gen.getTorque(speed(i));
            pwr(j,i) = gen.getPower(abs(speed(i)));
            curr(j,i) = gen.getArmatureCurrent(abs(speed(i)));
        end
    end
    % make a plot
    color = 'w';
    hfig1 = figure('Color',color);
    ax1 = axes('Parent',hfig1,'Color',color);
    hold on
    for j = 1:1:length(load)
        plot(ax1,speed*180/pi,tq(j,:),'LineWidth',2.0);
    end
    hold off
    legend(ax1,[sprintfc('%3.2e',load)],'Location','best');
    xlabel(ax1,'Shaft Speed (RPM)'); ylabel(ax1,'Torque (Nm)');
    
    hfig2= figure('Color',color);
    ax2 = axes('Parent',hfig2,'Color',color);
    hold on
    for j = 1:1:length(load)
        plot(ax2,speed*180/pi,pwr(j,:),'LineWidth',2.0);
    end
    hold off
    legend(ax2,[sprintfc('%3.2e',load)],'Location','best');
    xlabel(ax2,'Shaft Speed (RPM)'); ylabel(ax2,'Electrical Power (W)');
    
%     case2plot = 1;
%     if case2plot > 1
%         loadnow = load(case2plot-1);
%     else
%         loadnow = inf;
%     end
    hfig3 = figure('Position',[100 100 900 800],'Color',color);
    ax31 = subplot(2,2,1,'Color',color,'Parent',hfig3);
    ax32 = subplot(2,2,2,'Color',color,'Parent',hfig3);
    ax33 = subplot(2,2,3,'Color',color,'Parent',hfig3);
    ax34 = subplot(2,2,4,'Color',color,'Parent',hfig3);
    plot(ax31,speed*180/pi,speed.*tq(1,:),speed*180/pi,pwr(1,:),'LineWidth',2.0); hold on
    %plot(ax31,speed*180/pi,pwr(case2plot,:));% hold off
    plot(ax32,speed*180/pi,speed.*tq(2,:),speed*180/pi,pwr(2,:),'LineWidth',2.0);% hold on
    %plot(ax32,speed*180/pi,pwr(case2plot,:));% hold off
    plot(ax33,speed*180/pi,speed.*tq(3,:),speed*180/pi,pwr(3,:),'LineWidth',2.0);% hold on
    %plot(ax33,speed*180/pi,pwr(case2plot,:));% hold off
    plot(ax34,speed*180/pi,speed.*tq(end,:),speed*180/pi,pwr(end,:),'LineWidth',2.0);% hold on
    % plot(ax34,speed*180/pi,pwr(case2plot,:)); hold off
    legend(ax31,{'Mechanical','Electrical'},'Location','best');
    xlabel(ax33,'Shaft Speed (RPM)'); ylabel(ax31,'Power (W)');
    xlabel(ax34,'Shaft Speed (RPM)'); ylabel(ax33,'Power (W)');
    title(ax31,['Load resistance = ' num2str(load(1),'%3.2f') ' (\Omega)']);
    title(ax32,['Load resistance = ' num2str(load(2),'%3.2f') ' (\Omega)']);
    title(ax33,['Load resistance = ' num2str(load(3),'%3.2f') ' (\Omega)']);
    title(ax34,['Load resistance = ' num2str(load(end),'%3.2f') ' (\Omega)']);
    ax31.Box = 'off'; ax32.Box = 'off'; ax33.Box = 'off'; ax34.Box = 'off';
%     
%     figure
%     plot(tq(:,end),pwr(:,end),'or')
%     figure
%     hold on
%     for j = 1:1:length(load)+1
%         plot(speed*180/pi,curr(j,:));
%     end
%     hold off
%     xlabel('Shaft Speed (RPM)'); ylabel('Current (A)');

end % generator

% todo Make a summary of all the returns