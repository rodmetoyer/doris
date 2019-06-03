% unitTests.m
% Runs unit tests on classes and modules

clearvars; close all; clc;
addpath('..\src');

% Global options
opts.verbose = true;
opts.resultsfile = '';
opts.plot = true;
opts.movie = false;

runairfoil = false; runfluid = false; runrotor = false; runblade = false;
runbaldesection = false; runvehicle = false;
unittests = {'All','Airfoil','Fluid','Rotor','Blade','BladeSection','Vehicle'};
[indx, tf] = listdlg('PromptString','Select a test to run?','ListString',unittests);
if ~tf
    error('Test canceled.');
else
    for i=1:1:numel(indx)        
        switch char(unittests(indx(i)))
            case 'All'
                runairfoil = true; runfluid = true; runrotor = true; runblade = true; runbaldesection = true;
                break;
            case 'Fluid'
                runfluid = true;
            case 'Rotor'
                runrotor = true;
            case 'Airfoil'
                runairfoil = true;
            case 'Blade'
                runblade = true;
            case 'BladeSection'
                runbaldesection = true;
            case 'Vehicle'
                runvehicle = true;
            otherwise
                error('Something strange happened. What do I do with this?');
        end
    end
end

% Fluid
f = fluid('water'); % This object gets used later in bladesection unit test
% todo make all these stand alone. If you need objects for the unit test
% make them there then destroy them before moving on.
f.velocity = [1.0,0.0,0.0];
if runfluid
    fltest = fluidUnitTest(f);
    if ~fltest
        error('Fluid unit test failed.');
    end
end

% Airfoil
if runairfoil
    aftest = airfoilUnitTest(opts);
    if ~aftest
        error('Airfoil unit test failed.');
    end
end

% bladesection
% Make new airfoil object
if runbaldesection
    af = airfoil('SG6040');
    bs = bladesection(0.1,0.1,af);
    passed = bladesectionUnitTest(bs,f,true);
    if ~passed
        error('Bladesection failed unit test');
    end
end

% blade
% Make a blade out of blade sections
% (root)<-bs->-<-bs1->-<-bs2->-<--bs3-->-<---bs4--->-<bs5>(tip)
if runblade
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

if runrotor
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

if runvehicle
    vtest = vehicleUnitTest(opts);    
end

% todo Make a summary of all the returns