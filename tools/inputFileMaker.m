% inputFileMaker.m
% makes input files

clearvars; close all; clc;

relativeDensities = [1 0.99 0.98 0.97 0.96 0.95];
%relativeDensities = [0.999 0.998 0.997 0.996];
% relativeDensities = [0.74 0.76 0.78 0.8 0.82 0.84 0.86 0.88 1.12 1.14 1.16 1.18 1.2 1.22 1.24 1.26 1.28 1.3 1.32];
ballastZLocationsPrcnt = [0 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 0.5];
% ballastZLocationsPrcnt = [-0.35 -0.3 -0.25 -0.2 -0.15 -0.1 -0.05];
%ballastZLocationsPrcnt = 0;
windwardFlowFactors = 0.667; %[0.4 0.6 0.8 1.0];
leewardFlowFactors = 0.667; %[0.4 0.6 0.8 1.0];
% windwardFlowFactors = 0.8;
% leewardFlowFactors = 0.4;
pathToInputFolder = '..\input\';
casesName = 'DB2case';
caseNumber = 1;
for i1=1:1:length(relativeDensities)
    for i2=1:1:length(ballastZLocationsPrcnt)
        for i3=1:1:length(windwardFlowFactors)
            for i4=1:1:length(leewardFlowFactors)
                inputFileName = [casesName num2str(caseNumber) '.m'];
                inputFileID = fopen([pathToInputFolder inputFileName],'w');
                fprintf(inputFileID,'%s\n',['runname = ''' casesName num2str(caseNumber) ''';']);
                fprintf(inputFileID,'%s\n','fluidtype = ''water'';');
                fprintf(inputFileID,'%s\n','fluidBaseVelocity = [1.5;0.0;0];');
                fprintf(inputFileID,'%s\n','flowtype = ''steady'';');
                fprintf(inputFileID,'%s\n','flowparms = [];');
                fprintf(inputFileID,'%s\n','vblength = 18.0;');
                fprintf(inputFileID,'%s\n','vbradius = 0.6;');
                fprintf(inputFileID,'%s\n','vbinnerradius = 0.967*vbradius;');
                fprintf(inputFileID,'%s\n','vbmass = 7000;');
                fprintf(inputFileID,'%s\n','vbnorm = 1.2;');
                fprintf(inputFileID,'%s\n','vbax = 0.1;');
                fprintf(inputFileID,'%s\n','useBEMT = false;');
                fprintf(inputFileID,'%s\n','usePrandtl = false;');
                fprintf(inputFileID,'%s\n','bladeMass1 = 5000;');
                fprintf(inputFileID,'%s\n','airfoiltype1 = ''NACA0015'';');
                fprintf(inputFileID,'%s\n','aspectRatio1 = 10;');
                fprintf(inputFileID,'%s\n','bladeLength1 = 18;');
                fprintf(inputFileID,'%s\n','secChord1 = bladeLength1/aspectRatio1;');
                fprintf(inputFileID,'%s\n','numSections1 = 18;');
                fprintf(inputFileID,'%s\n','numBlades1 = 3;');
                fprintf(inputFileID,'%s\n','bladeDZfrac1 = 0.1;');
                fprintf(inputFileID,'%s\n','twist1.AoAopt_deg = 10.0;');
                fprintf(inputFileID,'%s\n','twist1.numBlades = numBlades1;');
                fprintf(inputFileID,'%s\n','twist1.bladeDZfrac = bladeDZfrac1;');
                fprintf(inputFileID,'%s\n',['axflowfactor1 = ' num2str(windwardFlowFactors(i3),'%4.3f') ';']);
                fprintf(inputFileID,'%s\n','tnflowfactor1 = 1.0;');
                fprintf(inputFileID,'%s\n','bladeMass2 = 5000;');
                fprintf(inputFileID,'%s\n','airfoiltype2 = ''NACA0015'';');
                fprintf(inputFileID,'%s\n','aspectRatio2 = 10;');
                fprintf(inputFileID,'%s\n','bladeLength2 = 18;');
                fprintf(inputFileID,'%s\n','secChord2 = bladeLength1/aspectRatio1;');
                fprintf(inputFileID,'%s\n','numSections2 = 18;');
                fprintf(inputFileID,'%s\n','numBlades2 = 3;');
                fprintf(inputFileID,'%s\n','bladeDZfrac2 = 0.1;');
                fprintf(inputFileID,'%s\n','twist2.AoAopt_deg = 10.0;');
                fprintf(inputFileID,'%s\n','twist2.numBlades = numBlades1;');
                fprintf(inputFileID,'%s\n','twist2.bladeDZfrac = bladeDZfrac1;');
                fprintf(inputFileID,'%s\n',['axflowfactor2 = ' num2str(leewardFlowFactors(i4),'%4.3f') ';']);
                fprintf(inputFileID,'%s\n','tnflowfactor2 = 1.0;');
                fprintf(inputFileID,'%s\n','I = [1/12*vbmass*(3*(vbradius^2+vbinnerradius^2)+vblength^2),0,0;0,1/12*vbmass*(3*(vbradius^2+vbinnerradius^2)+vblength^2),0;0,0,1/2*vbmass*(vbradius^2+vbinnerradius^2)];');
                fprintf(inputFileID,'%s\n','vbcentermass = [0.0;0;0.0*vblength/2];');
                fprintf(inputFileID,'%s\n','ballastMass = 9000;');
                fprintf(inputFileID,'%s\n',['ballastLoc = [0.4*vbradius;0;' num2str(ballastZLocationsPrcnt(i2),'%4.3f') '*vblength];']);
                %fprintf(inputFileID,'%s\n',['ballastLoc = [1.6*vbradius;0;' num2str(ballastZLocationsPrcnt(i2),'%4.3f') '*vblength];']);
                fprintf(inputFileID,'%s\n','vcentermass = [];');
                fprintf(inputFileID,'%s\n','vbtetherpoint = [0;0;-vblength/2];');
                fprintf(inputFileID,'%s\n','vbbuoypoint = [0;0;-0.0*vblength/2];');
                fprintf(inputFileID,'%s\n',['vreldensity = ' num2str(relativeDensities(i1),'%4.3f') ';']);
                fprintf(inputFileID,'%s\n','rot1point = [0;0;-vblength/2];');
                fprintf(inputFileID,'%s\n','rot2point = [0;0;vblength/2];');
                fprintf(inputFileID,'%s\n','rot1rad = bladeLength1;');
                fprintf(inputFileID,'%s\n','rot2rad = bladeLength2;');
                fprintf(inputFileID,'%s\n','rot1ornt = [0;0;0];');
                fprintf(inputFileID,'%s\n','rot2ornt = [0;0;0];');
                fprintf(inputFileID,'%s\n','rot1initRPM = 11;');
                fprintf(inputFileID,'%s\n','rot2initRPM = -4;');
%                 fprintf(inputFileID,'%s\n','rot1initRPM = 0;');
%                 fprintf(inputFileID,'%s\n','rot2initRPM = 0;');
                fprintf(inputFileID,'%s\n','addedMass = [];');
                fprintf(inputFileID,'%s\n','rotorVisc = 0.02;');
                fprintf(inputFileID,'%s\n','vtMod = 1.0;');
                fprintf(inputFileID,'%s\n','hifiTors = false;');
                fprintf(inputFileID,'%s\n','tmod = 5.0e9;');
                fprintf(inputFileID,'%s\n','tunstrch = 200;');
                fprintf(inputFileID,'%s\n','tradius = 0.1;');
                fprintf(inputFileID,'%s\n','tspring = tmod*pi*tradius^2/tunstrch;');
                fprintf(inputFileID,'%s\n','tdamp = [];');
                fprintf(inputFileID,'%s\n','tdampfac = 1.0;');
                fprintf(inputFileID,'%s\n','tnnodes = 0;');
                fprintf(inputFileID,'%s\n','tnodlocs = [];');
                fprintf(inputFileID,'%s\n','gmconst = 1;');
                fprintf(inputFileID,'%s\n','gflux = 200;');
                fprintf(inputFileID,'%s\n','grarm = 0.1;');
                fprintf(inputFileID,'%s\n','gkvisc = 1.0e-1;');
                fprintf(inputFileID,'%s\n','gmass = 1000.0;');
                fprintf(inputFileID,'%s\n','grload = 0.1;');
                fprintf(inputFileID,'%s\n','gpoint = [0;0;0];');
                fprintf(inputFileID,'%s\n','totalSimTime = 3600;');
                fprintf(inputFileID,'%s\n','tstep = 0.2;');
                
                fprintf(inputFileID,'%s\n','initialYaw = 0*pi/180;');
%                 initpitch = 310.5*relativeDensities(i1)-307.85;
%                 if relativeDensities(i1) > 1
%                     initpitch = 242.57*relativeDensities(i1)-242.67;
%                 end

                initpitch = 90;%+initpitch;
                fprintf(inputFileID,'%s\n',['initialPitch = ' num2str(initpitch,'%5.3f') '*pi/180;']);
                fprintf(inputFileID,'%s\n','initialRoll = 0*pi/180;');
                fprintf(inputFileID,'%s\n','initialLateral = 0;');
                fprintf(inputFileID,'%s\n','initialLongitudinal = tunstrch*1.05;');
%                 fprintf(inputFileID,'%s\n','initialLongitudinal = 0;');
                fprintf(inputFileID,'%s\n','initialVertical = 0;');
%                 fprintf(inputFileID,'%s\n','initialVertical = tunstrch*1.05;');
                fprintf(inputFileID,'%s\n','initialSway = 0;');
                fprintf(inputFileID,'%s\n','initialSurge = 0;');
                fprintf(inputFileID,'%s\n','initialHeave = 0;');
                fprintf(inputFileID,'%s\n','initialYawRate = 0;');
                fprintf(inputFileID,'%s\n','initialPitchRate = 0;');
                fprintf(inputFileID,'%s\n','initialRollRate = 0;');
                fprintf(inputFileID,'%s\n','makeplots = false;');
                fprintf(inputFileID,'%s\n','makemovie = false;');
                fprintf(inputFileID,'%s\n','speedfactor = 60;');

                fclose(inputFileID);
                caseNumber = caseNumber + 1;
            end
        end
    end
end