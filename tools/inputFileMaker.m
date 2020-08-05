% inputFileMaker.m
% makes input files

clearvars; close all; clc;

% These are the input files to make
% If you make a new set of params for the in. struct put them at the top
% and if(any) the input file string array (see below for examples)
infiles2make = ["BAL","BAH","BL2"];


%% BLB
sweepID = "BLB";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.8;
in.leewardFlowFactors = 0.4;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end BLB

%% EFF
sweepID = "EFF";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.667;
in.leewardFlowFactors = 0.667;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end EFF

%% EF2
sweepID = "EF2";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '1.6';
in.windwardFlowFactors = 0.667;
in.leewardFlowFactors = 0.667;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end EF2

%% EFT
sweepID = "EFT";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '2.4';
in.windwardFlowFactors = 0.667;
in.leewardFlowFactors = 0.667;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end EFT

%% EAA
sweepID = "EAA";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.6;
in.leewardFlowFactors = 0.6;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end EAA

%% EBB
sweepID = "EBB";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.7;
in.leewardFlowFactors = 0.7;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end EBB

%% EAA
sweepID = "ECC";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.8;
in.leewardFlowFactors = 0.8;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end ECC

%% DBB
sweepID = "DBB";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '1.6';
in.windwardFlowFactors = 0.8;
in.leewardFlowFactors = 0.4;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end DBB

%% DB2
sweepID = "DB2";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.4';
in.windwardFlowFactors = 0.8;
in.leewardFlowFactors = 0.4;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end DB2

%% BLL
sweepID = "BLL";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.8;
in.leewardFlowFactors = 0.6;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end BLL

%% BL2
sweepID = "BL2";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.6;
in.leewardFlowFactors = 0.4;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end BL2

%% BAH
sweepID = "BAH";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.8;
in.leewardFlowFactors = 0.6;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.6;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end BAH

%% BAL
sweepID = "BAL";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.8;
in.leewardFlowFactors = 0.6;
in.numLeeBlades = '3';
in.numWindBlades = '3';
in.thrunstrched = 200;
in.flowspeed = 1.4;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end BAL

%% FBL
sweepID = "FBL";
if any(strcmp(infiles2make,sweepID))
in.casesName = [char(sweepID) 'case'];
in.caseNumberStart = 1;
% Params
in.relativeDensities = 1:-0.01:0.95;
in.ballastZLocationsPrcnt = 0:0.05:0.5;
in.ballastXLoc = '0.8';
in.windwardFlowFactors = 0.8;
in.leewardFlowFactors = 0.4;
in.numWindBlades = '3';
in.numLeeBlades = '4';
in.thrunstrched = 200;
in.flowspeed = 1.5;
%ICs
in.initpitch = 90;
in.initvertical = 0;
in.initlongitudinal = 1.05*in.thrunstrched;
makeFiles(in)
disp(['Made files for ' char(sweepID)]);
end %%%%%%%%%%%%%%% end FBL

function makeFiles(in)
pathToInputFolder = '..\input\';
caseNumberStart = in.caseNumberStart;
for i1=1:1:length(in.relativeDensities)
    for i2=1:1:length(in.ballastZLocationsPrcnt)
        for i3=1:1:length(in.windwardFlowFactors)
            for i4=1:1:length(in.leewardFlowFactors)
                inputFileName = [in.casesName num2str(caseNumberStart) '.m'];
                inputFileID = fopen([pathToInputFolder inputFileName],'w');
                fprintf(inputFileID,'%s\n',['runname = ''' in.casesName num2str(caseNumberStart) ''';']);
                fprintf(inputFileID,'%s\n','fluidtype = ''water'';');
                fprintf(inputFileID,'%s\n',['fluidBaseVelocity = [' num2str(in.flowspeed,12) ';0.0;0];']);
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
                fprintf(inputFileID,'%s\n',['numBlades1 = ' in.numWindBlades ';']);
                fprintf(inputFileID,'%s\n','bladeDZfrac1 = 0.1;');
                fprintf(inputFileID,'%s\n','twist1.AoAopt_deg = 10.0;');
                fprintf(inputFileID,'%s\n','twist1.numBlades = numBlades1;');
                fprintf(inputFileID,'%s\n','twist1.bladeDZfrac = bladeDZfrac1;');
                fprintf(inputFileID,'%s\n',['axflowfactor1 = ' num2str(in.windwardFlowFactors(i3),'%4.3f') ';']);
                fprintf(inputFileID,'%s\n','tnflowfactor1 = 1.0;');
                fprintf(inputFileID,'%s\n','bladeMass2 = 5000;');
                fprintf(inputFileID,'%s\n','airfoiltype2 = ''NACA0015'';');
                fprintf(inputFileID,'%s\n','aspectRatio2 = 10;');
                fprintf(inputFileID,'%s\n','bladeLength2 = 18;');
                fprintf(inputFileID,'%s\n','secChord2 = bladeLength1/aspectRatio1;');
                fprintf(inputFileID,'%s\n','numSections2 = 18;');
                fprintf(inputFileID,'%s\n',['numBlades2 = ' in.numLeeBlades ';']);
                fprintf(inputFileID,'%s\n','bladeDZfrac2 = 0.1;');
                fprintf(inputFileID,'%s\n','twist2.AoAopt_deg = 10.0;');
                fprintf(inputFileID,'%s\n','twist2.numBlades = numBlades1;');
                fprintf(inputFileID,'%s\n','twist2.bladeDZfrac = bladeDZfrac1;');
                fprintf(inputFileID,'%s\n',['axflowfactor2 = ' num2str(in.leewardFlowFactors(i4),'%4.3f') ';']);
                fprintf(inputFileID,'%s\n','tnflowfactor2 = 1.0;');
                fprintf(inputFileID,'%s\n','I = [1/12*vbmass*(3*(vbradius^2+vbinnerradius^2)+vblength^2),0,0;0,1/12*vbmass*(3*(vbradius^2+vbinnerradius^2)+vblength^2),0;0,0,1/2*vbmass*(vbradius^2+vbinnerradius^2)];');
                fprintf(inputFileID,'%s\n','vbcentermass = [0.0;0;0.0*vblength/2];');
                fprintf(inputFileID,'%s\n','ballastMass = 9000;');
                fprintf(inputFileID,'%s\n',['ballastLoc = [' in.ballastXLoc '*vbradius;0;' num2str(in.ballastZLocationsPrcnt(i2),'%4.3f') '*vblength];']);
                %fprintf(inputFileID,'%s\n',['ballastLoc = [1.6*vbradius;0;' num2str(ballastZLocationsPrcnt(i2),'%4.3f') '*vblength];']);
                fprintf(inputFileID,'%s\n','vcentermass = [];');
                fprintf(inputFileID,'%s\n','vbtetherpoint = [0;0;-vblength/2];');
                fprintf(inputFileID,'%s\n','vbbuoypoint = [0;0;-0.0*vblength/2];');
                fprintf(inputFileID,'%s\n',['vreldensity = ' num2str(in.relativeDensities(i1),'%4.3f') ';']);
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
                fprintf(inputFileID,'%s\n',['tunstrch = ' num2str(in.thrunstrched,12) ';']);
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

                %initpitch = 90;%+initpitch;
                fprintf(inputFileID,'%s\n',['initialPitch = ' num2str(in.initpitch,12) '*pi/180;']);
                fprintf(inputFileID,'%s\n','initialRoll = 0*pi/180;');
                fprintf(inputFileID,'%s\n','initialLateral = 0;');
                fprintf(inputFileID,'%s\n',['initialLongitudinal = ' num2str(in.initlongitudinal,12) ';']);
%                 fprintf(inputFileID,'%s\n','initialLongitudinal = 0;');
                fprintf(inputFileID,'%s\n',['initialVertical = ' num2str(in.initvertical,12) ';']);
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
                caseNumberStart = caseNumberStart + 1;
            end
        end
    end
end
end % makeFiles function