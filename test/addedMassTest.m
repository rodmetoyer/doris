% addedMassTest2.m

clearvars; close all; clc;

addpath('..\src');

% set-up
casetoload = 'tacticalBaseline';
datapath = 'D:\School\doris\products\data';
sim = simulation.loadsim(casetoload,datapath);
savefigs = false;
thickness = 0.15;

% First thing is to orient the body and make sure it looks right
sim.vhcl.orientation = [0;0;0];
sim.vhcl.rotors(1).orientation = [0;0;0]*pi/180;
sim.vhcl.rotors(2).orientation = [0;0;0]*pi/180;
%sim.showmerotors;

% make an added mass vehicle object
v = amvehicle;

% Now add sections for the body
% addSection(location,orientation)
nsects = 27; % make this an even number
nsects = 2*round(nsects/2); % ok, don't worry about making it even
halfEllipse = sim.vhcl.body.length/2;
sectWidth = sim.vhcl.body.length/nsects;
for i=1:1:nsects/2 % add circles in the +x direction
    z = (i-1)*sectWidth + sectWidth/2;
    r = sim.vhcl.body.radius*sqrt(1-(z)^2/halfEllipse^2);
    v.addSection(amsection('ellipse',r,r,sectWidth),[0;0;z],[0;pi/2;0]);
end
for i=1:1:nsects/2 % add circles in the -x direction
    z = -((i-1)*sectWidth + sectWidth/2);
    r = sim.vhcl.body.radius*sqrt(1-(z)^2/halfEllipse^2);
    v.addSection(amsection('ellipse',r,r,sectWidth),[0;0;z],[0;pi/2;0]);
end
% see if it worked
%v.showme('b');
%MA = v.getAddedMass;
% so far so good

for i=1:1:numel(sim.vhcl.rotors)
    for j=1:1:numel(sim.vhcl.rotors(i).blades)
        for k=1:1:numel(sim.vhcl.rotors(i).blades(j).sections)
            % for each section, first you rotate about the z axis by -pi/2
            % to get the section x aligned with the blade y, then add the
            % blade angle.
            a_C_s = [0 1 0; -1 0 0; 0 0 1]; % from that section to our section
            b_C_a = sim.vhcl.rotors(i).blades(j).b_C_a(:,:,k);
            P_C_b = sim.vhcl.rotors(i).P_C_bx(:,:,j);
            A_C_P = transpose(sim.vhcl.rotors(i).P_C_A);
            A_C_s = A_C_P*P_C_b*b_C_a*a_C_s;
            beta = atan2(A_C_s(1,2),A_C_s(1,1));
            theta = atan2(A_C_s(2,3),A_C_s(3,3));
            gamma = -asin(A_C_s(1,3));
            %thB2R = atan2(sim.vhcl.rotors(i).P_C_bx(1,2,j),sim.vhcl.rotors(i).P_C_bx(1,1,j))+pi/2;
            % Next, rotate about the section y. This would be for precone.
            % Don't have that right now.
            % Finally, rotate about the section x. This is the angle that
            % the section makes in the blade frame.
            %thS2B = atan2(sim.vhcl.rotors(i).blades(j).b_C_a(1,3,k),sim.vhcl.rotors(i).blades(j).b_C_a(1,1,k));
            v.addSection(amsection('ellipse',sim.vhcl.rotors(i).blades(j).sections(k).chord/2,thickness*sim.vhcl.rotors(i).blades(j).sections(k).chord/2,sim.vhcl.rotors(i).blades(j).sections(k).width),...
                sim.vhcl.rotorLocs(:,i)+A_C_P*sim.vhcl.rotors(i).P_C_bx(:,:,j)*sim.vhcl.rotors(i).blades(j).sectLocs(:,k),[beta;gamma;theta]);
        end %sections
        %v.showme('b');
    end %blades
end %rotors

hfig1 = v.showme('k');

if savefigs
    if ~exist('output\figs','dir')
        mkdir('output\figs');
    end
    hfig1.CurrentAxes.Color = 'none';
    hfig1.CurrentAxes.Title.String = ['Sections for ' casetoload ' in the body frame.'];
    hfig1.CurrentAxes.XLabel.String = 'x';
    hfig1.CurrentAxes.YLabel.String = 'y';
    hfig1.CurrentAxes.ZLabel.String = 'z';
    savefig(hfig1,['output\figs\' casetoload '.fig']);
    export_fig(['output\figs\' casetoload], '-png', '-transparent','-m5');
end

% OK, let's see the added mass matrix for this guy
zeroTolPlaces = 14;
MA = v.getAddedMass;
oldMA = [5.83392941639579 5.09575021068187e-18 -2.80916782890994e-16 -7.72494047895922e-18 -5.00901403688303e-17 -4.33680868994202e-19;5.09575021068187e-18 5.83392941639581 -2.45029690981724e-17 1.38561037643647e-16 2.35271871429354e-17 -2.16840434497101e-19;-2.80916782890994e-16 -2.45029690981724e-17 4.83381740053976 1.38777878078145e-17 -6.93889390390723e-18 -2.49366499671666e-18;-7.72494047895922e-18 1.38561037643647e-16 1.38777878078145e-17 0.652246547013703 1.73472347597681e-17 -7.86046575051991e-19;-5.00901403688303e-17 2.35271871429354e-17 -6.93889390390723e-18 1.73472347597681e-17 0.652246547013703 1.95156391047391e-18;-4.33680868994202e-19 -2.16840434497101e-19 -2.49366499671666e-18 -7.86046575051991e-19 1.95156391047391e-18 0.0195603762357379];
diffMA = round((MA - oldMA),zeroTolPlaces);
divMA = oldMA;
divMA(abs(oldMA) < 10^-zeroTolPlaces) = 1;
pdiffMA = round((MA - oldMA)./divMA,zeroTolPlaces);
disp('Added Mass Matrix:'); disp(round(MA,zeroTolPlaces));
disp('Absolute Difference from Tactical Baseline:'); disp(diffMA);
disp('Percent Difference (for non-zero elements, otherwise absolute again):'); disp(pdiffMA);
%MA = round(MA) % round to the nearest integer to remove numerical noise