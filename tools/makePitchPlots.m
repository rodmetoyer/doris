% makePitchPlots
% Make Pitch Plots

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder

% get sims
sim(1) = simulation.loadsim('EFScase58Long');
sim(2) = simulation.loadsim('EFScase58LongFromUnder');
sim(3) = simulation.loadsim('EFScase58LongUnsteady');
sim(4) = simulation.loadsim('EFScase58LongUnsteadyFromUnder');
sim(5) = simulation.loadsim('EFScase36Long');
sim(6) = simulation.loadsim('EFScase36LongUnsteadyExtended2');
sim(7) = simulation.loadsim('EFScase36LongUnsteadyExtended2Extended');

for i=1:1:numel(sim)
    theta = sim(i).states(:,4); 
    gamma = sim(i).states(:,5); 
    beta = sim(i).states(:,6);  
    
    for j=1:1:length(theta)
        ct = cos(theta(j)); st = sin(theta(j));
        cg = cos(gamma(j)); sg = sin(gamma(j));
        cb = cos(beta(j));  sb = sin(beta(j));     
        A_C_O = [cb*ct + sb*sg*st, cg*sb, sb*ct*sg - cb*st;cb*sg*st - sb*ct, cb*cg, sb*st + cb*ct*sg;cg*st,-sg,cg*ct];
        O_C_A = transpose(A_C_O);
        ro = O_C_A*sim(i).vhcl.rotorLocs(:,2);
        pitch(j,i) = -asind(ro(3)/sim(i).vhcl.rotorLocs(3,2));
    end
   time = sim(i).times;
   hvhcl = sim(i).vhcl;
   eqtheta = atan2d(2*hvhcl.centermass(1)*hvhcl.relDensity,hvhcl.body.length*(1-hvhcl.relDensity)-2*hvhcl.centermass(3)*hvhcl.relDensity);
   eqpitch = eqtheta - 90;
   figure   
   plot(time/3600,pitch(:,i),'r')
   xlabel('Time (hr)');
   ylabel('Pitch (deg)');
   yline(eqpitch);
   imagedir = [pwd '\tools\products\images\' sim(i).name];
   if ~exist(imagedir,'dir')
       mkdir(imagedir);
   end
   set(gca,'Color','none')
   export_fig(gcf,[imagedir '\pitchTimeEvo.png'],'-png','-transparent','-m3');
end
cd tools