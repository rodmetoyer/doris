function datOut = getEqOrient(casenames,reldensCols)
    addpath('..\src');
    cd ..\ % Working from the top folder
    inputfiles = strcat(casenames,".m");    
    ballastRows = numel(inputfiles)/reldensCols;    
    ssitr = 1;
    steadyTolTime_s = 10;
    steadyTolDeg = 1/10;
    for i=1:1:numel(inputfiles)
        sim = simulation.loadsim(inputfiles(i));
        flowspeed(i) = sim.fld.velocity(1);
        a1(i) = sim.vhcl.centermass(1)/sim.vhcl.body.length;
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
        meanskew(i) = acos(cos(meanyaw(i))*sin(meanpitch(i)));
        meandepth(i) = mean(depth(end-ceil(1/sim.timestep):end));
        meandrift(i) = mean(drift(end-ceil(1/sim.timestep):end));
        meanstreamwise(i) = mean(streamwise(end-ceil(1/sim.timestep):end));
        cm(:,i) = sim.vhcl.centermass/sim.vhcl.body.length;
        try
         if ~isempty(sim.vhcl.ballastpoint)
             ballLoc = sim.vhcl.ballastpoint;
             ballLocRad(i) = ballLoc(1);
             ballLocAx(i) = ballLoc(3);
         else
             error('Old simulation with no ballast point');
         end
        catch % either the ballast location is empty or it doesn't even exist
            % in this case we assume that we are dealing with an old sim
            % where the ballast was always 9000kg.
            ballLocRad(i) = sim.vhcl.mass/9000*sim.vhcl.body.length/sim.vhcl.body.radius*cm(1,i);
            ballLocAx(i) = sim.vhcl.mass/9000*cm(3,i);
        end
        numBladesUpstream(i) = sim.vhcl.rotors(1).numblades;
        numBladesDownstream(i) = sim.vhcl.rotors(2).numblades;
        radUpsream(i) = sim.vhcl.rotors(1).blades(1).length;
        radDownsream(i) = sim.vhcl.rotors(2).blades(1).length;
        ffUpstream(i) = sim.vhcl.rotors(1).axflowfac;
        ffDownstream(i) = sim.vhcl.rotors(2).axflowfac;
        relativeDensity(i) = sim.vhcl.relDensity;
        wff(i) = sim.vhcl.rotors(1).axflowfac;
        lff(i) = sim.vhcl.rotors(2).axflowfac;
    end
    meanpitch = meanpitch*180/pi - 90;
    meanpitch(meanpitch > 180) = mod(meanpitch(meanpitch > 180),360)-360;
    meanpitch(meanpitch < -180) = mod(meanpitch(meanpitch < -180),360)-360;
    meanyaw = meanyaw*180/pi;
    meanskew = meanskew*180/pi;

    datOut.meanpitch = meanpitch;
    datOut.meanyaw = meanyaw;
    datOut.meanskew = meanskew;
    datOut.cmaxial = cm(3,:)*100;
    datOut.cmradial = cm(1,:)*100;
    datOut.wff = wff;
    datOut.lff = lff;
    datOut.relativeDensity = relativeDensity;
    datOut.ballLocRad = ballLocRad;
    datOut.ballLocAx = ballLocAx;
    datOut.flowspeed = flowspeed;
    datOut.meandepth = meandepth;
    datOut.meandrift = meandrift;
    cd tools
end