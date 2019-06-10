function twist = computeTwist(aoaopt,bladeLength,bladeDZfrac,numSections,numBlades)
    rtos = 2.0;    % This is the ratio of rotor radius to disturbed fluid
    % stream length for computing optimal TSR. A good rule-of-thumb value is
    % 2.0 (Ragheb and Ragheb 2011).
    AoAopt_deg = aoaopt; % Optimal angle of attack for the airfoil todo(rodney) needs to be on the airfoil object
    % Compute optimal TSR using method described in (Ragheb and Ragheb 2011)
    TSRopt = 2*pi/numBlades*rtos;
    % Compute Twist using method described in Ch. 5 of (Gasch and Twele 2012)
    locs = linspace(bladeLength*bladeDZfrac,bladeLength,numSections);
    twist = atand(2/3*bladeLength/TSRopt*1./locs)-AoAopt_deg;
    twist = twist*pi/180;
end