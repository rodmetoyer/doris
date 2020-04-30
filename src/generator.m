classdef generator < handle
    % generator A generator modeled as separatly excited dc machine.
    % initiate with:
    %   k   - the machine constant
    %   phi - magnetic flux (from pm or excitation coil)
    %   ra  - the armature resistance
    %   c   - the constant for the viscous friction model
    % The generator begins with an infinite load resistance (i.e. no load).
    %
    % To add a load resistance use the setLoadResistance method.
    
    properties (SetAccess = private)
        kmach   % machine constant
        flux    % magnetic flux
        rarm    % total armature resistance
        rload   % load resistance
        kvisc   % damping constant for viscous friction
        mass    % mass of the generator
    end % end parameters
    
    methods
        % Constructor
        function hobj = generator(k,phi,ra,c,m)
            hobj.kmach = k;
            hobj.flux = phi;
            hobj.rarm = ra;
            hobj.rload = inf; % no load when you first make the generator
            hobj.kvisc = c;
            hobj.mass = m;
        end % generator
        
        function tq = getTorque(hobj,omg)
            k = 0.8; % this is an exponential that determines the shape of viscous friction vs. speed.
            tq = -(hobj.kmach*hobj.flux)^2*omg/(hobj.rarm + hobj.rload) - hobj.kvisc^k*omg;
        end % getTorque
        
        function pwr = getPower(hobj,omg)
            pwr = (hobj.kmach*hobj.flux*omg)^2/(hobj.rarm + hobj.rload);
        end
        
        function i = getArmatureCurrent(hobj,omg)
            i = (hobj.kmach*hobj.flux*omg)/(hobj.rarm + hobj.rload);
        end
        
        function setLoadResistance(hobj,r)
            hobj.rload = r;
        end
    end % end methods    
end % end generator