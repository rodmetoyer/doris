classdef generator < handle
    % The generator
    
    properties (SetAccess = private)
        efficiency
    end % end parameters
    properties
        torque
    end % end variable properties
    
    methods
        function hobj = generator(tq,e)
            % ARGUMENTS
                % tq = torque
                % e = efficiency
            if nargin == 0
                hobj.torque = 0;
                hobj.efficiency = 0.9;
            else
                hobj.torque = tq;
                hobj.efficiency = e;
            end
        end
        
        function computeTorque(hobj,p,omega)
            % ARGUMENTS
                % p = power in watts
                % omega = angular speed in rad/s
            if abs(omega) < 10e-9 && p > 1
                error('Generator has stalled');
            end
            hobj.torque = hobj.efficiency*p/omega;
        end
        
    end % end methods
    
end % end generator