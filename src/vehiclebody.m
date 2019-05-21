classdef vehiclebody < handle
    % The vehilce body
    
    properties (SetAccess = private)
        mass    % scalar mass of the vehicle body        
    end % properties
    
    methods
        % Constructor
        function hobj=vehiclebody(m)
            hobj.mass = m;
        end
        
    end % methods
    
end % vehiclebody