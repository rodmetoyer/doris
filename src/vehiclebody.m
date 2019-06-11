classdef vehiclebody < handle
    % The vehilce body
    % A vehicle body is a special component of the vehicle. The vehicle
    % frame is attached to the vehicle body. A vehicle must contain 
    
    properties (SetAccess = private)
        mass       % scalar mass of the vehicle body
        centermass % 3x1 vector location of center of mass of the vehicle body
        relDensity % The total density of the vehicle body relative to water
        inertia    % 3x3 array intertia matrix in the vehicle frame
    end % properties
    
    methods
        % Constructor
        function hobj=vehiclebody(m,I)
            hobj.mass = m;
            hobj.inertia = I;
        end
        
        function setRelativeDensity(hobj,rd)
            hobj.relDensity = rd;
        end        
    end % methods   
end % vehiclebody