classdef vehiclebody < handle
    % The vehilce body
    % A vehicle body is a special component of the vehicle. The vehicle
    % frame is attached to the vehicle body. A vehicle must contain 
    
    properties (SetAccess = private)
        mass       % scalar mass of the vehicle body
        centermass % 3x1 vector location of center of mass of the vehicle body (c1, c2, c3)
        inertia    % 3x3 array intertia matrix in the vehicle frame
        length     % the length of a slender body
        radius     % the radius of a slender body
    end % properties
    
    methods
        % Constructor
        function hobj=vehiclebody(m,I,cm)
            if nargin < 1
                m = 0;
                I = zeros(3);
            end
            hobj.mass = m;
            hobj.inertia = I;
            hobj.centermass = cm;
        end                   
        
        function setLength(hobj,l)
            hobj.length = l;
        end
        
        function setRadius(hobj,r)
            hobj.radius = r;
        end
    end % methods   
end % vehiclebody