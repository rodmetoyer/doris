classdef tether < handle
    properties
        numnodes  % number of internal nodes - tether terminates with links
        nodelocs  % 3xnumnodes array of node locations in inertial space
        stiffness % 1xnumnodes array of spring stiffnesses
        damping   % 1xnumnodes array of damping constants
        uslength  % 1xnumnodes+1 unstretched length of the links
    end
    
    methods
        
        function hobj = tether(n,locs,k,c,usl)
            hobj.numnodes = n;
            hobj.nodelocs = locs;
            hobj.stiffness = k;
            hobj.damping = c;
            hobj.uslength = usl;
        end % constructor
        
    end % methds
end% tether