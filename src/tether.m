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
        
        function t = computeTension(hobj,r,v)
            usleng = hobj.uslength;
            currentlength = norm(r);
            unitvec = r/currentlength;
            stretch = currentlength-usleng; % If it moves from the origin there is a restoring force
            stretchd = dot(r,v)/currentlength;
            if currentlength < 0
                unitvec = [0;0;0];
                stretchd = 0;
            end
            Fmag = 0;
            if stretch > 0
                Fmag = -stretch*hobj.stiffness;
                Fmag = Fmag - stretchd*hobj.damping; % changed to make it damp both out and in as long as it is taut. todo(rodney) investigate what is most accurate
            end
            t = [Fmag*unitvec(1);Fmag*unitvec(2);Fmag*unitvec(3)];
        end
    end % methds
end% tether