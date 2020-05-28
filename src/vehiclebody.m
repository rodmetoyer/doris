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
        torsionMod % to modify the dissipating viscous torsion (e.g. baffles)
        torsSelect % section parameter to choose between hifi and low-fi torsion model
        normCoeff  % normal force coefficient
        axCoeff    % axial force coeficient
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
        
        function computeNormalCoeff(hobj,Re)
            % todo finish this
            % using the equations in huston1981representation
            if Re < 1
                hobj.normCoeff = 10;
            elseif Re < 30
                hobj.normCoeff = 1.45+8.55*Re^(-0.4);
            elseif Re < 100
                hobj.normCoeff = 1.0+4*Re^(-0.5);
            elseif Re < 1000
                hobj.normCoeff = 2.25 - 0.45*log10(Re);
            elseif Re < 4000
                hobj.normCoeff = 0.9;
            elseif Re < 15000
                hobj.normCoeff = -1.05 + 0.54*log10(Re);
            elseif Re < 150000
                hobj.normCoeff = 1.21;
            else
                hobj.normCoeff = 0.3;
            end
        end
        
        function setLength(hobj,l)
            hobj.length = l;
        end
        
        function setRadius(hobj,r)
            hobj.radius = r;
        end
        function setViscTorsionModifier(hobj,tm,hf)
            hobj.torsionMod = tm;
            hobj.torsSelect = hf;
        end
        function setViscDamping(hobj,n,a)
            hobj.normCoeff = n;
            hobj.axCoeff = a;
        end
    end % methods   
end % vehiclebody