classdef vehicle < handle
    % The vehicle class.
    % A vehicle may be comprised of a vehicle body and rotors. It does not
    % include any attached tethers.
    % A vehicle is the top-level object that is tethered to another
    % top-level object (typically a mechanical ground).
    % A vehicle is the only object in the tethered system model that knows
    % its position and orientation in inertial space.
    
    properties (SetAccess = private)
        body        % the vehicle body object
        rotors      % nx1 vector of rotor objects
        centermass  % 3x1 vector location of center of mass of the vehicle
        tetherpoint % 3x1 vector of location of the tether attachment point in the vehicle body frame
        buoypoint   % 3x1 vector of location of the center of buoyancy and/or buoy tether attachment point
        % The buoyforce is a property of some other object. buoyforce   % scalar buoyant force magnitide (always in inertial z direction)
        mass        % scalar total mass of the vehicle
        type        % uint identifier | see typeName get method for list of types
    end
    
    properties (Dependent)
        A_C_O       % % 3x3 tranformation matrix from vehicle body frame to inertial frame
        typeName
    end
    
    properties
        position    % 3x1 Position vector of mass center in the inertial frame IN METERS
        orientation % 3x1 orientation vector following 2-1-3 (theta, gamma, beta) IN RADIANS
        velocity    % 3x1 Velocity vector of mass center in the inertial frame IN METERS/SEC
        angvel      % 3x1 Angular velocity vector in the rotor frame (omega_x,omega_y,omega_z)IN RADIANS/SEC
    end
    
    methods
        %% Constructor
        function hobj = vehicle(bod,rot,cm,tp,bp)
            if nargin == 0
                % Make sure to call init
            elseif nargin == 2
               % assume simple single rotor with cm at body [0;0;0];
               hobj.body = vehiclebody;
               hobj.rotors = rot;
               hobj.mass = rot.mass;
               %hobj.buoyforce = 0;
            else
               hobj.rotors = rot;
               hobj.body = bod;
               hobj.centermass = cm;
               hobj.tetherpoint = tp;
               hobj.buoypoint = bp;
               m = 0;
               for i=1:1:numel(rot)
                   m = m + rot(i).mass;
               end
               hobj.mass = m + bod.mass;
               %hobj.buoyforce = bf;
            end
        end
        
        %% Initialization method
        function init(hobj,bod,rot,cm,tp,bp)
            if nargin < 3
                % This is a body-only vehicle (no rotor)
                error('Body-only vehicle not currently supported');
            else
                hobj.body = bod;
                hobj.rotors = rot;
                hobj.centermass = cm;
                hobj.tetherpoint = tp;
                hobj.buoypoint = bp;
                %hobj.buoyforce = bf;
                m = 0;
                for i=1:1:numel(rot)
                    m = m + rot.mass;
                end
                hobj.mass = m + bod.mass;
            end
        end
        
        %% Other class methods
        function sinebuoyforce(hobj,t,a,w)
            hobj.buoyforce = a*sin(w*t);
        end
        
        % Getters
        function m = get.A_C_O(hobj)
            beta = hobj.orientation(3);  cb = cos(beta);  sb = sin(beta);
            gamma = hobj.orientation(2); cg = cos(gamma); sg = sin(gamma);
            theta = hobj.orientation(1); ct = cos(theta); st = sin(theta);
            m = [cb*ct + sb*sg*st, cg*sb, sb*ct*sg - cb*st;cb*sg*st - sb*ct, cb*cg, sb*st + cb*ct*sg;cg*st,-sg,cg*ct];
        end
        function n = get.typeName(hobj)
            switch hobj.type
                case 1
                    n = 'coaxial';
                case 2
                    n = 'parallel';
                case 3
                    n = 'single';
                otherwise
                    n = 'unknown';
            end
        end
        
    end
end