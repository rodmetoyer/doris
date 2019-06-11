classdef vehicle < handle
    % The vehicle class.
    % A vehicle may be comprised of a vehicle body and rotors. It does not
    % include any attached tethers.
    % A vehicle is the top-level object that is tethered to another
    % top-level object (typically a mechanical ground).
    % A vehicle is the only object in the tethered system model that knows
    % its position and orientation in inertial space.
    
    properties (SetAccess = private)
        % Objects
        body        % the vehicle body object
        rotors      % nx1 vector of rotor objects n is the number of rotors
        % Points expressed in the vehicle (A) frame
        rotorLocs   % 3xn vector of rotor locations expressed in the vehicle frame where n is the number of rotors (g1,g2,g3) and (h1,h2,h3)
        centermass  % 3x1 vector location of center of mass of the vehicle body in the vehicle frame (c1,c2,c3)
        tetherpoint % 3xm vector of location of the tether attachment point(s) in the vehicle frame (t1, t2, t3)
        buoypoint   % 3x1 vector of location of the center of buoyancy (b1, b2, b3)
        mass        % scalar total mass of the vehicle
        type        % uint identifier | see typeName get method for list of types
    end
    
    properties (Dependent)
        A_C_O       % 3x3 tranformation matrix from vehicle frame to inertial frame
        typeName    % Name of the type
    end
    
    properties
        % Vehicle state variables
        position    % 3x1 Position vector of mass center in the inertial frame IN METERS (x1,x2,x3)
        orientation % 3x1 orientation of the vehicle frame w.r.t. the inertial frame following 2-1-3 (theta, gamma, beta) IN RADIANS
        velocity    % 3x1 Velocity vector of mass center in the inertial frame IN METERS/SEC (dx1,dx2,dx3)
        angvel      % 3x1 Angular velocity vector in the VEHICLE frame (omega_x,omega_y,omega_z)IN RADIANS/SEC
    end
    
    properties
        % Vehicle loads
        force       % 3x1 Net aerodynamic force in the vehicle body frame
        torque      % 3x1 Net torque about the centermass of the vehicle system (which is the origin of the vehicle frame)
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
        function init(hobj,bod,rot,rotLocs,cm,tp,bp)
            if nargin < 3
                % This is a body-only vehicle (no rotor)
                error('Body-only vehicle not currently supported');
            else
                hobj.body = bod;
                hobj.rotors = rot;
                hobj.centermass = cm; % todo compute the center of mass of the vehicle from cm of vbod and rotors
                hobj.tetherpoint = tp;
                hobj.buoypoint = bp;
                %hobj.buoyforce = bf;
                hobj.rotorLocs = rotLocs;
                m = 0;
                switch numel(rot)
                    case 0
                        hobj.type = 0;
                    case 1
                        hobj.type = 1;
                    case 2
                        hobj.type = 2;
                        warning('VEHICLE:construction','Assuming vehicle type is coaxial. Use setType to change after init if this is incorrect.');
                    otherwise
                        error('Only 0, 1, or 2 rotor systems currently supported');
                end
                if numel(rot) >0
                    for i=1:1:numel(rot)
                        m = m + rot(i).mass;                    
                    end
                end
                hobj.mass = m + bod.mass;
            end
        end
        
        %% Other class methods
        
        function computeHydroLoads(hobj,f)
            % Computes the net hydrodynamic force and torque vectors
            frc = 0;
            trq = 0;
            % Get aerodynamic loads on rotors, cross and sum
            for i=1:1:numel(hobj.rotors)
                hobj.rotors(i).computeHydroLoads(f);
                % 3xjxn array of lift and drag at each section expressed in the rotor frame
                sectionforces = hobj.rotors(i).sectLift + hobj.rotors(i).sectDrag;
                % Aerodynamic moments not currently functional. When we go to implement them
                % I think we can just sum the section moments.
                % sectiontorques(:,:,:,i) = hobj.rotors(i).sectMomt;
                r_ayxA_A = hobj.rotorLocs(i) + hobj.rotors(i).sectPos;
                trq = trq + cross(r_ayxA_A,sectionforces);
                frc = frc + sectionforces;
            end
            
            % Get aerodynamic loads on body, cross and sum
            % todo(rodney) add vehicle hydrodynamic loads.
            
            % Sum along the 2dim gives 3x1xnumBlades of total blade loads
            % then sum along the 3dim to get 3x1 vector of total loads
            hobj.force = sum(sum(frc,2),3);
            hobj.torque = sum(sum(trq,2),3);
        end
        
        function addTetherLoads(hobj,frc)
            % Adds the tether loads to the total vehicle force and torque
            % INPUTS:
                % frc = 3xm force in the vehicle frame
            [~,numloads] = size(frc);
            for i=1:1:numloads
                hobj.force = hobj.force + frc(:,i);
                hobj.torque = hobj.torque + cross(hobj.tetherpoint(:,i),frc(:,i));
            end
        end
        
        function addRotor(hobj,loc)
            % Concatenate as attached
            hobj.rotorLocs = [hobj.rotorLocs,loc];
        end
            
        
        function sinebuoyforce(hobj,t,a,w)
            hobj.buoyforce = a*sin(w*t);
        end
        
        % Setters
        function setType(hobj,t)
            hobj.type = t;
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
                case 0
                    n = 'rotorless';
                case 1
                    n = 'single';
                case 2
                    n = 'coaxial';
                case 3
                    n = 'parallel';
                otherwise
                    n = 'unknown';
            end
        end
        
    end
end