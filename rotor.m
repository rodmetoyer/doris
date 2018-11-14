classdef rotor < handle
    % A rotor must contain one or more blade. In the current
    % implementation, a rotor must contain 2 or more blades with the mass
    % center located at the origin of the rotor plane.
    properties (SetAccess = private)
        blades     % 1xn vector of blade objects that make up the rotor
        inertia    % 3x3 intertia tensor in the rotor frame
        numblades  % Number of blades in the rotor
        bladeAngls % 3xn vector angles of rotation of blades in rotor plane
        B_C_bx     % 3x3xn vector tranformation matrix from blade to rotor
        bx_C_B     % 3x3xn vector tranformation matrix from rotor to blade
        mass       % scalar rotor mass (kg)
        sectPos    % 3xjxn array of position vectors to blade section locations in the rotor frame, j=num blade sections
    end
    
    properties
        position    % 3x1 Position of mass center in the inertial frame
        orientation % 3x1 orientation following 2-1-3 (theta, gamma, beta)
        velocity    % 3x1 Velocity of mass center in the inertial frame
        angvel      % 3x1 Angular Velocity in the rotor frame (omega_x,omega_y,omega_z)
    end
    
    methods
        % Constructor
        function hobj = rotor(blade,numblades,radius,airfoil)
            % Only manual construction for now. Pass in a vector of blade
            % objects. The constructor method will assemble a rotor of n
            % equally spaced blades where n=numel(blade).
            if nargin > 1
                error('Only manual rotor construction is currently available');
            else
                nblades = numel(blade);                
                incr = 360/nblades;
                eta = 0:1:nblades-1;
                eta = eta*incr;
                mass = 0;
                
                I_B = zeros(3,3);
                for i=1:1:nblades
                    ceta = cosd(eta(i)); seta = sind(eta(i));
                    hobj.B_C_bx(:,:,i) = [ceta, -seta, 0; seta, ceta, 0; 0 0 1];
                    hobj.bx_C_B(:,:,i) = [ceta, seta, 0; -seta, ceta, 0; 0 0 1];
                    I_bj = blade(i).inertia;
                    mass = mass + blade(i).mass;
                    % Rotate and add him to the rotor tensor
                    I_B = I_B + hobj.B_C_bx(:,:,i)*I_bj*hobj.bx_C_B(:,:,i);
                    for j=1:1:blade(i).numsects
                        hobj.sectPos(:,j,i) = hobj.B_C_bx(:,:,i)*blade(i).sectionLocs(:,j);
                    end
                end
            end
            hobj.inertia = I_B;
            hobj.blades = blade;
            hobj.numblades = nblades;
            hobj.bladeAngls = eta;
            mass = blade.mass;
            hobj.mass = mass;
        end
        
        %%%%% Class methods %%%%%
        
        % computeAeroLoadsBasic
        % Computes the aerodynamic loads on the rotor using simple
        % assumptions and returns force and torque in the rotor frame.
            % Assumption 1: stable quasi-static flow
            % Assumption 2: rotor and components do not affect the flow 
                % (i.e. no induction)
            % Assumption 3: aerodyamic loads from sections act only in the
                % section planes
        function [netforce, nettorque] = computeAeroLoadsBasic(hobj,fluid)
            % force and torque in the rotor (B) frame
            netforce = [0;0;0];
            nettorque = [0;0;0];
            % Get current orientation and make B_C_O matrix
            beta = hobj.orientation(3);  cb = cos(beta);  sb = sin(beta);
            gamma = hobj.orientation(2); cg = cos(gamma); sg = sin(gamma);
            theta = hobj.orientation(1); ct = cos(theta); st = sin(theta);
            B_C_O = [cb*ct + sb*sg*st, cg*sb, sb*ct*sg - cb*st;cb*sg*st - sb*ct, cb*cg, sb*st + cb*ct*sg;cg*st,-sg,cg*ct]; 
            % Rotate fluid velocity to rotor frame
            Ufluid_B = B_C_O*fluid.velocity; % Think about how to do this for a vector field for future functionality
            % Rotate rotor center mass velocity to rotor frame
            V_Bx_O_B = B_C_O*hobj.velocity;
            % Get aero loads for each blade
            for i=1:1:hobj.numblades
                % Get section aero loads
                for j=1:1:hobj.blades(i).numsects
                    % Rotate coordinates to this section in the rotor
                    % frame. NOTE that this assumes that the blade root is
                    % at the rotor frame origin. Need to generalize. Do
                    % that by adding a bladerootlocations property on the
                    % rotor.
                    r_p_b = hobj.B_C_bx(:,:,i)*hobj.blades(i).sectionLocs(:,j);
                    % Compute relative velocity in the rotor frame at the
                    % section location.
                    V_p_Bx_B = cross(hobj.angvel,r_p_b);
                    U_rel = Ufluid_B - V_Bx_O_B - V_p_Bx_B;
                    
                    % Rotate the relative velocity into the blade section
                    % frame. First, compute bx_C_a (section frame to blade
                    % frame)
                    ang = hobj.blades(i).orientations(2,j); % Note that this assumes only twist about y axis. todo(rodney) make this more general.
                    bx_C_a = [cosd(-ang),0,-sind(-ang);0,1,0;sind(-ang),0,cosd(-ang)];
                    
                    U_rel_bs = transpose(bx_C_a)*hobj.bx_C_B(:,:,i)*U_rel;
                    
                    % Get loads from each section of the blade. INFO: The
                    % computeLoads method only uses the x and z components
                    % of the relative velocity vector.
                    [lift,drag,~] = hobj.blades(i).sections(j).computeLoads(U_rel_bs,fluid);
                    % No moments right now so I'm not dealing with them.
                    % Need to add that functionality.                    
                    
                    % Transform these loads into the rotor frame
                    drag = hobj.B_C_bx(:,:,i)*bx_C_a*drag;
                    lift = hobj.B_C_bx(:,:,i)*bx_C_a*lift;
                    
                    % Add them and compute torque from loads
                    sectionforce = lift+drag;
                    tau = cross(r_p_b,sectionforce);
                    
                    % Add aerodynamic torque
                    % todo(rodney) add this functionality
                    
                    % Add everything to the rotor load vectors
                    netforce = netforce + sectionforce;
                    nettorque = nettorque + tau;
                end % end bladesection loop
                % Add any blade force
                % todo(rodney) add the loads along the length of the balde
                % that are ignored in the section method. For this basic
                % method they will be modeled as loads on a cylinder.                
            end % end blade loop
        end % end computeAeroLoadsBasic
        
        % computeAeroLoadArrays computes a force and moment vector in the rotor frame for each
        % bladesection object in the rotor object
        function [force, torque, U_relSections, LiftSections, DragSections] = computeAeroLoadArrays(hobj,fluid)
            % Get current orientation and make B_C_O matrix
            beta = hobj.orientation(3);  cb = cosd(beta);  sb = sind(beta);
            gamma = hobj.orientation(2); cg = cosd(gamma); sg = sind(gamma);
            theta = hobj.orientation(1); ct = cosd(theta); st = sind(theta);
            B_C_O = [cb*ct + sb*sg*st, cg*sb, sb*ct*sg - cb*st;cb*sg*st - sb*ct, cb*cg, sb*st + cb*ct*sg;cg*st,-sg,cg*ct]; 
            % Rotate fluid velocity to rotor frame
            Ufluid_B = B_C_O*fluid.velocity; % Think about how to do this for a vector field for future functionality
            % Rotate rotor center mass velocity to rotor frame
            V_Bx_O = B_C_O*hobj.velocity;
            
            % Get aero loads for each blade
            for i=1:1:hobj.numblades
                % Get section aero loads
                for j=1:1:hobj.blades(i).numsects
                    % Rotate coordinates to this section in the rotor
                    % frame. NOTE that this assumes that the blade root is
                    % at the rotor frame origin. Need to generalize.
                    r_p_b = hobj.B_C_bx(:,:,i)*hobj.blades(i).sectionLocs(:,j);
                    % Compute relative velocity in the rotor frame at the
                    % section location.
                    V_p_Bx = cross(hobj.angvel,r_p_b);
                    U_rel_B = Ufluid_B - V_Bx_O - V_p_Bx;
                    U_relSections(:,j,i) = U_rel_B; % Velocity vectors for blade i section j in the rotor frame.
                    
                    % Rotate the relative velocity into the blade section
                    % frame. First, compute b_this_i_C_a
                    ang = hobj.blades(i).orientations(2,j); % Note that this assumes only twist about y axis. todo(rodney) make this more generic.
                    bx_C_a = [cosd(-ang),0,-sind(-ang);0,1,0;sind(-ang),0,cosd(-ang)];
                    % U in section frame =
                    % section_C_blade*blade_C_rotor*U_rotor
                    U_rel_bs = transpose(bx_C_a)*hobj.bx_C_B(:,:,i)*U_rel_B;
                    
                    % Get loads from each section of the blade. INFO: The
                    % computeLoads method only uses the x and z components
                    % of the relative velocity vector.
                    [lift,drag,~] = hobj.blades(i).sections(j).computeLoads(U_rel_bs,fluid);
                    % No moments right now so I'm not dealing with them.
                    % Need to add that functionality.                    
                                        
                    % Transform these loads into the rotor frame
                    drag = hobj.B_C_bx(:,:,i)*bx_C_a*drag;
                    lift = hobj.B_C_bx(:,:,i)*bx_C_a*lift;
                    
                    LiftSections(:,j,i) = lift;
                    DragSections(:,j,i) = drag;
                    force(:,j,i) = lift + drag;
                    
                    % Compute torque from loads
                    tau = cross(r_p_b,force(:,j,i));
                    torque(:,j,i) = tau;
                end % end bladesection loop               
            end % end blade loop            
        end % end computeAeroLoadArrays
        
        
        % Setters
        function set.position(hobj,p)
            if numel(p) ~= 3
                error('rotor: Position must be 3x1 vector');
            end
            [m,~] = size(p);
            if m < 3
                warning('rotor: Someone just gave me a 1x3 position vector');
                p = p.';
            end
            hobj.position = p;
        end
        function set.orientation(hobj,r)
            if numel(r) ~= 3
                error('rotor: orientation must be 3x1 vector');
            end
            [m,~] = size(r);
            if m < 3
                warning('rotor: Someone just gave me a 1x3 orientation vector');
                r = r.';
            end
            hobj.orientation = r;
        end
        function set.velocity(hobj,v)
            if numel(v) ~= 3
                error('rotor: Velocity must be 3x1 vector');
            end
            [m,~] = size(v);
            if m < 3
                warning('rotor: Someone just gave me a 1x3 velocity vector');
                v = v.';
            end
            hobj.velocity = v;
        end
        function set.angvel(hobj,av)
            if numel(av) ~= 3
                error('rotor: Position must be 3x1 vector');
            end
            [m,~] = size(av);
            if m < 3
                warning('rotor: Someone just gave me a 1x3 angvel vector');
                av = av.';
            end
            hobj.angvel = av;
        end
        
        % Getters
        
    end % end methods
end % end classdef