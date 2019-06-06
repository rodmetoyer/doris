classdef rotor < handle
    % A rotor must contain one or more blades. In the current
    % implementation, a rotor must contain 2 or more blades with the mass
    % center located at the origin of the rotor plane.
% General information:
% todo(rodney) at a minimum, describe the rotor frame definition
    
% vehicle <aggregate-- rotor <compose-- blade <compose-- bladesection
% --generalize> airfoil
    properties (SetAccess = private)
        % Properties that do not change during simulation
        blades     % 1xn array of blade objects that make up the rotor, n=num blades
        inertia    % 3x3 array intertia matrix in the rotor frame
        numblades  % scalar number of blades in the rotor
        bladeAngls % 3xn array angles of rotation of blades in rotor plane
        P_C_bx     % 3x3xn array tranformation matrix from blade to rotor
        bx_C_P     % 3x3xn array tranformation matrix from rotor to blade
        mass       % scalar rotor mass (kg)
        sectPos    % 3xjxn array of position vectors to blade section locations in the rotor frame, j=num blade sections
        % rotor doesn't know where he is, but he knows which vehicle he belongs to and can get position that way position   % 3x1 Position vector of mass center in the vehicle frame IN METERS
        vehicle    % The vehicle object that the rotor is attached to.
        ID         % Identifier specifying which rotor it is in the vehicle
    end % end private parameters and constants that do not change during simulation
    properties (SetAccess = private)
        % Properties that change every timestep that can only be changed by
        % the rotor class
        sectDrag   % 3xjxn array of drag at each section expressed in the rotor frame
        sectLift   % 3xjxn array of lift at each section expressed in the rotor frame
        sectMomt   % 3xjxn array of moment at each section expressed in the rotor frame
        torqueCM   % 3x1 total torque about the center mass expressed in the rotor frame
    end
    properties
        % Properties that change during simulation
        orientation % 3x1 orientation of the rotor frame w.r.t. the vehicle frame following 1-2-3 (xi1, xi2, xi3) IN RADIANS        
        angvel      % 3x1 Angular velocity of the rotor frame w.r.t. the vehicle frame in the rotor frame (omega_x,omega_y,omega_z)IN RADIANS/SEC
    end % end states    
    properties (Dependent)
        % Properties that are computed only on demand
        velocity   % 3x1 Velocity vector of mass center w.r.t the inertial frame expressed in the vehicle frame IN METERS/SEC
        P_C_A       % 3x3 tranformation matrix from vehicle frame to rotor frame
    end % end dependent properties
    
    methods
        % Constructor
        function hobj = rotor(bladearray,numblades,radius,airfoil,ID)
            % Only manual construction for now. Pass in a vector of blade
            % objects. The constructor method will assemble a rotor of n
            % equally spaced blades where n=numel(blade).
            if nargin > 1
                error('Only manual rotor construction is currently available');
            else
                nblades = numel(bladearray);                
                incr = 360/nblades;
                eta = 0:1:nblades-1;
                eta = eta*incr;
                mass = 0;
                
                I_B = zeros(3,3);
                for i=1:1:nblades
                    ceta = cosd(eta(i)); seta = sind(eta(i));
                    hobj.P_C_bx(:,:,i) = [ceta, -seta, 0; seta, ceta, 0; 0 0 1];
                    hobj.bx_C_P(:,:,i) = [ceta, seta, 0; -seta, ceta, 0; 0 0 1];
                    I_bj = bladearray(i).inertia;
                    mass = mass + bladearray(i).mass;
                    % Rotate and add him to the rotor tensor
                    I_B = I_B + hobj.P_C_bx(:,:,i)*I_bj*hobj.bx_C_P(:,:,i);
                    for j=1:1:bladearray(i).numsects
                        hobj.sectPos(:,j,i) = hobj.P_C_bx(:,:,i)*bladearray(i).sectLocs(:,j);
                    end
                end
            end
            hobj.inertia = I_B;
            hobj.blades = bladearray;
            hobj.numblades = nblades;
            hobj.bladeAngls = eta;
            mass = bladearray.mass;
            hobj.mass = mass;
            hobj.ID = [];
        end % end constructor
        
        %% Class methods        
        
        function [netforce, nettorque] = computeNetAeroLoads(hobj,fluid)
        % computeNetAeroLoads
            % Computes the aerodynamic loads on the rotor using simple
            % assumptions and returns force and torque in the rotor frame.
            % Assumption 1: stable quasi-static flow
            % Assumption 2: rotor and components do not affect the flow 
                % (i.e. no induction)
            % Assumption 3: aerodyamic loads from sections act only in the
                % section planes
            error('This method is currently broken');
            % todo(rodney) fix it? Make it into torque about cm rotor? I
            % think the best way to do this is to have two methods. First
            % is compute forces at sections. Second is compute torques
            % about center mass. That way vehicle can call compute forces
            % at sections and then use those forces to compute torques
            % about center mass of the vehicle.
            
            % force and torque in the rotor (B) frame
            netforce = [0;0;0];
            nettorque = [0;0;0];
            % Rotate fluid velocity to rotor frame
            
            Ufluid_P = hobj.P_C_A*hobj.vehicle.A_C_O*fluid.velocity; % Think about how to do this for a vector field for future functionality
            % Rotate rotor center mass velocity to rotor frame
            V_Bx_O_P = hobj.P_C_A*hobj.vehicle.A_C_O*hobj.velocity;
            % Get aero loads for each blade
            for i=1:1:hobj.numblades
                % Get section aero loads
                for j=1:1:hobj.blades(i).numsects
                    % Rotate coordinates to this section in the rotor
                    % frame. NOTE that this assumes that the blade root is
                    % at the rotor frame origin. Need to generalize. Do
                    % that by adding a bladerootlocations property on the
                    % rotor.
                    r_p_b = hobj.P_C_bx(:,:,i)*hobj.blades(i).sectionLocs(:,j);
                    % Compute relative velocity in the rotor frame at the
                    % section location.
                    V_p_Bx_B = cross(hobj.angvel,r_p_b);
                    % Sum for relative velocity of the fluid w.r.t. the
                    % blade section expressed in the rotor frame
                    U_rel = Ufluid_P - V_Bx_O_P - V_p_Bx_B;
                    
                    % Rotate the relative velocity into the blade section
                    % frame. First, compute bx_C_a (section frame to blade
                    % frame)
                    ang = hobj.blades(i).sectOrnts(2,j); % Note that this assumes only twist about y axis. todo(rodney) make this more general.
                    bx_C_a = [cos(-ang),0,-sin(-ang);0,1,0;sin(-ang),0,cos(-ang)];
                    
                    U_rel_bs = transpose(bx_C_a)*hobj.bx_C_P(:,:,i)*U_rel;
                    
                    % Get loads from each section of the blade. INFO: The
                    % computeLoads method only uses the x and z components
                    % of the relative velocity vector.
                    [lift,drag,~] = hobj.blades(i).sections(j).computeLoads(U_rel_bs,fluid);
                    %[lift,drag,~] = hobj.blades(i).sections(j).computeLoadsFast(U_rel_bs,fluid);
                    % No moments right now so I'm not dealing with them.
                    % Need to add that functionality.                    
                    
                    % Transform these loads into the rotor frame
                    drag = hobj.P_C_bx(:,:,i)*bx_C_a*drag;
                    lift = hobj.P_C_bx(:,:,i)*bx_C_a*lift;
                    
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
        % THIS METHOD IS OBSOLETE
        function [force, torque, U_relSections, LiftSections, DragSections] = computeAeroLoadArrays(hobj,fluid)
            % This method computes the loads on a rotor and returns the
            % loads as arrays of size 3xnxm where n is number of sections
            % in each blade and m is the number of blades (currently the
            % number of sections must be equal for each blade).
            % INPUTS:
                % hobj = rotor object handle
                % fluid = fluid object handle
            % OUTPUTS:
                % force = 3xnxm array of force vectors at each blade section
                % torque = 3xnxm array of torque vectors at each section
                % U_relSections = 3xnxm array of relative velocity vectors at 
                % LiftSections
                % DragSections            
            % Rotate fluid velocity to rotor frame
            error('Obsolete method. Use computeHydroLoads');
        end % end computeAeroLoadArrays
        
        function [U_relSections] = computeHydroLoads(hobj,fluid)
            % Computes the hydrodynamic loads at all sections in the rotor
            % as well as the total hydrodynamic torque about the center
            % mass of the rotor given the current state of the rotor.
            % INPUTS:
                % A fluid object - For now only need fluid object because uniform flow and no fluid-structure interaction.
            % OUTPUTS: 
                % The relative velocity at all of the sections in the
                % rotor. This is returned primarily for verification as it
                % would be a pain in the ass to get it externally and we
                % need to compute it anyway.
            % UPDATED Properties:
                % sectLift
                % sectDrag
                % sectMomt
                % torqueCM            
            
            % Rotate the fluid velocity into the rotor frame.
            % todo(rodney) Think about how to do this for a vector field for future functionality.
            Ufluid_P = hobj.P_C_A*hobj.vehicle.A_C_O*fluid.velocity;
            % Rotate rotor center mass velocity to rotor frame
            OVpo_P = hobj.P_C_A*hobj.velocity; % OVpo_P is O derivative of r_<to><from>_<ExprsdFrame>
            
            % Get aero loads for each blade
            % Preallocate the output and container arrays
            % NOTE: This assumes that the number of sections is the same in
            % each blade. I'm OK with that for now because we use the same
            % assumption in the loop. Need to fix this when we generalize.
            U_relSections = NaN(3,hobj.blades(1).numsects,hobj.numblades); % todo fix this when you make the number of sections per blade arbitrary.
            LiftSections = U_relSections;
            DragSections = U_relSections;
            ForceSections = U_relSections;
            % Init torque sum
            torque = 0;
            for i=1:1:hobj.numblades
                % Get section aero loads
                for j=1:1:hobj.blades(i).numsects
                    % Rotate coordinates to this section in the rotor
                    % frame. NOTE that this assumes that the blade root is
                    % at the rotor frame origin. Need to generalize.
                    rap_P = hobj.P_C_bx(:,:,i)*hobj.blades(i).sectLocs(:,j); % hobj.blades(i).sectionLocs(:,j) is rap_bx
                    % Compute relative velocity in the rotor frame at the
                    % section location.
                    V_p_P = cross(hobj.angvel,rap_P);
                    U_rel_P = Ufluid_P - OVpo_P - V_p_P;
                    U_relSections(:,j,i) = U_rel_P; % Velocity vectors for blade i section j in the rotor frame.
                    
                    % Rotate the relative velocity into the blade section
                    % frame. First, compute b_this_i_C_a
                    ang = hobj.blades(i).sectOrnts(2,j); % Note that this assumes only twist about y axis. todo(rodney) make this more generic.
                    bx_C_a = [cos(-ang),0,-sin(-ang);0,1,0;sin(-ang),0,cos(-ang)];
                    % U in section frame =
                    % section_C_blade*blade_C_rotor*U_rotor
                    U_rel_bs = transpose(bx_C_a)*hobj.bx_C_P(:,:,i)*U_rel_P;
                    
                    % Get loads from each section of the blade. INFO: The
                    % computeLoads method only uses the x and z components
                    % of the relative velocity vector.
                    [lift,drag,~] = hobj.blades(i).sections(j).computeLoads(U_rel_bs,fluid);
                    %[lift,drag,~] = hobj.blades(i).sections(j).computeLoadsFast(U_rel_bs,fluid);
                    % No moments right now so I'm not dealing with them.
                    % todo(rodney) add section moment functionality.                    
                                        
                    % Transform these loads into the rotor frame
                    drag = hobj.P_C_bx(:,:,i)*bx_C_a*drag;
                    lift = hobj.P_C_bx(:,:,i)*bx_C_a*lift;
                    
                    LiftSections(:,j,i) = lift; % todo This will crap out if the number of sections is differnt in each blade. Make cells to genericize
                    DragSections(:,j,i) = drag;
                    % TorqueSections(:,j,i) % Not currently functional
                    ForceSections(:,j,i) = lift + drag;
                    
                    % Compute torque from loads
                    tauprev = cross(rap_P,ForceSections(:,j,i));
                    torque = torque + tauprev;
                end % end bladesection loop               
            end % end blade loop
            % Update class properties
                    hobj.sectLift = LiftSections;
                    hobj.sectDrag = DragSections;
                    % hobj.sectMomt = TorqueSections;
                    hobj.torqueCM = torque; % + sum(TorqueSections) I think, right?
        end % end computeHydroLoads
        
        function connectVehicle(hobj,v)
            % Arguments:
                % v = the vehicle object to connect to
            hobj.vehicle = v;
        end % end ConnectVehicle
        
        % Setters
%         function set.position(hobj,p)
%             if numel(p) ~= 3
%                 error('rotor: Position must be 3x1 vector');
%             end
%             [m,~] = size(p);
%             if m < 3
%                 warning('rotor: Someone just gave me a 1x3 position vector');
%                 p = p.';
%             end
%             hobj.position = p;
%         end
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
%         function set.velocity(hobj,v)
%             if numel(v) ~= 3
%                 error('rotor: Velocity must be 3x1 vector');
%             end
%             [m,~] = size(v);
%             if m < 3
%                 warning('rotor: Someone just gave me a 1x3 velocity vector');
%                 v = v.';
%             end
%             hobj.velocity = v;
%         end
        function set.angvel(hobj,av)
            if numel(av) ~= 3
                error('rotor: angvel must be 3x1 vector');
            end
            [m,~] = size(av);
            if m < 3
                warning('rotor: Someone just gave me a 1x3 angvel vector');
                av = av.';
            end
            hobj.angvel = av;
        end
        
        function setID(hobj,ID)
            hobj.ID = ID;
        end
        
        % Getters
        function m = get.P_C_A(hobj)
            xi3 = hobj.orientation(3);  c3 = cos(xi3);  s3 = sin(xi3);
            xi2 = hobj.orientation(2); c2 = cos(xi2); s2 = sin(xi2);
            xi1 = hobj.orientation(1); c1 = cos(xi1); s1 = sin(xi1);
            m = [c2*c3,  s3*c1+c3*s1*s2, s3*s1-c3*c1*s2;...
                 -s3*c2, c3*c1-s3*s1*s2, c3*s1+s3*c1*s2;...
                 s2,     -c2*s1,         c1*c2];
        end
        
        function v = get.velocity(hobj)
            v = cross(hobj.vehicle.angvel,hobj.vehicle.rotorLocs(:,hobj.ID))+hobj.vehicle.velocity;
        end
        
    end % end methods
end % end classdef