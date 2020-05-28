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
        axflowfac  % Axial flow factor (1-induction factor)
        BEMT       % 3x1 bool array [use BEMT, use Prandtl's factor, use Glauert correction]
    end % end private parameters and constants that do not change during simulation
    properties (SetAccess = private)
        % Properties that change every timestep that can only be changed by
        % the rotor class
        sectDrag   % 3xjxn array of drag at each section expressed in the rotor frame
        sectLift   % 3xjxn array of lift at each section expressed in the rotor frame
        sectMomt   % 3xjxn array of moment at each section expressed in the rotor frame
        torqueCM   % 3x1 total hydrodynamic torque about the center mass expressed in the rotor frame
        force      % 3x1 total hydrodynamic force expressed in the rotor frame
    end % end properties that change every timestep
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
            hobj.mass = mass;
            hobj.ID = [];
        end % end constructor
        
        %% Class methods
        
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
                % rotor.velocity returns the velocity in the vehicle frame
                % todo not sure why I do it this way. See if you can just
                % % return in the rotor frame. That would be easier to
                % remember. He gets used in vehicle.
            % Note that rotor.velocity is dependent - it is computed in the
            % get method.
            OVpo_P = hobj.P_C_A*hobj.velocity; % OVpo_P is O derivative of r_<to><from>_<ExprsdFrame>
            
            % Get aero loads for each blade
            % Preallocate the output and container arrays
            % NOTE: This assumes that the number of sections is the same in
            % each blade. I'm OK with that for now because we use the same
            % assumption in the loop. Need to fix this when we generalize.
            % todo fix this if/when you make the number of sections per blade arbitrary.
            U_relSections = NaN(3,hobj.blades(1).numsects,hobj.numblades); 
            LiftSections = U_relSections;
            DragSections = U_relSections;
            ForceSections = U_relSections;
            % Init torque sum
            torque = [0;0;0];
            totforce = [0;0;0];
            opts = optimset('Display','off'); % fzero options for BEMT empirical
            for i=1:1:hobj.numblades
                % Get section aero loads
                for j=1:1:hobj.blades(i).numsects
                    % Get section positions
                    rap_P = hobj.sectPos(:,j,i);
                    % Compute relative velocity in the rotor frame at the
                    % section location.
                        % Need {O_W_A}_P
                    O_W_P = hobj.P_C_A*hobj.vehicle.angvel + hobj.angvel;
                    V_ap_P = cross(O_W_P,rap_P); % O velocity of a w.r.t. p in frame P
                    U_rel_P = Ufluid_P - OVpo_P - V_ap_P; % Velocity of the fluid relative to the section in the rotor frame
                    % ASSUMPION - Coaxial turbines, therefore z-axes are
                    % alighed. Therefore we can modify the axial flow with
                    % the scalar modifier.
                    
                    % If we are using BEMT, compute the axial flow factor
                    % Get relative velocity at this section in the blade frame
                    U_rel_bx = hobj.bx_C_P(:,:,i)*U_rel_P;
                    % The blade z is the axial component (more coaxial
                    % assumption baked in here) and the blade x is the
                    % omega*r component. We are only doing axial induction.
                    if hobj.BEMT(1)
                        BEMTitr = 0;
                        aprev = 1;
                        a = 0;
                        diff_aaprev = 1;
                        while diff_aaprev > 1.0e-3
                            BEMTitr = BEMTitr + 1;
                            if BEMTitr > 100
                                wstr = ['BEMT reached 100 iterations. Final a: ' num2str(a,3)];
                                warning(wstr);
                                break;
                            end
                            try
                            phi = atan2((1-a)*U_rel_bx(3),U_rel_bx(1));
                            if phi < 0
                                error('phi is negative. Stop for a second.');
                            end
                            catch
                                error('what?');
                            end
                            r = abs(hobj.blades(i).sectLocs(2));
                            F = 1;
                            if hobj.BEMT(2) % Use Prandtl tip loss factor
                                f = hobj.numblades/2*(hobj.blades(i).length - r)/(r*sin(phi));
                                F = 2/pi*acos(exp(-f));
                            end                                                    
                            bx_C_a = hobj.blades(i).b_C_a(:,:,j);
                            U_rel_a = transpose(bx_C_a)*hobj.bx_C_P(:,:,i)*U_rel_P;
                            [cl,cd,~] = hobj.blades(i).sections(j).computeForceCoeffs(U_rel_a,fluid);
                            cn = cl*cos(phi)+cd*sin(phi);
                            sigma = hobj.numblades*hobj.blades(i).sections(j).chord/(2*pi*r);
                            a = 1/((4*F*sin(phi)^2)/(sigma*cn)+1); 
                            ac = 0.2;
                            a = min(a,1);
%                             if a > 0.4 && a < 1 % Using the equation described Buhl2004new eqn. 18
%                                 buhlf = @(a,F,sigma,phi,cn) (1-a)^2*sigma*cn/sin(phi)^2 - (8/9+(4*F-40/9)*a+(50/9-4*F)*a^2);
%                                 f = @(a) buhlf(a,F,sigma,phi,cn);
%                                 [a,~,flg,~] = fzero(f,0.6,opts);
%                                 if flg ~= 1
%                                     % no solution, if it was close just use
%                                     % the last known value
%                                     if diff_aaprev < 0.1
%                                         a = aprev;
%                                     else
%                                         error('BEMT failed to converge.');
%                                     end
%                                 end
%                             end
                            % Using the Wilson-Walker equation described in
                            % Hansen2007aerodynamics
                            if abs(a) > ac
                                K = 4*F*sin(phi)^2/(sigma*cn);
                                K = max(K,0);
                                Kb = sqrt((K*(1-2*ac) + 2)^2 + 4*(K*ac^2-1));
                                a = 0.5*(2 + K*(1-2*ac)-Kb);
                            end
                            diff_aaprev  = abs(a-aprev);
                            aprev = a;
                        end
                        temp = 0.5*fluid.density*norm([U_rel_a(3),U_rel_a(1)],2)^2*hobj.blades(i).sections(j).chord*hobj.blades(i).sections(j).width;
                        Lmag = cl*temp;
                        Dmag = cd*temp; 
                        temp = sqrt(U_rel_a(1)^2+U_rel_a(3)^2); % Magnitude of the relative velocity in the section frame
                        salpha = 0;
                        calpha = 0;
                        if temp > 1.0e-12
                            salpha = U_rel_a(3)/temp;
                            calpha = U_rel_a(1)/temp;
                        end
                        % loads in the section frame
                        drag = [Dmag*calpha;0;Dmag*salpha];
                        lift = [-Lmag*salpha;0;Lmag*calpha];
                        %hobj.computeAxialFlowFactor(U_rel_P
                        hobj.axflowfac = 1-a;
                        U_rel_P(3) = hobj.axflowfac*U_rel_P(3);
                        U_relSections(:,j,i) = U_rel_P; % Velocity vectors for blade i section j in the rotor frame.
                    else % not using BEMT
                        % Another baked-in assumption of coaxial
                        U_rel_P(3) = hobj.axflowfac*U_rel_P(3);
                        U_relSections(:,j,i) = U_rel_P; % Velocity vectors for blade i section j in the rotor frame.

                        % Rotate the relative velocity into the blade section
                        % frame. First, compute bx_C_a
                        ang = hobj.blades(i).sectOrnts(2,j); 
                        bx_C_a = hobj.blades(i).b_C_a(:,:,j);
                        % U in section frame =
                        % section_C_blade*blade_C_rotor*U_rotor
                        U_rel_a = transpose(bx_C_a)*hobj.bx_C_P(:,:,i)*U_rel_P;

                        % Get loads from each section of the blade. INFO: The
                        % computeLoads method only uses the x and z components
                        % of the relative velocity vector.
                        [lift,drag,~] = hobj.blades(i).sections(j).computeLoads(U_rel_a,fluid);
                        %[lift,drag,~] = hobj.blades(i).sections(j).computeLoadsFast(U_rel_bs,fluid);
                        % No moments right now so I'm not dealing with them.
                        % todo(rodney) add section moment functionality.                    
                    end                    
                    % Transform these loads into the rotor frame
                    drag = hobj.P_C_bx(:,:,i)*bx_C_a*drag;
                    lift = hobj.P_C_bx(:,:,i)*bx_C_a*lift;
                    
                    LiftSections(:,j,i) = lift; % todo This will crap out if the number of sections is differnt in each blade. Make cells to genericize
                    DragSections(:,j,i) = drag;
                    % TorqueSections(:,j,i) % Not currently functional
                    ForceSections(:,j,i) = lift + drag;
                    totforce = totforce + ForceSections(:,j,i);
                    
                    % Compute torque from loads
                    tauprev = cross(rap_P,ForceSections(:,j,i));
                    torque = torque + tauprev;
                end % end bladesection loop               
            end % end blade loop
            % Compute generator torque
            %genTorque = hobj.vehicle.generator.getTorque(hobj.angvel);
            % Update class properties
            hobj.sectLift = LiftSections;
            hobj.sectDrag = DragSections;
            % hobj.sectMomt = TorqueSections;
            hobj.force = totforce;
            hobj.torqueCM = torque;
        end % end computeHydroLoads
        
        function hfig = visualizeSectionLoads(hobj,hide,scale)
            % This method creates a plot of the current loads on the rotor
            % at each section.
            % ARGS: hide = bool hide the plot?
            % OUT: hfig = figure handle
            if nargin < 2
                hide = false;                
            end
            if nargin < 3
                scale = 0.5;
            end
                temp = size(hobj.sectPos);
                for i=1:1:temp(2)
                    for j=1:1:temp(3)
                        rap1_P(:,i,j) = hobj.sectPos(:,i,j);
                        L_P(:,i,j) = hobj.sectLift(:,i,j);
                        D_P(:,i,j) = hobj.sectDrag(:,i,j);
                        R_P(:,i,j) = L_P(:,i,j) + D_P(:,i,j);
                    end
                end
                % Show the forces and/or velocity vectors as quivers applied at the section
                % locations in the rotor frame
                v = 'on';
                if hide
                    v = 'off';
                end
                hfig = figure('visible',v);
                quiver3(rap1_P(1,:,:),rap1_P(2,:,:),rap1_P(3,:,:),L_P(1,:,:),L_P(2,:,:),L_P(3,:,:),scale,'-','color',[0 0 1],'LineWidth',2);
                hold on
                quiver3(rap1_P(1,:,:),rap1_P(2,:,:),rap1_P(3,:,:),D_P(1,:,:),D_P(2,:,:),D_P(3,:,:),scale,'-','color',[1 0 0],'LineWidth',2);
                quiver3(rap1_P(1,:,:),rap1_P(2,:,:),rap1_P(3,:,:),R_P(1,:,:),R_P(2,:,:),R_P(3,:,:),scale,'-','color',[0 0 0],'LineWidth',2);
                axis equal
                xlabel('x'); ylabel('y'); zlabel('z');
                title(['Force Vectors | Simple Rotation Rate = ' num2str(hobj.angvel(3)*30/pi,3) ' RPM']);
                legend({'Lift','Drag','Total'},'Location','northeast','color','none');
                view(-140,17)
                hold off
                ax = gca;
                ax.FontSize = 10;
                set(ax,'color','none');
        end % end visualizeSectionLoads
        
        function connectVehicle(hobj,v)
            % Arguments:
                % v = the vehicle object to connect to
            hobj.vehicle = v;
        end % end ConnectVehicle
        function addTorque(hobj,tq)
            hobj.torqueCM(3) = hobj.torqueCM(3) + tq;
        end
        function setAxialFlowFactor(hobj,ff)
            hobj.axflowfac = ff;
        end
        
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
        function setBEMT(hobj,ba)
            if length(ba) < 2
                warning('BEMT model has two choices, assuming you don"t want to use tip loss correction.');
                hobj.BEMT(1) = boolean(ba);
                hobj.BEMT(2) = false;
            else
                hobj.BEMT = boolean(ba);
            end
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
            % Inertial velocity of the rotor frame expressed in the vehicle frame
            v = cross(hobj.vehicle.angvel,hobj.vehicle.rotorLocs(:,hobj.ID))+hobj.vehicle.velocity;
        end
        
    end % end methods
end % end classdef