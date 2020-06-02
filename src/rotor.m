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
        tnflowfac  % Tangential flow factor (1+induction factor)
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
            U_relP_P = Ufluid_P - OVpo_P; % relative velocity of any blade root in the P frame.
            
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
                % Get the rotation matrices for all sections in this blade;
                bx_C_a = hobj.blades(i).b_C_a;
                % Get section aero loads
                for j=1:1:hobj.blades(i).numsects
                    % Get section positions
                    rap_P = hobj.sectPos(:,j,i);
                    % Compute relative velocity in the rotor frame at the
                    % section location.
                        % Need {O_W_A}_P
                    O_W_P_P = hobj.P_C_A*hobj.vehicle.angvel + hobj.angvel;
                    V_ap_P = cross(O_W_P_P,rap_P); % O velocity of a w.r.t. p in frame P
                    U_relSec_P = Ufluid_P - OVpo_P - V_ap_P; % Velocity of the fluid relative to the section in the rotor frame
                    % ASSUMPION - Coaxial turbines, therefore z-axes are
                    % alighed. Therefore we can modify the axial flow with
                    % the scalar modifier.
                    
                    % If we are using an axial flow factor then we are
                    % modifying the axial flow a priori.
                    U_relSec_P(3) = hobj.axflowfac*U_relSec_P(3);
                    
                    % Then, whether or not we are using BEMT, we need the
                    % relative velocity in the blade frame. If we are not
                    % using BEMT we need it in the section frame as well.
                    U_relSec_bx = hobj.bx_C_P(:,:,i)*U_relSec_P;
                    % Modify it by the a prior tangential flow factor
                    U_relSec_bx(1) = hobj.tnflowfac*U_relSec_bx(1);
                    U_relSec_a = transpose(bx_C_a(:,:,j))*hobj.bx_C_P(:,:,i)*U_relSec_P;
                    % The blade z is the axial component (more coaxial
                    % assumption baked in here) and the blade x is the
                    % omega*r component. We are only doing axial induction.
                    if hobj.BEMT(1)
                        bld = i;
                        sec = j;
                        r = abs(hobj.blades(bld).sectLocs(2,sec));
                        
                        % Need the local TSR w.r.t. the axial component.
                        % Since the vehicle is also rotating, get from the z
                        % component of O_W_P_P.
                        localAxialTSR = abs(O_W_P_P(3))*r/(hobj.axflowfac*U_relP_P(3));                        
                        nearZeroPlusSide = 1.0e-6; % Avoiding exactly zero - see Ning2014Simple
                        % Find the bounds for the search.
                        % First, check 0 < phi < pi/2                         
                        resids = hobj.getBEMTresids(pi/2,bld,sec,localAxialTSR);
                        if resids > 0 % solution is in the first (most likely) half of the momentum/empirical windmill region
                            fun = @(phi) hobj.getBEMTresids(phi,bld,sec,localAxialTSR);
                            phistar = fzero(fun,[nearZeroPlusSide pi/2]);
                        else % if not, search the propeller brake region
                            phi_0 = nearZeroPlusSide;
                            resids1 = hobj.getBEMTresidsPropBrake(-pi/4,bld,sec,localAxialTSR);
                            resids2 = hobj.getBEMTresidsPropBrake(nearZeroPlusSide,bld,sec,localAxialTSR);
                            % compute fpb for -pi/4 and nearZeroPlusSide
                            if resids1 < 0 && resids2 > 0 % solution is in the propeller brake region
                                fun = @(phi) hobj.getBEMTresidsPropBrake(phi,bld,sec,localAxialTSR);
                                phistar = fzero(fun,[-pi/4 -nearZeroPlusSide]);
                            else % solution is in the remaining region
                                fun = @(phi) hobj.getBEMTresids(phi,bld,sec,localAxialTSR);
                                phistar = fzero(fun,[pi/2 pi]);
                            end
                        end
                        % now we do one more fuction evaluation with
                        % phistar to get the lift, drag, and whatever else
                        % we need.
                        [a,ap,~,~,cl,cd] = hobj.getBEMTparams(phistar,bld,sec);
                        % The lift and drag coefficients are in the section
                        % frame. They get rotated outside of the BEMT if
                        % block (a few lines down).
                        vrelx_bx = U_relSec_bx(1)*(1+ap);
                        vrelz_bx = U_relSec_bx(3)*(1-a);
%                         vrelx_bx = U_rel_bx(1)*(1+ap);
%                         vrelz_bx = U_rel_bx(3)*(1-a);
                        vrelmagsqrd = vrelx_bx^2 + vrelz_bx^2;
%                         aoa = atan2d(vrelz,vrelx);
                        aoa = phistar - hobj.blades(i).sectOrnts(2,j);
                        temp = 0.5*fluid.density*vrelmagsqrd*hobj.blades(i).sections(j).chord*hobj.blades(i).sections(j).width;
                        Lmag = cl*temp;
                        Dmag = cd*temp;
                        % Put the lift and drag into the section frame (not
                        % the BEMT frame)!
                        drag = [Dmag*cos(aoa);0;Dmag*sin(aoa)];
                        lift = [-Lmag*sin(aoa);0;Lmag*cos(aoa)];
                    else % not using BEMT - but we can still use tip-loss
                        % Another baked-in assumption of coaxial
                        % Get loads from each section of the blade. INFO: The
                        % computeLoads method only uses the x and z components
                        % of the relative velocity vector.
                        F = 1;
                        if hobj.BEMT(2)
                        phi = atan2(U_relSec_bx(3),U_relSec_bx(1));
                        F = getTipLossFactor(hobj,phi,i,j);
                        end
                        [lift,drag,~] = hobj.blades(i).sections(j).computeLoads(U_relSec_a*F,fluid);
                        % No moments right now so I'm not dealing with them.
                        % todo(rodney) add section moment functionality.                    
                    end
                    
                    % Save the user modified flow for visualization
                    % Not showing the BEMT modification to the relative
                    % flow, only the user specified modification.
                    % todo we may also want to see the BEMT results at each
                    % section, both the modified velocity and the induction
                    % factors.
                    U_relSections(:,j,i) = U_relSec_P; % Velocity vectors for blade i section j in the rotor frame.
                    
                    % Transform the loads into the rotor frame
                    drag = hobj.P_C_bx(:,:,i)*bx_C_a(:,:,j)*drag;
                    lift = hobj.P_C_bx(:,:,i)*bx_C_a(:,:,j)*lift;
                    
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
        
        function hfig = showmetorquecurve(hobj,speed,fld)
            % Makes a figure showing the torque curve for a rotor in the
            % current orientation in the passed fluid
            % speed = array of speed to plot
            % fld = fluid object
            
            for i=1:1:length(speed)    
                hobj.angvel = [0;0;speed(i)];
                hobj.computeHydroLoads(fld);
                torque(:,i) = hobj.torqueCM;
            end
            hfig = figure('Color','w');
            temp1 = hobj.blades(1).length/norm(fld.velocity);
            temp2 = 0.5*fld.density*pi*hobj.blades(1).length^2*norm(fld.velocity);
            plot(speed*temp1,torque(3,:)/temp2,'r');
            xlabel('TSR'); ylabel('C_T_R_Q')
        end
        
        function hfig = showmepowercurve(hobj,speed,fld)
            % Makes a figure showing the torque curve for a rotor in the
            % current orientation in the passed fluid
            % speed = array of speed to plot
            % fld = fluid object
            
            for i=1:1:length(speed)    
                hobj.angvel = [0;0;speed(i)];
                hobj.computeHydroLoads(fld);
                torque(:,i) = hobj.torqueCM;
            end
            hfig = figure('Color','w');
            temp1 = hobj.blades(1).length/norm(fld.velocity);
            temp2 = 0.5*fld.density*pi*hobj.blades(1).length^2*norm(fld.velocity);
            plot(speed*temp1,torque(3,:).*speed/temp2,'r');
            xlabel('TSR'); ylabel('C_P')
        end
        
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
        
        function setTangentialFlowFactor(hobj,ff)
            hobj.tnflowfac = ff;
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
        
        function setBladePitch(hobj,p)
            for i=1:1:numel(hobj.blades)
                hobj.blades(i).adjustPitch(p);
            end
        end
        
        function F = getTipLossFactor(hobj,phi,bld,sec)
            % Computes the Prandtl tip loss factor using Glauert's
            % approximation.
            % phi = angle that the relative velocity makes with the rotor plane
            % bld = blade number in the rotor
            
            % Because of the way that we define sections there will never
            % be a 0/0 here even if there is a zero in the denomenator
            % (i.e. inf).
            r = abs(hobj.blades(bld).sectLocs(2,sec));
            f = hobj.numblades/2*(hobj.blades(bld).length - r)/(r*sin(phi));
            F = 2/pi*acos(exp(-f));
        end
        
        function [a,ap,kappa,kappaprime,cl,cd] = getBEMTparams(hobj,phi,bld,sec)
            r = abs(hobj.blades(bld).sectLocs(2,sec));
            localSolidity = hobj.numblades*hobj.blades(bld).sections(sec).chord/(2*pi*r);
            F = 1;
            if hobj.BEMT(2) % Use Prandtl tip loss factor
                F = hobj.getTipLossFactor(phi,bld,sec);
            end
            aoa = phi - hobj.blades(bld).sectOrnts(2,sec);
            [cl,cd,~] = hobj.blades(bld).sections(sec).computeForceCoeffs(aoa,fluid);
            cn = cl*cos(phi) + cd*sin(phi);
            ct = cl*sin(phi) - cd*cos(phi);
            kappa = localSolidity*cn/(4*F*sin(phi)^2);
            kappaprime = localSolidity*ct/(4*F*sin(phi)*cos(phi));
            if kappa < 2/3
                a = kappa/(1+kappa);
            else
                % Glauert/Buhl correction for empirical region
                gam1 = 2*F*kappa-(10/9-F);
                gam2 = 2*F*kappa-F*(4/3-F);
                gam3 = 2*F*kappa-(25/9-2*F);
                a = (gam1 - sqrt(gam2))/gam3;
            end
            ap = kappaprime/(1-kappaprime);
        end
        
        function resids = getBEMTresids(hobj,phi,bld,sec,locTSR)
            [a,~,~,kapp] = getBEMTparams(hobj,phi,bld,sec);
            %resids = sin(phi)/(1-a) - cos(phi)/(locTSR*(1+ap));
            locTSR = max(1.0e-12,locTSR); % The rotor must be moving a little to avoid inf for the fzero solution
            cosphi = cos(phi);
            if abs(cosphi) < 1.0e-12
                cosphi = 0;
            end
            resids = sin(phi)/(1-a) - cosphi*(1-kapp)/locTSR; % this way is better and mathematically equivalent
        end
        
        function resids = getBEMTresidsPropBrake(hobj,phi,bld,sec,locTSR)
            [~,~,kap,kapp] = getBEMTparams(hobj,phi,bld,sec);
            %resids = sin(phi)/(1-a) - cos(phi)/(locTSR*(1+ap));
            locTSR = max(1.0e-12,locTSR); % The rotor must be moving a little to avoid inf for the fzero solution
            resids = sin(phi)/(1-kap) - cos(phi)*(1-kapp)/locTSR; % this way is better and mathematically equivalent
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