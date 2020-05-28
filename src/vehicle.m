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
        generator   % generator
        % Points expressed in the vehicle (A) frame
        rotorLocs   % 3xn vector of rotor locations expressed in the vehicle frame where n is the number of rotors (g1,g2,g3) and (h1,h2,h3)
        centermass  % 3x1 vector location of center of mass of the vehicle in the vehicle frame (s1,s2,s3)
        tetherpoint % 3xm vector of location of the tether attachment point(s) in the vehicle frame (t1, t2, t3)
        buoypoint   % 3x1 vector of location of the center of buoyancy (b1, b2, b3)
        mass        % scalar total mass of the vehicle
        type        % uint identifier | see typeName get method for list of types
        Mstar       % mass inertia matrix of the multibody vehicle
        MstarInv    % inverse of Mstar
        Mam         % added mass matrix
        Mtot        % total inertia matrix, equal to Mstar + Mam
        MtotInv     % Inverse of the Mtot matrix
        relDensity  % The total density of the vehicle (all components) relative to water
        rotorFric   % Viscosity constant between vehicle body and rotor(s)
    end % End private properties    
    properties (Dependent)
        A_C_O       % 3x3 tranformation matrix from inertial frame to vehicle frame
        typeName    % Name of the type
    end % End dependent properties    
    properties
        % Vehicle state variables
        position    % 3x1 Position vector of mass center in the inertial frame IN METERS (x1,x2,x3)
        orientation % 3x1 orientation of the vehicle frame w.r.t. the inertial frame following 2-1-3 (theta, gamma, beta) IN RADIANS
        velocity    % 3x1 Velocity vector of mass center in the inertial frame IN METERS/SEC (dx1,dx2,dx3)
        angvel      % 3x1 Angular velocity vector in the VEHICLE frame (omega_x,omega_y,omega_z)IN RADIANS/SEC
        % Vehicle loads
        force       % 3x1 Net aerodynamic force in the vehicle body frame
        torque      % 3x1 Net torque about the centermass of the vehicle system (which is the origin of the vehicle frame)
    end % End public properties
    
    methods
        %% Constructor
        function hobj = vehicle(bod,rot,cm,tp,bp)
            if nargin == 2
               % assume simple single rotor with cm at body [0;0;0];
               hobj.body = bod;
               hobj.rotors = rot;
               hobj.mass = rot.mass;
            elseif nargin > 0
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
            end
            hobj.Mstar = [];
            hobj.MstarInv = [];
            hobj.Mam = [];
            hobj.Mtot = [];
            hobj.MtotInv = [];
        end % End constructor
        
        %% Initialization method
        function init(hobj,bod,rot,rotLocs,cm,tp,bp)
            if nargin < 3
                % This is a body-only vehicle (no rotor)
                error('Body-only vehicle not currently supported');
            else
                hobj.body = bod;
                hobj.rotors = rot;                
                hobj.tetherpoint = tp;
                hobj.buoypoint = bp;
                hobj.rotorLocs = rotLocs;
                m = 0;
                switch numel(rot)
                    case 0
                        hobj.type = 0;
                    case 1
                        hobj.type = 1;
                    case 2
                        if isempty(hobj.type)
                        hobj.type = 2;
                        warning('VEHICLE:construction','Assuming vehicle type is coaxial. Use setType to change after init if this is incorrect.');
                        end
                    otherwise
                        error('Only 0, 1, or 2 rotor systems currently supported');
                end
                if numel(rot) >0
                    for i=1:1:numel(rot)
                        m = m + rot(i).mass;
                    end
                end
                hobj.mass = m + bod.mass;
                % if you set a cm in the input file then that's what get's
                % used. Note that if you add a generator later at a point
                % other than the cm the cm will move.
                if isempty(cm) 
                    rcsa_A = [0;0;0];
                    for i=1:1:numel(rot)
                        rcsa_A = rcsa_A + rot(i).mass/hobj.mass*rotLocs(:,i);
                    end
                    % Compute system center of mass
                    % todo check if there is a generator and, if so, adjust
                    % center mass accordingly. For now we assume that the
                    % generator is added later.
                    rcsa_A = rcsa_A + bod.mass/hobj.mass*bod.centermass;
                    hobj.centermass = rcsa_A;
                else
                    hobj.centermass = cm;
                end
            end
        end % End init
        %% computeHydroLoads
        function urel = computeHydroLoads(hobj,f)
            % Computes the net hydrodynamic force and torque vectors
            frc = 0;
            trq = 0;
            % Get aerodynamic loads on rotors, cross and sum
            % Need to preallocate the arrays based on whichever rotor has
            % more blades and sections. For now assume all blades have the
            % same max number of sections as the blades in first rotor.
            % todo enable each blade to have a different number of sections
            numrotors = numel(hobj.rotors);
            maxnumblades = max(hobj.rotors.numblades);
            numsects = hobj.rotors(1).blades(1).numsects; % this approach assumes all same number of sections
            sectionforces = zeros(3,numsects,maxnumblades);
            sectionpositions = sectionforces;
            %urel = zeros(3,numsects,maxnumblades,numrotors);
            for i=1:1:numrotors
                % todo need to fix urel to get showme to work again.
                %urel(:,:,1:hobj.rotors(i).numblades,i) = hobj.rotors(i).computeHydroLoads(f); % Don't actually need this. 
                hobj.rotors(i).computeHydroLoads(f);
                % 3xjxn array of lift and drag at each section expressed in
                % the rotor frame (j=maxnum sections, n=maxnum blades)
                % We want to overwrite only the elements that exist in this
                % rotor                
                sectionforces(:,:,1:hobj.rotors(i).numblades) = hobj.rotors(i).sectLift + hobj.rotors(i).sectDrag; % this approach assumes all same numner of sections
                % 3xjxn array of position of each section expressed in the rotor frame
                sectionpositions(:,:,1:hobj.rotors(i).numblades) = hobj.rotors(i).sectPos;
                %[~,numsecs,numblades] = size(sectionpositions);
                % Rotate into the vehicle frame
                for j = 1:1:numsects % this approach assumes all same numner of sections
                    for n = 1:1:hobj.rotors(i).numblades
                        sectionpositions(:,j,n) = transpose(hobj.rotors(i).P_C_A)*sectionpositions(:,j,n);
                        sectionforces(:,j,n) = transpose(hobj.rotors(i).P_C_A)*sectionforces(:,j,n);
                    end
                end
                
                % Aerodynamic moments not currently functional. When we go to implement them
                % I think we can just sum the section moments.
                % sectiontorques(:,:,:,i) = hobj.rotors(i).sectMomt;
                r_ayxA_A = hobj.rotorLocs(i) + sectionpositions;
                trq = trq + cross(r_ayxA_A,sectionforces);
                frc = frc + sectionforces;
                
                % Add a viscous force to the rotors to simulate bearing
                % friction. This is basically a low-fidelity journal
                % bearnign model, but this load is so small compared to
                % everything else that I'm sacrificing a little fidelity
                % for parsimony.
                rtrspeed = hobj.rotors(i).angvel(3);
                viscConst = hobj.rotorFric;
                % The way the model is formulated, this torque needs to be
                % added to the rotors.
                hobj.rotors(i).addTorque(-viscConst*0.5*f.density*pi*rtrspeed*abs(rtrspeed)*hobj.body.radius^4*hobj.body.length);
                % In small lab-scale systems you can end up with a numerical
                % torque on the body even when the rotors are not spinning.
                % This doesn't correct iteself because the body-rotor loads
                % are internal and the external viscous torsion is small
                % compared to the numerical error.
            end % end rotor loop
            
            % Get Hydrodynamic loads on body, cross and sum            
            % Rotate fluid velocity into the vehicle frame
            vF_A = hobj.A_C_O*f.velocity;
            % Now compute the relative velocity
            vRel_A = vF_A - hobj.velocity;
            % the normal component is made up of the 1 and 2 components,
            % and the axial component is the 3 component.
            q = f.density*hobj.body.radius;
            vRelnorm_A = norm(vRel_A(1:2))*vRel_A(1:2);
            vRelax_A = vRel_A(3)*vRel_A(3);
            % Compute normal coefficient - see huston1981representation
            %Re = f.getRe(norm(vRel_A(1:2)),2*hobj.body.radius);
            %hobj.body.computeNormalCoeff(Re);
            % todo enable computation. For now these are constant
            Fn = hobj.body.normCoeff*q*vRelnorm_A; %1.2
            Fa = hobj.body.axCoeff*q*vRelax_A;   %0.1
            %Fbod = [Fn;Fa];
            Fbod = [0;0;0];
            
            % Compute the moment coefficient
            cmc = hobj.getCylinderMomentCoefficient(f,hobj.body.torsSelect);
            % Add viscous torsion
            bodyTors = -hobj.body.torsionMod*0.5*f.density*pi*hobj.angvel(3)*abs(hobj.angvel(3))*hobj.body.radius^4*hobj.body.length*cmc;
                        
            % Sum along the 2dim gives 3x1xnumBlades of total blade loads
            % then sum along the 3dim to get 3x1 vector of total loads
            hobj.force = sum(sum(frc,2),3) + Fbod;
            hobj.torque = sum(sum(trq,2),3) + [0;0;bodyTors];
        end % end computeHydroLoads
        %% addTetherLoads
        function addTetherLoads(hobj,frc)
            % Adds the tether loads to the total vehicle force and torque
            % INPUTS:
                % frc = 3xm force in the vehicle frame
            [~,numloads] = size(frc);
            for i=1:1:numloads
                hobj.force = hobj.force + frc(:,i);
                hobj.torque = hobj.torque + cross(hobj.tetherpoint(:,i),frc(:,i));
            end
        end % end addTetherLoads
        
        %% addRotor
        function addRotor(hobj,loc)
            % Concatenate as attached
            hobj.rotorLocs = [hobj.rotorLocs,loc];
            % todo adjust the center of mass of the vehicle
        end % end addRotor
        %% visualizeSectionLoads
        function frame = visualizeSectionLoads(hobj,hide,scale,totalOnly)
            % This method creates a plot of the current loads on the
            % ARGS: 
                % hide = bool hide the plot?
                % scale = to scale the vectors?
            % OUT: hfig = figure handle
            if nargin < 2
                hide = false;                
            end
            if nargin < 3
                scale = 0.5;
            end
            if nargin < 4
                totalOnly = false;
            end
            as = 'on';
            for k=1:1:numel(hobj.rotors) % rotors loop
                temp = size(hobj.rotors(k).sectPos); % 3 x nSects x nBlades
                A_C_P = transpose(hobj.rotors(k).P_C_A);
                for i=1:1:temp(2)
                    for j=1:1:temp(3)
                        % 3 x nSects x nBlades x nRotors
                        raA_A(:,i,j,k) = A_C_P*hobj.rotors(k).sectPos(:,i,j) + hobj.rotorLocs(:,k);                        
                        L_A(:,i,j,k) = A_C_P*hobj.rotors(k).sectLift(:,i,j);
                        D_A(:,i,j,k) = A_C_P*hobj.rotors(k).sectDrag(:,i,j);
                        R_A(:,i,j,k) = A_C_P*L_A(:,i,j,k) + D_A(:,i,j,k);
                    end
                end
                % Show the forces and/or velocity vectors as quivers applied at the section
                % locations in the rotor frame
            end % End rotors loop            
            v = 'on';
            if hide
                v = 'off';
            end
            
            hfig = figure('visible',v,'Position',[100 100 900 900]);
            
            quiver3(raA_A(1,:,:,1),raA_A(2,:,:,1),raA_A(3,:,:,1),R_A(1,:,:,1),R_A(2,:,:,1),R_A(3,:,:,1),scale,'-','color',[0 0 0],'LineWidth',2,'AutoScale',as);
            hold on
            if ~totalOnly
                quiver3(raA_A(1,:,:,1),raA_A(2,:,:,1),raA_A(3,:,:,1),D_A(1,:,:,1),D_A(2,:,:,1),D_A(3,:,:,1),scale,'-','color',[1 0 0],'LineWidth',2,'AutoScale',as);
                quiver3(raA_A(1,:,:,1),raA_A(2,:,:,1),raA_A(3,:,:,1),L_A(1,:,:,1),L_A(2,:,:,1),L_A(3,:,:,1),scale,'-','color',[0 0 1],'LineWidth',2,'AutoScale',as);
            end
            text((hobj.rotorLocs(1,1)+R_A(1,end,1,1))*0.5,(hobj.rotorLocs(2,1)+R_A(2,end,1,1))*0.5,(hobj.rotorLocs(3,1)+R_A(3,end,1,1))*0.5,['Rotor Rotation Rate = ' num2str(hobj.rotors(1).angvel(3)*30/pi,3) ' RPM'],'color','m','FontSize',12,'FontWeight','bold');
            text((hobj.rotorLocs(1,1)+R_A(1,end,1,1))*0.6,(hobj.rotorLocs(2,1)+R_A(2,end,1,1))*0.6,(hobj.rotorLocs(3,1)+R_A(3,end,1,1))*0.6,['Torque = ' num2str(hobj.rotors(1).torqueCM(3),3) ' N'],'color','m','FontSize',12,'FontWeight','bold');
            text((hobj.rotorLocs(1,1)+R_A(1,end,1,1))*0.7,(hobj.rotorLocs(2,1)+R_A(2,end,1,1))*0.7,(hobj.rotorLocs(3,1)+R_A(3,end,1,1))*0.7,['Rotor Number = ' num2str(hobj.rotors(1).ID,1)],'color','m','FontSize',12,'FontWeight','bold');
            for i=2:1:numel(hobj.rotors)
                quiver3(raA_A(1,:,:,i),raA_A(2,:,:,i),raA_A(3,:,:,i),R_A(1,:,:,i),R_A(2,:,:,i),R_A(3,:,:,i),scale,'-','color',[0 0 0],'LineWidth',2,'AutoScale',as);
                if ~totalOnly
                    quiver3(raA_A(1,:,:,i),raA_A(2,:,:,i),raA_A(3,:,:,i),L_A(1,:,:,i),L_A(2,:,:,i),L_A(3,:,:,i),scale,'-','color',[0 0 1],'LineWidth',2,'AutoScale',as);
                    quiver3(raA_A(1,:,:,i),raA_A(2,:,:,i),raA_A(3,:,:,i),D_A(1,:,:,i),D_A(2,:,:,i),D_A(3,:,:,i),scale,'-','color',[1 0 0],'LineWidth',2,'AutoScale',as);
                end
%                 text((hobj.rotorLocs(1,i)+R_A(1,:,:,i))*0.5,(hobj.rotorLocs(2,i)+R_A(2,:,:,i))*0.5,(hobj.rotorLocs(3,i)+R_A(3,:,:,i))*0.5,['Rotor Rotation Rate = ' num2str(hobj.rotors(i).angvel(3)*30/pi,3) ' RPM']);
%                 text((hobj.rotorLocs(1,i)+R_A(1,:,:,i))*0.6,(hobj.rotorLocs(2,i)+R_A(2,:,:,i))*0.6,(hobj.rotorLocs(3,i)+R_A(3,:,:,i))*0.6,['Torque = ' num2str(hobj.rotors(i).torqueCM(3),3) ' N']);
%                 text((hobj.rotorLocs(1,i)+R_A(1,:,:,i))*0.7,(hobj.rotorLocs(2,i)+R_A(2,:,:,i))*0.7,(hobj.rotorLocs(3,i)+R_A(3,:,:,i))*0.7,['Rotor Number = ' num2str(hobj.rotors(i).ID,1)]);
            end
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z');
            title({'Force Vectors',['Vehicle Angular Rate = [' num2str(hobj.angvel(1)*30/pi,3) ' ' num2str(hobj.angvel(2)*30/pi,3) ' ' num2str(hobj.angvel(3)*30/pi,3) '] RPM']});
            legndstr = "Total Hydrodynamic Load";
            if ~totalOnly
                legndstr = ["Total","Drag","Lift"];
            end
            legend(legndstr,'Location','northeast','color','none');
            view(-140,17)
            hold off
            ax = gca;
            ax.FontSize = 10;
            set(ax,'color','none');
            frame = getframe(hfig);
            if hide
                close(hfig);
            end
        end % end visualizeSectionLoads
        %% visualizeRelativeVelocities
        function hfig = visualizeRelativeVelocities(hobj,fluid,hide,scale)
            % This method creates a plot of the current relative
            % velocities at each section in the vehicle frame.
            % ARGS: 
                % fluid = a fluid object?
                % hide = bool hide the plot?
                % scale = to scale the vectors?
            % OUT: hfig = figure handle
            if nargin < 3
                hide = false;                
            end
            if nargin < 4
                scale = 0.5;
            end
            
            for k=1:1:numel(hobj.rotors) % rotors loop
                % Velocity of the fluid in the rotor frame
                A_C_P = transpose(hobj.rotors(k).P_C_A);
                Ufluid_P = hobj.rotors(k).P_C_A*hobj.A_C_O*fluid.velocity;
                % Velocity of point p in the rotor frame
                OVpo_P = hobj.rotors(k).P_C_A*hobj.rotors(k).velocity; % OVpo_P is O derivative of r_<to><from>_<ExprsdFrame>
                temp = size(hobj.rotors(k).sectPos); % 3 x nSects x nBlades
                for i=1:1:temp(2)
                    for j=1:1:temp(3)                        
                        rap_P = hobj.rotors(k).sectPos(:,i,j);
                        % Compute relative velocity in the rotor frame at the
                        % section location.
                        temp2 = hobj.rotors(k).P_C_A*hobj.angvel + hobj.rotors(k).angvel;
                        OV_ap_P = cross(temp2,rap_P);
                        OV_ao_P = OV_ap_P + OVpo_P;
                        U_rel_P = Ufluid_P - OV_ao_P;
                        % 3 x nSects x nBlades x nRotors
                        U_relSections(:,i,j,k) = U_rel_P;
                        raA_A(:,i,j,k) = A_C_P*rap_P + hobj.rotorLocs(:,k);
                    end
                end
                % Show the forces and/or velocity vectors as quivers applied at the section
                % locations in the rotor frame
                lgnd(k) = join(["U_{rel}",num2str(k)]);
            end % End rotors loop
            
            
            v = 'on';
            if hide
                v = 'off';
            end
            hfig = figure('visible',v,'Position',[600 100 900 900]);
            quiver3(raA_A(1,:,:,1),raA_A(2,:,:,1),raA_A(3,:,:,1),U_relSections(1,:,:,1),U_relSections(2,:,:,1),U_relSections(3,:,:,1),scale,'-','color','r','LineWidth',2);
            hold on
            text(hobj.rotorLocs(1,1),hobj.rotorLocs(2,1),hobj.rotorLocs(3,1)*0.9,['Rotor Rotation Rate = ' num2str(hobj.rotors(1).angvel(3)*30/pi,3) ' RPM']);
            for i=2:1:numel(hobj.rotors)
                clrs = '-gcym';
                quiver3(raA_A(1,:,:,i),raA_A(2,:,:,i),raA_A(3,:,:,i),U_relSections(1,:,:,i),U_relSections(2,:,:,i),U_relSections(3,:,:,i),scale,'-','color',clrs(k),'LineWidth',2);
                text(hobj.rotorLocs(1,i),hobj.rotorLocs(2,i),hobj.rotorLocs(3,i)*1.1,['Rotor Rotation Rate = ' num2str(hobj.rotors(i).angvel(3)*30/pi,3) ' RPM']);
            end
            % Add freestream
            y = -1:0.5:1; x = y; %z = 0:0.5:2;
            [Y,X] = meshgrid(y, x);
            Z = zeros(size(Y));
            fluidvel_A = hobj.A_C_O*fluid.velocity;
            U = ones(size(X))*fluidvel_A(1); V = ones(size(Y))*fluidvel_A(2); W = ones(size(Z))*fluidvel_A(3);
            quiver3(X,Y,Z,U,V,W,1.0*scale,'b','LineWidth',2);
            lgnd = [lgnd,"Freestream"];
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z');
            title({'Relative Veloctiy Vectors',['Vehicle Angular Rate = [' num2str(hobj.angvel(1)*30/pi,3) ' ' num2str(hobj.angvel(2)*30/pi,3) ' ' num2str(hobj.angvel(3)*30/pi,3) '] RPM']});
            legend(lgnd,'Location','northeast','color','none');
            view(-140,17)
            hold off
            ax = gca;
            ax.FontSize = 10;
            set(ax,'color','none');
        end % end visualizeRelativeVelocities
        %% visualizeSectionFrames
        function hfig = visualizeSections(hobj,hide,Oframe)
            % This method creates a plot of all of the section frames in the
            % vehicle in the vehicle frame.
            % ARGS: hide = bool hide the plot?
                  % Oframe = bool show in the inertial (O) frame?
            % OUT: hfig = figure handle
            if nargin < 2
                hide = false;
            end
            if nargin < 3
                Oframe = false;
            end
            % Leading edge of the section in the section frame
            leadEdge_a = [-1;0;0];
            for k=1:1:numel(hobj.rotors) % rotors loop
                % Velocity of the fluid in the rotor frame
                A_C_P = transpose(hobj.rotors(k).P_C_A);
                temp = size(hobj.rotors(k).sectPos); % 3 x nSects x nBlades
                for i=1:1:temp(2) % sections
                    for j=1:1:temp(3) % blades
                        % Location of the frame origin w.r.t. point A in
                        % frame A.
                        raA_A = A_C_P*(hobj.rotors(k).sectPos(:,i,j)) + hobj.rotorLocs(:,k);
                        % So far we have a 3 x nSects x nBlades x nRotors
                        % Rotate section coordinates from section frame
                        % into blade frame then blade frame into rotor
                        % frame then rotor frame into vehicle frame.
                        sectCoords = A_C_P*hobj.rotors(k).P_C_bx(:,:,j)*hobj.rotors(k).blades(j).b_C_a(:,:,i)*hobj.rotors(k).blades(j).sections(i).coords;
                        % Now we can get 3 x nPoints x nSect x nBlades x nRotors
                        coords2plot(:,:,i,j,k) = raA_A + sectCoords;
                        % of points to the chord/4 point. Now let's get 
                        %leadEdge_A(:,i,j,k) = A_C_P*hobj.rotors(k).P_C_bx(:,:,j)*hobj.rotors(k).blades(j).b_C_a(:,:,i)*leadEdge_a + raA_A(:,i,j,k);
                    end
                end
%                 lgnd(k) = join(["U_{rel}",num2str(k)]);
            end % End rotors loop            
            v = 'on';
            if hide
                v = 'off';
            end
            hfig = figure('visible',v,'Position',[600 100 900 900]);
            % 3 x nSects x nBlades x nRotors
            %quiver3(raA_A(1,:,:,1),raA_A(2,:,:,1),raA_A(3,:,:,1),leadEdge_A(1,:,:,1),leadEdge_A(2,:,:,1),leadEdge_A(3,:,:,1),scale,'-','color','r','LineWidth',2);
            hold on
            %text(hobj.rotorLocs(1,1),hobj.rotorLocs(2,1),hobj.rotorLocs(3,1)*0.9,['Rotor Rotation Rate = ' num2str(hobj.rotors(1).angvel(3)*30/pi,3) ' RPM']);
            for k=1:1:numel(hobj.rotors)
                clrs = '-gcym';
                temp = size(hobj.rotors(k).sectPos); % 3 x nSects x nBlades
                for i=1:1:temp(2) % Sections
                    for j=1:1:temp(3) % Blades
                        plot3(coords2plot(1,:,i,j,k),coords2plot(2,:,i,j,k),coords2plot(3,:,i,j,k));
                        text(hobj.rotorLocs(1,k),hobj.rotorLocs(2,k),hobj.rotorLocs(3,k)*0.8,['Rotor Number = ' num2str(hobj.rotors(k).ID,1)]);
                    end                    
                end
                %quiver3(raA_A(1,:,:,i),raA_A(2,:,:,i),raA_A(3,:,:,i),leadEdge_A(1,:,:,i),leadEdge_A(2,:,:,i),leadEdge_A(3,:,:,i),scale,'-','color',clrs(k),'LineWidth',2);
                %text(hobj.rotorLocs(1,i),hobj.rotorLocs(2,i),hobj.rotorLocs(3,i)*1.1,['Rotor Rotation Rate = ' num2str(hobj.rotors(i).angvel(3)*30/pi,3) ' RPM']);
            end
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z');
            title('Frames in the Vehicle');
            view(-140,17)
            hold off
            ax = gca;
            ax.FontSize = 10;
            set(ax,'color','none');
        end % end visualizeSectionFrames
        
        %% Other methods
        function cmc = getCylinderMomentCoefficient(hobj,f,fine)
            % Second argument lets you use the high-fidlity model for low
            % Reynolds numbers.
            if nargin < 3; fine = false; end
            if fine
                re = f.getRotationalRe(hobj.angvel(3),hobj.body.radius);
                if re > 60
                    syms x
                    cmc = vpasolve(x==(1/(-0.8572+1.25*log(re*sqrt(x))))^2,x,0.02); % 0.02 is initial guess
                    clear x;
                elseif re < 1.0e-6
                    cmc = 10/hobj.body.radius;
                else
                    cmc = 8/re;
                end
            else
                cmc = 0.02;
            end
        end
        function sinebuoyforce(hobj,t,a,w)
            % todo this is not a property of the vehicle. Get rid of this.
            hobj.buoyforce = a*sin(w*t);
        end
        function addGenerator(hobj,g,loc)
            hobj.generator = g;
            % Adjuct center mass
            hobj.centermass = hobj.centermass*hobj.mass/(hobj.mass+g.mass) + loc*g.mass/(hobj.mass+g.mass);
            % add mass to total
            hobj.mass = hobj.mass + g.mass;            
        end
        function computeMstar(hobj)
            if isempty(hobj.rotors)
                error('Need an assembled vehicle to compute the mass matrix');
            end
            M11 = hobj.mass*eye(3);
            r_csa_A = hobj.centermass;
            r_csa_A_X = makecrossmat(r_csa_A);
            M12 = -r_csa_A_X;
            P_C_A = hobj.rotors(1).P_C_A;
            Q_C_A = hobj.rotors(2).P_C_A;
            A_C_P = P_C_A.';
            A_C_Q = Q_C_A.';
            Ip_P = hobj.rotors(1).inertia;
            Iq_Q = hobj.rotors(2).inertia;
            M23 = A_C_P*Ip_P;
            M24 = A_C_Q*Iq_Q;
            M33 = Ip_P;
            M44 = Iq_Q;
            r_pa_A = hobj.rotorLocs(:,1);
            r_pa_A_X = makecrossmat(r_pa_A);
            r_qa_A = hobj.rotorLocs(:,2);
            r_qa_A_X = makecrossmat(r_qa_A);
            Ia_A = hobj.body.inertia;
            m1 = hobj.rotors(1).mass;
            m2 = hobj.rotors(2).mass;
            A = Ia_A - m1*r_pa_A_X*r_pa_A_X - m2*r_qa_A_X*r_qa_A_X + A_C_P*Ip_P*A_C_P + A_C_Q*Iq_Q*A_C_Q;
            xi = [0 0 1];
            hobj.Mstar = [M11 M12 zeros(3,1) zeros(3,1);...
                transpose(M12) A M23*xi.' M24*xi.';...
                zeros(1,3) xi*transpose(M23) xi*M33*xi.' 0;...
                zeros(1,3) xi*transpose(M24) 0 xi*M44*xi.'];
            hobj.MstarInv = inv(hobj.Mstar);
        end % computeMstar
        
        function computeAddedMass(hobj,fld,bodySecs)
            if isempty(hobj.rotors)
                error('Need an assembled vehicle to compute the added mass matrix');
            end
            if nargin < 2
                density = 1000; % assume water if no fluid is passed
                warning('Assuming water for added mass calculations');
            else
                density = fld.density;
            end
            if nargin < 3
                bodySecs = 27;
            end
            thickness = 0.15; % this is chord thickness. todo - make a property on the airfoil and get it from there.
            v = amvehicle; % this is an added mass vehicle object
            for i=1:1:numel(hobj.rotors)
                vr(i) = amvehicle;
            end
            % body
            bodySecs = 2*round(bodySecs/2); % in case an odd number is passed in
            halfEllipse = hobj.body.length/2;
            sectWidth = hobj.body.length/bodySecs;
            for i=1:1:bodySecs/2 % add circles in the +x direction
                z = (i-1)*sectWidth + sectWidth/2;
                r = hobj.body.radius*sqrt(1-(z)^2/halfEllipse^2);
                v.addSection(amsection('ellipse',r,r,sectWidth),[0;0;z],[0;pi/2;0]);
            end
            for i=1:1:bodySecs/2 % add circles in the -x direction
                z = -((i-1)*sectWidth + sectWidth/2);
                r = hobj.body.radius*sqrt(1-(z)^2/halfEllipse^2);
                v.addSection(amsection('ellipse',r,r,sectWidth),[0;0;z],[0;pi/2;0]);
            end
            % rotors
            for i=1:1:numel(hobj.rotors)
                for j=1:1:numel(hobj.rotors(i).blades)
                    for k=1:1:numel(hobj.rotors(i).blades(j).sections)
                        a_C_s = [0 1 0; -1 0 0; 0 0 1]; % from that amsection frame to our section frame
                        b_C_a = hobj.rotors(i).blades(j).b_C_a(:,:,k); % section to blade
                        P_C_b = hobj.rotors(i).P_C_bx(:,:,j); % blade to rotor
                        A_C_P = transpose(hobj.rotors(i).P_C_A); % rotor to vehicle
                        A_C_s = A_C_P*P_C_b*b_C_a*a_C_s; % all the way, airborne
                        % get the Euler angles
                        beta = atan2(A_C_s(1,2),A_C_s(1,1));
                        theta = atan2(A_C_s(2,3),A_C_s(3,3));
                        gamma = -asin(A_C_s(1,3));
                        v.addSection(amsection('ellipse',hobj.rotors(i).blades(j).sections(k).chord/2,thickness*hobj.rotors(i).blades(j).sections(k).chord/2,hobj.rotors(i).blades(j).sections(k).width),...
                            hobj.rotorLocs(:,i)+A_C_P*hobj.rotors(i).P_C_bx(:,:,j)*hobj.rotors(i).blades(j).sectLocs(:,k),[beta;gamma;theta]);
                        vr(i).addSection(amsection('ellipse',hobj.rotors(i).blades(j).sections(k).chord/2,thickness*hobj.rotors(i).blades(j).sections(k).chord/2,hobj.rotors(i).blades(j).sections(k).width),...
                            A_C_P*hobj.rotors(i).P_C_bx(:,:,j)*hobj.rotors(i).blades(j).sectLocs(:,k),[beta;gamma;theta]);
                    end %sections
                end %blades
            end %rotors
            % fill in the added mass matrix
            % This is another place where coaxial is assumed.
            hobj.Mam = zeros(size(hobj.Mstar));
            hobj.Mam(1:6,1:6) = v.getAddedMass(density);
            Mam1 = vr(1).getAddedMass(density);
            Mam2 = vr(2).getAddedMass(density);
            hobj.Mam(3,7) = Mam1(3,6);
            hobj.Mam(3,8) = Mam2(3,6);
            hobj.Mam(7,7) = Mam1(6,6);
            hobj.Mam(8,8) = Mam2(6,6);
            hobj.Mam(7,3) = Mam1(6,3);
            hobj.Mam(8,3) = Mam2(6,3);
        end % end computeAddedMass
        
        function computeMtot(hobj)
            if isempty(hobj.Mam)
                warning('No added mass matrix. Assuming a zero matrix');
                hobj.Mam = zeros(size(hobj.Mstar));
            end
            hobj.Mtot = hobj.Mstar - hobj.Mam;
            hobj.MtotInv = inv(hobj.Mtot);
        end
        
        function setRelativeDensity(hobj,rd)
            hobj.relDensity = rd;
        end
        function setAddedMassMatrix(hobj,am)
            hobj.Mam = am;
        end
        function setRotorFricConst(hobj,c)
            hobj.rotorFric = c;
        end
        %$ Setters
        function setType(hobj,t)
            hobj.type = t;
        end
        
        %$ Getters
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

function mat = makecrossmat(x)
mat=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end