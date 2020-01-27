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
        centermass  % 3x1 vector location of center of mass of the vehicle body in the vehicle frame (c1,c2,c3)
        tetherpoint % 3xm vector of location of the tether attachment point(s) in the vehicle frame (t1, t2, t3)
        buoypoint   % 3x1 vector of location of the center of buoyancy (b1, b2, b3)
        mass        % scalar total mass of the vehicle
        type        % uint identifier | see typeName get method for list of types
    end % End private properties    
    properties (Dependent)
        A_C_O       % 3x3 tranformation matrix from vehicle frame to inertial frame
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
            if nargin == 0
                % Make sure to call init
                warning('Make sure to initialize vehicle');
            elseif nargin == 2
               % assume simple single rotor with cm at body [0;0;0];
               hobj.body = bod;
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
        end % End constructor
        
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
        end % End init
        %% computeHydroLoads
        function urel = computeHydroLoads(hobj,f)
            % Computes the net hydrodynamic force and torque vectors
            frc = 0;
            trq = 0;
            % Get aerodynamic loads on rotors, cross and sum
            for i=1:1:numel(hobj.rotors)
                urel(:,:,:,i) = hobj.rotors(i).computeHydroLoads(f);
                % 3xjxn array of lift and drag at each section expressed in
                % the rotor frame (j=num sections, n=num blades)
                sectionforces = hobj.rotors(i).sectLift + hobj.rotors(i).sectDrag;
                % 3xjxn array of position of each section expressed in the rotor frame
                sectionpositions = hobj.rotors(i).sectPos;
                [~,numsecs,numblades] = size(sectionpositions);
                % Rotate into the vehicle frame
                for j = 1:1:numsecs
                    for n = 1:1:numblades
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
            end % end rotor loop
            
            % Get aerodynamic loads on body, cross and sum
            % todo(rodney) add vehicle hydrodynamic loads.
            
            % Sum along the 2dim gives 3x1xnumBlades of total blade loads
            % then sum along the 3dim to get 3x1 vector of total loads
            hobj.force = sum(sum(frc,2),3);
            hobj.torque = sum(sum(trq,2),3);
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
        function sinebuoyforce(hobj,t,a,w)
            % todo this is not a property of the vehicle. Get rid of this.
            hobj.buoyforce = a*sin(w*t);
        end
        function addGenerator(hobj,g)
            hobj.generator = g;
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