classdef simulation < handle
    
    properties
        % objects
        vhcl % a vehicle object
        fld  % a fluid object
        thr  % a tether object
        % other
        name     % str unique name for the simulation case
        % caseID % is this needed?
        timestep % real timestep
        duration % real duration of simulation (total sim time)
    end
    properties (SetAccess = 'private')
        times  % numSteps,1 array of simulation times
        states % numSteps,numStates array of states
    end
    
    methods
        % Constructor
        function hobj = simulation(fn)
            hobj.setup(fn);
        end
        
        function setup(hobj,fn)
            if isempty(fn)
                % Manual setup
            else
                % Input file setup
                fid = fopen(['input\' fn]);
                if fid < 0
                    error(['Error opening: input\' fn]);
                end
                while true
                    tline = fgetl(fid);            
                    if isnumeric(tline)
                        break;
                    end
                    eval(tline);
                end
                fclose(fid);
                hobj.name = runname;
                hobj.timestep = tstep;
                hobj.duration = totalSimTime;
                
                %% Make objects
                % fluid
                hobj.fld = fluid(fluidtype); % No arguments to fluid gives the obj water properties
                hobj.fld.velocity = fluidVelocity;
                % airfoils - same for the entire rotor so we just need one
                af = airfoil(airfoiltype);
                % blade sections
                bs = bladesection(secChord,secWidth,af);
                % Make a blade comprised of the same section.
                % Rotor 1 blades
                for i=1:1:numSections
                    section(i) = bs;
                end
                for i=1:1:numBlades
                    bld1(i) = blade(section,bladeMass,twist);
                end
                % Rotor 2 blades need to twist in the opposite direction
                for i=1:1:numBlades
                    bld2(i) = blade(section,bladeMass,twist);
                    bld2(i).reverseTwist;
                end
                % Make a set of rotors
                r1 = rotor(bld1);
                r1.setID(1);
                % Need to make rotor 2 have blades with twist 180-
                r2 = rotor(bld2);
                r2.setID(2);
                % Make a vehicle body
                vbod = vehiclebody(vbmass,I);
                vbod.setRelativeDensity(vbreldensity);
                % Make a vehicle
                rotPoints = [rot1point,rot2point];
                hobj.vhcl = vehicle();
                hobj.vhcl.init(vbod,[r1,r2],rotPoints,vbcentermass,vbtetherpoint,vbbuoypoint);
                % Associate rotor objects with vehicle object
                r1.connectVehicle(hobj.vhcl);
                r2.connectVehicle(hobj.vhcl);
                % set initial conditions
                hobj.vhcl.orientation = [initialPitch;initialYaw;initialRoll];
                hobj.vhcl.position = [initialLongitudinal;initialLateral;initialVertical];
                hobj.vhcl.velocity = [initialSurge;initialSway;initialHeave];
                w10 = cos(initialYaw)*sin(initialRoll)*initialPitchRate + cos(initialRoll)*initialYawRate;
                w20 = cos(initialYaw)*cos(initialRoll)*initialPitchRate - sin(initialRoll)*initialYawRate;
                w30 = -sin(initialYaw)+initialRollRate;
                hobj.vhcl.angvel = [w10;w20;w30];                
            end
        end
        
        function write2file(hobj,datflnm)
            % Optional argument is the name of the file that you want to
            % save to. Otherwise the filename will be whatever the
            % simulation name is.
            outDir = 'products\data\';
            if ~exist(outDir,'dir')
                mkdir(outDir);
            end
            %flnm = ['data\simCase' num2str(simCaseID) '.txt'];
            if nargin < 2
                flnm = [outDir hobj.name '.txt'];
            else
                flnm = [outDir datflnm '.txt'];
            end
            % Break into states for writing
            [~,nStates] = size(hobj.states);
            dualrtr = false;
            if nStates == 16
                disp('Writing dual-rotor system data to file.');
                frmspc = '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n';
                dualrtr = true;
            elseif nStates == 14
                disp('Writing single-rotor system data to file.');
                frmspc = '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n';
            else
                error('The data you want me to write contains a strange number of states.');
            end
            headers = ["time","x1","x2","x3","theta","gamma","beta","w1","w2","w3","u1","u2","u3","p3","fi3","q3","sy3"];
            fid = fopen(flnm,'w');
            if fid < 0
                error(['Unable to open: ' flnm ' to write']);
            end
            if ~dualrtr
                headers = headers(1:end-2);
            end
            dat = [hobj.times,hobj.states].';
            fprintf(fid,frmspc,headers);
            frmspc = strrep(frmspc,'s','f');
            fprintf(fid,frmspc,dat);
            fclose(fid);
            fprintf('\nDone\n');
        end
        
        function simulate(hobj,varargin)
            defaultTspan = 0:hobj.timestep:hobj.duration;
            defaultSolver = 'ode45';
            defaultStats = 'on';
            defaultOutput = @odeplot;
            p = inputParser;
            %validateOutput = @(x) isa(x,'function_handle');
            addParameter(p,'tspan',defaultTspan,@isnumeric);
            addParameter(p,'solver',defaultSolver,@ischar);
            addParameter(p,'stats',defaultStats,@ischar);
            addParameter(p,'output',defaultOutput);
            parse(p,varargin{:});
            % Initial states
            % State vector
            % x = [x1; x2; x3; theta; gamma; beta; w1; w2; w3; u1; u2; u3; p3; fi3; q3; sy3];
            x0 = [hobj.vhcl.position; hobj.vhcl.orientation; hobj.vhcl.angvel; hobj.vhcl.velocity;...
                hobj.vhcl.rotors(1).angvel(3); hobj.vhcl.rotors(1).orientation(3); hobj.vhcl.rotors(2).angvel(3); hobj.vhcl.rotors(2).orientation(3)];
            % todo add tether states We are going to add a tether with two links and one central node
            if strcmpi(p.Results.solver,'ode45')
            disp('Running the simulation');
            opts = odeset('RelTol',1e-6,'AbsTol',1e-6,'Stats',p.Results.stats,'OutputFcn',p.Results.output);
            %opts = odeset('RelTol',1e-5,'AbsTol',1e-6);
            % todo make a simulation class.
            % todo also make a system class that includes the tether. Move completely
            % away from the state files. Let's take advantage of this hierarchy.
            [t, y] = ode45(@(t,y) vehicleState( t,y,hobj.vhcl,hobj.fld),p.Results.tspan,x0,opts);
            hobj.times = t;
            hobj.states = y;
            else
                error('only ode45 currently supported');
            end
        end
        
        function makeMovie(hobj)
            % todo make this happen captain
            moviefile = ['products\videos\' hobj.name '.avi'];
        end
        
%         function showme(hobj)
%             
%             [rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(hobj.vhcl,hobj.fld);
%             x = 300; y = 100; w = x+600; h = y+600;
%             hfig = figure('position',[x y w h]);
%             scale = 1;
%             y = linspace(-v.rotors(1).blades(1).length,v.rotors(1).blades(1).length,10);
%             z = v.position(3)+y;
%             [Y,Z] = meshgrid(y, z);
%             X = zeros(size(Y));
%             U = ones(size(X))*f.velocity(1); V = ones(size(Y))*f.velocity(2); W = ones(size(Z))*f.velocity(3);
%             %quiver3(0,0,0,f.velocity(1),f.velocity(2),f.velocity(3),scale,'b');
%             quiver3(X,Y,Z,U,V,W,scale,'b');
%             hold on
%             vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
%             plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
%             quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c')
%             quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g')
%             axis equal
%             xlabel('x'); ylabel('y'); zlabel('z');
%             title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
%             legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','bestoutside','color','none','Orientation','horizontal');
%             view(-130,20)
%             hold off
%             
% 
%             % subfunction
%             function [rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,L1_O,D1_O,L2_O,D2_O] = getPlotArrays(v,f)
%                 Urel1_P1 = v.rotors(1).computeHydroLoads(f);
%                 Urel2_P2 = v.rotors(2).computeHydroLoads(f);
%                 temp = size(v.rotors(1).sectPos);
%                 O_C_A = transpose(v.A_C_O);
%                 O_C_P1 = O_C_A*transpose(v.rotors(1).P_C_A);
%                 O_C_P2 = O_C_A*transpose(v.rotors(2).P_C_A);
%                 for i=1:1:temp(2)
%                     for j=1:1:temp(3)
%                         rap1_O(:,i,j) = O_C_P1*v.rotors(1).sectPos(:,i,j);
%                         rap2_O(:,i,j) = O_C_P2*v.rotors(2).sectPos(:,i,j);
%                         Urel1_O(:,i,j) = O_C_P1*Urel1_P1(:,i,j);
%                         Urel2_O(:,i,j) = O_C_P2*Urel2_P2(:,i,j);
%                         L1_O(:,i,j) = O_C_P1*v.rotors(1).sectLift(:,i,j);
%                         D1_O(:,i,j) = O_C_P1*v.rotors(1).sectDrag(:,i,j);
%                         L2_O(:,i,j) = O_C_P2*v.rotors(2).sectLift(:,i,j);
%                         D2_O(:,i,j) = O_C_P2*v.rotors(2).sectDrag(:,i,j);
%                     end
%                 end
%                 % Show the forces and/or velocity vectors as quivers applied at the section
%                 % locations. For that we need rp1o_O and rp2o_O
%                 rco_O = v.position;
%                 rp1c_O = O_C_A*v.rotorLocs(:,1);
%                 rp2c_O = O_C_A*v.rotorLocs(:,2);
%                 rp1o_O = rco_O + rp1c_O;
%                 rp2o_O = rco_O + rp2c_O;
%             end
%         end
%         
    end % methods
    
end