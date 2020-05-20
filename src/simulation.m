classdef simulation < handle
    
    properties % todo make private. No reason all these are exposed.
        % objects
        vhcl % a vehicle object
        fld  % a fluid object
        thr  % a tether object
        % other
        name     % str unique name for the simulation case
        % caseID % is this needed?
        timestep % real timestep
        duration % real duration of simulation (total sim time)
    end % end public properties
    properties (SetAccess = 'private')
        times  % numSteps,1 array of simulation times
        states % numSteps,numStates array of states
    end % end private properties
    
    methods
        % Constructor
        function hobj = simulation(fn)
            hobj.setup(fn);
        end
        
        function setup(hobj,fn)
            if isempty(fn)
                % Manual setup
                warning('Manual simulation setup');
            else
                % Input file setup
                % if it is an m-file execute, otherwise read line by line
                % and evaluate each line
                tkn = strsplit(fn,'.');
                if strcmp(tkn(2),'m')
                    run(['input\' char(tkn(1))]);
                else
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
                end
                if ~strcmp(runname,tkn)
                    error('Run name does not match the file name passed in');
                end
                hobj.name = runname;
                hobj.timestep = tstep;
                hobj.duration = totalSimTime;
                
                %% Make objects
                % fluid
                hobj.fld = fluid; % No arguments to fluid gives the obj water properties
                hobj.fld.init(fluidtype);
                disp('Fluid Initialized');
                try
                    hobj.fld.setFlowType(flowtype,flowparms);
                catch
                    disp('Assuming steady flow');
                    flowtype = 1;
                    flowparms = [];
                    hobj.fld.setFlowType(flowtype,flowparms);
                    fluidBaseVelocity = fluidVelocity;
                end
                hobj.fld.setMeanVelocity(fluidBaseVelocity);
                % airfoils - same for the entire rotor so we just need two
                af1 = airfoil(airfoiltype1);
                if isstruct(twist1)
                    af1.setAoAopt(twist1.AoAopt_deg);
                end
                af2 = airfoil(airfoiltype2);
                if isstruct(twist2)
                    af2.setAoAopt(twist2.AoAopt_deg);
                end
                % blade sections - moving to blade
                %bs1 = bladesection(secChord1,secWidth1,af1);
                %bs2 = bladesection(secChord2,secWidth2,af2);
                % Make a blade comprised of the same section.
                % Rotor 1 blades
%                 for i=1:1:numSections1
%                     section1(i) = bs1;
%                 end
                for i=1:1:numBlades1
                    bld1(i) = blade;
                    bld1(i).init(secChord1,af1,bladeLength1,numSections1,bladeMass1,twist1);
                end
                % Rotor 2 blades need to twist in the opposite direction
%                 for i=1:1:numSections2
%                     section2(i) = bs2;
%                 end
                for i=1:1:numBlades2
                    bld2(i) = blade;
                    bld2(i).init(secChord2,af2,bladeLength2,numSections2,bladeMass2,twist2);
                    bld2(i).reverseTwist;
                end
                % Make a set of rotors
                r1 = rotor(bld1);
                r1.setID(1);
                r1.setAxialFlowFactor(axflowfactor1);
                % Need to make rotor 2 have blades with twist 180-
                r2 = rotor(bld2);
                r2.setID(2);
                r2.setAxialFlowFactor(axflowfactor2);
                % Make a vehicle body
                vbod = vehiclebody(vbmass,I,vbcentermass);                
                vbod.setLength(vblength);
                vbod.setRadius(vbradius);
                % Make a vehicle
                rotPoints = [rot1point,rot2point];
                hobj.vhcl = vehicle();
                hobj.vhcl.init(vbod,[r1,r2],rotPoints,vcentermass,vbtetherpoint,vbbuoypoint);
                hobj.vhcl.setRelativeDensity(vreldensity);
                disp('Vehicle initialized');
                % Associate rotor objects with vehicle object
                r1.connectVehicle(hobj.vhcl);
                r2.connectVehicle(hobj.vhcl);
                % set the rotor orientation
                hobj.vhcl.rotors(1).orientation = rot1ornt;
                hobj.vhcl.rotors(2).orientation = rot2ornt;
                % set initial conditions
                hobj.vhcl.orientation = [initialPitch;initialYaw;initialRoll];
                hobj.vhcl.position = [initialLongitudinal;initialLateral;initialVertical];
                hobj.vhcl.velocity = [initialSurge;initialSway;initialHeave];
                w10 = cos(initialYaw)*sin(initialRoll)*initialPitchRate + cos(initialRoll)*initialYawRate;
                w20 = cos(initialYaw)*cos(initialRoll)*initialPitchRate - sin(initialRoll)*initialYawRate;
                w30 = -sin(initialYaw)+initialRollRate;
                hobj.vhcl.angvel = [w10;w20;w30];
                hobj.vhcl.rotors(1).angvel = [0;0;rot1initRPM/60*2*pi];
                hobj.vhcl.rotors(2).angvel = [0;0;rot2initRPM/60*2*pi];               
                % add generator
                gen = generator(gmconst,gflux,grarm,gkvisc,gmass); % todo size generator constant so that torque is reasonable
                hobj.vhcl.addGenerator(gen,gpoint);
                hobj.vhcl.generator.setLoadResistance(grload);
                % add tether
                if isempty(tdamp)
                    tdamp = tdampfac*2*sqrt(tspring*hobj.vhcl.mass);
                end
                %tthr = tether(tnnodes,tnodlocs,tspring,tdamp,tunstrch);
                hobj.addTether(tether(tnnodes,tnodlocs,tspring,tdamp,tunstrch));
                % Compute the mass matrix
                hobj.vhcl.computeMstar;
                
                % Add apparent mass to the mass matrix
%                 am = zeros(size(hobj.vhcl.Mstar));
%                 if isempty(addedMass)
%                     % todo Compute the added mass matrix
%                     error('Compute added mass is a todo');
%                 elseif numel(addedMass) ~= 8
%                     error('Full 6x6 added mass matrix not yet supported. Provide values for the 6 diagonals or leave empty to compute.');
%                 else
%                     am = addedMass;
%                 end
                
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
            fprintf('\nDone writing data file.\n');
            
            % Now write a vehilce configuration file of the smae same that
            % holds all of the simulation configuration information.
            sim = hobj;
            svcmd = ['save ' pwd '\products\data\' hobj.name '.mat sim'];
            eval(svcmd); clear sim;
            fprintf(['\nDone writing configuration file: ' hobj.name '.mat\n']);
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
            [t, y] = ode45(@(t,y) vehicleState( t,y,hobj.vhcl,hobj.fld,hobj.thr),p.Results.tspan,x0,opts);
            hobj.times = t;
            hobj.states = y;
            else
                error('only ode45 currently supported');
            end
        end
        
        function addTether(hobj,t)
            hobj.thr = t;
        end         
        
        function hfig = showme(hobj,vis)
            if nargin < 2
                vis = 'on';
            end
            f = hobj.fld;
            v = hobj.vhcl;
            % Get plot arrays from subfunction
            [rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,f);
            x = 300; y = 100; w = x+600; h = y+600;
            hfig = figure('position',[x y w h],'visible',vis);
            scale = 1;
            y = linspace(-v.rotors(1).blades(1).length,v.rotors(1).blades(1).length,10);
            z = v.position(3)+y;
            [Y,Z] = meshgrid(y, z);
            X = zeros(size(Y));
            U = ones(size(X))*f.velocity(1); V = ones(size(Y))*f.velocity(2); W = ones(size(Z))*f.velocity(3);
            %quiver3(0,0,0,f.velocity(1),f.velocity(2),f.velocity(3),scale,'b');
            quiver3(X,Y,Z,U,V,W,scale,'b');
            hold on
            vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
            plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
            quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c')
            quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g')
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z');
            title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
            legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','bestoutside','color','none','Orientation','horizontal');
            view(-130,20)
            hold off            
        end % end showme
        
        function hfig = showmevehicle(hobj,vis)
            if nargin < 2
                vis = 'on';
            end
            f = hobj.fld;
            v = hobj.vhcl;
            % Get plot arrays from subfunction
            [rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,f);
            x = 300; y = 100; w = x+600; h = y+600;
            hfig = figure('position',[x y w h],'visible',vis);
            scale = 1;
            y = linspace(-v.rotors(1).blades(1).length,v.rotors(1).blades(1).length,10);
            z = v.position(3)+y;
            [Y,Z] = meshgrid(y, z);
            X = zeros(size(Y));
            U = ones(size(X))*f.velocity(1); V = ones(size(Y))*f.velocity(2); W = ones(size(Z))*f.velocity(3);
            %quiver3(0,0,0,f.velocity(1),f.velocity(2),f.velocity(3),scale,'b');
            quiver3(X,Y,Z,U,V,W,scale,'b');
            hold on
            bline = [v.position+transpose(v.A_C_O)*[0;0;v.body.length/2],v.position-transpose(v.A_C_O)*[0;0;v.body.length/2]];
            vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
            
            plot3(bline(1,:),bline(2,:),bline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
            for i=1:1:numel(v.rotors)
                for j=1:1:numel(v.rotors(i).blades)
                    if j == 1
                        mrk = '-';
                        clr = [235 158 52;52 185 235]/255;
                    else
                        mrk = '-';
                        clr = [1 0 0;0 0 1];
                    end
                    P_C_bx = v.rotors(i).P_C_bx(:,:,j);
                    P_C_A = v.rotors(i).P_C_A; A_C_P = transpose(P_C_A);
                    rblines = v.position+transpose(v.A_C_O)*(v.rotorLocs(:,i)+A_C_P*P_C_bx*[0;v.rotors(i).blades(j).tipLoc;0]);
                    plot3([vline(1,i),rblines(1,:)],[vline(2,i),rblines(2,:)],[vline(3,i),rblines(3,:)],mrk,'LineWidth',3,'color',clr(i,:));
                end
            end
            %quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c')
            %quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g')
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z');
            title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
            legend({'Freestream Velocity','Vehicle Body','','','Rotor 1','','','Rotor 2'},'Location','bestoutside','color','none','Orientation','horizontal');
            view(-30,15)
            hold off            
        end % end showmevehicle
        
        function hfig = showmelift(hobj,vis)
            if nargin < 2
                vis = 'on';
            end
            f = hobj.fld;
            v = hobj.vhcl;
            % Get plot arrays from subfunction
            [rp1o_O,rp2o_O,rap1_O,rap2_O,~,~,L1_O,~,L2_O,~] = getPlotArrays(v,f);
            x = 300; y = 100; w = x+600; h = y+600;
            hfig = figure('position',[x y w h],'visible',vis);
            scale = 1;
            y = linspace(-v.rotors(1).blades(1).length,v.rotors(1).blades(1).length,10);
            z = v.position(3)+y;
            [Y,Z] = meshgrid(y, z);
            X = zeros(size(Y));
            U = ones(size(X))*f.velocity(1); V = ones(size(Y))*f.velocity(2); W = ones(size(Z))*f.velocity(3);
            %quiver3(0,0,0,f.velocity(1),f.velocity(2),f.velocity(3),scale,'b');
            quiver3(X,Y,Z,U,V,W,scale,'color','none');
            hold on
            vbodyline = [v.position+transpose(v.A_C_O)*[0;0;v.body.length*0.5],v.position+transpose(v.A_C_O)*[0;0;-v.body.length*0.5]];
            plot3(vbodyline(1,:),vbodyline(2,:),vbodyline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
            quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),L1_O(1,:,:),L1_O(2,:,:),L1_O(3,:,:),scale,'c')
            quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),L2_O(1,:,:),L2_O(2,:,:),L2_O(3,:,:),scale,'g')
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z');
            title(['Rotor Speed in Body Frame | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
            legend({'Freestream Velocity','Vehicle Body','Lift Rotor 1','Lift Rotor 2'},'Location','bestoutside','color','none','Orientation','horizontal');
            view(-130,20)
            hold off            
        end % end showmelift
        
        function hfig = showmedrag(hobj,vis)
            if nargin < 2
                vis = 'on';
            end
            f = hobj.fld;
            v = hobj.vhcl;
            % Get plot arrays from subfunction
            [rp1o_O,rp2o_O,rap1_O,rap2_O,~,~,~,D1_O,~,D2_O] = getPlotArrays(v,f);
            x = 300; y = 100; w = x+600; h = y+600;
            hfig = figure('position',[x y w h],'visible',vis);
            scale = 1;
            y = linspace(-v.rotors(1).blades(1).length,v.rotors(1).blades(1).length,10);
            z = v.position(3)+y;
            [Y,Z] = meshgrid(y, z);
            X = zeros(size(Y));
            U = ones(size(X))*f.velocity(1); V = ones(size(Y))*f.velocity(2); W = ones(size(Z))*f.velocity(3);
            %quiver3(0,0,0,f.velocity(1),f.velocity(2),f.velocity(3),scale,'b');
            quiver3(X,Y,Z,U,V,W,scale,'color','none');
            hold on
            vbodyline = [v.position+transpose(v.A_C_O)*[0;0;v.body.length*0.5],v.position+transpose(v.A_C_O)*[0;0;-v.body.length*0.5]];
            plot3(vbodyline(1,:),vbodyline(2,:),vbodyline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
            quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),D1_O(1,:,:),D1_O(2,:,:),D1_O(3,:,:),scale,'c')
            quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),D2_O(1,:,:),D2_O(2,:,:),D2_O(3,:,:),scale,'g')
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z');
            title(['Rotor Speed in Body Frame | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
            legend({'Freestream Velocity','Vehicle Body','Drag Rotor 1','Drag Rotor 2'},'Location','bestoutside','color','none','Orientation','horizontal');
            view(-130,20)
            hold off            
        end % end showmedrag
        
        function hfig = showmeloads(hobj,vis)
            if nargin < 2
                vis = 'on';
            end
            f = hobj.fld;
            v = hobj.vhcl;
            % Get plot arrays from subfunction
            [rp1o_O,rp2o_O,rap1_O,rap2_O,~,~,L1_O,D1_O,L2_O,D2_O] = getPlotArrays(v,f);
            T1_O = L1_O + D1_O;
            T2_O = L2_O + D2_O;
            x = 300; y = 100; w = x+600; h = y+600;
            hfig = figure('position',[x y w h],'visible',vis);
            scale = 1;
            temp = 0;
            for i=1:1:numel(hobj.vhcl.rotors)
                for j=1:1:numel(hobj.vhcl.rotors(i).blades)
                    if hobj.vhcl.rotors(i).blades(j).length > temp
                        temp = hobj.vhcl.rotors(i).blades(j).length;
                    end
                end
            end
            temp2 = max(hobj.vhcl.body.length,temp);
            y = linspace(-v.position(2)-temp2,v.position(2)+temp2,10);
            z = v.position(3)+y;
            [Y,Z] = meshgrid(y, z);            
            
            X = ones(size(Y))*(v.position(1)-2*temp2);
            U = ones(size(X))*f.velocity(1); V = ones(size(Y))*f.velocity(2); W = ones(size(Z))*f.velocity(3);
            %quiver3(0,0,0,f.velocity(1),f.velocity(2),f.velocity(3),scale,'b');
            quiver3(X,Y,Z,U,V,W,scale,'color',[52 195 235]/255);
            hold on
            vbodyline = [v.position+transpose(v.A_C_O)*[0;0;v.body.length*0.5],v.position+transpose(v.A_C_O)*[0;0;-v.body.length*0.5]];
            plot3(vbodyline(1,:),vbodyline(2,:),vbodyline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
            quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),T1_O(1,:,:),T1_O(2,:,:),T1_O(3,:,:),scale,'k')
            quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),T2_O(1,:,:),T2_O(2,:,:),T2_O(3,:,:),scale,'k')
            
            axis equal
            axis([v.position(1)-2*temp2 v.position(1)+temp2 v.position(2)-temp2 v.position(2)+temp2 v.position(3)-temp2 v.position(3)+temp2]);
            
            xlabel('x'); ylabel('y'); zlabel('z');
            title(['Rotor Speed in Body Frame | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
            legend({'Freestream Velocity','Vehicle Body','Loads Rotor 1','Loads Rotor 2'},'Location','bestoutside','color','none','Orientation','horizontal');
            view(-45,25)
            hold off            
        end % end showmeloads
        
        function hfig = showmerotors(hobj,vis)
            %disp('Displaying vehicle');
            if nargin < 2
                vis = 'on';
            end
            x = 300; y = 100; w = x+600; h = y+600;
            hfig = figure('position',[x y w h],'visible',vis);
            ax1 = axes('Parent',hfig); hold on;
            for i=1:1:numel(hobj.vhcl.rotors)
                for j=1:1:numel(hobj.vhcl.rotors(i).blades)
                    if j == 1
                        mrk = '-';
                        clr = [235 158 52;52 185 235]/255;
                    else
                        mrk = '-';
                        clr = [1 0 0;0 0 1];
                    end
                    for k=1:1:numel(hobj.vhcl.rotors(i).blades(j).sections)
                        a = hobj.vhcl.rotors(i).blades(j).sections(k).coords;                         
                        % rotate into blade
                        a = hobj.vhcl.rotors(i).blades(j).b_C_a(:,:,k)*a;
                        % move to proper location in blade
                        b = a + hobj.vhcl.rotors(i).blades(j).sectLocs(:,k);
                        % rotate into rotor
                        b = hobj.vhcl.rotors(i).P_C_bx(:,:,j)*b;
                        % rotate into vehicle
                        b = transpose(hobj.vhcl.rotors(i).P_C_A)*b;
                        % move to proper location in vehicle
                        b = b + hobj.vhcl.rotorLocs(:,i);
                        % finally put in earth frame
                        b = transpose(hobj.vhcl.A_C_O)*b;
                        plot3(ax1,b(1,:),b(2,:),b(3,:),'color',clr(i,:));                        
                    end %sections
                end %blades
            end %rotors
            temp = 0;
            for i=1:1:numel(hobj.vhcl.rotors)
                for j=1:1:numel(hobj.vhcl.rotors(i).blades)
                    if hobj.vhcl.rotors(i).blades(j).length > temp
                        temp = hobj.vhcl.rotors(i).blades(j).length;
                    end
                end
            end
                
            temp2 = max(hobj.vhcl.body.length,temp);
            axis([-temp2 temp2 -temp2 temp2]);
            axis equal;
            hold off;
            xlabel('x'); ylabel('y'); zlabel('z');
            view(-45,30);
        end
        
        function makeSinglePlot(hobj,plt,varargin)
            % Makes a sinlge plot of the state data
            defaultPlots = 'all';
            defaultFigsize = [50 50 600 400];
            defaultFigcolor = 'none';
            defaultAxcolor = 'none';
            defaultSavefigs = false;
            defaultVisible = 'on';
            p = inputParser;
            %validateOutput = @(x) isa(x,'function_handle');
            addParameter(p,'figsize',defaultFigsize,@isnumeric);
            addParameter(p,'figcolor',defaultFigcolor,@ischar);
            addParameter(p,'axcolor',defaultAxcolor,@ischar);
            addParameter(p,'savefigs',defaultSavefigs,@islogical);
            addParameter(p,'visible',defaultVisible,@ischar);
            parse(p,varargin{:});
            
            t = hobj.times;
            y = hobj.states;
            % todo make this and other formatting controllable with vararg
            plotlowx = p.Results.figsize(1); plotlowy = p.Results.figsize(2); plotw = p.Results.figsize(3); ploth = p.Results.figsize(4);
            if p.Results.savefigs
                if ~exist(['products\images\' hobj.name],'dir')
                    mkdir(['products\images\' hobj.name]);
                end
            end
            
            switch plt
                case 'x'
                    figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
                    plot(t,y(:,1),'r');
                    set(gca,'Color',p.Results.axcolor);
                    xlabel('Time (s)'); ylabel('Position (m)');
                    legend({'x'},'Location','Best');
                    title(['Case: ' hobj.name]);
                    if p.Results.savefigs
                        export_fig(['products\images\' hobj.name '\xposition.png'],'-png','-transparent','-m3');
                    end
                case 'y'
                    figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
                    plot(t,y(:,2),'b');
                    set(gca,'Color',p.Results.axcolor);
                    xlabel('Time (s)'); ylabel('Position (m)');
                    legend({'y'},'Location','Best');
                    title(['Case: ' hobj.name]);
                    if p.Results.savefigs
                        export_fig(['products\images\' hobj.name '\yposition.png'],'-png','-transparent','-m3');
                    end
                case 'z'
                    figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
                    plot(t,y(:,3),'g');
                    set(gca,'Color',p.Results.axcolor);
                    xlabel('Time (s)'); ylabel('Position (m)');
                    legend({'z'},'Location','Best');
                    title(['Case: ' hobj.name]);
                    if p.Results.savefigs
                        export_fig(['products\images\' hobj.name '\zposition.png'],'-png','-transparent','-m3');
                    end
                case 'theta'
                    plotlowy = plotlowy+ploth; % Move up
                    figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
                    plot(t,y(:,4)*180/pi,'r');
                    set(gca,'Color',p.Results.axcolor);
                    xlabel('Time (s)'); ylabel('Angle (deg)');
                    legend({'\theta'},'Location','Best');
                    title(['Case: ' hobj.name]);
                    if p.Results.savefigs
                        export_fig(['products\images\' hobj.name '\thetaorientation.png'],'-png','-transparent','-m3');
                    end
                case 'gamma'
                    plotlowy = plotlowy+ploth; % Move up
                    figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
                    plot(t,y(:,5)*180/pi,'b');
                    set(gca,'Color',p.Results.axcolor);
                    xlabel('Time (s)'); ylabel('Angle (deg)');
                    legend({'\gamma'},'Location','Best');
                    title(['Case: ' hobj.name]);
                    if p.Results.savefigs
                        export_fig(['products\images\' hobj.name '\gammaorientation.png'],'-png','-transparent','-m3');
                    end
                case 'beta'
                    plotlowy = plotlowy+ploth; % Move up
                    figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
                    plot(t,y(:,6)*180/pi,'g');
                    set(gca,'Color',p.Results.axcolor);
                    xlabel('Time (s)'); ylabel('Angle (deg)');
                    legend({'\beta'},'Location','Best');
                    title(['Case: ' hobj.name]);
                    if p.Results.savefigs
                        export_fig(['products\images\' hobj.name '\betaorientation.png'],'-png','-transparent','-m3');
                    end
                otherwise
                    errstr = [plt ' is not a supported plot. Add it or try something else'];
                    error(errstr);
            end
            
            
        end % end makeSinglePlot
               
    end % methods
    
    methods(Static)
        function makeMovie(infn, varargin)
            % makeMovie  Static method that makes a movie out of a data file
            %   ARGS:
            %       infn - name of the data file
            %       varargin - name,value pair to control the movie
            defaultOutfile = infn;
            defaultFramerate = 24;
            defaultSpeedfactor = 1;
            p = inputParser;
            %validateOutput = @(x) isa(x,'function_handle');
            addParameter(p,'outfile',defaultOutfile,@ischar);
            addParameter(p,'framerate',defaultFramerate,@isnumeric);
            addParameter(p,'speedfactor',defaultSpeedfactor,@isnumeric);
            parse(p,varargin{:});           

            moviefile = ['products\videos\' p.Results.outfile '.avi'];
            % get data
            try
                dat = importdata(['products\data\' infn '.txt']);
            catch
                disp(['Unable to make a movie of ' infn]);
                % todo log and move on
                return;
            end
            headers = dat.colheaders;
            dat = dat.data;
            
            % get the simulation object
            svcmd = ['load ' pwd '\products\data\' infn '.mat'];
            eval(svcmd); clear svcmd;
            
            % make the vectors and everything
            r_ao_O = dat(:,2:4);
            Ov_ao_A = dat(:,11:13);
            O_omg_A_A = dat(:,8:10);
            theta = dat(:,5);
            gamma = dat(:,6);
            beta = dat(:,7);
            p3 = dat(:,14);
            fi3 = dat(:,15);
            q3 = dat(:,16);
            sy3 = dat(:,17);
%             r_pa_A = sim.vhcl.rotorLocs(:,1);
%             r_qa_A = sim.vhcl.rotorLocs(:,2);            
            
            % plot the data and grab frames
            tottime = dat(end,1); % sim time in seconds
            
            nsamps = length(dat(:,1));
            tstep = mean(diff(dat(:,1))); % seconds per sample            
            framerate = p.Results.framerate; % target framerate in frames per second
            speedfactor = p.Results.speedfactor;
            sPerFrame = 1/framerate;
            nskips = round(speedfactor*sPerFrame/tstep);
            if nskips < 1
                error('Not enough resolution in data to achieve target framerate.');
            end
            framerate = round(speedfactor/(nskips*tstep)); % Actual frame rate
            nframes = floor(nsamps/nskips); % Actual number of frames

            hfig = figure('position',[0 0 1728 972]);
            ax = axes('parent',hfig);
            maxx = max(r_ao_O(:,1)) + max([sim.vhcl.body.length sim.vhcl.rotors(1).blades(1).length sim.vhcl.rotors(2).blades(1).length]);
            maxy = max(abs(r_ao_O(:,2))) + max([sim.vhcl.body.length sim.vhcl.rotors(1).blades(1).length sim.vhcl.rotors(2).blades(1).length]);
            maxz = max(abs(r_ao_O(:,3))) + max([sim.vhcl.body.length sim.vhcl.rotors(1).blades(1).length sim.vhcl.rotors(2).blades(1).length]);
            maxy = max([maxy 0.3*maxx]);
            maxz = max([maxz 0.3*maxx]);
            cmap = colormap(jet(100));
            cmaxtensionvalue = 2000;
            for i = 1:1:nframes
                smp = nskips*(i-1)+1;
                % Update velocity on the fluid object
                sim.fld.updateVelocity(dat(smp,1));
                % breakout
                cthe = cos(theta(smp)); sthe = sin(theta(smp));
                cgam = cos(gamma(smp)); sgam = sin(gamma(smp));
                cbet = cos(beta(smp)); sbet = sin(beta(smp));
                cfi3 = cos(fi3(smp)); sfi3 = sin(fi3(smp));
                csy3 = cos(sy3(smp)); ssy3 = sin(sy3(smp));
                cxi3 = [cfi3 csy3]; sxi3 = [sfi3 ssy3]; % holds both fi3 and sy3 for the loop
                
                A_C_O = [cbet*cthe + sbet*sgam*sthe, cgam*sbet, sbet*cthe*sgam - cbet*sthe;...
                    cbet*sgam*sthe - sbet*cthe, cbet*cgam, sbet*sthe + cbet*cthe*sgam;...
                    cgam*sthe,-sgam,cgam*cthe];
                O_C_A = transpose(A_C_O);
                
                r_epo_O = r_ao_O(smp,:).' - O_C_A*[0;0;sim.vhcl.body.length*0.5]; % body end proximal - assuming that the slender body is straight
                r_edo_O = r_ao_O(smp,:).' + O_C_A*[0;0;sim.vhcl.body.length*0.5]; % body end distal - assuming that the slender body is straight
%                 r_pa_O = O_C_A*r_pa_A;
%                 r_qa_O = O_C_A*r_qa_A;
                
                % plot the tether
                r_tpo_O = r_ao_O(smp,:).' + O_C_A*sim.vhcl.tetherpoint;
                Ov_tpo_O = O_C_A*Ov_ao_A(smp,:).' + O_C_A*cross(O_omg_A_A(smp,:).',sim.vhcl.tetherpoint);
                tetherforce_O = sim.thr.computeTension(r_tpo_O,Ov_tpo_O);
                str = ['Tether tension: ' num2str(norm(tetherforce_O),'%10.2f') 'N'];
                
                thclr = round(interp1([0,cmaxtensionvalue],[1,100],min(cmaxtensionvalue,norm(tetherforce_O))));
                plot3(ax,[0 r_tpo_O(1)],[0 r_tpo_O(2)],[0 r_tpo_O(3)],'--','LineWidth',1.0,'Color',cmap(thclr,:));
                hold on
                % plot the body
                plot3(ax,[r_epo_O(1) r_edo_O(1)],[r_epo_O(2) r_edo_O(2)],[r_epo_O(3) r_edo_O(3)],'k','LineWidth',2.0);
                r_cso_O = r_ao_O(smp,:).' + O_C_A*sim.vhcl.centermass;
                r_cbo_O = r_ao_O(smp,:).' + O_C_A*sim.vhcl.buoypoint;
                plot3(ax,r_cso_O(1),r_cso_O(2),r_cso_O(3),'kv','MarkerSize',6.0);
                plot3(ax,r_ao_O(smp,1),r_ao_O(smp,2),r_ao_O(smp,3),'go','MarkerSize',6.0);
                plot3(ax,r_cbo_O(1),r_cbo_O(2),r_cbo_O(3),'b^','MarkerSize',6.0);
                % plot each blade in rotors
                for jj = 1:1:numel(sim.vhcl.rotors)
                    r_pa_A = sim.vhcl.rotorLocs(:,jj);
                    r_pa_O = O_C_A*r_pa_A;
                    r_po_O = r_ao_O(smp,:).' + r_pa_O;
                    P_C_A = [cxi3(jj) sxi3(jj) 0; -sxi3(jj) cxi3(jj) 0; 0 0 1];
                    A_C_P = transpose(P_C_A);                    
                    for ii=1:1:numel(sim.vhcl.rotors(jj).blades)
                        if ii == 1
                            clr = [235 158 52;52 185 235]/255;
                        else
                            clr = [1 0 0;0 0 1];
                        end
                        r_tipp_bx = [0;sim.vhcl.rotors(jj).blades(ii).length;0];
                        r_tipp_O = O_C_A*A_C_P*sim.vhcl.rotors(jj).P_C_bx(:,:,ii)*r_tipp_bx;
                        plot3(ax,[r_po_O(1) r_po_O(1)+r_tipp_O(1)],...
                            [r_po_O(2) r_po_O(2)+r_tipp_O(2)],...
                            [r_po_O(3) r_po_O(3)+r_tipp_O(3)],'-','color',clr(jj,:),'LineWidth',2.0);
                    end
                end
                % plot the velocity vector at the origin
                quiver3(ax,0,0,0,sim.fld.velocity(1),sim.fld.velocity(2),sim.fld.velocity(3),'Color','c','LineWidth',2.0,'MaxHeadSize',0.5);
                %annotation('textbox',[0.3 0.2 0.5 0.2],'String',{'U_\infty Vector'},'FitBoxToText','on');
                axis equal                
                axis([-1 maxx -maxy maxy -maxz maxz]);
                view(-30,20)
                
                xlabel('x'); ylabel('y'); zlabel('z');
%                 
%                 title(['\fontsize{20}RPM1 = ' num2str((p3(smp)/(2*pi)*60),'%5.2f'),...
%                     '  |  \fontsize{20}RPM2 = ' num2str((q3(smp)/(2*pi)*60),'%5.2f'),...
%                     '  |  U_\infty = ' num2str(norm(sim.fld.velocity),'%5.2f'),...
%                     '  |  Time = ' num2str(dat(smp,1),'%5.2f'),...
%                     '  |  Relative Density = ' num2str(sim.vhcl.body.relDensity,'%2.1f')],'FontSize',12);                
                title(['\fontsize{20}RPM_R_e_l = ' num2str(((p3(smp)-q3(smp))/(2*pi)*60),'%5.2f'),...
                    '  |  U_\infty = ' num2str(norm(sim.fld.velocity),'%5.2f'),...
                    '  |  Time = ' num2str(dat(smp,1),'%5.2f'),...
                    '  |  Relative Density = ' num2str(sim.vhcl.relDensity,'%3.2f')],'FontSize',12);
                hold off
                text(0,0,-0.2*maxz,str,'Fontsize',12);
                F(i) = getframe(hfig);
                %clear an;
            end % end loop through data
            % make the movie
            vw = VideoWriter(moviefile);            
            vw.FrameRate = framerate;
            %v.Quality = 100;% v.Width = 800; w.Height = 450;
            open(vw);
            writeVideo(vw,F); close(vw);
        end % end makeMovie
        function makePlots(infn, varargin)
            % makePlots  Static method that makes plots from a data file
            %   makePlots(infn) makes plots of the data in the file infn
            %   makePlots(infn,Name,Value) specifies properties using one or more Name,Value pair arguments 
            %       'plots',value - which plots to run
                        % 'all' (default)
                        % 'position', 'orientation', 'angrate', 'speed', 'rotang',
                        % 'rotspeed' 
                        % 'rotspeedAp' rotor speeds in the A' frame
                    % figsize - size of the figure [x,y,w,h]
                    % figcolor - color of the figure
                    
            % get data 
            try
                % Load the simulation object
                % The mat file has all of the data that is in the corresponding
                % text file. The text file is for consumption outside of
                % matlab.
                svcmd = ['load ' pwd '\products\data\' infn '.mat'];
                eval(svcmd); clear svcmd;
            catch
                disp(['Unable to load ' infn]);
                % todo log and move on
                return;
            end
            defaultPlots = 'all';
            defaultFigsize = [50 50 600 400];
            defaultFigcolor = 'none';
            defaultAxcolor = 'none';
            defaultSavefigs = false;
            defaultVisible = 'on';
            p = inputParser;
            %validateOutput = @(x) isa(x,'function_handle');
            addParameter(p,'plots',defaultPlots,@ischar);
            addParameter(p,'figsize',defaultFigsize,@isnumeric);
            addParameter(p,'figcolor',defaultFigcolor,@ischar);
            addParameter(p,'axcolor',defaultAxcolor,@ischar);
            addParameter(p,'savefigs',defaultSavefigs,@islogical);
            addParameter(p,'visible',defaultVisible,@ischar);
            parse(p,varargin{:});
            whatplots = p.Results.plots;
                        
            t = sim.times;
            y = sim.states;
            % todo make this and other formatting controllable with vararg
            plotlowx = p.Results.figsize(1); plotlowy = p.Results.figsize(2); plotw = p.Results.figsize(3); ploth = p.Results.figsize(4);
            if p.Results.savefigs
                if ~exist(['products\images\' sim.name],'dir')
                    mkdir(['products\images\' sim.name]);
                end
            end
            
            if (strcmp(whatplots,'position') || strcmp(whatplots,'all'))
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,1),'r',t,y(:,2),'b',t,y(:,3),'g');
            set(gca,'Color',p.Results.axcolor);
            xlabel('Time (s)'); ylabel('Position (m)');
            legend({'x','y','z'},'Location','Best');
            title(['Case: ' sim.name]);
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\position.png'],'-png','-transparent','-m3');
            end
            end
            
            if (strcmp(whatplots,'orientation') || strcmp(whatplots,'all'))
            plotlowy = plotlowy+ploth; % Move up
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,4)*180/pi,'r',t,y(:,5)*180/pi,'b',t,y(:,6)*180/pi,'g');
            set(gca,'Color',p.Results.axcolor);
            xlabel('Time (s)'); ylabel('Angle (deg)');
            legend({'\theta','\gamma','\beta'},'Location','Best');
            title(['Case: ' sim.name]);
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\orientation.png'],'-png','-transparent','-m3');
            end
            end

            if (strcmp(whatplots,'angrate') || strcmp(whatplots,'all'))
            plotlowx = plotlowx+plotw; plotlowy = 50; % Move over
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,7)*30/pi,'r',t,y(:,8)*30/pi,'b',t,y(:,9)*30/pi,'g');
            set(gca,'Color',p.Results.axcolor);
            xlabel('Time (s)'); ylabel('Angular Rate (RPM)');
            legend({'\omega_1','\omega_2','\omega_3'},'Location','Best');
            title(['Case: ' sim.name]);
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\angrate.png'],'-png','-transparent','-m3');
            end
            end

            if (strcmp(whatplots,'speed') || strcmp(whatplots,'all'))
            plotlowy = plotlowy+ploth; % Move up
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,10),'r',t,y(:,11),'b',t,y(:,12),'g');
            set(gca,'Color',p.Results.axcolor);
            xlabel('Time (s)'); ylabel('Speed (m/s)');
            legend({'u_1','u_2','u_3'},'Location','Best');
            title(['Case: ' sim.name]);
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\speed.png'],'-png','-transparent','-m3');
            end
            end

            if (strcmp(whatplots,'rotang') || strcmp(whatplots,'all'))
            plotlowx = plotlowx+plotw; plotlowy = 50; % Move over
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,14)*180/pi,'r',t,y(:,16)*180/pi,'b');
            set(gca,'Color',p.Results.axcolor);
            xlabel('Time (s)'); ylabel('Angle (deg)');
            legend({'\phi_3','\psi_3'},'Location','Best');
            title(['Case: ' sim.name]);
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\rotang.png'],'-png','-transparent','-m3');
            end
            end

            if (strcmp(whatplots,'rotspeed') || strcmp(whatplots,'all'))
            plotlowy = plotlowy+ploth; % Move up
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,13)*30/pi,'r',t,y(:,15)*30/pi,'b'); %rad/s*180/pi*60/360 = 30/pi
            set(gca,'Color',p.Results.axcolor);
            xlabel('Time (s)'); ylabel('Angular Rate (RPM - in Body Frame)');
            legend({'p_3','q_3'},'Location','Best');
            title(['Case: ' sim.name]);
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\rotspeed.png'],'-png','-transparent','-m3');
            end
            end
            
            if (strcmp(whatplots,'rotspeedAp') || strcmp(whatplots,'all'))
            plotlowx = plotlowx-0.5*plotw; % Move up
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,(y(:,13)+y(:,9))*30/pi,'r',t,(y(:,15)+y(:,9))*30/pi,'b'); %rad/s*180/pi*60/360 = 30/pi
            set(gca,'Color',p.Results.axcolor);
            xlabel('Time (s)'); ylabel('Angular Rate (RPM - in Intermediate Frame)');
            legend({'$\hat{p}_3$','$\hat{q}_3$'},'Location','Best','Interpreter','latex');
            title(['Case: ' sim.name]);
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\rotspeedIF.png'],'-png','-transparent','-m3');
            end
            end
        end
                     
        function makeInputFile(varargin)
            % todo make this
        end
        
        function sim = loadsim(infn,datapath)
            if nargin < 2
                svcmd = ['load ' pwd '\products\data\' infn '.mat'];
            else
                svcmd = ['load ' datapath '\' infn '.mat'];
            end
            eval(svcmd); clear svcmd;
        end
    end % static methods
end

% visualization subfunction
% todo - make this a static method?
function [rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,L1_O,D1_O,L2_O,D2_O] = getPlotArrays(v,f)
    Urel1_P1 = v.rotors(1).computeHydroLoads(f);
    Urel2_P2 = v.rotors(2).computeHydroLoads(f);
    temp = size(v.rotors(1).sectPos);
    O_C_A = transpose(v.A_C_O);
    O_C_P1 = O_C_A*transpose(v.rotors(1).P_C_A);
    O_C_P2 = O_C_A*transpose(v.rotors(2).P_C_A);
    for i=1:1:temp(2)
        for j=1:1:temp(3)
            rap1_O(:,i,j) = O_C_P1*v.rotors(1).sectPos(:,i,j);
            %rap2_O(:,i,j) = O_C_P2*v.rotors(2).sectPos(:,i,j);
            Urel1_O(:,i,j) = O_C_P1*Urel1_P1(:,i,j);
            %Urel2_O(:,i,j) = O_C_P2*Urel2_P2(:,i,j);
            L1_O(:,i,j) = O_C_P1*v.rotors(1).sectLift(:,i,j);
            D1_O(:,i,j) = O_C_P1*v.rotors(1).sectDrag(:,i,j);
            %L2_O(:,i,j) = O_C_P2*v.rotors(2).sectLift(:,i,j);
            %D2_O(:,i,j) = O_C_P2*v.rotors(2).sectDrag(:,i,j);
        end
    end
    temp = size(v.rotors(2).sectPos);
    for i=1:1:temp(2)
        for j=1:1:temp(3)
            %rap1_O(:,i,j) = O_C_P1*v.rotors(1).sectPos(:,i,j);
            rap2_O(:,i,j) = O_C_P2*v.rotors(2).sectPos(:,i,j);
            %Urel1_O(:,i,j) = O_C_P1*Urel1_P1(:,i,j);
            Urel2_O(:,i,j) = O_C_P2*Urel2_P2(:,i,j);
            %L1_O(:,i,j) = O_C_P1*v.rotors(1).sectLift(:,i,j);
            %D1_O(:,i,j) = O_C_P1*v.rotors(1).sectDrag(:,i,j);
            L2_O(:,i,j) = O_C_P2*v.rotors(2).sectLift(:,i,j);
            D2_O(:,i,j) = O_C_P2*v.rotors(2).sectDrag(:,i,j);
        end
    end
    % Show the forces and/or velocity vectors as quivers applied at the section
    % locations. For that we need rp1o_O and rp2o_O
    rco_O = v.position;
    rp1c_O = O_C_A*v.rotorLocs(:,1);
    rp2c_O = O_C_A*v.rotorLocs(:,2);
    rp1o_O = rco_O + rp1c_O;
    rp2o_O = rco_O + rp2c_O;
end