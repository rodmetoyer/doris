classdef simulation < handle
    
    properties (SetAccess = 'private')
        % objects
        vhcl % a vehicle object
        fld  % a fluid object
        thr  % a tether object
        % Sim control
        name      % str unique name for the simulation case
        timestep  % real timestep
        duration  % real duration of simulation (total sim time)
        visuals = struct('makeplots',false,'makemovie',false,'speedfactor',1);
        % data
        times  % numSteps,1 array of simulation times
        states % numSteps,numStates array of states
    end % end public properties
    
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
                if numel(tkn) == 2
                    if strcmp(tkn(2),'m')
                        run(['input\' char(tkn(1))]);
                    elseif strcmp(tkn(2),'txt')
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
                else % just a name, assume it is an m file
                    run(['input\' char(tkn(1)) '.m']);
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
                hobj.fld.updateVelocity(0);
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
                r1.setTangentialFlowFactor(tnflowfactor1);
                try
                    r1.setBEMT([useBEMT,usePrandtl]);
                catch
                    % assume no BEMT
                    r1.setBEMT([false,false]);
                end
                % Need to make rotor 2 have blades with twist 180-
                r2 = rotor(bld2);
                r2.setID(2);
                r2.setAxialFlowFactor(axflowfactor2);
                r2.setTangentialFlowFactor(tnflowfactor2);
                try
                    r2.setBEMT([useBEMT,usePrandtl]);
                catch
                    % assume no BEMT
                    r2.setBEMT([false,false]);
                end                
                % Make a vehicle body
                vbod = vehiclebody(vbmass,I,vbcentermass);                
                vbod.setLength(vblength);
                vbod.setRadius(vbradius);
                vbod.setViscTorsionModifier(vtMod,hifiTors);
                vbod.setViscDamping(vbnorm,vbax);
                % Make a vehicle
                rotPoints = [rot1point,rot2point];
                hobj.vhcl = vehicle();
                hobj.vhcl.setType(2); % 2 is coaxial
                hobj.vhcl.init(vbod,[r1,r2],rotPoints,vcentermass,vbtetherpoint,vbbuoypoint);
                hobj.vhcl.setRelativeDensity(vreldensity);
                hobj.vhcl.setRotorFricConst(rotorVisc);
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
                % add ballast
                try
                    hobj.vhcl.addBallast(ballastMass,ballastLoc);
                catch % No ballast specified
                    warning('No ballast specified. Best practice is to at least specify zero mass ballast.');
                end
                % add tether
                if isempty(tdamp)
                    tdamp = tdampfac*2*sqrt(tspring*hobj.vhcl.mass);
                end
                %tthr = tether(tnnodes,tnodlocs,tspring,tdamp,tunstrch);
                hobj.addTether(tether(tnnodes,tnodlocs,tspring,tdamp,tunstrch));
                % Compute the mass matrices
                hobj.vhcl.computeMstar;
                if isempty(addedMass)
                    hobj.vhcl.computeAddedMass(hobj.fld);
                else
                    hobj.vhcl.setAddedMassMatrix(addedMass);
                end
                hobj.vhcl.computeMtot;
                
                % Simulation control - todo organize. all sim contorl in
                % one spot.
                hobj.visuals.makeplots = makeplots;
                hobj.visuals.makemovie = makemovie;
                hobj.visuals.speedfactor = speedfactor;
            end
        end
        function setStaticICs(hobj)
            % orientation
            hvhcl = hobj.vhcl;
            initialtheta = atan2(2*hvhcl.centermass(1)*hvhcl.relDensity,hvhcl.body.length*(1-hvhcl.relDensity)-2*hvhcl.centermass(3)*hvhcl.relDensity);
            hvhcl.orientation = [initialtheta;0;0];
            % position
            weight = hobj.fld.gravity*hvhcl.mass;
            tension = weight*((1-hvhcl.relDensity)/hvhcl.relDensity);
            stretch = tension/hobj.thr.stiffness;
            phi = pi-initialtheta;
            hvhcl.position = [hvhcl.body.length/2*sin(phi);0;hobj.thr.uslength+stretch-hvhcl.body.length/2*cos(phi)];            
        end
        function written = write2file(hobj,datflnm)
            % First optional argument is the name of the file that you want to
            % save to. Otherwise the filename will be whatever the
            % simulation name is.
            
            outDir = 'products\data\plaintext\';
            if ~exist(outDir,'dir')
                mkdir(outDir);
            end
            %flnm = ['data\simCase' num2str(simCaseID) '.txt'];
            if nargin < 2
                flnm = [outDir hobj.name '.txt'];
            else
                flnm = [outDir datflnm '.txt'];
            end
            if exist(flnm,'file')
                opts.Interpreter = 'tex';
                opts.Default = 'No';
%                 quest = ['You are about to overwrite an existing data file.' newline,...
%                     'If you are batch re-running sims and want to avoid this dialogue you should delete the existing data files.' newline,...
%                     '\fontsize{20}Do you want to overwrite ' flnm '?'];
                quest = ['You are about to overwrite an existing data file.', newline,...
                    'If you are batch re-running sims and want to avoid this dialogue you should delete the existing data files.', newline,...
                    '\color{red}\fontsize{16}Do you want to overwrite ' hobj.name '?'];
                answer = questdlg(quest,'Data File Overwrite Confirmation','Yes','No',opts);
                if strcmp(answer,'No') || isempty(answer)
                    written = false;
                    return;
                end
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
            written = true;
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
            %[t, y] = ode45(@(t,y) vehicleState( t,y,hobj.vhcl,hobj.fld,hobj.thr),p.Results.tspan,x0,opts);
            [t, y] = ode45(@(t,y) hobj.computeStateDerivatives(t,y),p.Results.tspan,x0,opts);
            hobj.times = t;
            hobj.states = y;
            else
                error('only ode45 currently supported');
            end
        end
        
        function addTether(hobj,t)
            hobj.thr = t;
        end         
        
        function hfig = showmeold(hobj,vis)
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
        end % end showmeold
        
        function hfig = showme(hobj,vis)
            if nargin < 2
                vis = 'on';
            end
            x = 300; y = 100; w = x+600; h = y+600;
            hfig = figure('position',[x y w h],'visible',vis);
            ax1 = axes('Parent',hfig); hold on;
            v = hobj.vhcl;
            for i=1:1:numel(hobj.vhcl.rotors)
                for j=1:1:numel(hobj.vhcl.rotors(i).blades)
                    if j == 1
                        clr = [235 158 52;52 185 235]/255;
                    else
                        clr = [1 0 0;0 0 1];
                    end
                    b_C_a = hobj.vhcl.rotors(i).blades(j).b_C_a;
                    for k=1:1:numel(hobj.vhcl.rotors(i).blades(j).sections)
                        a = hobj.vhcl.rotors(i).blades(j).sections(k).coords;                         
                        % rotate into blade
                        a = b_C_a(:,:,k)*a;
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
                        plot3(ax1,hobj.vhcl.position(1)+b(1,:),hobj.vhcl.position(2)+b(2,:),hobj.vhcl.position(3)+b(3,:),'color',clr(i,:));                        
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
            vline = [hobj.vhcl.position+transpose(hobj.vhcl.A_C_O)*hobj.vhcl.rotorLocs(:,1),hobj.vhcl.position+transpose(hobj.vhcl.A_C_O)*hobj.vhcl.rotorLocs(:,2)];
            plot3(ax1,vline(1,:),vline(2,:),vline(3,:),'-','LineWidth',3,'color','k');
            tetherline = [[0;0;0],hobj.vhcl.position+transpose(hobj.vhcl.A_C_O)*hobj.vhcl.tetherpoint];
            plot3(ax1,tetherline(1,:),tetherline(2,:),tetherline(3,:),':','LineWidth',2,'color','b');
            quiver3(ax1,0,0,0,hobj.fld.velocity(1),hobj.fld.velocity(2),hobj.fld.velocity(3),round(max(hobj.vhcl.position)/5),'LineWidth',3,'Marker','*','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',6);
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z');
            view(-100,20)
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
            quiver3(X,Y,Z,U,V,W,scale,'color','none');
            hold on
            vbodyline = [v.position+transpose(v.A_C_O)*[0;0;v.body.length*0.5],v.position+transpose(v.A_C_O)*[0;0;-v.body.length*0.5]];
            plot3(vbodyline(1,:),vbodyline(2,:),vbodyline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
            quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),L1_O(1,:,:),L1_O(2,:,:),L1_O(3,:,:),scale,'c')
            quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),L2_O(1,:,:),L2_O(2,:,:),L2_O(3,:,:),scale,'g')
            axis equal
            axis([v.position(1)-2*temp2 v.position(1)+temp2 v.position(2)-temp2 v.position(2)+temp2 v.position(3)-temp2 v.position(3)+temp2]);
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
            quiver3(X,Y,Z,U,V,W,scale,'color','none');
            hold on
            vbodyline = [v.position+transpose(v.A_C_O)*[0;0;v.body.length*0.5],v.position+transpose(v.A_C_O)*[0;0;-v.body.length*0.5]];
            plot3(vbodyline(1,:),vbodyline(2,:),vbodyline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
            quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),D1_O(1,:,:),D1_O(2,:,:),D1_O(3,:,:),scale,'c')
            quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),D2_O(1,:,:),D2_O(2,:,:),D2_O(3,:,:),scale,'g')
            axis equal
            axis([v.position(1)-2*temp2 v.position(1)+temp2 v.position(2)-temp2 v.position(2)+temp2 v.position(3)-temp2 v.position(3)+temp2]);
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
                    b_C_a = hobj.vhcl.rotors(i).blades(j).b_C_a;
                    for k=1:1:numel(hobj.vhcl.rotors(i).blades(j).sections)
                        a = hobj.vhcl.rotors(i).blades(j).sections(k).coords;                         
                        % rotate into blade
                        a = b_C_a(:,:,k)*a;
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
        end % end showmerotors
        
        function hfig = showmesection(hobj,rbs,vis)
            % rbs is (rotor, blade, section)
            %disp('Displaying vehicle');
            if nargin < 3
                vis = 'on';
            end
            x = 300; y = 100; w = x+600; h = y+600;
            hfig = figure('position',[x y w h],'visible',vis);
            ax1 = axes('Parent',hfig); hold on;
            a = hobj.vhcl.rotors(rbs(1)).blades(rbs(2)).sections(rbs(3)).coords;                         
            % rotate into blade
            b = hobj.vhcl.rotors(rbs(1)).blades(rbs(2)).b_C_a(:,:,rbs(3))*a;
%             % rotate into rotor
%             b = hobj.vhcl.rotors(rbs(1)).P_C_bx(:,:,rbs(2))*b;
%             % rotate into vehicle
%             b = transpose(hobj.vhcl.rotors(rbs(1)).P_C_A)*b;
%             % finally put in earth frame
%             b = transpose(hobj.vhcl.A_C_O)*b;
            plot(ax1,a(1,:),a(3,:),'r',b(1,:),b(3,:),'b');                        
            axis equal;
            xlabel('x'); ylabel('z');
        end % end showmesection
        
        function showmesectionvrel(hobj)
             if nargin < 2
                vis = 'on';
            end
            f = hobj.fld;
            v = hobj.vhcl;
            % Get plot arrays from subfunction
            [rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1,Urel2,~,~,~,~] = getPlotArrays(v,f);
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
            quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1(1,:,:),Urel1(2,:,:),Urel1(3,:,:),scale,'r')
            quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2(1,:,:),Urel2(2,:,:),Urel2(3,:,:),scale,'b')
            
            axis equal
            axis([v.position(1)-2*temp2 v.position(1)+temp2 v.position(2)-temp2 v.position(2)+temp2 v.position(3)-temp2 v.position(3)+temp2]);
            
            xlabel('x'); ylabel('y'); zlabel('z');
            title(['Rotor Speed in Body Frame | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
            legend({'Freestream Velocity','Vehicle Body','Velocity Rotor 1','Velocity Rotor 2'},'Location','bestoutside','color','none','Orientation','horizontal');
            view(-45,25)
            hold off  
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
                case 'skew'
                    skew = acos(cos(y(:,5)).*sin(y(:,4)));
                    figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
                    plot(t,skew*180/pi,'r');
                    set(gca,'Color',p.Results.axcolor);
                    xlabel('Time (s)'); ylabel('Skew (deg)');
                    legend({'\xi'},'Location','Best');
                    title(['Case: ' hobj.name]);
                    if p.Results.savefigs
                        export_fig(['products\images\' hobj.name '\xposition.png'],'-png','-transparent','-m3');
                    end
                otherwise
                    errstr = [plt ' is not a supported plot. Add it or try something else'];
                    error(errstr);
            end % switch            
        end % end makeSinglePlot
        
        function xdot = computeStateDerivatives(hobj,t,x)
            % State derivative function for the vehicle prototype code
            % INPUTS:
                % t = time (s)
                % x = state vector
                % v = vehicle object
                % f = fluid object representing the freestream
                % thr = tether object

            xdot = NaN(size(x));

            %% Modify fluid as a function of time

            %% Update object states from current state vector
            hobj.vhcl.position = [x(1);x(2);x(3)];   % Expressed in O frame
            hobj.vhcl.orientation = [x(4);x(5);x(6)]; % theta, gamma, beta
            hobj.vhcl.angvel = [x(7);x(8);x(9)];
            hobj.vhcl.velocity = [x(10);x(11);x(12)];
            hobj.vhcl.rotors(1).angvel(3) = x(13);
            hobj.vhcl.rotors(1).orientation(3) = x(14);
            hobj.vhcl.rotors(2).angvel(3) = x(15);
            hobj.vhcl.rotors(2).orientation(3) = x(16);

            %% Update fluid velocity and generator load resistance
            hobj.fld.updateVelocity(t);
            hobj.vhcl.generator.updateLoadResistance(t);

            %% Compute hydrodynamic loads
            hobj.vhcl.computeHydroLoads(hobj.fld);

            %% Compute tether loads and add them to the vehicle loads
            % For now, single-element elastic tether fixed at the origin of length 1m
            % todo(rodney) make a tether class. Number nodes = number links-1. End
            % nodes are part of the signal. Output is loads on end links and position
            % of internal nodes
            O_C_A = transpose(hobj.vhcl.A_C_O);
            r_to_O = [x(1); x(2); x(3)] + O_C_A*hobj.vhcl.tetherpoint;
            Ov_to_O = O_C_A*hobj.vhcl.velocity + O_C_A*cross(hobj.vhcl.angvel,hobj.vhcl.tetherpoint);
            [A,Ad] = hobj.thr.getAnchorPoint(t);
            hobj.thr.setLocationA(A);
            hobj.thr.setVelocityA(Ad);
            hobj.thr.setLocationB(r_to_O);
            hobj.thr.setVelocityB(Ov_to_O);
            if hobj.thr.numnodes > 0
                % States for two rotors 16+1=17 to 16+3N are tether node positions and states 16+3N+1 to
                % 16+3N+3N=16+6N are tether node velocities
                numstates = numel(x);
                % First update the tether states with the current information x(numstates+1:numstates+6*thr.numnodes)
                hobj.thr.nodelocs = x(numstates+1:numstates+3*hobj.thr.numnodes);
                hobj.thr.nodevels = x(numstates+3*hobj.thr.numnodes+1:numstates+6*hobj.thr.numnodes);
                % now compute the state derivatives
                [tetherforce,xdot(numstates+1+3*hobj.thr.numnodes:numstates+6*hobj.thr.numnodes)] = hobj.thr.computeTension(A,B,r_to_O,Ov_to_O,hobj.fld);
                xdot(numstates+1:numstates+3*hobj.thr.numnodes) = x(numstates+1+3*hobj.thr.numnodes:numstates+6*hobj.thr.numnodes);
            else % no internal nodes
                hobj.thr.computeTension(hobj.fld);
                tetherforce = hobj.thr.tension(:,2);
            end
            hobj.vhcl.addTetherLoads(hobj.vhcl.A_C_O*tetherforce);

            %% Compute hydrostatic loads
            buoyForceA = transpose(O_C_A)*[0;0;(1/hobj.vhcl.relDensity)*hobj.fld.gravity*hobj.vhcl.mass];
            weightA = transpose(O_C_A)*[0;0;-hobj.vhcl.mass*hobj.fld.gravity];
            buoyTorqueA = cross(hobj.vhcl.buoypoint,buoyForceA);
            weightTorqueA = cross(hobj.vhcl.centermass,weightA);
            hobj.vhcl.force = hobj.vhcl.force + buoyForceA + weightA;
            hobj.vhcl.torque = hobj.vhcl.torque + buoyTorqueA + weightTorqueA;

            %% Add generator loads
            % All of the hydrodynamic loads are computed and updated in the
            % computeHydroLoads method, but we need to add the generator loads to the
            % rotor torques.
            relRotSpeed = x(13)-x(15);
            gtq = hobj.vhcl.generator.getTorque(relRotSpeed);
            hobj.vhcl.rotors(1).addTorque(gtq);
            hobj.vhcl.rotors(2).addTorque(-gtq);

            %% prepare to compute state derivatives
            % Precompute transcedentals. I feel like this should make it faster rather
            % than computing over and over for the state derivatives.
            cosbeta = cos(x(6)); sinbeta = sin(x(6));
            cosgamma = cos(x(5)); singamma = sin(x(5));
            costheta = cos(x(4)); sintheta = sin(x(4));
            fi3 = x(14); sy3 = x(16);
            cosfi3 = cos(fi3); sinfi3 = sin(fi3);
            cossy3 = cos(sy3); sinsy3 = sin(sy3);
            % And get all the other stuff for the state derivative computations
            ta1 = hobj.vhcl.torque(1); ta2 = hobj.vhcl.torque(2); ta3 = hobj.vhcl.torque(3);
            f1 = hobj.vhcl.force(1); f2 = hobj.vhcl.force(2); f3 = hobj.vhcl.force(3);
            m1 = hobj.vhcl.rotors(1).mass; m2 = hobj.vhcl.rotors(2).mass; mv = hobj.vhcl.body.mass;
            c1 = hobj.vhcl.body.centermass(1); % c2 = hobj.vhcl.body.centermass(2);
            c3 = hobj.vhcl.body.centermass(3);  % Only this will be non-zero for a coaxial turbine
            %g1 = hobj.vhcl.rotorLocs(1,1); g2 = hobj.vhcl.rotorLocs(2,1);
            g3 = hobj.vhcl.rotorLocs(3,1); % Only this will be non-zero for a coaxial turbine
            %h1 = hobj.vhcl.rotorLocs(1,2); h2 = hobj.vhcl.rotorLocs(2,2);
            h3 = hobj.vhcl.rotorLocs(3,2); % Only this will be non-zero for a coaxial turbine
            % Position from point A to CM of system
            s1 = hobj.vhcl.centermass(1); % s2 = hobj.vhcl.centermass(2); 
            s3 = hobj.vhcl.centermass(3);
            Icv11 = hobj.vhcl.body.inertia(1,1);
            Icv22 = hobj.vhcl.body.inertia(2,2); 
            Icv33 = hobj.vhcl.body.inertia(3,3);
            Icv13 = hobj.vhcl.body.inertia(1,3);
            Ip11 = hobj.vhcl.rotors(1).inertia(1,1);
            Ip22 = hobj.vhcl.rotors(1).inertia(2,2);
            Ip33 = hobj.vhcl.rotors(1).inertia(3,3);
            Iq11 = hobj.vhcl.rotors(2).inertia(1,1);
            Iq22 = hobj.vhcl.rotors(2).inertia(2,2);
            Iq33 = hobj.vhcl.rotors(2).inertia(3,3);
            tp3 = hobj.vhcl.rotors(1).torqueCM(3);
            tq3 = hobj.vhcl.rotors(2).torqueCM(3);

            %% Compute state derivatives
            cvecstar = NaN(8,1);
            cvecstar(1) =  - (s1*x(8)^2 + x(9)*(s1*x(9) - s3*x(7)))*(m1 + m2 + mv) - (x(11)*x(9) - x(12)*x(8))*(m1 + m2 + mv);
            cvecstar(2) =  (s1*x(7)*x(8) + s3*x(8)*x(9))*(m1 + m2 + mv) + (x(10)*x(9) - x(12)*x(7))*(m1 + m2 + mv);
            cvecstar(3) =  - (s3*x(8)^2 - x(7)*(s1*x(9) - s3*x(7)))*(m1 + m2 + mv) - (x(10)*x(8) - x(11)*x(7))*(m1 + m2 + mv);
            cvecstar(4) =  x(8)*(x(9)*(Icv33 + c1^2*mv) - x(7)*(Icv13 + c1*c3*mv)) - (x(15) + x(9))*(x(7)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(8)*(Iq22*cossy3^2 + Iq11*sinsy3^2)) + mv*(x(9)*(c1*x(12) - c3*x(10)) + c1*x(11)*x(8)) - (x(7)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(8)*(Ip22*cosfi3^2 + Ip11*sinfi3^2))*(x(13) + x(9)) - x(8)*x(9)*(Icv22 + c1^2*mv + c3^2*mv) - x(13)*x(7)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(13)*x(8)*(Ip11*cosfi3^2 + Ip22*sinfi3^2) + Ip33*x(8)*(x(13) + x(9)) + Iq33*x(8)*(x(15) + x(9)) - x(15)*x(7)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(15)*x(8)*(Iq11*cossy3^2 + Iq22*sinsy3^2) - g3*m1*x(10)*x(9) - h3*m2*x(10)*x(9) - g3^2*m1*x(8)*x(9) - h3^2*m2*x(8)*x(9);
            cvecstar(5) =  (x(15) + x(9))*(x(8)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(7)*(Iq11*cossy3^2 + Iq22*sinsy3^2)) + x(9)*(x(7)*(Icv11 + c3^2*mv) - x(9)*(Icv13 + c1*c3*mv)) - x(7)*(x(9)*(Icv33 + c1^2*mv) - x(7)*(Icv13 + c1*c3*mv)) - mv*(c1*x(11)*x(7) + c3*x(11)*x(9)) + (x(8)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(7)*(Ip11*cosfi3^2 + Ip22*sinfi3^2))*(x(13) + x(9)) + x(13)*x(8)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) - x(13)*x(7)*(Ip22*cosfi3^2 + Ip11*sinfi3^2) - Ip33*x(7)*(x(13) + x(9)) - Iq33*x(7)*(x(15) + x(9)) + x(15)*x(8)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) - x(15)*x(7)*(Iq22*cossy3^2 + Iq11*sinsy3^2) - g3*m1*x(11)*x(9) - h3*m2*x(11)*x(9) + g3^2*m1*x(7)*x(9) + h3^2*m2*x(7)*x(9);
            cvecstar(6) =  x(7)*(x(7)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(8)*(Iq22*cossy3^2 + Iq11*sinsy3^2)) - x(8)*(x(8)*(Iq11*cossy3*sinsy3 - Iq22*cossy3*sinsy3) + x(7)*(Iq11*cossy3^2 + Iq22*sinsy3^2)) - x(8)*(x(7)*(Icv11 + c3^2*mv) - x(9)*(Icv13 + c1*c3*mv)) - mv*(x(7)*(c1*x(12) - c3*x(10)) - c3*x(11)*x(8)) + m1*(g3*x(10)*x(7) + g3*x(11)*x(8)) + m2*(h3*x(10)*x(7) + h3*x(11)*x(8)) + x(7)*(x(7)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(8)*(Ip22*cosfi3^2 + Ip11*sinfi3^2)) - x(8)*(x(8)*(Ip11*cosfi3*sinfi3 - Ip22*cosfi3*sinfi3) + x(7)*(Ip11*cosfi3^2 + Ip22*sinfi3^2)) + x(7)*x(8)*(Icv22 + c1^2*mv + c3^2*mv);
            cvecstar(7) =  Ip22*(x(7)*cosfi3 + x(8)*sinfi3)*(x(8)*cosfi3 - x(7)*sinfi3) - Ip11*(x(7)*cosfi3 + x(8)*sinfi3)*(x(8)*cosfi3 - x(7)*sinfi3);
            cvecstar(8) =  Iq22*(x(7)*cossy3 + x(8)*sinsy3)*(x(8)*cossy3 - x(7)*sinsy3) - Iq11*(x(7)*cossy3 + x(8)*sinsy3)*(x(8)*cossy3 - x(7)*sinsy3);

            tau = [f1;f2;f3;ta1;ta2;ta3;tp3;tq3];

            dtheta =  (x(8)*cosbeta + x(7)*sinbeta)/cosgamma;
            dgamma =  x(7)*cosbeta - x(8)*sinbeta;
            dbeta =  (x(9)*cosgamma - x(8)*cosbeta*singamma + x(7)*sinbeta*singamma)/cosgamma;
            dx1 =  x(10)*(cosbeta*costheta + sinbeta*singamma*sintheta) - x(11)*(sinbeta*costheta - cosbeta*singamma*sintheta) + x(12)*cosgamma*sintheta;
            dx2 =  x(11)*cosbeta*cosgamma - x(12)*singamma + x(10)*cosgamma*sinbeta;
            dx3 =  x(11)*(sinbeta*sintheta + cosbeta*costheta*singamma) - x(10)*(cosbeta*sintheta - sinbeta*costheta*singamma) + x(12)*cosgamma*costheta;
            dfi3 =  x(13);
            dsy3 =  x(15);
            %betavec = hobj.vhcl.MstarInv*(tau-cvec);
            betavec = hobj.vhcl.MtotInv*(tau-hobj.vhcl.Mtot*hobj.vhcl.MstarInv*cvecstar);

            xdot(1) = dx1;
            xdot(2) = dx2;
            xdot(3) = dx3;
            xdot(4) = dtheta;
            xdot(5) = dgamma;
            xdot(6) = dbeta;
            xdot(7) = betavec(4);  % omg_1
            xdot(8) = betavec(5);  % omg_2
            xdot(9) = betavec(6);  % omg_3
            xdot(10) = betavec(1); % u_1
            xdot(11) = betavec(2); % u_2
            xdot(12) = betavec(3); % u_3
            xdot(13) = betavec(7); % p_3
            xdot(14) = dfi3;
            xdot(15) = betavec(8); % q_3
            xdot(16) = dsy3;
            % Not currently using the following, but may be usefull in the future.
            % See DERIVATION OF DYNAMICAL EQUATIONS OF MOTION for derivation.
            % [tp11;tp12;tp21;tp22] = hobj.vhcl.Mtot*betavec + cvecstar;

        end % computeStateDerivatives
        
        function changeName(hobj,nn,sure)
            if ~strcmp(sure,'Sure')
                answr = questdlg(['Are you sure you want to change the name of ' hobj.name],'Name change verification','Yes','No','Yes');
                if strcmp(answr,'Yes')
                    hobj.name = nn;
                else
                    disp('OK, nothing has changed');
                end
            else
                hobj.name = nn;
            end
        end
               
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
%             try
%                 dat = importdata(['products\data\' infn '.txt']);
%             catch
%                 disp(['Unable to make a movie of ' infn]);
%                 % todo log and move on
%                 return;
%             end
%             headers = dat.colheaders;
%             dat = dat.data;
            
            % get the simulation object
            svcmd = ['load ' pwd '\products\data\' infn '.mat'];
            eval(svcmd); clear svcmd;
            dat = sim.times;
            dat = [dat sim.states];
            
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
                sim.thr.setLocationA([0;0;0]);
                sim.thr.setVelocityA([0;0;0]);
                sim.thr.setLocationB(r_tpo_O);
                sim.thr.setVelocityB(Ov_tpo_O);
                sim.thr.computeTension(sim.fld);
                tetherforce_O = sim.thr.tension(:,2);
                str = ['Tether tension: ' num2str(norm(tetherforce_O),'%10.2g') 'N'];
                
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
        
        function makePlots(infn,varargin)
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
            dirnow = pwd;
            dd = split(dirnow,'\');
            if ~strcmp(dd(end),'doris')
                cd ..\;
                dd = split(pwd,'\');
                if ~strcmp(dd(end),'doris')
                    cd(dirnow);
                    error('You can only make plots from the doris directory or a subdirectory one level down');
                end
            end
            defaultPlots = 'all';
            defaultFigsize = [50 50 600 400];
            defaultFigcolor = 'none';
            defaultAxcolor = 'none';
            defaultSavefigs = false;
            defaultVisible = 'on';
            defaultTitle = true;
            p = inputParser;
            %validateOutput = @(x) isa(x,'function_handle');
            addParameter(p,'plots',defaultPlots,@ischar);
            addParameter(p,'figsize',defaultFigsize,@isnumeric);
            addParameter(p,'figcolor',defaultFigcolor,@ischar);
            addParameter(p,'axcolor',defaultAxcolor,@ischar);
            addParameter(p,'savefigs',defaultSavefigs,@islogical);
            addParameter(p,'visible',defaultVisible,@ischar);
            addParameter(p,'showtitle',defaultTitle,@islogical);
            parse(p,varargin{:});
            whatplots = p.Results.plots;
                        
            try
                % Load the simulation object
                % The mat file has all of the data that is in the corresponding
                % text file. The text file is for consumption outside of
                % matlab.
                svcmd = ['load ' pwd '\products\data\' infn '.mat'];
                eval(svcmd); clear svcmd;
            catch ME
                try
                    svcmd = ['load ' pwd '\..\products\data\' infn '.mat'];
                    eval(svcmd); clear svcmd;
                catch
                    disp(['Unable to load ' infn]);
                    rethrow(ME);
                    % todo log and move on
                    % return;
                end                
            end
                        
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
            plot(t,y(:,1),'-r',t,y(:,2),'--b',t,y(:,3),'-.g','LineWidth',2.0);
            set(gca,'Color',p.Results.axcolor,'XLim',[0,max(t)]);
            xlabel('Time (s)'); ylabel('Position (m)');
            legend({'x','y','z'},'Location','Best');
            if p.Results.showtitle
                title(['Case: ' sim.name]);
            end
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\position.png'],'-png','-transparent','-m3');
            end
            end
            
            if (strcmp(whatplots,'orientation') || strcmp(whatplots,'all'))
            plotlowy = plotlowy+ploth; % Move up
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible);
            plot(t,y(:,4)*180/pi,'-r',t,y(:,5)*180/pi,'--b',t,y(:,6)*180/pi,'-.g','LineWidth',2.0);
            set(gca,'Color',p.Results.axcolor,'XLim',[0,max(t)]);
            xlabel('Elapsed Time (s)'); ylabel('Angle (deg)');
            legend({'\theta','\gamma','\beta'},'Location','Best');
            if p.Results.showtitle
                title(['Case: ' sim.name]);
            end
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\orientation.png'],'-png','-transparent','-m3');
                saveas(gcf,['products\images\' sim.name '\position.fig'],'fig');
            end
            end

            if (strcmp(whatplots,'angrate') || strcmp(whatplots,'all'))
            plotlowx = plotlowx+plotw; plotlowy = 50; % Move over
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,7)*30/pi,'-r',t,y(:,8)*30/pi,'--b',t,y(:,9)*30/pi,'-.g','LineWidth',2.0);
            set(gca,'Color',p.Results.axcolor,'XLim',[0,max(t)]);
            xlabel('Time (s)'); ylabel('Angular Rate (RPM)');
            legend({'\omega_1','\omega_2','\omega_3'},'Location','Best');
            if p.Results.showtitle
                title(['Case: ' sim.name]);
            end
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\angrate.png'],'-png','-transparent','-m3');
            end
            end

            if (strcmp(whatplots,'speed') || strcmp(whatplots,'all'))
            plotlowy = plotlowy+ploth; % Move up
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,10),'-r',t,y(:,11),'--b',t,y(:,12),'-.g','LineWidth',2.0);
            set(gca,'Color',p.Results.axcolor,'XLim',[0,max(t)]);
            xlabel('Time (s)'); ylabel('Speed (m/s)');
            legend({'u_1','u_2','u_3'},'Location','Best');
            if p.Results.showtitle
                title(['Case: ' sim.name]);
            end
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\speed.png'],'-png','-transparent','-m3');
            end
            end

            if (strcmp(whatplots,'rotang') || strcmp(whatplots,'all'))
            plotlowx = plotlowx+plotw; plotlowy = 50; % Move over
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,14)*180/pi,'-r',t,y(:,16)*180/pi,'--b','LineWidth',2.0);
            set(gca,'Color',p.Results.axcolor,'XLim',[0,max(t)]);
            xlabel('Time (s)'); ylabel('Angle (deg)');
            legend({'\phi_3','\psi_3'},'Location','Best');
            if p.Results.showtitle
                title(['Case: ' sim.name]);
            end
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\rotang.png'],'-png','-transparent','-m3');
            end
            end

            if (strcmp(whatplots,'rotspeed') || strcmp(whatplots,'all'))
            plotlowy = plotlowy+ploth; % Move up
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,y(:,13)*30/pi,'-r',t,y(:,15)*30/pi,'--b','LineWidth',2.0); %rad/s*180/pi*60/360 = 30/pi
            set(gca,'Color',p.Results.axcolor,'XLim',[0,max(t)]);
            xlabel('Time (s)'); ylabel('Angular Rate (RPM - in Body Frame)');
            legend({'p_3','q_3'},'Location','Best');
            if p.Results.showtitle
                title(['Case: ' sim.name]);
            end
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\rotspeed.png'],'-png','-transparent','-m3');
            end
            end
            
            if (strcmp(whatplots,'rotspeedAp') || strcmp(whatplots,'all'))
            plotlowx = plotlowx-0.5*plotw; % Move up
            figure('Position',[plotlowx plotlowy plotw ploth],'Color',p.Results.figcolor,'visible',p.Results.visible)
            plot(t,(y(:,13)+y(:,9))*30/pi,'-r',t,(y(:,15)+y(:,9))*30/pi,'--b','LineWidth',2.0); %rad/s*180/pi*60/360 = 30/pi
            set(gca,'Color',p.Results.axcolor,'XLim',[0,max(t)]);
            xlabel('Time (s)'); ylabel('Angular Rate (RPM - in Intermediate Frame)');
            legend({'$\hat{p}_3$','$\hat{q}_3$'},'Location','Best','Interpreter','latex');
            if p.Results.showtitle
                title(['Case: ' sim.name]);
            end
            if p.Results.savefigs
                export_fig(['products\images\' sim.name '\rotspeedIF.png'],'-png','-transparent','-m3');
            end
            end
            cd(dirnow);
        end
                     
        function makeInputFile(varargin)
            % todo make this
        end
        
        function sim = loadsim(infn,datapath)
            if nargin < 2
                infnc = char(infn);
                if strcmp(infnc(end-1:end),'.m')
                    infn = infnc(1:end-2);
                end
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