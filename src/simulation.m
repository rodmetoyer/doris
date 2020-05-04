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
                hobj.name = runname;
                hobj.timestep = tstep;
                hobj.duration = totalSimTime;
                
                %% Make objects
                % fluid
                hobj.fld = fluid(fluidtype); % No arguments to fluid gives the obj water properties
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
                af2 = airfoil(airfoiltype2);
                % blade sections
                bs1 = bladesection(secChord1,secWidth1,af1);
                bs2 = bladesection(secChord2,secWidth2,af2);
                % Make a blade comprised of the same section.
                % Rotor 1 blades
                for i=1:1:numSections1
                    section1(i) = bs1;
                end
                for i=1:1:numBlades1
                    bld1(i) = blade(section1,bladeMass1,twist1);
                end
                % Rotor 2 blades need to twist in the opposite direction
                for i=1:1:numSections2
                    section2(i) = bs2;
                end
                for i=1:1:numBlades2
                    bld2(i) = blade(section2,bladeMass2,twist2);
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
                vbod.setLength(vblength);
                vbod.setRadius(vbradius);
                % Make a vehicle
                rotPoints = [rot1point,rot2point];
                hobj.vhcl = vehicle();
                hobj.vhcl.init(vbod,[r1,r2],rotPoints,vbcentermass,vbtetherpoint,vbbuoypoint);
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
                % add tether
                tthr = tether(tnnodes,tnodlocs,tspring,tdamp,tunstrch);
                hobj.addTether(tthr);
                % add generator
                gen = generator(gmconst,gflux,grarm,gkvisc,gmass); % todo size generator constant so that torque is reasonable
                hobj.vhcl.addGenerator(gen);
                % Compute the mass matrix
                hobj.vhcl.computeMstar;
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
               
    end % methods
    
    methods(Static)
        function makeMovie(infn, outfn, varargin)
            % makeMovie  Static method that makes a movie out of a data file
            %   ARGS:
            %       infn - name of the data file
            %       outfn - name for the movie file
            %       varargin - name,value pair to control the movie
            if nargin < 2
                outfn = infn;
            end

            moviefile = ['products\videos\' outfn '.avi'];
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
            Ov_ao_O = dat(:,11:13);
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
            tstep = mean(diff(dat(:,1))); % seconds per frame            
            framerate = 24; % target framerate in frames per second
            sPerFrame = 1/framerate;
            nskips = round(sPerFrame/tstep);
            framerate = 1/(nskips*tstep); % Actual frame rate
            nframes = floor(nsamps/nskips);
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
                Ov_tpo_O = O_C_A*Ov_ao_O(smp,:).' + O_C_A*cross(O_omg_A_A(smp,:).',sim.vhcl.tetherpoint);
                tetherforce = sim.thr.computeTension(r_tpo_O,Ov_tpo_O);
                str = ['Tether tension: ' num2str(norm(tetherforce),'%10.02f') 'N'];
                
                thclr = round(interp1([0,cmaxtensionvalue],[1,100],min(cmaxtensionvalue,norm(tetherforce))));
                plot3(ax,[0 r_tpo_O(1)],[0 r_tpo_O(2)],[0 r_tpo_O(3)],'--','LineWidth',1.0,'Color',cmap(thclr,:));
                hold on
                % plot the body
                plot3(ax,[r_epo_O(1) r_edo_O(1)],[r_epo_O(2) r_edo_O(2)],[r_epo_O(3) r_edo_O(3)],'k','LineWidth',2.0);
                % plot each blade in rotors
                color = ["r","b"];
                for jj = 1:1:numel(sim.vhcl.rotors)
                    r_pa_A = sim.vhcl.rotorLocs(:,jj);
                    r_pa_O = O_C_A*r_pa_A;
                    r_po_O = r_ao_O(smp,:).' + r_pa_O;
                    P_C_A = [cxi3(jj) sxi3(jj) 0; -sxi3(jj) cxi3(jj) 0; 0 0 1];
                    A_C_P = transpose(P_C_A);
                    for ii=1:1:numel(sim.vhcl.rotors(jj).blades)
                        r_tipp_bx = [0;sim.vhcl.rotors(jj).blades(ii).length;0];
                        r_tipp_O = O_C_A*A_C_P*sim.vhcl.rotors(jj).P_C_bx(:,:,ii)*r_tipp_bx;
                        plot3(ax,[r_po_O(1) r_po_O(1)+r_tipp_O(1)],...
                            [r_po_O(2) r_po_O(2)+r_tipp_O(2)],...
                            [r_po_O(3) r_po_O(3)+r_tipp_O(3)],color(jj),'LineWidth',2.0);
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
                title(['\fontsize{20}RPM_R_E_L = ' num2str(((p3(smp)-q3(smp))/(2*pi)*60),'%5.2f'),...
                    '  |  U_\infty = ' num2str(norm(sim.fld.velocity),'%5.2f'),...
                    '  |  Time = ' num2str(dat(smp,1),'%5.2f'),...
                    '  |  Relative Density = ' num2str(sim.vhcl.body.relDensity,'%2.1f')],'FontSize',12);
                hold off
                text(0,0,-1,str,'Fontsize',12);
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