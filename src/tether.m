classdef tether < handle
    properties (SetAccess = private)
        numnodes  % number of internal nodes - tether terminates with links
        nodelocs  % 3xnumnodes array of node locations in inertial space
        nodevels  % 3xnumnodes array of node velocities in inertial space
        stiffness % 1xnumnodes+1 array of spring stiffnesses
        damping   % 1xnumnodes+1 array of damping constants
        reldens   % 1xnumnodes array of relative density of the nodes
        mass      % 1xnumnodes array of node masses
        uslength  % 1xnumnodes+1 unstretched length of the links
        radius    % 1xnumnodes+1 radius of the links
        tension   % 3x2 tension at the ends                
    end
    properties (SetAccess = private)
        % The endpoints are not a part of the tether model, but they are
        % tether data. You should update using setEndpoints or setPointA
        % and setPointB
        endpnts   % 3x2 location of the endpoints of the tether
        endvels % 3x2 velocity of the endpoints
    end
    properties
        name    % name of the tether
    end
    
    methods
        
        function hobj = tether(n,locs,k,c,usl)
            if nargin > 0
            hobj.numnodes = n;
            hobj.nodelocs = locs;
            hobj.nodevels = zeros(size(locs));
            hobj.stiffness = k;
            hobj.damping = c;
            hobj.uslength = usl;
            end
        end % constructor
        
        function [xd] = computeTension(hobj,f)
            % Input
                % f is a fluid object so we know the relative velocity
            % Output
                % xd = 1x6*nunnodes vector of node states
            
            % x is a vector of tether node states
            x(1:3*hobj.numnodes) = hobj.nodelocs;
            x(3*hobj.numnodes+1:6*hobj.numnodes) = hobj.nodevels;
            xd = NaN(1,6*hobj.numnodes);
            if hobj.numnodes < 1
                % Note that, for the single-length tether, we currently
                % neglect the hydrodyamic loads on the tether.
                % todo add hydro loads for a single length if you want to.
                r = hobj.endpnts(:,2)-hobj.endpnts(:,1);
                v = hobj.endvels(:,2)-hobj.endvels(:,1);
                usleng = hobj.uslength;
                currentlength = norm(r);
                unitvec = r/currentlength; % From anchor to vehicle attachment point
                stretch = currentlength-usleng; % If it moves from the origin there is a restoring force
                stretchd = dot(r,v)/currentlength;
                if currentlength < 0 % I don't see how this is possible. Norm should always be positive.
                    unitvec = [0;0;0];
                    stretchd = 0;
                end
                Fmag = 0;
                if stretch > 0
                    Fmag = -stretch*hobj.stiffness;
                    Fmag = Fmag - stretchd*hobj.damping; % changed to make it damp both out and in as long as it is taut. todo(rodney) investigate what is most accurate
                end
                t = [Fmag*unitvec(1);Fmag*unitvec(2);Fmag*unitvec(3)];
                hobj.tension(:,1) = -t;
                hobj.tension(:,2) = t;
            else % we have nodes                               
                % loop over all of the nodes
                for i=1:1:hobj.numnodes
                    % first, populate the speeds
                    xd(3*i-2:3*i) = hobj.nodevels(:,i);                     
                    % now compute the internal loads
                    if i==1                        
                        rminus = hobj.endpnts(:,1);
                        vminus = hobj.endvels(:,1);
                        if hobj.numnodes == 1
                            rplus = hobj.endpnts(:,2);
                            vplus = hobj.endvels(:,2);
                        else
                            rplus = x(3*(i+1)-2:3*(i+1)).';
                            vplus = x(3*hobj.numnodes+3*(i+1)-2:3*hobj.numnodes+3*(i+1)).';
                        end
                    elseif i==hobj.numnodes
                        rplus = hobj.endpnts(:,2);
                        rminus = x(3*(i-1)-2:3*(i-1)).';
                        vminus = x(3*hobj.numnodes+3*(i-1)-2:3*hobj.numnodes+3*(i-1)).';
                        vplus = hobj.endvels(:,2);
                    else
                        rminus = x(3*(i-1)-2:3*(i-1)).';
                        rplus = x(3*(i+1)-2:3*(i+1)).';
                        vminus = x(3*hobj.numnodes+3*(i-1)-2:3*hobj.numnodes+3*(i-1)).';
                        vplus = x(3*hobj.numnodes+3*(i+1)-2:3*hobj.numnodes+3*(i+1)).';
                    end
                    ri = x(3*i-2:3*i).';
                    vi = x(3*hobj.numnodes+3*i-2:3*hobj.numnodes+3*i).';
                    di = norm(rminus-ri);
                    diplus = norm(ri-rplus);
                    ei = (rminus-ri)/di;
                    eiplus = (ri-rplus)/diplus;
                    did = dot((vminus-vi),ei)*ei;
                    didplus = dot((vi-vplus),eiplus)*eiplus;
                    fci = hobj.damping(i)*did;
                    fciplus = hobj.damping(i+1)*didplus;
                    stretchi = max(0,di-hobj.uslength(i));
                    stretchplus = max(0,diplus-hobj.uslength(i+1));
                    fsi = hobj.stiffness(i)*stretchi*ei;
                    fsiplus = hobj.stiffness(i+1)*stretchplus*eiplus;
                    
                    
                    % now compute the external loads
                    % Buoyancy - this is wrong. No one is every going to
                    % question it, but it is wrong. Buoyancy only acts
                    % normal to the discrete element unless it is an end
                    % element.
                    % todo replace buoyancy calc with more realistic model.
                    Fb = (1-1/hobj.reldens(i))*hobj.mass(i)*[0;0;-f.gravity];
                    % Relative Velocity vector
                    vreli = f.velocity-(vi/2+vminus/2);
                    vrelplus = f.velocity-(vplus/2+vi/2);
                    uaxi = dot(vreli,ei)*ei;
                    uaxplus = dot(vrelplus,eiplus)*eiplus;
                    unormi = vreli-uaxi;
                    unormplus = vrelplus-uaxplus;
                    % assuming an axial force coefficint of 0.1 - todo expose this
                    q = 0.5*f.density*hobj.radius*diplus; % eqns are force per unit length, so including diplus in q
                    Fax = (0.1*q*uaxi + 0.1*q*uaxplus);
                    % assuming a normal force coefficint of 1.2 - todo expose this
                    Fnorm = (1.2*q*unormi + 1.2*q*unormplus);
                    Fext = Fnorm + Fax + Fb;
                    % finally, populate the accelerations
                    allforce = Fext + fsi + fci - fsiplus - fciplus;
                    xd(3*hobj.numnodes+3*i-2:3*hobj.numnodes+3*i) = allforce/hobj.mass(i);
                    % populate the tension
                    if i==1
                        hobj.tension(:,1) = -fsi - fci;
                        if hobj.numnodes == 1
                            hobj.tension(:,2) = fsiplus + fciplus;
                        end
                    elseif i==hobj.numnodes
                        hobj.tension(:,2) = fsiplus + fciplus;
                    end
                end % loop over nodes
            end % end tether with nodes block
        end % computeTension
        
        function init(hobj,nnodes,endpoints,f,reldens,radius,stf,dmp,endmass)
            % init function assumes you want a tether with total
            % unstretched length being a straight line between the two
            % points in the endpoints argument
            % nnodes = scalar number of nodes
            % endpoints = 2x3 initial endpoints
            % f = either a fluid object or the density of the fluid that the tether will be in 
            % reldens = scalar relative density (specific gravity if in H2O)
            % radius = scalar radius
            % dmp = either 1xnnodes+1 array of damping constants or a scalar damping ratio for equal links 
            % sft = either 1xnnodes+1 or single value of stiffness
            hobj.radius = radius;
            hobj.numnodes = nnodes;
            hobj.reldens = ones(1,nnodes)*reldens;
            nlinks = nnodes + 1;
            totlength = norm(endpoints(:,2)-endpoints(:,1));
            hobj.uslength = ones(1,nlinks)*totlength/nlinks;
            if numel(stf) > 1
                if numel(stf) ~= nnodes+1
                    error('Bad stiffness array size');
                else
                    hobj.stiffness = stf;
                end
            else
                % one value total tether stiffness
                hobj.stiffness = ones(1,nlinks)*nlinks*stf;
            end
            if numel(dmp) > 1
                if numel(dmp) ~= nnodes+1
                    error('Bad damping ratio array size');
                else
                    hobj.damping = dmp;
                end
            else
                ct = dmp*2*sqrt(stf*endmass);
                hobj.damping = ones(1,nlinks)*nlinks*ct;
            end
            % assuming a cylindrical tether, the mass is computed from the
            % relative density and the volume
            if isa(f,'fluid')
                fdens = f.density;
            else
                fdens = f;
            end
            hobj.mass = ones(1,nnodes)*fdens*reldens*totlength*pi*radius^2/nnodes;
            hobj.tension = zeros(3,2);
            direction = (endpoints(:,2)-endpoints(:,1))/totlength;
            lgnth = hobj.uslength(1);
            for i=1:1:hobj.numnodes
                hobj.nodelocs(:,i) = endpoints(:,1) + lgnth*direction;
                hobj.nodevels(:,i) = [0;0;0];
                lgnth = lgnth + hobj.uslength(i);
            end            
        end
        
        function initinternals(hobj,f,reldens,radius,endpnts)
            % init method that works with a vehicle
                        
            hobj.radius = radius;
            hobj.reldens = ones(1,hobj.numnodes)*reldens;
            nlinks = hobj.numnodes + 1;
            hobj.endpnts = endpnts;
            hobj.uslength = ones(1,nlinks)*hobj.uslength/nlinks;

            if numel(hobj.stiffness) > 1
                if numel(hobj.stiffness) ~= hobj.numnodes+1
                    error('Bad stiffness array size');
                end
            else
                % one value total tether stiffness
                hobj.stiffness = ones(1,nlinks)*nlinks*hobj.stiffness;
            end
            if numel(hobj.damping) > 1
                if numel(hobj.damping) ~= hobj.numnodes+1
                    error('Bad damping ratio array size');
                end
            else
                hobj.damping = ones(1,nlinks)*nlinks*hobj.damping;
            end
            % assuming a cylindrical tether, the mass is computed from the
            % relative density and the volume
            if isa(f,'fluid')
                fdens = f.density;
            else
                fdens = f;
            end
            uslength = sum(hobj.uslength,'all');
            hobj.mass = ones(1,hobj.numnodes)*fdens*reldens*uslength*pi*radius^2/hobj.numnodes;
            hobj.tension = zeros(3,2);           
        end % initinternals
        
        function m = getTotalMass(hobj)
            m = sum(hobj.mass);
        end
        
        function setRelativeDensity(hobj,rd)
            hobj.reldens = rd;
        end
        
        function setRadius(hobj,r)
            hobj.radius = r;
        end
        
        function setMass(hobj,m)
            hobj.mass = m;
        end
        
        function setNodeVelocity(hobj,v)
            % todo need to validate on the way in
            v = reshape(v,[3 hobj.numnodes]);
            hobj.nodevels = v;
        end
        function setNodePosition(hobj,p)
            % todo need to validate on the way in
            % Turn into a 3xnumnodes
            p = reshape(p,[3 hobj.numnodes]);
            hobj.nodelocs = p;
        end
        function setEndpoints(hobj,pts)
            if numel(pts) ~= 6
                error('setEndpoints is for setting the position of both endpoints at once');
            end
            hobj.endpnts = pts;
        end
        function setLocationA(hobj,pt)
            hobj.endpnts(:,1) = pt;
        end
        function setLocationB(hobj,pt)
            hobj.endpnts(:,2) = pt;
        end
        function setVelocityA(hobj,pt)
            hobj.endvels(:,1) = pt;
        end
        function setVelocityB(hobj,pt)
            hobj.endvels(:,2) = pt;
        end
        function hfig = showme(hobj,axlims,vis)
            if nargin < 2
                axlims = [-inf inf -inf inf -inf inf];
                vis = 'on';
            end
            hfig = figure('visible',vis);
            ax1 = axes('Parent',hfig);
            hold(ax1,'on');
            if ~isempty(hobj.nodelocs) && ~isempty(hobj.endpnts)
                % Has ends and nodes
                plot3(ax1,hobj.nodelocs(1,:),hobj.nodelocs(2,:),hobj.nodelocs(3,:),'*-b');
                plot3(ax1,[hobj.endpnts(1,1),hobj.nodelocs(1,1)],[hobj.endpnts(2,1),hobj.nodelocs(2,1)],[hobj.endpnts(3,1),hobj.nodelocs(3,1)],'-b');
                plot3(ax1,hobj.endpnts(1,1),hobj.endpnts(2,1),hobj.endpnts(3,1),'*g');
                plot3(ax1,hobj.endpnts(1,2),hobj.endpnts(2,2),hobj.endpnts(3,2),'*r');
                plot3(ax1,[hobj.nodelocs(1,end),hobj.endpnts(1,2)],[hobj.nodelocs(2,end),hobj.endpnts(2,2)],[hobj.nodelocs(3,end),hobj.endpnts(3,2)],'-b');
            elseif isempty(hobj.nodelocs) && isempty(hobj.endpnts)
                warning('Nothing to plot');
            elseif ~isempty(hobj.nodelocs) && isempty(hobj.endpnts)
                % has nodes only
                plot3(ax1,hobj.nodelocs(1,:),hobj.nodelocs(2,:),hobj.nodelocs(3,:),'*-b');
            elseif isempty(hobj.nodelocs) && ~isempty(hobj.endpnts)
                % has ends only
                plot3(ax1,hobj.endpnts(1,1),hobj.endpnts(2,1),hobj.endpnts(3,1),'*g');
                plot3(ax1,hobj.endpnts(1,2),hobj.endpnts(2,2),hobj.endpnts(3,2),'*r');
                plot3(ax1,[hobj.endpnts(1,1),hobj.endpnts(1,2)],[hobj.endpnts(2,1),hobj.endpnts(2,2)],[hobj.endpnts(3,1),hobj.endpnts(3,2)],'-b');
            end
            
            hold(ax1,'off');
            axis equal
            axis(axlims);
            view(-30,10);
            xlabel('x'); ylabel('y'); zlabel('z');
        end
        
        function [a,ad] = getAnchorPoint(hobj,t)
            % a is the location of the anchor point
            % ad is the velocity of the anchor point
            % todo future functionality make this time dependent function
            % to simulate platform on waves and whatnot
            a = [0;0;0];
            ad = [0;0;0];
        end
        
        function makeMovie(hobj, varargin)
            % makeMovie  Static method that makes a movie out of a data file
            %   ARGS:
            %       infn - name of the data file
            %       varargin - name,value pair to control the movie
            defaultOutfile = hobj.name;
            defaultFramerate = 24;
            defaultSpeedfactor = 1;
            defaultHide = false;
            p = inputParser;
            %validateOutput = @(x) isa(x,'function_handle');
            addParameter(p,'outfile',defaultOutfile,@ischar);
            addParameter(p,'framerate',defaultFramerate,@isnumeric);
            addParameter(p,'speedfactor',defaultSpeedfactor,@isnumeric);
            addParameter(p,'times',[],@isnumeric);
            addParameter(p,'Adata',[],@isnumeric);
            addParameter(p,'Bdata',[],@isnumeric);
            addParameter(p,'tetherdata',[],@isnumeric);
            addParameter(p,'fluid',[]);
            addParameter(p,'hide',defaultHide);
            parse(p,varargin{:});           

            moviefile = ['output\videos\' p.Results.outfile '.avi'];
            t = p.Results.times;
            Adat = p.Results.Adata;
            Bdat = p.Results.Bdata;
            tetherdat = p.Results.tetherdata;
            if isempty(tetherdat)
                tetherdat = NaN(length(t),3);
            end
            vis = 'on';
            if p.Results.hide
                vis = 'off';
            end
            
            % make the vectors and everything            
            nsamps = length(t);
            tstep = mean(diff(t)); % seconds per sample            
            framerate = p.Results.framerate; % target framerate in frames per second
            speedfactor = p.Results.speedfactor;
            sPerFrame = 1/framerate;
            nskips = round(speedfactor*sPerFrame/tstep);
            if nskips < 1
                error('Not enough resolution in data to achieve target framerate.');
            end
            framerate = round(speedfactor/(nskips*tstep)); % Actual frame rate
            nframes = floor(nsamps/nskips); % Actual number of frames
            maxx = max([max(abs(Adat(:,1))),max(abs(Bdat(:,1))),max(abs(tetherdat(:,1)))]);
            maxy = max([max(abs(Adat(:,2))),max(abs(Bdat(:,2))),max(abs(tetherdat(:,2)))]);
            maxz = max([max(abs(Adat(:,3))),max(abs(Bdat(:,3))),max(abs(tetherdat(:,3)))]);
            if maxx == 0; maxx = inf; end
            if maxy == 0; maxy = inf; end
            if maxz == 0; maxz = inf; end
            axlims = [-maxx maxx -maxy maxy -maxz maxz];
            cmap = colormap(jet(100));
            cmaxtensionvalue = 2000;
            disp('Making movie of tether motion');
            hfig = figure('visible',vis);
            ax1 = axes('Parent',hfig);
            for i = 1:1:nframes
                if mod(i,30) == 0
                    str = ['On frame ' num2str(i) ' of ' num2str(nframes)];
                    disp(str);
                end
                smp = nskips*(i-1)+1;
                fluidVelocity = [0;0;0];
                if ~isempty(p.Results.fluid)
                    % update the fluid velocity
                    %fluidVelocity = ;
                end
                % update the positions
                A = [Adat(smp,1);Adat(smp,2);Adat(smp,3)];
                Ad = [Adat(smp,4);Adat(smp,5);Adat(smp,6)];
                B = [Bdat(smp,1);Bdat(smp,2);Bdat(smp,3)];
                Bd = [Bdat(smp,4);Bdat(smp,5);Bdat(smp,6)];
                % update tether position and locations
                hobj.setNodePosition(tetherdat(smp,1:3*hobj.numnodes));
                hobj.setNodeVelocity(tetherdat(smp,3*hobj.numnodes+1:6*hobj.numnodes));
                hobj.setLocationA(A);
                hobj.setLocationB(B);
                hold(ax1,'on');
                if ~isempty(hobj.nodelocs) && ~isempty(hobj.endpnts)
                    % Has ends and nodes
                    plot3(ax1,hobj.nodelocs(1,:),hobj.nodelocs(2,:),hobj.nodelocs(3,:),'*-b');
                    plot3(ax1,[hobj.endpnts(1,1),hobj.nodelocs(1,1)],[hobj.endpnts(2,1),hobj.nodelocs(2,1)],[hobj.endpnts(3,1),hobj.nodelocs(3,1)],'-b');
                    plot3(ax1,hobj.endpnts(1,1),hobj.endpnts(2,1),hobj.endpnts(3,1),'*g');
                    plot3(ax1,hobj.endpnts(1,2),hobj.endpnts(2,2),hobj.endpnts(3,2),'*r');
                    plot3(ax1,[hobj.nodelocs(1,end),hobj.endpnts(1,2)],[hobj.nodelocs(2,end),hobj.endpnts(2,2)],[hobj.nodelocs(3,end),hobj.endpnts(3,2)],'-b');
                elseif isempty(hobj.nodelocs) && isempty(hobj.endpnts)
                    warning('Nothing to plot');
                elseif ~isempty(hobj.nodelocs) && isempty(hobj.endpnts)
                    % has nodes only
                    plot3(ax1,hobj.nodelocs(1,:),hobj.nodelocs(2,:),hobj.nodelocs(3,:),'*-b');
                elseif isempty(hobj.nodelocs) && ~isempty(hobj.endpnts)
                    % has ends only
                    plot3(ax1,hobj.endpnts(1,1),hobj.endpnts(2,1),hobj.endpnts(3,1),'*g');
                    plot3(ax1,hobj.endpnts(1,2),hobj.endpnts(2,2),hobj.endpnts(3,2),'*r');
                    plot3(ax1,[hobj.endpnts(1,1),hobj.endpnts(1,2)],[hobj.endpnts(2,1),hobj.endpnts(2,2)],[hobj.endpnts(3,1),hobj.endpnts(3,2)],'-b');
                end
    
                % plot the tether
                %hfig = hobj.showme(axlims,'off');
%                 hfig = 
%                 ax = hfig.CurrentAxes;
%                 hold(ax1,'on');
                % plot the fluid velocity vector at the origin
                quiver3(ax1,0,0,0,fluidVelocity(1),fluidVelocity(2),fluidVelocity(3),'Color','c','LineWidth',2.0,'MaxHeadSize',0.5);
                %annotation('textbox',[0.3 0.2 0.5 0.2],'String',{'U_\infty Vector'},'FitBoxToText','on');
                hold(ax1,'off');
                axis equal
                axis(axlims);
                view(-30,10);
                xlabel('x'); ylabel('y'); zlabel('z');
                set(ax1,'DataAspectRatio',[1 1 1]);
                F(i) = getframe(hfig);
                cla(ax1);
%                 close(hfig)
                %clear an;
            end % end loop through data
            % make the movie
            disp('Saving tether motion movie');
            vw = VideoWriter(moviefile);            
            vw.FrameRate = framerate;
            %v.Quality = 100;% v.Width = 800; w.Height = 450;
            open(vw);
            writeVideo(vw,F); close(vw);
        end % end makeMovie
        
    end % methds
end% tether