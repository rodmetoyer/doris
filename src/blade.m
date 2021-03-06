classdef blade < handle
    % A blade in a rotor.
% vehicle <aggregate-- rotor <compose-- blade <compose-- bladesection
    % --generalize> airfoil
    
    properties
        % Could be public for future aeroelasticity functionality.
%         centermass  % 3x1 Location of the center of mass in the blade frame
%         sectLocs % 3xn Location of sections expressed in the blade frame
        % todo(rodney) need to unorder these. Use ids or something.
    end % end public properties
    properties (SetAccess = private)
        centermass   % 3x1 Location of the center of mass in the blade frame        
        sections     % 1xn(n=numsects) vector of bladesections
        sectLocs     % 3xn Location of sections expressed in the blade frame
        sectOrnts    % 3xn vector of section orientation angles in the blade frame IN RADIANS 
        numsects     % Number of sections that comprise the blade
        length       % Length of the blade (m)
        mass         % Mass of the blade (kg)
        inertia      % Intertia tensor of the blade
        rootLoc      % Location of the blade root in the blade frame
        tipLoc       % Location of the blade tip in the blade frame
        orientation  % Orientation of the blade frame w.r.t. the rotor frame
        % todo(rodney) Do we need this? meanChord  % Mean chord of the blade (m)
        bladeforce   % The force on the blade not due to blade sections expressed in the blade frame
        blademoment  % The moment on the blade not due to blade sections expressed in the blade frame
        pitch        % Pitch of the blade todo implement
        % id         % Unique integer identifier - maybe don't need
    end % end private properties
    properties (Dependent) % Calculated only when asked for
        b_C_a        % 3x3xn array tranformation matrix from the section frame into the blade frame
    end % end dependent properties
    
    methods
        %% Constructor
        function hobj = blade(section,mass,twist,ls,chord,numsects)
            % INPUTS:
                % section = either a vector of blade section objects
                    % (manual construction method) or a 1xn vector of airfoil
                    % IDs where n=1 for constant airfoil or n=size(length) for
                    % varying airfoil.
                % mass
                % twist
                % length = either a 1xn array of section distances or a
                    % scalar value for the blade length (m)
                % chord = either a 1xn array of chords or a scalar constant
                    % chord (m)
                % nsects = optional number of sections (only used if
                    % length is scalar.
            if nargin == 0
                % remember to init
                % warning('Make sure to init blades.');
            elseif nargin == 3
                % This is manual blade creation where a vector of
                    % bladesection objects is passed in
                if ~isvalid(section)
                    error('blade: To create a blade manually you need to pass an object vector of bladesections.');
                end
                hobj.rootLoc = 0; % Note manual construciton assumes blade root location of 0.
                ls = 0;
                %mass = 0;
                secLocs = NaN(3,numel(section));
                for i=1:1:numel(section)
                    secLocs(:,i) = [0;ls + section(i).width/2;0];
                    ls = ls + section(i).width;
                    %mass = mass + section(i).mass;
                end
                hobj.length = ls;
                hobj.tipLoc = ls; % See note above.
                hobj.sections = section;
                hobj.numsects = numel(hobj.sections);
                hobj.sectLocs = secLocs;
                hobj.centermass = [0;ls/2;0];
                hobj.mass = mass;
                hobj.inertia = thinrod(mass,ls);
                % todo(rodney) add in the blade.orientation with is
                % essentially pitch plus some pre-cone and lead-lag
                if isstruct(twist)
                    twist = hobj.computeTwist(twist.AoAopt_deg,twist.numBlades,twist.bladeDZfrac);
                end
                hobj.sectOrnts = [zeros(size(twist));twist;zeros(size(twist))];
                % todo(rodney) add other methods for computation of inertia tensor
            else
                % This is the automatic blade construction method. The idea
                % is that all the blade parameters can be passed down from
                % the rotor creation method so that one could make blades
                % and blade sections automatically when making rotors.
                error('blade: Only manual (section construction) method of blade construction is currently supported');
                % todo(rodney) make these automatic methods
                    % Automatic constant chord and airfoil - makes
                    % numsecs bladesection objects to cover length
                        % section - scalar airfoil
                        % length - scalar real
                        % chord - scalar real
                        % numsecs - scalar real
                    % Automatic varying chord and/or airfoil - either
                    % section or chord must be vector (both may be, but
                    % must be same size).
                        % section - scalar or vector of airfoil
                        % length - vector real
                        % chord - scalar or vector real    
            end            
        end
        
        % Initialization
        function init(hobj,scrd,af,ltot,numsecs,mass,twist)
            if isstruct(twist)
                hobj.rootLoc = twist.bladeDZfrac*ltot;
            else
                warning('Blade dead zone doesn"t work with numerical twist array');
                hobj.rootLoc = 0;
            end
            ls = hobj.rootLoc;
            swdth = (ltot-hobj.rootLoc)/numsecs;
            %mass = 0;
            secLocs = NaN(3,numsecs);
            for i=1:1:numsecs
                sec(i) = bladesection(scrd,swdth,af);
                secLocs(:,i) = [0;ls + sec(i).width/2;0];
                ls = ls + sec(i).width;
                %mass = mass + section(i).mass;
            end
            
            hobj.length = ls;
            hobj.tipLoc = ls; % See note above.
            hobj.sections = sec;
            hobj.numsects = numel(hobj.sections);
            hobj.sectLocs = secLocs;
            hobj.centermass = [0;ls/2;0];
            hobj.mass = mass;
            hobj.inertia = thinrod(mass,ls);
            % todo(rodney) add in the blade.orientation with is
            % essentially pitch plus some pre-cone and lead-lag
            if isstruct(twist)
                twist = hobj.computeTwist(twist.AoAopt_deg,twist.numBlades,twist.bladeDZfrac);
            end
            hobj.sectOrnts = [zeros(size(twist));twist;zeros(size(twist))];
        end
        
        %% Other class methods
        function bladeforce = computeBladeforce(hobj,vr)
           % Computes the blade force assuming uniform flow and cylindrical
           % blade. Intent is to add more complex methods in the future.
           % INPUTS: object handle and the relative velocit vector
           
           % todo(rodney) write force computation code which will be an
           % adjustment to the blade element method to account for finite
           % blade length.
           bladeforce = vr*0;
           hobj.bladeforce = bladeforce;
        end
        function blademoment = computeBlademoment(hobj,vr)
           % Computes the blade force assuming uniform flow and cylindrical
           % blade. Intent is to add more complex methods in the future.
           % INPUTS: object handle and the relative velocity vector
           
           % todo(rodney) write moment computation code (see note above)
           blademoment = vr*0;
           hobj.blademoment = blademoment;
        end
        
        function twist = computeTwist(hobj,aoaopt,numblds,bladeDZfrac)
            rtos = 2.0;    % This is the ratio of rotor radius to disturbed fluid
            % stream length for computing optimal TSR. A good rule-of-thumb value is
            % 2.0 (Ragheb and Ragheb 2011).
            % Compute optimal TSR using method described in (Ragheb and Ragheb 2011)
            TSRopt = 2*pi/numblds*rtos;
            % Compute Twist using method described in Ch. 5 of (Gasch and Twele 2012)
            %bladeDZfrac = 0.0; % todo(rodney) un-hardcode this
            locs = linspace(hobj.length*bladeDZfrac,hobj.length,numel(hobj.sections));
            twist = atan(2/3*hobj.length/TSRopt*1./locs)-aoaopt*pi/180;
        end
        
        function reverseTwist(hobj)
            % Reverses the blade twist to make a contrarotating rotor by
            % mirroring the location of the blade sections.
            % I'm commenting out the warning because there are assumptions
            % baked in everywhere. Anyone using this code at this point
            % recognizes that it is only valid for a dual-rotor coaxial
            % turbine. 
            % warning('BLADE:construction','reverseTwist assumes that the twist is only about the blade-j axis. Just mirroring the blade by moving sections.');
            for i=1:1:numel(hobj)
                hobj(i).sectLocs = -hobj(i).sectLocs;
            end
        end
        
        function adjustPitch(hobj,p)
            hobj.sectOrnts(2,:) = hobj.sectOrnts(2,:)+p;
        end
        
        %% Setters
        function set.centermass(hobj,cm)
            if numel(cm)<3
                cm = [0;cm;0];
            end
            hobj.centermass = cm;
        end
        
        function set.sectLocs(hobj,locs)
           if ~isempty(hobj.sectLocs) && any(size(locs) ~= size(hobj.sectLocs))
               error('blade: You cannot change number of section locations');
           end
           hobj.sectLocs = locs;
        end
        
        %% Getters
        function m = get.b_C_a(hobj)
            % sectOrnts is 3xn orientation angles
            % todo need to document the specific sequence.
            % As of 17JUN2019 we assume that the rotation is only about the
            % j_b = j_a axis.
            % REMEMBER: This matrix is analogous to the O_C_B matrix using
            % the Mazzoleni convention. In other words, when you go from
            % the section frame to the blade frame you rotate th about j_a,
            % which is a pitch-up maneuver of the section/
            % To go from the blade to the section you rotate -th about j_b.
            temp = size(hobj.sectOrnts);
            m = NaN(3,3,temp(2));
            for i=1:1:temp(2)
                th = hobj.sectOrnts(2,i);
                m(:,:,i) = [cos(th) 0 -sin(th);...
                            0 1 0;...
                            sin(th) 0 cos(th)];
            end            
        end
        
    end
    
end
% Functions outside of classdef are just local functions
function t = thinrod(l,m)
    t = [1/3*m*l^2 0 0; 0 0 0; 0 0 1/3*m*l^2];
end