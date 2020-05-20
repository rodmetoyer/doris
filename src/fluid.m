classdef fluid < handle
    % Fluid medium
    properties (SetAccess = private)
        density     % Fluid density (kg/m^3)
        dynVisc     % Dynamic viscosity (Pa*s)
        kinVisc     % Kinematic viscocity (m^2/s)
        temp        % Temperature (degC)
        pressure    % Pressure (Pa)
        type        % uint identifier | see typeName get method for list of types
        flowtype    % the type of flow, always uniform (1 - steady, 2 - sinusoidal, 3 - ramped, 4 - disturbed, 5 - waves)
        flowparms   % nx1 array of flow parameters whose number and function is based on the flowtype
        meanvel     % 3x1 mean velocity in the inertial frame
        gravity     % The acceleration due to gravity where the fluid is %todo should this be on an environment class or something?
    end
    
    properties (Dependent)
        typeName
    end
    
    properties
        % Need a way to make velocity a funciton of time an position in the
        % flow field. For now, every point in the flow has the same
        % velocity. 
        velocity    % 3x1 The fluid velocity in the inertial frame
    end
    
    methods
        % Constructor
        function hobj = fluid(density,dynVisc,kinVisc,temp,pressure,velocity)
            % todo make this varargin
%             if nargin == 0
%                 % make sure to init or everything will break
%                 % warning('Make sure to init the fluid');
            if nargin == 1
                if ischar(density)
                    switch lower(density)
                        case 'water'
                            hobj.type = 1;
                        case 'air'
                            hobj.type = 2;
                        otherwise
                            error('Unknown fluid type');
                    end
                else
                    hobj.type = density;
                end                            
                hobj.density = 997;
                hobj.dynVisc = 9.482*10^-4;
                hobj.kinVisc = 9.504*10^-7;
                hobj.temp = 23;
                hobj.pressure = 103421;
                hobj.velocity = [0;0;0];
            elseif nargin > 1
                hobj.density = density;
                hobj.dynVisc = dynVisc;
                hobj.kinVisc = kinVisc;
                hobj.temp = temp;
                hobj.pressure = pressure;
                hobj.velocity = velocity;
            end
            hobj.meanvel = hobj.velocity;
            hobj.gravity = 9.81;
        end
        
        % Initialization
        function init(hobj,type)
                switch type
                    case 'water'
                        hobj.density = 997;
                        hobj.dynVisc = 9.482*10^-4;
                        hobj.kinVisc = 9.504*10^-7;
                        hobj.temp = 23;
                        hobj.pressure = 103421;
                        hobj.velocity = [0;0;0];
                        hobj.type = 1;
                    case 'air'
                        hobj.density = 1.204;
                        hobj.dynVisc = 1.825*10^-5;
                        hobj.kinVisc = 1.516*10^-5;
                        hobj.temp = 20;
                        hobj.pressure = 103421;
                        hobj.velocity = [0;0;0];
                        hobj.type = 2;
                    otherwise
                        error('Unknown fluid type');
                end
        end
        
        % Other class methods
        function rampvelocity(hobj,t)
            error('obsolete method');
            hobj.velocity = [0.6/(1+exp(-2.0*(t-4)));0;0]; %todo(rodney) parameterize this
        end
        
        function setFlowType(hobj,ft,p)
            hobj.flowparms = p;
            if isnumeric(ft)
                hobj.flowtype = ft;
            elseif strcmp(ft,'steady')
                hobj.flowtype = 1;
            elseif strcmp(ft,'sinusoidal')
                hobj.flowtype = 2;
            elseif strcmp(ft,'ramped')
                hobj.flowtype = 3;
            elseif strcmp(ft,'disturbed')
                hobj.flowtype = 4;
            else
                error('Trying to set fluid type to unknown value');
            end
        end
        
        function updateVelocity(hobj,t)
            switch hobj.flowtype
                case 1 % steady
                    hobj.velocity = hobj.meanvel;
                    v = hobj.velocity;
                case 2 % sinusoidal
                    wave = hobj.flowparms(1)*[sin(2*pi*hobj.flowparms(2)*t+hobj.flowparms(3));cos(2*pi*hobj.flowparms(2)*t+hobj.flowparms(3))];
                    hobj.velocity = hobj.meanvel + [wave(1);0;wave(2)];
                case 3 % ramped
                    % hobj.flowparms = [rampspeed,starttime,0];
                    hobj.velocity = hobj.meanvel./(1+exp(-hobj.flowparms(1)*(t-hobj.flowparms(2))));
                case 4 % disturbed
                    % hobj.flowparms = [rampspeed,starttime,duration,maximum];
                    disturbance = hobj.flowparms(4)/(1+exp(-hobj.flowparms(1)*(t-hobj.flowparms(2))))...
                        - hobj.flowparms(4)/(1+exp(-hobj.flowparms(1)*(t-hobj.flowparms(2)-hobj.flowparms(3))));
                    hobj.velocity = hobj.meanvel + [0;disturbance;0];
                otherwise
                    error('Unknown fluid type');
            end
        end
        
        function setMeanVelocity(hobj,v)
            hobj.meanvel = v;
        end
        
        function re = getRotationalRe(hobj,om,r)
            re = hobj.density*om*r^2/hobj.dynVisc;
        end
        
        % setters
        function set.velocity(hobj,v)
            if numel(v) ~= 3
                error('fluid: Velocity must be 3x1 vector');
            end
            [m,~] = size(v);
            if m < 3
                v = v.';
            end
            hobj.velocity = v;
        end
        % getters
        function n = get.typeName(hobj)
            switch hobj.type
                case 1
                    n = 'water';
                case 2
                    n = 'air';
                otherwise
                    n = 'unknown';
            end
        end
    end
end

