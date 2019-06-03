classdef fluid < handle
    % Fluid medium
    % Construct with no arguments for water at 23degC and ~103kPa
    properties (SetAccess = private)
        density     % Fluid density (kg/m^3)
        dynVisc     % Dynamic viscosity (Pa*s)
        kinVisc     % Kinematic viscocity (m^2/s)
        temp        % Temperature (degC)
        pressure    % Pressure (Pa)
        type        % uint identifier | see typeName get method for list of types
    end
    
    properties (Dependent)
        typeName
    end
    
    properties
        velocity    % 3x1 The fluid velocity in the inertial frame
    end
    
    methods
        % Constructor
        function hobj = fluid(density,dynVisc,kinVisc,temp,pressure,velocity)
            if nargin == 0
                % make sure to init
            elseif nargin == 1
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
            else
                hobj.density = density;
                hobj.dynVisc = dynVisc;
                hobj.kinVisc = kinVisc;
                hobj.temp = temp;
                hobj.pressure = pressure;
                hobj.velocity = velocity;
            end
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
            hobj.velocity = [0.6/(1+exp(-2.0*(t-4)));0;0]; %todo(rodney) parameterize this
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

