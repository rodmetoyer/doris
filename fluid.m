classdef fluid < handle
    % Fluid medium
    % Construct with no arguments for water at 23degC and ~103kPa
    properties (SetAccess = private)
        density     % Fluid density (kg/m^3)
        dynVisc     % Dynamic viscosity (Pa*s)
        kinVisc     % Kinematic viscocity (m^2/s)
        temp        % Temperature (degC)
        pressure    % Pressure (Pa)        
    end
    
    properties
        velocity    % 3x1 The fluid velocity in the inertial frame
    end
    
    methods
        function hobj = fluid(density,dynVisc,kinVisc,temp,pressure,velocity)
            if nargin < 1
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
    end
end

