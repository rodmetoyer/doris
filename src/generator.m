classdef generator < handle
    % generator A generator modeled as separatly excited dc machine.
    % initiate with:
    %   k   - the machine constant
    %   phi - magnetic flux (from pm or excitation coil)
    %   ra  - the armature resistance
    %   c   - the constant for the viscous friction model
    % The generator begins with an infinite load resistance (i.e. no load).
    %
    % To add a load resistance use the setLoadResistance method.
    
    properties (SetAccess = private)
        kmach    % machine constant
        flux     % magnetic flux
        rarm     % total armature resistance
        rload    % load resistance
        kvisc    % damping constant for viscous friction
        mass     % mass of the generator
        rlparams % 3 parameters that control a variable load - empty means constant load, 2 means ramp, 4 means sinusoid [amp,freq,phase,mean] 
    end % end parameters
    
    methods
        % Constructor
        function hobj = generator(k,phi,ra,c,m)
            hobj.kmach = k;
            hobj.flux = phi;
            hobj.rarm = ra;
            hobj.rload = inf; % no load when you first make the generator
            hobj.kvisc = c;
            hobj.mass = m;
        end % generator
        
        function tq = getTorque(hobj,omg)
            k = 0.8; % this is an exponential that determines the shape of viscous friction vs. speed.
            tq = -(hobj.kmach*hobj.flux)^2*omg/(hobj.rarm + hobj.rload) - hobj.kvisc^k*omg;
        end % getTorque
        
        function pwr = getPower(hobj,omg)
            pwr = (hobj.kmach*hobj.flux*omg)^2/(hobj.rarm + hobj.rload);
        end
        
        function i = getArmatureCurrent(hobj,omg)
            i = (hobj.kmach*hobj.flux*omg)/(hobj.rarm + hobj.rload);
        end
        function setMachineConstant(hobj,k)
            hobj.kmach = k;
        end
        function setFlux(hobj,f)
            hobj.flux = f;
        end
        function setViscosity(hobj,c)
            hobj.kvisc = c;
        end
        
        function setLoadResistance(hobj,r)
            hobj.rload = r;
        end
        function setVariableResistanceParameters(hobj,r)
            % Set 4 parameters for sinusoid [amp,freq,phase,mean]
            % Set 2 parameters for a ramp [rate,final] - NOTE it ramps down
            % from inf
            hobj.rlparams = r;
        end
        
        function hfig = plotPower(hobj,omg,lr)
            originalResistance = hobj.rload;
            for i=1:1:length(lr)
                hobj.setLoadResistance(lr(i));
                for j=1:1:length(omg)
                    pwr(i,j) = hobj.getPower(1.57*j/omg(end));
                end
            end
            hfig = figure;
            ax = axes('Parent',hfig);
            hold(ax,'on');
            for i=1:1:length(omg)
                plot(ax,lr,pwr(:,i),'DisplayName',['\omega = ' num2str(omg(i)*30/pi,'%4.2f') ' RPM']);
            end
            ax.XScale = 'log';
            legend('Location','Best');
            xlabel('Load Resistance (Ohm)'); ylabel('Power (W)');
            hobj.setLoadResistance(originalResistance);
        end
        
        function hfig = plotTorque(hobj,omg,lr)
            originalResistance = hobj.rload;
            for i=1:1:length(lr)
                hobj.setLoadResistance(lr(i));
                for j=1:1:length(omg)
                    trq(i,j) = hobj.getTorque(1.57*j/omg(end));
                end
            end
            hfig = figure;
            ax = axes('Parent',hfig);
            hold(ax,'on');
            for i=1:1:length(omg)
                plot(ax,lr,trq(:,i),'DisplayName',['\omega = ' num2str(omg(i)*30/pi,'%4.2f') ' RPM']);
            end
            ax.XScale = 'log';
            legend('Location','Best');
            xlabel('Load Resistance (Ohm)'); ylabel('Torque (Nm)');
            hobj.setLoadResistance(originalResistance);
        end
        
        function updateLoadResistance(hobj,t)
            if numel(hobj.rlparams) > 2 % sinusoid
                hobj.rload = hobj.rlparams(4) + hobj.rlparams(1)*sin(2*pi*hobj.rlparams(2)*t+hobj.rlparams(3));
            elseif ~isempty(hobj.rlparams) % ramp
                error('Ramp generator load resistance doesn"t work yet.');
                % todo ramp down from inf to hobj.rlparmams(2) at
                % hobj.rlparmams(1) rate
            end
            % otherwise just keep the current value
        end
    end % end methods    
end % end generator