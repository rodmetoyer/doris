classdef bladesection < handle
    % A section (element) of a blade.
    % Constructor inputs:
        % chord = Chord length (m)
        % width = Section width (m)
        % af = either:
            % an airfoil id, which will create a unique airfoil object for
                % this bladesection or:
            % a handle for an airfoil object
% General Information:
% Bladesection frame defined by i_a pointing from leading edge to training
% edge and coincident with the chord, j_a perpendicular to the bladesection
% plane and pointing such that positive rotation about j_a results in a 
% increase in angle of attack, and k_a = cross(i_a,j_a).

% vehicle <aggregate-- rotor <compose-- blade <compose-- bladesection
% --generalize> airfoil
    properties (SetAccess = private)
        airfoil     % The object of class airfoil for this section
        chord       % Chord length (m)
        width       % Width of blade domain covered by section (m)
        coords      % Coordinates of the section edge in the section frame
        %mass    right now the section can't have mass, only the blade.
        %todo(rodney) add functionality to enable distributed mass
%         lift        % lift (N)
%         drag        % drag (N)
%         momentqc    % Moment about the quarter chord (Nm)
    end
    
%     properties
%         % todo(rodney) make these private? Maybe not. Blade knows where he
%         % is. He doesn't know.
%         location   % 3xn location on the blade in the blade frame (m)
%         rotation   % 3xn rotation on the blade following 3-2-1 sequence (deg)
%     end
    
    methods
        %% Constructor
        function hobj=bladesection(chord,width,af)
            % af may be either an airfoil ID or a handle for an instantiated airfoil object
            if nargin < 3
                error('A bladesection must be instantiated with chord and width');
            else
                if isnumeric(af)
                    % Create an instance of airfoil to go with the section
                    hobj.airfoil = airfoil(af);
                else
                hobj.airfoil = af;
                end
            end
            hobj.chord = chord;
            hobj.width = width;
            % Move to chord/4
            coords = hobj.airfoil.basecoords;
            coords(1,:) = coords(1,:) - 0.25;
            % Not shifting to mean chamber line because of how frame is
            % defined.
%            % Find middle to shift from chord line to mean chamber line
%             ln = round(numel(coords(:,1))/2);
%             yt = pchip(coords(1:ln,1),coords(1:ln,2),0.25);
%             yb = pchip(coords(ln:end,1),coords(ln:end,2),0.25);
%             % shift from chord line to mean chamber line
%             coords(:,2) = coords(:,2)-(yb+yt)/2;
            % Rotate into bladesection frame. -90 degrees about x.
            coords = [1 0 0; 0 0 -1; 0 1 0]*coords;
            % Finally scale to chord
            coords = coords*hobj.chord;
            hobj.coords = coords;
        end
        
        %% Other class methods
        function [lift,drag,moment] = computeLoads(hobj,vrel,fluid)
            % Computes the aerodynamic loads on the section from the airfoil cl
                % and cd curves. Only the rotor knows the position of the
                % blade sections in space.
            % INPUTS:
                % vrel = 3x1 relative velocity vector expressed in the blade
                    % section frame (i_a, j_a, k_a).
                % fluid = a fluid object
            % OUTPUTS:
                % [lift,drag,moment] = loads relative to the vr argument
                
            % The blade section only knows his orientation with respect to
            % the velocity vector passed into this method.
            % NOTE aoa is in degrees in the curves.
            aoa = atan2d(vrel(3),vrel(1)); % Section aerodynamic loads are due only to in-plane velocity.
            % Out-of-plane component of relative velocity is deal with by
            % the blade (which is the object with physical width).
            cl = ppval(hobj.airfoil.clpp,aoa);
            cd = ppval(hobj.airfoil.cdpp,aoa);
            cm = ppval(hobj.airfoil.cmpp,aoa);
            temp = 0.5*fluid.density*norm([vrel(3),vrel(1)],2)^2*hobj.chord*hobj.width;
            Lmag = cl*temp;
            Dmag = cd*temp;
            Mmag = cm*temp*hobj.chord; 
            temp = sqrt(vrel(1)^2+vrel(3)^2);
            salpha = 0;
            calpha = 0;
            if temp > 1.0e-12
                salpha = vrel(3)/temp;
                calpha = vrel(1)/temp;
            end
            drag = [Dmag*calpha;0;Dmag*salpha];
            lift = [-Lmag*salpha;0;Lmag*calpha];
            moment = [0;Mmag;0];
%             hobj.lift = Lmag;
%             hobj.drag = Dmag;
%             hobj.momentqc = Mmag;
        end
        
        function [cl,cd,cm] = computeForceCoeffs(hobj,vrel,fluid)
            % Computes the aerodynamic loads on the section from the airfoil cl
                % and cd curves. Only the rotor knows the position of the
                % blade sections in space.
            % INPUTS:
                % vrel = 3x1 relative velocity vector expressed in the blade
                    % section frame (i_a, j_a, k_a).
                % fluid = a fluid object
            % OUTPUTS:
                % [cl,cd,cm] = force coefficients

            aoa = atan2d(vrel(3),vrel(1)); % Section aerodynamic loads are due only to in-plane velocity.
            cl = ppval(hobj.airfoil.clpp,aoa);
            cd = ppval(hobj.airfoil.cdpp,aoa);
            cm = ppval(hobj.airfoil.cmpp,aoa);
        end % computeForceCoeffs
        
        function [L,D,M] = computeLoadsFast(hobj,vr,fluid)
            % This method is obsolete. See GIT history.
            error('Obsolete method');
        end % end computeLoadsFast
        % setters
    end    
end