classdef bladesection < handle
    % A section (element) of a blade.
    % Constructor inputs:
        % chord = Chord length (m)
        % width = Section width (m)
        % af = either:
            % an airfoil id, which will create a unique airfoil object for
                % this bladesection or:
            % a handle for an airfoil object
        % afname (optional) = name of the airfoil (only if creating airfoil
            % objects. If you pass a handle to an existing airfoil object
            % you will not be able to rename the airfoil when creating the
            % section object.
% General Information:
% Bladesection frame defined by i_a pointing from leading edge to training
% edge and coincident with the chord, j_a perpendicular to the bladesection
% plane and pointing such that positive rotation about j_a results in a 
% increase in angle of attack, and k_a = cross(i_a,j_a).

% vehicle <aggregate-- rotor <compose-- blade <compose-- bladesection
% --generalize> airfoil
    properties (SetAccess = private)
        hAirfoil   % The object of class airfoil for this section
        chord      % Chord length (m)
        width      % Width of blade domain covered by section (m)
        %mass    right now the section can't have mass, only the blade.
        %todo(rodney) add functionality to enable distributed mass
        lift       % lift (N)
        drag       % drag (N)
        momentqc   % Moment about the quarter chord (Nm)
        %id         % Unique integer identifier - maybe don't need for sections
    end
    
%     properties
%         % todo(rodney) make these private? Maybe not. Blade knows where he
%         % is. He doesn't know.
%         location   % 3xn location on the blade in the blade frame (m)
%         rotation   % 3xn rotation on the blade following 3-2-1 sequence (deg)
%     end
    
    methods
        %% Constructor
        function hobj=bladesection(chord,width,af,afname)
            % af may be either an airfoil ID or a handle for an instantiated airfoil object
            if nargin > 3
                if isnumeric(af)
                    % Create an instance of airfoil to go with the section
                    hobj.hAirfoil = airfoil(af,afname);
                else
                    % Should be an airfoil handle getting passed in.
                    % todo(any) can we check that? 
                    try
                        isvalid(af);
                    catch
                        error('bladesection: Problem with the airfoil designation - not numeric, not a valid handle... I don"t know how to proceed.');
                    end
                    warning('You cannot rename the airfoil.');
                end
            else
                if isnumeric(af)
                    % Create an instance of airfoil to go with the section
                    afname = 'unnamed';
                    hobj.hAirfoil = airfoil(af,afname);
                else
                hobj.hAirfoil = af;
                end
            end
            hobj.chord = chord;
            hobj.width = width;
        end
        
        %% Class Methods
        function [L,D,M] = computeLoads(hobj,vr,fluid)
            % Computes the aerodynamic loads on the section from the airfoil cl
                % and cd curves. Only the rotor knows the position of the
                % blade sections in space.
            % INPUTS:
                % vr = 3x1 relative velocity vector expressed in the blade
                    % section frame (i_a, j_a, k_a).
                % fluid = a fluid object
                
            % The blade section only knows his orientation with respect to
            % the velocity vector passed into this method. First must
            % compute angle of attack.
            aoa = atan2d(vr(3),vr(1)); % Section aerodynamic loads are due only to in-plane velocity.
            % Out-of-plane component of relative velocity is deal with by
            % the blade (which is the object with physical width).
            cl = ppval(hobj.hAirfoil.clpp,aoa);
            cd = ppval(hobj.hAirfoil.clpp,aoa);
            cm = ppval(hobj.hAirfoil.clpp,aoa);
            temp = 0.5*fluid.density*norm([vr(3),vr(1)],2)^2*hobj.chord*hobj.width;
            Lmag = cl*temp;
            Dmag = cd*temp;
            Mmag = cm*temp*hobj.chord; 
            temp = sqrt(vr(1)^2+vr(3)^2);
            salpha = 0;
            calpha = 0;
            if temp ~=0
                salpha = vr(3)/temp;
                calpha = vr(1)/temp;
            end
            D = [Dmag*calpha;0;Dmag*salpha];
            L = [-Lmag*salpha;0;Lmag*calpha];
            M = [0;Mmag;0];
            hobj.lift = Lmag;
            hobj.drag = Dmag;
            hobj.momentqc = Mmag;
        end
        
        function [L,D,M] = computeLoadsFast(hobj,vr,fluid)
            error('Obsolete method');
%             % Computes the aerodynamic loads on the section from the airfoil cl
%                 % and cd curves. Only the rotor knows the position of the
%                 % blade sections in space.
%             % INPUTS:
%                 % vr = 3x1 relative velocity vector expressed in the blade
%                     % section frame (i_a, j_a, k_a).
%                 % fluid = a fluid object
%                 
%             % The blade section only knows his orientation with respect to
%             % the velocity vector passed into this method. First must
%             % compute angle of attack.
%             aoa = atan2d(vr(3),vr(1)); % Section aerodynamic loads are due only to in-plane velocity.
%             % Out-of-plane component of relative velocity is deal with by
%             % the blade (which is the object with physical width).
%             clcurve = hobj.hAirfoil.clcurve;
%             cdcurve = hobj.hAirfoil.cdcurve;
%             cmcurve = hobj.hAirfoil.cmcurve;
%             cl = interp1(clcurve(1,:),clcurve(2,:),round(aoa));
%             cd = interp1(cdcurve(1,:),cdcurve(2,:),round(aoa));
%             cm = interp1(cmcurve(1,:),cmcurve(2,:),round(aoa));
%             temp = 0.5*fluid.density*norm([vr(3),vr(1)],2)^2*hobj.chord*hobj.width;
%             Lmag = cl*temp;
%             Dmag = cd*temp;
%             Mmag = cm*temp*hobj.chord; 
%             temp = sqrt(vr(1)^2+vr(3)^2);
%             salpha = 0;
%             calpha = 0;
%             if temp ~=0
%                 salpha = vr(3)/temp;
%                 calpha = vr(1)/temp;
%             end
%             D = [Dmag*calpha;0;Dmag*salpha];
%             L = [-Lmag*salpha;0;Lmag*calpha];
%             M = [0;Mmag;0];
%             hobj.lift = Lmag;
%             hobj.drag = Dmag;
%             hobj.momentqc = Mmag;
        end
        % setters
    end    
end