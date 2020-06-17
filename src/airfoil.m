classdef airfoil < handle
    % An airfoil
    % List of supported airfoils as of 03JUN2019 (git history supercedes this note)
    % SG6040 | ID = 0
    % S814   | ID = 1
    
    % todo use enumeration and clean up the functionality in this class.
    properties (SetAccess = private)
        % These properties are set at instantiation and do not change.
        airfoilID   % Unique ID to identify the airfoil of this section        
        clcurve     % 2x1..* array defining cl curve (AoA vs. lift coeff.)
        cdcurve     % 2x1..* array defining cd curve (AoA vs. drag coeff.)
        cmcurve     % 2x1..* array defining cm curve (AoA vs. pitching moment coeff.)
        clpp        % Piecewise polynomial object for the cl alpha curve
        cdpp        % Piecewise polynomial object for the cd alpha curve
        cmpp        % Piecewise polynomial object for the cm alpha curve
        basecoords  % 3xn base (unit) coordinates for the airfoil geometry in the airfoil frame (xyz by nPoints)
        aoaopt      % Optimal angle of attack for the airfoil
    end
    
    properties (Dependent)
        %aoaopt      % Optimal angle of attack for the airfoil
        airfoilName % Name of the airfoil of this section
    end
    
    methods
        % Constructor
        function hobj=airfoil(id)
            if ischar(id)
                switch id
                    case 'SG6040'
                        hobj.airfoilID = airfoilID.SG6040;
                    case 'S814'
                        hobj.airfoilID = airfoilID.S814;
                    case 'NACA0015'
                        hobj.airfoilID = airfoilID.NACA0015;
                    case 'NACA0009'
                        hobj.airfoilID = airfoilID.NACA0009;
                    case 'FLAT'
                        hobj.airfoilID = airfoilID.FLAT;
                    otherwise
                        error('Unknown aifoil');
                end % switch
            else            
            hobj.airfoilID = id;
            end
            try
                [hobj.clcurve,hobj.cdcurve,hobj.cmcurve] = getClCdCmCurve(hobj.airfoilID);
            catch ME
                if strcmp(ME.identifier,'airfoil:badID')
                warning([ME.identifier ' - ' ME.message newline 'Using S814 and driving on.']);
                [hobj.clcurve,hobj.cdcurve,hobj.cmcurve] = getClCdCmCurve(1);
                else
                    error('airfoil: something went wrong, check code.');
                end
            end
            % Make the piecewise polynomial
            hobj.clpp = pchip(hobj.clcurve(1,:),hobj.clcurve(2,:));
            hobj.cdpp = pchip(hobj.cdcurve(1,:),hobj.cdcurve(2,:));
            hobj.cmpp = pchip(hobj.cmcurve(1,:),hobj.cmcurve(2,:));
            basecoords = getBaseCoords(hobj.airfoilID);
            hobj.basecoords = transpose(basecoords);
        end
        
        function setAoAopt(hobj,a)
            hobj.aoaopt = a;
        end
        % Other methods
        
        % Getters
%         function a = get.aoaopt(hobj)
%             switch hobj.airfoilID
%                 case 0
%                     a = 8.0;
%                 case 1
%                     a = 10.0; 
%                 case 2
%                     a = 10.0;
%                 case 3
%                     a = 8.0;
%                 case 4
%                     a = 8.0;
%                 otherwise
%             end
%         end
        function n = get.airfoilName(hobj)
            switch hobj.airfoilID
                case 0
                    n = 'SG6040';
                case 1
                    n = 'S814';
                case 2
                    n = 'NACA0015';
                case 3
                    n = 'NACA0009';
                case 4
                    n = 'FLAT';
                otherwise
            end
        end
        function hfig = showme(hobj,vis)
            if nargin < 2
                vis = 'on';
            end
            aoa = -180:0.2:180;
            cl1 = ppval(hobj.clpp,aoa);
            cd1 = ppval(hobj.cdpp,aoa);
            hfig = figure;
            plot(aoa,cl1,'LineWidth',2.0);
            hold on
            plot(aoa,cd1,'LineWidth',2.0);
            hold off
            title(['Force Coefficients for ' hobj.airfoilName ' airfoil']);
            xlabel('Angle of Attack (deg)'); ylabel('Coefficient Value');
            legend('C_L','C_D','Location','Best');
        end % showme
        function hfig = showmeltod(hobj,vis)
            if nargin < 2
                vis = 'on';
            end
            aoa = -180:1.5:180;
            cl1 = ppval(hobj.clpp,aoa);
            cd1 = ppval(hobj.cdpp,aoa);
            figure
            plot(aoa,cl1./cd1,'LineWidth',2.0);
            title(['L/D for ' hobj.airfoilName ' airfoil']);
            xlabel('Angle of Attack (deg)'); ylabel('Ratio');
        end % showmeltod
    end % end methods
end

% Functions outside of the classdef block are not class functions. These
% are just typical sub functions used in the conventional way to execute
% some routine.
function [clcurve, cdcurve, cmcurve] = getClCdCmCurve(af)
% INPUT
%   af = airfoilID
% OUTPUT
%   [clcurve, cdcurve] = [alpha vs. lift coeff., alpha vs. drag coeff.]

% todo(rodney) priority(1) add cmcurves if available
    % For now all cm points are cm0
cm0 = -0.088;
%cm0 = 0;
% todo(any) change execution to switch? Not sure what is more efficient.
% todo(any) add airfoils as needed
% todo(any) we need to get these from file rather than hardcode
% todo(any) add an airfoilID enumeration class

    % Support for string representation. Obviously this needs work.
    if af == 'S814' 
        AoA_cl = [-180.1;-176.3;-164;-156;-130;-99;-90;-58;-32;-26;-6;0;18;25;50;90;94.1;138;156;166;180.1];
        CL = [-0.14678;0;0.67087;0.53545;0.58294;0;-0.3048;-0.93947;-0.65806;-0.89269;0;0.28483;1.1347;0.80797;1.0797;0.1656;0;-0.78652;-0.89;-1.1924;-0.14678];
        AoA_cd = [-180.1;-130;-90;-30;-26;-10;-2;0;19;23;28;38;46;90;128;180];
        CD = [0.0797;1.2065;2.3162;0.43284;0.25791;0.12041;0.069511;0.054681;0.18731;0.38311;0.56152;0.88344;1.2171;2.3866;1.2507;0.0797]; 
        clcurve = [AoA_cl';CL'];
        cdcurve = [AoA_cd';CD'];
        cmcurve = [AoA_cl';cm0*ones(size(AoA_cl'))]; % todo(rodney) change this when(if) you find cm curve data
    elseif af == 'SG6040' 
        % todo this is same as SG6040 - fix it
        AoA_cl = [-180.1;-177;-168;-160;-140;-93;-90;-42;-20;-4;-1.5;0;15;18;40;90;93.0;130;160;166;180];
        CL = [-0.09;0;0.63;0.45;0.7;0;-0.1;-0.85;-0.3;-0.4;0;0.1;1.27;0.65;0.9;0.1;0;-0.85;-0.45;-0.75;-0.09];
        AoA_cd = [-180;-150;-90;-30;-4.0;0;8.3;30;90;150;180.1];
        CD = [0.04;0.4;1.5;0.4;0.0262;0.0335;0.0247;0.4;1.9;0.4;0.04]; 
        clcurve = [AoA_cl';CL'];
        cdcurve = [AoA_cd';CD'];
        cmcurve = [AoA_cl';cm0*ones(size(AoA_cl'))]; % todo(rodney) change this when(if) you find cm curve data
    elseif af == 'NACA0015' % NACA0015
        % todo this is same as SG6040 - fix it
        AoA_cl = [-180.1;-171;-160.4;-140.5;-93.2;-90;-42.45;-19.3;-14.32;0;14.32;19.3;42.45;90;93.2;140.5;160.4;171;180.1];
        CL = [0;0.908;0.616;0.95;0;-0.09;-1.05;-0.59;-1.122;0;1.122;0.59;1.05;0.09;0;-0.95;-0.616;-0.908;0];
        AoA_cd = [-180.1;-150;-90;-70.19;-18.66;-14.3;-8.4;0;8.4;14.3;18.66;70.19;90;135.24;150;180.1];
        CD = [0.0094;0.6;1.84;1.659;0.145;0.031;0.016;0.0069;0.016;0.031;0.145;1.659;1.84;1.105;0.6;0.0094]; 
        clcurve = [AoA_cl';CL'];
        cdcurve = [AoA_cd';CD'];
        cmcurve = [AoA_cl';cm0*ones(size(AoA_cl'))]; % todo(rodney) change this when(if) you find cm curve data
    elseif af == 'NACA0009' % Data from Sheldahl and Klimas
        % todo this is same as SG6040 - fix it
        AoA_cl = [-180.1;-171.1;-163.2;-140.6;-91.246;-42.51;-15.57;-9.031;0;9.031;15.57;42.51;91.246;140.6;163.2;171.1;180.1];
        CL = [0;0.78;0.669;0.993;0.003;-1.11;-0.694;-0.777;0;0.777;0.694;1.11;-0.003;-0.993;-0.669;-0.78;0];
        AoA_cd = [-190;-180;-90;0;90;180;190];
        CD = [0.052;0.034;1.867;0.028;1.867;0.034;0.052];  
        clcurve = [AoA_cl';CL'];
        cdcurve = [AoA_cd';CD'];
        cmcurve = [AoA_cl';cm0*ones(size(AoA_cl'))]; % todo(rodney) change this when(if) you find cm curve data
    elseif af == 'FLAT' % Compare to Ortiz, Rival and Wood 2015
        % todo this is same as SG6040 - fix it
        AoA_cl = [-180.1;-140.6;-91.246;-40;0;40;91.246;140.6;180.1];
%         CL = [0;0.8;0.003;-0.8;0;0.8;-0.003;-0.8;0];
        CL = 0.8*sind(2*AoA_cl);
        AoA_cd = [-190;-180;-90;0;90;180;190];
        %CD = [0.052;0.034;1.867;0.034;1.867;0.034;0.052];  
        CD = 1.8*sind(AoA_cd).^2+0.034;
        clcurve = [AoA_cl';CL'];
        cdcurve = [AoA_cd';CD'];
        cmcurve = [AoA_cl';cm0*ones(size(AoA_cl'))]; % todo(rodney) change this when(if) you find cm curve data
    else
        msg = 'Right now the only airfoils supported are S814(id=1), SG6040(id=2), and SG6040_24thk(id=3)';
        ME = MException('airfoil:badID',msg);
        throw(ME);
    end
end

function coords = getBaseCoords(id)
    if id==0
        coords = [1,0,0;0.997880000000000,0.000480000000000000,0;0.991830000000000,0.00207000000000000,0;0.982330000000000,0.00477000000000000,0;0.969770000000000,0.00845000000000000,0;0.954420000000000,0.0128300000000000,0;0.936370000000000,0.0176600000000000,0;0.915590000000000,0.0228500000000000,0;0.892210000000000,0.0284500000000000,0;0.866470000000000,0.0344600000000000,0;0.838610000000000,0.0408000000000000,0;0.808870000000000,0.0473900000000000,0;0.777520000000000,0.0541500000000000,0;0.744820000000000,0.0609400000000000,0;0.711040000000000,0.0675900000000000,0;0.676320000000000,0.0739400000000000,0;0.640810000000000,0.0799200000000000,0;0.604770000000000,0.0854100000000000,0;0.568370000000000,0.0903200000000000,0;0.531800000000000,0.0946100000000000,0;0.495260000000000,0.0981500000000000,0;0.458910000000000,0.100890000000000,0;0.422970000000000,0.102790000000000,0;0.387580000000000,0.103780000000000,0;0.352860000000000,0.103860000000000,0;0.319030000000000,0.102990000000000,0;0.286190000000000,0.101160000000000,0;0.254490000000000,0.0984300000000000,0;0.224050000000000,0.0947900000000000,0;0.194960000000000,0.0902800000000000,0;0.167340000000000,0.0850600000000000,0;0.141370000000000,0.0792100000000000,0;0.117190000000000,0.0728300000000000,0;0.0949600000000000,0.0659800000000000,0;0.0748100000000000,0.0587500000000000,0;0.0568600000000000,0.0512300000000000,0;0.0412300000000000,0.0434700000000000,0;0.0279800000000000,0.0355900000000000,0;0.0172500000000000,0.0277100000000000,0;0.00913000000000000,0.0198700000000000,0;0.00362000000000000,0.0121200000000000,0;0.000720000000000000,0.00449000000000000,0;0.000270000000000000,-0.00268000000000000,0;0.00289000000000000,-0.00884000000000000,0;0.00906000000000000,-0.0145000000000000,0;0.0182900000000000,-0.0202300000000000,0;0.0302700000000000,-0.0258000000000000,0;0.0448300000000000,-0.0310300000000000,0;0.0620200000000000,-0.0356600000000000,0;0.0819300000000000,-0.0397100000000000,0;0.104460000000000,-0.0433600000000000,0;0.129390000000000,-0.0465700000000000,0;0.156550000000000,-0.0493700000000000,0;0.185740000000000,-0.0517200000000000,0;0.216770000000000,-0.0536100000000000,0;0.249430000000000,-0.0550300000000000,0;0.283490000000000,-0.0559400000000000,0;0.318730000000000,-0.0563200000000000,0;0.354900000000000,-0.0561300000000000,0;0.391770000000000,-0.0552800000000000,0;0.429160000000000,-0.0537100000000000,0;0.466910000000000,-0.0513200000000000,0;0.504790000000000,-0.0480100000000000,0;0.542980000000000,-0.0436800000000000,0;0.581640000000000,-0.0386300000000000,0;0.620490000000000,-0.0332300000000000,0;0.659250000000000,-0.0277200000000000,0;0.697620000000000,-0.0223100000000000,0;0.735260000000000,-0.0171800000000000,0;0.771810000000000,-0.0125000000000000,0;0.806900000000000,-0.00840000000000000,0;0.840160000000000,-0.00497000000000000,0;0.871200000000000,-0.00228000000000000,0;0.899630000000000,-0.000330000000000000,0;0.925110000000000,0.000900000000000000,0;0.947280000000000,0.00148000000000000,0;0.965870000000000,0.00154000000000000,0;0.980630000000000,0.00122000000000000,0;0.991320000000000,0.000700000000000000,0;0.997820000000000,0.000220000000000000,0;1,0,0];
    elseif id ==1
        coords = [1,0,0;0.996277000000000,0.00107900000000000,0;0.985681000000000,0.00464400000000000,0;0.969429000000000,0.0106910000000000,0;0.948574000000000,0.0185250000000000,0;0.923625000000000,0.0271570000000000,0;0.894505000000000,0.0357380000000000,0;0.860850000000000,0.0441780000000000,0;0.823023000000000,0.0527480000000000,0;0.781586000000000,0.0614240000000000,0;0.737130000000000,0.0701080000000000,0;0.690273000000000,0.0786590000000000,0;0.641651000000000,0.0869010000000000,0;0.591910000000000,0.0946330000000000,0;0.541692000000000,0.101631000000000,0;0.491625000000000,0.107648000000000,0;0.442317000000000,0.112418000000000,0;0.394345000000000,0.115645000000000,0;0.348257000000000,0.116962000000000,0;0.304384000000000,0.115627000000000,0;0.261983000000000,0.111612000000000,0;0.221337000000000,0.105629000000000,0;0.182903000000000,0.0979970000000000,0;0.147112000000000,0.0889660000000000,0;0.114367000000000,0.0787630000000000,0;0.0850390000000000,0.0676190000000000,0;0.0594810000000000,0.0557660000000000,0;0.0380070000000000,0.0434580000000000,0;0.0209520000000000,0.0309540000000000,0;0.00857700000000000,0.0185560000000000,0;0.00143100000000000,0.00667200000000000,0;0.00111900000000000,0.00579700000000000,0;0.000338000000000000,0.00295900000000000,0;2.00000000000000e-06,0.000223000000000000,0;0,0,0;0.000245000000000000,-0.00254900000000000,0;0.000678000000000000,-0.00458400000000000,0;0.000925000000000000,-0.00549100000000000,0;0.00638100000000000,-0.0174300000000000,0;0.0167920000000000,-0.0315760000000000,0;0.0313670000000000,-0.0464920000000000,0;0.0496410000000000,-0.0616550000000000,0;0.0712400000000000,-0.0767130000000000,0;0.0956100000000000,-0.0910930000000000,0;0.122438000000000,-0.104309000000000,0;0.151203000000000,-0.115726000000000,0;0.181669000000000,-0.124578000000000,0;0.213672000000000,-0.130235000000000,0;0.247139000000000,-0.131310000000000,0;0.283942000000000,-0.127757000000000,0;0.323782000000000,-0.120293000000000,0;0.367326000000000,-0.109093000000000,0;0.414593000000000,-0.0952490000000000,0;0.465255000000000,-0.0796600000000000,0;0.518814000000000,-0.0632360000000000,0;0.574574000000000,-0.0468940000000000,0;0.631638000000000,-0.0315200000000000,0;0.688908000000000,-0.0179040000000000,0;0.745125000000000,-0.00668800000000000,0;0.798908000000000,0.00169800000000000,0;0.848830000000000,0.00707000000000000,0;0.893492000000000,0.00951700000000000,0;0.931609000000000,0.00939500000000000,0;0.962086000000000,0.00727900000000000,0;0.983819000000000,0.00388900000000000,0;0.996132000000000,0.00101400000000000,0;1,0,0];
    elseif id == 2
        coords = getNACA00xx(15);
    elseif id == 3
        coords = getNACA00xx(09);
    elseif id == 4
        coords = getNACA00xx(0);
    else
        error('No available coordinates for unknown airfoil id.');
    end
end

function coords = getNACA00xx(xx)
        cup = [1:-.1:0.1 0.09:-0.01:0];
        cdwn = [0.01:0.01:0.09 0.1:0.1:1];
        yup = 5*xx/100*(0.2969*sqrt(cup)-0.1260*cup-0.3516*cup.^2+0.2843*cup.^3-0.1036*cup.^4);
        ydwn = -5*xx/100*(0.2969*sqrt(cdwn)-0.1260*cdwn-0.3516*cdwn.^2+0.2843*cdwn.^3-0.1036*cdwn.^4);
        c = [cup cdwn]; y = [yup ydwn]; z = zeros(size(c));
        coords = [c; y; z].';
end