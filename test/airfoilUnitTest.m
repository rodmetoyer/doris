function pf = airfoilUnitTest(opts)
% Airfoil unit test.
% INPUTS:
   % opts = struct. containing options
        % opts.verbose = true
        % opts.resultsfile = 'airfoilUnitTest_results.txt'
        % opts.plots = false
        % opts.movie = false
% OUTPUTS
    % pf = pass/fail bool
    
% Option defaults
if ~isfield(opts,'verbose')
    opts.verbose = true;
end
if ~isfield(opts,'resultsfile')
    opts.resultsfile = '';
end
if ~isfield(opts,'plots')
    opts.plots = false;
end
if ~isfield(opts,'movie')
    opts.movie = false;
end

% Make an airfoil
af = airfoil('S814');
% Make a plot using the coeff. pp's
% todo(rodney) add functionality to just compare to a baseline
aoa = -180:1.5:180;
cl = ppval(af.clpp,aoa);
cd = ppval(af.cdpp,aoa);
if opts.plots
    figure
    plot(aoa,cl,'LineWidth',2.0);
    hold on
    plot(aoa,cd,'LineWidth',2.0);
    hold off
    title(['Force Coefficients for ' af.airfoilName ' airfoil']);
    xlabel('Angle of Attack (deg)'); ylabel(['\color{red}C_L','\color{black} | ','\color{blue}C_D']);
end
clear af

% Make another airfoil
af = airfoil('SG6040');
% Make a plot using the coeff. pp's
% todo(rodney) add functionality to just compare to a baseline
aoa = -180:1.5:180;
cl = ppval(af.clpp,aoa);
cd = ppval(af.cdpp,aoa);
if opts.plots
    figure
    plot(aoa,cl,'LineWidth',2.0);
    hold on
    plot(aoa,cd,'LineWidth',2.0);
    hold off
    title(['Force Coefficients for ' af.airfoilName ' airfoil']);
    xlabel('Angle of Attack (deg)'); ylabel(['\color{red}C_L','\color{black} | ','\color{blue}C_D']);
end
clear af
try
afoops = airfoil('SG6040_35');
catch
disp('The badID warning is expected');
end

% All passed
pf = true;