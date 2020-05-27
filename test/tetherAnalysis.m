% tetherAnalysis.m

clear all; close all; clc;

%fn = ["dropTestBig0water.mat","dropTestBig1water.mat","dropTestBig5water.mat","dropTestBig10water.mat"];
fn = ["circTest0air.mat","circTest2air.mat","circTest5air.mat"];
savefn = 'circTestAirComp';
makePlots(fn,savefn);

function makePlots(fn,savefn)
    % this one makes a set of plots for all the files in fns
    hfigx = figure('Position',[100 100 900 600]);
    axx = axes('Parent',hfigx); hold(axx,'on');
    hfigy = figure('Position',[200 100 900 600]);
    axy = axes('Parent',hfigy); hold(axy,'on');
    hfigz = figure('Position',[250 150 900 600]);
    axz = axes('Parent',hfigz); hold(axz,'on');
    mrkr = ["-","--",":","-."];
    for i=1:1:length(fn)
        % load the mat file and save the data to arrays
        load(['output\tetherTestData\' char(fn(i))]);        
        [~,yend] = size(y);
        numnodes = (yend-12)/6;        
        % make plots        
        plot(axx,t,y(:,7),[char(mrkr(i)) 'r'],'LineWidth',2.0,'DisplayName',['Nodes: ' num2str(numnodes)]);
        xlabel(axx,'time (s)'); ylabel(axx,'x coord end body'); legend(axx,'Color','none','FontSize',16);
        plot(axy,t,y(:,8),[char(mrkr(i)) 'b'],'LineWidth',2.0,'DisplayName',['Nodes: ' num2str(numnodes)]);
        xlabel(axy,'time (s)'); ylabel(axy,'y coord end body'); legend(axy,'Color','none','FontSize',16);
        plot(axz,t,y(:,9),[char(mrkr(i)) 'g'],'LineWidth',2.0,'DisplayName',['Nodes: ' num2str(numnodes)]);
        xlabel(axz,'time (s)'); ylabel(axz,'z coord end body'); legend(axz,'Color','none','FontSize',16);
        %legend('x','y','z');
        % clear vars
        clear y t yend
    end
    axx.Color = 'none';
    axy.Color = 'none';
    axz.Color = 'none';
    export_fig(hfigx,['output\figs\' savefn 'Xs.png'],'-png','-transparent');
    export_fig(hfigy,['output\figs\' savefn 'Ys.png'],'-png','-transparent');
    export_fig(hfigz,['output\figs\' savefn 'Zs.png'],'-png','-transparent');
end


function makeMovies(fn)
    for i=1:1:length(fn)
        % load the mat file
        load(fn(i));
        %numnodes = str2num(regexp(fn(i), '\d+', 'match'));
        % make a movie
        [~,yend] = size(y);
        tdat = [];
        if yend > 12
            tdat = y(:,13:yend);
        end
        thr.makeMovie('Adata',y(:,1:6),'Bdata',y(:,7:12),'tetherdata',tdat,'times',t,'framerate',24,'hide',true);
        % clear vars
        clearvars -except i fn nnodes thr
    end
end