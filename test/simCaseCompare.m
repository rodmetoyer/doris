% simCaseCompare.m
% Compares simulation results to baseline

clearvars; close all; clc;

%% Get simulations
% Get all simulation data from the data folder
resultsfiles = dir('../data/*.txt');
for i=1:1:numel(resultsfiles)
    rf(i) = string(resultsfiles(i).name);
end
%% Get baseline data
baselinefiles = dir('baseline/*.txt');
for i=1:1:numel(baselinefiles)
    bf(i) = string(baselinefiles(i).name);
end

%% Compare and create a report
% todo is there a more efficent way to do this?
itr = 1;
for j=1:1:numel(rf)
    for i=1:1:numel(bf)
        if strcmp(rf(j),bf(i))
            RfBfMatch(1,itr) = j;
            RfBfMatch(2,itr) = i;
            itr = itr + 1;
        end
    end
end
% RfBfMatch columns are the matching files. That's what we want to compare.
hfig = figure;
nFigs = size(RfBfMatch,2);
for i=1:1:nFigs
    Arf = importdata(['../data/' char(rf(RfBfMatch(1,i)))]);
    datrf(:,:,i) = Arf.data;
    Abf = importdata(['baseline/' char(bf(RfBfMatch(2,i)))]);
    datbf(:,:,i) = Abf.data;
    % Check that the data are equal
    rfEQbf = datrf(:,:,i) ~= datbf(:,:,i);
    if sum(rfEQbf,'all') > 0
        wrnstr = [char(rf(RfBfMatch(1,i))) ' does not match baseline file ' char(bf(RfBfMatch(2,i)))];
        warning(wrnstr);
        rptstr(itr) = string(wrnstr);
    end
    subplot(nFigs,1,i);
    plot(datrf(:,1,i),datrf(:,2:end,i),'-','LineWidth',2.0)
    hold on
    plot(datbf(:,1,i),datbf(:,2:end,i),':','LineWidth',2.0)
    hold off
    xlabel('Time (s)'); ylabel('State Values');
    title(['Results of: ' char(rf(RfBfMatch(1,i))) '  Compared to Baseline: ' char(bf(RfBfMatch(2,i)))]);
end
hfig.Position=[200 200 800 300*nFigs];
fignm = ['compareResults\compareResults_' date '.png'];
saveas(hfig,fignm);
%% Make a report file
% todo make a report file
