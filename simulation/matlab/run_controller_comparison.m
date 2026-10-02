%% RUN_CONTROLLER_COMPARISON
% LQR, MPC and cascaded PID with noisy aided-IMU navigation.
% Author: Luca Obwegs
clear; clc; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
fprintf('Designing LQR, constrained MPC and cascaded PID...\n');
cfg = comparisonConfiguration();
p = cfg.p;
traj = cfg.traj;
Ts = cfg.Ts;
scenarios = cfg.scenarios;
controllers = cfg.controllers;
names = cfg.names;
summaryRows = cell(1,numel(scenarios)*numel(names));
comparisons = repmat(struct('outputs',[],'metrics',[]),1,numel(scenarios));
for i = 1:numel(scenarios)
    fprintf('\nSimulating %s...\n',scenarios(i).name);
    outputs = cell(1,3);
    for j = 1:3
        outputs{j} = simulateClosedLoop(p,traj,controllers{j},Ts,cfg.tEnd,cfg.x0,scenarios(i));
    end
    comparisons(i).outputs = outputs;
    directory = fullfile(here,'..','results',scenarios(i).slug);
    selected = comparisonPlotSelection(scenarios(i).slug);
    comparisons(i).metrics = plotComparison(outputs,p,directory,traj.rampTime,selected);
    for j = 1:3
        metrics = comparisons(i).metrics.(names{j});
        row = metrics;
        row.scenario = scenarios(i).name;
        row.controller = names{j};
        summaryRows{(i-1)*3+j} = row;
    end
end
summary = struct2table([summaryRows{:}]);
writetable(summary,fullfile(here,'..','results','metrics.csv'));
fprintf('\nAll controllers use %.0f Hz sampling and the same input/rate limits.\n',1/Ts);
disp(summary(:,{'scenario','controller','rmse3D','maxError3D','estimationPositionRMSE'}));
fprintf('Figures and metrics saved under %s\n',fullfile(here,'..','results'));
