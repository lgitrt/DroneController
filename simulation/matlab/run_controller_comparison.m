%% RUN_CONTROLLER_COMPARISON
% LQR versus constrained linear MPC on a shared nonlinear quadrotor plant.
% Author: Luca Obwegs
clear; clc; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
p = parameters();
traj = struct('radius',2,'omega',2*pi/30,'altitude',2.5,'rampTime',5);
Ts = 0.05;
tEnd = 45;
x0 = zeros(12,1);
tuning = 'Balanced';
scenarios = comparisonScenarios();

fprintf('Designing matched-cost discrete LQR and constrained MPC...\n');
lqrController = struct('type','LQR', ...
    'K',designLQR(p,'Tuning',tuning,'SampleTime',Ts));
mpcController = struct('type','MPC','mpcObj',designMPC_Linear(p,Ts, ...
    'Tuning',tuning,'MaxInputStep',scenarios(1).maxInputStep));
summaryRows = {};
for i = 1:numel(scenarios)
    fprintf('\nSimulating %s...\n',scenarios(i).name);
    comparisons(i).lqr = simulateClosedLoop(p,traj,lqrController,Ts,tEnd,x0,scenarios(i));
    comparisons(i).mpc = simulateClosedLoop(p,traj,mpcController,Ts,tEnd,x0,scenarios(i));
    directory = fullfile(here,'..','results',scenarios(i).slug);
    comparisons(i).metrics = plotComparison(comparisons(i).lqr, ...
        comparisons(i).mpc,p,directory,traj.rampTime);
    names = {'LQR','MPC'};
    for j = 1:2
        metrics = comparisons(i).metrics.(names{j});
        row = metrics;
        row.scenario = scenarios(i).name;
        row.controller = names{j};
        summaryRows{end+1} = row;
    end
end
summary = struct2table([summaryRows{:}]);
writetable(summary,fullfile(here,'..','results','metrics.csv'));
fprintf('\nBoth controllers use %.0f Hz sampling and the same input/rate limits.\n',1/Ts);
disp(summary(:,{'scenario','controller','rmse3D','rmseCircle3D','maxError3D','meanCommandDeviation'}));
fprintf('Figures and metrics saved under %s\n',fullfile(here,'..','results'));
