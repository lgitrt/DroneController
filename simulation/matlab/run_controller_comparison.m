%% RUN_CONTROLLER_COMPARISON
% LQR, MPC and cascaded PID with noisy aided-IMU navigation.
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
scenarios = comparisonScenarios(true);

fprintf('Designing LQR, constrained MPC and cascaded PID...\n');
lqrController = struct('type','LQR', ...
    'K',designLQR(p,'Tuning',tuning,'SampleTime',Ts));
mpcController = struct('type','MPC','mpcObj',designMPC_Linear(p,Ts, ...
    'Tuning',tuning,'MaxInputStep',scenarios(1).maxInputStep));
pidController = designPID();
controllers = {lqrController,mpcController,pidController};
names = {'LQR','MPC','PID'};
summaryRows = cell(1,numel(scenarios)*numel(names));
comparisons = repmat(struct('outputs',[],'metrics',[]),1,numel(scenarios));
for i = 1:numel(scenarios)
    fprintf('\nSimulating %s...\n',scenarios(i).name);
    outputs = cell(1,3);
    for j = 1:3
        outputs{j} = simulateClosedLoop(p,traj,controllers{j},Ts,tEnd,x0,scenarios(i));
    end
    comparisons(i).outputs = outputs;
    directory = fullfile(here,'..','results',scenarios(i).slug);
    comparisons(i).metrics = plotComparison(outputs,p,directory,traj.rampTime);
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
