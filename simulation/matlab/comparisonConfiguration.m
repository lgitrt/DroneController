% Author: Luca Obwegs
function cfg = comparisonConfiguration()
%COMPARISONCONFIGURATION Shared experiment definition for plots and audits.
cfg.p = parameters();
cfg.traj = struct('radius',2,'omega',2*pi/30,'altitude',2.5,'rampTime',5);
cfg.Ts = 0.05;
cfg.tEnd = 45;
cfg.x0 = zeros(12,1);
cfg.scenarios = comparisonScenarios(true);
cfg.names = {'LQR','MPC','PID'};
cfg.controllers = { ...
    struct('type','LQR','K',designLQR(cfg.p,'Tuning','Balanced','SampleTime',cfg.Ts)), ...
    struct('type','MPC','mpcObj',designMPC_Linear(cfg.p,cfg.Ts, ...
        'Tuning','Balanced','MaxInputStep',cfg.scenarios(1).maxInputStep)), ...
    designPID()};
end
