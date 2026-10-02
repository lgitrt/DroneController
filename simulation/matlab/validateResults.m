function report = validateResults(seeds)
%VALIDATERESULTS Independently score sensed runs across explicit noise seeds.
%   Exports numerical evidence only; no figures or controller retuning.
if nargin < 1
    seeds = 20261002:20261004;
end
validateattributes(seeds,{'double'},{'vector','nonempty','integer','nonnegative','finite'});
cfg = comparisonConfiguration();
rows = cell(1,numel(seeds)*2*numel(cfg.controllers));
rowIndex = 0;
for seed = seeds(:)'
    for i = 2:3
        scenario = cfg.scenarios(i);
        scenario.sensorConfig.seed = seed;
        for j = 1:numel(cfg.controllers)
            fprintf('Auditing seed %d | %s | %s\n',seed,scenario.name,cfg.names{j});
            out = simulateClosedLoop(cfg.p,cfg.traj,cfg.controllers{j}, ...
                cfg.Ts,cfg.tEnd,cfg.x0,scenario);
            m = trackingMetrics(out,cfg.p,cfg.traj.rampTime);
            positionError = out.X(1:3,:)-out.Ref(1:3,:);
            independentlyScoredRMS = sqrt(sum(positionError(:).^2)/numel(out.t));
            assert(abs(m.rmse3D-independentlyScoredRMS) < 1e-12,'RMS definition mismatch.');
            assert(all(isfinite(out.X),'all') && all(isfinite(out.XEstimated),'all'), ...
                'Nonfinite plant or estimator state.');
            assert(all(out.U >= cfg.p.uMin-1e-6 & out.U <= cfg.p.uMax+1e-6,'all'), ...
                'Command exceeds physical rotor bounds.');
            assert(all(out.UApplied >= cfg.p.uMin-1e-6 & out.UApplied <= cfg.p.uMax+1e-6,'all'), ...
                'Applied input exceeds physical rotor bounds.');
            assert(m.peakCommandStep <= scenario.maxInputStep+1e-10,'Command rate exceeds limit.');
            assert(min(eig(out.finalEstimatorCovariance)) >= -1e-12,'Invalid EKF covariance.');
            if strcmp(cfg.names{j},'MPC')
                assert(all(out.solverIterations > 0),'Unsuccessful MPC solve.');
            end
            distanceSquared = sum(positionError.^2,1);
            windWindow = out.t >= 12 & out.t <= 28;
            recovery = out.t >= 32;
            row = struct('seed',seed,'scenario',scenario.name,'controller',cfg.names{j}, ...
                'rmse3D',m.rmse3D,'rmseCircle3D',m.rmseCircle3D, ...
                'rmseWindWindow',sqrt(mean(distanceSquared(windWindow))), ...
                'rmseRecovery',sqrt(mean(distanceSquared(recovery))), ...
                'maxError3D',m.maxError3D, ...
                'estimationPositionRMSE',m.estimationPositionRMSE, ...
                'estimationAttitudeRMSEDeg',m.estimationAttitudeRMSEDeg, ...
                'maxTiltDeg',rad2deg(max(abs(out.X(4:5,:)),[],'all')), ...
                'saturatedSamples',m.saturatedSamples,'rateLimitedSamples',m.rateLimitedSamples);
            rowIndex = rowIndex+1;
            rows{rowIndex} = row;
        end
    end
end
report = struct2table([rows{:}]);
directory = fullfile(fileparts(mfilename('fullpath')),'..','results');
if ~exist(directory,'dir')
    mkdir(directory);
end
writetable(report,fullfile(directory,'validation.csv'));
disp(report(:,{'seed','scenario','controller','rmse3D','rmseWindWindow','rmseRecovery'}));
end
