function metrics = plotComparison(agg, cons, p, outDir)
%PLOTCOMPARISON Generate comparison figures for Aggressive vs Conservative LQR.
%   METRICS = PLOTCOMPARISON(AGG, CONS, P, OUTDIR) takes the simulation
%   results of SIMULATECLOSEDLOOP for both tunings, saves a set of
%   comparison figures (PNG) into OUTDIR, and returns a struct of RMSE /
%   control-effort metrics.

if nargin < 4 || isempty(outDir)
    outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
end
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

colAgg  = [0.00 0.45 0.74];
colCons = [0.85 0.33 0.10];
colRef  = [0.15 0.15 0.15];

%% Figure 1: XY plane circle tracking
f1 = figure('Name', 'Circle Tracking - XY plane', 'Color', 'w', 'Position', [100 100 700 650]);
plot(agg.Ref(1,:), agg.Ref(2,:), '--', 'Color', colRef, 'LineWidth', 1.5); hold on;
plot(agg.X(1,:),  agg.X(2,:),  '-',  'Color', colAgg,  'LineWidth', 1.8);
plot(cons.X(1,:), cons.X(2,:), '-',  'Color', colCons, 'LineWidth', 1.8);
axis equal; grid on; box on;
xlabel('x [m]'); ylabel('y [m]');
title('Circle Trajectory Tracking: Aggressive vs Conservative LQR');
legend('Reference', 'Aggressive LQR', 'Conservative LQR', 'Location', 'bestoutside');
saveas(f1, fullfile(outDir, 'xy_tracking_comparison.png'));

%% Figure 2: position vs time
f2 = figure('Name', 'Position tracking', 'Color', 'w', 'Position', [100 100 900 750]);
labels = {'x [m]', 'y [m]', 'z [m]'};
for i = 1:3
    subplot(3,1,i);
    plot(agg.t, agg.Ref(i,:), '--', 'Color', colRef, 'LineWidth', 1.3); hold on;
    plot(agg.t, agg.X(i,:), '-', 'Color', colAgg, 'LineWidth', 1.5);
    plot(cons.t, cons.X(i,:), '-', 'Color', colCons, 'LineWidth', 1.5);
    ylabel(labels{i}); grid on; box on;
    if i == 1
        title('Position Tracking vs Time');
        legend('Reference', 'Aggressive LQR', 'Conservative LQR', 'Location', 'bestoutside');
    end
end
xlabel('Time [s]');
saveas(f2, fullfile(outDir, 'xyz_tracking_comparison.png'));

%% Figure 3: attitude vs time
f3 = figure('Name', 'Attitude tracking', 'Color', 'w', 'Position', [100 100 900 750]);
angLabels = {'Roll \phi [deg]', 'Pitch \theta [deg]', 'Yaw \psi [deg]'};
for i = 1:3
    subplot(3,1,i);
    plot(agg.t, rad2deg(agg.Ref(3+i,:)), '--', 'Color', colRef, 'LineWidth', 1.3); hold on;
    plot(agg.t, rad2deg(agg.X(3+i,:)), '-', 'Color', colAgg, 'LineWidth', 1.5);
    plot(cons.t, rad2deg(cons.X(3+i,:)), '-', 'Color', colCons, 'LineWidth', 1.5);
    ylabel(angLabels{i}); grid on; box on;
    if i == 1
        title('Attitude Tracking vs Time');
        legend('Reference', 'Aggressive LQR', 'Conservative LQR', 'Location', 'bestoutside');
    end
end
xlabel('Time [s]');
saveas(f3, fullfile(outDir, 'attitude_comparison.png'));

%% Figure 4: rotor commands
f4 = figure('Name', 'Rotor commands', 'Color', 'w', 'Position', [100 100 900 600]);
subplot(2,1,1);
plot(agg.t(1:end-1), agg.U', 'LineWidth', 1.1); grid on; box on;
yline(p.uHover, ':k'); ylabel('u_i [rad^2/s^2]');
title('Aggressive LQR Rotor Commands'); ylim([0, p.uMax]);
subplot(2,1,2);
plot(cons.t(1:end-1), cons.U', 'LineWidth', 1.1); grid on; box on;
yline(p.uHover, ':k'); ylabel('u_i [rad^2/s^2]'); xlabel('Time [s]');
title('Conservative LQR Rotor Commands'); ylim([0, p.uMax]);
saveas(f4, fullfile(outDir, 'rotor_commands_comparison.png'));

%% Metrics: RMSE tracking error + control effort
posErrAgg  = agg.X(1:3,:)  - agg.Ref(1:3,:);
posErrCons = cons.X(1:3,:) - cons.Ref(1:3,:);

metrics.rmse.Aggressive  = sqrt(mean(posErrAgg(:).^2));
metrics.rmse.Conservative = sqrt(mean(posErrCons(:).^2));
metrics.rmsePerAxis.Aggressive  = sqrt(mean(posErrAgg.^2, 2))';
metrics.rmsePerAxis.Conservative = sqrt(mean(posErrCons.^2, 2))';

effortAgg  = mean(vecnorm(agg.U  - p.uHover, 2, 1));
effortCons = mean(vecnorm(cons.U - p.uHover, 2, 1));
metrics.controlEffort.Aggressive  = effortAgg;
metrics.controlEffort.Conservative = effortCons;

%% Figure 5: RMSE bar chart summary
f5 = figure('Name', 'Tracking error summary', 'Color', 'w', 'Position', [100 100 650 450]);
bar(categorical({'x','y','z'}), [metrics.rmsePerAxis.Aggressive; metrics.rmsePerAxis.Conservative]');
grid on; box on;
ylabel('Position RMSE [m]');
title('Tracking Error Summary: Aggressive vs Conservative LQR');
legend('Aggressive LQR', 'Conservative LQR', 'Location', 'best');
saveas(f5, fullfile(outDir, 'rmse_summary.png'));

fprintf('\n=== Tracking performance summary ===\n');
fprintf('%-20s %15s %15s\n', '', 'Aggressive', 'Conservative');
fprintf('%-20s %15.4f %15.4f   [m]\n', 'Overall RMSE', metrics.rmse.Aggressive, metrics.rmse.Conservative);
fprintf('%-20s %15.4f %15.4f   [m]\n', 'RMSE x', metrics.rmsePerAxis.Aggressive(1), metrics.rmsePerAxis.Conservative(1));
fprintf('%-20s %15.4f %15.4f   [m]\n', 'RMSE y', metrics.rmsePerAxis.Aggressive(2), metrics.rmsePerAxis.Conservative(2));
fprintf('%-20s %15.4f %15.4f   [m]\n', 'RMSE z', metrics.rmsePerAxis.Aggressive(3), metrics.rmsePerAxis.Conservative(3));
fprintf('%-20s %15.1f %15.1f   [rad^2/s^2]\n', 'Mean control effort', effortAgg, effortCons);

end
