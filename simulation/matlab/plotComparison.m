function metrics = plotComparison(outputs, p, outDir, rampTime, selected)
%PLOTCOMPARISON Export selected figures; use {'all'} for local diagnostics.
if nargin < 5
    selected = {'all'};
end
validNames = {'all','xy_tracking_comparison','xyz_tracking_comparison', ...
    'attitude_comparison','rotor_commands_comparison','rmse_summary', ...
    'tracking_error_comparison','diagnostics','estimation_errors','imu_measurements'};
if ~iscellstr(selected) || any(~ismember(selected,validNames))
    error('plotComparison:Selection','Selection must contain supported figure names or ''all''.');
end
include = @(name) any(strcmp(selected,'all')) || any(strcmp(selected,name));
names = {'LQR','MPC','PID'};
colors = [0 0.35 0.65;0.85 0.28 0.05;0.12 0.55 0.28];
styles = {'-','--','-.'};
t = outputs{1}.t;
ref = outputs{1}.Ref;
caption = outputs{1}.scenario;
errors = cell(1,3);
for j = 1:3
    if ~isequal(outputs{j}.t,t) || ~isequal(outputs{j}.Ref,ref)
        error('plotComparison:Comparison','Controller time grids and references must match.');
    end
    metrics.(names{j}) = trackingMetrics(outputs{j},p,rampTime);
    errors{j} = outputs{j}.X-ref;
end
if ~isempty(selected) && ~exist(outDir,'dir')
    mkdir(outDir);
end

if include('xy_tracking_comparison')
    [f,layout] = canvas([1240 600],1,2,[caption ' | circle tracking']);
    ax = nexttile(layout);
    pathLines(ax,outputs,ref,colors,styles);
    axis(ax,'equal'); xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
    title(ax,'Complete path (including ramp)');
    legendAbove(ax,[{'Reference'},names]);
    ax = nexttile(layout);
    pathLines(ax,outputs,ref,colors,styles);
    separation = zeros(size(t));
    for j = 1:3
        separation = max(separation,vecnorm(errors{j}(1:2,:),2,1));
    end
    separation(t < rampTime) = -inf;
    [~,k] = max(separation);
    points = [ref(1:2,k),outputs{1}.X(1:2,k),outputs{2}.X(1:2,k),outputs{3}.X(1:2,k)];
    center = mean(points,2);
    halfWidth = max(0.05,1.2*max(abs(points-center),[],'all'));
    axis(ax,'equal'); xlim(ax,center(1)+[-1 1]*halfWidth); ylim(ax,center(2)+[-1 1]*halfWidth);
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
    title(ax,sprintf('Detail near largest horizontal error, t = %.2f s',t(k)));
    saveFigure(f,outDir,'xy_tracking_comparison');
end

if include('xyz_tracking_comparison')
    [f,layout] = canvas([1240 940],3,2,[caption ' | position and signed tracking errors']);
    for i = 1:3
        ax = nexttile(layout);
        referenceLine(ax,t,ref(i,:));
        values = cellfun(@(o) o.X(i,:),outputs,'UniformOutput',false);
        controllerLines(ax,t,values,colors,styles);
        ylabel(ax,sprintf('%s [m]',char('x'+i-1)));
        if i == 1
            title(ax,'True position'); legendAbove(ax,[{'Reference'},names]);
        end
        ax = nexttile(layout);
        values = cellfun(@(e) 100*e(i,:),errors,'UniformOutput',false);
        controllerLines(ax,t,values,colors,styles);
        ylabel(ax,sprintf('e_%s [cm]',char('x'+i-1)));
        yline(ax,0,':k','HandleVisibility','off');
        if i == 1
            title(ax,'True position - reference');
        end
    end
    saveFigure(f,outDir,'xyz_tracking_comparison');
end

labels = {'Roll','Pitch','Yaw'};
if include('attitude_comparison')
    [f,layout] = canvas([1240 940],3,2,[caption ' | attitude and signed tracking errors']);
    for i = 1:3
        ax = nexttile(layout);
        referenceLine(ax,t,rad2deg(ref(3+i,:)));
        values = cellfun(@(o) rad2deg(o.X(3+i,:)),outputs,'UniformOutput',false);
        controllerLines(ax,t,values,colors,styles);
        ylabel(ax,[labels{i} ' [deg]']);
        if i == 1
            title(ax,'True attitude'); legendAbove(ax,[{'Reference'},names]);
        end
        ax = nexttile(layout);
        values = cellfun(@(e) rad2deg(e(3+i,:)),errors,'UniformOutput',false);
        controllerLines(ax,t,values,colors,styles);
        ylabel(ax,[labels{i} ' error [deg]']);
        if i == 1
            title(ax,'True attitude - reference');
        end
    end
    saveFigure(f,outDir,'attitude_comparison');
end

if include('rotor_commands_comparison')
    [f,layout] = canvas([1240 1080],4,2,[caption ' | rotor squared-speed deviations']);
    for i = 1:4
        ax = nexttile(layout);
        values = cellfun(@(o) 100*(o.U(i,:)/p.uHover-1),outputs,'UniformOutput',false);
        controllerLines(ax,t(1:end-1),values,colors,styles);
        ylabel(ax,sprintf('Rotor %d [%% hover]',i));
        if i == 1
            title(ax,'Requested inputs (after shared bounds)');
            legendAbove(ax,names);
        end
        ax = nexttile(layout);
        values = cellfun(@(o) 100*(o.UApplied(i,:)/p.uHover-1),outputs,'UniformOutput',false);
        controllerLines(ax,t,values,colors,styles);
        ylabel(ax,sprintf('Rotor %d [%% hover]',i));
        if i == 1
            title(ax,'Applied motor inputs (includes lag)');
        end
    end
    saveFigure(f,outDir,'rotor_commands_comparison');
end

if include('rmse_summary')
    [f,layout] = canvas([1240 600],1,2,[caption ' | physical-distance tracking metrics']);
    ax = nexttile(layout);
    data = zeros(3,3);
    for j = 1:3
        data(:,j) = 100*metrics.(names{j}).rmsePerAxis';
    end
    metricBars(ax,data,{'x','y','z'},colors);
    ylabel(ax,'Per-axis RMSE [cm]'); title(ax,'Whole run, including ramp');
    legendAbove(ax,names);
    ax = nexttile(layout);
    for j = 1:3
        m = metrics.(names{j});
        data(:,j) = 100*[m.rmse3D;m.rmseCircle3D;m.maxError3D];
    end
    metricBars(ax,data,{'3D RMS, all','3D RMS, circle','Peak'},colors);
    ylabel(ax,'Euclidean position error [cm]');
    title(ax,sprintf('Circle metrics use t >= %.1f s',rampTime));
    saveFigure(f,outDir,'rmse_summary');
end

if include('tracking_error_comparison')
    [f,layout] = canvas([1240 740],2,1,[caption ' | tracking error magnitudes']);
    for i = 1:2
        ax = nexttile(layout);
        dimensions = 1:3;
        if i == 2
            dimensions = 1:2;
        end
        values = cellfun(@(e) 100*vecnorm(e(dimensions,:),2,1),errors,'UniformOutput',false);
        controllerLines(ax,t,values,colors,styles);
        ylabel(ax,'Position error [cm]');
        if i == 1
            title(ax,'3D Euclidean distance'); legendAbove(ax,names);
        else
            title(ax,'Horizontal distance');
        end
        xline(ax,rampTime,':','Ramp end','HandleVisibility','off');
    end
    saveFigure(f,outDir,'tracking_error_comparison');
end

if include('diagnostics')
    [f,layout] = canvas([1240 780],2,2,[caption ' | disturbance and controller diagnostics']);
    ax = nexttile(layout); styleAxes(ax);
    plot(ax,t,outputs{1}.windForce','LineWidth',1.5);
    xlabel(ax,'Time [s]'); ylabel(ax,'Inertial force [N]');
    title(ax,'Same wind force for all controllers');
    legend(ax,{'F_x','F_y','F_z'},'Location','northoutside','Orientation','horizontal');
    ax = nexttile(layout);
    values = cellfun(@(o) 100*max(abs(diff([p.uHover*ones(4,1),o.U],1,2)),[],1)/p.uHover, ...
        outputs,'UniformOutput',false);
    controllerLines(ax,t(1:end-1),values,colors,styles);
    yline(ax,100*outputs{1}.maxInputStep,':k','Shared limit','HandleVisibility','off');
    ylabel(ax,'Largest command step [% hover]'); title(ax,'Shared input/rate constraints');
    legend(ax,names,'Location','northoutside','Orientation','horizontal');
    ax = nexttile(layout);
    values = cellfun(@(o) 1000*o.solveTime,outputs,'UniformOutput',false);
    controllerLines(ax,t(1:end-1),values,colors,styles);
    ylabel(ax,'Host controller time [ms]'); title(ax,'Excludes EKF and plant integration');
    ax = nexttile(layout); styleAxes(ax);
    stairs(ax,t(1:end-1),outputs{2}.solverIterations,'Color',colors(2,:),'LineWidth',1.5);
    xlabel(ax,'Time [s]'); ylabel(ax,'QP iterations'); title(ax,'MPC: failed solves stop the run');
    saveFigure(f,outDir,'diagnostics');
end

if outputs{1}.estimationEnabled
    if include('estimation_errors')
        [f,layout] = canvas([1240 960],3,2,[caption ' | EKF estimation errors (not tracking errors)']);
        for i = 1:3
            ax = nexttile(layout);
            values = cellfun(@(o) 100*(o.XEstimated(i,:)-o.X(i,:)),outputs,'UniformOutput',false);
            controllerLines(ax,t,values,colors,styles);
            ylabel(ax,sprintf('%s estimate error [cm]',char('x'+i-1)));
            if i == 1
                title(ax,'Estimated - true position'); legendAbove(ax,names);
            end
            ax = nexttile(layout);
            values = cellfun(@(o) rad2deg(atan2(sin(o.XEstimated(3+i,:)-o.X(3+i,:)), ...
                cos(o.XEstimated(3+i,:)-o.X(3+i,:)))),outputs,'UniformOutput',false);
            controllerLines(ax,t,values,colors,styles);
            ylabel(ax,[labels{i} ' estimate error [deg]']);
            if i == 1
                title(ax,'Estimated - true attitude');
            end
        end
        saveFigure(f,outDir,'estimation_errors');
    end

    if include('imu_measurements')
        [f,layout] = canvas([1240 960],3,2,[caption ' | raw body IMU (MPC run, 200 Hz)']);
        sensors = outputs{2}.sensors;
        for i = 1:3
            ax = nexttile(layout); styleAxes(ax);
            plot(ax,sensors.t,sensors.accel(i,:),'Color',[0.65 0.75 0.85],'LineWidth',0.6);
            plot(ax,sensors.t,sensors.accelTruth(i,:),'k','LineWidth',1.2);
            xlabel(ax,'Time [s]'); ylabel(ax,sprintf('f_%s [m/s^2]',char('x'+i-1)));
            if i == 1
                title(ax,'Accelerometer specific force (includes gravity)');
                legendAbove(ax,{'Noisy + biased measurement','Truth (plot only)'});
            end
            ax = nexttile(layout); styleAxes(ax);
            plot(ax,sensors.t,rad2deg(sensors.gyro(i,:)),'Color',[0.65 0.75 0.85],'LineWidth',0.6);
            plot(ax,sensors.t,rad2deg(sensors.gyroTruth(i,:)),'k','LineWidth',1.2);
            xlabel(ax,'Time [s]'); ylabel(ax,sprintf('\\omega_%s [deg/s]',char('x'+i-1)));
            if i == 1
                title(ax,'Gyroscope body angular velocity');
            end
        end
        saveFigure(f,outDir,'imu_measurements');
    end
end
end

function [f,layout] = canvas(sizePixels,rows,columns,heading)
f = figure('Color','w','Name',heading,'Position',[60 60 sizePixels]);
layout = tiledlayout(f,rows,columns,'TileSpacing','compact','Padding','compact');
title(layout,heading,'FontSize',16,'FontWeight','bold','Interpreter','none');
end

function styleAxes(ax)
hold(ax,'on'); grid(ax,'on'); box(ax,'on'); ax.FontSize = 11; ax.LineWidth = 0.8;
end

function controllerLines(ax,t,values,colors,styles)
styleAxes(ax);
for j = 1:3
    plot(ax,t,values{j},styles{j},'Color',colors(j,:),'LineWidth',1.5);
end
xlabel(ax,'Time [s]'); xlim(ax,[t(1) t(end)]);
end

function referenceLine(ax,t,values)
styleAxes(ax); plot(ax,t,values,':','Color',[0.25 0.25 0.25],'LineWidth',1.8);
end

function pathLines(ax,outputs,ref,colors,styles)
styleAxes(ax); plot(ax,ref(1,:),ref(2,:),':k','LineWidth',1.8);
for j = 1:3
    plot(ax,outputs{j}.X(1,:),outputs{j}.X(2,:),styles{j},'Color',colors(j,:),'LineWidth',1.5);
end
end

function metricBars(ax,data,labels,colors)
styleAxes(ax); bars = bar(ax,data);
for j = 1:3
    bars(j).FaceColor = colors(j,:);
    text(ax,bars(j).XEndPoints,bars(j).YEndPoints,compose('%.2f',bars(j).YData), ...
        'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',10);
end
xticks(ax,1:3); xticklabels(ax,labels); ylim(ax,[0,max(data,[],'all')*1.25+0.05]);
end

function legendAbove(ax,labels)
lgd = legend(ax,labels,'Orientation','horizontal','FontSize',11);
lgd.Layout.Tile = 'north';
end

function saveFigure(f,directory,name)
drawnow; exportgraphics(f,fullfile(directory,[name '.png']),'Resolution',180);
end
