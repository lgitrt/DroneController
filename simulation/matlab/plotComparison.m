function metrics = plotComparison(lqrOut, mpcOut, p, outDir, rampTime)
%PLOTCOMPARISON Readable LQR/MPC figures with error panels and diagnostics.
if nargin < 4 || isempty(outDir)
    outDir = fullfile(fileparts(mfilename('fullpath')),'..','results');
end
if nargin < 5
    rampTime = 5;
end
if ~isequal(lqrOut.t,mpcOut.t) || ~isequal(lqrOut.Ref,mpcOut.Ref)
    error('plotComparison:Comparison', 'Time grids and references must be identical.');
end
if ~exist(outDir,'dir')
    mkdir(outDir);
end
blue = [0.00 0.35 0.65];
orange = [0.85 0.28 0.05];
gray = [0.25 0.25 0.25];
t = lqrOut.t;
ref = lqrOut.Ref;
errorLQR = lqrOut.X-ref;
errorMPC = mpcOut.X-ref;
caption = sprintf('%s | LQR vs MPC',lqrOut.scenario);
metrics.LQR = trackingMetrics(lqrOut,p,rampTime);
metrics.MPC = trackingMetrics(mpcOut,p,rampTime);

[f,layout] = canvas([1200 590],1,2,[caption ' - circle tracking']);
ax = nexttile(layout);
trajectory(ax,lqrOut,mpcOut,blue,orange,gray);
axis(ax,'equal');
xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); title(ax,'Complete path (including smooth start)');
legendAbove(ax,{'Reference','LQR','MPC'});
ax = nexttile(layout);
trajectory(ax,lqrOut,mpcOut,blue,orange,gray);
indices = find(t >= rampTime);
[~,idx] = max(vecnorm(lqrOut.X(1:2,indices)-mpcOut.X(1:2,indices),2,1));
k = indices(idx);
points = [ref(1:2,k),lqrOut.X(1:2,k),mpcOut.X(1:2,k)];
center = mean(points,2);
halfWidth = max(0.04,1.2*max(abs(points-center),[],'all'));
axis(ax,'equal');
xlim(ax,center(1)+[-1 1]*halfWidth);
ylim(ax,center(2)+[-1 1]*halfWidth);
xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
title(ax,sprintf('Detail near t = %.2f s (same paths)',t(k)));
saveFigure(f,outDir,'xy_tracking_comparison');

[f,layout] = canvas([1200 950],3,2,[caption ' - position and signed errors']);
for i = 1:3
    ax = nexttile(layout);
    referenceLines(ax,t,ref(i,:),lqrOut.X(i,:),mpcOut.X(i,:),blue,orange,gray);
    ylabel(ax,sprintf('%s [m]',char('x'+i-1)));
    if i == 1
        title(ax,'Position'); legendAbove(ax,{'Reference','LQR','MPC'});
    end
    ax = nexttile(layout);
    pairLines(ax,t,100*errorLQR(i,:),100*errorMPC(i,:),blue,orange);
    yline(ax,0,':','Color',gray);
    ylabel(ax,sprintf('e_%s [cm]',char('x'+i-1)));
    if i == 1
        title(ax,'Actual - reference (independent error scale)');
    end
end
saveFigure(f,outDir,'xyz_tracking_comparison');

[f,layout] = canvas([1200 950],3,2,[caption ' - attitude and signed errors']);
labels = {'Roll','Pitch','Yaw'};
for i = 1:3
    ax = nexttile(layout);
    referenceLines(ax,t,rad2deg(ref(3+i,:)),rad2deg(lqrOut.X(3+i,:)), ...
        rad2deg(mpcOut.X(3+i,:)),blue,orange,gray);
    ylabel(ax,[labels{i} ' [deg]']);
    if i == 1
        title(ax,'Euler angles'); legendAbove(ax,{'Reference','LQR','MPC'});
    end
    ax = nexttile(layout);
    pairLines(ax,t,rad2deg(errorLQR(3+i,:)),rad2deg(errorMPC(3+i,:)),blue,orange);
    yline(ax,0,':','Color',gray);
    ylabel(ax,[labels{i} ' error [deg]']);
    if i == 1
        title(ax,'Actual - reference');
    end
end
saveFigure(f,outDir,'attitude_comparison');

[f,layout] = canvas([1200 760],2,2,[caption ' - rotor input deviations']);
for i = 1:4
    ax = nexttile(layout);
    pairLines(ax,t(1:end-1),100*(lqrOut.U(i,:)/p.uHover-1), ...
        100*(mpcOut.U(i,:)/p.uHover-1),blue,orange);
    plot(ax,t,100*(lqrOut.UApplied(i,:)/p.uHover-1),':','Color',blue,'LineWidth',1);
    plot(ax,t,100*(mpcOut.UApplied(i,:)/p.uHover-1),'-.','Color',orange,'LineWidth',1);
    yline(ax,0,':','Color',gray);
    xlabel(ax,'Time [s]'); ylabel(ax,'Deviation from hover [% of u_{hover}]');
    title(ax,sprintf('Rotor %d | u = squared rotor speed',i));
    if i == 1
        legendAbove(ax,{'LQR command','MPC command','LQR applied','MPC applied'});
    end
end
saveFigure(f,outDir,'rotor_commands_comparison');

[f,layout] = canvas([1200 570],1,2,[caption ' - tracking metrics']);
ax = nexttile(layout);
data = 100*[metrics.LQR.rmsePerAxis;metrics.MPC.rmsePerAxis]';
bars = bar(ax,1:3,data);
bars(1).FaceColor = blue; bars(2).FaceColor = orange;
xticks(ax,1:3); xticklabels(ax,{'x','y','z'});
ylabel(ax,'Per-axis RMSE [cm]'); title(ax,'Whole run, including ramp');
legendAbove(ax,{'LQR','MPC'});
labelBars(ax,bars);
ax = nexttile(layout);
data = 100*[metrics.LQR.rmse3D metrics.MPC.rmse3D; ...
    metrics.LQR.rmseCircle3D metrics.MPC.rmseCircle3D; ...
    metrics.LQR.maxError3D metrics.MPC.maxError3D];
bars = bar(ax,1:3,data);
bars(1).FaceColor = blue; bars(2).FaceColor = orange;
xticks(ax,1:3); xticklabels(ax,{'3D RMS, all','3D RMS, circle','Peak distance'});
ylabel(ax,'Euclidean position error [cm]');
title(ax,sprintf('Circle metrics start at t = %.1f s',rampTime));
labelBars(ax,bars);
saveFigure(f,outDir,'rmse_summary');

[f,layout] = canvas([1200 770],2,1,[caption ' - error magnitudes']);
ax = nexttile(layout);
pairLines(ax,t,100*vecnorm(errorLQR(1:3,:),2,1), ...
    100*vecnorm(errorMPC(1:3,:),2,1),blue,orange);
ylabel(ax,'3D position error [cm]'); title(ax,'Euclidean distance to reference');
legendAbove(ax,{'LQR','MPC'});
xline(ax,rampTime,':','Ramp end','HandleVisibility','off');
ax = nexttile(layout);
pairLines(ax,t,100*vecnorm(errorLQR(1:2,:),2,1), ...
    100*vecnorm(errorMPC(1:2,:),2,1),blue,orange);
ylabel(ax,'Horizontal error [cm]'); title(ax,'XY distance to reference');
saveFigure(f,outDir,'tracking_error_comparison');

[f,layout] = canvas([1200 780],2,2,[caption ' - scenario and solver diagnostics']);
ax = nexttile(layout);
plot(ax,t,lqrOut.windForce','LineWidth',1.6);
xlabel(ax,'Time [s]'); ylabel(ax,'External inertial force [N]');
title(ax,'Same unmeasured force for both controllers');
legend(ax,{'F_x','F_y','F_z'},'Location','northoutside','Orientation','horizontal');
ax = nexttile(layout);
pairLines(ax,t(1:end-1),100*max(abs(diff([p.uHover*ones(4,1),lqrOut.U],1,2)),[],1)/p.uHover, ...
    100*max(abs(diff([p.uHover*ones(4,1),mpcOut.U],1,2)),[],1)/p.uHover,blue,orange);
limit = 100*lqrOut.maxInputStep;
yline(ax,limit,':k',sprintf('Shared %.0f%% step limit',limit));
ylabel(ax,'Largest rotor command step [% of u_{hover}]');
title(ax,'Identical actuator command limits');
legend(ax,{'LQR','MPC'},'Location','northoutside','Orientation','horizontal');
ax = nexttile(layout);
pairLines(ax,t(1:end-1),1000*lqrOut.solveTime,1000*mpcOut.solveTime,blue,orange);
ylabel(ax,'Controller update time [ms]');
title(ax,'Host timing (includes MPC reference preview)');
ax = nexttile(layout);
stairs(ax,t(1:end-1),mpcOut.solverIterations,'Color',orange,'LineWidth',1.5);
xlabel(ax,'Time [s]'); ylabel(ax,'QP iterations');
title(ax,'MPC solver: nonpositive status aborts the run');
saveFigure(f,outDir,'diagnostics');
end

function [f,layout] = canvas(sizePixels,rows,columns,heading)
f = figure('Color','w','Name',heading,'Position',[60 60 sizePixels]);
layout = tiledlayout(f,rows,columns,'TileSpacing','compact','Padding','compact');
title(layout,heading,'FontSize',16,'FontWeight','bold','Interpreter','none');
end

function styleAxes(ax)
hold(ax,'on'); grid(ax,'on'); box(ax,'on');
ax.FontSize = 11;
ax.LineWidth = 0.8;
end

function pairLines(ax,t,a,b,blue,orange)
styleAxes(ax);
plot(ax,t,a,'-','Color',blue,'LineWidth',1.7);
plot(ax,t,b,'--','Color',orange,'LineWidth',1.7);
xlabel(ax,'Time [s]');
xlim(ax,[t(1) t(end)]);
end

function referenceLines(ax,t,r,a,b,blue,orange,gray)
styleAxes(ax);
plot(ax,t,r,':','Color',gray,'LineWidth',1.8);
plot(ax,t,a,'-','Color',blue,'LineWidth',1.5);
plot(ax,t,b,'--','Color',orange,'LineWidth',1.5);
xlabel(ax,'Time [s]');
xlim(ax,[t(1) t(end)]);
end

function trajectory(ax,a,b,blue,orange,gray)
styleAxes(ax);
plot(ax,a.Ref(1,:),a.Ref(2,:),':','Color',gray,'LineWidth',2);
plot(ax,a.X(1,:),a.X(2,:),'-','Color',blue,'LineWidth',1.7);
plot(ax,b.X(1,:),b.X(2,:),'--','Color',orange,'LineWidth',1.7);
end

function legendAbove(ax,labels)
lgd = legend(ax,labels,'Orientation','horizontal','FontSize',11);
lgd.Layout.Tile = 'north';
end

function labelBars(ax,bars)
styleAxes(ax);
for i = 1:numel(bars)
    text(ax,bars(i).XEndPoints,bars(i).YEndPoints, ...
        compose('%.2f',bars(i).YData),'HorizontalAlignment','center', ...
        'VerticalAlignment','bottom','FontSize',11);
end
ylim(ax,[0 max([bars.YData],[],'all')*1.22+0.05]);
end

function saveFigure(f,directory,name)
drawnow;
exportgraphics(f,fullfile(directory,[name '.png']),'Resolution',180);
end
