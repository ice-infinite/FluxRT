function report = plot_foc_stability(varargin)
%PLOT_FOC_STABILITY Run and plot the current closed-loop stability baseline.
ip = inputParser;
ip.addParameter('duration',10,@(x)isnumeric(x)&&isscalar(x)&&x>3);
ip.addParameter('outputDir',fullfile(fileparts(mfilename('fullpath')),'results'), ...
    @(s)ischar(s)||isstring(s));
ip.parse(varargin{:});
o = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(here);
outputDir = char(o.outputDir);
if ~exist(outputDir,'dir'), mkdir(outputDir); end

ideal = run_foc_sim('closedLoop',true,'duration',o.duration, ...
    'enableDeadTime',false);
if bdIsLoaded('foc_bringup'), close_system('foc_bringup',0); end
deadtime = run_foc_sim('closedLoop',true,'duration',o.duration, ...
    'enableDeadTime',true,'deadTimeNs',550);

steadyStart = max(2.5,o.duration-2.0);
report = struct();
report.generatedAt = char(datetime('now','Format','yyyy-MM-dd''T''HH:mm:ss'));
report.durationS = o.duration;
report.steadyWindowS = [steadyStart o.duration];
report.ideal = stability_metrics(ideal,steadyStart);
report.deadtime550ns = stability_metrics(deadtime,steadyStart);

% Plot at 1 kHz. All statistics above retain the native 12 kHz samples.
idealPlot = decimate_run(ideal,16);
deadPlot = decimate_run(deadtime,16);
t = deadPlot.time;
transitionIndex = find(deadPlot.state==6,1);
closedIndex = find(deadPlot.state==7,1);
transitionTime = NaN; closedTime = NaN;
if ~isempty(transitionIndex), transitionTime = t(transitionIndex); end
if ~isempty(closedIndex), closedTime = t(closedIndex); end

fig = figure('Visible','off','Color','w','Position',[50 50 1700 1050]);
layout = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(layout,1);
plot(ax1,t,deadPlot.trueSpeedRpm,'LineWidth',1.35); hold(ax1,'on');
plot(ax1,t,deadPlot.obsSpeedRpm,'LineWidth',1.0);
plot(ax1,t,deadPlot.speedReferenceRpm,'--','LineWidth',1.1);
yline(ax1,deadtime.p.timing.targetRpm,':','Target 582 rpm','LineWidth',1.0);
mark_handoff(ax1,transitionTime,closedTime);
grid(ax1,'on'); ylabel(ax1,'Speed (rpm)'); title(ax1,'550 ns dead-time: speed');
legend(ax1,{'plant truth','observer','speed reference','target'},'Location','southeast');

ax2 = nexttile(layout,2);
plot(ax2,t,deadPlot.iqRef,'LineWidth',1.25); hold(ax2,'on');
plot(ax2,t,deadPlot.iqMeas,'LineWidth',0.9);
plot(ax2,t,deadPlot.idMeas,'LineWidth',0.9);
mark_handoff(ax2,transitionTime,closedTime);
grid(ax2,'on'); ylabel(ax2,'Current (A)'); title(ax2,'Current-loop tracking');
legend(ax2,{'Iq ref','Iq measured','Id measured'},'Location','northeast');

ax3 = nexttile(layout,3);
speedError = deadPlot.obsSpeedRpm-deadPlot.trueSpeedRpm;
plot(ax3,t,speedError,'LineWidth',1.0); hold(ax3,'on'); yline(ax3,0,'k:');
mark_handoff(ax3,transitionTime,closedTime);
grid(ax3,'on'); ylabel(ax3,'Error (rpm)'); title(ax3,'Observer speed error');

ax4 = nexttile(layout,4);
observerAngleError = wrap_pi(deadPlot.obsAngle-deadPlot.trueAngle);
controlAngleError = wrap_pi(deadPlot.controlAngle-deadPlot.trueAngle);
plot(ax4,t,observerAngleError,'LineWidth',0.9); hold(ax4,'on');
plot(ax4,t,controlAngleError,'LineWidth',1.1); yline(ax4,0,'k:');
mark_handoff(ax4,transitionTime,closedTime);
grid(ax4,'on'); ylabel(ax4,'Error (rad)'); title(ax4,'Electrical-angle error');
legend(ax4,{'observer - truth','control - truth'},'Location','southeast');

ax5 = nexttile(layout,5);
yyaxis(ax5,'left'); stairs(ax5,t,deadPlot.state,'LineWidth',1.2);
ylabel(ax5,'State'); ylim(ax5,[2.5 8.5]); yticks(ax5,3:8);
yyaxis(ax5,'right'); stairs(ax5,t,deadPlot.reliable,'LineWidth',1.0);
hold(ax5,'on'); stairs(ax5,t,deadPlot.faultFlags,'--','LineWidth',1.0);
ylabel(ax5,'Reliable / fault'); ylim(ax5,[-0.05 1.1]);
mark_handoff(ax5,transitionTime,closedTime);
grid(ax5,'on'); xlabel(ax5,'Time (s)'); title(ax5,'State, reliability and faults');
legend(ax5,{'state','observer reliable','fault flags'},'Location','southeast');

ax6 = nexttile(layout,6);
plot(ax6,idealPlot.time,idealPlot.trueSpeedRpm,'LineWidth',1.2); hold(ax6,'on');
plot(ax6,deadPlot.time,deadPlot.trueSpeedRpm,'LineWidth',1.2);
plot(ax6,deadPlot.time,deadPlot.obsSpeedRpm,'--','LineWidth',0.9);
yline(ax6,deadtime.p.timing.targetRpm,':','LineWidth',1.0);
grid(ax6,'on'); xlabel(ax6,'Time (s)'); ylabel(ax6,'Speed (rpm)');
title(ax6,'Ideal inverter vs averaged dead-time');
legend(ax6,{'ideal truth','550 ns truth','550 ns observer','target'}, ...
    'Location','southeast');

titleText = sprintf(['Closed-loop stability | steady %.1f-%.1f s | ' ...
    'dead-time speed %.3f +/- %.3f rpm | Iq RMSE %.5f A | faults 0x%X'], ...
    steadyStart,o.duration,report.deadtime550ns.trueSpeedMeanRpm, ...
    report.deadtime550ns.trueSpeedStdRpm,report.deadtime550ns.iqTrackingRmseA, ...
    uint32(report.deadtime550ns.finalFaultFlags));
title(layout,titleText,'FontWeight','bold');

pngPath = fullfile(outputDir,'foc_closed_loop_stability_10s.png');
figPath = fullfile(outputDir,'foc_closed_loop_stability_10s.fig');
matPath = fullfile(outputDir,'foc_closed_loop_stability_10s.mat');
jsonPath = fullfile(outputDir,'foc_closed_loop_stability_10s.json');
exportgraphics(fig,pngPath,'Resolution',180);
savefig(fig,figPath);
trace = struct('ideal',idealPlot,'deadtime550ns',deadPlot);
save(matPath,'report','trace');
fid = fopen(jsonPath,'w');
if fid < 0, error('plot_foc_stability:write','Cannot write %s.',jsonPath); end
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
fclose(fid);
close(fig);
if bdIsLoaded('foc_bringup'), close_system('foc_bringup',0); end

report.files = struct('png',pngPath,'fig',figPath,'mat',matPath,'json',jsonPath);
fprintf('Wrote %s\n',pngPath);
fprintf('Steady true speed %.4f +/- %.4f rpm; observer error RMSE %.4f rpm; Iq RMSE %.6f A; faults=0x%X\n', ...
    report.deadtime550ns.trueSpeedMeanRpm,report.deadtime550ns.trueSpeedStdRpm, ...
    report.deadtime550ns.observerSpeedRmseRpm,report.deadtime550ns.iqTrackingRmseA, ...
    uint32(report.deadtime550ns.finalFaultFlags));
end

function m = stability_metrics(r,startTime)
mask = r.time>=startTime;
m = struct();
m.sampleCount = sum(mask);
m.finalState = r.state(end);
m.finalFaultFlags = r.faultFlags(end);
m.transitionTimeS = first_time(r,6);
m.closedLoopTimeS = first_time(r,7);
m.trueSpeedMeanRpm = mean(r.trueSpeedRpm(mask));
m.trueSpeedStdRpm = std(r.trueSpeedRpm(mask));
m.trueSpeedPeakToPeakRpm = max(r.trueSpeedRpm(mask))-min(r.trueSpeedRpm(mask));
m.observerSpeedMeanRpm = mean(r.obsSpeedRpm(mask));
m.observerSpeedStdRpm = std(r.obsSpeedRpm(mask));
m.observerSpeedRmseRpm = sqrt(mean((r.obsSpeedRpm(mask)-r.trueSpeedRpm(mask)).^2));
m.iqReferenceMeanA = mean(r.iqRef(mask));
m.iqTrackingRmseA = sqrt(mean((r.iqMeas(mask)-r.iqRef(mask)).^2));
m.idTrackingRmseA = sqrt(mean(r.idMeas(mask).^2));
m.peakPhaseCurrentA = max(abs(r.phaseCurrents),[],'all');
m.maximumIqStepA = max(abs(diff(r.iqRef)));
m.reliableFraction = mean(r.reliable(mask)~=0);
end

function t = first_time(r,stateValue)
index = find(r.state==stateValue,1);
if isempty(index), t = NaN; else, t = r.time(index); end
end

function out = decimate_run(r,divider)
index = 1:divider:numel(r.time);
names = {'time','trueSpeedRpm','obsSpeedRpm','speedReferenceRpm','iqRef', ...
    'iqMeas','idMeas','obsAngle','trueAngle','controlAngle','state', ...
    'reliable','faultFlags'};
out = struct();
for k = 1:numel(names)
    value = r.(names{k});
    out.(names{k}) = value(index,:);
end
end

function y = wrap_pi(x)
y = mod(x+pi,2*pi)-pi;
end

function mark_handoff(ax,transitionTime,closedTime)
if isfinite(transitionTime), xline(ax,transitionTime,'--','transition'); end
if isfinite(closedTime), xline(ax,closedTime,'--','closed'); end
end
