function report = plot_deadtime_compensation(varargin)
%PLOT_DEADTIME_COMPENSATION Compare ideal, uncompensated and compensated runs.
% This is a simulation-only qualification step. It does not change firmware.

ip = inputParser;
ip.addParameter('duration',10,@(x)isnumeric(x)&&isscalar(x)&&x>=7);
ip.addParameter('deadTimeNs',550,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('compensationGain',1.0,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('currentBandA',0.005,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('outputDir',fullfile(fileparts(mfilename('fullpath')),'results'), ...
    @(s)ischar(s)||isstring(s));
ip.parse(varargin{:});
o = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(here);
outputDir = char(o.outputDir);
if ~exist(outputDir,'dir'), mkdir(outputDir); end

ideal = run_case(false,false,false,o);
uncompensated = run_case(true,false,false,o);
compensated = run_case(true,true,true,o);

steadyStart = o.duration-2;
segment = [4 5];
report = struct();
report.generatedAt = char(datetime('now','Format','yyyy-MM-dd''T''HH:mm:ss'));
report.durationS = o.duration;
report.deadTimeNs = o.deadTimeNs;
report.compensationGain = o.compensationGain;
report.currentBandA = o.currentBandA;
report.deadTimeVoltageAtNominalBusV = ...
    compensated.p.inverter.deadTimeVoltageAtNominalBusV;
report.steadyWindowS = [steadyStart o.duration];
report.transientWindowS = segment;
report.ideal = metrics(ideal,steadyStart,segment);
report.uncompensated = metrics(uncompensated,steadyStart,segment);
report.compensated = metrics(compensated,steadyStart,segment);
report.improvement = improvement(report.uncompensated,report.compensated);

idealPlot = decimate_run(ideal,16);
basePlot = decimate_run(uncompensated,16);
compPlot = decimate_run(compensated,16);

fig = figure('Visible','off','Color','w','Position',[50 50 1700 1050]);
layout = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(layout,1);
plot(ax1,idealPlot.time,idealPlot.trueSpeedRpm,'LineWidth',1.0); hold(ax1,'on');
plot(ax1,basePlot.time,basePlot.trueSpeedRpm,'LineWidth',1.15);
plot(ax1,compPlot.time,compPlot.trueSpeedRpm,'LineWidth',1.25);
yline(ax1,compensated.p.timing.targetRpm,'k:','target');
grid(ax1,'on'); ylabel(ax1,'Speed (rpm)'); title(ax1,'Full speed response');
legend(ax1,{'ideal','550 ns, no compensation','550 ns, compensated','target'}, ...
    'Location','southeast');

ax2 = nexttile(layout,2);
maskBase = basePlot.time>=3.5 & basePlot.time<=6.5;
maskComp = compPlot.time>=3.5 & compPlot.time<=6.5;
plot(ax2,basePlot.time(maskBase),basePlot.trueSpeedRpm(maskBase),'LineWidth',1.15); hold(ax2,'on');
plot(ax2,compPlot.time(maskComp),compPlot.trueSpeedRpm(maskComp),'LineWidth',1.25);
yline(ax2,compensated.p.timing.targetRpm,'k:','target');
xline(ax2,4,'k--'); xline(ax2,5,'k--');
grid(ax2,'on'); ylabel(ax2,'Speed (rpm)'); title(ax2,'3.5-6.5 s zoom');
legend(ax2,{'no compensation','compensated','target'},'Location','southeast');

ax3 = nexttile(layout,3);
plot(ax3,basePlot.time,basePlot.obsSpeedRpm-basePlot.trueSpeedRpm,'LineWidth',0.9); hold(ax3,'on');
plot(ax3,compPlot.time,compPlot.obsSpeedRpm-compPlot.trueSpeedRpm,'LineWidth',1.1);
yline(ax3,0,'k:'); grid(ax3,'on'); ylabel(ax3,'Error (rpm)');
title(ax3,'Observer speed error');
legend(ax3,{'no compensation','compensated'},'Location','southeast');

ax4 = nexttile(layout,4);
plot(ax4,basePlot.time,basePlot.iqMeas-basePlot.iqRef,'LineWidth',0.85); hold(ax4,'on');
plot(ax4,compPlot.time,compPlot.iqMeas-compPlot.iqRef,'LineWidth',1.0);
yline(ax4,0,'k:'); grid(ax4,'on'); ylabel(ax4,'Iq error (A)');
title(ax4,'Current-loop tracking error');
legend(ax4,{'no compensation','compensated'},'Location','northeast');

ax5 = nexttile(layout,5);
plot(ax5,basePlot.time,basePlot.idMeas,'LineWidth',0.85); hold(ax5,'on');
plot(ax5,compPlot.time,compPlot.idMeas,'LineWidth',1.0);
yline(ax5,0,'k:'); grid(ax5,'on'); xlabel(ax5,'Time (s)'); ylabel(ax5,'Id (A)');
title(ax5,'D-axis disturbance');
legend(ax5,{'no compensation','compensated'},'Location','northeast');

ax6 = nexttile(layout,6);
yyaxis(ax6,'left'); stairs(ax6,compPlot.time,compPlot.state,'LineWidth',1.1);
ylabel(ax6,'State'); ylim(ax6,[2.5 8.5]); yticks(ax6,3:8);
yyaxis(ax6,'right'); stairs(ax6,compPlot.time,compPlot.reliable,'LineWidth',1.0); hold(ax6,'on');
stairs(ax6,compPlot.time,compPlot.faultFlags,'--','LineWidth',1.0);
ylabel(ax6,'Reliable / fault'); ylim(ax6,[-0.05 1.1]);
grid(ax6,'on'); xlabel(ax6,'Time (s)'); title(ax6,'Compensated state and protection');
legend(ax6,{'state','observer reliable','fault flags'},'Location','southeast');

title(layout,sprintf([ ...
    'Dead-time compensation | %.0f ns / %.4f A band / gain %.2f | ' ...
    '4-5 s speed RMSE %.3f -> %.3f rpm | observer RMSE %.3f -> %.3f rpm'], ...
    o.deadTimeNs,o.currentBandA,o.compensationGain, ...
    report.uncompensated.transientSpeedTargetRmseRpm, ...
    report.compensated.transientSpeedTargetRmseRpm, ...
    report.uncompensated.transientObserverRmseRpm, ...
    report.compensated.transientObserverRmseRpm),'FontWeight','bold');

pngPath = fullfile(outputDir,'foc_deadtime_compensation_10s.png');
figPath = fullfile(outputDir,'foc_deadtime_compensation_10s.fig');
matPath = fullfile(outputDir,'foc_deadtime_compensation_10s.mat');
jsonPath = fullfile(outputDir,'foc_deadtime_compensation_10s.json');
exportgraphics(fig,pngPath,'Resolution',180);
savefig(fig,figPath);
trace = struct('ideal',idealPlot,'uncompensated',basePlot,'compensated',compPlot);
save(matPath,'report','trace');
fid = fopen(jsonPath,'w');
if fid < 0, error('plot_deadtime_compensation:write','Cannot write %s.',jsonPath); end
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
fclose(fid);
close(fig);
if bdIsLoaded('foc_bringup'), close_system('foc_bringup',0); end

report.files = struct('png',pngPath,'fig',figPath,'mat',matPath,'json',jsonPath);
fprintf('Wrote %s\n',pngPath);
fprintf(['4-5 s target-speed RMSE %.4f -> %.4f rpm; observer RMSE %.4f -> %.4f rpm; ' ...
    'steady Iq RMSE %.6f -> %.6f A; faults 0x%X -> 0x%X\n'], ...
    report.uncompensated.transientSpeedTargetRmseRpm, ...
    report.compensated.transientSpeedTargetRmseRpm, ...
    report.uncompensated.transientObserverRmseRpm, ...
    report.compensated.transientObserverRmseRpm, ...
    report.uncompensated.steadyIqTrackingRmseA, ...
    report.compensated.steadyIqTrackingRmseA, ...
    uint32(report.uncompensated.finalFaultFlags), ...
    uint32(report.compensated.finalFaultFlags));
end

function r = run_case(deadTime,feedForward,observerComp,o)
if bdIsLoaded('foc_bringup'), close_system('foc_bringup',0); end
r = run_foc_sim('closedLoop',true,'duration',o.duration, ...
    'enableDeadTime',deadTime,'deadTimeNs',o.deadTimeNs, ...
    'enableDeadTimeCompensation',feedForward, ...
    'enableObserverDeadTimeCompensation',observerComp, ...
    'deadTimeCompensationGain',o.compensationGain, ...
    'deadTimeCompensationCurrentBandA',o.currentBandA);
end

function m = metrics(r,steadyStart,segment)
steady = r.time>=steadyStart;
transient = r.time>=segment(1) & r.time<=segment(2);
target = r.p.timing.targetRpm;
m = struct();
m.finalState = r.state(end);
m.finalFaultFlags = max(r.faultFlags);
m.closedLoopTimeS = first_time(r,7);
m.steadyTrueSpeedMeanRpm = mean(r.trueSpeedRpm(steady));
m.steadyTrueSpeedStdRpm = std(r.trueSpeedRpm(steady));
m.steadyTrueSpeedPeakToPeakRpm = ...
    max(r.trueSpeedRpm(steady))-min(r.trueSpeedRpm(steady));
m.steadySpeedTargetRmseRpm = sqrt(mean((r.trueSpeedRpm(steady)-target).^2));
m.steadyObserverRmseRpm = sqrt(mean((r.obsSpeedRpm(steady)-r.trueSpeedRpm(steady)).^2));
m.steadyIqTrackingRmseA = sqrt(mean((r.iqMeas(steady)-r.iqRef(steady)).^2));
m.steadyIdRmseA = sqrt(mean(r.idMeas(steady).^2));
m.transientTrueSpeedMinRpm = min(r.trueSpeedRpm(transient));
m.transientTrueSpeedMaxRpm = max(r.trueSpeedRpm(transient));
m.transientSpeedTargetRmseRpm = sqrt(mean((r.trueSpeedRpm(transient)-target).^2));
m.transientObserverRmseRpm = sqrt(mean((r.obsSpeedRpm(transient)-r.trueSpeedRpm(transient)).^2));
m.reliableFraction = mean(r.reliable(steady)~=0);
end

function out = improvement(base,comp)
out = struct();
out.transientSpeedTargetRmsePercent = reduction(base.transientSpeedTargetRmseRpm,comp.transientSpeedTargetRmseRpm);
out.transientObserverRmsePercent = reduction(base.transientObserverRmseRpm,comp.transientObserverRmseRpm);
out.steadySpeedStdPercent = reduction(base.steadyTrueSpeedStdRpm,comp.steadyTrueSpeedStdRpm);
out.steadyIqTrackingRmsePercent = reduction(base.steadyIqTrackingRmseA,comp.steadyIqTrackingRmseA);
out.steadyIdRmsePercent = reduction(base.steadyIdRmseA,comp.steadyIdRmseA);
end

function value = reduction(before,after)
value = 100*(before-after)/max(before,eps);
end

function t = first_time(r,stateValue)
index = find(r.state==stateValue,1);
if isempty(index), t = NaN; else, t = r.time(index); end
end

function out = decimate_run(r,divider)
index = 1:divider:numel(r.time);
names = {'time','trueSpeedRpm','obsSpeedRpm','iqRef','iqMeas','idMeas', ...
    'state','reliable','faultFlags'};
out = struct();
for k = 1:numel(names)
    out.(names{k}) = r.(names{k})(index,:);
end
end
