function report = plot_foc_stability(varargin)
%PLOT_FOC_STABILITY Run and plot the current closed-loop stability baseline.
%
% FluxRT —— 生成当前闭环稳定性基线图与 JSON 统计（结果是仿真，不是实机结论）。
% FluxRT - produces the current closed-loop stability baseline figure and JSON statistics.
%
% 职责 / Responsibility:
%   - 跑两组仿真：理想逆变器（enableDeadTime=false，对应 Rust 主机 plant 的逐拍对拍）
%     和 550 ns 平均死区（enableDeadTime=true，对应功率板趋势），输出 6 联图对比；
%   - 统计量在 12 kHz 原始样本上计算，只有画图才抽取到 1 kHz（decimate 16）。
%   - Runs two cases, an ideal inverter (enableDeadTime=false, the tick-by-tick Rust host
%     plant) and 550 ns average dead time (enableDeadTime=true, the board trend), and plots
%     them side by side. All statistics use the native 12 kHz samples; only the plotted
%     traces are decimated to 1 kHz.
%
% 单位 / Units: 转速 [rpm]、电流 [A]、角度 [rad]、时间 [s]、占空比/状态/可靠标志 [-]。
% 关键判据：稳态窗口 trueSpeedMeanRpm/StdRpm [rpm]、observerSpeedRmseRpm [rpm]、
% iqTrackingRmseA/idTrackingRmseA [A]、peakPhaseCurrentA [A]、finalFaultFlags [-]。
% Speeds in [rpm], currents in [A], angles in [rad], time in [s]; state, reliability and duty
% are dimensionless. The reported metrics carry the units listed above.
%
% 边界 / Boundary: 纯仿真，不接触串口与功率级；输出的 PNG/FIG/MAT/JSON 文件名固定为
% `..._10s.*`（与默认 duration=10 对应），改 duration 不会改变文件名。
% Pure simulation: no serial port, no power stage. The exported PNG/FIG/MAT/JSON names are
% fixed as `..._10s.*` to match the default duration of 10 s even if duration is changed.
%
% 参考 / Reference: simulink/Simulink仿真工程说明.md, docs/仿真实机相关性验证.md

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

% 两组工况 / Two cases: 理想逆变器与 550 ns 平均死区。理想组用于和 Rust 主机 plant 对拍
% （两边都是理想平均逆变器），死区组用于看功率板的实际趋势；两组不能混在同一张"精确
% 一致"表里，这是本目录一直强调的边界。
% The ideal case is for tick-by-tick comparison with the Rust host plant (both use an ideal
% average-value inverter) and the dead-time case shows the board trend. The two must not be
% mixed into one "exact match" table.
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
% 画图抽取到 1 kHz（12 kHz / 16）：只影响可读性，所有上面的统计量仍用 12 kHz 原始样本，
% 因此图上"看起来平滑"与统计的数值精度无关。
% The plotted traces are decimated to 1 kHz (12 kHz / 16). This is a readability choice only:
% every statistic above still uses the native 12 kHz samples, so a smooth-looking plot says
% nothing about numerical accuracy.
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
% 计算稳态窗口 [startTime, end] 内的统计量（startTime [s]，其余见字段名量纲）。
% 未辨识/未验证的量不要从这里外推：trueSpeedRpm 是 plant 真值，实机没有对应测量。
% Computes statistics over the steady window [startTime, end] (startTime in [s]; other units
% follow the field names). Do not extrapolate from here to hardware: trueSpeedRpm is plant
% truth and has no counterpart measurement on the rig.
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
% 首次进入某状态的时刻 [s]；没进入过返回 NaN，调用方用 isfinite 判断后再画线。
% First time [s] the run enters a given state, NaN if it never does; callers test with
% isfinite before drawing a marker.
index = find(r.state==stateValue,1);
if isempty(index), t = NaN; else, t = r.time(index); end
end

function out = decimate_run(r,divider)
% 按 divider 抽取结果结构体（divider [-] 为整数倍数，本文件传 16 → 1 kHz 图线）。
% 抽取只用于绘图，且所有被抽取字段都来自同一次运行，时间轴保持一致。
% Decimates the result struct by an integer factor (16 here, i.e. 1 kHz plot lines). Used for
% plotting only, and every decimated field comes from the same run so the time base stays
% consistent.
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
% 把角度 [rad] 折到 (-pi, pi]，用于画"误差"曲线时避免 ±2*pi 跳变。
% 注意这里是本文件的局部实现（+foc/wrap_pi.m 是另一份），两者语义相同，改动需同步。
% Wraps an angle [rad] into (-pi, pi] so error curves do not jump by ±2*pi. This is a local
% copy (the package +foc/wrap_pi.m is a separate one); keep both in sync.
y = mod(x+pi,2*pi)-pi;
end

function mark_handoff(ax,transitionTime,closedTime)
% 在图上标出接管时刻 [s]（状态 6 与状态 7 的首次进入）；NaN 表示该状态没出现过，跳过。
% Marks the handoff instants [s] (first entry into states 6 and 7); NaN means the state never
% occurred and the marker is skipped.
if isfinite(transitionTime), xline(ax,transitionTime,'--','transition'); end
if isfinite(closedTime), xline(ax,closedTime,'--','closed'); end
end
