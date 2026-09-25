function report = plot_deadtime_compensation(varargin)
%PLOT_DEADTIME_COMPENSATION Compare ideal, uncompensated and compensated runs.
% This is a simulation-only qualification step. It does not change firmware.
%
% FluxRT —— 死区补偿的三组对照：理想逆变器 / 550 ns 无补偿 / 550 ns 双层补偿。
% FluxRT - three-way dead-time comparison: ideal inverter, 550 ns uncompensated, 550 ns with
% both compensation layers.
%
% 职责 / Responsibility:
%   - 分别跑理想、无补偿、双层补偿三组闭环仿真，量化 4..5 s 过渡段和稳态窗口的转速、
%     观察器和电流误差，并输出 PNG/FIG/MAT/JSON；
%   - 两层补偿可以独立开关：调制器前馈（按相电流极性在 SVPWM 前补回预计损失）和观察器
%     电压重构补偿（SMO 重构端电压时扣掉同样损失）。
%   - Runs the ideal, uncompensated and fully compensated closed-loop cases, quantifies speed,
%     observer and current errors over the transient and steady windows, and exports
%     PNG/FIG/MAT/JSON. The two compensation layers are independently switchable: modulator
%     feed-forward and the observer's terminal-voltage reconstruction.
%
% 单位 / Units: deadTimeNs [ns]、currentBandA [A]、compensationGain [-]、
% deadTimeVoltageAtNominalBusV [V]（12.3 V / 550 ns / 12 kHz 下约 0.1624 V）、
% duration/窗口 [s]、*RmseRpm [rpm]、*RmseA/steadyIdRmseA [A]、reduction 百分比 [%]。
% deadTimeNs [ns], currentBandA [A], compensationGain [-], deadTimeVoltageAtNominalBusV [V]
% (about 0.1624 V at 12.3 V / 550 ns / 12 kHz), windows in [s], speed RMSE in [rpm], current
% RMSE in [A] and improvements in [%].
%
% 结论的边界 / Limit of the conclusion: 本目录已记录的结果是补偿明显改善电流误差和过渡段
% 速度误差，但稳态速度标准差反而变差，因此**不能**把补偿描述成"所有指标都改善"。
% 550 ns 与 5 mA 目前只是仿真名义值，实机首次试验应从 0.75 或更低增益开始。
% The recorded result in this folder is that compensation clearly improves current error and
% transient speed error while steady-state speed standard deviation gets worse, so it must not
% be described as improving everything. The 550 ns and 5 mA values are still nominal
% simulation figures and the first hardware trial should start at gain 0.75 or lower.
%
% 安全 / Safety: 纯仿真，不接触串口与功率级；默认值仍然是"两级补偿都关闭"。
% Pure simulation: no serial port, no power stage, and both compensation layers stay disabled
% by default.
%
% 参考 / Reference: simulink/Simulink仿真工程说明.md「死区补偿仿真」

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

% 只跑三种组合 / Only three combinations are run:
%   理想（建模仿真都无死区）、有死区但无补偿、有死区且双层补偿全开。
%   注意这里没有单独测"只开前馈"和"只开观察器补偿"的消融；那两个组合要手工调用
%   run_foc_sim（参数 enableDeadTimeCompensation / enableObserverDeadTimeCompensation）。
% Only three combinations are covered: ideal, dead time with no compensation, and dead time
% with both layers on. The single-layer ablations are not run here; use run_foc_sim directly
% with the two enable flags for those.
ideal = run_case(false,false,false,o);
uncompensated = run_case(true,false,false,o);
compensated = run_case(true,true,true,o);

steadyStart = o.duration-2;
segment = [4 5];
% 统计窗口 [s]：稳态取最后 2 s（steadyStart..duration），过渡段固定取 4..5 s
% （接管发生在 2.1 s 左右，这一窗覆盖接管后的速度恢复过程）。窗口固定，因此 duration
% 必须 ≥7 s（见入参校验），否则过渡窗会落到运行之外。
% Windows in [s]: steady state is the last 2 s and the transient window is fixed at 4..5 s,
% which covers the speed recovery just after handoff. The windows are fixed, hence the
% duration >= 7 s requirement in the argument validator.
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
% improvement 是相对无补偿组的百分比改善 [%]，负值表示该指标变差。已知实测仿真结论里
% steadySpeedStdPercent 可能为负（稳态速度标准差变差），报告时不能只挑正数说。
% improvement holds percentage changes [%] versus the uncompensated case; a negative value
% means the metric got worse. In the recorded simulation result steadySpeedStdPercent can be
% negative, so the report must not quote only the favourable numbers.

% 图上轨迹抽取到 1 kHz；report 里的统计量仍用 12 kHz 原始样本。
% Plotted traces are decimated to 1 kHz while every statistic in report still uses the native
% 12 kHz samples.
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
% 输出文件名固定带 `_10s`：与默认 duration=10 对应，改 duration 不会改文件名，
% 因此复跑不同时长会覆盖同一批文件，比较前要留意生成时间戳（report.generatedAt）。
% The output names are fixed with `_10s` to match the default duration of 10 s; a different
% duration overwrites the same files, so check report.generatedAt before comparing runs.
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
% 跑一种补偿组合并返回 run_foc_sim 的结果结构体。
% Runs one compensation combination and returns the run_foc_sim result struct.
%
% 参数 / Parameters: deadTime [-] 是否建模死区损失；feedForward [-] 是否开调制器前馈；
% observerComp [-] 是否开观察器电压重构补偿；o 为已解析的场景选项
% （duration [s]、deadTimeNs [ns]、compensationGain [-]、currentBandA [A]）。
% deadTime enables the modelled loss, feedForward the modulator feed-forward, observerComp the
% observer voltage reconstruction and o carries the scenario options with the units above.
%
% 每次都先关闭模型：MATLAB Function 块的 persistent 状态只在重新加载时清零，不关会把
% 上一组的观察器/积分器状态带进下一组，三组对照就不再是同一初始条件。
% The model is closed first every time because the MATLAB Function block's persistent state
% only resets on reload; otherwise observer and integrator state leaks between cases and the
% comparison no longer shares one initial condition.
if bdIsLoaded('foc_bringup'), close_system('foc_bringup',0); end
r = run_foc_sim('closedLoop',true,'duration',o.duration, ...
    'enableDeadTime',deadTime,'deadTimeNs',o.deadTimeNs, ...
    'enableDeadTimeCompensation',feedForward, ...
    'enableObserverDeadTimeCompensation',observerComp, ...
    'deadTimeCompensationGain',o.compensationGain, ...
    'deadTimeCompensationCurrentBandA',o.currentBandA);
end

function m = metrics(r,steadyStart,segment)
% 汇总一次运行的稳态与过渡段指标：finalState/finalFaultFlags [-]，时间 [s]，
% 转速 [rpm]，电流 [A]，reliableFraction [-]（可靠样本占比）。
% Summarises one run: state and fault flags are dimensionless, times in [s], speeds in [rpm],
% currents in [A] and reliableFraction is the fraction of reliable samples [-].
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
% 逐项计算"无补偿 → 补偿"的相对改善 [%]，负值即变差（见 report.improvement 处的说明）。
% Per-metric relative improvement [%] from uncompensated to compensated; negative means the
% metric got worse (see the note at report.improvement).
out = struct();
out.transientSpeedTargetRmsePercent = reduction(base.transientSpeedTargetRmseRpm,comp.transientSpeedTargetRmseRpm);
out.transientObserverRmsePercent = reduction(base.transientObserverRmseRpm,comp.transientObserverRmseRpm);
out.steadySpeedStdPercent = reduction(base.steadyTrueSpeedStdRpm,comp.steadyTrueSpeedStdRpm);
out.steadyIqTrackingRmsePercent = reduction(base.steadyIqTrackingRmseA,comp.steadyIqTrackingRmseA);
out.steadyIdRmsePercent = reduction(base.steadyIdRmseA,comp.steadyIdRmseA);
end

function value = reduction(before,after)
% 相对下降百分比 [%]：(before-after)/max(before,eps)*100。用 eps 兜住 before==0，
% 避免除零得到 Inf 污染整份报告。
% Relative reduction in [%]: (before-after)/max(before,eps)*100. eps guards against a zero
% baseline so the report cannot be polluted by Inf.
value = 100*(before-after)/max(before,eps);
end

function t = first_time(r,stateValue)
% 首次进入某状态的时刻 [s]；未进入则为 NaN，供报告判空。
% First time [s] the run enters a state, NaN if never, so the report can test for it.
index = find(r.state==stateValue,1);
if isempty(index), t = NaN; else, t = r.time(index); end
end

function out = decimate_run(r,divider)
% 按 divider 抽取用于绘图的字段（时间 [s]，转速 [rpm]，电流 [A]，状态/可靠/故障 [-]）。
% 抽取只为图线可读，报告里的统计量不做抽取。
% Decimates the fields used for plotting (time [s], speed [rpm], currents [A], state and
% flags [-]). Decimation is a readability measure only; report statistics are not decimated.
index = 1:divider:numel(r.time);
names = {'time','trueSpeedRpm','obsSpeedRpm','iqRef','iqMeas','idMeas', ...
    'state','reliable','faultFlags'};
out = struct();
for k = 1:numel(names)
    out.(names{k}) = r.(names{k})(index,:);
end
end
