function summary = compare_foc_traces(options)
%COMPARE_FOC_TRACES Plot the same 582 rpm scenario from PC and real board.
%
% FluxRT 仿真—实机叠加绘图入口：读入两份归一化 FOC CSV，把仿真插值到实机时间轴，
% 计算误差摘要并导出六联图（PNG / FIG / MAT / 摘要 CSV）。
% FluxRT simulation-vs-hardware overlay entry point: reads two normalized FOC CSVs,
% interpolates the simulation onto the hardware time base, computes summary metrics, and
% exports a six-tile figure plus PNG / FIG / MAT / summary CSV.
%
% 在链路中的位置 / Position in the chain:
%   capture_hardware_trace.py 或 foc-bringup-sim 生成 CSV -> 本函数绘图 ->
%   simulation/results/ 下的图片与摘要；不参与控制，不访问串口或功率级。
%   CSV producers -> this plotting step -> artifacts in simulation/results. It never
%   participates in control and never touches the serial port or the power stage.
%
% 单位 / Units: 电流 [A]、电压 [V]、占空比 [标幺 0..1]、转速 [rpm]、角度 [rad]。
% 两类 CSV 必须同列同名，这正是不必做列映射的原因。
% Both CSVs must share the same column names, which is why no column mapping is needed.
%
% 参考 / Reference: docs/仿真实机相关性验证.md

% 输入 / Inputs:
%   SimulationCsv  PC 仿真 CSV，默认 bringup_sim_582rpm_12v3_default_final.csv
%   HardwareCsv    实机 CSV，默认 hardware_582rpm_12v3_default_final.csv
%   OutputDir      输出目录，默认 <project>/simulation/results（已被 git 忽略）
%   OutputStem     输出文件名主干，默认 foc_sim_vs_hardware
%   Visible        true 时保留图窗；批处理（run_matlab_correlation.ps1）传 false
arguments
    options.SimulationCsv (1,1) string = ""
    options.HardwareCsv (1,1) string = ""
    options.OutputDir (1,1) string = ""
    options.OutputStem (1,1) string = "foc_sim_vs_hardware"
    options.Visible (1,1) logical = usejava("desktop")
end

% 用脚本自身位置定位工程根目录，保证从任意当前目录调用都能找到默认输入与输出。
% Locates the project root from this file's own path, so defaults resolve regardless of
% the current working directory.
scriptDir = fileparts(mfilename("fullpath"));
projectDir = fileparts(fileparts(scriptDir));
if strlength(options.OutputDir) == 0
    outputDir = fullfile(projectDir, "simulation", "results");
else
    outputDir = options.OutputDir;
end
% 默认输入对应当前基线工况：12 kHz、约 12.3 V、582 rpm、空载、5 s。
% The defaults match the current baseline scenario: 12 kHz, about 12.3 V, 582 rpm,
% no load, 5 s.
if strlength(options.SimulationCsv) == 0
    options.SimulationCsv = fullfile(outputDir, "bringup_sim_582rpm_12v3_default_final.csv");
end
if strlength(options.HardwareCsv) == 0
    options.HardwareCsv = fullfile(outputDir, "hardware_582rpm_12v3_default_final.csv");
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

% VariableNamingRule="preserve" 保留原始列名（否则 MATLAB 会改写非法标识符），
% 列名是跨语言契约，必须原样保留。
% "preserve" keeps the raw column names; they are a cross-language contract and must not
% be rewritten into MATLAB identifiers.
sim = readtable(options.SimulationCsv, VariableNamingRule="preserve");
hw = readtable(options.HardwareCsv, VariableNamingRule="preserve");
% 本函数实际用到的列：这一列表是契约的一部分，缺失必须报错而不是画出错误曲线。
% The columns this function actually consumes; a missing one must fail loudly rather
% than silently plotting something wrong.
required = ["time_s", "state", "iq_ref_a", "iq_a", "id_a", "vd_v", ...
    "vq_v", "duty_a", "observer_speed_rpm", "observer_reliable"];
if ~isempty(setdiff(required, string(sim.Properties.VariableNames))) || ...
        ~isempty(setdiff(required, string(hw.Properties.VariableNames)))
    error("FOC:TraceSchemaMismatch", "Correlation CSV schema mismatch.");
end

% 样本对齐：以实机时间轴为基准，把仿真量线性插值到实机采样时刻。
% Sample alignment: the hardware time base is the reference and the simulation is
% linearly interpolated onto those instants.
%
% 对齐假设 / Assumptions:
%   - 两侧 time_s 都以"控制使能那一刻"为零点（固件在使能 PWM 前清零 step，仿真从
%     tick 0 起算，均按控制周期换算成秒），因此不做时移/互相关；
%     若实际存在固定时延，这里的误差会整体偏大；
%   - 仿真采样比实机密（sample-every 通常为 320），插值引入的误差远小于绘图分辨率；
%   - "extrap" 允许实机尾部超出仿真时长，此时尾部是外推值，不能当作仿真结论。
%   - Both time bases start at the moment control is enabled, so no shift or
%     cross-correlation is applied; a real fixed delay would inflate every error here.
%   - "extrap" covers a hardware tail longer than the simulation: those points are
%     extrapolated, not simulation evidence.
simIqAtHw = interp1(sim.time_s, sim.iq_a, hw.time_s, "linear", "extrap");
simIdAtHw = interp1(sim.time_s, sim.id_a, hw.time_s, "linear", "extrap");
simVqAtHw = interp1(sim.time_s, sim.vq_v, hw.time_s, "linear", "extrap");
simVdAtHw = interp1(sim.time_s, sim.vd_v, hw.time_s, "linear", "extrap");
% 三相电流峰值 [A]：逐样本取三相最大绝对值，因此是"CSV 采样得到"的下界，
% 固件的高速峰值锁存会更高。
% Phase-current peak [A]: per-sample max over the three phases, hence a sampled lower
% bound; the firmware's high-rate peak latch reads higher.
phasePeakSim = max(abs([sim.phase_current_a; sim.phase_current_b; sim.phase_current_c]));
phasePeakHw = max(abs([hw.phase_current_a; hw.phase_current_b; hw.phase_current_c]));
% 摘要指标：RMSE 的量纲随信号（电流 A、电压 V），可靠样本数是 observer_reliable 计数。
% Summary metrics: RMSE carries the unit of its signal (A or V); reliable-sample counts
% come from observer_reliable.
summary = struct( ...
    SimulationSamples=height(sim), ...
    HardwareSamples=height(hw), ...
    SimulationPeakPhaseCurrentA=phasePeakSim, ...
    HardwarePeakPhaseCurrentA=phasePeakHw, ...
    IqRmseA=sqrt(mean((hw.iq_a - simIqAtHw).^2)), ...
    IdRmseA=sqrt(mean((hw.id_a - simIdAtHw).^2)), ...
    VqRmseV=sqrt(mean((hw.vq_v - simVqAtHw).^2)), ...
    VdRmseV=sqrt(mean((hw.vd_v - simVdAtHw).^2)), ...
    SimulationReliableSamples=sum(sim.observer_reliable ~= 0), ...
    HardwareReliableSamples=sum(hw.observer_reliable ~= 0));

% Visible=false 时用不可见图窗，批处理（无桌面会话）也能导出同样的图片。
% With Visible=false the figure is invisible so headless batch runs export identical
% images.
visibility = "off";
if options.Visible
    visibility = "on";
end
% 六联图：状态机、Iq、Id、dq 电压、转速、duty。同一输入文件对代表同一工况。
% Six tiles: state machine, Iq, Id, dq voltage, speed, duty, all for the same scenario.
figureHandle = figure(Name="FOC simulation vs hardware", Color="white", ...
    Visible=visibility, Position=[80 60 1500 920]);
layout = tiledlayout(figureHandle, 3, 2, TileSpacing="compact", Padding="compact");

% 状态机对比用 stairs（状态是离散量，折线会假装存在中间状态）。
% The state machine uses stairs because state is discrete; a line would imply
% intermediate values that never occur.
nexttile(layout);
stairs(sim.time_s, sim.state, LineWidth=1.2); hold on;
stairs(hw.time_s, hw.state, "--", LineWidth=1.2);
grid on; xlabel("Time (s)"); ylabel("State");
legend("Simulation", "Hardware", Location="best"); title("Rev-up state");

nexttile(layout);
plot(sim.time_s, sim.iq_ref_a, ":", LineWidth=1.0); hold on;
plot(sim.time_s, sim.iq_a, LineWidth=1.2);
plot(hw.time_s, hw.iq_a, "--", LineWidth=1.1);
grid on; xlabel("Time (s)"); ylabel("Iq (A)");
legend("Reference", "Simulation", "Hardware", Location="best"); title("Q-axis current");

nexttile(layout);
plot(sim.time_s, sim.id_a, LineWidth=1.2); hold on;
plot(hw.time_s, hw.id_a, "--", LineWidth=1.1);
grid on; xlabel("Time (s)"); ylabel("Id (A)");
legend("Simulation", "Hardware", Location="best"); title("D-axis current");

nexttile(layout);
plot(sim.time_s, sim.vd_v, LineWidth=1.0); hold on;
plot(hw.time_s, hw.vd_v, "--", LineWidth=1.0);
plot(sim.time_s, sim.vq_v, LineWidth=1.0);
plot(hw.time_s, hw.vq_v, "--", LineWidth=1.0);
grid on; xlabel("Time (s)"); ylabel("Voltage (V)");
legend("Vd sim", "Vd hw", "Vq sim", "Vq hw", Location="best"); title("Voltage command");

% 转速图只有仿真侧有 plant 真值；实机没有编码器/测速仪，
% 其"速度"是 SMO 估算值，因此不能作为真值比较（图题已注明）。
% Only the simulation has plant truth; without an encoder or tachometer the hardware
% "speed" is the SMO estimate and cannot serve as truth (the tile title says so).
nexttile(layout);
plot(sim.time_s, sim.true_speed_rpm, LineWidth=1.3); hold on;
plot(sim.time_s, sim.observer_speed_rpm, LineWidth=0.9);
plot(hw.time_s, hw.observer_speed_rpm, "--", LineWidth=0.9);
grid on; xlabel("Time (s)"); ylabel("Speed (rpm)");
legend("Plant truth", "Observer sim", "Observer hw", Location="best");
title("Speed: hardware truth is not measured");

% duty 已由采集脚本从 per-mille 转成标幺，因此这里固定 ylim([0 1]) 便于跨次对比。
% Duty was already converted from per-mille to per-unit by the capture script, so the
% fixed ylim([0 1]) keeps runs directly comparable.
nexttile(layout);
plot(sim.time_s, sim.duty_a, LineWidth=1.0); hold on;
plot(hw.time_s, hw.duty_a, "--", LineWidth=1.0);
plot(sim.time_s, sim.duty_b, LineWidth=0.8);
plot(hw.time_s, hw.duty_b, "--", LineWidth=0.8);
grid on; ylim([0 1]); xlabel("Time (s)"); ylabel("Duty");
legend("A sim", "A hw", "B sim", "B hw", Location="best"); title("PWM duty");

% 图题汇总工况与关键指标，便于图片脱离上下文单独引用时不被误读。目标转速和
% 母线电压必须从本次实机 CSV 读取，不能沿用默认 582 rpm 文本。
% The super-title restates the scenario and key metrics so a standalone image cannot be
% misread out of context.
targetRpm = median(hw.target_speed_rpm, "omitmissing");
busVoltageV = median(hw.dc_bus_voltage_v, "omitmissing");
title(layout, sprintf(["Same firmware path, 12 kHz / %.2f V / %.0f rpm | " + ...
    "Iq RMSE %.3f A | peak %.3f/%.3f A"], ...
    busVoltageV, targetRpm, summary.IqRmseA, phasePeakSim, phasePeakHw));

% 输出 / Outputs（全部落在 outputDir，默认 simulation/results，已被 .gitignore 忽略）:
%   <stem>.png            180 dpi 六联图，报告用
%   <stem>.fig            可继续编辑的 MATLAB 图
%   <stem>.mat            sim / hw 两份 table + summary + options，便于复算
%   <stem>_summary.csv    核心指标表
% All four artifacts land in the git-ignored simulation/results directory.
pngPath = fullfile(outputDir, options.OutputStem + ".png");
figPath = fullfile(outputDir, options.OutputStem + ".fig");
matPath = fullfile(outputDir, options.OutputStem + ".mat");
summaryPath = fullfile(outputDir, options.OutputStem + "_summary.csv");
exportgraphics(figureHandle, pngPath, Resolution=180);
savefig(figureHandle, figPath);
save(matPath, "sim", "hw", "summary", "options");
writetable(struct2table(summary), summaryPath);
% 不可见图窗导出后立即关闭，避免批处理会话里堆积图窗句柄。
% The invisible figure is closed after export so batch sessions do not accumulate
% figure handles.
if ~options.Visible
    close(figureHandle);
end

% MATLAB_FOC_CORRELATION_* 标记供 run_matlab_correlation.ps1 / CI 解析；
% "PASS" 只表示绘图与摘要成功完成，不代表误差达标。
% The MATLAB_FOC_CORRELATION_* markers are parsed by run_matlab_correlation.ps1 and CI;
% "PASS" only means the plot and summary were produced, not that the error is acceptable.
fprintf("MATLAB_FOC_CORRELATION_PASS iq_rmse_a=%.4f peak_sim_a=%.3f peak_hw_a=%.3f\n", ...
    summary.IqRmseA, phasePeakSim, phasePeakHw);
fprintf("MATLAB_FOC_CORRELATION_PLOT=%s\n", pngPath);
end
