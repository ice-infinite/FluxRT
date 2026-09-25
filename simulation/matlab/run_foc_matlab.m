function summary = run_foc_matlab(options)
%RUN_FOC_MATLAB Run the real Rust FOC simulation and create MATLAB plots.
%
% The Rust executable remains the source of controller and PMSM behavior.
% MATLAB launches it, reads its deterministic CSV trace, calculates summary
% metrics, and exports PNG/FIG/MAT artifacts.
%
% FluxRT "Rust 计算、MATLAB 分析"入口：由 MATLAB 调用 foc-sim（真实 Rust 控制器 +
% PMSM plant），读取其确定性 CSV，统计指标并导出六联图。
% FluxRT "Rust computes, MATLAB analyses" entry point: MATLAB launches foc-sim (the real
% Rust controller plus PMSM plant), reads its deterministic CSV, computes summary metrics
% and exports a six-tile figure.
%
% 边界 / Boundary:
%   - MATLAB 不复制 Clarke/Park、PI 或 SVPWM，Rust 始终是唯一的 FOC 实现，
%     避免两份算法逐渐漂移；
%   - 不访问串口、不驱动电机，也没有停机路径可言：本函数只跑 PC 仿真。
%   - No second implementation of the control math, and no serial or power-stage access;
%     there is no stop path because nothing is energized.
%
% plant 边界 / Plant limits: 平均值逆变器 + dq PMSM 方程 + 理想转子角度，
% 不含开关纹波、死区、采样噪声、ADC 量化、母线动态与热模型。
% Average-value inverter, dq PMSM equations and ideal rotor angle: no switching ripple,
% dead time, sampling noise, ADC quantisation, bus dynamics or thermal model.
%
% 参考 / Reference: docs/MATLAB联合仿真.md

% 场景参数 / Scenario parameters:
%   DurationS      仿真时长 [s]，必须为正，默认 3.0
%   TargetRpm      速度目标 [rpm]，默认 524.0
%   LoadStepTimeS  负载阶跃时刻 [s]，默认 1.0；负值被 arguments 校验拒绝
%   LoadTorqueNm   阶跃后的负载转矩 [N·m]，默认 0.004
%   SampleEvery    每多少个 12 kHz 电流环周期记录一个 CSV 样本，默认 10；
%                  只改变输出数据密度，不改变控制器与 plant 的计算频率
%   OutputDir      输出目录，默认 <project>/simulation/results（已被 git 忽略）
%   Visible        true 时保留图窗；run_matlab_sim.ps1 传 false 以支持批处理

arguments
    options.DurationS (1,1) double {mustBePositive} = 3.0
    options.TargetRpm (1,1) double {mustBeFinite} = 524.0
    options.LoadStepTimeS (1,1) double {mustBeNonnegative,mustBeFinite} = 1.0
    options.LoadTorqueNm (1,1) double {mustBeFinite} = 0.004
    options.SampleEvery (1,1) double {mustBeInteger,mustBePositive} = 10
    options.OutputDir (1,1) string = ""
    options.Visible (1,1) logical = usejava("desktop")
end

% 用脚本自身位置定位工程根目录，保证从任意当前目录调用都能解析默认路径。
% Locates the project root from this file's own path so defaults resolve from any
% working directory.
scriptDir = fileparts(mfilename("fullpath"));
projectDir = fileparts(fileparts(scriptDir));
if strlength(options.OutputDir) == 0
    outputDir = fullfile(projectDir, "simulation", "results");
else
    outputDir = options.OutputDir;
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

% 优先用用户目录下已安装的 cargo；找不到时退回 PATH 里的 cargo，避免硬编码机器路径。
% Prefers the user's installed cargo and falls back to PATH so no machine-specific path
% is hard-coded.
cargoExe = fullfile(getenv("USERPROFILE"), ".cargo", "bin", "cargo.exe");
if ~isfile(cargoExe)
    cargoExe = "cargo";
end
% --release --locked 与工程其它基准保持一致，并避免跑基线时悄悄更新依赖版本；
% 这里调用的 foc-sim 导出的 CSV 就是交给 MATLAB 的唯一交接格式。
% --release --locked matches the other benchmarks and prevents a baseline run from
% silently upgrading dependencies; the CSV is the only hand-off format to MATLAB.
manifestPath = fullfile(projectDir, "rust", "Cargo.toml");
csvPath = fullfile(outputDir, "foc_sim_trace.csv");

% %.9g 显式给出往返精度，避免时间/转速在字符串化时被截短而悄悄改变场景；
% 路径用双引号包裹以兼容含空格的目录（例如 Program Files 下的 cargo）。
% %.9g states round-trip precision explicitly so time and speed are not truncated while
% being stringified, and the double quotes tolerate paths containing spaces.
command = sprintf([ ...
    '"%s" run --manifest-path "%s" --package foc-sim --release --locked -- ' ...
    '--csv "%s" --duration %.9g --target-rpm %.9g ' ...
    '--load-step-time %.9g --load-torque %.9g --sample-every %d'], ...
    cargoExe, manifestPath, csvPath, options.DurationS, options.TargetRpm, ...
    options.LoadStepTimeS, options.LoadTorqueNm, options.SampleEvery);

% 先回显 Rust 侧输出再判状态：缺列、参数越界等错误信息只出现在 cargo 输出里。
% The Rust output is echoed before the status check, because errors such as an
% out-of-range argument appear only there.
fprintf("Running Rust FOC simulation...\n");
[status, commandOutput] = system(command);
fprintf("%s", commandOutput);
if status ~= 0
    error("FOC:RustSimulationFailed", ...
        "Rust simulation exited with status %d.", status);
end

data = readtable(csvPath, VariableNamingRule="preserve");
% 必需列清单是与 Rust 导出格式的契约；缺少任何一列都必须报错，
% 而不是用 NaN 画出一条看似正常的曲线。
% The required-column list is the contract with the Rust export; a missing column must
% fail loudly instead of plotting a plausible-looking curve built from NaN.
required = ["time_s", "target_speed_rpm", "measured_speed_rpm", ...
    "phase_current_a", "phase_current_b", "phase_current_c", ...
    "id_ref_a", "iq_ref_a", "id_a", "iq_a", "vd_v", "vq_v", ...
    "duty_a", "duty_b", "duty_c", "load_torque_nm", ...
    "electromagnetic_torque_nm", "voltage_limited"];
missing = setdiff(required, string(data.Properties.VariableNames));
if ~isempty(missing)
    error("FOC:TraceSchemaMismatch", ...
        "Rust trace is missing columns: %s", strjoin(missing, ", "));
end

% 稳态窗口取最后 0.5 s（而不是全程）：阶跃与收敛过程会主导全程 RMSE，
% 无法反映"稳态跟踪质量"。
% The steady window is the last 0.5 s rather than the whole run, because the load step and
% the settling transient would otherwise dominate the RMSE.
time = data.time_s;
steadyStart = max(0.0, options.DurationS - 0.5);
steady = time >= steadyStart;
% 符号约定为"目标 - 实测" [rpm]，因此正误差表示转速偏低。
% Sign convention is target minus measured [rpm], so a positive error means too slow.
speedError = data.target_speed_rpm - data.measured_speed_rpm;
% 三相电流峰值 [A]：逐样本取三相最大绝对值，是采样下界（非真实瞬时峰值）。
% Phase-current peak [A]: per-sample max over three phases, a sampled lower bound.
phasePeak = max(abs([data.phase_current_a; data.phase_current_b; data.phase_current_c]));

% 摘要字段名会出现在 foc_sim_summary.csv 与 MATLAB_FOC_PASS 行里，
% 报告和回归脚本按名字引用，因此不要重命名。
% These summary field names reach foc_sim_summary.csv and the MATLAB_FOC_PASS line, and
% the reports reference them by name, so do not rename them.
summary = struct( ...
    FinalSpeedRpm=data.measured_speed_rpm(end), ...
    TargetSpeedRpm=options.TargetRpm, ...
    FinalErrorRpm=speedError(end), ...
    SteadyRmseRpm=sqrt(mean(speedError(steady).^2)), ...
    PeakPhaseCurrentA=phasePeak, ...
    PeakIqA=max(abs(data.iq_a)), ...
    VoltageLimitedSamples=sum(data.voltage_limited ~= 0), ...
    TraceSamples=height(data));

if options.Visible
    figureVisibility = "on";
else
    figureVisibility = "off";
end
% 六联图：转速闭环（带负载阶跃标记）、dq 电流、三相电流、dq 电压与限幅标志、
% SVPWM 占空比、电磁转矩对负载转矩。
% Six tiles: speed loop with the load-step marker, dq currents, phase currents, dq
% voltage with the limiting flag, SVPWM duty, and electromagnetic vs load torque.
figureHandle = figure(Name="Rust FOC + MATLAB", ...
    Color="white", Visible=figureVisibility, Position=[100 80 1400 900]);
layout = tiledlayout(figureHandle, 3, 2, TileSpacing="compact", Padding="compact");

% 用 xline 标出负载阶跃时刻 [s]，便于判断响应延迟是否与阶跃对齐。
% The xline marks the load-step instant [s] so the response delay can be read against it.
nexttile(layout);
plot(time, data.target_speed_rpm, "--", LineWidth=1.2); hold on;
plot(time, data.measured_speed_rpm, LineWidth=1.4);
xline(options.LoadStepTimeS, ":", "Load step");
grid on; xlabel("Time (s)"); ylabel("Speed (rpm)");
legend("Target", "Measured", Location="best"); title("Speed closed loop");

nexttile(layout);
plot(time, data.id_ref_a, "--", LineWidth=1.0); hold on;
plot(time, data.id_a, LineWidth=1.1);
plot(time, data.iq_ref_a, "--", LineWidth=1.0);
plot(time, data.iq_a, LineWidth=1.1);
xline(options.LoadStepTimeS, ":");
grid on; xlabel("Time (s)"); ylabel("Current (A)");
legend("Id ref", "Id", "Iq ref", "Iq", Location="best"); title("dq currents");

nexttile(layout);
plot(time, data.phase_current_a, LineWidth=0.9); hold on;
plot(time, data.phase_current_b, LineWidth=0.9);
plot(time, data.phase_current_c, LineWidth=0.9);
xline(options.LoadStepTimeS, ":");
grid on; xlabel("Time (s)"); ylabel("Current (A)");
legend("Ia", "Ib", "Ic", Location="best"); title("Phase currents");

% voltage_limited 是 0/1 标志，用 stairs 画成台阶以保持其离散含义（不是电压值）。
% voltage_limited is a 0/1 flag; stairs keeps its discrete meaning (it is not a voltage).
nexttile(layout);
plot(time, data.vd_v, LineWidth=1.0); hold on;
plot(time, data.vq_v, LineWidth=1.0);
stairs(time, double(data.voltage_limited), ":", LineWidth=1.0);
xline(options.LoadStepTimeS, ":");
grid on; xlabel("Time (s)"); ylabel("Voltage (V) / flag");
legend("Vd", "Vq", "Limited", Location="best"); title("Voltage command");

nexttile(layout);
plot(time, data.duty_a, LineWidth=0.9); hold on;
plot(time, data.duty_b, LineWidth=0.9);
plot(time, data.duty_c, LineWidth=0.9);
xline(options.LoadStepTimeS, ":");
grid on; ylim([0 1]); xlabel("Time (s)"); ylabel("Duty");
legend("A", "B", "C", Location="best"); title("SVPWM duty");

nexttile(layout);
plot(time, data.electromagnetic_torque_nm, LineWidth=1.1); hold on;
stairs(time, data.load_torque_nm, "--", LineWidth=1.1);
xline(options.LoadStepTimeS, ":");
grid on; xlabel("Time (s)"); ylabel("Torque (N m)");
legend("Electromagnetic", "Load", Location="best"); title("Torque disturbance");

title(layout, sprintf([ ...
    'Rust FOC simulation | final %.2f rpm | error %.2f rpm | ' ...
    'phase peak %.3f A'], summary.FinalSpeedRpm, summary.FinalErrorRpm, ...
    summary.PeakPhaseCurrentA));

% 输出 / Outputs（全部写入 outputDir，默认 simulation/results，已被 .gitignore 忽略）:
%   foc_sim_trace.csv        Rust 导出的逐样本时序（本函数的输入，也是 hand-off 格式）
%   foc_sim_overview.png/.fig  六联图（180 dpi）
%   foc_sim_results.mat      data / summary / options，便于离线复算
%   foc_sim_summary.csv      最终速度、误差、RMSE、峰值电流等摘要
% All artifacts land in the git-ignored simulation/results directory.
pngPath = fullfile(outputDir, "foc_sim_overview.png");
figPath = fullfile(outputDir, "foc_sim_overview.fig");
matPath = fullfile(outputDir, "foc_sim_results.mat");
summaryPath = fullfile(outputDir, "foc_sim_summary.csv");
exportgraphics(figureHandle, pngPath, Resolution=180);
savefig(figureHandle, figPath);
save(matPath, "data", "summary", "options");
writetable(struct2table(summary), summaryPath);
% 不可见图窗导出后关闭，避免批处理会话堆积句柄。
% Closes the invisible figure after export so batch sessions do not pile up handles.
if ~options.Visible
    close(figureHandle);
end

% MATLAB_FOC_* 是 run_matlab_sim.ps1 与 CI 的解析契约；
% "PASS" 只表示仿真跑通并出图，不代表模型已经过实机辨识。
% The MATLAB_FOC_* lines are the parsing contract for run_matlab_sim.ps1 and CI; "PASS"
% means the run completed and produced plots, not that the model is hardware-validated.
fprintf("MATLAB_FOC_PASS final_rpm=%.2f error_rpm=%.2f peak_phase_current_a=%.3f\n", ...
    summary.FinalSpeedRpm, summary.FinalErrorRpm, summary.PeakPhaseCurrentA);
fprintf("MATLAB_FOC_PLOT=%s\n", pngPath);
fprintf("MATLAB_FOC_DATA=%s\n", matPath);
end
