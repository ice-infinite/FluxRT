function out = compare_with_hardware(varargin)
%COMPARE_WITH_HARDWARE  Compare the Simulink model against captured rig traces.
%
%   out = compare_with_hardware()
%   out = compare_with_hardware('trace','../simulation/results/hardware_582rpm_12v3.csv')
%
%   Loads a hardware trace captured by simulation/capture_hardware_trace.py,
%   runs the Simulink model for the same scenario, aligns the two by simulation
%   time, and reports the same error metrics that simulation/compare_traces.py
%   uses for the Rust reference simulation. That keeps the Simulink result
%   directly comparable with the numbers in
%   simulation/results/comparison_582rpm_12v3.json.
%
%   The hardware trace has no true angle or true speed, so the aligned
%   quantities are the ones both sides really measure:
%     iqMeas / idMeas   current-loop tracking
%     obsSpeedRpm       observer speed
%     obsAngle          observer electrical angle
%   Angles are compared as a wrapped difference.
%
% FluxRT —— 把 Simulink 模型和实机采集的 trace 放在同一时间轴上对拍。
% FluxRT - compares the Simulink model against a captured rig trace on a common time axis.
%
% 职责 / Responsibility:
%   - 读取 simulation/capture_hardware_trace.py 采集的 CSV，用同一场景跑一遍模型，按仿真
%     时间插值对齐，再算 simulation/compare_traces.py 用的同一组误差指标，使结果可以和
%     simulation/results/comparison_582rpm_12v3.json 里的数字直接对照；
%   - 指标为每个信号的 MAE / RMSE / max|err|；角度按折到 (-pi,pi] 的差值比较。
%   - Reads the CSV captured by simulation/capture_hardware_trace.py, runs the same scenario
%     in the model, aligns the two by simulation time and reports the same error metrics
%     compare_traces.py uses, so the numbers stay comparable with the Rust reference
%     comparison. Metrics are MAE/RMSE/max|err| per signal, with angles compared as a
%     wrapped difference.
%
% 安全语义 / Safety: 本函数只做仿真，不打开串口、不使能功率级。它消费的 trace 由
% capture_hardware_trace.py 采集，那个脚本在所有退出路径（正常、解析失败、写文件失败、
% 异常）都会先发 `foc_stop`，因此 trace 对应的功率级一定是已关闭状态。
% This function only simulates: no serial port, no power stage. The trace it consumes was
% captured by capture_hardware_trace.py, which sends `foc_stop` on every exit path (normal,
% parse failure, write failure, exception), so the power stage behind that trace is always
% left disabled.
%
% 可比量 / Comparable quantities: 实机没有真值角度与真值转速，所以只比较双方真正都测到
% 的量——iqMeas/idMeas [A]、obsSpeedRpm [rpm]、obsAngle [rad]。plant 真值不参与对拍。
% The rig has no true angle or true speed, so only quantities both sides actually measure
% are compared: iqMeas/idMeas [A], obsSpeedRpm [rpm] and obsAngle [rad]. Plant truths are
% never compared.
%
% 参考 / Reference: docs/仿真实机相关性验证.md

ip = inputParser;
ip.addParameter('trace', '', @(s) ischar(s) || isstring(s));
ip.addParameter('profile', 'firmware', @(s) ischar(s) || isstring(s));
ip.addParameter('duration', 5.0, @(x) isnumeric(x) && isscalar(x));
ip.addParameter('writeResults', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
o = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(here);

tracePath = char(o.trace);
if isempty(tracePath)
    tracePath = fullfile(here, '..', 'simulation', 'results', ...
                         'hardware_582rpm_12v3_default_final.csv');
end
if ~exist(tracePath, 'file')
    % 报错信息里给出采集命令：trace 必须由实机采集脚本产生，不能用仿真结果顶替，否则
    % 这整个"模型 vs 实机"的比较就退化成自己和自己比。
    % The error carries the capture command because the trace must come from the rig: using a
    % simulation export instead would turn this into a self-comparison.
    error('compare_with_hardware:noTrace', ...
          ['Hardware trace not found: %s\n' ...
           'Capture one first with:\n' ...
           '  python ../simulation/capture_hardware_trace.py ' ...
           '--output ../simulation/results/hardware_582rpm_12v3.csv'], tracePath);
end

hw = readtable(tracePath);

% ---------------------------------------------------------------- model run
if bdIsLoaded('foc_bringup'), close_system('foc_bringup', 0); end
% 场景必须与实机那一轮一致：开环（实机闭环仍默认关闭）+ 启用平均死区模型（实机功率板
% 存在 550 ns 死区，理想逆变器会系统性高估电流环跟踪质量）。
% The scenario must match the hardware run: open loop (the rig still keeps the closed loop
% disabled by default) with the averaged dead-time model enabled, since the real board has
% 550 ns of dead time and an ideal inverter would systematically flatter current tracking.
r = run_foc_sim('observerProfile', o.profile, 'duration', o.duration, ...
                'closedLoop', false, 'enableDeadTime', true, 'rebuild', false);

% ------------------------------------------------------- align by sim time
% Hardware trace time_s is relative to the first received sample; shift it so
% that both start at the same absolute simulation time. The firmware starts
% sending traces immediately after foc_start, which is t=0 of the run.
% 对齐方式 / Alignment: 实机 trace 的 time_s [s] 以第一个收到的样本为原点，而固件在
% foc_start 之后立即开始发 trace，也就是该次运行的 t=0，因此两边可以按绝对仿真时间对齐。
% 插值用 linear + NaN 外插：落在仿真窗口之外的点直接丢弃（isfinite 过滤），绝不外推。
% The trace's time_s [s] is relative to the first received sample, and the firmware starts
% streaming right after foc_start, i.e. at t=0 of the run, so both share an absolute time
% axis. Interpolation is linear with NaN extrapolation, so points outside the simulated
% window are dropped by the isfinite filter rather than extrapolated.
tHw  = hw.time_s;
tSim = r.time;

q = {'iqMeas', 'idMeas', 'obsSpeedRpm', 'obsAngle'};
simVals = struct();
simVals.iqMeas      = interp1(tSim, r.iqMeas,      tHw, 'linear', NaN);
simVals.idMeas      = interp1(tSim, r.idMeas,      tHw, 'linear', NaN);
simVals.obsSpeedRpm = interp1(tSim, r.obsSpeedRpm, tHw, 'linear', NaN);
simVals.obsAngle    = interp1(tSim, r.obsAngle,    tHw, 'linear', NaN);

hwVals = struct();
% 实机字段单位 / Hardware field units: 新格式给 iq_a/id_a [A]，旧格式给 iq_ma/id_ma [mA]，
% 这里统一换成 [A] 再比较；角度字段同为 observer_angle_rad [rad] 或 _mrad [mrad]。
% New traces provide iq_a/id_a in [A]; older ones provide iq_ma/id_ma in [mA], converted
% here to [A]. The angle field is observer_angle_rad [rad] or the older _mrad [mrad].
if ismember('iq_a', hw.Properties.VariableNames)
    hwVals.iqMeas = hw.iq_a;
    hwVals.idMeas = hw.id_a;
else
    hwVals.iqMeas = hw.iq_ma / 1000;
    hwVals.idMeas = hw.id_ma / 1000;
end
hwVals.obsSpeedRpm = hw.observer_speed_rpm;
if ismember('observer_angle_rad', hw.Properties.VariableNames)
    hwVals.obsAngle = hw.observer_angle_rad;
else
    hwVals.obsAngle = hw.observer_angle_mrad / 1000;
end

fprintf('=== Simulink model vs captured hardware trace ===\n');
fprintf('trace   : %s\n', tracePath);
fprintf('profile : %s\n', o.profile);
fprintf('samples : hardware %d, compared %d\n\n', height(hw), numel(tHw));

fprintf('%-16s %10s %10s %10s\n', 'signal', 'MAE', 'RMSE', 'max|err|');
fprintf('%s\n', repmat('-', 1, 50));
metrics = struct();
for k = 1:numel(q)
    name = q{k};
    a = hwVals.(name);
    b = simVals.(name);
    if strcmp(name, 'obsAngle')
        d = mod(a - b + pi, 2*pi) - pi;      % wrapped angle difference
    else
        d = a - b;
    end
    % 指标定义 / Metric definitions: d = 实机 - 仿真，逐样本；MAE/RMSE/max|err| 与
    % simulation/compare_traces.py 同定义，便于和 Rust 参考仿真的数字直接对照。
    % d is hardware minus simulation per sample, and MAE/RMSE/max|err| follow the same
    % definitions as simulation/compare_traces.py so the numbers line up with the Rust
    % reference comparison.
    ok = isfinite(d);
    metrics.(name).mae     = mean(abs(d(ok)));
    metrics.(name).rmse    = sqrt(mean(d(ok).^2));
    metrics.(name).max_abs = max(abs(d(ok)));
    fprintf('%-16s %10.4f %10.4f %10.4f\n', name, ...
        metrics.(name).mae, metrics.(name).rmse, metrics.(name).max_abs);
end

out.metrics = metrics;
out.hardwareTrace = tracePath;
out.profile = o.profile;
out.sim = r;

if o.writeResults
    % 只写指标，不写整段 trace：JSON 是给后续对比脚本读的，体积保持可控。
    % Only the metrics are written, not the full traces, to keep the JSON small enough for
    % downstream comparison scripts.
    dataDir = fullfile(here, 'data');
    if ~exist(dataDir, 'dir')
        mkdir(dataDir);
    end
    fid = fopen(fullfile(dataDir, 'simulink_vs_hardware_current.json'), 'w');
    fprintf(fid, '%s\n', jsonencode(out.metrics, 'PrettyPrint', true));
    fclose(fid);
    fprintf('\nWrote data/simulink_vs_hardware_current.json\n');
end
end
