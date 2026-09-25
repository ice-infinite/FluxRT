function results = run_optimize(varargin)
%RUN_OPTIMIZE  Sweep the sliding-mode observer tuning and rank the outcomes.
%
%   results = run_optimize()
%   results = run_optimize('duration',3.0,'quick',true)
%
%   Sweeps the three parameters that were shown to decide whether the observer
%   locks at all (see the firmware analysis):
%
%     observer_smo_k_slide_v   sliding gain          [V]
%     observer_smo_boundary_a  boundary layer         [A]
%     observer_emf_filter_alpha EMF extraction filter [-]
%
%   Scoring follows the firmware's own reliability gate so the ranking is
%   comparable with what foc_status reports on the target:
%     reliableSamples  higher is better
%     holdSpeedRmseRpm lower is better
%     holdAngleRmseRad lower is better
%
%   Results are written to data/observer_sweep_current.json and
%   data/observer_sweep_current.csv. The best row is NOT applied automatically.
%
%   IMPORTANT: this script is only meaningful once the plant/controller angle
%   convention has been verified (see RUN_FOC_VALIDATION.m). Running an
%   optimizer on a model whose current loop does not reproduce the firmware
%   will produce a confident, wrong answer.
%
% FluxRT —— 观察器整定扫描：跑网格、按固件同款可靠性门排序、写出候选表。
% FluxRT - observer tuning sweep: runs the grid, ranks by the firmware's own reliability
% gate and writes a candidate table.
%
% 职责 / Responsibility:
%   - 扫描三个被证明决定"观察器能否锁定"的参数（单位见下），每个点都重新跑一遍
%     开环仿真，用固件相同的判据统计可靠样本数，并统计保持段的速度/角度 RMSE；
%   - 结果写入 data/observer_sweep_current.csv/json，**最优行不会自动生效**：
%     扫描结果只是候选，任何参数上板前都必须重新走安全流程。
%   - Sweeps the three parameters that decide whether the observer locks at all, re-running
%     the open-loop simulation per point, counting reliable samples with the firmware's own
%     gate and measuring hold-phase speed/angle RMSE. Results go to
%     data/observer_sweep_current.csv/json and the best row is never applied automatically:
%     sweep output is a candidate list, not a hardware configuration.
%
% 前提 / Precondition: 只有在 plant/controller 的角度约定已被 run_foc_validation.m 验证过
% 之后，扫描才有意义。在被控对象本身对不上固件的模型上做优化，会得到一个"自信但错误"
% 的最优点。
% The sweep is only meaningful once run_foc_validation.m has verified the plant/controller
% angle convention; optimising a plant that does not reproduce the firmware yields a
% confident, wrong answer.
%
% 安全 / Safety: 全部是 PC 仿真，不开串口、不使能功率级；本函数不改写任何 .m 源文件。
% Everything here is PC simulation: no serial port, no power stage, no .m source rewriting.
%
% 参考 / Reference: simulink/Simulink仿真工程说明.md「旧扫描结果」,
% docs/仿真实机相关性验证.md §5

ip = inputParser;
ip.addParameter('duration', 5.0, @(x) isnumeric(x) && isscalar(x) && x > 2.5);
ip.addParameter('quick', false, @(x) islogical(x) || isnumeric(x));
ip.addParameter('writeResults', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
o = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(here);

if o.quick
    % 快速网格 3x3x3 = 27 点，量纲同上（kSlideV [V]、boundaryA [A]、emfAlpha [-]）。
    % Quick grid: 3x3x3 = 27 points, same units as above.
    kGrid   = [3.0 4.0 5.0];
    bndGrid = [0.12 0.16 0.20];
    aGrid   = [0.025 0.05 0.10];
else
    % 完整网格 5x5x5 = 125 点。旧数据 data/optimize_grid.csv 是另一轮 216 点扫描，用的是
    % 错误的电流 PI 换算和 4 ms 可靠性窗口，不能与本轮结果混用（见 Simulink仿真工程说明.md）。
    % Full grid: 5x5x5 = 125 points. The legacy data/optimize_grid.csv came from a different
    % 216-point sweep with wrong current-PI scaling and a 4 ms reliability window, and must
    % not be mixed with current results.
    kGrid   = [2.0 3.0 4.0 5.0 6.0];
    bndGrid = [0.08 0.12 0.16 0.20 0.24];
    aGrid   = [0.025 0.05 0.075 0.10 0.20];
end

holdStart = max(2.5, 0.6 * o.duration);
% 保持段起点 [s] 取运行时长的 60%（不低于 2.5 s），把 alignment/ramp/接管瞬态排除在
% 评分之外：扫描要比较的是稳态观察质量，不是启动快慢。
% The hold window starts at 60% of the run (never before 2.5 s) so the alignment, ramp and
% handoff transients are excluded from scoring: the sweep compares steady observer quality,
% not start-up speed.
total = numel(kGrid) * numel(bndGrid) * numel(aGrid);
fprintf('=== observer sweep: %d combinations, %.1f s each ===\n', total, o.duration);

rows = zeros(total, 7);
n = 0;
for ik = 1:numel(kGrid)
    for ib = 1:numel(bndGrid)
        for ia = 1:numel(aGrid)
            n = n + 1;
            % 每个点先关闭模型：MATLAB Function 块的 persistent 状态只在模型重新加载时
            % 初始化，不关会把这些点的状态串到下一点，扫描结果就不可比了。
            % rebuild=false 是安全的：run_foc_sim 内部仍会做时间戳过期检查，源文件比
            % .slx 新时会自动重建（正是为了避免整轮扫描跑在过期模型上）。
            % The model is closed before every point because the MATLAB Function block's
            % persistent state only resets on reload; without this, state leaks from one
            % point into the next and the sweep is not comparable. rebuild=false is safe
            % because run_foc_sim still performs the timestamp staleness check and rebuilds
            % when a source file is newer than the .slx.
            if bdIsLoaded('foc_bringup'), close_system('foc_bringup', 0); end
            try
                r = run_foc_sim( ...
                    'observerProfile', 'firmware', ...
                    'closedLoop', false, ...
                    'enableDeadTime', false, ...
                    'duration', o.duration, ...
                    'rebuild', false, ...
                    'smoSlide',    kGrid(ik), ...
                    'smoBoundary', bndGrid(ib), ...
                    'emfAlpha',    aGrid(ia));
                hold = r.time >= holdStart;
                rows(n, :) = [ ...
                    kGrid(ik), bndGrid(ib), aGrid(ia), ...
                    sum(r.reliable ~= 0), ...
                    sqrt(mean(r.speedErrorRpm(hold).^2)), ...
                    sqrt(mean(r.angleErrorRad(hold).^2)), ...
                    max(abs(r.iqPlant)) ];
            catch ME
                % 单点失败不能中断整轮扫描：记 0 可靠样本 + NaN 指标，保持表格为矩形，
                % 并在控制台打印失败原因，避免"少了几行"被忽略。
                % A single failing point must not abort the sweep: it is recorded with 0
                % reliable samples and NaN metrics so the table stays rectangular, and the
                % reason is printed so a silently missing row is impossible.
                warning('run_optimize:runFailed', ...
                        'k=%.3g bnd=%.3g alpha=%.3g failed: %s', ...
                        kGrid(ik), bndGrid(ib), aGrid(ia), ME.message);
                rows(n, :) = [kGrid(ik), bndGrid(ib), aGrid(ia), 0, NaN, NaN, NaN];
            end
            fprintf('  [%3d/%3d] k=%6.3g bnd=%6.4g alpha=%5.3g -> rel=%6d spd=%8.2f ang=%7.4f\n', ...
                n, total, rows(n,1), rows(n,2), rows(n,3), ...
                rows(n,4), rows(n,5), rows(n,6));
        end
    end
end

results = struct();
results.table = array2table(rows, 'VariableNames', ...
    {'kSlideV','boundaryA','emfAlpha','reliableSamples', ...
     'holdSpeedRmseRpm','holdAngleRmseRad','peakIqA'});
results.duration = o.duration;
results.firmwareAbi = '0x00080000';
results.note = 'Current firmware-equivalent open-loop observer sweep; hardware validation still required.';
% Locale-independent timestamp. datestr(now, 'yyyy-mm-dd HH:mm:ss') throws on
% some Windows locale configurations, which previously aborted this function
% before anything was written to disk and silently discarded a full sweep.
% 时间戳同样避开 datestr(now,fmt)：它曾在部分 Windows locale 下抛错，导致整轮扫描在
% 落盘前就被丢弃且没有明显提示。
% The stamp also avoids datestr(now,fmt), which threw on some Windows locales and discarded
% a whole sweep before anything was written to disk.
results.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss'));

% Rank by reliable samples first, then angle error.
% 排序规则 / Ranking rule: 先按可靠样本数降序（能不能锁住是硬门槛），再按保持段角度
% RMSE [rad] 升序（锁住之后看角度精度）。速度 RMSE [rpm] 只做展示，不参与排序。
% Rank by reliable samples descending (locking at all is the hard gate) and then by hold
% angle RMSE [rad] ascending. Speed RMSE [rpm] is reported but not ranked on.
t = results.table;
valid = t.reliableSamples > 0 & isfinite(t.holdAngleRmseRad);
if any(valid)
    sub = t(valid, :);
    [~, order] = sortrows([-sub.reliableSamples, sub.holdAngleRmseRad]);
    best = sub(order(1), :);
    results.best = table2struct(best);
    fprintf('\n=== best combination ===\n');
    fprintf('  k_slide = %.4g V\n', best.kSlideV);
    fprintf('  boundary = %.4g A\n', best.boundaryA);
    fprintf('  emf_alpha = %.4g\n', best.emfAlpha);
    fprintf('  reliable samples = %d\n', best.reliableSamples);
    fprintf('  hold speed RMSE = %.2f rpm\n', best.holdSpeedRmseRpm);
    fprintf('  hold angle RMSE = %.4f rad\n', best.holdAngleRmseRad);
else
    results.best = struct();
    % 一个可靠点都没有时，结论是"模型的角度约定错了"而不是"整定得不好"：单靠三个
    % 观察器参数无法让一个符号接反的系统锁定。
    % When no point is reliable the conclusion is a broken angle convention, not bad tuning:
    % no combination of these three observer parameters can lock a system whose sign is
    % flipped.
    fprintf('\n=== no combination produced a reliable observer ===\n');
    fprintf('This is the signature of a broken plant/controller angle convention,\n');
    fprintf('not of bad tuning. Fix that first (see Simulink仿真工程说明.md).\n');
end

if o.writeResults
    % 结果写到 data/observer_sweep_current.*。注意与旧文件 data/optimal_observer.json、
    % data/optimize_grid.csv（216 点那一轮，已作废）区分，两者不能混用。
    % Results go to data/observer_sweep_current.*, deliberately separate from the superseded
    % legacy files data/optimal_observer.json and data/optimize_grid.csv (the 216-point run).
    dataDir = fullfile(here, 'data');
    if ~exist(dataDir, 'dir')
        mkdir(dataDir);
    end
    writetable(results.table, fullfile(dataDir, 'observer_sweep_current.csv'));
    fid = fopen(fullfile(dataDir, 'observer_sweep_current.json'), 'w');
    fprintf(fid, '%s\n', jsonencode(results, 'PrettyPrint', true));
    fclose(fid);
    fprintf('\nWrote data/observer_sweep_current.csv and data/observer_sweep_current.json\n');
end
end
