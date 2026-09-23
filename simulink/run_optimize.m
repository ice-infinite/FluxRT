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

ip = inputParser;
ip.addParameter('duration', 5.0, @(x) isnumeric(x) && isscalar(x) && x > 2.5);
ip.addParameter('quick', false, @(x) islogical(x) || isnumeric(x));
ip.addParameter('writeResults', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
o = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(here);

if o.quick
    kGrid   = [3.0 4.0 5.0];
    bndGrid = [0.12 0.16 0.20];
    aGrid   = [0.025 0.05 0.10];
else
    kGrid   = [2.0 3.0 4.0 5.0 6.0];
    bndGrid = [0.08 0.12 0.16 0.20 0.24];
    aGrid   = [0.025 0.05 0.075 0.10 0.20];
end

holdStart = max(2.5, 0.6 * o.duration);
total = numel(kGrid) * numel(bndGrid) * numel(aGrid);
fprintf('=== observer sweep: %d combinations, %.1f s each ===\n', total, o.duration);

rows = zeros(total, 7);
n = 0;
for ik = 1:numel(kGrid)
    for ib = 1:numel(bndGrid)
        for ia = 1:numel(aGrid)
            n = n + 1;
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
results.firmwareAbi = '0x00070000';
results.note = 'Current firmware-equivalent open-loop observer sweep; hardware validation still required.';
% Locale-independent timestamp. datestr(now, 'yyyy-mm-dd HH:mm:ss') throws on
% some Windows locale configurations, which previously aborted this function
% before anything was written to disk and silently discarded a full sweep.
results.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss'));

% Rank by reliable samples first, then angle error.
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
    fprintf('\n=== no combination produced a reliable observer ===\n');
    fprintf('This is the signature of a broken plant/controller angle convention,\n');
    fprintf('not of bad tuning. Fix that first (see README_simulink.md).\n');
end

if o.writeResults
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
