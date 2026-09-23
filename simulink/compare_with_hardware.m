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
    error('compare_with_hardware:noTrace', ...
          ['Hardware trace not found: %s\n' ...
           'Capture one first with:\n' ...
           '  python ../simulation/capture_hardware_trace.py ' ...
           '--output ../simulation/results/hardware_582rpm_12v3.csv'], tracePath);
end

hw = readtable(tracePath);

% ---------------------------------------------------------------- model run
if bdIsLoaded('foc_bringup'), close_system('foc_bringup', 0); end
r = run_foc_sim('observerProfile', o.profile, 'duration', o.duration, ...
                'closedLoop', false, 'enableDeadTime', true, 'rebuild', false);

% ------------------------------------------------------- align by sim time
% Hardware trace time_s is relative to the first received sample; shift it so
% that both start at the same absolute simulation time. The firmware starts
% sending traces immediately after foc_start, which is t=0 of the run.
tHw  = hw.time_s;
tSim = r.time;

q = {'iqMeas', 'idMeas', 'obsSpeedRpm', 'obsAngle'};
simVals = struct();
simVals.iqMeas      = interp1(tSim, r.iqMeas,      tHw, 'linear', NaN);
simVals.idMeas      = interp1(tSim, r.idMeas,      tHw, 'linear', NaN);
simVals.obsSpeedRpm = interp1(tSim, r.obsSpeedRpm, tHw, 'linear', NaN);
simVals.obsAngle    = interp1(tSim, r.obsAngle,    tHw, 'linear', NaN);

hwVals = struct();
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
