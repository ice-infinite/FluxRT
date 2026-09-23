function summary = run_foc_matlab(options)
%RUN_FOC_MATLAB Run the real Rust FOC simulation and create MATLAB plots.
%
% The Rust executable remains the source of controller and PMSM behavior.
% MATLAB launches it, reads its deterministic CSV trace, calculates summary
% metrics, and exports PNG/FIG/MAT artifacts.

arguments
    options.DurationS (1,1) double {mustBePositive} = 3.0
    options.TargetRpm (1,1) double {mustBeFinite} = 524.0
    options.LoadStepTimeS (1,1) double {mustBeNonnegative,mustBeFinite} = 1.0
    options.LoadTorqueNm (1,1) double {mustBeFinite} = 0.004
    options.SampleEvery (1,1) double {mustBeInteger,mustBePositive} = 10
    options.OutputDir (1,1) string = ""
    options.Visible (1,1) logical = usejava("desktop")
end

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

cargoExe = fullfile(getenv("USERPROFILE"), ".cargo", "bin", "cargo.exe");
if ~isfile(cargoExe)
    cargoExe = "cargo";
end
manifestPath = fullfile(projectDir, "rust", "Cargo.toml");
csvPath = fullfile(outputDir, "foc_sim_trace.csv");

command = sprintf([ ...
    '"%s" run --manifest-path "%s" --package foc-sim --release --locked -- ' ...
    '--csv "%s" --duration %.9g --target-rpm %.9g ' ...
    '--load-step-time %.9g --load-torque %.9g --sample-every %d'], ...
    cargoExe, manifestPath, csvPath, options.DurationS, options.TargetRpm, ...
    options.LoadStepTimeS, options.LoadTorqueNm, options.SampleEvery);

fprintf("Running Rust FOC simulation...\n");
[status, commandOutput] = system(command);
fprintf("%s", commandOutput);
if status ~= 0
    error("FOC:RustSimulationFailed", ...
        "Rust simulation exited with status %d.", status);
end

data = readtable(csvPath, VariableNamingRule="preserve");
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

time = data.time_s;
steadyStart = max(0.0, options.DurationS - 0.5);
steady = time >= steadyStart;
speedError = data.target_speed_rpm - data.measured_speed_rpm;
phasePeak = max(abs([data.phase_current_a; data.phase_current_b; data.phase_current_c]));

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
figureHandle = figure(Name="Rust FOC + MATLAB", ...
    Color="white", Visible=figureVisibility, Position=[100 80 1400 900]);
layout = tiledlayout(figureHandle, 3, 2, TileSpacing="compact", Padding="compact");

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

pngPath = fullfile(outputDir, "foc_sim_overview.png");
figPath = fullfile(outputDir, "foc_sim_overview.fig");
matPath = fullfile(outputDir, "foc_sim_results.mat");
summaryPath = fullfile(outputDir, "foc_sim_summary.csv");
exportgraphics(figureHandle, pngPath, Resolution=180);
savefig(figureHandle, figPath);
save(matPath, "data", "summary", "options");
writetable(struct2table(summary), summaryPath);
if ~options.Visible
    close(figureHandle);
end

fprintf("MATLAB_FOC_PASS final_rpm=%.2f error_rpm=%.2f peak_phase_current_a=%.3f\n", ...
    summary.FinalSpeedRpm, summary.FinalErrorRpm, summary.PeakPhaseCurrentA);
fprintf("MATLAB_FOC_PLOT=%s\n", pngPath);
fprintf("MATLAB_FOC_DATA=%s\n", matPath);
end
