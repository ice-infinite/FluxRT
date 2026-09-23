function summary = compare_foc_traces(options)
%COMPARE_FOC_TRACES Plot the same 582 rpm scenario from PC and real board.

arguments
    options.SimulationCsv (1,1) string = ""
    options.HardwareCsv (1,1) string = ""
    options.OutputDir (1,1) string = ""
    options.OutputStem (1,1) string = "foc_sim_vs_hardware"
    options.Visible (1,1) logical = usejava("desktop")
end

scriptDir = fileparts(mfilename("fullpath"));
projectDir = fileparts(fileparts(scriptDir));
if strlength(options.OutputDir) == 0
    outputDir = fullfile(projectDir, "simulation", "results");
else
    outputDir = options.OutputDir;
end
if strlength(options.SimulationCsv) == 0
    options.SimulationCsv = fullfile(outputDir, "bringup_sim_582rpm_12v3_default_final.csv");
end
if strlength(options.HardwareCsv) == 0
    options.HardwareCsv = fullfile(outputDir, "hardware_582rpm_12v3_default_final.csv");
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

sim = readtable(options.SimulationCsv, VariableNamingRule="preserve");
hw = readtable(options.HardwareCsv, VariableNamingRule="preserve");
required = ["time_s", "state", "iq_ref_a", "iq_a", "id_a", "vd_v", ...
    "vq_v", "duty_a", "observer_speed_rpm", "observer_reliable"];
if ~isempty(setdiff(required, string(sim.Properties.VariableNames))) || ...
        ~isempty(setdiff(required, string(hw.Properties.VariableNames)))
    error("FOC:TraceSchemaMismatch", "Correlation CSV schema mismatch.");
end

simIqAtHw = interp1(sim.time_s, sim.iq_a, hw.time_s, "linear", "extrap");
simIdAtHw = interp1(sim.time_s, sim.id_a, hw.time_s, "linear", "extrap");
simVqAtHw = interp1(sim.time_s, sim.vq_v, hw.time_s, "linear", "extrap");
simVdAtHw = interp1(sim.time_s, sim.vd_v, hw.time_s, "linear", "extrap");
phasePeakSim = max(abs([sim.phase_current_a; sim.phase_current_b; sim.phase_current_c]));
phasePeakHw = max(abs([hw.phase_current_a; hw.phase_current_b; hw.phase_current_c]));
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

visibility = "off";
if options.Visible
    visibility = "on";
end
figureHandle = figure(Name="FOC simulation vs hardware", Color="white", ...
    Visible=visibility, Position=[80 60 1500 920]);
layout = tiledlayout(figureHandle, 3, 2, TileSpacing="compact", Padding="compact");

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

nexttile(layout);
plot(sim.time_s, sim.true_speed_rpm, LineWidth=1.3); hold on;
plot(sim.time_s, sim.observer_speed_rpm, LineWidth=0.9);
plot(hw.time_s, hw.observer_speed_rpm, "--", LineWidth=0.9);
grid on; xlabel("Time (s)"); ylabel("Speed (rpm)");
legend("Plant truth", "Observer sim", "Observer hw", Location="best");
title("Speed: hardware truth is not measured");

nexttile(layout);
plot(sim.time_s, sim.duty_a, LineWidth=1.0); hold on;
plot(hw.time_s, hw.duty_a, "--", LineWidth=1.0);
plot(sim.time_s, sim.duty_b, LineWidth=0.8);
plot(hw.time_s, hw.duty_b, "--", LineWidth=0.8);
grid on; ylim([0 1]); xlabel("Time (s)"); ylabel("Duty");
legend("A sim", "A hw", "B sim", "B hw", Location="best"); title("PWM duty");

title(layout, sprintf(["Same firmware path, 12 kHz / 12.3 V / 582 rpm | " + ...
    "Iq RMSE %.3f A | peak %.3f/%.3f A"], ...
    summary.IqRmseA, phasePeakSim, phasePeakHw));

pngPath = fullfile(outputDir, options.OutputStem + ".png");
figPath = fullfile(outputDir, options.OutputStem + ".fig");
matPath = fullfile(outputDir, options.OutputStem + ".mat");
summaryPath = fullfile(outputDir, options.OutputStem + "_summary.csv");
exportgraphics(figureHandle, pngPath, Resolution=180);
savefig(figureHandle, figPath);
save(matPath, "sim", "hw", "summary", "options");
writetable(struct2table(summary), summaryPath);
if ~options.Visible
    close(figureHandle);
end

fprintf("MATLAB_FOC_CORRELATION_PASS iq_rmse_a=%.4f peak_sim_a=%.3f peak_hw_a=%.3f\n", ...
    summary.IqRmseA, phasePeakSim, phasePeakHw);
fprintf("MATLAB_FOC_CORRELATION_PLOT=%s\n", pngPath);
end
