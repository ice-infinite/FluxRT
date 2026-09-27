function report = run_phase_voltage_fault_scenarios(options)
%RUN_PHASE_VOLTAGE_FAULT_SCENARIOS Independent MATLAB A24 fault matrix.
%
% Rebuilds the phase-voltage fixtures and quality decisions with MATLAB
% formulas, then compares state/source semantics with the Python reference.
% This is PC-only evidence: it opens no serial port and grants no hardware or
% observer authorization. Thresholds are deterministic test fixtures, not
% production calibration.

arguments
    options.ReferencePath (1,1) string = ""
    options.OutputPath (1,1) string = ""
end

scriptDir = fileparts(mfilename("fullpath"));
projectDir = fileparts(fileparts(scriptDir));
if strlength(options.ReferencePath) == 0
    referencePath = fullfile(projectDir, "profiles", "identification", ...
        "evidence", "phase-voltage-20260927", "a24_fault_scenario_matrix.json");
else
    referencePath = options.ReferencePath;
end
if strlength(options.OutputPath) == 0
    outputPath = fullfile(projectDir, "profiles", "identification", ...
        "evidence", "phase-voltage-20260927", "a24_matlab_fault_scenario_matrix.json");
else
    outputPath = options.OutputPath;
end

cfg = qualityConfig();
scenarios = buildScenarios(cfg);
rows = repmat(emptyRow(), 1, numel(scenarios));
for index = 1:numel(scenarios)
    quality = evaluateQuality(scenarios(index), cfg);
    rows(index).name = scenarios(index).name;
    rows(index).quality_state = quality.state;
    rows(index).quality_reason = quality.reason;
    rows(index).command_model_selection = "CommandModel";
    if quality.measuredEligible
        rows(index).measured_selection = "Measured";
        rows(index).hybrid_selection = "Measured";
    else
        rows(index).measured_selection = "Unavailable";
        rows(index).hybrid_selection = "CommandModel";
    end
end

reference = jsondecode(fileread(referencePath));
mismatches = strings(0, 1);
for index = 1:numel(rows)
    refIndex = find(strcmp({reference.scenarios.name}, rows(index).name), 1);
    if isempty(refIndex)
        mismatches(end + 1) = "missing_reference:" + rows(index).name; %#ok<AGROW>
        continue
    end
    ref = reference.scenarios(refIndex);
    checks = [
        string(ref.quality.state) == rows(index).quality_state
        string(ref.selection.CommandModel.selected_source) == rows(index).command_model_selection
        string(ref.selection.Measured.selected_source) == rows(index).measured_selection
        string(ref.selection.Hybrid.selected_source) == rows(index).hybrid_selection
    ];
    if ~all(checks)
        mismatches(end + 1) = "semantic_mismatch:" + rows(index).name; %#ok<AGROW>
    end
end
if numel(reference.scenarios) ~= numel(rows)
    mismatches(end + 1) = "scenario_count"; %#ok<AGROW>
end

report = struct();
report.schema = "fluxrt.phase-voltage-fault-scenarios.matlab";
report.schema_version = 1;
report.scope = "PC-only independent MATLAB formulas; no hardware authorization";
report.thresholds_are_production_approved = false;
report.reference_schema = string(reference.schema);
report.matlab_version = string(version);
report.scenario_count = numel(rows);
report.mismatch_count = numel(mismatches);
report.mismatches = mismatches;
report.scenarios = rows;

payload = jsonencode(report, PrettyPrint=true);
parentDir = fileparts(outputPath);
if ~isfolder(parentDir)
    mkdir(parentDir);
end
fileId = fopen(outputPath, "w", "n", "UTF-8");
if fileId < 0
    error("FluxRT:PhaseVoltage:OutputOpen", "Cannot open output: %s", outputPath);
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, "%s\n", payload);

if ~isempty(mismatches)
    error("FluxRT:PhaseVoltage:Mismatch", ...
        "MATLAB/Python phase-voltage semantics differ: %s", strjoin(mismatches, ","));
end
fprintf("MATLAB_PHASE_VOLTAGE_PASS scenarios=%d mismatches=0\n", numel(rows));
fprintf("MATLAB_PHASE_VOLTAGE_JSON=%s\n", outputPath);
end

function cfg = qualityConfig()
cfg = struct(...
    adcMaxCode=4095, ...
    phaseFullScaleV=18.3, ...
    rawLowRailMax=8, ...
    rawHighRailMin=4087, ...
    maxAgeTicks=2, ...
    lineResidualLimitV=0.35, ...
    openMeasuredSpanMaxV=0.04, ...
    openExpectedSpanMinV=0.50);
end

function scenarios = buildScenarios(cfg)
steps = (0:7).';
angle = 2.0 * pi * steps / 8.0;
phase = [0.0, -2.0 * pi / 3.0, 2.0 * pi / 3.0];
expected = 6.15 + 2.0 * sin(angle + phase);
identity = voltsToRaw(expected, cfg);

scenarios = repmat(emptyScenario(), 1, 13);
scenarios(1) = scenario("baseline", expected, identity);
scenarios(2) = scenario("offset", expected, voltsToRaw(expected + 0.08, cfg));
scenarios(3) = scenario("gain", expected, voltsToRaw(expected * 1.01, cfg));

raw = identity;
quantum = 2^(12 - 9);
raw = round(raw / quantum) * quantum;
scenarios(4) = scenario("quantization", expected, raw);

scenarios(5) = scenario("delay", expected(1:end-2, :), identity(1:end-2, :));
scenarios(5).ageTicks = 2;
scenarios(6) = scenario("stale", expected, identity);
scenarios(6).ageTicks = cfg.maxAgeTicks + 1;
scenarios(7) = scenario("dropout", expected, identity);
scenarios(7).sampleAvailable = false;

raw = identity;
raw(end, 1) = 0;
scenarios(8) = scenario("low_saturation", expected, raw);
raw = identity;
raw(end, 2) = cfg.adcMaxCode;
scenarios(9) = scenario("high_saturation", expected, raw);

measured = expected;
measured(:, 3) = expected(1, 3);
scenarios(10) = scenario("open_suspicion", expected, voltsToRaw(measured, cfg));
measured = expected;
measured(:, 2) = measured(:, 2) + 0.80;
scenarios(11) = scenario("three_phase_inconsistency", expected, voltsToRaw(measured, cfg));
scenarios(12) = scenario("unconfigured", expected, identity);
scenarios(12).configured = false;
scenarios(13) = scenario("uncalibrated", expected, identity);
scenarios(13).calibrated = false;
end

function value = emptyScenario()
value = struct(name="", expected=zeros(0, 3), raw=zeros(0, 3), ...
    configured=true, calibrated=true, sampleAvailable=true, ageTicks=0);
end

function value = scenario(name, expected, raw)
value = emptyScenario();
value.name = name;
value.expected = expected;
value.raw = raw;
end

function raw = voltsToRaw(volts, cfg)
clipped = min(cfg.phaseFullScaleV, max(0.0, volts));
raw = round(clipped * cfg.adcMaxCode / cfg.phaseFullScaleV);
end

function volts = rawToVolts(raw, cfg)
volts = raw * cfg.phaseFullScaleV / cfg.adcMaxCode;
end

function quality = evaluateQuality(value, cfg)
if ~value.configured
    state = "unconfigured";
    reason = "phase_voltage_path_not_configured";
elseif ~value.calibrated
    state = "uncalibrated";
    reason = "board_calibration_not_observer_eligible";
elseif ~value.sampleAvailable || isempty(value.raw)
    state = "dropout";
    reason = "sample_not_available";
elseif value.ageTicks > cfg.maxAgeTicks
    state = "stale";
    reason = "sample_age_exceeds_limit";
elseif any(value.raw(end, :) <= cfg.rawLowRailMax)
    state = "low_saturation";
    reason = "adc_code_at_low_rail";
elseif any(value.raw(end, :) >= cfg.rawHighRailMin)
    state = "high_saturation";
    reason = "adc_code_at_high_rail";
else
    measured = rawToVolts(value.raw, cfg);
    expectedSpan = max(value.expected, [], 1) - min(value.expected, [], 1);
    measuredSpan = max(measured, [], 1) - min(measured, [], 1);
    stuck = expectedSpan >= cfg.openExpectedSpanMinV & ...
        measuredSpan <= cfg.openMeasuredSpanMaxV;
    if any(stuck)
        state = "open_suspicion";
        reason = "excited_phase_is_stuck";
    else
        measuredLine = [measured(:, 1) - measured(:, 2), ...
            measured(:, 2) - measured(:, 3), measured(:, 3) - measured(:, 1)];
        expectedLine = [value.expected(:, 1) - value.expected(:, 2), ...
            value.expected(:, 2) - value.expected(:, 3), ...
            value.expected(:, 3) - value.expected(:, 1)];
        maximumResidual = max(abs(measuredLine - expectedLine), [], "all");
        if maximumResidual > cfg.lineResidualLimitV
            state = "three_phase_inconsistency";
            reason = "line_to_line_residual_exceeds_limit";
        else
            state = "valid";
            reason = "quality_gate_passed";
        end
    end
end
quality = struct(state=state, reason=reason, measuredEligible=(state == "valid"));
end

function row = emptyRow()
row = struct(name="", quality_state="", quality_reason="", ...
    command_model_selection="", measured_selection="", hybrid_selection="");
end
