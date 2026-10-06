function run_lsi_h2_replay_contract(projectRoot, outputDir)
%RUN_LSI_H2_REPLAY_CONTRACT Independently replay a frozen H2 Ls(I) trace.
% Offline S3 only: this function never opens a serial port or authorises PWM.

arguments
    projectRoot (1,:) char
    outputDir (1,:) char
end

scenarioPath = fullfile(projectRoot, 'simulation', 'scenarios', ...
    'lsi_h2_replay_v1.json');
doc = jsondecode(fileread(scenarioPath));
assert(strcmp(doc.contract, 'fluxrt-lsi-h2-replay'));
assert(doc.version == 1);
assert(doc.sample_rate_hz > 0 && doc.resistance_ohm > 0);
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

sampleRate = double(doc.sample_rate_hz);
resistance = double(doc.resistance_ohm);
candidates = double(doc.candidate_inductance_h(:));
cases = doc.cases;
rows = numel(cases) * numel(candidates);
caseId = strings(rows,1);
angleValid = zeros(rows,1);
angleSource = strings(rows,1);
angleSampleCount = zeros(rows,1);
angleSpanCounts = zeros(rows,1);
mechanicalAngle = zeros(rows,1);
electricalAngle = zeros(rows,1);
candidateL = zeros(rows,1);
transitionCount = zeros(rows,1);
rmse = zeros(rows,1);
maximumResidual = zeros(rows,1);

row = 0;
for caseIndex = 1:numel(cases)
    c = cases(caseIndex);
    assert(c.pole_pairs >= 1 && c.pole_pairs <= 255);
    assert(c.angle_index_valid == 0 || c.angle_index_valid == 1);
    if c.angle_index_valid == 1
        assert(strcmp(c.angle_source, 'as5600-window'));
        assert(c.angle_sample_count >= 3 && c.angle_span_counts <= 2);
    else
        assert(strcmp(c.angle_source, 'unavailable-historical'));
    end
    tracePath = safe_project_path(projectRoot, char(c.trace_path));
    trace = readtable(tracePath, 'VariableNamingRule', 'preserve');
    required = {'control_tick','current_u_a','applied_phase_u_v','flags'};
    assert(all(ismember(required, trace.Properties.VariableNames)));
    flags = double(trace.flags);
    pulse = bitand(uint32(flags), uint32(96)) ~= 0;
    adjacent = diff(double(trace.control_tick)) == 1;
    keep = pulse(1:end-1) & pulse(2:end) & adjacent;
    current = double(trace.current_u_a(1:end-1));
    nextCurrent = double(trace.current_u_a(2:end));
    voltage = double(trace.applied_phase_u_v(1:end-1));
    current = current(keep); nextCurrent = nextCurrent(keep); voltage = voltage(keep);
    assert(numel(current) >= 3);
    if c.angle_index_valid == 1
        electrical = mod(double(c.mechanical_angle_rad) * double(c.pole_pairs), 2*pi);
    else
        electrical = 0.0;
    end
    for candidateIndex = 1:numel(candidates)
        row = row + 1;
        inductance = candidates(candidateIndex);
        assert(isfinite(inductance) && inductance > 0);
        decay = exp(-resistance / sampleRate / inductance);
        predicted = voltage / resistance + ...
            (current - voltage / resistance) * decay;
        residual = nextCurrent - predicted;
        caseId(row) = string(c.id);
        angleValid(row) = double(c.angle_index_valid);
        angleSource(row) = string(c.angle_source);
        angleSampleCount(row) = double(c.angle_sample_count);
        angleSpanCounts(row) = double(c.angle_span_counts);
        mechanicalAngle(row) = double(c.mechanical_angle_rad);
        electricalAngle(row) = electrical;
        candidateL(row) = inductance;
        transitionCount(row) = numel(residual);
        rmse(row) = sqrt(mean(residual.^2));
        maximumResidual(row) = max(abs(residual));
    end
end

result = table(caseId, angleValid, angleSource, angleSampleCount, ...
    angleSpanCounts, mechanicalAngle, electricalAngle, ...
    candidateL, transitionCount, rmse, maximumResidual, ...
    'VariableNames', {'case_id','angle_index_valid','angle_source', ...
    'angle_sample_count','angle_span_counts','mechanical_angle_rad', ...
    'electrical_angle_rad','candidate_inductance_h','transition_count', ...
    'rmse_a','maximum_absolute_residual_a'});
writetable(result, fullfile(outputDir, 'matlab-lsi-h2-replay.csv'));

revision = getenv('FLUXRT_WORKSPACE_REVISION');
if isempty(revision), revision = 'UNRECORDED'; end
fid = fopen(fullfile(outputDir, 'matlab-lsi-h2-replay-d0.txt'), 'w');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'contract=fluxrt-lsi-h2-replay\n');
fprintf(fid, 'version=1\nengine=matlab\n');
fprintf(fid, 'workspace_revision=%s\n', revision);
fprintf(fid, 'case_count=%d\nrow_count=%d\n', numel(cases), height(result));
fprintf(fid, 'angle_evidence=BLOCKED_UNTIL_VALID_INDEX\nresult=PASS\n');
fprintf('FLUXRT_MATLAB_LSI_H2_REPLAY_PASS cases=%d rows=%d\n', ...
    numel(cases), height(result));
end

function path = safe_project_path(projectRoot, relative)
assert(~isempty(relative));
assert(~startsWith(relative, '/') && isempty(regexp(relative, '^[A-Za-z]:', 'once')));
parts = regexp(strrep(relative, '\', '/'), '/', 'split');
assert(all(~strcmp(parts, '..')) && all(~strcmp(parts, '.')) && all(~strcmp(parts, '')));
path = fullfile(projectRoot, parts{:});
end
