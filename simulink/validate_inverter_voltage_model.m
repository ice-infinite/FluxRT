function report = validate_inverter_voltage_model(csvPath, jsonPath)
%VALIDATE_INVERTER_VOLTAGE_MODEL Independently checks the Rust CM2 vector table.
%
% This is a numerical cross-check, not a call into Rust and not hardware proof.
% MATLAB recomputes the current filter, soft zero band, dead-time voltage,
% device drop, observer correction and clamped PWM feed-forward from CSV inputs.

arguments
    csvPath (1,1) string
    jsonPath (1,1) string = ""
end

t = readtable(csvPath,VariableNamingRule="preserve");
required = ["step","pwm_a","pwm_b","pwm_c","current_a","current_b","current_c", ...
    "vbus_v","dead_time_s","pwm_period_s","device_drop_v","zero_band_a", ...
    "filter_alpha","compensation_gain","filtered_a","filtered_b","filtered_c", ...
    "loss_a_v","loss_b_v","loss_c_v","observer_pwm_a","observer_pwm_b", ...
    "observer_pwm_c","feedforward_pwm_a","feedforward_pwm_b","feedforward_pwm_c"];
missing = setdiff(required,string(t.Properties.VariableNames));
if ~isempty(missing)
    error('validate_inverter_voltage_model:schema','Missing columns: %s',strjoin(missing,','));
end
if height(t) < 2
    error('validate_inverter_voltage_model:rows','At least two sequential rows are required.');
end
if any(t.step ~= (0:height(t)-1)')
    error('validate_inverter_voltage_model:sequence','step must be contiguous and zero based.');
end

filtered = zeros(height(t),3);
loss = zeros(height(t),3);
observer = zeros(height(t),3);
feedforward = zeros(height(t),3);
for k = 1:height(t)
    sample = [t.current_a(k),t.current_b(k),t.current_c(k)];
    if k == 1
        filtered(k,:) = sample;
    else
        alpha = t.filter_alpha(k);
        filtered(k,:) = filtered(k-1,:) + alpha*(sample-filtered(k-1,:));
    end
    polarity = soft_polarity(filtered(k,:),t.zero_band_a(k));
    magnitude = 2*t.dead_time_s(k)/t.pwm_period_s(k)*t.vbus_v(k) + ...
        t.device_drop_v(k);
    loss(k,:) = magnitude*polarity;
    pwm = [t.pwm_a(k),t.pwm_b(k),t.pwm_c(k)];
    observer(k,:) = pwm-loss(k,:)/t.vbus_v(k);
    feedforward(k,:) = min(max(pwm + ...
        t.compensation_gain(k)*loss(k,:)/t.vbus_v(k),0),1);
end

rustFiltered = [t.filtered_a,t.filtered_b,t.filtered_c];
rustLoss = [t.loss_a_v,t.loss_b_v,t.loss_c_v];
rustObserver = [t.observer_pwm_a,t.observer_pwm_b,t.observer_pwm_c];
rustFeedforward = [t.feedforward_pwm_a,t.feedforward_pwm_b,t.feedforward_pwm_c];
errors = [abs(filtered-rustFiltered),abs(loss-rustLoss), ...
    abs(observer-rustObserver),abs(feedforward-rustFeedforward)];

report = struct();
report.schema = 'fluxrt-inverter-model-gate-v1';
report.rows = height(t);
report.tolerance = 2e-6;
report.maxAbsoluteError = max(errors,[],'all');
report.maxFilteredCurrentErrorA = max(abs(filtered-rustFiltered),[],'all');
report.maxVoltageLossErrorV = max(abs(loss-rustLoss),[],'all');
report.maxObserverDutyError = max(abs(observer-rustObserver),[],'all');
report.maxFeedforwardDutyError = max(abs(feedforward-rustFeedforward),[],'all');
report.pass = report.maxAbsoluteError <= report.tolerance;
if ~report.pass
    error('validate_inverter_voltage_model:mismatch', ...
        'Rust/MATLAB maximum absolute error %.9g exceeds %.9g.', ...
        report.maxAbsoluteError,report.tolerance);
end

if strlength(jsonPath) > 0
    parent = fileparts(jsonPath);
    if ~isempty(parent) && ~exist(parent,'dir'), mkdir(parent); end
    fid = fopen(jsonPath,'w');
    if fid < 0, error('validate_inverter_voltage_model:write','Cannot write %s.',jsonPath); end
    cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
end
fprintf('INVERTER_MODEL_MATLAB_PASS rows=%d max_abs_error=%.9g tolerance=%.9g\n', ...
    report.rows,report.maxAbsoluteError,report.tolerance);
end

function polarity = soft_polarity(current,band)
if band > 0
    polarity = min(max(current/band,-1),1);
else
    polarity = double(current>0)-double(current<0);
end
end
