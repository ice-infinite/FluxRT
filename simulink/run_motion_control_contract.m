function result = run_motion_control_contract(projectRoot,outputDir)
%RUN_MOTION_CONTROL_CONTRACT Independent MATLAB motion-control contract model.
% Implements the planner and Position-P/Velocity-PI/Torque-to-Iq cascade
% directly. It never calls Rust and never consumes a Rust-produced trace.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(projectRoot,'simulation','results','motion');
end
projectRoot = char(projectRoot); outputDir = char(outputDir);
loaded = load_fluxrt_simulation_contract(projectRoot);
scenarioPath = fullfile(projectRoot,'simulation','scenarios', ...
    'motion_control_matrix_v1.json');
[scenario,scenarioBytes] = motion_read_json(scenarioPath);
motion_validate_scenario(scenario,loaded.profile);

rowCount = 0;
for k = 1:numel(scenario.cases)
    c = motion_case_at(scenario.cases,k);
    rowCount = rowCount + double(c.evaluation_ticks);
end
rows = repmat(motion_empty_row(),rowCount,1); cursor = 0;
for k = 1:numel(scenario.cases)
    caseRows = motion_run_case(motion_case_at(scenario.cases,k),scenario.runtime);
    rows(cursor+1:cursor+numel(caseRows)) = caseRows;
    cursor = cursor + numel(caseRows);
end
motion_verify_semantics(scenario,rows);
if ~exist(outputDir,'dir'), mkdir(outputDir); end

workspaceRevision = getenv('FLUXRT_WORKSPACE_REVISION');
if isempty(workspaceRevision), workspaceRevision = 'UNRECORDED'; end
b = loaded.bundle; id = loaded.identity;
metadata = { ...
    'contract_gate_result_version','1'; ...
    'engine_id','fluxrt.matlab.motion.v1'; ...
    'workspace_revision',workspaceRevision; ...
    'bundle_id',char(b.bundle_id); ...
    'bundle_sha256',id.bundleSha256; ...
    'profile_id',char(b.profile_id); ...
    'profile_revision',sprintf('%.0f',b.profile_revision); ...
    'profile_sha256',id.profileSha256; ...
    'base_model_revision',char(loaded.profile.model_revision); ...
    'trace_schema_id',char(b.trace_schema_id); ...
    'trace_schema_version',sprintf('%.0f',b.trace_schema_version); ...
    'trace_schema_sha256',id.traceSchemaSha256; ...
    'comparison_id',char(b.comparison_id); ...
    'comparison_version',sprintf('%.0f',b.comparison_version); ...
    'comparison_sha256',id.comparisonSha256; ...
    'scenario_contract',char(scenario.contract); ...
    'scenario_version',sprintf('%.0f',scenario.version); ...
    'motion_scenario_id',char(scenario.scenario_id); ...
    'motion_scenario_revision',sprintf('%.0f',scenario.revision); ...
    'motion_scenario_sha256',motion_sha256_hex(scenarioBytes); ...
    'motion_model_revision',char(scenario.model_revision); ...
    'case_count',sprintf('%d',numel(scenario.cases)); ...
    'row_count',sprintf('%d',numel(rows)); ...
    'd0','PASS'; 'd1','PASS'; 'd2','PASS'; 'd3','PASS'; 'd4','PASS'; ...
    'e2_acceptance','PASS'; ...
    'feature_off','PASS'};
motion_write_metadata(fullfile(outputDir,'matlab-motion-d0.txt'),metadata);
motion_write_trace(fullfile(outputDir,'matlab-motion-trace.csv'),rows);
result = struct('metadata',{metadata},'rows',rows,'scenario',scenario);
fprintf('FLUXRT_MOTION_PASS D0=PASS D1=PASS D2=PASS D3=PASS D4=PASS cases=%d rows=%d\n', ...
    numel(scenario.cases),numel(rows));
end

function rows = motion_run_case(testCase,runtime)
dt = single(1/double(runtime.control_frequency_hz));
cfg = motion_runtime_config(runtime,dt);
planner = motion_planner_default(); cascade = motion_cascade_default();
planner.enabled = logical(testCase.enabled); cascade.enabled = logical(testCase.enabled);
feedbackBase = struct('position_valid',true,'velocity_valid',true,'iq_valid',true, ...
    'position_rad',single(testCase.initial_feedback.position_rad), ...
    'velocity_rad_s',single(testCase.initial_feedback.velocity_rad_s), ...
    'iq_a',single(testCase.initial_feedback.iq_a));
rows = repmat(motion_empty_row(),double(testCase.evaluation_ticks),1);
for index = 1:double(testCase.evaluation_ticks)
    tick = index-1;
    command = motion_active_command(testCase.commands,tick);
    feedback = motion_feedback_at(feedbackBase,testCase.feedback_faults,tick);
    plannerBefore = planner; cascadeBefore = cascade;
    [plannerNext,reference,plannerStatus] = motion_planner_step(planner,cfg,command,feedback);
    if ~strcmp(plannerStatus,'ok')
        assert(isequaln(planner,plannerBefore) && isequaln(cascade,cascadeBefore), ...
            'FluxRT:Motion:Transaction','planner failure advanced state');
        row = motion_failure_row(testCase,tick,dt,command,feedback,plannerStatus);
    else
        [cascadeNext,output,cascadeStatus] = motion_cascade_step( ...
            cascade,cfg,reference,feedback);
        if ~strcmp(cascadeStatus,'ok')
            assert(isequaln(planner,plannerBefore) && isequaln(cascade,cascadeBefore), ...
                'FluxRT:Motion:Transaction','cascade failure advanced state');
            row = motion_failure_row(testCase,tick,dt,command,feedback,cascadeStatus);
        else
            planner = plannerNext; cascade = cascadeNext;
            row = motion_success_row(testCase,tick,dt,command,feedback,reference,output,cascade);
        end
    end
    rows(index) = row;
    if isfield(testCase,'mechanical_plant')
        assert(strcmp(row.status,'ok'),'FluxRT:Motion:E2', ...
            '%s dynamic acceptance failed at tick %d: %s', ...
            char(testCase.case_id),tick,row.status);
        if index < double(testCase.evaluation_ticks)
            feedbackBase = motion_advance_mechanical_plant( ...
                feedbackBase,output,testCase.mechanical_plant,cfg);
        end
    end
end
if isfield(testCase,'acceptance')
    motion_verify_acceptance_case(testCase,rows,cfg);
end
end

function [next,reference,status] = motion_planner_step(state,cfg,command,feedback)
next = state; reference = motion_reference_default(); status = 'ok';
if ~state.enabled, status = 'planner_disabled'; return; end
if ~motion_valid_config(cfg), status = 'planner_invalid_config'; return; end
if ~motion_valid_command(command), status = 'planner_invalid_command'; return; end
control = char(command.control_mode); input = char(command.input_mode);
if ~any(strcmp(control,{'Torque','Velocity','Position'}))
    status = 'planner_unsupported_control_mode'; return
end
if (feedback.position_valid && ~isfinite(feedback.position_rad)) || ...
        (feedback.velocity_valid && ~isfinite(feedback.velocity_rad_s))
    status = 'planner_invalid_feedback'; return
end
if strcmp(control,'Velocity') && ~feedback.velocity_valid
    status = 'planner_missing_velocity'; return
end
if strcmp(control,'Position') && ~feedback.position_valid
    status = 'planner_missing_position'; return
end
if strcmp(control,'Position') && ~feedback.velocity_valid
    status = 'planner_missing_velocity'; return
end

transition = ~state.initialized || ~strcmp(state.control_mode,control) || ...
    ~strcmp(state.input_mode,input);
if transition
    next = motion_seed_transition(state,cfg,command,feedback,control,input);
else
    next = motion_advance_planner(state,cfg,command,control,input);
end
reference = next.reference;
end

function next = motion_seed_transition(state,cfg,command,feedback,control,input)
[currentLimit,torqueLimit,velocityLimit] = motion_effective_limits(cfg,command);
if state.initialized, torqueSeed = state.reference.torque_ref_nm; else, torqueSeed = single(0); end
torqueSeed = motion_clamp(torqueSeed,-torqueLimit,torqueLimit);
if feedback.velocity_valid, velocitySeed = feedback.velocity_rad_s; else, velocitySeed = single(0); end
velocitySeed = motion_clamp(velocitySeed,-velocityLimit,velocityLimit);
if feedback.position_valid, positionSeed = feedback.position_rad; else, positionSeed = single(0); end
next = state;
next.reference = struct('control_mode',control,'input_mode',input, ...
    'position_ref_rad',positionSeed,'velocity_ref_rad_s',velocitySeed, ...
    'torque_ref_nm',torqueSeed,'velocity_feedforward_rad_s',single(0), ...
    'torque_feedforward_nm',single(0),'current_limit_a',currentLimit, ...
    'torque_limit_nm',torqueLimit,'velocity_limit_rad_s',velocityLimit, ...
    'flags',uint32(1));
next.trajectory_velocity_rad_s = velocitySeed;
next.control_mode = control; next.input_mode = input; next.initialized = true;
end

function next = motion_advance_planner(state,cfg,command,control,input)
[currentLimit,torqueLimit,velocityLimit] = motion_effective_limits(cfg,command);
flags = uint32(0); out = motion_reference_default();
out.control_mode = control; out.input_mode = input;
out.current_limit_a = currentLimit; out.torque_limit_nm = torqueLimit;
out.velocity_limit_rad_s = velocityLimit; next = state;
switch control
    case 'Torque'
        [target,flags] = motion_clamp_flag(single(command.torque_ref_nm),torqueLimit,flags,uint32(2));
        if strcmp(input,'TorqueRamp')
            out.torque_ref_nm = motion_move_towards(state.reference.torque_ref_nm,target, ...
                single(cfg.torque_ramp_rate_nm_s*cfg.dt));
        else
            out.torque_ref_nm = target;
        end
    case 'Velocity'
        [target,flags] = motion_clamp_flag(single(command.velocity_ref_rad_s),velocityLimit,flags,uint32(4));
        if strcmp(input,'VelocityRamp')
            out.velocity_ref_rad_s = motion_move_towards(state.reference.velocity_ref_rad_s,target, ...
                single(cfg.velocity_ramp_rate_rad_s2*cfg.dt));
        else
            out.velocity_ref_rad_s = target;
        end
        [out.torque_feedforward_nm,flags] = motion_clamp_flag( ...
            single(command.torque_feedforward_nm),torqueLimit,flags,uint32(2));
    case 'Position'
        [target,flags] = motion_clamp_position(cfg,single(command.position_ref_rad),flags);
        switch input
            case {'Passthrough','ExternalSynchronized'}
                out.position_ref_rad = target; next.trajectory_velocity_rad_s = single(0);
            case 'PositionFilter'
                bandwidthStep = single(cfg.position_filter_bandwidth_rad_s*cfg.dt);
                alpha = single(bandwidthStep/single(1+bandwidthStep));
                desiredDelta = single(single(target-state.reference.position_ref_rad)*alpha);
                maximumDelta = single(velocityLimit*cfg.dt);
                delta = motion_clamp(desiredDelta,-maximumDelta,maximumDelta);
                if delta ~= desiredDelta, flags = bitor(flags,uint32(4)); end
                out.position_ref_rad = single(state.reference.position_ref_rad+delta);
                out.velocity_ref_rad_s = single(delta/cfg.dt);
                next.trajectory_velocity_rad_s = out.velocity_ref_rad_s;
            case 'TrapezoidalTrajectory'
                [out.position_ref_rad,out.velocity_ref_rad_s] = motion_step_trapezoidal( ...
                    state.reference.position_ref_rad,state.trajectory_velocity_rad_s,target, ...
                    velocityLimit,cfg.trajectory_acceleration_rad_s2, ...
                    cfg.trajectory_deceleration_rad_s2,cfg.dt);
                next.trajectory_velocity_rad_s = out.velocity_ref_rad_s;
        end
        [out.velocity_feedforward_rad_s,flags] = motion_clamp_feedforward( ...
            out.velocity_ref_rad_s,single(command.velocity_feedforward_rad_s), ...
            velocityLimit,flags,uint32(4));
        [out.torque_feedforward_nm,flags] = motion_clamp_flag( ...
            single(command.torque_feedforward_nm),torqueLimit,flags,uint32(2));
end
out.flags = flags; next.reference = out;
end

function [next,output,status] = motion_cascade_step(state,cfg,reference,feedback)
next = state; output = motion_output_default();
if ~state.enabled, status = 'cascade_disabled'; return; end
if ~motion_valid_config(cfg), status = 'cascade_invalid_config'; return; end
if ~motion_valid_reference(reference), status = 'cascade_invalid_reference'; return; end
if bitand(reference.flags,uint32(hex2dec('FFFFFFE0'))) ~= 0
    status = 'cascade_unknown_reference_flags'; return
end
if ~any(strcmp(reference.control_mode,{'Torque','Velocity','Position'}))
    status = 'cascade_unsupported_control_mode'; return
end
if ~motion_compatible(reference.control_mode,reference.input_mode)
    status = 'cascade_unsupported_input_mode'; return
end
if (feedback.position_valid && ~isfinite(feedback.position_rad)) || ...
        (feedback.velocity_valid && ~isfinite(feedback.velocity_rad_s)) || ...
        (feedback.iq_valid && ~isfinite(feedback.iq_a))
    status = 'cascade_invalid_feedback'; return
end
if strcmp(reference.control_mode,'Position') && ~feedback.position_valid
    status = 'cascade_missing_position'; return
end
if any(strcmp(reference.control_mode,{'Velocity','Position'})) && ~feedback.velocity_valid
    status = 'cascade_missing_velocity'; return
end
if bitand(reference.flags,uint32(1)) ~= 0 && ~feedback.iq_valid
    status = 'cascade_missing_current'; return
end
changed = ~state.initialized || ~strcmp(state.control_mode,reference.control_mode) || ...
    ~strcmp(state.input_mode,reference.input_mode);
transition = bitand(reference.flags,uint32(1)) ~= 0;
if changed && ~transition, status = 'cascade_transition_required'; return; end

next = state;
[next,output,status] = motion_cascade_inner(next,cfg,reference,feedback,transition);
if ~strcmp(status,'ok'), next = state; output = motion_output_default(); return; end
next.initialized = true; next.control_mode = reference.control_mode;
next.input_mode = reference.input_mode; next.output = output;
end

function [state,out,status] = motion_cascade_inner(state,cfg,ref,feedback,transition)
status = 'ok'; out = motion_output_default(); flags = uint32(0);
currentTorqueLimit = single(ref.current_limit_a*cfg.torque_constant_nm_per_a);
if ~isfinite(currentTorqueLimit), status = 'cascade_non_finite'; return; end
effectiveTorqueLimit = min(ref.torque_limit_nm,currentTorqueLimit);
if currentTorqueLimit < ref.torque_limit_nm, flags = bitor(flags,uint32(32)); end
if strcmp(ref.control_mode,'Position')
    positionError = single(ref.position_ref_rad-feedback.position_rad);
else
    positionError = single(0);
end
if ~isfinite(positionError), status = 'cascade_non_finite'; return; end
velocityReference = single(0); velocityError = single(0);
switch ref.control_mode
    case 'Torque'
        state.pi = motion_pi_default();
        if transition
            flags = bitor(flags,uint32(4));
            [torqueReference,flags] = motion_clamp_flag( ...
                single(feedback.iq_a*cfg.torque_constant_nm_per_a), ...
                effectiveTorqueLimit,flags,uint32(16));
        else
            [torqueReference,flags] = motion_clamp_flag( ...
                ref.torque_ref_nm,effectiveTorqueLimit,flags,uint32(16));
        end
    case {'Velocity','Position'}
        flags = bitor(flags,uint32(2));
        if strcmp(ref.control_mode,'Position')
            flags = bitor(flags,uint32(1));
            raw = single(single(ref.velocity_ref_rad_s+ref.velocity_feedforward_rad_s) + ...
                single(cfg.position_kp_per_s*positionError));
            if ~isfinite(raw), status = 'cascade_non_finite'; return; end
            [velocityReference,flags] = motion_clamp_flag(raw,ref.velocity_limit_rad_s,flags,uint32(8));
        else
            [velocityReference,flags] = motion_clamp_flag( ...
                ref.velocity_ref_rad_s,ref.velocity_limit_rad_s,flags,uint32(8));
        end
        velocityError = single(velocityReference-feedback.velocity_rad_s);
        if ~isfinite(velocityError), status = 'cascade_non_finite'; return; end
        [feedforward,flags] = motion_clamp_flag( ...
            ref.torque_feedforward_nm,effectiveTorqueLimit,flags,uint32(16));
        pi = struct('kp',cfg.velocity_kp_nm_per_rad_s, ...
            'ki',cfg.velocity_ki_nm_per_rad,'ts',cfg.dt, ...
            'out_min',single(-effectiveTorqueLimit-feedforward), ...
            'out_max',single(effectiveTorqueLimit-feedforward), ...
            'integrator_min',single(-effectiveTorqueLimit-feedforward), ...
            'integrator_max',single(effectiveTorqueLimit-feedforward));
        if transition
            flags = bitor(flags,uint32(4));
            applied = motion_clamp(single(feedback.iq_a*cfg.torque_constant_nm_per_a), ...
                -effectiveTorqueLimit,effectiveTorqueLimit);
            [state.pi,piTorque] = motion_pi_preload(state.pi,pi,velocityReference, ...
                feedback.velocity_rad_s,single(applied-feedforward));
        else
            [state.pi,piTorque] = motion_pi_update(state.pi,pi,velocityReference,feedback.velocity_rad_s);
        end
        if piTorque <= pi.out_min || piTorque >= pi.out_max
            flags = bitor(flags,uint32(16));
        end
        [torqueReference,flags] = motion_clamp_flag(single(piTorque+feedforward), ...
            effectiveTorqueLimit,flags,uint32(16));
end
rawIq = single(torqueReference/cfg.torque_constant_nm_per_a);
if ~isfinite(rawIq), status = 'cascade_non_finite'; return; end
iqRef = motion_clamp(rawIq,-ref.current_limit_a,ref.current_limit_a);
if iqRef ~= rawIq, flags = bitor(flags,bitor(uint32(32),uint32(16))); end
finalTorque = single(iqRef*cfg.torque_constant_nm_per_a);
out = struct('control_mode',ref.control_mode,'input_mode',ref.input_mode, ...
    'position_error_rad',positionError,'velocity_reference_rad_s',velocityReference, ...
    'velocity_error_rad_s',velocityError,'torque_reference_nm',finalTorque, ...
    'id_ref_a',single(0),'iq_ref_a',iqRef,'reference_flags',ref.flags,'flags',flags);
values = [out.position_error_rad out.velocity_reference_rad_s out.velocity_error_rad_s ...
    out.torque_reference_nm out.id_ref_a out.iq_ref_a];
if ~all(isfinite(values)), status = 'cascade_non_finite'; out = motion_output_default(); end
end

function [state,value] = motion_pi_preload(state,p,reference,feedback,desired)
state.error = single(reference-feedback); proportional = single(p.kp*state.error);
state.output = motion_clamp(desired,p.out_min,p.out_max);
state.integrator = motion_clamp(single(state.output-proportional),p.integrator_min,p.integrator_max);
state.output = motion_clamp(single(proportional+state.integrator),p.out_min,p.out_max);
value = state.output;
end

function [state,value] = motion_pi_update(state,p,reference,feedback)
state.error = single(reference-feedback); proportional = single(p.kp*state.error);
state.integrator = single(state.integrator + single(single(p.ki*p.ts)*state.error));
state.integrator = motion_clamp(state.integrator,p.integrator_min,p.integrator_max);
unclamped = single(proportional+state.integrator);
state.output = motion_clamp(unclamped,p.out_min,p.out_max);
if unclamped ~= state.output
    state.integrator = motion_clamp(single(state.output-proportional), ...
        p.integrator_min,p.integrator_max);
end
value = state.output;
end

function command = motion_active_command(commands,tick)
indices = find(arrayfun(@(c)double(c.start_tick)<=tick,commands));
assert(~isempty(indices),'FluxRT:Motion:Command','no active command at tick %d',tick);
command = commands(indices(end));
end

function feedback = motion_feedback_at(base,faults,tick)
feedback = base;
for k = 1:numel(faults)
    f = faults(k);
    if tick >= double(f.start_tick) && tick < double(f.end_tick)
        feedback.position_valid = logical(f.position_valid);
        feedback.velocity_valid = logical(f.velocity_valid);
        feedback.iq_valid = logical(f.current_q_valid);
    end
end
end

function next = motion_advance_mechanical_plant(feedback,out,plant,cfg)
alpha = motion_clamp(single(single(plant.current_response_bandwidth_rad_s)*cfg.dt), ...
    single(0),single(1));
currentError = single(out.iq_ref_a-feedback.iq_a);
nextIq = single(feedback.iq_a+single(alpha*currentError));
electromagneticTorque = single(nextIq*cfg.torque_constant_nm_per_a);
viscousTorque = single(single(plant.viscous_friction_nm_s)*feedback.velocity_rad_s);
netTorque = single(single(electromagneticTorque-single(plant.load_torque_nm))-viscousTorque);
acceleration = single(netTorque/single(plant.inertia_kg_m2));
nextVelocity = single(feedback.velocity_rad_s+single(acceleration*cfg.dt));
nextPosition = single(feedback.position_rad+single(nextVelocity*cfg.dt));
assert(all(isfinite([nextPosition nextVelocity nextIq])), ...
    'FluxRT:Motion:E2','mechanical plant produced non-finite state');
next = feedback;
next.position_rad = nextPosition;
next.velocity_rad_s = nextVelocity;
next.iq_a = nextIq;
end

function motion_verify_acceptance_case(c,rows,cfg)
a = c.acceptance;
peakIq = max(max(abs([rows.feedback_iq_a])),max(abs([rows.output_iq_a])));
peakVelocity = max(abs([rows.feedback_velocity_rad_s]));
assert(peakIq<=double(a.maximum_absolute_iq_a), ...
    'FluxRT:Motion:E2','%s peak iq exceeds acceptance',char(c.case_id));
assert(peakVelocity<=double(a.maximum_absolute_velocity_rad_s), ...
    'FluxRT:Motion:E2','%s peak velocity exceeds acceptance',char(c.case_id));
last = rows(end); command = c.commands(end); mode = char(a.mode);
switch mode
    case 'Torque'
        finalTorque = single(single(last.feedback_iq_a)*cfg.torque_constant_nm_per_a);
        finalError = abs(single(finalTorque-single(command.torque_ref_nm)));
        assert(finalError<=single(a.maximum_final_torque_error_nm), ...
            'FluxRT:Motion:E2','%s final torque error exceeds acceptance',char(c.case_id));
        assert(single(last.feedback_velocity_rad_s)>=single(a.minimum_final_velocity_rad_s), ...
            'FluxRT:Motion:E2','%s final velocity is below acceptance',char(c.case_id));
    case 'Velocity'
        target = single(command.velocity_ref_rad_s);
        finalError = abs(single(single(last.feedback_velocity_rad_s)-target));
        overshoot = max(single(0),single(single(max([rows.feedback_velocity_rad_s]))-target));
        assert(finalError<=single(a.maximum_final_velocity_error_rad_s), ...
            'FluxRT:Motion:E2','%s final velocity error exceeds acceptance',char(c.case_id));
        assert(overshoot<=single(a.maximum_velocity_overshoot_rad_s), ...
            'FluxRT:Motion:E2','%s velocity overshoot exceeds acceptance',char(c.case_id));
    case 'Position'
        target = single(command.position_ref_rad);
        finalError = abs(single(single(last.feedback_position_rad)-target));
        overshoot = max(single(0),single(single(max([rows.feedback_position_rad]))-target));
        finalVelocity = abs(single(last.feedback_velocity_rad_s));
        assert(finalError<=single(a.maximum_final_position_error_rad), ...
            'FluxRT:Motion:E2','%s final position error exceeds acceptance',char(c.case_id));
        assert(overshoot<=single(a.maximum_position_overshoot_rad), ...
            'FluxRT:Motion:E2','%s position overshoot exceeds acceptance',char(c.case_id));
        assert(finalVelocity<=single(a.maximum_final_absolute_velocity_rad_s), ...
            'FluxRT:Motion:E2','%s final velocity exceeds acceptance',char(c.case_id));
    otherwise
        error('FluxRT:Motion:E2','unsupported acceptance mode %s',mode);
end
end

function [current,torque,velocity] = motion_effective_limits(cfg,command)
current = cfg.maximum_current_a; torque = cfg.maximum_torque_nm;
velocity = cfg.maximum_velocity_rad_s;
if command.current_limit_a ~= 0, current = min(current,single(command.current_limit_a)); end
if command.torque_limit_nm ~= 0, torque = min(torque,single(command.torque_limit_nm)); end
if command.velocity_limit_rad_s ~= 0, velocity = min(velocity,single(command.velocity_limit_rad_s)); end
end

function [value,flags] = motion_clamp_position(cfg,value,flags)
if value < cfg.soft_limit_min_rad
    value = cfg.soft_limit_min_rad; flags = bitor(flags,uint32(8));
elseif value > cfg.soft_limit_max_rad
    value = cfg.soft_limit_max_rad; flags = bitor(flags,uint32(16));
end
end

function [value,flags] = motion_clamp_flag(value,limit,flags,flag)
clamped = motion_clamp(value,-limit,limit);
if clamped ~= value, flags = bitor(flags,flag); end
value = clamped;
end

function [value,flags] = motion_clamp_feedforward(base,feedforward,limit,flags,flag)
sumValue = single(base+feedforward); combined = motion_clamp(sumValue,-limit,limit);
if combined ~= sumValue, flags = bitor(flags,flag); end
value = single(combined-base);
end

function value = motion_move_towards(current,target,maximumDelta)
delta = single(target-current);
if delta > maximumDelta
    value = single(current+maximumDelta);
elseif delta < -maximumDelta
    value = single(current-maximumDelta);
else
    value = target;
end
end

function [position,velocity] = motion_step_trapezoidal(position,velocity,target, ...
        maximumVelocity,acceleration,deceleration,dt)
error = single(target-position);
stoppingVelocity = single(sqrt(single(single(2)*deceleration*abs(error))));
if error > 0, direction = single(1); elseif error < 0, direction = single(-1); else, direction = single(0); end
desiredVelocity = single(direction*min(maximumVelocity,stoppingVelocity));
if velocity*desiredVelocity < 0 || abs(desiredVelocity) < abs(velocity)
    rate = deceleration;
else
    rate = acceleration;
end
nextVelocity = motion_clamp(motion_move_towards(velocity,desiredVelocity,single(rate*dt)), ...
    -maximumVelocity,maximumVelocity);
nextPosition = single(position+single(nextVelocity*dt));
if error == 0 || single(error*single(target-nextPosition)) <= 0
    position = target; velocity = single(0);
else
    position = nextPosition; velocity = nextVelocity;
end
end

function value = motion_clamp(value,lower,upper)
value = min(max(value,lower),upper);
end

function tf = motion_valid_config(c)
positive = [c.dt c.maximum_current_a c.maximum_torque_nm c.maximum_velocity_rad_s ...
    c.torque_ramp_rate_nm_s c.velocity_ramp_rate_rad_s2 ...
    c.position_filter_bandwidth_rad_s c.trajectory_acceleration_rad_s2 ...
    c.trajectory_deceleration_rad_s2 c.position_kp_per_s c.torque_constant_nm_per_a];
nonnegative = [c.velocity_kp_nm_per_rad_s c.velocity_ki_nm_per_rad];
tf = all(isfinite(positive)) && all(positive>0) && c.dt<=1 && ...
    all(isfinite(nonnegative)) && all(nonnegative>=0) && sum(nonnegative)>0 && ...
    isfinite(c.soft_limit_min_rad) && isfinite(c.soft_limit_max_rad) && ...
    c.soft_limit_min_rad<c.soft_limit_max_rad;
end

function tf = motion_valid_command(c)
names = {'Torque','Velocity','Position'};
tf = any(strcmp(char(c.control_mode),names)) && ...
    motion_compatible(char(c.control_mode),char(c.input_mode));
values = [c.position_ref_rad c.velocity_ref_rad_s c.torque_ref_nm ...
    c.velocity_feedforward_rad_s c.torque_feedforward_nm c.current_limit_a ...
    c.torque_limit_nm c.velocity_limit_rad_s];
tf = tf && all(isfinite(values)) && all([c.current_limit_a c.torque_limit_nm c.velocity_limit_rad_s]>=0);
if ~tf, return; end
switch char(c.control_mode)
    case 'Torque'
        tf = c.position_ref_rad==0 && c.velocity_ref_rad_s==0 && ...
            c.velocity_feedforward_rad_s==0 && c.torque_feedforward_nm==0 && ...
            c.velocity_limit_rad_s==0;
    case 'Velocity'
        tf = c.position_ref_rad==0 && c.torque_ref_nm==0 && c.velocity_feedforward_rad_s==0;
    case 'Position'
        tf = c.velocity_ref_rad_s==0 && c.torque_ref_nm==0;
end
end

function tf = motion_valid_reference(r)
values = [r.position_ref_rad r.velocity_ref_rad_s r.torque_ref_nm ...
    r.velocity_feedforward_rad_s r.torque_feedforward_nm r.current_limit_a ...
    r.torque_limit_nm r.velocity_limit_rad_s];
tf = all(isfinite(values)) && r.current_limit_a>0 && r.torque_limit_nm>0 && ...
    r.velocity_limit_rad_s>0;
end

function tf = motion_compatible(control,input)
switch input
    case {'Passthrough','ExternalSynchronized'}, tf = ~strcmp(control,'Inactive');
    case 'TorqueRamp', tf = strcmp(control,'Torque');
    case 'VelocityRamp', tf = strcmp(control,'Velocity');
    case {'PositionFilter','TrapezoidalTrajectory'}, tf = strcmp(control,'Position');
    otherwise, tf = false;
end
end

function cfg = motion_runtime_config(r,dt)
cfg = struct('dt',dt,'maximum_current_a',single(r.maximum_current_a), ...
    'maximum_torque_nm',single(r.maximum_torque_nm), ...
    'maximum_velocity_rad_s',single(r.maximum_velocity_rad_s), ...
    'torque_ramp_rate_nm_s',single(r.torque_ramp_rate_nm_s), ...
    'velocity_ramp_rate_rad_s2',single(r.velocity_ramp_rate_rad_s2), ...
    'position_filter_bandwidth_rad_s',single(r.position_filter_bandwidth_rad_s), ...
    'trajectory_acceleration_rad_s2',single(r.trajectory_acceleration_rad_s2), ...
    'trajectory_deceleration_rad_s2',single(r.trajectory_deceleration_rad_s2), ...
    'soft_limit_min_rad',single(r.soft_limit_min_rad), ...
    'soft_limit_max_rad',single(r.soft_limit_max_rad), ...
    'position_kp_per_s',single(r.position_kp_per_s), ...
    'velocity_kp_nm_per_rad_s',single(r.velocity_kp_nm_per_rad_s), ...
    'velocity_ki_nm_per_rad',single(r.velocity_ki_nm_per_rad), ...
    'torque_constant_nm_per_a',single(r.torque_constant_nm_per_a));
end

function s = motion_planner_default()
s = struct('enabled',false,'initialized',false,'control_mode','Inactive', ...
    'input_mode','Inactive','reference',motion_reference_default(), ...
    'trajectory_velocity_rad_s',single(0));
end

function s = motion_cascade_default()
s = struct('enabled',false,'initialized',false,'control_mode','Inactive', ...
    'input_mode','Inactive','pi',motion_pi_default(),'output',motion_output_default());
end

function p = motion_pi_default()
p = struct('integrator',single(0),'error',single(0),'output',single(0));
end

function r = motion_reference_default()
r = struct('control_mode','Inactive','input_mode','Inactive', ...
    'position_ref_rad',single(0),'velocity_ref_rad_s',single(0), ...
    'torque_ref_nm',single(0),'velocity_feedforward_rad_s',single(0), ...
    'torque_feedforward_nm',single(0),'current_limit_a',single(0), ...
    'torque_limit_nm',single(0),'velocity_limit_rad_s',single(0),'flags',uint32(0));
end

function o = motion_output_default()
o = struct('control_mode','Inactive','input_mode','Inactive','position_error_rad',single(0), ...
    'velocity_reference_rad_s',single(0),'velocity_error_rad_s',single(0), ...
    'torque_reference_nm',single(0),'id_ref_a',single(0),'iq_ref_a',single(0), ...
    'reference_flags',uint32(0),'flags',uint32(0));
end

function row = motion_success_row(c,tick,dt,cmd,fb,ref,out,cascade)
row = motion_base_row(c,tick,dt,cmd,fb,'ok');
row.planned_position_rad = double(ref.position_ref_rad);
row.planned_velocity_rad_s = double(ref.velocity_ref_rad_s);
row.planned_torque_nm = double(ref.torque_ref_nm);
row.output_velocity_ref_rad_s = double(out.velocity_reference_rad_s);
row.output_torque_nm = double(out.torque_reference_nm);
row.output_id_a = double(out.id_ref_a); row.output_iq_a = double(out.iq_ref_a);
row.planner_flags = double(ref.flags); row.cascade_flags = double(out.flags);
row.velocity_integrator_nm = double(cascade.pi.integrator);
end

function row = motion_failure_row(c,tick,dt,cmd,fb,status)
row = motion_base_row(c,tick,dt,cmd,fb,status);
end

function row = motion_base_row(c,tick,dt,cmd,fb,status)
row = motion_empty_row(); row.case_id = char(c.case_id); row.control_tick = tick;
row.time_s = double(single(single(tick)*dt)); row.enabled = logical(c.enabled);
row.control_mode = char(cmd.control_mode); row.input_mode = char(cmd.input_mode);
row.status = status; row.command_position_rad = double(cmd.position_ref_rad);
row.command_velocity_rad_s = double(cmd.velocity_ref_rad_s);
row.command_torque_nm = double(cmd.torque_ref_nm);
row.feedback_position_rad = double(fb.position_rad);
row.feedback_velocity_rad_s = double(fb.velocity_rad_s);
row.feedback_iq_a = double(fb.iq_a);
end

function row = motion_empty_row()
row = struct('case_id','','control_tick',0,'time_s',0,'enabled',false, ...
    'control_mode','Inactive','input_mode','Inactive','status','', ...
    'command_position_rad',0,'command_velocity_rad_s',0,'command_torque_nm',0, ...
    'feedback_position_rad',0,'feedback_velocity_rad_s',0,'feedback_iq_a',0, ...
    'planned_position_rad',0,'planned_velocity_rad_s',0,'planned_torque_nm',0, ...
    'output_velocity_ref_rad_s',0,'output_torque_nm',0,'output_id_a',0,'output_iq_a',0, ...
    'planner_flags',0,'cascade_flags',0,'velocity_integrator_nm',0);
end

function motion_validate_scenario(s,p)
motion_exact_fields(s,{'contract','version','scenario_id','revision','units','profile_id', ...
    'profile_revision','model_revision','runtime','cases'},'motion scenario');
assert(strcmp(char(s.contract),'fluxrt-motion-control-scenario') && s.version==1 && ...
    strcmp(char(s.units),'SI'),'FluxRT:Motion:Version','unknown contract/version/units');
assert(strcmp(char(s.profile_id),char(p.profile_id)) && s.profile_revision==p.revision && ...
    ~isempty(char(s.model_revision)), ...
    'FluxRT:Motion:Identity','profile identity or motion model revision mismatch');
runtimeFields = {'control_frequency_hz','maximum_current_a','maximum_torque_nm', ...
    'maximum_velocity_rad_s','torque_ramp_rate_nm_s','velocity_ramp_rate_rad_s2', ...
    'position_filter_bandwidth_rad_s','trajectory_acceleration_rad_s2', ...
    'trajectory_deceleration_rad_s2','soft_limit_min_rad','soft_limit_max_rad', ...
    'position_kp_per_s','velocity_kp_nm_per_rad_s','velocity_ki_nm_per_rad', ...
    'torque_constant_nm_per_a'};
motion_exact_fields(s.runtime,runtimeFields,'motion runtime');
assert(s.runtime.control_frequency_hz==p.control_frequency_hz && ...
    s.runtime.control_frequency_hz>0 && floor(s.runtime.control_frequency_hz)==s.runtime.control_frequency_hz, ...
    'FluxRT:Motion:Timing','control frequency mismatch');
cfg = motion_runtime_config(s.runtime,single(1/double(s.runtime.control_frequency_hz)));
assert(motion_valid_config(cfg),'FluxRT:Motion:Range','invalid runtime');
ids = strings(1,numel(s.cases)); foundDisabled = false;
for k = 1:numel(s.cases)
    c = motion_case_at(s.cases,k); ids(k) = string(c.case_id);
    if isfield(c,'mechanical_plant') || isfield(c,'acceptance')
        assert(isfield(c,'mechanical_plant') && isfield(c,'acceptance'), ...
            'FluxRT:Motion:E2','plant and acceptance must be provided together');
        motion_exact_fields(c,{'case_id','enabled','evaluation_ticks','initial_feedback', ...
            'commands','feedback_faults','mechanical_plant','acceptance'},'dynamic motion case');
    else
        motion_exact_fields(c,{'case_id','enabled','evaluation_ticks','initial_feedback', ...
            'commands','feedback_faults'},'motion case');
    end
    motion_exact_fields(c.initial_feedback,{'position_rad','velocity_rad_s','iq_a'}, ...
        'initial feedback');
    assert(c.evaluation_ticks>0 && c.evaluation_ticks<=100000 && ...
        floor(c.evaluation_ticks)==c.evaluation_ticks && ...
        all(isfinite([c.initial_feedback.position_rad c.initial_feedback.velocity_rad_s ...
        c.initial_feedback.iq_a])),'FluxRT:Motion:Range','invalid case range/feedback');
    foundDisabled = foundDisabled || ~logical(c.enabled);
    assert(~isempty(c.commands) && c.commands(1).start_tick==0, ...
        'FluxRT:Motion:Command','commands must start at tick zero');
    previous = -1;
    for j = 1:numel(c.commands)
        cmd = c.commands(j);
        motion_exact_fields(cmd,{'start_tick','control_mode','input_mode','position_ref_rad', ...
            'velocity_ref_rad_s','torque_ref_nm','velocity_feedforward_rad_s', ...
            'torque_feedforward_nm','current_limit_a','torque_limit_nm','velocity_limit_rad_s'}, ...
            'motion command');
        assert(cmd.start_tick>previous && cmd.start_tick<c.evaluation_ticks && ...
            floor(cmd.start_tick)==cmd.start_tick && motion_valid_command(cmd), ...
            'FluxRT:Motion:Command','invalid or unordered command');
        previous = cmd.start_tick;
    end
    for j = 1:numel(c.feedback_faults)
        f = c.feedback_faults(j);
        motion_exact_fields(f,{'start_tick','end_tick','position_valid','velocity_valid','current_q_valid'}, ...
            'feedback fault');
        assert(f.start_tick<f.end_tick && f.end_tick<=c.evaluation_ticks && ...
            floor(f.start_tick)==f.start_tick && floor(f.end_tick)==f.end_tick, ...
            'FluxRT:Motion:Fault','invalid feedback fault window');
    end
    if isfield(c,'mechanical_plant')
        motion_exact_fields(c.mechanical_plant,{'inertia_kg_m2','viscous_friction_nm_s', ...
            'load_torque_nm','current_response_bandwidth_rad_s'},'mechanical plant');
        pvalues = [c.mechanical_plant.inertia_kg_m2 c.mechanical_plant.viscous_friction_nm_s ...
            c.mechanical_plant.load_torque_nm c.mechanical_plant.current_response_bandwidth_rad_s];
        assert(logical(c.enabled) && isempty(c.feedback_faults) && all(isfinite(pvalues)) && ...
            c.mechanical_plant.inertia_kg_m2>0 && c.mechanical_plant.viscous_friction_nm_s>=0 && ...
            c.mechanical_plant.current_response_bandwidth_rad_s>0, ...
            'FluxRT:Motion:E2','invalid mechanical acceptance plant');
        afields = fieldnames(c.acceptance);
        switch char(c.acceptance.mode)
            case 'Torque'
                expected = {'mode','maximum_absolute_iq_a','maximum_absolute_velocity_rad_s', ...
                    'maximum_final_torque_error_nm','minimum_final_velocity_rad_s'};
                assert(strcmp(char(c.commands(end).control_mode),'Torque'), ...
                    'FluxRT:Motion:E2','torque acceptance mode mismatch');
            case 'Velocity'
                expected = {'mode','maximum_absolute_iq_a','maximum_absolute_velocity_rad_s', ...
                    'maximum_final_velocity_error_rad_s','maximum_velocity_overshoot_rad_s'};
                assert(strcmp(char(c.commands(end).control_mode),'Velocity'), ...
                    'FluxRT:Motion:E2','velocity acceptance mode mismatch');
            case 'Position'
                expected = {'mode','maximum_absolute_iq_a','maximum_absolute_velocity_rad_s', ...
                    'maximum_final_position_error_rad','maximum_position_overshoot_rad', ...
                    'maximum_final_absolute_velocity_rad_s'};
                assert(strcmp(char(c.commands(end).control_mode),'Position'), ...
                    'FluxRT:Motion:E2','position acceptance mode mismatch');
            otherwise
                error('FluxRT:Motion:E2','unknown acceptance mode');
        end
        assert(isequal(sort(string(afields)),sort(string(expected(:)))) && ...
            all(structfun(@(v)~isnumeric(v) || all(isfinite(v(:))) && all(v(:)>0),c.acceptance)), ...
            'FluxRT:Motion:E2','invalid acceptance field set or threshold');
    end
end
assert(numel(unique(ids))==numel(ids) && foundDisabled, ...
    'FluxRT:Motion:Identity','duplicate case or missing disabled case');
end

function motion_verify_semantics(s,rows)
expectedRows = 0;
for k = 1:numel(s.cases)
    c = motion_case_at(s.cases,k);
    expectedRows = expectedRows + double(c.evaluation_ticks);
end
assert(numel(rows)==expectedRows, ...
    'FluxRT:Motion:Rows','row count mismatch');
disabled = rows(~[rows.enabled]);
assert(~isempty(disabled) && all(strcmp({disabled.status},'planner_disabled')) && ...
    motion_rows_are_zero(disabled),'FluxRT:Motion:D0','disabled path is not fail closed');
enabled = rows([rows.enabled]); ok = enabled(strcmp({enabled.status},'ok'));
assert(~isempty(ok) && any(bitand(uint32([ok.planner_flags]),uint32(1))~=0), ...
    'FluxRT:Motion:D1','no valid transition/preload path');
assert(any(arrayfun(@(r)r.planned_position_rad~=r.command_position_rad || ...
    r.planned_velocity_rad_s~=r.command_velocity_rad_s || ...
    r.planned_torque_nm~=r.command_torque_nm,ok)), ...
    'FluxRT:Motion:D2','no shaped reference observed');
assert(any(bitand(uint32([ok.planner_flags]),uint32(30))~=0) || ...
    any(bitand(uint32([ok.cascade_flags]),uint32(56))~=0), ...
    'FluxRT:Motion:D3','no planner/cascade limit observed');
failed = enabled(~strcmp({enabled.status},'ok'));
assert(~isempty(failed) && motion_rows_are_zero(failed) && ...
    any(contains(string({failed.status}),'missing_')), ...
    'FluxRT:Motion:D4','feedback fault did not fail closed');
end

function tf = motion_rows_are_zero(rows)
if isempty(rows), tf = true; return; end
names = {'planned_position_rad','planned_velocity_rad_s','planned_torque_nm', ...
    'output_velocity_ref_rad_s','output_torque_nm','output_id_a','output_iq_a', ...
    'planner_flags','cascade_flags','velocity_integrator_nm'};
tf = true;
for k = 1:numel(names), tf = tf && all([rows.(names{k})]==0); end
end

function c = motion_case_at(cases,index)
if iscell(cases)
    c = cases{index};
else
    c = cases(index);
end
end

function motion_write_metadata(path,metadata)
fid = fopen(path,'wb'); assert(fid>=0,'FluxRT:Motion:IO','cannot create %s',path);
cleanup = onCleanup(@()fclose(fid));
for k = 1:size(metadata,1), fprintf(fid,'%s=%s\n',metadata{k,1},metadata{k,2}); end
clear cleanup
end

function motion_write_trace(path,rows)
fid = fopen(path,'wb'); assert(fid>=0,'FluxRT:Motion:IO','cannot create %s',path);
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,['case_id,control_tick,time_s,enabled,control_mode,input_mode,status,' ...
    'command_position_rad,command_velocity_rad_s,command_torque_nm,' ...
    'feedback_position_rad,feedback_velocity_rad_s,feedback_iq_a,' ...
    'planned_position_rad,planned_velocity_rad_s,planned_torque_nm,' ...
    'output_velocity_ref_rad_s,output_torque_nm,output_id_a,output_iq_a,' ...
    'planner_flags,cascade_flags,velocity_integrator_nm\n']);
for k = 1:numel(rows)
    r = rows(k);
    fprintf(fid,['%s,%.0f,%.9f,%d,%s,%s,%s,' ...
        '%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,' ...
        '%.9f,%.9f,%.9f,%.9f,%.0f,%.0f,%.9f\n'], ...
        r.case_id,r.control_tick,r.time_s,r.enabled,r.control_mode,r.input_mode,r.status, ...
        r.command_position_rad,r.command_velocity_rad_s,r.command_torque_nm, ...
        r.feedback_position_rad,r.feedback_velocity_rad_s,r.feedback_iq_a, ...
        r.planned_position_rad,r.planned_velocity_rad_s,r.planned_torque_nm, ...
        r.output_velocity_ref_rad_s,r.output_torque_nm,r.output_id_a,r.output_iq_a, ...
        r.planner_flags,r.cascade_flags,r.velocity_integrator_nm);
end
clear cleanup
end

function motion_exact_fields(value,expected,label)
assert(isequal(sort(string(fieldnames(value))),sort(string(expected(:)))), ...
    'FluxRT:Motion:Fields','%s field set mismatch',label);
end

function [value,bytes] = motion_read_json(path)
fid = fopen(path,'rb'); assert(fid>=0,'FluxRT:Motion:IO','cannot open %s',path);
cleanup = onCleanup(@()fclose(fid)); bytes = fread(fid,Inf,'*uint8'); clear cleanup
value = jsondecode(native2unicode(bytes','UTF-8'));
assert(isstruct(value) && isscalar(value),'FluxRT:Motion:JSON','root must be object');
end

function hash = motion_sha256_hex(bytes)
digest = javaMethod('getInstance','java.security.MessageDigest','SHA-256');
digest.update(typecast(uint8(bytes),'int8')); raw = typecast(digest.digest(),'uint8');
hash = lower(reshape(dec2hex(raw,2).',1,[]));
end
