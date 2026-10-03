function result = run_feedback_sensor_contract(projectRoot,outputDir)
%RUN_FEEDBACK_SENSOR_CONTRACT Independent MATLAB D0/D2/D3/D4 feedback model.
% Reads the shared SI profile and feedback fault matrix, but does not call the
% Rust runner or consume the Rust trace.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(projectRoot,'simulation','results','feedback');
end
projectRoot = char(projectRoot); outputDir = char(outputDir);
loaded = load_fluxrt_simulation_contract(projectRoot);
scenarioPath = fullfile(projectRoot,'simulation','scenarios', ...
    'feedback_sensor_fault_matrix_v1.json');
[scenario,scenarioBytes] = read_json_bytes(scenarioPath);
validate_scenario(scenario,loaded.profile);

rowCount = double(scenario.evaluation_ticks)*numel(scenario.cases);
rows = repmat(empty_row(),rowCount,1);
cursor = 0;
for k = 1:numel(scenario.cases)
    caseRows = run_case(scenario.cases(k),scenario,loaded.profile);
    rows(cursor+1:cursor+numel(caseRows)) = caseRows;
    cursor = cursor + numel(caseRows);
end
verify_semantics(scenario,rows);
if ~exist(outputDir,'dir'), mkdir(outputDir); end

workspaceRevision = getenv('FLUXRT_WORKSPACE_REVISION');
if isempty(workspaceRevision), workspaceRevision = 'UNRECORDED'; end
b = loaded.bundle; id = loaded.identity;
metadata = { ...
    'feedback_gate_result_version','1'; ...
    'engine_id','fluxrt.matlab.feedback_sensor.v1'; ...
    'workspace_revision',workspaceRevision; ...
    'base_bundle_id',char(b.bundle_id); ...
    'base_bundle_sha256',id.bundleSha256; ...
    'profile_id',char(b.profile_id); ...
    'profile_revision',sprintf('%.0f',b.profile_revision); ...
    'profile_sha256',id.profileSha256; ...
    'trace_schema_id',char(b.trace_schema_id); ...
    'trace_schema_version',sprintf('%.0f',b.trace_schema_version); ...
    'trace_schema_sha256',id.traceSchemaSha256; ...
    'comparison_id',char(b.comparison_id); ...
    'comparison_version',sprintf('%.0f',b.comparison_version); ...
    'comparison_sha256',id.comparisonSha256; ...
    'feedback_scenario_id',char(scenario.scenario_id); ...
    'feedback_scenario_revision',sprintf('%.0f',scenario.revision); ...
    'feedback_scenario_sha256',sha256_hex(scenarioBytes); ...
    'feedback_model_revision',char(scenario.model_revision); ...
    'case_count',sprintf('%d',numel(scenario.cases)); ...
    'row_count',sprintf('%d',numel(rows)); ...
    'd0','PASS'; 'd2','PASS'; 'd3','PASS'; 'd4','PASS'; 'feature_off','PASS'};
write_metadata(fullfile(outputDir,'matlab-feedback-d0.txt'),metadata);
write_trace(fullfile(outputDir,'matlab-feedback-trace.csv'),rows);
result = struct('metadata',{metadata},'rows',rows,'scenario',scenario);
fprintf(['FLUXRT_FEEDBACK_SENSOR_PASS D0=PASS D2=PASS D3=PASS D4=PASS ' ...
    'feature_off=PASS cases=%d rows=%d\n'],numel(scenario.cases),numel(rows));
end

function rows = run_case(testCase,s,p)
CAL = uint32(1); INDEX = uint32(2); DIR = uint32(4);
VALID_ALL = uint32(31); VALID_COMMON = uint32(28);
STALE = uint32(8); DEGRADED = uint32(16);
frequency = double(p.control_frequency_hz);
maximumAgeUs = ceil(double(s.router.maximum_age_control_ticks)*1e6/frequency);
primary = empty_tracker(); backup = empty_tracker();
active = 'None'; previousState = 'None'; previousMode = 'None';
encoderSequence = uint32(0); hallSequence = uint32(0);
rows = repmat(empty_row(),double(s.evaluation_ticks),1);
for index = 1:double(s.evaluation_ticks)
    tick = index-1; timeS = tick/frequency;
    truthPosition = double(s.initial_mechanical_position_rad) + ...
        double(s.mechanical_velocity_rad_s)*timeS;
    trueElectrical = wrap_tau(truthPosition*double(p.pole_pairs));
    if ~logical(testCase.feature_enabled)
        row = empty_row(); row.case_id = char(testCase.case_id); row.control_tick = tick;
        row.time_s = timeS; row.truth_position_rad = truthPosition;
        row.truth_velocity_rad_s = double(s.mechanical_velocity_rad_s);
        row.feedback_selected_position_rad = truthPosition;
        row.feedback_selected_velocity_rad_s = double(s.mechanical_velocity_rad_s);
        row.feedback_selected_electrical_angle_rad = trueElectrical;
        row.feedback_active_mode = 'Sensorless'; row.feedback_route_state = 'Bypassed';
        row.feedback_valid_flags = double(VALID_ALL); row.feedback_quality_flags = double(DIR);
        if tick==0, row.feedback_event='feature_off'; end
        rows(index)=row; continue
    end

    [encoder,encoderSequence] = encoder_sample(tick,testCase,s,p,encoderSequence);
    [hall,hallSequence] = hall_sample(tick,testCase,s,p,hallSequence);
    if ~isempty(encoder), primary.present=true; primary.sample=encoder; end
    if ~isempty(hall), backup.present=true; backup.sample=hall; end
    nowUs = tick_to_us(tick,frequency);
    [primary,primaryGood] = evaluate_tracker(primary,nowUs,maximumAgeUs,'encoder');
    [backup,backupGood] = evaluate_tracker(backup,nowUs,maximumAgeUs,'hall');
    primaryAcquired = primaryGood && primary.good>=double(s.router.acquire_good_samples);
    primaryRecovered = primaryGood && primary.good>=double(s.router.recovery_good_samples);
    primaryLost = ~primaryGood && primary.bad>=double(s.router.loss_bad_samples);
    backupAcquired = backupGood && backup.good>=double(s.router.acquire_good_samples);
    backupLost = ~backupGood && backup.bad>=double(s.router.loss_bad_samples);
    switch active
        case 'None'
            if primaryAcquired, active='Primary'; elseif backupAcquired, active='Backup'; end
        case 'Primary'
            if primaryLost
                if backupAcquired, active='Backup'; else, active='None'; end
            end
        case 'Backup'
            if primaryRecovered, active='Primary'; elseif backupLost, active='None'; end
    end
    if strcmp(active,'Primary')
        state='Primary'; selected=primary.sample; usable=primaryGood; fallback=false;
        activeMode='IncrementalEncoder';
    elseif strcmp(active,'Backup')
        state='Fallback'; selected=backup.sample; usable=backupGood; fallback=true;
        activeMode='Hall';
    else
        primaryExhausted = primary.present && primaryLost;
        backupExhausted = ~backup.present || backupLost;
        if primaryExhausted && backupExhausted, state='Lost'; else, state='Acquiring'; end
        selected=[]; usable=false; fallback=false; activeMode='None';
    end
    valid=uint32(0); quality=STALE; ageUs=0;
    selectedPosition=0; selectedVelocity=0; selectedElectrical=0;
    if ~isempty(selected)
        ageUs = nowUs-selected.sampled_at_us;
        if usable
            valid=selected.valid_flags; quality=selected.quality_flags;
            selectedPosition=selected.mechanical_position_rad;
            selectedVelocity=selected.mechanical_velocity_rad_s;
            selectedElectrical=selected.electrical_angle_rad;
        else
            quality=bitor(selected.quality_flags,DEGRADED);
            if ageUs>maximumAgeUs, quality=bitor(quality,STALE); end
        end
        if fallback && usable, quality=bitor(quality,DEGRADED); end
    end
    event = transition_event(previousState,previousMode,state);
    previousState=state; previousMode=activeMode;
    row=empty_row(); row.case_id=char(testCase.case_id); row.control_tick=tick;
    row.time_s=timeS; row.feedback_primary_available=~isempty(encoder);
    row.feedback_backup_available=~isempty(hall); row.truth_position_rad=truthPosition;
    row.truth_velocity_rad_s=double(s.mechanical_velocity_rad_s);
    if ~isempty(encoder), row.feedback_encoder_position_rad=encoder.mechanical_position_rad; end
    if ~isempty(hall), row.feedback_hall_electrical_angle_rad=hall.electrical_angle_rad; end
    row.feedback_selected_position_rad=selectedPosition;
    row.feedback_selected_velocity_rad_s=selectedVelocity;
    row.feedback_selected_electrical_angle_rad=selectedElectrical;
    row.feedback_active_mode=activeMode; row.feedback_route_state=state;
    row.feedback_valid_flags=double(valid); row.feedback_quality_flags=double(quality);
    row.feedback_sample_age_s=ageUs*1e-6; row.feedback_event=event;
    rows(index)=row;
end
end

function [sample,sequence] = encoder_sample(tick,c,s,p,sequence)
delay=double(s.encoder.delay_control_ticks); sourceTick=tick-delay;
drop=logical(c.encoder_drop_enabled) && tick>=double(c.encoder_drop_start_tick) && ...
    tick<=double(c.encoder_drop_end_tick);
if sourceTick<0 || drop, sample=[]; return; end
sequence=sequence+1; sourceTime=sourceTick/double(p.control_frequency_hz);
position=double(s.initial_mechanical_position_rad)+double(s.mechanical_velocity_rad_s)*sourceTime;
step=2*pi/double(s.encoder.counts_per_revolution); multi=round(position/step)*step;
mechanical=wrap_tau(multi);
electrical=wrap_tau(mechanical*double(p.pole_pairs)+double(s.encoder.electrical_offset_rad));
quality=bitor(uint32(1),uint32(4));
if ~logical(c.encoder_index_missing), quality=bitor(quality,uint32(2)); end
sample=struct('sequence',sequence,'sampled_at_us',tick_to_us(sourceTick,double(p.control_frequency_hz)), ...
    'valid_flags',uint32(31),'quality_flags',quality, ...
    'mechanical_position_rad',double(single(mechanical)), ...
    'mechanical_velocity_rad_s',double(single(s.mechanical_velocity_rad_s)), ...
    'electrical_angle_rad',double(single(electrical)));
end

function [sample,sequence] = hall_sample(tick,c,s,p,sequence)
delay=double(s.hall.delay_control_ticks); sourceTick=tick-delay;
if sourceTick<0, sample=[]; return; end
sequence=sequence+1; sourceTime=sourceTick/double(p.control_frequency_hz);
mechanical=double(s.initial_mechanical_position_rad)+double(s.mechanical_velocity_rad_s)*sourceTime;
electrical=wrap_tau(mechanical*double(p.pole_pairs));
raw=min(5,floor(electrical/(2*pi/6))); map=double(s.hall.sector_map(:)); mapped=map(raw+1);
angle=(mapped+0.5)*2*pi/6; quality=bitor(uint32(1),uint32(4));
wrong=logical(c.hall_wrong_order_enabled) && tick>=double(c.hall_wrong_order_start_tick) && ...
    tick<=double(c.hall_wrong_order_end_tick);
if wrong, quality=bitor(quality,uint32(16)); end
sample=struct('sequence',sequence,'sampled_at_us',tick_to_us(sourceTick,double(p.control_frequency_hz)), ...
    'valid_flags',uint32(28),'quality_flags',quality,'mechanical_position_rad',0, ...
    'mechanical_velocity_rad_s',double(single(s.mechanical_velocity_rad_s)), ...
    'electrical_angle_rad',double(single(angle)));
end

function [tracker,good] = evaluate_tracker(tracker,nowUs,maxAgeUs,mode)
isNew=tracker.present && tracker.sample.sequence~=tracker.last_evaluated_sequence;
good=tracker.present && nowUs-tracker.sample.sampled_at_us<=maxAgeUs && ...
    sample_gate(tracker.sample,mode);
if good
    tracker.bad=0;
    if isNew, tracker.good=tracker.good+1; tracker.last_evaluated_sequence=tracker.sample.sequence; end
else
    tracker.good=0; tracker.bad=tracker.bad+1;
    if isNew, tracker.last_evaluated_sequence=tracker.sample.sequence; end
end
end

function good = sample_gate(sample,mode)
if bitand(sample.quality_flags,uint32(24))~=0, good=false; return; end
common=uint32(28); direction=uint32(4);
if strcmp(mode,'encoder')
    good=bitand(sample.valid_flags,uint32(31))==uint32(31) && ...
        bitand(sample.quality_flags,uint32(7))==uint32(7);
else
    good=bitand(sample.valid_flags,common)==common && ...
        bitand(sample.quality_flags,bitor(direction,uint32(1)))==bitor(direction,uint32(1));
end
end

function validate_scenario(s,p)
exact_fields(s,{'contract','version','scenario_id','revision','units','profile_id', ...
    'profile_revision','model_revision','evaluation_ticks','initial_mechanical_position_rad', ...
    'mechanical_velocity_rad_s','router','encoder','hall','cases'},'feedback scenario');
assert(strcmp(char(s.contract),'fluxrt-feedback-sensor-scenario') && s.version==1 && ...
    strcmp(char(s.units),'SI'),'FluxRT:Feedback:Version','unknown contract/version/units');
assert(strcmp(char(s.profile_id),char(p.profile_id)) && s.profile_revision==p.revision, ...
    'FluxRT:Feedback:Identity','profile mismatch');
assert(s.revision>0 && s.evaluation_ticks>=4 && s.evaluation_ticks<=100000 && ...
    s.mechanical_velocity_rad_s~=0,'FluxRT:Feedback:Range','invalid scenario range');
exact_fields(s.router,{'primary_mode','backup_mode','maximum_age_control_ticks', ...
    'acquire_good_samples','loss_bad_samples','recovery_good_samples'},'router');
assert(strcmp(char(s.router.primary_mode),'IncrementalEncoder') && ...
    strcmp(char(s.router.backup_mode),'Hall'),'FluxRT:Feedback:Mode','invalid route tuple');
assert(all([s.router.maximum_age_control_ticks s.router.acquire_good_samples ...
    s.router.loss_bad_samples s.router.recovery_good_samples]>0), ...
    'FluxRT:Feedback:Range','router thresholds must be non-zero');
exact_fields(s.encoder,{'counts_per_revolution','delay_control_ticks', ...
    'electrical_offset_rad'},'encoder');
exact_fields(s.hall,{'delay_control_ticks','sector_map'},'hall');
assert(s.encoder.counts_per_revolution>0 && numel(s.hall.sector_map)==6 && ...
    isequal(sort(double(s.hall.sector_map(:)))',(0:5)), ...
    'FluxRT:Feedback:Range','invalid encoder/Hall configuration');
ids=strings(1,numel(s.cases)); foundOff=false;
for k=1:numel(s.cases)
    c=s.cases(k); exact_fields(c,{'case_id','feature_enabled','encoder_drop_enabled', ...
        'encoder_drop_start_tick','encoder_drop_end_tick','encoder_index_missing', ...
        'hall_wrong_order_enabled','hall_wrong_order_start_tick','hall_wrong_order_end_tick'}, ...
        'feedback case');
    ids(k)=string(c.case_id); foundOff=foundOff || (ids(k)=="feature_off" && ~c.feature_enabled);
    if c.encoder_drop_enabled
        assert(c.encoder_drop_start_tick<=c.encoder_drop_end_tick && ...
            c.encoder_drop_end_tick<s.evaluation_ticks,'FluxRT:Feedback:Range','bad encoder window');
    end
    if c.hall_wrong_order_enabled
        assert(c.hall_wrong_order_start_tick<=c.hall_wrong_order_end_tick && ...
            c.hall_wrong_order_end_tick<s.evaluation_ticks,'FluxRT:Feedback:Range','bad Hall window');
    end
end
assert(numel(unique(ids))==numel(ids) && foundOff, ...
    'FluxRT:Feedback:Identity','duplicate cases or missing feature_off');
end

function verify_semantics(s,rows)
assert(numel(rows)==double(s.evaluation_ticks)*numel(s.cases), ...
    'FluxRT:Feedback:Rows','row count mismatch');
off=rows(strcmp({rows.case_id},'feature_off'));
assert(numel(off)==double(s.evaluation_ticks) && all(strcmp({off.feedback_route_state},'Bypassed')) && ...
    all(strcmp({off.feedback_active_mode},'Sensorless')) && ...
    all([off.feedback_selected_position_rad]==[off.truth_position_rad]) && ...
    all([off.feedback_selected_velocity_rad_s]==[off.truth_velocity_rad_s]), ...
    'FluxRT:Feedback:FeatureOff','feature-off is not exactly equivalent');
drop=rows(strcmp({rows.case_id},'encoder_drop_fallback'));
assert(any([drop.control_tick]==7 & strcmp({drop.feedback_event},'fallback_entered')) && ...
    any([drop.control_tick]==12 & strcmp({drop.feedback_event},'primary_recovered')), ...
    'FluxRT:Feedback:D2','fallback/recovery timing mismatch');
missing=rows(strcmp({rows.case_id},'encoder_index_missing'));
assert(any(strcmp({missing.feedback_route_state},'Fallback')), ...
    'FluxRT:Feedback:D4','missing Index did not force fallback');
combined=rows(strcmp({rows.case_id},'hall_wrong_order_and_primary_drop'));
assert(any(strcmp({combined.feedback_route_state},'Lost') & [combined.feedback_valid_flags]==0), ...
    'FluxRT:Feedback:D4','combined fault did not fail closed');
end

function event = transition_event(previousState,previousMode,state)
if strcmp(state,'Fallback') && (~strcmp(previousState,'Fallback') || ~strcmp(previousMode,'Hall'))
    event='fallback_entered';
elseif strcmp(state,'Primary') && strcmp(previousState,'Fallback')
    event='primary_recovered';
elseif strcmp(state,'Primary') && ~strcmp(previousState,'Primary')
    event='primary_acquired';
elseif strcmp(state,'Lost') && ~strcmp(previousState,'Lost')
    event='feedback_lost';
else
    event='none';
end
end

function tracker = empty_tracker()
tracker=struct('present',false,'sample',[],'last_evaluated_sequence',uint32(0), ...
    'good',0,'bad',0);
end

function row = empty_row()
row=struct('case_id','','control_tick',0,'time_s',0, ...
    'feedback_primary_available',false,'feedback_backup_available',false, ...
    'truth_position_rad',0,'truth_velocity_rad_s',0, ...
    'feedback_encoder_position_rad',NaN,'feedback_hall_electrical_angle_rad',NaN, ...
    'feedback_selected_position_rad',0,'feedback_selected_velocity_rad_s',0, ...
    'feedback_selected_electrical_angle_rad',0,'feedback_active_mode','None', ...
    'feedback_route_state','Acquiring','feedback_valid_flags',0, ...
    'feedback_quality_flags',0,'feedback_sample_age_s',0,'feedback_event','none');
end

function write_metadata(path,metadata)
fid=fopen(path,'wb'); assert(fid>=0,'FluxRT:Feedback:IO','cannot create %s',path);
cleanup=onCleanup(@()fclose(fid));
for k=1:size(metadata,1), fprintf(fid,'%s=%s\n',metadata{k,1},metadata{k,2}); end
clear cleanup
end

function write_trace(path,rows)
fid=fopen(path,'wb'); assert(fid>=0,'FluxRT:Feedback:IO','cannot create %s',path);
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,['case_id,control_tick,time_s,feedback_primary_available,feedback_backup_available,' ...
    'truth_position_rad,truth_velocity_rad_s,feedback_encoder_position_rad,' ...
    'feedback_hall_electrical_angle_rad,feedback_selected_position_rad,' ...
    'feedback_selected_velocity_rad_s,feedback_selected_electrical_angle_rad,' ...
    'feedback_active_mode,feedback_route_state,feedback_valid_flags,' ...
    'feedback_quality_flags,feedback_sample_age_s,feedback_event\n']);
for k=1:numel(rows)
    r=rows(k);
    fprintf(fid,'%s,%.0f,%.9f,%d,%d,%.9f,%.9f,%s,%s,%.9f,%.9f,%.9f,%s,%s,%.0f,%.0f,%.9f,%s\n', ...
        r.case_id,r.control_tick,r.time_s,r.feedback_primary_available, ...
        r.feedback_backup_available,r.truth_position_rad,r.truth_velocity_rad_s, ...
        optional_number(r.feedback_encoder_position_rad), ...
        optional_number(r.feedback_hall_electrical_angle_rad), ...
        r.feedback_selected_position_rad,r.feedback_selected_velocity_rad_s, ...
        r.feedback_selected_electrical_angle_rad,r.feedback_active_mode, ...
        r.feedback_route_state,r.feedback_valid_flags,r.feedback_quality_flags, ...
        r.feedback_sample_age_s,r.feedback_event);
end
clear cleanup
end

function text = optional_number(value)
if isnan(value), text=''; else, text=sprintf('%.9f',value); end
end

function value = tick_to_us(tick,frequency)
value=floor(double(tick)*1e6/double(frequency));
end

function value = wrap_tau(value)
value=mod(value,2*pi);
end

function exact_fields(value,expected,label)
assert(isequal(sort(string(fieldnames(value))),sort(string(expected(:)))), ...
    'FluxRT:Feedback:Fields','%s field set mismatch',label);
end

function [value,bytes] = read_json_bytes(path)
fid=fopen(path,'rb'); assert(fid>=0,'FluxRT:Feedback:IO','cannot open %s',path);
cleanup=onCleanup(@()fclose(fid)); bytes=fread(fid,Inf,'*uint8'); clear cleanup
value=jsondecode(native2unicode(bytes','UTF-8'));
assert(isstruct(value) && isscalar(value),'FluxRT:Feedback:JSON','root must be object');
end

function hash = sha256_hex(bytes)
digest=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
digest.update(typecast(uint8(bytes),'int8')); raw=typecast(digest.digest(),'uint8');
hash=lower(reshape(dec2hex(raw,2).',1,[]));
end
