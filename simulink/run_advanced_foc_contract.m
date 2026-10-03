function run_advanced_foc_contract(projectRoot, outputDir)
%RUN_ADVANCED_FOC_CONTRACT Independent MATLAB implementation of the shared
% advanced-FOC policy matrix.  This does not call Rust or target firmware.

arguments
    projectRoot (1,:) char
    outputDir (1,:) char
end

scenarioPath = fullfile(projectRoot, 'simulation', 'scenarios', ...
    'advanced_foc_matrix_v1.json');
doc = jsondecode(fileread(scenarioPath));
assert(strcmp(doc.contract, 'fluxrt-advanced-foc-scenario'));
assert(doc.version == 1);
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

cases = doc.cases;
n = numel(cases);
case_id = strings(n, 1);
values = zeros(n, 15);
for k = 1:n
    c = cases(k);
    case_id(k) = string(c.id);
    values(k, :) = run_case(doc.motor, doc.config, ...
        double(doc.control_frequency_hz), c);
end

T = table(case_id, values(:,1), values(:,2), values(:,3), values(:,4), ...
    values(:,5), values(:,6), values(:,7), values(:,8), values(:,9), ...
    values(:,10), values(:,11), values(:,12), values(:,13), values(:,14), ...
    values(:,15), 'VariableNames', {'case_id','id_ref_a','iq_ref_a', ...
    'vd_ff_v','vq_ff_v','injection_alpha_v','injection_beta_v', ...
    'voltage_limit_v','region','modulation','active_features', ...
    'current_limited','hfi_valid','flying_state','flying_angle_rad', ...
    'flying_speed_rad_s'});
writetable(T, fullfile(outputDir, 'matlab-advanced-trace.csv'));

revision = getenv('FLUXRT_WORKSPACE_REVISION');
if isempty(revision), revision = 'UNRECORDED'; end
fid = fopen(fullfile(outputDir, 'matlab-advanced-d0.txt'), 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'contract=fluxrt-advanced-foc-scenario\n');
fprintf(fid, 'version=1\nengine=matlab\n');
fprintf(fid, 'workspace_revision=%s\n', revision);
fprintf(fid, 'case_count=%d\nresult=PASS\n', n);
fprintf('FLUXRT_MATLAB_ADVANCED_CONTRACT_PASS cases=%d\n', n);
end

function row = run_case(m, p, frequency, c)
MTPA=1; FW=2; MTPV=4; DECOUPLE=8; DPWM=16; OVERMOD=32; ...
    HFI=64; FLYING=128;
features = double(c.features);
sqrt3 = sqrt(3.0);
region = 0; modulation = 0; active = 0;
weakeningActive = false; mtpvActive = false;
dpwmActive = false; overmodActive = false;
flyingState = 0; flyingStable = 0; flyingElapsed = 0;
flyingAngle = 0; flyingSpeed = 0;
idRef = double(c.base_id_a); iqRef = double(c.base_iq_a);
vdff = 0; vqff = 0; injA = 0; injB = 0; hfiValid = 0;
vLimit = double(c.linear_voltage_utilization)*double(c.dc_bus_voltage_v)/sqrt3;

for step = 1:double(c.steps) %#ok<NASGU>
    idRef = double(c.base_id_a); iqRef = double(c.base_iq_a);
    baseMagnitude = hypot(idRef, iqRef);
    if baseMagnitude > double(p.current_limit_a)
        scale = double(p.current_limit_a)/baseMagnitude;
        idRef=idRef*scale; iqRef=iqRef*scale;
    end
    demand = hypot(idRef, iqRef);
    region = 0; active = 0; modulation = 0;
    vLimit = double(c.linear_voltage_utilization)*double(c.dc_bus_voltage_v)/sqrt3;

    if logical(c.closed_loop_active)
        if bitand(features,MTPA) ~= 0 && demand >= double(p.mtpa_min_current_a)
            [idRef, iqRef] = mtpa_search(m, demand, sign_nonzero(iqRef), ...
                double(p.mtpa_search_steps));
            region=1; active=bitor(active,MTPA);
        end
        voltageRatio = hypot(double(c.previous_vd_v),double(c.previous_vq_v))/ ...
            max(double(c.dc_bus_voltage_v)/sqrt3, realmin);
        if bitand(features,FW) ~= 0
            if weakeningActive
                if voltageRatio <= double(p.weakening_exit_utilization)
                    weakeningActive=false;
                end
            elseif voltageRatio >= double(p.weakening_entry_utilization)
                weakeningActive=true;
            end
            if weakeningActive
                fwLimit=double(p.weakening_entry_utilization)*double(c.dc_bus_voltage_v)/sqrt3;
                targetId=idRef-double(p.weakening_kp_a_per_v)* ...
                    max(0,hypot(double(c.previous_vd_v),double(c.previous_vq_v))-fwLimit);
                targetId=max(double(p.weakening_id_min_a),min(0,targetId));
                maximumStep=double(p.weakening_slew_a_per_s)/frequency* ...
                    double(p.region_update_divider);
                idRef=move_towards(idRef,targetId,maximumStep);
                region=2; active=FW;
            end
        end
        speed=abs(double(c.electrical_speed_rad_s));
        if bitand(features,MTPV) ~= 0
            if mtpvActive
                if speed <= double(p.mtpv_exit_electrical_speed_rad_s)
                    mtpvActive=false;
                end
            elseif speed >= double(p.mtpv_entry_electrical_speed_rad_s)
                mtpvActive=true;
            end
        else
            mtpvActive=false;
        end
        if mtpvActive && demand > 0
            [candidateD,candidateQ,valid] = mtpv_search(m,demand, ...
                double(c.linear_voltage_utilization)*double(c.dc_bus_voltage_v)/sqrt3, ...
                speed,sign_nonzero(iqRef),double(p.mtpv_search_steps));
            if valid
                idRef=candidateD; iqRef=candidateQ; region=3;
                active=MTPV;
            end
        end
        mag=hypot(idRef,iqRef);
        if mag > double(p.current_limit_a)
            scale=double(p.current_limit_a)/mag; idRef=idRef*scale; iqRef=iqRef*scale;
        end
    end

    if bitand(features,DECOUPLE) ~= 0 && logical(c.closed_loop_active)
        omega=double(c.electrical_speed_rad_s); gain=double(p.decoupling_gain);
        vdff=-gain*omega*double(m.lq_h)*double(c.measured_iq_a);
        vqff=gain*omega*(double(m.ld_h)*double(c.measured_id_a)+double(m.flux_linkage_wb));
        active=bitor(active,DECOUPLE);
    else
        vdff=0; vqff=0;
    end

    prior=hypot(double(c.previous_vd_v),double(c.previous_vq_v))/ ...
        max(double(c.dc_bus_voltage_v)/sqrt3,realmin);
    if bitand(features,OVERMOD) ~= 0
        if overmodActive
            if prior <= double(p.overmodulation_exit_modulation), overmodActive=false; end
        elseif prior >= double(p.overmodulation_entry_modulation)
            overmodActive=true;
        end
    else
        overmodActive=false;
    end
    if overmodActive
        modulation=3; vLimit=double(c.dc_bus_voltage_v)*double(p.overmodulation_max_voltage_ratio);
        active=bitor(active,OVERMOD);
    else
        if bitand(features,DPWM) ~= 0
            if dpwmActive
                if prior <= double(p.dpwm_exit_modulation), dpwmActive=false; end
            elseif prior >= double(p.dpwm_entry_modulation)
                dpwmActive=true;
            end
        else
            dpwmActive=false;
        end
        if dpwmActive
            modulation=1+double(p.dpwm_mode ~= 0); active=bitor(active,DPWM);
        end
    end

    % This matrix deliberately checks the explicit HFI gate in its denied state.
    if bitand(features,HFI) ~= 0 && logical(c.allow_voltage_injection)
        error('HFI active-vector case is not part of V1 matrix');
    end
    injA=0; injB=0; hfiValid=0;

    if bitand(features,FLYING)==0 || ~logical(c.request_flying_start)
        flyingState=0; flyingStable=0; flyingElapsed=0;
    elseif flyingState~=2 && flyingState~=3
        flyingState=1; flyingElapsed=flyingElapsed+1;
        if logical(c.observer_reliable) && abs(double(c.electrical_speed_rad_s)) >= ...
                double(p.flying_start_min_electrical_speed_rad_s)
            flyingStable=flyingStable+1;
            flyingAngle=double(c.electrical_angle_rad); flyingSpeed=double(c.electrical_speed_rad_s);
            if flyingStable >= double(p.flying_start_stable_samples), flyingState=2; end
        else
            flyingStable=0;
        end
        if flyingElapsed >= double(p.flying_start_timeout_samples) && flyingState~=2
            flyingState=3;
        end
    end
    if flyingState~=0, active=bitor(active,FLYING); end
end
currentLimited = hypot(double(c.base_id_a),double(c.base_iq_a)) > ...
    double(p.current_limit_a)+eps('single');
row=[idRef iqRef vdff vqff injA injB vLimit region modulation double(active) ...
    double(currentLimited) hfiValid flyingState flyingAngle flyingSpeed];
end

function [id,iq] = mtpa_search(m,currentMagnitude,iqSign,steps)
best=-1e30; id=0; iq=abs(currentMagnitude);
for i=0:steps
    candidateD=-abs(currentMagnitude)*(i/steps);
    candidateQ=sqrt(max(0,currentMagnitude^2-candidateD^2));
    score=(double(m.flux_linkage_wb)+(double(m.ld_h)-double(m.lq_h))*candidateD)*candidateQ;
    if score>best, best=score; id=candidateD; iq=candidateQ; end
end
iq=iq*iqSign;
end

function [id,iq,valid] = mtpv_search(m,currentLimit,voltageLimit,omega,iqSign,steps)
best=-1e30; id=-currentLimit; iq=0; valid=false;
for i=0:steps
    candidateD=-currentLimit*(i/steps);
    candidateQ=iqSign*sqrt(max(0,currentLimit^2-candidateD^2));
    vd=-omega*double(m.lq_h)*candidateQ;
    vq=omega*(double(m.ld_h)*candidateD+double(m.flux_linkage_wb));
    score=(double(m.flux_linkage_wb)+(double(m.ld_h)-double(m.lq_h))*candidateD)*abs(candidateQ);
    if hypot(vd,vq)<=voltageLimit && score>best
        best=score; id=candidateD; iq=candidateQ; valid=true;
    end
end
end

function value = move_towards(current,target,maximumStep)
delta=target-current;
if abs(delta)<=maximumStep, value=target; else, value=current+sign(delta)*maximumStep; end
end

function value = sign_nonzero(input)
if input<0, value=-1; else, value=1; end
end
