function run_hfi_active_plant_contract(projectRoot, outputDir)
%RUN_HFI_ACTIVE_PLANT_CONTRACT Independent active rotating-HFI plant model.
% This is an offline S3 model. It does not call Rust, target firmware or PWM
% hardware and therefore cannot authorise voltage injection on a board.

arguments
    projectRoot (1,:) char
    outputDir (1,:) char
end

scenarioPath = fullfile(projectRoot, 'simulation', 'scenarios', ...
    'hfi_active_plant_v1.json');
doc = jsondecode(fileread(scenarioPath));
assert(strcmp(doc.contract, 'fluxrt-hfi-active-plant'));
assert(doc.version == 1);
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

frequency = double(doc.control_frequency_hz);
dt = 1.0/frequency;
p = doc.injection;
cases = doc.cases;
n = numel(cases);
case_id = strings(n,1);
values = zeros(n,7);
for k = 1:n
    c = cases(k);
    case_id(k) = string(c.id);
    values(k,:) = run_case(c,p,dt,double(doc.samples));
    if logical(values(k,7)) ~= logical(c.expected_valid)
        error('case %s validity mismatch', c.id);
    end
end

T = table(case_id,values(:,1),values(:,2),values(:,3),values(:,4), ...
    values(:,5),values(:,6),values(:,7), ...
    'VariableNames',{'case_id','true_theta_mod_pi_rad', ...
    'estimated_theta_mod_pi_rad','angle_error_rad','response_magnitude_a', ...
    'relative_saliency','peak_current_a','valid'});
writetable(T,fullfile(outputDir,'matlab-hfi-trace.csv'));

revision = getenv('FLUXRT_WORKSPACE_REVISION');
if isempty(revision), revision='UNRECORDED'; end
fid=fopen(fullfile(outputDir,'matlab-hfi-d0.txt'),'w');
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'contract=fluxrt-hfi-active-plant\n');
fprintf(fid,'version=1\nengine=matlab\n');
fprintf(fid,'workspace_revision=%s\n',revision);
fprintf(fid,'case_count=%d\nresult=PASS\n',n);
fprintf('FLUXRT_MATLAB_HFI_CONTRACT_PASS cases=%d\n',n);
end

function row = run_case(c,p,dt,samples)
rs=double(c.resistance_ohm); ld=double(c.ld_h); lq=double(c.lq_h);
theta=double(c.theta_rad); amplitude=double(p.amplitude_v);
carrierFrequency=double(p.frequency_hz); alpha=double(p.demod_alpha);
phaseOffset=negative_sequence_phase(rs,ld,lq,carrierFrequency,dt);

current=[0;0]; voltage=[0;0]; negative=[0;0]; carrier=0;
peakCurrent=0; estimate=0; response=0;
ct=cos(theta); st=sin(theta);
L=[ld*ct^2+lq*st^2, (ld-lq)*ct*st; ...
   (ld-lq)*ct*st, ld*st^2+lq*ct^2];
for step=1:samples %#ok<NASGU>
    current=current+(L\(voltage-rs*current))*dt;
    peakCurrent=max(peakCurrent,hypot(current(1),current(2)));

    cc=cos(carrier); sc=sin(carrier);
    demod=[current(1)*cc-current(2)*sc; ...
           current(1)*sc+current(2)*cc];
    negative=negative+alpha*(demod-negative);
    response=hypot(negative(1),negative(2));
    twiceTheta=mod(atan2(negative(2),negative(1))-phaseOffset,2*pi);
    estimate=0.5*twiceTheta;

    carrier=mod(carrier+2*pi*carrierFrequency*dt,2*pi);
    voltage=amplitude*[cos(carrier);sin(carrier)];
end
truth=mod(theta,pi);
angleError=abs(mod(estimate-truth+0.5*pi,pi)-0.5*pi);
relativeSaliency=abs(lq-ld)/max(ld,lq);
valid=relativeSaliency>=double(p.minimum_relative_saliency) && ...
    response>=double(p.minimum_response_a) && ...
    angleError<=double(p.maximum_angle_error_rad);
row=[truth estimate angleError response relativeSaliency peakCurrent double(valid)];
end

function phase = negative_sequence_phase(rs,ld,lq,frequency,dt)
step=2*pi*frequency*dt;
hd=discrete_axis_response(rs,ld,dt,step);
hq=discrete_axis_response(rs,lq,dt,step);
coefficient=conj(hd-hq)/2;
phase=angle(coefficient);
end

function response = discrete_axis_response(rs,inductance,dt,step)
a=1-rs*dt/inductance;
b=dt/inductance;
response=b/(1-a*exp(-1i*step));
end
