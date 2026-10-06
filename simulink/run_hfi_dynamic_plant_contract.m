function run_hfi_dynamic_plant_contract(projectRoot, outputDir)
%RUN_HFI_DYNAMIC_PLANT_CONTRACT Independent moving-rotor active-HFI model.
% Offline S3 only: no target I/O and no permission to inject a real motor.

arguments
    projectRoot (1,:) char
    outputDir (1,:) char
end

scenarioPath=fullfile(projectRoot,'simulation','scenarios', ...
    'hfi_dynamic_plant_v1.json');
doc=jsondecode(fileread(scenarioPath));
assert(strcmp(doc.contract,'fluxrt-hfi-dynamic-plant'));
assert(doc.version==1);
if ~exist(outputDir,'dir'), mkdir(outputDir); end

cases=doc.cases; n=numel(cases); case_id=strings(n,1); values=zeros(n,5);
for k=1:n
    case_id(k)=string(cases(k).id);
    values(k,:)=run_case(doc,cases(k));
    if logical(values(k,5))~=logical(cases(k).expected_valid)
        error('case %s validity mismatch',cases(k).id);
    end
end
T=table(case_id,values(:,1),values(:,2),values(:,3),values(:,4),values(:,5), ...
    'VariableNames',{'case_id','rms_angle_error_rad','maximum_angle_error_rad', ...
    'mean_response_a','peak_current_a','valid'});
writetable(T,fullfile(outputDir,'matlab-hfi-dynamic-metrics.csv'));

revision=getenv('FLUXRT_WORKSPACE_REVISION');
if isempty(revision), revision='UNRECORDED'; end
fid=fopen(fullfile(outputDir,'matlab-hfi-dynamic-d0.txt'),'w');
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'contract=fluxrt-hfi-dynamic-plant\nversion=1\nengine=matlab\n');
fprintf(fid,'workspace_revision=%s\ncase_count=%d\nresult=PASS\n',revision,n);
fprintf('FLUXRT_MATLAB_HFI_DYNAMIC_CONTRACT_PASS cases=%d\n',n);
end

function row=run_case(doc,c)
frequency=double(doc.control_frequency_hz); dt=1/frequency; p=doc.injection;
phaseOffset=calibrate_phase(doc,c);
trackingPhase=first_order_low_pass_phase(double(p.demod_alpha), ...
    2*double(c.electrical_speed_rad_s)*dt);
observer=observer_init(phaseOffset+trackingPhase);
plant=[0;0]; separator=[0;0]; delay=zeros(2,8); writeIndex=1;
theta=mod(double(c.initial_theta_rad),2*pi);
squared=0; maximumError=0; responseSum=0; peakCurrent=0; metricSamples=0;
for sample=0:double(doc.samples)-1
    currentBefore=measured_current(plant,c,theta,0,sample);
    applied=apply_inverter_loss(observer.voltage,currentBefore, ...
        double(c.dead_time_s),double(doc.dc_bus_voltage_v), ...
        double(doc.pwm_frequency_hz));
    plant=plant_step(plant,applied,theta,double(c.electrical_speed_rad_s),c,dt);
    theta=mod(theta+double(c.electrical_speed_rad_s)*dt,2*pi);
    measured=measured_current(plant,c,theta,double(c.noise_a),sample);
    peakCurrent=max(peakCurrent,hypot(measured(1),measured(2)));
    [delayed,delay,writeIndex]=delay_push(delay,writeIndex,measured, ...
        double(c.delay_samples));
    [highFrequency,separator]=separate_current(separator,delayed, ...
        double(p.separator_alpha));
    observer=observer_step(observer,highFrequency,p,dt);
    if sample>=double(doc.settling_samples)
        error=mod_pi_error(observer.theta,mod(theta,pi));
        squared=squared+error^2; maximumError=max(maximumError,error);
        responseSum=responseSum+observer.response; metricSamples=metricSamples+1;
    end
end
rmsError=sqrt(squared/metricSamples); meanResponse=responseSum/metricSamples;
relativeSaliency=abs(double(c.lq_h)-double(c.ld_h))/max(double(c.ld_h),double(c.lq_h));
valid=relativeSaliency>=double(p.minimum_relative_saliency) && ...
    meanResponse>=double(p.minimum_response_a) && ...
    rmsError<=double(p.maximum_rms_error_rad) && ...
    maximumError<=double(p.maximum_peak_error_rad);
row=[rmsError maximumError meanResponse peakCurrent double(valid)];
end

function phase=calibrate_phase(doc,c)
dt=1/double(doc.control_frequency_hz); p=doc.injection;
cal=c; cal.initial_theta_rad=0; cal.electrical_speed_rad_s=0;
cal.base_id_a=0; cal.base_iq_a=0; cal.post_step_id_a=0;
cal.post_step_iq_a=0; cal.base_current_step_sample=doc.samples; cal.noise_a=0;
observer=observer_init(0); plant=[0;0]; separator=[0;0];
delay=zeros(2,8); writeIndex=1;
for sample=0:double(doc.settling_samples)-1
    currentBefore=measured_current(plant,cal,0,0,sample);
    applied=apply_inverter_loss(observer.voltage,currentBefore, ...
        double(cal.dead_time_s),double(doc.dc_bus_voltage_v), ...
        double(doc.pwm_frequency_hz));
    plant=plant_step(plant,applied,0,0,cal,dt);
    measured=measured_current(plant,cal,0,0,sample);
    [delayed,delay,writeIndex]=delay_push(delay,writeIndex,measured, ...
        double(cal.delay_samples));
    [highFrequency,separator]=separate_current(separator,delayed, ...
        double(p.separator_alpha));
    observer=observer_step(observer,highFrequency,p,dt);
end
phase=atan2(observer.negative(2),observer.negative(1));
end

function next=plant_step(currentDq,voltageAb,theta,speed,c,dt)
voltageDq=park_transform(voltageAb,theta);
did=(voltageDq(1)-double(c.resistance_ohm)*currentDq(1)+ ...
    speed*double(c.lq_h)*currentDq(2))/double(c.ld_h);
diq=(voltageDq(2)-double(c.resistance_ohm)*currentDq(2)- ...
    speed*double(c.ld_h)*currentDq(1))/double(c.lq_h);
next=currentDq+[did;diq]*dt;
end

function current=measured_current(hfDq,c,theta,noise,sample)
if sample>=double(c.base_current_step_sample)
    base=[double(c.post_step_id_a);double(c.post_step_iq_a)];
else
    base=[double(c.base_id_a);double(c.base_iq_a)];
end
total=base+hfDq;
current=inverse_park_transform(total,theta);
current(1)=current(1)+noise*sin(double(sample)*0.731);
current(2)=current(2)+noise*sin(double(sample)*0.527+0.4);
end

function applied=apply_inverter_loss(commanded,current,deadTime,vbus,pwmFrequency)
if deadTime<=0, applied=commanded; return; end
abc=inverse_clarke_transform(current); magnitude=2*deadTime*pwmFrequency*vbus;
polarity=max(-1,min(1,abc/0.002)); loss=magnitude*polarity;
lossAb=clarke_transform(loss);
applied=commanded-lossAb;
end

function [output,line,nextIndex]=delay_push(line,index,value,delaySamples)
line(:,index)=value;
readIndex=mod((index-1)+8-delaySamples,8)+1;
output=line(:,readIndex); nextIndex=mod(index,8)+1;
end

function [high,low]=separate_current(low,input,alpha)
low=low+alpha*(input-low); high=input-low;
end

function state=observer_init(phaseOffset)
state.carrier=0; state.voltage=[0;0]; state.negative=[0;0];
state.theta=0; state.response=0; state.phaseOffset=phaseOffset;
end

function state=observer_step(state,current,p,dt)
cc=cos(state.carrier); sc=sin(state.carrier);
demod=[current(1)*cc-current(2)*sc;current(1)*sc+current(2)*cc];
alpha=double(p.demod_alpha);
state.negative=state.negative+alpha*(demod-state.negative);
state.response=hypot(state.negative(1),state.negative(2));
twiceTheta=mod(atan2(state.negative(2),state.negative(1))-state.phaseOffset,2*pi);
state.theta=0.5*twiceTheta;
state.carrier=mod(state.carrier+2*pi*double(p.frequency_hz)*dt,2*pi);
state.voltage=double(p.amplitude_v)*[cos(state.carrier);sin(state.carrier)];
end

function phase=first_order_low_pass_phase(alpha,inputStep)
oneMinus=1-alpha;
phase=atan2(-oneMinus*sin(inputStep),1-oneMinus*cos(inputStep));
end

function error=mod_pi_error(estimate,truth)
error=abs(mod(estimate-truth+0.5*pi,pi)-0.5*pi);
end

function dq=park_transform(ab,theta)
c=cos(theta); s=sin(theta); dq=[ab(1)*c+ab(2)*s;-ab(1)*s+ab(2)*c];
end

function ab=inverse_park_transform(dq,theta)
c=cos(theta); s=sin(theta); ab=[dq(1)*c-dq(2)*s;dq(1)*s+dq(2)*c];
end

function abc=inverse_clarke_transform(ab)
abc=[ab(1);-0.5*ab(1)+sqrt(3)/2*ab(2);-0.5*ab(1)-sqrt(3)/2*ab(2)];
end

function ab=clarke_transform(abc)
ab=[abc(1);(abc(1)+2*abc(2))/sqrt(3)];
end
