function run_sensorless_full_speed_chain_contract(projectRoot, outputDir)
%RUN_SENSORLESS_FULL_SPEED_CHAIN_CONTRACT Independent full-speed angle chain.
% Offline S3 only. This model does not call Rust or authorize target injection.

arguments
    projectRoot (1,:) char
    outputDir (1,:) char
end
doc=jsondecode(fileread(fullfile(projectRoot,'simulation','scenarios', ...
    'sensorless_full_speed_chain_v1.json')));
assert(strcmp(doc.contract,'fluxrt-sensorless-full-speed-chain') && doc.version==1);
if ~exist(outputDir,'dir'), mkdir(outputDir); end

traceRows=cell(0,15); metricRows=cell(0,9); config=chain_config(doc);
for k=1:numel(doc.cases)
    [rows,metric]=run_case(doc,doc.cases(k),config);
    traceRows=[traceRows;rows]; %#ok<AGROW>
    metricRows(end+1,:)=metric; %#ok<AGROW>
end
traceNames={'case_id','tick','time_s','true_angle_rad','electrical_speed_rad_s', ...
    'estimated_angle_rad','angle_error_rad','source','reliable','fallback_required', ...
    'stage','hfi_valid','bemf_valid','injection_alpha_v','injection_beta_v'};
metricNames={'case_id','rms_angle_error_rad','maximum_angle_error_rad', ...
    'maximum_angle_step_rad','unreliable_fraction','source_mask','transition_mask','fallback_seen', ...
    'final_reliable'};
writetable(cell2table(traceRows,'VariableNames',traceNames), ...
    fullfile(outputDir,'matlab-sensorless-full-speed-trace.csv'));
writetable(cell2table(metricRows,'VariableNames',metricNames), ...
    fullfile(outputDir,'matlab-sensorless-full-speed-metrics.csv'));
revision=getenv('FLUXRT_WORKSPACE_REVISION'); if isempty(revision), revision='UNRECORDED'; end
fid=fopen(fullfile(outputDir,'matlab-sensorless-full-speed-d0.txt'),'w');
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'contract=fluxrt-sensorless-full-speed-chain\nversion=1\nengine=matlab-independent-chain\n');
fprintf(fid,'workspace_revision=%s\ncase_count=%d\nrow_count=%d\n', ...
    revision,numel(doc.cases),size(traceRows,1));
fprintf(fid,'result=PASS\nmodel_scope=design-not-hardware-truth\n');
fprintf('FLUXRT_MATLAB_SENSORLESS_FULL_SPEED_PASS cases=%d rows=%d\n', ...
    numel(doc.cases),size(traceRows,1));
end

function [rows,metric]=run_case(doc,c,config)
frequency=double(doc.control_frequency_hz); dt=1/frequency;
ticks=round(double(c.duration_s)*frequency); decimation=double(doc.trace_decimation);
chain=chain_init(); previous=output_init(); theta=mod(double(c.initial_theta_rad),2*pi);
squared=0; maximumError=0; maximumStep=0; reliableCount=0; unreliableCount=0;
sourceMask=uint32(0); transitionMask=uint32(0); lastReliableSource=uint32(0);
fallbackSeen=false; haveLast=false; lastAngle=0;
rows=cell(0,15);
for tick=0:ticks-1
    time=tick*dt; speed=speed_profile(c,time);
    hfiPermitted=~inside(time,double(c.hfi_dropout_start_s),double(c.hfi_dropout_end_s));
    bemfDropout=inside(time,double(c.bemf_dropout_start_s),double(c.bemf_dropout_end_s));
    high=synthetic_hfi_current(previous,theta,double(doc.hfi_response_a), ...
        double(doc.noise_a),tick);
    measured=synthetic_polarity_current(previous,theta);
    bemfValid=abs(speed)+1e-9>=double(doc.minimum_bemf_speed_rad_s) && ~bemfDropout;
    bemfAngle=mod(theta+0.005*sin(double(tick)*0.017),2*pi);
    input=struct('injectionPermitted',hfiPermitted,'high',high,'measured',measured, ...
        'bemfAngle',bemfAngle,'bemfValid',bemfValid,'speed',speed);
    [chain,output]=chain_step(chain,config,input);
    if output.fusion.reliable
        angleError=abs(wrap_pi(output.fusion.angle-theta));
        squared=squared+angleError^2; maximumError=max(maximumError,angleError);
        reliableCount=reliableCount+1; sourceMask=bitor(sourceMask,source_bit(output.fusion.source));
        currentSource=uint32(output.fusion.source);
        if lastReliableSource~=0 && lastReliableSource~=currentSource
            transitionMask=bitor(transitionMask,transition_bit(lastReliableSource,currentSource));
        end
        lastReliableSource=currentSource;
        if haveLast, maximumStep=max(maximumStep,abs(wrap_pi(output.fusion.angle-lastAngle))); end
        lastAngle=output.fusion.angle; haveLast=true;
    else
        angleError=0; unreliableCount=unreliableCount+1; haveLast=false;
        lastReliableSource=uint32(0);
    end
    fallbackSeen=fallbackSeen || output.fusion.fallback;
    if mod(tick,decimation)==0 || tick+1==ticks
        rows(end+1,:)={char(c.id),tick,time,theta,speed,output.fusion.angle,angleError, ...
            output.fusion.source,double(output.fusion.reliable),double(output.fusion.fallback), ...
            output.stage,double(output.hfiValid),double(bemfValid),output.voltage(1),output.voltage(2)}; %#ok<AGROW>
    end
    previous=output; theta=mod(theta+speed*dt,2*pi);
end
assert(bitand(sourceMask,uint32(c.expected_source_mask))==uint32(c.expected_source_mask));
assert(bitand(transitionMask,uint32(c.expected_transition_mask))==uint32(c.expected_transition_mask));
assert(fallbackSeen==logical(c.expected_fallback));
assert(sqrt(squared/reliableCount)<=double(c.maximum_rms_error_rad));
assert(maximumStep<=double(c.maximum_angle_step_rad));
assert(previous.fusion.reliable);
metric={char(c.id),sqrt(squared/reliableCount),maximumError,maximumStep, ...
    unreliableCount/ticks,double(sourceMask),double(transitionMask),double(fallbackSeen),double(previous.fusion.reliable)};
end

function c=chain_config(doc)
c.dt=1/double(doc.control_frequency_hz); c.amplitude=1; c.frequency=1000;
c.demodAlpha=0.2; c.phaseOffset=0; c.minimumResponse=0.02;
c.maximumHfiSpeed=180; c.axisStableRequired=24;
c.pulseVoltage=0.5; c.pulseTicks=2; c.minimumOffTicks=2;
c.settleTimeoutTicks=24; c.maximumCurrent=0.5; c.maximumResidual=0.05;
c.minimumResponseDelta=0.02; c.maximumPulsePairs=3;
c.blendEnter=81.3; c.hfiReenter=61.7; c.bemfEnter=141.1; c.bemfExit=111.3;
c.maximumDisagreement=0.45; c.weightSlew=0.02; c.stableRequired=24;
c.invalidTimeout=120;
end

function s=chain_init()
s.stage=0; s.failure=0; s.hfi=hfi_init(); s.polarity=polarity_init();
s.fusion=fusion_init(); s.axisStable=0; s.candidateAxis=0;
s.polarityStarted=false; s.polarityResolved=false; s.addPi=false; s.trackingSamples=0;
end

function o=output_init()
o.voltage=[0;0]; o.stage=0; o.hfiAngle=0; o.hfiResponse=0; o.hfiValid=false;
o.polarityResolved=false; o.addPi=false; o.fusion=fusion_output();
end

function [s,o]=chain_step(s,c,in)
speedAbs=abs(in.speed); hfiAllowed=speedAbs<=c.maximumHfiSpeed;
wasTracking=s.stage==2; voltage=[0;0]; hfiValid=false;
switch s.stage
    case 0
        if in.injectionPermitted && hfiAllowed
            [s.hfi,voltage]=hfi_step(s.hfi,c,in.high);
            responseGood=s.hfi.response>=c.minimumResponse;
            s.axisStable=stable_count(s.axisStable,responseGood);
            if s.axisStable>=c.axisStableRequired
                s.candidateAxis=s.hfi.theta; s.stage=1; s.polarityStarted=false; voltage=[0;0];
            end
        else
            s.axisStable=0;
        end
    case 1
        pin=struct('start',~s.polarityStarted,'permitted',in.injectionPermitted && hfiAllowed, ...
            'axis',s.candidateAxis,'current',in.measured);
        [s.polarity,pout]=polarity_step(s.polarity,c,pin); s.polarityStarted=true;
        voltage=pout.voltage;
        if pout.state==5
            s.polarityResolved=true; s.addPi=pout.addPi; s.stage=2;
            s.axisStable=0; s.trackingSamples=0; s.hfi=hfi_init(); voltage=[0;0];
        elseif pout.state==6
            s.stage=3; voltage=[0;0];
        end
    case 2
        s.trackingSamples=s.trackingSamples+1;
        if in.injectionPermitted && hfiAllowed
            [s.hfi,voltage]=hfi_step(s.hfi,c,in.high);
            responseGood=s.hfi.response>=c.minimumResponse;
            s.axisStable=stable_count(s.axisStable,responseGood);
            hfiValid=s.axisStable>=c.axisStableRequired;
        else
            s.axisStable=0;
        end
end
fin=struct('hfiAngle',s.hfi.theta,'hfiValid',hfiValid, ...
    'polarityResolved',s.polarityResolved,'addPi',s.addPi, ...
    'bemfAngle',in.bemfAngle,'bemfValid',in.bemfValid,'speedAbs',speedAbs);
[s.fusion,fout]=fusion_step(s.fusion,c,fin);
o=output_init(); o.voltage=voltage; o.stage=s.stage; o.hfiAngle=s.hfi.theta;
o.hfiResponse=s.hfi.response; o.hfiValid=hfiValid; o.polarityResolved=s.polarityResolved;
o.addPi=s.addPi; o.fusion=fout;
expired=s.trackingSamples>=c.invalidTimeout+c.axisStableRequired;
if fout.fallback && ~in.bemfValid && ~hfiValid && wasTracking && ...
        (~in.injectionPermitted || expired)
    s.hfi=hfi_init(); s.polarity=polarity_init(); s.fusion=fusion_init();
    s.stage=0; s.axisStable=0; s.polarityStarted=false;
    s.polarityResolved=false; s.addPi=false; s.trackingSamples=0;
end
end

function h=hfi_init()
h.carrier=0; h.voltage=[0;0]; h.negative=[0;0]; h.theta=0; h.response=0;
end

function [h,voltage]=hfi_step(h,c,current)
cc=cos(h.carrier); sc=sin(h.carrier);
demod=[current(1)*cc-current(2)*sc;current(1)*sc+current(2)*cc];
h.negative=h.negative+c.demodAlpha*(demod-h.negative);
h.response=hypot(h.negative(1),h.negative(2));
h.theta=0.5*mod(atan2(h.negative(2),h.negative(1))-c.phaseOffset,2*pi);
h.carrier=mod(h.carrier+2*pi*c.frequency*c.dt,2*pi);
h.voltage=c.amplitude*[cos(h.carrier);sin(h.carrier)]; voltage=h.voltage;
end

function p=polarity_init()
p.state=0; p.failure=0; p.axisCos=0; p.axisSin=0; p.phaseTicks=0;
p.pairs=0; p.positivePeak=0; p.negativePeak=0; p.delta=0; p.addPi=false;
end

function [p,o]=polarity_step(p,c,in)
if hypot(in.current(1),in.current(2))>c.maximumCurrent
    p.state=6; p.failure=4; o=polarity_output(p,[0;0]); return
end
if p.state~=0 && p.state~=5 && p.state~=6 && ~in.permitted
    p.state=6; p.failure=2; o=polarity_output(p,[0;0]); return
end
switch p.state
    case 0
        if ~in.start, o=polarity_output(p,[0;0]); return; end
        if ~in.permitted, p.state=6; p.failure=2; o=polarity_output(p,[0;0]); return; end
        if hypot(in.current(1),in.current(2))>c.maximumResidual
            p.state=6; p.failure=5; o=polarity_output(p,[0;0]); return
        end
        p.axisCos=cos(in.axis); p.axisSin=sin(in.axis); p.state=1; p.phaseTicks=0;
        [p,o]=positive_pulse(p,c,in.current);
    case 1
        [p,o]=positive_pulse(p,c,in.current);
    case 2
        p.phaseTicks=p.phaseTicks+1;
        if p.phaseTicks>=c.minimumOffTicks && hypot(in.current(1),in.current(2))<=c.maximumResidual
            p.state=3; p.phaseTicks=0;
        elseif p.phaseTicks>=c.settleTimeoutTicks
            p.state=6; p.failure=5;
        end
        o=polarity_output(p,[0;0]);
    case 3
        [p,o]=negative_pulse(p,c,in.current);
    case 4
        p.phaseTicks=p.phaseTicks+1;
        if p.phaseTicks>=c.minimumOffTicks && hypot(in.current(1),in.current(2))<=c.maximumResidual
            [p,o]=evaluate_pair(p,c); return
        elseif p.phaseTicks>=c.settleTimeoutTicks
            p.state=6; p.failure=5;
        end
        o=polarity_output(p,[0;0]);
    otherwise
        o=polarity_output(p,[0;0]);
end
end

function [p,o]=positive_pulse(p,c,current)
projection=current(1)*p.axisCos+current(2)*p.axisSin;
p.positivePeak=max(p.positivePeak,max(projection,0)); p.phaseTicks=p.phaseTicks+1;
voltage=c.pulseVoltage*[p.axisCos;p.axisSin];
if p.phaseTicks>=c.pulseTicks, p.state=2; p.phaseTicks=0; end
o=polarity_output(p,voltage);
end

function [p,o]=negative_pulse(p,c,current)
projection=current(1)*p.axisCos+current(2)*p.axisSin;
p.negativePeak=max(p.negativePeak,max(-projection,0)); p.phaseTicks=p.phaseTicks+1;
voltage=-c.pulseVoltage*[p.axisCos;p.axisSin];
if p.phaseTicks>=c.pulseTicks, p.state=4; p.phaseTicks=0; end
o=polarity_output(p,voltage);
end

function [p,o]=evaluate_pair(p,c)
p.pairs=p.pairs+1; p.delta=p.positivePeak-p.negativePeak;
if abs(p.delta)>=c.minimumResponseDelta
    p.addPi=p.delta<0; p.state=5; o=polarity_output(p,[0;0]); return
end
if p.pairs>=c.maximumPulsePairs
    p.state=6; p.failure=6; o=polarity_output(p,[0;0]); return
end
p.positivePeak=0; p.negativePeak=0; p.phaseTicks=0; p.state=1;
o=polarity_output(p,[0;0]);
end

function o=polarity_output(p,voltage)
o=struct('voltage',voltage,'state',p.state,'addPi',p.addPi);
end

function f=fusion_init()
f.source=0; f.weight=0; f.hfiStable=0; f.bemfStable=0; f.invalid=0;
f.lastAngle=0; f.lastHfi=0; f.branchInitialized=false;
end

function o=fusion_output()
o=struct('angle',0,'hfiAngle',0,'bemfAngle',0,'weight',0, ...
    'disagreement',0,'source',0,'reliable',false,'fallback',false);
end

function [f,o]=fusion_step(f,c,in)
hfiGood=in.hfiValid && in.polarityResolved && isfinite(in.hfiAngle);
bemfGood=in.bemfValid && isfinite(in.bemfAngle);
f.hfiStable=stable_count(f.hfiStable,hfiGood); f.bemfStable=stable_count(f.bemfStable,bemfGood);
hfiReady=f.hfiStable>=c.stableRequired; bemfReady=f.bemfStable>=c.stableRequired;
if bemfGood, bemfAngle=mod(in.bemfAngle,2*pi); else, bemfAngle=f.lastAngle; end
if hfiGood
    wrapped=mod(in.hfiAngle,2*pi); if wrapped>=pi, base=wrapped-pi; else, base=wrapped; end
    b0=base; b1=base+pi;
    % A merely-valid BEMF sample cannot flip the branch while HFI owns the
    % output.  BEMF can seed reacquisition only while it is already trusted.
    if f.source==3 && bemfGood, reference=bemfAngle;
    elseif f.branchInitialized, reference=f.lastHfi;
    else, reference=mod(base+double(in.addPi)*pi,2*pi); end
    if abs(wrap_pi(b0-reference))<=abs(wrap_pi(b1-reference)), selected=b0; else, selected=b1; end
    f.lastHfi=mod(selected,2*pi); f.branchInitialized=true; hfiAngle=f.lastHfi;
else
    hfiAngle=f.lastHfi;
end
disagreement=abs(wrap_pi(bemfAngle-hfiAngle)); agree=disagreement<=c.maximumDisagreement;
speed=in.speedAbs;
switch f.source
    case 0
        if hfiReady, f.source=1;
        elseif bemfReady && speed>=c.bemfEnter, f.source=3; end
    case 1
        if ~hfiGood
            if bemfReady && speed>=c.bemfExit, f.source=3; else, f.source=0; end
        elseif bemfReady && agree && speed>=c.blendEnter, f.source=2; end
    case 2
        if speed<=c.hfiReenter && hfiReady, f.source=1;
        elseif bemfReady && (speed>=c.bemfEnter || ~hfiGood), f.source=3;
        elseif ~bemfGood && hfiReady, f.source=1;
        elseif ~hfiGood && ~bemfGood, f.source=0; end
    case 3
        if ~bemfGood
            if hfiReady, f.source=1; else, f.source=0; end
        elseif speed<=c.bemfExit && hfiReady && agree, f.source=2; end
end
switch f.source
    case {0,1}, f.weight=0;
    case 3, f.weight=1;
    case 2
        target=min(max((speed-c.blendEnter)/(c.bemfEnter-c.blendEnter),0),1);
        f.weight=move_towards(f.weight,target,c.weightSlew);
end
reliable=(f.source==1 && hfiGood) || (f.source==2 && hfiGood && bemfGood && agree) || ...
    (f.source==3 && bemfGood);
if reliable, f.invalid=0; else, f.invalid=f.invalid+1; end
fallback=f.invalid>=c.invalidTimeout;
if reliable, f.lastAngle=mod(hfiAngle+f.weight*wrap_pi(bemfAngle-hfiAngle),2*pi); end
o=struct('angle',f.lastAngle,'hfiAngle',hfiAngle,'bemfAngle',bemfAngle, ...
    'weight',f.weight,'disagreement',disagreement,'source',f.source, ...
    'reliable',reliable,'fallback',fallback);
end

function current=synthetic_hfi_current(previous,theta,response,noise,tick)
if previous.stage==1 || hypot(previous.voltage(1),previous.voltage(2))<0.75
    current=[0;0]; return
end
carrier=atan2(previous.voltage(2),previous.voltage(1)); phase=2*theta-carrier;
current=[response*cos(phase)+noise*sin(double(tick)*0.731); ...
    response*sin(phase)+noise*sin(double(tick)*0.527+0.4)];
end

function current=synthetic_polarity_current(previous,theta)
if previous.stage~=1 || hypot(previous.voltage(1),previous.voltage(2))==0
    current=[0;0]; return
end
axis=previous.hfiAngle; projection=previous.voltage(1)*cos(axis)+previous.voltage(2)*sin(axis);
addPi=mod(theta,2*pi)>=pi; strongerPositive=~addPi;
if (projection>0)==strongerPositive, magnitude=0.12; else, magnitude=0.05; end
current=sign(projection)*magnitude*[cos(axis);sin(axis)];
end

function speed=speed_profile(c,time)
if time<double(c.ramp_up_start_s), speed=double(c.speed_initial_rad_s);
elseif time<double(c.ramp_up_end_s), speed=lerp(double(c.speed_initial_rad_s), ...
        double(c.speed_peak_rad_s),ratio(time,double(c.ramp_up_start_s),double(c.ramp_up_end_s)));
elseif time<double(c.ramp_down_start_s), speed=double(c.speed_peak_rad_s);
elseif time<double(c.ramp_down_end_s), speed=lerp(double(c.speed_peak_rad_s), ...
        double(c.speed_final_rad_s),ratio(time,double(c.ramp_down_start_s),double(c.ramp_down_end_s)));
else, speed=double(c.speed_final_rad_s); end
end

function value=source_bit(source)
if source==1, value=uint32(1); elseif source==2, value=uint32(2);
elseif source==3, value=uint32(4); else, value=uint32(0); end
end
function value=transition_bit(from,to)
if from==1 && to==2, value=uint32(1);
elseif from==2 && to==3, value=uint32(2);
elseif from==3 && to==2, value=uint32(4);
elseif from==2 && to==1, value=uint32(8);
elseif from==1 && to==3, value=uint32(16);
elseif from==3 && to==1, value=uint32(32);
else, value=uint32(0); end
end
function value=stable_count(value,valid); if valid, value=value+1; else, value=0; end; end
function yes=inside(value,startValue,endValue); yes=value>=startValue && value<endValue; end
function value=ratio(x,a,b); value=min(max((x-a)/(b-a),0),1); end
function value=lerp(a,b,t); value=a+(b-a)*t; end
function value=move_towards(value,target,step)
if abs(target-value)<=step, value=target; elseif target>value, value=value+step; else, value=value-step; end
end
function value=wrap_pi(value); value=mod(value+pi,2*pi)-pi; end
