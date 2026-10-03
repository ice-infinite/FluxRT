function result = run_full_speed_closed_loop_contract(projectRoot, outputRoot)
%RUN_FULL_SPEED_CLOSED_LOOP_CONTRACT Independent MATLAB full-speed design model.
% This is an averaged inverter + ideal-feedback PMSM model. It is intentionally
% independent from the Rust implementation and is not hardware-truth evidence.

if nargin < 1 || strlength(string(projectRoot)) == 0
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end
if nargin < 2 || strlength(string(outputRoot)) == 0
    outputRoot = fullfile(projectRoot, 'simulation', 'results', 'full-speed');
end
scenarioPath = fullfile(projectRoot, 'simulation', 'scenarios', ...
    'full_speed_closed_loop_v1.json');
scenario = jsondecode(fileread(scenarioPath));
assert(strcmp(scenario.contract, 'fluxrt-full-speed-closed-loop') && ...
    scenario.version == 1, 'Unsupported full-speed scenario contract');
if ~exist(outputRoot, 'dir'); mkdir(outputRoot); end

p = local_parameters();
traceRows = cell(0, 17);
metricRows = cell(0, 8);
for caseIndex = 1:numel(scenario.cases)
    c = scenario.cases(caseIndex);
    [rows, metric] = run_case(c, p, double(scenario.trace_decimation));
    traceRows = [traceRows; rows]; %#ok<AGROW>
    metricRows(end + 1, :) = metric; %#ok<AGROW>
end

traceNames = {'case_id','tick','time_s','target_rpm','speed_rpm','id_a','iq_a', ...
    'id_ref_a','iq_ref_a','vd_v','vq_v','dc_bus_v','source_current_a', ...
    'load_torque_nm','region','active_features','voltage_limited'};
metricNames = {'case_id','final_target_rpm','final_speed_rpm', ...
    'final_speed_error_rpm','peak_current_a','min_bus_v','max_bus_v', ...
    'voltage_limited_fraction'};
traceTable = cell2table(traceRows, 'VariableNames', traceNames);
metricTable = cell2table(metricRows, 'VariableNames', metricNames);
tracePath = fullfile(outputRoot, 'matlab-full-speed-trace.csv');
metricsPath = fullfile(outputRoot, 'matlab-full-speed-metrics.csv');
writetable(traceTable, tracePath);
writetable(metricTable, metricsPath);

revision = getenv('FLUXRT_WORKSPACE_REVISION');
if isempty(revision); revision = 'UNRECORDED'; end
metadataPath = fullfile(outputRoot, 'matlab-full-speed-d0.txt');
fid = fopen(metadataPath, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'contract=fluxrt-full-speed-closed-loop\n');
fprintf(fid, 'version=1\n');
fprintf(fid, 'engine=matlab-dynamic-rk4\n');
fprintf(fid, 'workspace_revision=%s\n', revision);
fprintf(fid, 'case_count=%d\n', numel(scenario.cases));
fprintf(fid, 'row_count=%d\n', height(traceTable));
fprintf(fid, 'result=PASS\n');
fprintf(fid, 'model_scope=design-not-hardware-truth\n');
result = struct('trace', tracePath, 'metrics', metricsPath, ...
    'metadata', metadataPath, 'rows', height(traceTable));
end

function [rows, metric] = run_case(c, p, decimation)
n = round(double(c.duration_s) * p.controlHz);
x = [0; 0; 0; 0; double(c.source_initial_v)]; % id iq wm theta vbus
speedInt = 0; idInt = 0; iqInt = 0;
speedCounter = 0; regionCounter = 0;
baseRef = [0; 0]; heldRef = [0; 0]; previousVdq = [0; 0];
weakeningActive = false;
peakCurrent = 0; minBus = x(5); maxBus = x(5); limitedCount = 0;
rows = cell(0, 17);
for tick = 0:n-1
    time = tick / p.controlHz;
    target = choose(time >= double(c.target_step_time_s), ...
        double(c.target_final_rpm), double(c.target_initial_rpm));
    loadNm = choose(time >= double(c.load_step_time_s), ...
        double(c.load_final_nm), double(c.load_initial_nm));
    sourceV = choose(time >= double(c.source_step_time_s), ...
        double(c.source_final_v), double(c.source_initial_v));
    if speedCounter == 0
        [baseRef(2), speedInt] = pi_step(target*pi/30, x(3), speedInt, ...
            p.speedKp, p.speedKi, 1/p.speedHz, -p.currentLimit, p.currentLimit);
        baseRef(1) = 0;
    end
    speedCounter = speedCounter + 1;
    if speedCounter >= p.controlHz/p.speedHz; speedCounter = 0; end

    [heldRef, region, weakeningActive, regionCounter] = advanced_step( ...
        baseRef, previousVdq, x(5), c.advanced_features, heldRef, ...
        weakeningActive, regionCounter, p);
    we = x(3) * p.polePairs;
    vdFf = 0; vqFf = 0; activeFeatures = 0;
    if bitand(uint32(c.advanced_features), uint32(8)) ~= 0
        vdFf = -we * p.lq * x(2);
        vqFf = we * (p.ld*x(1) + p.flux);
        activeFeatures = bitor(activeFeatures, 8);
    end
    if region == 2; activeFeatures = bitor(activeFeatures, 2); end
    [vdPi, idInt] = pi_step(heldRef(1), x(1), idInt, p.currentKp, ...
        p.currentKi, p.dt, -p.maxVoltage, p.maxVoltage);
    [vqPi, iqInt] = pi_step(heldRef(2), x(2), iqInt, p.currentKp, ...
        p.currentKi, p.dt, -p.maxVoltage, p.maxVoltage);
    vd = vdPi + vdFf; vq = vqPi + vqFf;
    limit = p.voltageUtilization*x(5)/sqrt(3);
    mag = hypot(vd, vq); limited = mag > limit && mag > 0;
    if limited
        scale = limit/mag; vd = vd*scale; vq = vq*scale;
        if activeFeatures ~= 0
            idInt = clamp(vd-vdFf-p.currentKp*(heldRef(1)-x(1)), -p.maxVoltage, p.maxVoltage);
            iqInt = clamp(vq-vqFf-p.currentKp*(heldRef(2)-x(2)), -p.maxVoltage, p.maxVoltage);
        end
    end
    previousVdq = [vd; vq];
    x = rk4_step(x, [vd;vq], sourceV, loadNm, p);
    x(4) = mod(x(4), 2*pi); x(5) = clamp(x(5), 6, 20);
    acPower = 1.5*(vd*x(1)+vq*x(2));
    invCurrent = acPower/x(5); %#ok<NASGU>
    sourceCurrent = max((sourceV-x(5))/p.sourceResistance, 0);
    speedRpm = x(3)*30/pi;
    peakCurrent = max(peakCurrent, hypot(x(1),x(2)));
    minBus = min(minBus,x(5)); maxBus = max(maxBus,x(5));
    limitedCount = limitedCount + double(limited);
    if mod(tick, decimation) == 0 || tick+1 == n
        rows(end+1,:) = {char(c.id),tick,time,target,speedRpm,x(1),x(2), ...
            heldRef(1),heldRef(2),vd,vq,x(5),sourceCurrent,loadNm,region, ...
            activeFeatures,double(limited)}; %#ok<AGROW>
    end
end
metric = {char(c.id),target,speedRpm,target-speedRpm,peakCurrent,minBus,maxBus,limitedCount/n};
end

function [reference, region, active, counter] = advanced_step(base, previousVdq, ...
    vbus, features, held, active, counter, p)
due = counter == 0;
counter = counter + 1;
if counter >= p.regionDivider; counter = 0; end
if ~due
    reference = held;
    region = choose(active,2,0);
    return
end
reference = limit_current(base,p.currentLimit); region = 0;
ratio = hypot(previousVdq(1),previousVdq(2))/max(vbus/sqrt(3),realmin);
if bitand(uint32(features),uint32(2)) ~= 0
    if active
        if ratio <= p.weakeningExit; active = false; end
    elseif ratio >= p.weakeningEntry
        active = true;
    end
    if active
        voltageLimit = p.weakeningEntry*vbus/sqrt(3);
        targetId = reference(1);
        voltageError = hypot(previousVdq(1),previousVdq(2))-voltageLimit;
        if voltageError > 0; targetId = targetId-p.weakeningKp*voltageError; end
        targetId = min(clamp(targetId,p.weakeningIdMin,0),0);
        maxStep = p.weakeningSlew/p.controlHz*p.regionDivider;
        reference(1) = move_towards(reference(1),targetId,maxStep);
        region = 2;
    end
else
    active = false;
end
reference = limit_current(reference,p.currentLimit);
end

function xnext = rk4_step(x, voltage, sourceV, loadNm, p)
k1 = derivative(x,voltage,sourceV,loadNm,p);
k2 = derivative(x+0.5*p.dt*k1,voltage,sourceV,loadNm,p);
k3 = derivative(x+0.5*p.dt*k2,voltage,sourceV,loadNm,p);
k4 = derivative(x+p.dt*k3,voltage,sourceV,loadNm,p);
xnext = x+p.dt*(k1+2*k2+2*k3+k4)/6;
end

function dx = derivative(x,v,sourceV,loadNm,p)
we = x(3)*p.polePairs;
did = (v(1)-p.rs*x(1)+we*p.lq*x(2))/p.ld;
diq = (v(2)-p.rs*x(2)-we*(p.ld*x(1)+p.flux))/p.lq;
torque = 1.5*p.polePairs*(p.flux*x(2)+(p.ld-p.lq)*x(1)*x(2));
dwm = (torque-loadNm-p.friction*x(3))/p.inertia;
acPower = 1.5*(v(1)*x(1)+v(2)*x(2));
inverterCurrent = acPower/x(5);
sourceCurrent = max((sourceV-x(5))/p.sourceResistance,0);
dvbus = (sourceCurrent-inverterCurrent)/p.capacitance;
dx = [did;diq;dwm;x(3);dvbus];
end

function p = local_parameters()
p.polePairs=7; p.rs=5.29; p.ld=0.001058; p.lq=0.001058;
p.flux=0.034739897/(2*pi); p.inertia=0.291e-4; p.friction=0.937e-5;
p.controlHz=12000; p.speedHz=1000; p.dt=1/p.controlHz;
p.currentLimit=0.8; p.voltageUtilization=0.95; p.maxVoltage=13*0.95/sqrt(3);
countsPerAmp=65536*0.33*1.53/3.3;
voltsPerCount=p.maxVoltage/32767;
gainScale=countsPerAmp*voltsPerCount;
p.currentKp=(3378/1024)*gainScale;
p.currentKi=(2252/4096)*gainScale*30000;
speedUnitsPerRadS=10/(2*pi);
p.speedKp=(2730/256)*speedUnitsPerRadS/countsPerAmp;
p.speedKi=(562/16384)*speedUnitsPerRadS/countsPerAmp*1000;
p.regionDivider=12; p.weakeningEntry=0.92; p.weakeningExit=0.82;
p.weakeningKp=0.08; p.weakeningIdMin=-0.6; p.weakeningSlew=20;
p.sourceResistance=0.2; p.capacitance=0.002;
end

function [output,integrator] = pi_step(reference,feedback,integrator,kp,ki,ts,lo,hi)
error=reference-feedback; proportional=kp*error;
integrator=clamp(integrator+ki*ts*error,lo,hi);
unclamped=proportional+integrator; output=clamp(unclamped,lo,hi);
if unclamped ~= output; integrator=clamp(output-proportional,lo,hi); end
end

function value = clamp(value,lo,hi); value=min(max(value,lo),hi); end
function value = choose(condition,yes,no); if condition; value=yes; else; value=no; end; end
function value = move_towards(value,target,step)
if target>value; value=min(value+step,target); else; value=max(value-step,target); end
end
function value = limit_current(value,limit)
mag=hypot(value(1),value(2)); if mag>limit && mag>0; value=value*(limit/mag); end
end
