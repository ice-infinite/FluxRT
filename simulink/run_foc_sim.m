function out = run_foc_sim(varargin)
%RUN_FOC_SIM Run the parameterized Simulink FOC model without rewriting code.
%
% out = run_foc_sim()                       % current firmware, open loop
% out = run_foc_sim('closedLoop',true,'duration',10)
% out = run_foc_sim('smoSlide',4.5,'emfAlpha',0.06)

ip = inputParser;
ip.addParameter('observerProfile','firmware',@(s)ischar(s)||isstring(s));
ip.addParameter('closedLoop',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('modelName','foc_bringup',@(s)ischar(s)||isstring(s));
ip.addParameter('targetRpm',582,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('busVoltage',12.3,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('duration',5.0,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('deadTimeNs',550,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('loadTorqueNm',0,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('enableDeadTime',true,@(x)islogical(x)||isnumeric(x));
ip.addParameter('enableDeadTimeCompensation',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('enableObserverDeadTimeCompensation',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('deadTimeCompensationGain',1.0,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('deadTimeCompensationCurrentBandA',0.005, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('smoSlide',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('smoBoundary',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('emfAlpha',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('pllKp',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('pllKi',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('minimumSpeedRpm',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('minimumBemfV',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('varianceRatio',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('consecutiveSamples',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('rebuild',false,@(x)islogical(x)||isnumeric(x));
ip.parse(varargin{:});
o = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(here);
p = init_foc_params( ...
    'observerProfile',o.observerProfile,'closedLoop',o.closedLoop, ...
    'targetRpm',o.targetRpm,'busVoltage',o.busVoltage, ...
    'duration',o.duration,'deadTimeNs',o.deadTimeNs, ...
    'loadTorqueNm',o.loadTorqueNm,'enableDeadTime',o.enableDeadTime, ...
    'enableDeadTimeCompensation',o.enableDeadTimeCompensation, ...
    'enableObserverDeadTimeCompensation',o.enableObserverDeadTimeCompensation, ...
    'deadTimeCompensationGain',o.deadTimeCompensationGain, ...
    'deadTimeCompensationCurrentBandA',o.deadTimeCompensationCurrentBandA);
if ~isempty(o.smoSlide), p.observer.kSlideV = o.smoSlide; end
if ~isempty(o.smoBoundary), p.observer.boundaryA = o.smoBoundary; end
if ~isempty(o.emfAlpha), p.observer.emfAlpha = o.emfAlpha; end
if ~isempty(o.pllKp), p.observer.pllKp = o.pllKp; end
if ~isempty(o.pllKi), p.observer.pllKi = o.pllKi; end
if ~isempty(o.minimumSpeedRpm), p.observer.reliabilityMinMeanRpm = o.minimumSpeedRpm; end
if ~isempty(o.minimumBemfV), p.observer.reliabilityMinEmfV = o.minimumBemfV; end
if ~isempty(o.varianceRatio), p.observer.reliabilityMaxVarRatio = o.varianceRatio; end
if ~isempty(o.consecutiveSamples), p.observer.reliabilityWindows = o.consecutiveSamples; end
p = foc_refresh_vectors(p);
assignin('base','p',p);
assignin('base','foc_iabc',[0;0;0]);

mdl = char(o.modelName);
slxPath = fullfile(here,[mdl '.slx']);
needBuild = logical(o.rebuild) || ~exist(slxPath,'file');
if ~needBuild
    sources = {'controller_core.m','pmsm_plant.m','make_foc_model.m'};
    modelStamp = dir(slxPath).datenum;
    for k = 1:numel(sources)
        if dir(fullfile(here,sources{k})).datenum > modelStamp
            needBuild = true;
        end
    end
end
if needBuild
    make_foc_model(mdl);
end
if ~bdIsLoaded(mdl)
    load_system(slxPath);
end

% The saved model initializes itself for direct GUI use. Scripted runs already
% supplied p, so disable the callback only in memory for this simulation.
savedInit = get_param(mdl,'InitFcn');
savedStop = get_param(mdl,'StopTime');
cleanup = onCleanup(@() restore_model(mdl,savedInit,savedStop));
set_param(mdl,'InitFcn','');
set_param(mdl,'StopTime',sprintf('%.17g',max(p.timing.duration-p.timing.ts,0)));
simOut = sim(mdl);
clear cleanup

out = struct();
out.mdl = mdl; out.p = p;
getv = @(n)simOut.get(n).signals.values;
out.time = getv('simTime');
out.trueSpeedRpm = getv('trueSpeedRpm');
out.obsSpeedRpm = getv('obsSpeedRpm');
out.trueAngle = getv('trueAngle');
out.obsAngle = getv('obsAngle');
out.forcedAngle = getv('forcedAngle');
out.controlAngle = getv('controlAngle');
out.speedReferenceRpm = getv('speedReferenceRpm');
out.state = getv('state');
out.reliable = getv('reliable');
out.faultFlags = getv('faultFlags');
out.idRef = getv('idRef'); out.iqRef = getv('iqRef');
out.idMeas = getv('idMeas'); out.iqMeas = getv('iqMeas');
out.vdCmd = getv('vdCmd'); out.vqCmd = getv('vqCmd');
out.torqueE = getv('torqueE');
out.idPlant = getv('idPlant'); out.iqPlant = getv('iqPlant');
out.phaseCurrents = getv('phaseCurrents');
out.angleErrorRad = mod(out.obsAngle-out.trueAngle+pi,2*pi)-pi;
out.speedErrorRpm = out.obsSpeedRpm-out.trueSpeedRpm;
end

function restore_model(mdl,initFcn,stopTime)
if bdIsLoaded(mdl)
    set_param(mdl,'InitFcn',initFcn,'StopTime',stopTime);
end
end
