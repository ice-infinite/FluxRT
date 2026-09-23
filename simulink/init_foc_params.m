function p = init_foc_params(varargin)
%INIT_FOC_PARAMS Parameters mirrored from the current C/Rust firmware.
%
% p = init_foc_params()
% p = init_foc_params('closedLoop',true,'duration',10)
%
% The default profile is the current firmware, not an experimental observer.
% controllerVector and plantVector are the only vectors consumed by Simulink;
% their named fields remain available for review, plots and validation.

ip = inputParser;
ip.addParameter('observerProfile','firmware',@(s)ischar(s)||isstring(s));
ip.addParameter('closedLoop',false,@(x)islogical(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('targetRpm',582,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('busVoltage',12.3,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('duration',5.0,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('deadTimeNs',550,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('loadTorqueNm',0,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('enableDeadTime',true,@(x)islogical(x)||isnumeric(x));
ip.addParameter('enableDeadTimeCompensation',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('enableObserverDeadTimeCompensation',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('deadTimeCompensationGain',1.0,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('deadTimeCompensationCurrentBandA',0.005, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.parse(varargin{:});
opt = ip.Results;

p = struct();
p.meta.createdBy = 'init_foc_params.m';
p.meta.matlabRelease = version('-release');
p.meta.profile = lower(char(opt.observerProfile));
p.meta.firmwareAbi = '0x00070000';
p.meta.configVersion = 4;
p.meta.simulationConfigVersion = 2;

%% Timing and test scenario
p.timing.timerClockHz = 170e6;
p.timing.pwmFrequencyHz = 12000;
p.timing.speedLoopFrequencyHz = 1000;
p.timing.speedLoopDivider = ...
    p.timing.pwmFrequencyHz/p.timing.speedLoopFrequencyHz;
p.timing.ts = 1/p.timing.pwmFrequencyHz;
p.timing.arr = p.timing.timerClockHz/(2*p.timing.pwmFrequencyHz);
p.timing.duration = opt.duration;
p.timing.targetRpm = opt.targetRpm;
p.loadTorqueNm = opt.loadTorqueNm;

%% Bus, motor and inverter
p.bus.nominalV = opt.busVoltage;
p.bus.minimumV = 7.0;
p.bus.maximumV = 18.0;
p.bus.partitionFactor = 0.0625;
p.bus.adcFullScale = 4095;
p.bus.adcRefV = 3.3;

p.motor.polePairs = 7;
p.motor.Rs = 5.29;
p.motor.Ld = 1.058e-3;
p.motor.Lq = 1.058e-3;
p.motor.fluxLinkageWb = 0.034739897/(2*pi);
p.motor.ratedCurrentA = 0.8;
p.motor.maxSpeedRpm = 1572;
p.motor.nominalBusV = 13.0;
p.motor.inertiaKgM2 = 0.291e-4;
p.motor.frictionNmS = 0.937e-5;

p.inverter.shuntOhm = 0.33;
p.inverter.amplifierGain = 1.53;
p.inverter.physicalAdcCountsPerAmp = ...
    p.bus.adcFullScale*p.inverter.shuntOhm*p.inverter.amplifierGain/p.bus.adcRefV;
p.inverter.mcsdkNormalizedCountsPerAmp = ...
    65536*p.inverter.shuntOhm*p.inverter.amplifierGain/p.bus.adcRefV;
p.inverter.dutyMin = 0.03;
p.inverter.dutyMax = 0.97;
p.inverter.enableDeadTime = logical(opt.enableDeadTime);
p.inverter.deadTimeNs = opt.deadTimeNs;
p.inverter.deadTimeCompensationEnabled = logical(opt.enableDeadTimeCompensation);
p.inverter.observerDeadTimeCompensationEnabled = ...
    logical(opt.enableObserverDeadTimeCompensation);
p.inverter.deadTimeCompensationGain = opt.deadTimeCompensationGain;
p.inverter.deadTimeCompensationCurrentBandA = ...
    opt.deadTimeCompensationCurrentBandA;
p.inverter.deadTimeVoltageAtNominalBusV = ...
    2*p.inverter.deadTimeNs*1e-9/p.timing.ts*p.bus.nominalV;

%% PI gains: exact formulas from foc-control/src/params.rs
referencePwmHz = 30000;
referenceVoltageUtilization = 0.95;
referenceMaxVoltage = p.motor.nominalBusV*referenceVoltageUtilization/sqrt(3);
voltsPerNormalizedCount = referenceMaxVoltage/32767;
currentGainScale = p.inverter.mcsdkNormalizedCountsPerAmp*voltsPerNormalizedCount;
p.control.currentRawKp = 3378/1024;
p.control.currentRawKiPerTick = 2252/4096;
p.control.idKp = p.control.currentRawKp*currentGainScale;
p.control.idKi = p.control.currentRawKiPerTick*currentGainScale*referencePwmHz;
p.control.iqKp = p.control.idKp;
p.control.iqKi = p.control.idKi;
p.control.currentOutMin = -referenceMaxVoltage;
p.control.currentOutMax = referenceMaxVoltage;
p.control.voltageUtilization = 0.90; % applications/main.c runtime override

speedUnitsPerRadS = 10/(2*pi);
p.control.speedRawKp = 2730/256;
p.control.speedRawKiPerTick = 562/16384;
p.control.speedKp = p.control.speedRawKp*speedUnitsPerRadS/ ...
                    p.inverter.mcsdkNormalizedCountsPerAmp;
p.control.speedKi = p.control.speedRawKiPerTick*speedUnitsPerRadS/ ...
                    p.inverter.mcsdkNormalizedCountsPerAmp* ...
                    p.timing.speedLoopFrequencyHz;
p.control.speedOutMin = -p.motor.ratedCurrentA;
p.control.speedOutMax = p.motor.ratedCurrentA;

%% Rev-up and closed-loop handoff
p.startup.alignmentS = 1.0;
p.startup.rampS = 1.164;
p.startup.transitionS = 0.025;
p.startup.finalSpeedRpm = 582.0;
p.startup.finalCurrentA = 0.8;
p.startup.closedLoopEnable = double(logical(opt.closedLoop));
p.startup.observerAcquisitionTimeoutS = 0.5;
p.startup.observerLossTimeoutS = 0.05;
p.startup.closedLoopSpeedRampRpmPerS = 500.0;
p.startup.speedPiPreloadRatio = 0.0;
p.startup.closedLoopCurrentSlewAPerS = 32.0;

%% Observer
switch p.meta.profile
    case 'firmware'
        p.observer.kSlideV = 4.0;
        p.observer.boundaryA = 0.16;
        p.observer.emfAlpha = 0.05;
        p.observer.pllKp = 80.0;
        p.observer.pllKi = 1000.0;
    case {'legacy-optimal','optimal'}
        % Retained only to reproduce the obsolete 2026-09-22 sweep. It is not
        % the current firmware profile and must not be transferred to hardware.
        p.meta.profile = 'legacy-optimal';
        p.observer.kSlideV = 1.5;
        p.observer.boundaryA = 0.064;
        p.observer.emfAlpha = 0.8;
        p.observer.pllKp = 220.0;
        p.observer.pllKi = 12000.0;
    otherwise
        error('init_foc_params:profile', ...
              'observerProfile must be firmware or legacy-optimal.');
end
p.observer.Rs = p.motor.Rs;
p.observer.Ls = p.motor.Ld;
p.observer.pllOmegaMin = -2000.0;
p.observer.pllOmegaMax = 2000.0;
p.observer.reliabilityFifoLength = 64;
p.observer.reliabilityDecimatorHz = 1000;
p.observer.reliabilityDecimator = ...
    p.timing.pwmFrequencyHz/p.observer.reliabilityDecimatorHz;
p.observer.reliabilityMinMeanRpm = 524.0;
p.observer.reliabilityMinEmfV = 0.25;
p.observer.reliabilityMaxVarRatio = 0.01;
p.observer.reliabilityWindows = 2;

%% Limits and current hardware references
p.limits.softwareTripA = 1.15;
p.limits.isrDeadlineCycles = 12500;
p.reference.trace = '../simulation/results/hardware_582rpm_12v3_12khz_openloop_run1_20260923.csv';
p.reference.openLoopReliableSamples = 126;
p.reference.hardwareObserverMeanRpm = 582.230158730159;
p.reference.hardwareObserverStdRpm = 31.4006783871719;
p.reference.hardwareIqRmseA = 0.009186;
p.reference.hardwareIdRmseA = 0.008869;
p.reference.hardwarePeakPhaseCurrentA = 0.838;
p.reference.hardwareLatchedPeakCurrentA = 0.903;
p.reference.hasMeasuredShaftSpeed = false;

p = foc_refresh_vectors(p);
end
