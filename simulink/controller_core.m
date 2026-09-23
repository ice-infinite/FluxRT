function [duty, obsSpeedRpm, state, reliable, ...
          idRef, iqRef, idMeas, iqMeas, vdCmd, vqCmd, ...
          forcedAngle, obsAngle, simTime, phaseCurrents, ...
          controlAngle, speedReferenceRpm, faultFlags] = ...
          controller_core(vbus, iabc, targetRpm, cfg)
%CONTROLLER_CORE Current STM32G431 Rust FOC control chain for Simulink.
% The configuration vector is produced by init_foc_params.m. Tunables stay on
% an input port so simulations never rewrite this source file.
% State: 3 alignment, 4 ramp, 5 hold, 6 transition, 7 closed, 8 fault.

%%#codegen
persistent init idInt iqInt speedInt
persistent obsTheta obsOmega obsPllInteg obsCurA obsCurB obsEmfA obsEmfB
persistent obsFifo obsFifoIdx obsValid obsGood obsSum obsSquareSum obsDecim
persistent prevDa prevDb prevDc elapsed forcedAngleSt
persistent transitionElapsed transitionStarted closedLoop
persistent handoffSpeedRpm handoffStartIq handoffEndIq
persistent observerWait observerLoss
persistent speedInitialized speedRef speedIqCommand speedCounter appliedIq
persistent faultLatched faultBits

if isempty(init)
    init = 1;
    idInt = 0; iqInt = 0; speedInt = 0;
    obsTheta = 0; obsOmega = 0; obsPllInteg = 0;
    obsCurA = 0; obsCurB = 0; obsEmfA = 0; obsEmfB = 0;
    obsFifo = zeros(1,64); obsFifoIdx = 1;
    obsValid = 0; obsGood = 0; obsSum = 0; obsSquareSum = 0; obsDecim = 0;
    prevDa = 0.5; prevDb = 0.5; prevDc = 0.5;
    elapsed = 0; forcedAngleSt = 0;
    transitionElapsed = 0; transitionStarted = 0; closedLoop = 0;
    handoffSpeedRpm = 0; handoffStartIq = 0; handoffEndIq = 0;
    observerWait = 0; observerLoss = 0;
    speedInitialized = 0; speedRef = 0; speedIqCommand = 0;
    speedCounter = 0; appliedIq = 0;
    faultLatched = 0; faultBits = 0;
end

% Configuration layout; see init_foc_params.m::build_controller_vector.
PP          = cfg(1);   RS          = cfg(2);   LS          = cfg(3);
TS          = cfg(4);   VUTIL       = cfg(5);   DUTYMIN     = cfg(6);
DUTYMAX     = cfg(7);   TRIP_A      = cfg(8);   BUS_MIN     = cfg(9);
BUS_MAX     = cfg(10);  IKP         = cfg(11);  IKI         = cfg(12);
IPI_MIN     = cfg(13);  IPI_MAX     = cfg(14);  ALIGN_S     = cfg(15);
RAMP_S      = cfg(16);  TRANS_S     = cfg(17);  FINAL_RPM   = cfg(18);
FINAL_A     = cfg(19);  CLOSED_EN   = cfg(20);  SMO_K       = cfg(21);
SMO_BND     = cfg(22);  SMO_ALPHA   = cfg(23);  PLL_KP      = cfg(24);
PLL_KI      = cfg(25);  PLL_WMIN    = cfg(26);  PLL_WMAX    = cfg(27);
REL_DECIM   = cfg(28);  REL_MIN_RPM = cfg(29);  REL_MIN_EMF = cfg(30);
REL_VAR     = cfg(31);  REL_GOOD    = cfg(32);  ACQUIRE_S   = cfg(33);
LOSS_S      = cfg(34);  SPEED_DIV   = cfg(35);  SPEED_KP    = cfg(36);
SPEED_KI    = cfg(37);  SPEED_MIN   = cfg(38);  SPEED_MAX   = cfg(39);
SPEED_RAMP  = cfg(40);  PRELOAD     = cfg(41);  IQ_SLEW     = cfg(42);
MAX_RPM     = cfg(43);  DEADTIME_NS = cfg(44);  DT_FF_EN    = cfg(45);
DT_FF_GAIN  = cfg(46);  DT_BAND_A   = cfg(47);  DT_OBS_EN   = cfg(48);

ia = iabc(1); ib = iabc(2); ic = iabc(3);
phaseCurrents = [ia; ib; ic];
peakPhase = max(abs([ia ib ic]));
if ~isfinite(ia) || ~isfinite(ib) || ~isfinite(ic) || peakPhase > TRIP_A
    faultLatched = 1;
    faultBits = double(bitor(uint32(faultBits),uint32(1)));
end
if ~isfinite(vbus) || vbus < BUS_MIN || vbus > BUS_MAX
    faultLatched = 1;
    faultBits = double(bitor(uint32(faultBits),uint32(2)));
end

% Fixed defaults for the latched-fault path and code-generation inference.
duty = [0.5; 0.5; 0.5];
obsSpeedRpm = obsOmega*30/(pi*PP);
state = 8; reliable = 0;
idRef = 0; iqRef = 0; idMeas = 0; iqMeas = 0; vdCmd = 0; vqCmd = 0;
forcedAngle = forcedAngleSt; obsAngle = obsTheta;
controlAngle = forcedAngleSt; speedReferenceRpm = speedRef;
simTime = elapsed;

if faultLatched == 0
    %% Previous PWM -> SMO -> PLL.
    ialpha = (2*ia - ib - ic)/3;
    ibeta  = (ib - ic)/sqrt(3);
    signA = currentPolarityLocal(ia,DT_BAND_A);
    signB = currentPolarityLocal(ib,DT_BAND_A);
    signC = currentPolarityLocal(ic,DT_BAND_A);
    deadVoltage = (2*DEADTIME_NS*1e-9/TS)*vbus;
    common = (prevDa + prevDb + prevDc)/3;
    va = (prevDa - common)*vbus;
    vb = (prevDb - common)*vbus;
    vc = (prevDc - common)*vbus;
    % The SMO must use estimated terminal voltage, not the pre-dead-time PWM
    % command. This term mirrors the plant loss and is independently switchable
    % from the modulator feed-forward for A/B testing.
    va = va - DT_OBS_EN*deadVoltage*signA;
    vb = vb - DT_OBS_EN*deadVoltage*signB;
    vc = vc - DT_OBS_EN*deadVoltage*signC;
    valpha = (2*va - vb - vc)/3;
    vbeta  = (vb - vc)/sqrt(3);
    errA = obsCurA - ialpha;
    errB = obsCurB - ibeta;
    slideA = SMO_K*satLocal(errA/SMO_BND);
    slideB = SMO_K*satLocal(errB/SMO_BND);
    obsCurA = obsCurA + (TS/LS)*(valpha - RS*obsCurA - obsEmfA - slideA);
    obsCurB = obsCurB + (TS/LS)*(vbeta - RS*obsCurB - obsEmfB - slideB);
    obsEmfA = obsEmfA + SMO_ALPHA*(slideA - obsEmfA);
    obsEmfB = obsEmfB + SMO_ALPHA*(slideB - obsEmfB);

    thetaEmf = mod(atan2(-obsEmfA, obsEmfB), 2*pi);
    pllErr = wrapPiLocal(thetaEmf - obsTheta);
    obsPllInteg = obsPllInteg + PLL_KI*TS*pllErr;
    proportional = PLL_KP*pllErr;
    rawOmega = proportional + obsPllInteg;
    obsOmega = min(max(rawOmega, PLL_WMIN), PLL_WMAX);
    if rawOmega ~= obsOmega
        obsPllInteg = obsOmega - proportional;
    end
    obsTheta = mod(obsTheta + obsOmega*TS, 2*pi);
    obsSpeedRpm = obsOmega*30/(pi*PP);

    % Decimate first, then insert: 64 samples at 1 kHz = 64 ms.
    obsDecim = obsDecim + 1;
    if obsDecim >= REL_DECIM
        obsDecim = 0;
        oldest = obsFifo(obsFifoIdx);
        obsSum = obsSum + obsSpeedRpm - oldest;
        obsSquareSum = obsSquareSum + obsSpeedRpm*obsSpeedRpm - oldest*oldest;
        obsFifo(obsFifoIdx) = obsSpeedRpm;
        obsFifoIdx = mod(obsFifoIdx,64) + 1;
        obsValid = min(obsValid + 1,64);
        if obsValid < 64
            obsGood = 0;
        else
            meanRpm = obsSum/64;
            variance = max(obsSquareSum/64 - meanRpm*meanRpm,0);
            emfSq = obsEmfA*obsEmfA + obsEmfB*obsEmfB;
            stable = isfinite(obsSpeedRpm) && meanRpm > REL_MIN_RPM && ...
                     meanRpm < MAX_RPM*1.10 && ...
                     emfSq > REL_MIN_EMF*REL_MIN_EMF && ...
                     variance < meanRpm*meanRpm*REL_VAR;
            if stable
                obsGood = min(obsGood + 1,65535);
            else
                obsGood = 0;
            end
        end
    end
    reliable = double(obsGood >= REL_GOOD);

    %% Observer-gated rev-up and shortest-path handoff.
    elapsed = elapsed + TS;
    alignmentComplete = elapsed >= ALIGN_S;
    rampProgress = min(max((elapsed - ALIGN_S)/RAMP_S,0),1);
    openSpeedRpm = FINAL_RPM*rampProgress;
    if ~alignmentComplete
        openPhase = 3; openIq = FINAL_A*elapsed/ALIGN_S;
    elseif rampProgress < 1
        openPhase = 4; openIq = FINAL_A;
    else
        openPhase = 5; openIq = FINAL_A;
    end

    handoffReady = CLOSED_EN ~= 0 && reliable ~= 0;
    transitionJustStarted = 0;
    if transitionStarted == 0 && closedLoop == 0 && alignmentComplete && handoffReady
        transitionStarted = 1;
        transitionJustStarted = 1;
        transitionElapsed = 0;
        handoffSpeedRpm = openSpeedRpm;
        handoffStartIq = openIq;
        observerIq = -ialpha*sin(obsTheta) + ibeta*cos(obsTheta);
        handoffEndIq = min(max(observerIq,-FINAL_A),FINAL_A);
    end

    if closedLoop ~= 0
        phase = 7; startupIq = handoffEndIq;
        forcedSpeedRpm = handoffSpeedRpm; transition = 1;
    elseif transitionStarted ~= 0
        if transitionJustStarted == 0
            transitionElapsed = transitionElapsed + TS;
        end
        if TRANS_S > 0
            transition = min(max(transitionElapsed/TRANS_S,0),1);
        else
            transition = 1;
        end
        startupIq = handoffStartIq + transition*(handoffEndIq - handoffStartIq);
        forcedSpeedRpm = handoffSpeedRpm;
        if transition >= 1
            closedLoop = 1; phase = 7; startupIq = handoffEndIq;
        else
            phase = 6;
        end
    else
        phase = openPhase; startupIq = openIq;
        forcedSpeedRpm = openSpeedRpm; transition = 0;
    end

    forcedAngleSt = mod(forcedAngleSt + forcedSpeedRpm*pi/30*PP*TS,2*pi);
    angleErr = wrapPiLocal(obsTheta - forcedAngleSt);
    ctrlAngle = mod(forcedAngleSt + transition*angleErr,2*pi);
    observerControls = phase == 6 || phase == 7;

    if CLOSED_EN ~= 0 && phase == 5
        observerWait = observerWait + TS;
        if observerWait >= ACQUIRE_S
            faultLatched = 1;
            faultBits = double(bitor(uint32(faultBits),uint32(4)));
        end
    else
        observerWait = 0;
    end
    if observerControls
        if reliable ~= 0
            observerLoss = 0;
        else
            observerLoss = observerLoss + TS;
            if observerLoss >= LOSS_S
                faultLatched = 1;
                faultBits = double(bitor(uint32(faultBits),uint32(8)));
            end
        end
    else
        observerLoss = 0;
    end

    %% Closed-loop speed PI and Iq slew.
    if phase == 7 && faultLatched == 0
        if speedInitialized == 0
            speedRef = obsSpeedRpm;
            desired = startupIq*PRELOAD;
            speedError = speedRef*pi/30 - obsOmega/PP;
            speedIqCommand = min(max(desired,SPEED_MIN),SPEED_MAX);
            speedInt = min(max(speedIqCommand - SPEED_KP*speedError,SPEED_MIN),SPEED_MAX);
            speedIqCommand = min(max(SPEED_KP*speedError + speedInt,SPEED_MIN),SPEED_MAX);
            appliedIq = startupIq;
            speedCounter = 0;
            speedInitialized = 1;
        elseif speedCounter == 0
            maxSpeedStep = SPEED_RAMP*TS*SPEED_DIV;
            speedRef = moveTowardsLocal(speedRef,targetRpm,maxSpeedStep);
            [speedIqCommand,speedInt] = piStepLocal( ...
                speedRef*pi/30,obsOmega/PP,SPEED_KP,SPEED_KI, ...
                TS*SPEED_DIV,speedInt,SPEED_MIN,SPEED_MAX);
        end
        speedCounter = mod(speedCounter + 1,SPEED_DIV);
        appliedIq = moveTowardsLocal(appliedIq,speedIqCommand,IQ_SLEW*TS);
        iqRef = appliedIq;
    else
        speedInitialized = 0; speedInt = 0; speedCounter = 0;
        speedRef = forcedSpeedRpm; speedIqCommand = startupIq;
        appliedIq = startupIq; iqRef = startupIq;
    end
    idRef = 0;

    %% Current PI, circle limit and SVPWM.
    cosT = cos(ctrlAngle); sinT = sin(ctrlAngle);
    idMeas = ialpha*cosT + ibeta*sinT;
    iqMeas = -ialpha*sinT + ibeta*cosT;
    [vdCmd,idInt] = piStepLocal(idRef,idMeas,IKP,IKI,TS,idInt,IPI_MIN,IPI_MAX);
    [vqCmd,iqInt] = piStepLocal(iqRef,iqMeas,IKP,IKI,TS,iqInt,IPI_MIN,IPI_MAX);
    vCircle = VUTIL*vbus/sqrt(3);
    magnitude = sqrt(vdCmd*vdCmd + vqCmd*vqCmd);
    if magnitude > vCircle && magnitude > 0
        vdCmd = vdCmd*vCircle/magnitude;
        vqCmd = vqCmd*vCircle/magnitude;
    end
    vAlpha = vdCmd*cosT - vqCmd*sinT;
    vBeta = vdCmd*sinT + vqCmd*cosT;
    % Phase-domain dead-time feed-forward. The plant subtracts the same
    % polarity-dependent voltage, so the controller adds it before SVPWM.
    % A narrow linear band avoids a discontinuous sign flip at current zero.
    compA = DT_FF_EN*DT_FF_GAIN*deadVoltage*signA;
    compB = DT_FF_EN*DT_FF_GAIN*deadVoltage*signB;
    compC = DT_FF_EN*DT_FF_GAIN*deadVoltage*signC;
    vAlpha = vAlpha + (2*compA - compB - compC)/3;
    vBeta = vBeta + (compB - compC)/sqrt(3);
    totalMagnitude = sqrt(vAlpha*vAlpha + vBeta*vBeta);
    if totalMagnitude > vCircle && totalMagnitude > 0
        vAlpha = vAlpha*vCircle/totalMagnitude;
        vBeta = vBeta*vCircle/totalMagnitude;
    end
    [da,db,dc] = svpwmLocal(vAlpha,vBeta,vbus);
    da = min(max(da,DUTYMIN),DUTYMAX);
    db = min(max(db,DUTYMIN),DUTYMAX);
    dc = min(max(dc,DUTYMIN),DUTYMAX);
    if ~isfinite(da) || ~isfinite(db) || ~isfinite(dc)
        faultLatched = 1;
        faultBits = double(bitor(uint32(faultBits),uint32(16)));
    end
    if faultLatched == 0
        prevDa = da; prevDb = db; prevDc = dc;
        duty = [da; db; dc]; state = phase; controlAngle = ctrlAngle;
    end
    forcedAngle = forcedAngleSt; obsAngle = obsTheta;
    speedReferenceRpm = speedRef; simTime = max(elapsed - TS,0);
end

if faultLatched ~= 0
    duty = [0.5;0.5;0.5];
    prevDa = 0.5; prevDb = 0.5; prevDc = 0.5;
    state = 8; reliable = 0;
end
faultFlags = double(faultBits);
end

function y = satLocal(x)
y = min(max(x,-1),1);
end

function s = currentPolarityLocal(current,band)
if band > 0
    s = min(max(current/band,-1),1);
elseif current > 0
    s = 1;
elseif current < 0
    s = -1;
else
    s = 0;
end
end

function y = wrapPiLocal(x)
y = mod(x + pi,2*pi) - pi;
end

function y = moveTowardsLocal(value,target,maximumStep)
delta = target - value;
if delta > maximumStep
    y = value + maximumStep;
elseif delta < -maximumStep
    y = value - maximumStep;
else
    y = target;
end
end

function [y,integ] = piStepLocal(ref,meas,kp,ki,ts,integ,outMin,outMax)
err = ref - meas;
proportional = kp*err;
integ = min(max(integ + ki*ts*err,outMin),outMax);
unclamped = proportional + integ;
y = min(max(unclamped,outMin),outMax);
if unclamped ~= y
    integ = min(max(y - proportional,outMin),outMax);
end
end

function [da,db,dc] = svpwmLocal(alpha,beta,vbus)
vbusSafe = max(vbus,1e-6);
va = alpha;
vb = -0.5*alpha + 0.5*sqrt(3)*beta;
vc = -0.5*alpha - 0.5*sqrt(3)*beta;
offset = 0.5*(max([va vb vc]) + min([va vb vc]));
da = min(max(0.5 + (va - offset)/vbusSafe,0),1);
db = min(max(0.5 + (vb - offset)/vbusSafe,0),1);
dc = min(max(0.5 + (vc - offset)/vbusSafe,0),1);
end
