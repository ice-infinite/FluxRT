function [iabc, trueSpeedRpm, trueAngle, torqueE, idPlant, iqPlant] = ...
         pmsm_plant(duty, vbus, loadTorque, cfg)
%PMSM_PLANT Discrete PMSM plant matching rust/crates/foc-sim.
% An optional averaged dead-time voltage loss is retained for hardware
% correlation. Disable it for a strict comparison with the Rust host plant.

%%#codegen
persistent init id iq mechSpeed mechAngle prevIa prevIb prevIc
if isempty(init)
    init = 1;
    id = 0; iq = 0; mechSpeed = 0; mechAngle = 0;
    prevIa = 0; prevIb = 0; prevIc = 0;
end

RS = cfg(1); LD = cfg(2); LQ = cfg(3); FLUX = cfg(4); PP = cfg(5);
J = cfg(6); B = cfg(7); TS = cfg(8);
DEAD_ENABLE = cfg(9); DEADTIME_NS = cfg(10);

da = duty(1); db = duty(2); dc = duty(3);
common = (da + db + dc)/3;
va = (da - common)*vbus;
vb = (db - common)*vbus;
vc = (dc - common)*vbus;
deadLoss = DEAD_ENABLE*(2*DEADTIME_NS*1e-9/TS)*vbus;
va = va - deadLoss*sgnLocal(prevIa);
vb = vb - deadLoss*sgnLocal(prevIb);
vc = vc - deadLoss*sgnLocal(prevIc);

valpha = (2*va - vb - vc)/3;
vbeta = (vb - vc)/sqrt(3);
elecAngle = mod(PP*mechAngle,2*pi);
cosT = cos(elecAngle); sinT = sin(elecAngle);
vd = valpha*cosT + vbeta*sinT;
vq = -valpha*sinT + vbeta*cosT;
elecSpeed = PP*mechSpeed;
did = (vd - RS*id + elecSpeed*LQ*iq)/LD;
diq = (vq - RS*iq - elecSpeed*(LD*id + FLUX))/LQ;
id = id + TS*did;
iq = iq + TS*diq;

torqueE = 1.5*PP*(FLUX*iq + (LD - LQ)*id*iq);
acceleration = (torqueE - loadTorque - B*mechSpeed)/J;
mechSpeed = mechSpeed + TS*acceleration;
mechAngle = mod(mechAngle + TS*mechSpeed,2*pi);
elecAngle = mod(PP*mechAngle,2*pi);

ia = id*cos(elecAngle) - iq*sin(elecAngle);
ib = id*cos(elecAngle - 2*pi/3) - iq*sin(elecAngle - 2*pi/3);
ic = id*cos(elecAngle + 2*pi/3) - iq*sin(elecAngle + 2*pi/3);
iabc = [ia;ib;ic];
prevIa = ia; prevIb = ib; prevIc = ic;
trueSpeedRpm = mechSpeed*30/pi;
trueAngle = elecAngle;
idPlant = id;
iqPlant = iq;
end

function s = sgnLocal(x)
if x > 0
    s = 1;
elseif x < 0
    s = -1;
else
    s = 0;
end
end
