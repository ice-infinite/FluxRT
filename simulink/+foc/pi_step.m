function [y, integ] = pi_step(ref, meas, kp, ki, ts, integ, outMin, outMax)
%PI_STEP One discrete PI update with anti-windup, matching foc-algorithm PiState.
%   The integrator is clamped, and when the summed output saturates the
%   integrator is back-calculated so it does not wind up further.
%   Mirrors rust/crates/foc-algorithm/src/controller.rs (PiState::update).
err  = ref - meas;
integ = integ + ki * ts * err;
integ = min(max(integ, outMin), outMax);
y = kp * err + integ;
if y > outMax
    y = outMax;
    integ = outMax - kp*err;
elseif y < outMin
    y = outMin;
    integ = outMin - kp*err;
end
integ = min(max(integ, outMin), outMax);
end
