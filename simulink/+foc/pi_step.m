function [y, integ] = pi_step(ref, meas, kp, ki, ts, integ, outMin, outMax)
%PI_STEP One discrete PI update with anti-windup, matching foc-algorithm PiState.
%   The integrator is clamped, and when the summed output saturates the
%   integrator is back-calculated so it does not wind up further.
%   Mirrors rust/crates/foc-algorithm/src/controller.rs (PiState::update).
%
% 一步离散 PI（带抗积分饱和）：ref/meas 同量纲，kp [输出/输入]、ki [输出/(输入*s)]、
% ts [s]、integ 与输出同量纲、outMin/outMax 同时限制输出和积分器。
% One discrete PI update with anti-windup: ref/meas share a unit, kp is [out/in], ki is
% [out/(in*s)], ts is [s], integ carries the output unit, and outMin/outMax clamp both the
% output and the integrator.
%
% 抗饱和方式 / Anti-windup: 先累加积分并把积分器钳进 outMin..outMax，然后判断 kp*err+integ
% 是否越限；越限时把积分器反算成 outMax-kp*err（或 outMin-kp*err）并再钳一次，使 PI 立刻
% 退出饱和而不是继续累积。
% The integral is accumulated and clamped into outMin..outMax first, then kp*err+integ is
% tested against the limits; on saturation the integrator is back-calculated to
% outMax-kp*err (or outMin-kp*err) and clamped again so the loop leaves saturation
% immediately instead of winding up further.
%
% 与 controller_core.m 的差异 / Difference from controller_core.m: 那里的 piStepLocal 是同族
% 但不等价的局部实现（用 unclamped~=y 判断并反算到 y-proportional），本包也没有被
% simulink/ 下任何脚本调用；改动其中一处必须核对另一处。
% piStepLocal in controller_core.m is a related but not equivalent local implementation, and
% nothing under simulink/ calls this package, so a change to either must be checked against
% the other.
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
