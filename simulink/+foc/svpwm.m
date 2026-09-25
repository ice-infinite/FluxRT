function [da, db, dc] = svpwm(alpha, beta, vbus)
%SVPWM Min-max (zero-sequence injection) SVPWM, identical to the firmware.
%   Mirrors rust/crates/foc-algorithm/src/modulation.rs::svpwm_update.
%   Output duty cycles are clamped to [0,1]; the firmware additionally guards
%   0.03..0.97 in the platform layer, which is applied separately.
%
% 最小-最大（零序注入）SVPWM：alpha/beta [V]、vbus [V] → 三相占空比 [0,1]。
% Min-max (zero-sequence injection) SVPWM: alpha/beta [V] and vbus [V] -> duty [0,1].
%
% 注入 -(max+min)/2 的零序分量等价于在每个扇区选择最近的可用矢量组合，比正弦调制多出约
% 15% 的线性区（线电压峰值可用到 vbus），这也是圆限幅用 vbus/sqrt(3) 作相电压峰值的原因。
% Injecting -(max+min)/2 is equivalent to picking the nearest vector combination per sector
% and buys about 15% more linear range than sine modulation (line peak up to vbus), which is
% why the circle limit uses vbus/sqrt(3) as the phase-voltage peak.
%
% vbus<=0 时直接返回 0.5/0.5/0.5，即零矢量、相电压为零，属于安全输出。控制器里的同形
% svpwmLocal 在该分支上改为把 vbus 下限钳到 1e-6，以便 MATLAB Function 块始终给出确定
% 宽度；两处行为刻意不同，不能声称完全逐字一致。
% A non-positive vbus returns 0.5/0.5/0.5 (zero vector, no phase voltage) as a safe output.
% The same-shaped svpwmLocal in the controller instead floors vbus at 1e-6 so the MATLAB
% Function block always produces a definite width, so the two differ deliberately on this
% branch and are not literally identical.
da = 0.5; db = 0.5; dc = 0.5;
if vbus <= 0
    return
end
va = alpha;
vb = -0.5*alpha + 0.5*sqrt(3)*beta;
vc = -0.5*alpha - 0.5*sqrt(3)*beta;
voff = 0.5 * (max([va vb vc]) + min([va vb vc]));
da = min(max(0.5 + (va - voff)/vbus, 0), 1);
db = min(max(0.5 + (vb - voff)/vbus, 0), 1);
dc = min(max(0.5 + (vc - voff)/vbus, 0), 1);
end
