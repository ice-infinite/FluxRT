function [da, db, dc] = svpwm(alpha, beta, vbus)
%SVPWM Min-max (zero-sequence injection) SVPWM, identical to the firmware.
%   Mirrors rust/crates/foc-algorithm/src/modulation.rs::svpwm_update.
%   Output duty cycles are clamped to [0,1]; the firmware additionally guards
%   0.03..0.97 in the platform layer, which is applied separately.
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
