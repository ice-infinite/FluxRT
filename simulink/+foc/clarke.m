function [alpha, beta] = clarke(a, b, c)
%CLARKE Amplitude-invariant Clarke transform, identical to the firmware.
%   Mirrors rust/crates/foc-algorithm/src/transform.rs::clarke so the Simulink
%   model and the target compute the same alpha/beta.
%
% 幅值不变 Clarke 变换：三相电流 [A] → alpha/beta [A]。
% Amplitude-invariant Clarke transform: phase currents [A] -> alpha/beta [A].
%
% 系数 2/3 与 1/sqrt(3) 保证 alpha/beta 的幅值等于相电流幅值，这是"幅值不变"而不是
% "功率不变"形式；换成后者会让所有电流环增益和观察器门限整体差 3/2 倍。
% 该变换隐含 ia+ib+ic=0（三相三线星形接法），采样共模零偏不会在这里被消除，
% 零偏要靠 ADC 零点校准处理。
% The 2/3 and 1/sqrt(3) factors keep the alpha/beta amplitude equal to the phase amplitude:
% this is the amplitude-invariant, not the power-invariant form, and switching would shift
% every current-loop gain and observer gate by 3/2. The transform assumes ia+ib+ic=0
% (three-wire star); a common-mode sampling offset is not removed here and must be handled by
% ADC zero calibration.
%
% 位置 / Placement: controller_core.m 用的是同形的局部实现，而 simulink/ 下目前没有任何
% 脚本调用本包，因此这里主要作为与固件算法对照的参考副本，改动必须手工同步两边。
% controller_core.m carries its own copy of the same formula and nothing under simulink/
% currently calls this package, so it is a reference copy against the firmware; any change must
% be mirrored by hand.
alpha = (2*a - b - c) / 3;
beta  = (b - c) / sqrt(3);
end
