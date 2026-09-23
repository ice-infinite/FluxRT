function [alpha, beta] = clarke(a, b, c)
%CLARKE Amplitude-invariant Clarke transform, identical to the firmware.
%   Mirrors rust/crates/foc-algorithm/src/transform.rs::clarke so the Simulink
%   model and the target compute the same alpha/beta.
alpha = (2*a - b - c) / 3;
beta  = (b - c) / sqrt(3);
end
