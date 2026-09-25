function y = wrap_2pi(x)
%WRAP_2PI Wrap angle to [0, 2*pi).
%   Mirrors rust/crates/foc-algorithm/src/math.rs::wrap_angle_0_to_2pi.
%
% 把角度 [rad] 折到 [0, 2*pi)：用于电角、强制角这类需要单调递增域的量。
% Wraps an angle [rad] into [0, 2*pi): used for electrical and forced angles, which need a
% monotonic increasing domain.
%
% 下面的 if 分支在 MATLAB 里实际上是防御性的：mod(x,2*pi) 对正模数已经保证结果非负，
% 正常路径不会进入；它保留的是"负角度也要补齐"的意图。与 Rust 版用循环加减 2*pi 不同，
% 这里用 mod，因此没有循环次数随 |角度| 线性增长的问题，但两者在浮点误差上并非逐位一致。
% The if branch below is effectively defensive in MATLAB: mod(x,2*pi) already guarantees a
% non-negative result for a positive modulus, so the normal path never enters it; it documents
% the "top up negative angles" intent. Unlike the Rust version, which loops with 2*pi
% add/subtract, this uses mod, so the work does not grow with |angle| - but the two are not
% bit-for-bit identical in floating point.
y = mod(x, 2*pi);
if y < 0
    y = y + 2*pi;
end
end
