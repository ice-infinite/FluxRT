function y = wrap_pi(x)
%WRAP_PI Wrap angle to (-pi, pi].
%   Mirrors rust/crates/foc-algorithm/src/math.rs::wrap_angle_minus_pi_to_pi.
%
% 把角度 [rad] 折到 (-pi, pi]：误差类量（PLL 相位误差、接管最短角差、对拍时的角度误差）
% 必须用最短角差，直接相减两个 [0,2*pi) 的角度会得到差 ±2*pi 的错误结果。
% Wraps an angle [rad] into (-pi, pi]: error-like quantities (PLL phase error, handoff shortest
% path, comparison angle error) need the shortest angular difference, while subtracting two
% [0,2*pi) angles directly can be off by ±2*pi.
%
% 区间细节 / Interval detail: mod(x+pi,2*pi)-pi 的取值域本来是 [-pi, pi)，下面的 if 把
% y==-pi 这一点映到 +pi，才得到右闭区间 (-pi, pi]。Rust 的 wrap_angle_minus_pi_to_pi 直接
% 返回 [-pi, pi)，因此两者只在 y==-pi 单点上不同（测度为零，不改变数值结论）。
% mod(x+pi,2*pi)-pi naturally lands in [-pi, pi); the if below maps the single point y==-pi to
% +pi to obtain the right-closed interval. The Rust version returns [-pi, pi) directly, so the
% two differ only at that single point, which does not change any numerical conclusion.
y = mod(x + pi, 2*pi) - pi;
if y <= -pi
    y = y + 2*pi;
end
end
