function y = wrap_pi(x)
%WRAP_PI Wrap angle to (-pi, pi].
%   Mirrors rust/crates/foc-algorithm/src/math.rs::wrap_angle_minus_pi_to_pi.
y = mod(x + pi, 2*pi) - pi;
if y <= -pi
    y = y + 2*pi;
end
end
