function y = wrap_2pi(x)
%WRAP_2PI Wrap angle to [0, 2*pi).
%   Mirrors rust/crates/foc-algorithm/src/math.rs::wrap_angle_0_to_2pi.
y = mod(x, 2*pi);
if y < 0
    y = y + 2*pi;
end
end
