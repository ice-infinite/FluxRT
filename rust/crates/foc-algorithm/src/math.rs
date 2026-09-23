//! Common/foc_math 的等价实现。

pub const PI: f32 = core::f32::consts::PI;
pub const TWO_PI: f32 = core::f32::consts::TAU;
pub const SQRT_3: f32 = 1.732_050_8;
pub const INV_SQRT_3: f32 = 0.577_350_26;

#[inline]
pub fn clamp(value: f32, mut min_value: f32, mut max_value: f32) -> f32 {
    if min_value > max_value {
        core::mem::swap(&mut min_value, &mut max_value);
    }
    if value < min_value {
        min_value
    } else if value > max_value {
        max_value
    } else {
        value
    }
}

#[inline]
pub fn wrap_angle_0_to_2pi(angle_rad: f32) -> f32 {
    if !angle_rad.is_finite() {
        return f32::NAN;
    }
    let mut wrapped = angle_rad;
    while wrapped >= TWO_PI {
        wrapped -= TWO_PI;
    }
    if wrapped < 0.0 {
        while wrapped < 0.0 {
            wrapped += TWO_PI;
        }
    }
    wrapped
}

#[inline]
pub fn wrap_angle_minus_pi_to_pi(angle_rad: f32) -> f32 {
    wrap_angle_0_to_2pi(angle_rad + PI) - PI
}

#[inline]
pub fn atan2_angle_0_to_2pi(y: f32, x: f32) -> f32 {
    wrap_angle_0_to_2pi(libm::atan2f(y, x))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    #[test]
    fn matches_c_reference_vectors() {
        assert_near(clamp(2.0, -1.0, 1.0), 1.0, 1e-6);
        assert_near(clamp(-2.0, -1.0, 1.0), -1.0, 1e-6);
        assert_near(clamp(2.0, 1.0, -1.0), 1.0, 1e-6);
        assert_near(wrap_angle_0_to_2pi(TWO_PI), 0.0, 1e-5);
        assert_near(wrap_angle_0_to_2pi(-0.5), TWO_PI - 0.5, 1e-5);
        assert_near(wrap_angle_minus_pi_to_pi(PI + 0.1), -PI + 0.1, 1e-5);
        assert_near(atan2_angle_0_to_2pi(1.0, 0.0), PI * 0.5, 1e-5);
        assert_near(atan2_angle_0_to_2pi(-1.0, 0.0), PI * 1.5, 1e-5);
    }
}
