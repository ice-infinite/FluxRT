//! Angle_CORDIC 的等价实现。

use crate::math::{wrap_angle_0_to_2pi, PI};

const ATAN_TABLE: [f32; 16] = [
    core::f32::consts::FRAC_PI_4,
    0.463_647_6,
    0.244_978_67,
    0.124_354_996,
    0.062_418_81,
    0.031_239_834,
    0.015_623_729,
    0.007_812_341,
    0.003_906_230_2,
    0.001_953_122_6,
    0.000_976_562_2,
    0.000_488_281_22,
    0.000_244_140_62,
    0.000_122_070_31,
    0.000_061_035_156,
    0.000_030_517_578,
];

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CordicOutput {
    pub angle_rad: f32,
    pub magnitude: f32,
}

fn iteration_count(iterations: i32) -> usize {
    if iterations <= 0 {
        ATAN_TABLE.len()
    } else {
        (iterations as usize).min(ATAN_TABLE.len())
    }
}

fn cordic_gain(count: usize) -> f32 {
    let mut gain = 1.0;
    for i in 0..count {
        let factor = libm::ldexpf(1.0, -(i as i32));
        gain *= libm::sqrtf(1.0 + factor * factor);
    }
    gain
}

pub fn cordic_atan2(y: f32, x: f32, iterations: i32) -> CordicOutput {
    if x == 0.0 && y == 0.0 {
        return CordicOutput::default();
    }

    let count = iteration_count(iterations);
    let mut x_work = x;
    let mut y_work = y;
    let mut quadrant_offset = 0.0;
    if x_work < 0.0 {
        x_work = -x_work;
        y_work = -y_work;
        quadrant_offset = PI;
    }

    let mut z = 0.0;
    for (i, atan) in ATAN_TABLE.iter().copied().enumerate().take(count) {
        let factor = libm::ldexpf(1.0, -(i as i32));
        let (x_next, y_next) = if y_work > 0.0 {
            z += atan;
            (x_work + y_work * factor, y_work - x_work * factor)
        } else {
            z -= atan;
            (x_work - y_work * factor, y_work + x_work * factor)
        };
        x_work = x_next;
        y_work = y_next;
    }

    CordicOutput {
        angle_rad: wrap_angle_0_to_2pi(z + quadrant_offset),
        magnitude: x_work / cordic_gain(count),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    #[test]
    fn matches_c_reference_vectors() {
        let out = cordic_atan2(0.0, 1.0, 16);
        near(out.angle_rad, 0.0, 1e-4);
        near(out.magnitude, 1.0, 1e-4);
        near(cordic_atan2(1.0, 0.0, 16).angle_rad, PI * 0.5, 1e-4);
        let q2 = cordic_atan2(1.0, -1.0, 16);
        near(q2.angle_rad, PI * 0.75, 1e-4);
        near(q2.magnitude, libm::sqrtf(2.0), 1e-4);
        near(cordic_atan2(-1.0, -1.0, 16).angle_rad, PI * 1.25, 1e-4);
        assert_eq!(cordic_atan2(0.0, 0.0, 16), CordicOutput::default());
    }
}
