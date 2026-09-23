//! Replaceable math operations used by the hard real-time current loop.
//!
//! The control algorithm depends on this small trait instead of an MCU. Host
//! tests and MCUs without a math peripheral use [`CpuMath`]. A board adapter may
//! implement the same contract with CORDIC, an accelerator, or a DSP library.

/// Math required once per current-control sample.
pub trait ControlMath {
    /// Returns `(sin(angle), cos(angle))`.
    fn sin_cos(&mut self, angle_rad: f32) -> (f32, f32);

    /// Returns `sqrt(x*x + y*y)`.
    fn magnitude(&mut self, x: f32, y: f32) -> f32;

    /// Returns `atan2(y, x)` wrapped to `[0, 2*pi)`.
    fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32;
}

/// Portable implementation used by default and as the hardware fallback.
#[derive(Clone, Copy, Debug, Default)]
pub struct CpuMath;

impl ControlMath for CpuMath {
    fn sin_cos(&mut self, angle_rad: f32) -> (f32, f32) {
        (libm::sinf(angle_rad), libm::cosf(angle_rad))
    }

    fn magnitude(&mut self, x: f32, y: f32) -> f32 {
        libm::sqrtf(x * x + y * y)
    }

    fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32 {
        foc_algorithm::atan2_angle_0_to_2pi(y, x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_backend_matches_expected_quadrant_and_magnitude() {
        let mut math = CpuMath;
        let (sin, cos) = math.sin_cos(core::f32::consts::FRAC_PI_2);
        assert!((sin - 1.0).abs() < 1e-6);
        assert!(cos.abs() < 1e-6);
        assert!((math.magnitude(3.0, 4.0) - 5.0).abs() < 1e-6);
        assert!((math.angle_0_to_2pi(1.0, 0.0) - core::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }
}
