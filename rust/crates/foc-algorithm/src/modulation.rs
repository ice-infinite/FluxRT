//! SinePWM、SVPWM 与 DPWM 的等价实现。

use crate::math::{clamp, PI, SQRT_3};
use crate::transform::AlphaBeta;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SinePwmOutput {
    pub duty_a: f32,
    pub duty_b: f32,
    pub duty_c: f32,
}

#[inline]
pub fn sine_pwm_update(theta_rad: f32, modulation: f32) -> SinePwmOutput {
    let m = clamp(modulation, 0.0, 1.0);
    SinePwmOutput {
        duty_a: clamp(0.5 + 0.5 * m * libm::sinf(theta_rad), 0.0, 1.0),
        duty_b: clamp(
            0.5 + 0.5 * m * libm::sinf(theta_rad - 2.0 * PI / 3.0),
            0.0,
            1.0,
        ),
        duty_c: clamp(
            0.5 + 0.5 * m * libm::sinf(theta_rad + 2.0 * PI / 3.0),
            0.0,
            1.0,
        ),
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SvpwmParam {
    pub v_bus: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SvpwmOutput {
    pub duty_a: f32,
    pub duty_b: f32,
    pub duty_c: f32,
    pub sector: i32,
}

impl Default for SvpwmOutput {
    fn default() -> Self {
        Self {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
            sector: 0,
        }
    }
}

#[inline]
fn svpwm_sector(va: f32, vb: f32, vc: f32) -> i32 {
    let mut code = 0;
    if va > 0.0 {
        code |= 1;
    }
    if vb > 0.0 {
        code |= 2;
    }
    if vc > 0.0 {
        code |= 4;
    }
    match code {
        3 => 1,
        1 => 2,
        5 => 3,
        4 => 4,
        6 => 5,
        2 => 6,
        _ => 0,
    }
}

#[inline]
pub fn svpwm_update(v_alpha_beta: AlphaBeta, param: &SvpwmParam) -> SvpwmOutput {
    if param.v_bus <= 0.0 {
        return SvpwmOutput::default();
    }
    let va = v_alpha_beta.alpha;
    let vb = -0.5 * v_alpha_beta.alpha + 0.5 * SQRT_3 * v_alpha_beta.beta;
    let vc = -0.5 * v_alpha_beta.alpha - 0.5 * SQRT_3 * v_alpha_beta.beta;
    let v_max = va.max(vb).max(vc);
    let v_min = va.min(vb).min(vc);
    let v_offset = 0.5 * (v_max + v_min);
    SvpwmOutput {
        duty_a: clamp(0.5 + (va - v_offset) / param.v_bus, 0.0, 1.0),
        duty_b: clamp(0.5 + (vb - v_offset) / param.v_bus, 0.0, 1.0),
        duty_c: clamp(0.5 + (vc - v_offset) / param.v_bus, 0.0, 1.0),
        sector: svpwm_sector(va, vb, vc),
    }
}

#[repr(i32)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum DpwmMode {
    #[default]
    ClampMax = 0,
    ClampMin = 1,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DpwmParam {
    pub v_bus: f32,
    pub mode: DpwmMode,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DpwmOutput {
    pub duty_a: f32,
    pub duty_b: f32,
    pub duty_c: f32,
    pub zero_sequence: f32,
}

impl Default for DpwmOutput {
    fn default() -> Self {
        Self {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
            zero_sequence: 0.0,
        }
    }
}

#[inline]
pub fn dpwm_update(v_alpha_beta: AlphaBeta, param: &DpwmParam) -> DpwmOutput {
    if param.v_bus <= 0.0 {
        return DpwmOutput::default();
    }
    let va = v_alpha_beta.alpha;
    let vb = -0.5 * v_alpha_beta.alpha + 0.5 * SQRT_3 * v_alpha_beta.beta;
    let vc = -0.5 * v_alpha_beta.alpha - 0.5 * SQRT_3 * v_alpha_beta.beta;
    let d0_a = 0.5 + va / param.v_bus;
    let d0_b = 0.5 + vb / param.v_bus;
    let d0_c = 0.5 + vc / param.v_bus;
    let d_max = d0_a.max(d0_b).max(d0_c);
    let d_min = d0_a.min(d0_b).min(d0_c);
    let zero = match param.mode {
        DpwmMode::ClampMin => -d_min,
        DpwmMode::ClampMax => 1.0 - d_max,
    };
    DpwmOutput {
        duty_a: clamp(d0_a + zero, 0.0, 1.0),
        duty_b: clamp(d0_b + zero, 0.0, 1.0),
        duty_c: clamp(d0_c + zero, 0.0, 1.0),
        zero_sequence: zero,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn in_duty_range(value: f32) -> bool {
        (0.0..=1.0).contains(&value)
    }

    #[test]
    fn sine_pwm_matches_c_reference() {
        let zero = sine_pwm_update(0.0, 0.0);
        assert!((zero.duty_a - 0.5).abs() <= 1e-6);
        assert!((zero.duty_b - 0.5).abs() <= 1e-6);
        assert!((zero.duty_c - 0.5).abs() <= 1e-6);
        let peak = sine_pwm_update(PI * 0.5, 1.0);
        assert!((peak.duty_a - 1.0).abs() <= 1e-6);
        assert!(in_duty_range(peak.duty_b) && in_duty_range(peak.duty_c));
    }

    #[test]
    fn svpwm_matches_c_reference() {
        let p = SvpwmParam { v_bus: 24.0 };
        let zero = svpwm_update(AlphaBeta::default(), &p);
        assert_eq!(zero, SvpwmOutput::default());
        let out = svpwm_update(
            AlphaBeta {
                alpha: 6.0,
                beta: 0.0,
            },
            &p,
        );
        assert!(in_duty_range(out.duty_a));
        assert!(in_duty_range(out.duty_b));
        assert!(in_duty_range(out.duty_c));
        assert!((1..=6).contains(&out.sector));
        assert!(out.duty_a > out.duty_b && out.duty_a > out.duty_c);
    }

    #[test]
    fn dpwm_matches_c_reference() {
        let voltage = AlphaBeta {
            alpha: 6.0,
            beta: 0.0,
        };
        let mut p = DpwmParam {
            v_bus: 24.0,
            mode: DpwmMode::ClampMax,
        };
        let max = dpwm_update(voltage, &p);
        assert!((max.duty_a - 1.0).abs() <= 1e-6);
        p.mode = DpwmMode::ClampMin;
        let min = dpwm_update(voltage, &p);
        assert!((min.duty_b - 0.0).abs() <= 1e-6);
        assert!((min.duty_c - 0.0).abs() <= 1e-6);
    }
}
