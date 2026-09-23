//! ADRC、非线性 `fal`、快速跟踪微分器与带宽整定。

use crate::math::clamp;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcParam {
    pub ts: f32,
    pub td_r: f32,
    pub beta01: f32,
    pub beta02: f32,
    pub beta03: f32,
    pub kp: f32,
    pub kd: f32,
    pub b0: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcState {
    pub v1: f32,
    pub v2: f32,
    pub z1: f32,
    pub z2: f32,
    pub z3: f32,
    pub error: f32,
    pub u0: f32,
    pub output: f32,
}

impl AdrcState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &AdrcParam, reference: f32, feedback: f32) -> f32 {
        if param.ts <= 0.0 {
            return 0.0;
        }

        let b0 = if param.b0 == 0.0 { 1.0 } else { param.b0 };
        let td_acc = param.td_r * (reference - self.v1) - 2.0 * param.td_r * self.v2;
        self.v1 += param.ts * self.v2;
        self.v2 += param.ts * td_acc;

        let eso_error = self.z1 - feedback;
        let z1_dot = self.z2 - param.beta01 * eso_error;
        let z2_dot = self.z3 - param.beta02 * eso_error + b0 * self.output;
        let z3_dot = -param.beta03 * eso_error;
        self.z1 += param.ts * z1_dot;
        self.z2 += param.ts * z2_dot;
        self.z3 += param.ts * z3_dot;

        self.error = self.v1 - self.z1;
        self.u0 = param.kp * (self.v1 - self.z1) + param.kd * (self.v2 - self.z2);
        self.output = clamp((self.u0 - self.z3) / b0, param.out_min, param.out_max);
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcFalParam {
    pub alpha: f32,
    pub delta: f32,
}

#[inline]
fn sign(value: f32) -> f32 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

pub fn adrc_fal(error: f32, param: &AdrcFalParam) -> f32 {
    let alpha = clamp(param.alpha, 0.0, 1.0);
    let delta = if param.delta <= 0.0 {
        1.0e-6
    } else {
        param.delta
    };
    let abs_error = error.abs();
    if abs_error <= delta {
        error / libm::powf(delta, 1.0 - alpha)
    } else {
        libm::powf(abs_error, alpha) * sign(error)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcFastTdParam {
    pub ts: f32,
    pub r: f32,
    pub h0: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcFastTdState {
    pub v1: f32,
    pub v2: f32,
    pub fh: f32,
}

impl AdrcFastTdState {
    pub fn new(initial_ref: f32) -> Self {
        Self {
            v1: initial_ref,
            ..Self::default()
        }
    }

    pub fn reset(&mut self, reference: f32) {
        *self = Self::new(reference);
    }

    pub fn update(&mut self, param: &AdrcFastTdParam, reference: f32) -> f32 {
        if param.ts <= 0.0 || param.r <= 0.0 || param.h0 <= 0.0 {
            return 0.0;
        }

        let d = param.r * param.h0 * param.h0;
        let a0 = param.h0 * self.v2;
        let y = self.v1 - reference + a0;
        let a1 = libm::sqrtf(d * (d + 8.0 * y.abs()));
        let a2 = a0 + sign(y) * (a1 - d) * 0.5;
        let sy = (sign(y + d) - sign(y - d)) * 0.5;
        let a = (a0 + y - a2) * sy + a2;
        let sa = (sign(a + d) - sign(a - d)) * 0.5;
        self.fh = -param.r * (a / d - sign(a)) * sa - param.r * sign(a);
        self.v1 += param.ts * self.v2;
        self.v2 += param.ts * self.fh;
        self.v1
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcNonlinearParam {
    pub base: AdrcParam,
    pub e1_fal: AdrcFalParam,
    pub e2_fal: AdrcFalParam,
}

impl AdrcState {
    pub fn update_nonlinear(
        &mut self,
        param: &AdrcNonlinearParam,
        reference: f32,
        feedback: f32,
    ) -> f32 {
        let base = &param.base;
        if base.ts <= 0.0 {
            return 0.0;
        }

        let b0 = if base.b0 == 0.0 { 1.0 } else { base.b0 };
        let td_acc = base.td_r * (reference - self.v1) - 2.0 * base.td_r * self.v2;
        self.v1 += base.ts * self.v2;
        self.v2 += base.ts * td_acc;

        let eso_error = self.z1 - feedback;
        self.z1 += base.ts * (self.z2 - base.beta01 * eso_error);
        self.z2 += base.ts * (self.z3 - base.beta02 * eso_error + b0 * self.output);
        self.z3 += base.ts * (-base.beta03 * eso_error);

        let e1 = self.v1 - self.z1;
        let e2 = self.v2 - self.z2;
        self.error = e1;
        self.u0 = base.kp * adrc_fal(e1, &param.e1_fal) + base.kd * adrc_fal(e2, &param.e2_fal);
        self.output = clamp((self.u0 - self.z3) / b0, base.out_min, base.out_max);
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcTuningParam {
    pub ts: f32,
    pub control_bandwidth: f32,
    pub observer_bandwidth: f32,
    pub b0: f32,
    pub td_r: f32,
    pub out_min: f32,
    pub out_max: f32,
}

pub fn tune_adrc_bandwidth(param: &AdrcTuningParam) -> AdrcParam {
    let wc = param.control_bandwidth.max(0.0);
    let wo = param.observer_bandwidth.max(0.0);
    AdrcParam {
        ts: param.ts,
        td_r: param.td_r,
        beta01: 3.0 * wo,
        beta02: 3.0 * wo * wo,
        beta03: wo * wo * wo,
        kp: wc * wc,
        kd: 2.0 * wc,
        b0: param.b0,
        out_min: param.out_min,
        out_max: param.out_max,
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

    fn base_param() -> AdrcParam {
        AdrcParam {
            ts: 0.01,
            td_r: 10.0,
            beta01: 20.0,
            beta02: 100.0,
            beta03: 50.0,
            kp: 5.0,
            kd: 2.0,
            b0: 1.0,
            out_min: -10.0,
            out_max: 10.0,
        }
    }

    #[test]
    fn adrc_matches_c_reference() {
        let mut param = base_param();
        let mut state = AdrcState::default();
        near(state.update(&param, 1.0, 0.0), 0.2, 1e-6);
        near(state.v1, 0.0, 1e-6);
        near(state.v2, 0.1, 1e-6);

        state.reset();
        state.z1 = 1.0;
        assert!(state.update(&param, 0.0, 0.0) < 0.0);
        near(state.z1, 0.8, 1e-6);
        near(state.z2, -1.0, 1e-6);
        near(state.z3, -0.5, 1e-6);
        param.out_min = -0.1;
        param.out_max = 0.1;
        near(state.update(&param, 10.0, 0.0), 0.1, 1e-6);
    }

    #[test]
    fn fal_matches_c_reference_and_sanitizes_parameters() {
        let param = AdrcFalParam {
            alpha: 0.5,
            delta: 0.01,
        };
        near(adrc_fal(0.0025, &param), 0.025, 1e-6);
        near(adrc_fal(4.0, &param), 2.0, 1e-6);
        near(adrc_fal(-4.0, &param), -2.0, 1e-6);
        assert!(adrc_fal(
            1.0e-7,
            &AdrcFalParam {
                alpha: 2.0,
                delta: 0.0
            }
        )
        .is_finite());
    }

    #[test]
    fn fast_td_matches_c_reference_and_rejects_invalid_timing() {
        let param = AdrcFastTdParam {
            ts: 0.001,
            r: 100.0,
            h0: 0.01,
        };
        let mut state = AdrcFastTdState::new(0.0);
        near(state.update(&param, 1.0), 0.0, 0.0);
        assert!(state.v2 > 0.0);
        let before = state;
        near(
            state.update(&AdrcFastTdParam { ts: 0.0, ..param }, 1.0),
            0.0,
            0.0,
        );
        assert_eq!(state, before);
    }

    #[test]
    fn nonlinear_adrc_matches_c_reference() {
        let param = AdrcNonlinearParam {
            base: base_param(),
            e1_fal: AdrcFalParam {
                alpha: 0.5,
                delta: 0.01,
            },
            e2_fal: AdrcFalParam {
                alpha: 0.25,
                delta: 0.01,
            },
        };
        let output = AdrcState::default().update_nonlinear(&param, 1.0, 0.0);
        assert!(output.is_finite() && output > 0.0 && output <= 10.0);
    }

    #[test]
    fn tuning_matches_c_reference() {
        let tuned = tune_adrc_bandwidth(&AdrcTuningParam {
            ts: 0.001,
            control_bandwidth: 20.0,
            observer_bandwidth: 100.0,
            b0: 2.0,
            td_r: 200.0,
            out_min: -5.0,
            out_max: 5.0,
        });
        near(tuned.kp, 400.0, 1e-6);
        near(tuned.kd, 40.0, 1e-6);
        near(tuned.beta01, 300.0, 1e-6);
        near(tuned.beta02, 30_000.0, 1e-3);
        near(tuned.beta03, 1_000_000.0, 1e-1);
        near(tuned.b0, 2.0, 1e-6);
    }
}
