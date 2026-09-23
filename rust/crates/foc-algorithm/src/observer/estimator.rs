//! 标量 Kalman、二阶 Luenberger、EKF、EKF+FOC 与一维 UKF。
//!
//! EKF/UKF 模型使用可空的 `extern "C" fn`，不使用堆分配或 trait object。

use crate::foc::{FocBasicInput, FocBasicParam, FocBasicState};
use crate::math::wrap_angle_0_to_2pi;
use crate::modulation::SvpwmOutput;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KalmanParam {
    pub a: f32,
    pub b: f32,
    pub h: f32,
    pub q: f32,
    pub r: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KalmanState {
    pub x: f32,
    pub p: f32,
    pub k: f32,
    pub residual: f32,
}

impl KalmanState {
    pub const fn new(initial_x: f32, initial_p: f32) -> Self {
        Self {
            x: initial_x,
            p: initial_p,
            k: 0.0,
            residual: 0.0,
        }
    }

    pub fn reset(&mut self, x: f32, p: f32) {
        *self = Self::new(x, p);
    }

    pub fn update(&mut self, param: &KalmanParam, u: f32, measurement: f32) -> f32 {
        let x_pred = param.a * self.x + param.b * u;
        let p_pred = param.a * self.p * param.a + param.q;
        let mut s = param.h * p_pred * param.h + param.r;
        if s == 0.0 {
            s = 1.0;
        }
        self.k = p_pred * param.h / s;
        self.residual = measurement - param.h * x_pred;
        self.x = x_pred + self.k * self.residual;
        self.p = (1.0 - self.k * param.h) * p_pred;
        self.x
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LuenbergerParam {
    pub a00: f32,
    pub a01: f32,
    pub a10: f32,
    pub a11: f32,
    pub b0: f32,
    pub b1: f32,
    pub c0: f32,
    pub c1: f32,
    pub l0: f32,
    pub l1: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LuenbergerState {
    pub x0: f32,
    pub x1: f32,
    pub y_hat: f32,
    pub error: f32,
}

impl LuenbergerState {
    pub const fn new(x0: f32, x1: f32) -> Self {
        Self {
            x0,
            x1,
            y_hat: 0.0,
            error: 0.0,
        }
    }

    pub fn reset(&mut self, x0: f32, x1: f32) {
        *self = Self::new(x0, x1);
    }

    pub fn update(&mut self, param: &LuenbergerParam, input_u: f32, measured_y: f32) -> f32 {
        self.y_hat = param.c0 * self.x0 + param.c1 * self.x1;
        self.error = measured_y - self.y_hat;
        let next_x0 =
            param.a00 * self.x0 + param.a01 * self.x1 + param.b0 * input_u + param.l0 * self.error;
        let next_x1 =
            param.a10 * self.x0 + param.a11 * self.x1 + param.b1 * input_u + param.l1 * self.error;
        self.x0 = next_x0;
        self.x1 = next_x1;
        self.y_hat = param.c0 * self.x0 + param.c1 * self.x1;
        self.y_hat
    }
}

pub type EkfStateFunction = extern "C" fn(x: f32, u: f32) -> f32;
pub type EkfMeasureFunction = extern "C" fn(x: f32) -> f32;

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct EkfParam {
    pub f: Option<EkfStateFunction>,
    pub df_dx: Option<EkfStateFunction>,
    pub h: Option<EkfMeasureFunction>,
    pub dh_dx: Option<EkfMeasureFunction>,
    pub q: f32,
    pub r: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EkfState {
    pub x: f32,
    pub p: f32,
    pub k: f32,
    pub residual: f32,
}

impl EkfState {
    pub const fn new(initial_x: f32, initial_p: f32) -> Self {
        Self {
            x: initial_x,
            p: initial_p,
            k: 0.0,
            residual: 0.0,
        }
    }

    pub fn reset(&mut self, x: f32, p: f32) {
        *self = Self::new(x, p);
    }

    pub fn update(&mut self, param: &EkfParam, u: f32, measurement: f32) -> f32 {
        let (Some(f), Some(df_dx), Some(h), Some(dh_dx)) =
            (param.f, param.df_dx, param.h, param.dh_dx)
        else {
            return 0.0;
        };
        let x_pred = f(self.x, u);
        let f_jac = df_dx(self.x, u);
        let p_pred = f_jac * self.p * f_jac + param.q;
        let h_value = h(x_pred);
        let h_jac = dh_dx(x_pred);
        let mut s = h_jac * p_pred * h_jac + param.r;
        if s == 0.0 {
            s = 1.0;
        }
        self.k = p_pred * h_jac / s;
        self.residual = measurement - h_value;
        self.x = x_pred + self.k * self.residual;
        self.p = (1.0 - self.k * h_jac) * p_pred;
        self.x
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct EkfFocParam {
    pub ekf: EkfParam,
    pub foc: FocBasicParam,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EkfFocInput {
    pub ekf_u: f32,
    pub theta_measurement_rad: f32,
    pub foc: FocBasicInput,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EkfFocState {
    pub ekf: EkfState,
    pub foc: FocBasicState,
    pub theta_rad: f32,
    pub pwm: SvpwmOutput,
}

impl Default for EkfFocState {
    fn default() -> Self {
        Self {
            ekf: EkfState::default(),
            foc: FocBasicState::default(),
            theta_rad: 0.0,
            pwm: SvpwmOutput::default(),
        }
    }
}

impl EkfFocState {
    pub fn new(initial_theta_rad: f32, initial_p: f32) -> Self {
        let mut state = Self::default();
        state.reset(initial_theta_rad, initial_p);
        state
    }

    pub fn reset(&mut self, theta_rad: f32, p: f32) {
        self.ekf.reset(theta_rad, p);
        self.foc.reset();
        self.theta_rad = wrap_angle_0_to_2pi(theta_rad);
        self.pwm = SvpwmOutput::default();
    }

    pub fn update(&mut self, param: &EkfFocParam, input: &EkfFocInput) -> SvpwmOutput {
        self.theta_rad = wrap_angle_0_to_2pi(self.ekf.update(
            &param.ekf,
            input.ekf_u,
            input.theta_measurement_rad,
        ));
        let mut foc_input = input.foc;
        foc_input.theta_e_rad = self.theta_rad;
        self.pwm = self.foc.update(&param.foc, &foc_input);
        self.pwm
    }
}

pub type UkfStateFunction = extern "C" fn(x: f32, u: f32) -> f32;
pub type UkfMeasureFunction = extern "C" fn(x: f32) -> f32;

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct UkfParam {
    pub f: Option<UkfStateFunction>,
    pub h: Option<UkfMeasureFunction>,
    pub q: f32,
    pub r: f32,
    pub alpha: f32,
    pub beta: f32,
    pub kappa: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UkfState {
    pub x: f32,
    pub p: f32,
    pub k: f32,
    pub residual: f32,
}

impl UkfState {
    pub const fn new(initial_x: f32, initial_p: f32) -> Self {
        Self {
            x: initial_x,
            p: initial_p,
            k: 0.0,
            residual: 0.0,
        }
    }

    pub fn reset(&mut self, x: f32, p: f32) {
        *self = Self::new(x, p);
    }

    pub fn update(&mut self, param: &UkfParam, u: f32, measurement: f32) -> f32 {
        let (Some(f), Some(h)) = (param.f, param.h) else {
            return 0.0;
        };
        let alpha = if param.alpha <= 0.0 {
            0.001
        } else {
            param.alpha
        };
        let lambda = alpha * alpha * (1.0 + param.kappa) - 1.0;
        let mut scale = 1.0 + lambda;
        if scale <= 0.0 {
            scale = 0.001;
        }
        let sqrt_scale_p = libm::sqrtf(scale * self.p);
        let sigma = [self.x, self.x + sqrt_scale_p, self.x - sqrt_scale_p];
        let sigma_pred = [f(sigma[0], u), f(sigma[1], u), f(sigma[2], u)];
        let wm0 = lambda / scale;
        let wc0 = wm0 + (1.0 - alpha * alpha + param.beta);
        let wi = 0.5 / scale;
        let x_pred = wm0 * sigma_pred[0] + wi * sigma_pred[1] + wi * sigma_pred[2];
        let mut p_pred = param.q;
        p_pred += wc0 * (sigma_pred[0] - x_pred) * (sigma_pred[0] - x_pred);
        p_pred += wi * (sigma_pred[1] - x_pred) * (sigma_pred[1] - x_pred);
        p_pred += wi * (sigma_pred[2] - x_pred) * (sigma_pred[2] - x_pred);
        let z_sigma = [h(sigma_pred[0]), h(sigma_pred[1]), h(sigma_pred[2])];
        let z_pred = wm0 * z_sigma[0] + wi * z_sigma[1] + wi * z_sigma[2];
        let mut pz = param.r;
        pz += wc0 * (z_sigma[0] - z_pred) * (z_sigma[0] - z_pred);
        pz += wi * (z_sigma[1] - z_pred) * (z_sigma[1] - z_pred);
        pz += wi * (z_sigma[2] - z_pred) * (z_sigma[2] - z_pred);
        let mut pxz = wc0 * (sigma_pred[0] - x_pred) * (z_sigma[0] - z_pred);
        pxz += wi * (sigma_pred[1] - x_pred) * (z_sigma[1] - z_pred);
        pxz += wi * (sigma_pred[2] - x_pred) * (z_sigma[2] - z_pred);
        if pz == 0.0 {
            pz = 1.0;
        }
        self.k = pxz / pz;
        self.residual = measurement - z_pred;
        self.x = x_pred + self.k * self.residual;
        self.p = p_pred - self.k * pz * self.k;
        self.x
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controller::PiParam;
    use crate::modulation::SvpwmParam;

    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    extern "C" fn add_input(x: f32, u: f32) -> f32 {
        x + u
    }

    extern "C" fn state_jacobian(_x: f32, _u: f32) -> f32 {
        1.0
    }

    extern "C" fn square_measurement(x: f32) -> f32 {
        x * x
    }

    extern "C" fn square_jacobian(x: f32) -> f32 {
        2.0 * x
    }

    extern "C" fn identity_measurement(x: f32) -> f32 {
        x
    }

    extern "C" fn identity_jacobian(_x: f32) -> f32 {
        1.0
    }

    #[test]
    fn kalman_matches_c_reference() {
        let param = KalmanParam {
            a: 1.0,
            b: 1.0,
            h: 1.0,
            q: 1.0,
            r: 1.0,
        };
        let mut state = KalmanState::new(0.0, 1.0);
        near(state.update(&param, 1.0, 2.0), 1.666_666_7, 1e-5);
        near(state.k, 0.666_666_7, 1e-5);
        near(state.p, 0.666_666_7, 1e-5);
    }

    #[test]
    fn luenberger_matches_c_reference() {
        let param = LuenbergerParam {
            a00: 1.0,
            a01: 0.1,
            a10: 0.0,
            a11: 1.0,
            b0: 0.0,
            b1: 0.1,
            c0: 1.0,
            c1: 0.0,
            l0: 0.5,
            l1: 0.2,
        };
        let mut state = LuenbergerState::default();
        near(state.update(&param, 1.0, 2.0), 1.0, 1e-6);
        near(state.x0, 1.0, 1e-6);
        near(state.x1, 0.5, 1e-6);
    }

    #[test]
    fn ekf_matches_c_reference() {
        let param = EkfParam {
            f: Some(add_input),
            df_dx: Some(state_jacobian),
            h: Some(square_measurement),
            dh_dx: Some(square_jacobian),
            q: 0.1,
            r: 0.5,
        };
        let mut state = EkfState::new(1.0, 1.0);
        near(state.update(&param, 1.0, 5.0), 2.243_094, 1e-5);
        near(state.k, 0.243_093_9, 1e-5);
        near(state.residual, 1.0, 1e-6);
    }

    #[test]
    fn ekf_rejects_missing_model_without_state_change() {
        let mut state = EkfState::new(2.0, 3.0);
        assert_eq!(state.update(&EkfParam::default(), 1.0, 4.0), 0.0);
        assert_eq!(state, EkfState::new(2.0, 3.0));
    }

    #[test]
    fn ekf_foc_matches_c_reference() {
        let pi = PiParam {
            kp: 1.0,
            ki: 0.0,
            ts: 0.001,
            out_min: -12.0,
            out_max: 12.0,
            integrator_min: -6.0,
            integrator_max: 6.0,
        };
        let param = EkfFocParam {
            ekf: EkfParam {
                f: Some(add_input),
                df_dx: Some(state_jacobian),
                h: Some(identity_measurement),
                dh_dx: Some(identity_jacobian),
                q: 0.0,
                r: 0.01,
            },
            foc: FocBasicParam {
                id_pi: pi,
                iq_pi: pi,
                svpwm: SvpwmParam { v_bus: 24.0 },
            },
        };
        let input = EkfFocInput {
            ekf_u: 0.0,
            theta_measurement_rad: 0.5,
            foc: FocBasicInput {
                ia: 0.0,
                ib: 0.0,
                ic: 0.0,
                id_ref: 0.0,
                iq_ref: 2.0,
                theta_e_rad: 0.0,
            },
        };
        let mut state = EkfFocState::new(0.0, 1.0);
        let output = state.update(&param, &input);
        assert!((0.49..0.51).contains(&state.theta_rad));
        near(state.foc.voltage_dq.q, 2.0, 1e-6);
        assert!((0.0..=1.0).contains(&output.duty_a));
        assert!((0.0..=1.0).contains(&output.duty_b));
        assert!((0.0..=1.0).contains(&output.duty_c));
    }

    #[test]
    fn ukf_matches_c_reference() {
        let param = UkfParam {
            f: Some(add_input),
            h: Some(identity_measurement),
            q: 1.0,
            r: 1.0,
            alpha: 1.0,
            beta: 2.0,
            kappa: 0.0,
        };
        let mut state = UkfState::new(0.0, 1.0);
        near(state.update(&param, 1.0, 2.0), 1.5, 1e-5);
        near(state.k, 0.5, 1e-5);
        near(state.p, 1.5, 1e-5);
    }

    #[test]
    fn estimator_state_sizes_are_fixed() {
        assert_eq!(core::mem::size_of::<KalmanState>(), 16);
        assert_eq!(core::mem::size_of::<EkfState>(), 16);
        assert_eq!(core::mem::size_of::<UkfState>(), 16);
    }
}
