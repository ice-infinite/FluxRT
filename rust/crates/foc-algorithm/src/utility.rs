//! Utility_LimitRamp 的等价实现。

use crate::math::clamp;

#[inline]
pub fn limit(input: f32, min_value: f32, max_value: f32) -> f32 {
    clamp(input, min_value, max_value)
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RampParam {
    pub rate_up: f32,
    pub rate_down: f32,
    pub ts: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RampState {
    pub output: f32,
}

impl RampState {
    #[inline]
    pub const fn new(initial_output: f32) -> Self {
        Self {
            output: initial_output,
        }
    }

    #[inline]
    pub fn reset(&mut self, output: f32) {
        self.output = output;
    }

    #[inline]
    pub fn update(&mut self, param: &RampParam, target: f32) -> f32 {
        let mut delta = target - self.output;
        let max_up = param.rate_up * param.ts;
        let max_down = param.rate_down * param.ts;
        if delta > max_up {
            delta = max_up;
        } else if delta < -max_down {
            delta = -max_down;
        }
        self.output += delta;
        self.output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_c_reference_vectors() {
        let p = RampParam {
            rate_up: 10.0,
            rate_down: 20.0,
            ts: 0.1,
        };
        let mut s = RampState::new(0.0);
        assert!((s.update(&p, 5.0) - 1.0).abs() <= 1e-6);
        assert!((s.update(&p, -5.0) + 1.0).abs() <= 1e-6);
        assert!((limit(5.0, -2.0, 2.0) - 2.0).abs() <= 1e-6);
    }
}
