//! Filter_LowPass 的等价实现。

use crate::math::clamp;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LowPassParam {
    pub alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LowPassState {
    pub output: f32,
    pub initialized: i32,
}

impl LowPassState {
    #[inline]
    pub const fn new(initial_output: f32) -> Self {
        Self {
            output: initial_output,
            initialized: 1,
        }
    }

    #[inline]
    pub fn reset(&mut self, output: f32) {
        self.output = output;
        self.initialized = 1;
    }

    #[inline]
    pub fn update(&mut self, param: &LowPassParam, input: f32) -> f32 {
        if self.initialized == 0 {
            self.reset(input);
            return self.output;
        }
        let alpha = clamp(param.alpha, 0.0, 1.0);
        self.output += alpha * (input - self.output);
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HighPassParam {
    pub alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HighPassState {
    pub last_input: f32,
    pub output: f32,
    pub initialized: i32,
}

impl HighPassState {
    pub const fn new(initial_input: f32) -> Self {
        Self {
            last_input: initial_input,
            output: 0.0,
            initialized: 1,
        }
    }

    pub fn reset(&mut self, input: f32) {
        *self = Self::new(input);
    }

    pub fn update(&mut self, param: &HighPassParam, input: f32) -> f32 {
        if self.initialized == 0 {
            self.reset(input);
            return self.output;
        }
        let alpha = clamp(param.alpha, 0.0, 1.0);
        self.output = alpha * (self.output + input - self.last_input);
        self.last_input = input;
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DifferentiatorParam {
    pub ts: f32,
    pub alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DifferentiatorState {
    pub last_input: f32,
    pub output: f32,
    pub initialized: i32,
}

impl DifferentiatorState {
    pub const fn new(initial_input: f32) -> Self {
        Self {
            last_input: initial_input,
            output: 0.0,
            initialized: 1,
        }
    }

    pub fn reset(&mut self, input: f32) {
        *self = Self::new(input);
    }

    pub fn update(&mut self, param: &DifferentiatorParam, input: f32) -> f32 {
        if param.ts <= 0.0 {
            return 0.0;
        }
        if self.initialized == 0 {
            self.reset(input);
            return self.output;
        }
        let alpha = clamp(param.alpha, 0.0, 1.0);
        let raw = (input - self.last_input) / param.ts;
        self.output += alpha * (raw - self.output);
        self.last_input = input;
        self.output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_c_reference_vectors() {
        let mut p = LowPassParam { alpha: 0.25 };
        let mut s = LowPassState::new(0.0);
        assert!((s.update(&p, 1.0) - 0.25).abs() <= 1e-6);
        assert!((s.update(&p, 1.0) - 0.4375).abs() <= 1e-6);
        p.alpha = 2.0;
        assert!((s.update(&p, 1.0) - 1.0).abs() <= 1e-6);
    }

    #[test]
    fn default_state_initializes_from_first_sample() {
        let mut s = LowPassState::default();
        let p = LowPassParam { alpha: 0.1 };
        assert_eq!(s.update(&p, 3.0), 3.0);
    }

    #[test]
    fn digital_filters_match_c_reference() {
        let hp_param = HighPassParam { alpha: 0.5 };
        let mut hp = HighPassState::new(0.0);
        assert!((hp.update(&hp_param, 1.0) - 0.5).abs() <= 1e-6);
        assert!((hp.update(&hp_param, 1.0) - 0.25).abs() <= 1e-6);

        let diff_param = DifferentiatorParam {
            ts: 0.1,
            alpha: 0.5,
        };
        let mut diff = DifferentiatorState::new(0.0);
        assert!((diff.update(&diff_param, 1.0) - 5.0).abs() <= 1e-6);
        assert!((diff.update(&diff_param, 1.0) - 2.5).abs() <= 1e-6);
    }
}
