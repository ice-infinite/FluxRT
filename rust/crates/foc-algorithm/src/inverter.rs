//! 逆变器平均电压损失的纯数学模型。
//! Pure average-value inverter voltage-loss model.
//!
//! 本模块只描述“指令相电压到实际平均相电压”之间的一阶误差：
//! 死区占比造成的压降，加上一个可选的等效器件导通压降。它不包含
//! PWM 开关纹波、MOSFET/二极管非线性、母线纹波或 ADC 量化。
//!
//! 公式的量纲为：
//!
//! `loss_phase_v = (2 * dead_time_s / pwm_period_s * vbus + device_drop_v) * polarity(i)`
//!
//! `polarity(i)` 在 `current_zero_band_a` 内线性过渡，带外饱和到 `[-1, 1]`。
//! 滤波和状态属于上层 `foc-control`；这里保持无状态、`no_std`和无 HAL。

use crate::Abc;

/// 逆变器损失模型的纯数学参数。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverterLossParameters {
    /// 单次换流死区 `[s]`。
    pub dead_time_s: f32,
    /// PWM 载波周期 `[s]`，不是控制分频后的周期。
    pub pwm_period_s: f32,
    /// 等效的单相器件导通压降 `[V]`。
    pub device_drop_v: f32,
    /// 电流极性的软零带 `[A]`；0 表示硬符号。
    pub current_zero_band_a: f32,
}

impl Default for InverterLossParameters {
    fn default() -> Self {
        Self {
            dead_time_s: 0.0,
            pwm_period_s: 1.0 / 12_000.0,
            device_drop_v: 0.0,
            current_zero_band_a: 0.0,
        }
    }
}

impl InverterLossParameters {
    /// 校验数学参数。上限只是防止单位误填，不是硬件额定值。
    #[inline]
    pub fn is_valid(self) -> bool {
        self.dead_time_s.is_finite()
            && (0.0..=0.01).contains(&self.dead_time_s)
            && self.pwm_period_s.is_finite()
            && self.pwm_period_s > 0.0
            && self.pwm_period_s <= 1.0
            && self.device_drop_v.is_finite()
            && (0.0..=100.0).contains(&self.device_drop_v)
            && self.current_zero_band_a.is_finite()
            && (0.0..=10.0).contains(&self.current_zero_band_a)
    }
}

/// 电流极性系数 `[-1, 1]`：零带内线性，带外饱和。
#[inline]
pub fn inverter_current_polarity(current_a: f32, current_zero_band_a: f32) -> f32 {
    if current_zero_band_a > 0.0 {
        (current_a / current_zero_band_a).clamp(-1.0, 1.0)
    } else if current_a > 0.0 {
        1.0
    } else if current_a < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// 死区部分的单相平均压降幅值 `[V]`，不含电流符号。
#[inline]
pub fn inverter_dead_time_voltage_loss_v(
    parameters: InverterLossParameters,
    dc_bus_voltage_v: f32,
) -> f32 {
    2.0 * parameters.dead_time_s / parameters.pwm_period_s * dc_bus_voltage_v
}

/// 返回三相“应从指令电压中减去”的平均压降 `[V]`。
///
/// 输入电流应由上层决定是否先滤波。返回值已包含极性；调用方对
/// 观测器电压做减法，对 PWM 前馈做加法。
#[inline]
pub fn average_inverter_phase_voltage_loss_v(
    parameters: InverterLossParameters,
    dc_bus_voltage_v: f32,
    phase_currents_a: Abc,
) -> Abc {
    let magnitude =
        inverter_dead_time_voltage_loss_v(parameters, dc_bus_voltage_v) + parameters.device_drop_v;
    Abc {
        a: magnitude
            * inverter_current_polarity(phase_currents_a.a, parameters.current_zero_band_a),
        b: magnitude
            * inverter_current_polarity(phase_currents_a.b, parameters.current_zero_band_a),
        c: magnitude
            * inverter_current_polarity(phase_currents_a.c, parameters.current_zero_band_a),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polarity_has_soft_zero_band_and_hard_sign_fallback() {
        assert_eq!(inverter_current_polarity(0.0, 0.005), 0.0);
        assert!((inverter_current_polarity(0.0025, 0.005) - 0.5).abs() < f32::EPSILON);
        assert_eq!(inverter_current_polarity(-0.02, 0.005), -1.0);
        assert_eq!(inverter_current_polarity(0.02, 0.0), 1.0);
        assert_eq!(inverter_current_polarity(-0.02, 0.0), -1.0);
    }

    #[test]
    fn dead_time_and_device_drop_add_with_current_polarity() {
        let parameters = InverterLossParameters {
            dead_time_s: 550.0e-9,
            pwm_period_s: 1.0 / 12_000.0,
            device_drop_v: 0.08,
            current_zero_band_a: 0.01,
        };
        let loss = average_inverter_phase_voltage_loss_v(
            parameters,
            12.3,
            Abc {
                a: 0.02,
                b: -0.005,
                c: 0.0,
            },
        );
        let magnitude = 2.0 * 550.0e-9 * 12_000.0 * 12.3 + 0.08;
        assert!((loss.a - magnitude).abs() < 1.0e-6);
        assert!((loss.b + 0.5 * magnitude).abs() < 1.0e-6);
        assert_eq!(loss.c, 0.0);
    }

    #[test]
    fn parameter_validation_rejects_unit_and_range_errors() {
        assert!(InverterLossParameters::default().is_valid());
        assert!(!InverterLossParameters {
            pwm_period_s: 0.0,
            ..InverterLossParameters::default()
        }
        .is_valid());
        assert!(!InverterLossParameters {
            current_zero_band_a: f32::NAN,
            ..InverterLossParameters::default()
        }
        .is_valid());
        assert!(!InverterLossParameters {
            device_drop_v: -0.1,
            ..InverterLossParameters::default()
        }
        .is_valid());
    }
}
