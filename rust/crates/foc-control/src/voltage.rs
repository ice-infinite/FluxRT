//! FluxRT —— 观察器电压来源的统一组合入口。
//! FluxRT - central composition point for observer voltage sources.
//!
//! 当前目标固件只批准 [`ObserverVoltageSource::CommandModel`]：用上一拍 PWM 命令和
//! 实测母线电压重构 αβ 电压。这与 ST MCSDK 默认 STO 数据流一致，且不会把尚未逐板
//! 标定的 TP6/TP7/TP8 原始码送进闭环。`PhaseVoltage` 与 `Hybrid` 只保留稳定枚举和
//! 能力查询；在质量门、物理量 ABI 与失效回退完成前，它们明确返回“不支持”。
//!
//! The target currently approves only `CommandModel`, reconstructed from the
//! previous PWM command and measured DC bus.  The measured and hybrid variants
//! reserve the switching contract but remain explicitly unavailable until the
//! calibration, quality, ABI and fallback gates are implemented.

use foc_algorithm::{
    average_inverter_phase_voltage_loss_v, Abc, AlphaBeta, InverterLossParameters,
};

use crate::{pwm_to_alpha_beta, PhaseCurrents, PwmCommand};

/// 观察器所用 αβ 电压的来源。数值固定，未来跨 ABI 暴露时不得重排。
#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObserverVoltageSource {
    /// 上一拍 PWM 命令 + 实测 Vbus；当前唯一批准的目标路径。
    CommandModel = 0,
    /// 已标定并通过质量门的三相端电压；尚未实现。
    PhaseVoltage = 1,
    /// CommandModel 与 PhaseVoltage 的融合及失效回退；尚未实现。
    Hybrid = 2,
}

/// 官方参考兼容的默认来源。改动它必须经过 ABI、仿真和实机门，不能静默切换。
pub const DEFAULT_OBSERVER_VOLTAGE_SOURCE: ObserverVoltageSource =
    ObserverVoltageSource::CommandModel;

/// 可在 Host 与 Cortex-M 共同编译的逆变器电压模型配置。
///
/// 默认值明确关闭模型和两个补偿出口，因此不改变现有目标机数值路径。
/// `pwm_period_s` 始终是载波周期，即使控制环分频也不得换成控制周期。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverterVoltageModelConfig {
    /// 总开关；关闭时观测器与 PWM 都保持原值。
    pub enabled: bool,
    /// 单次换流死区 `[s]`。
    pub dead_time_s: f32,
    /// PWM 载波周期 `[s]`。
    pub pwm_period_s: f32,
    /// PWM 前馈比例 `[-]`；观测器修正始终使用物理损失全量。
    pub compensation_gain: f32,
    /// 电流极性线性过零带 `[A]`。
    pub current_zero_band_a: f32,
    /// 相电流一阶滤波系数 `(0, 1]`；1 表示不滤波。
    pub current_sign_filter_alpha: f32,
    /// 等效单相器件导通压降 `[V]`。
    pub device_drop_v: f32,
    /// 是否修正观测器的 CommandModel 电压。
    pub observer_voltage_correction_enabled: bool,
    /// 是否向输出 PWM 叠加损失前馈。
    pub feedforward_enabled: bool,
    /// 电压来源；CM3 仍只批准 `CommandModel`。
    pub source: ObserverVoltageSource,
}

impl Default for InverterVoltageModelConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            dead_time_s: 0.0,
            pwm_period_s: 1.0 / 12_000.0,
            compensation_gain: 0.0,
            current_zero_band_a: 0.0,
            current_sign_filter_alpha: 1.0,
            device_drop_v: 0.0,
            observer_voltage_correction_enabled: false,
            feedforward_enabled: false,
            source: DEFAULT_OBSERVER_VOLTAGE_SOURCE,
        }
    }
}

impl InverterVoltageModelConfig {
    /// 返回纯数学层所需的损失参数。
    #[inline]
    pub fn loss_parameters(self) -> InverterLossParameters {
        InverterLossParameters {
            dead_time_s: self.dead_time_s,
            pwm_period_s: self.pwm_period_s,
            device_drop_v: self.device_drop_v,
            current_zero_band_a: self.current_zero_band_a,
        }
    }

    /// 整组校验；任一字段错误都拒绝，不允许部分生效。
    pub fn is_valid(self) -> bool {
        self.loss_parameters().is_valid()
            && self.compensation_gain.is_finite()
            && (0.0..=4.0).contains(&self.compensation_gain)
            && self.current_sign_filter_alpha.is_finite()
            && (0.0001..=1.0).contains(&self.current_sign_filter_alpha)
            && self.source == ObserverVoltageSource::CommandModel
            && (self.enabled
                || (!self.observer_voltage_correction_enabled && !self.feedforward_enabled))
    }
}

/// 逆变器电压模型的实时状态。
///
/// 三相使用同一个一阶系数；对平衡三线制电流，这与“先 Clarke、
/// 在 alpha/beta 滤波、再逆 Clarke”数学等价，但不额外增加两次变换的快环开销。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InverterVoltageModel {
    config: InverterVoltageModelConfig,
    filtered_phase_currents_a: Abc,
    current_filter_initialized: bool,
}

impl InverterVoltageModel {
    /// 构造经校验的模型；非法配置不会留下部分状态。
    pub fn try_new(config: InverterVoltageModelConfig) -> Option<Self> {
        config.is_valid().then_some(Self {
            config,
            filtered_phase_currents_a: Abc::default(),
            current_filter_initialized: false,
        })
    }

    /// 从调用方已经整组校验过的配置构造模型。
    /// Builds a model from a configuration already validated by the caller.
    ///
    /// 该入口供版本化 bridge 使用，避免目标固件在“ABI 校验”和“模型构造”中保留
    /// 两份相同的范围检查代码。Debug/Host 仍用断言捕获两边规则漂移；Release 目标
    /// 不为这个重复检查付出 Flash/WCET。
    #[inline]
    pub fn from_validated_config(config: InverterVoltageModelConfig) -> Self {
        debug_assert!(config.is_valid());
        Self {
            config,
            filtered_phase_currents_a: Abc::default(),
            current_filter_initialized: false,
        }
    }

    /// 返回当前配置的副本。
    #[inline]
    pub fn config(&self) -> InverterVoltageModelConfig {
        self.config
    }

    /// 清除电流滤波状态，不改变配置。
    #[inline]
    pub fn reset(&mut self) {
        self.filtered_phase_currents_a = Abc::default();
        self.current_filter_initialized = false;
    }

    /// 每个控制拍只调用一次，更新用于极性判断的滤波电流。
    pub fn update_phase_currents(&mut self, currents: PhaseCurrents) -> Abc {
        let sample = Abc {
            a: currents.a,
            b: currents.b,
            c: currents.c,
        };
        if self.current_filter_initialized {
            let alpha = self.config.current_sign_filter_alpha;
            self.filtered_phase_currents_a.a +=
                alpha * (sample.a - self.filtered_phase_currents_a.a);
            self.filtered_phase_currents_a.b +=
                alpha * (sample.b - self.filtered_phase_currents_a.b);
            self.filtered_phase_currents_a.c +=
                alpha * (sample.c - self.filtered_phase_currents_a.c);
        } else {
            self.filtered_phase_currents_a = sample;
            self.current_filter_initialized = true;
        }
        self.filtered_phase_currents_a
    }

    /// 返回当前滤波三相电流 `[A]`。
    #[inline]
    pub fn filtered_phase_currents(&self) -> Abc {
        self.filtered_phase_currents_a
    }

    /// 返回当前死区+器件压降的三相损失 `[V]`。
    pub fn phase_voltage_loss_v(&self, dc_bus_voltage_v: f32) -> Abc {
        if !self.config.enabled {
            return Abc::default();
        }
        average_inverter_phase_voltage_loss_v(
            self.config.loss_parameters(),
            dc_bus_voltage_v,
            self.filtered_phase_currents_a,
        )
    }

    /// 把物理损失换算为观测器使用的等效占空比，不限幅也不回写。
    pub fn observer_equivalent_pwm(&self, pwm: PwmCommand, dc_bus_voltage_v: f32) -> PwmCommand {
        if !self.config.enabled
            || !self.config.observer_voltage_correction_enabled
            || !dc_bus_voltage_v.is_finite()
            || dc_bus_voltage_v <= 0.0
        {
            return pwm;
        }
        let loss = self.phase_voltage_loss_v(dc_bus_voltage_v);
        PwmCommand {
            duty_a: pwm.duty_a - loss.a / dc_bus_voltage_v,
            duty_b: pwm.duty_b - loss.b / dc_bus_voltage_v,
            duty_c: pwm.duty_c - loss.c / dc_bus_voltage_v,
        }
    }

    /// 向实际 PWM 叠加损失前馈并钳位到 `[0,1]`。
    pub fn feedforward_pwm(&self, pwm: PwmCommand, dc_bus_voltage_v: f32) -> PwmCommand {
        if !self.config.enabled
            || !self.config.feedforward_enabled
            || !dc_bus_voltage_v.is_finite()
            || dc_bus_voltage_v <= 0.0
        {
            return pwm;
        }
        let loss = self.phase_voltage_loss_v(dc_bus_voltage_v);
        let gain_over_bus = self.config.compensation_gain / dc_bus_voltage_v;
        PwmCommand {
            duty_a: (pwm.duty_a + gain_over_bus * loss.a).clamp(0.0, 1.0),
            duty_b: (pwm.duty_b + gain_over_bus * loss.b).clamp(0.0, 1.0),
            duty_c: (pwm.duty_c + gain_over_bus * loss.c).clamp(0.0, 1.0),
        }
    }
}

/// 一拍观察器电压估计；source 与电压一起返回，避免遥测以后猜测数据来源。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ObserverVoltageEstimate {
    pub alpha_beta: AlphaBeta,
    pub source: ObserverVoltageSource,
}

/// 当前目标是否已经实现并批准给观察器使用。
#[inline]
pub const fn observer_voltage_source_available(source: ObserverVoltageSource) -> bool {
    matches!(source, ObserverVoltageSource::CommandModel)
}

///
/// 用上一拍 PWM 与实测母线电压重构观察器电压，保持原控制路径的数值语义不变。
/// The numerical path is intentionally the same as the former direct
/// `pwm_to_alpha_beta` calls; this function centralises policy without changing
/// the control law.
#[inline]
pub fn command_model_observer_voltage(
    previous_pwm: PwmCommand,
    dc_bus_voltage: f32,
) -> ObserverVoltageEstimate {
    ObserverVoltageEstimate {
        alpha_beta: pwm_to_alpha_beta(previous_pwm, dc_bus_voltage),
        source: ObserverVoltageSource::CommandModel,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn official_compatible_default_is_command_model_only() {
        assert_eq!(
            DEFAULT_OBSERVER_VOLTAGE_SOURCE,
            ObserverVoltageSource::CommandModel
        );
        assert!(observer_voltage_source_available(
            ObserverVoltageSource::CommandModel
        ));
        assert!(!observer_voltage_source_available(
            ObserverVoltageSource::PhaseVoltage
        ));
        assert!(!observer_voltage_source_available(
            ObserverVoltageSource::Hybrid
        ));
    }

    #[test]
    fn command_model_keeps_existing_voltage_reconstruction() {
        let pwm = PwmCommand {
            duty_a: 0.8,
            duty_b: 0.3,
            duty_c: 0.4,
        };
        let expected = pwm_to_alpha_beta(pwm, 12.3);
        let estimate = command_model_observer_voltage(pwm, 12.3);

        assert_eq!(estimate.source, ObserverVoltageSource::CommandModel);
        assert_eq!(estimate.alpha_beta, expected);
    }

    #[test]
    fn disabled_model_is_numerically_transparent() {
        let mut model = InverterVoltageModel::default();
        let pwm = PwmCommand {
            duty_a: 0.8,
            duty_b: 0.3,
            duty_c: 0.4,
        };
        model.update_phase_currents(PhaseCurrents {
            a: 0.2,
            b: -0.1,
            c: -0.1,
        });
        assert_eq!(model.observer_equivalent_pwm(pwm, 12.3), pwm);
        assert_eq!(model.feedforward_pwm(pwm, 12.3), pwm);
        assert_eq!(model.phase_voltage_loss_v(12.3), Abc::default());
    }

    #[test]
    fn legacy_dead_time_path_is_preserved_with_unity_filter_and_zero_device_drop() {
        let config = InverterVoltageModelConfig {
            enabled: true,
            dead_time_s: 550.0e-9,
            pwm_period_s: 1.0 / 12_000.0,
            compensation_gain: 0.75,
            current_zero_band_a: 0.005,
            current_sign_filter_alpha: 1.0,
            device_drop_v: 0.0,
            observer_voltage_correction_enabled: true,
            feedforward_enabled: true,
            source: ObserverVoltageSource::CommandModel,
        };
        let mut model = InverterVoltageModel::try_new(config).unwrap();
        let pwm = PwmCommand {
            duty_a: 0.6,
            duty_b: 0.4,
            duty_c: 0.5,
        };
        model.update_phase_currents(PhaseCurrents {
            a: 0.010,
            b: -0.0025,
            c: 0.0,
        });
        let dead_time_duty = 2.0 * 550.0e-9 / (1.0 / 12_000.0);
        let observer = model.observer_equivalent_pwm(pwm, 12.3);
        let feedforward = model.feedforward_pwm(pwm, 12.3);
        assert!((observer.duty_a - (pwm.duty_a - dead_time_duty)).abs() < 1.0e-7);
        assert!((observer.duty_b - (pwm.duty_b + 0.5 * dead_time_duty)).abs() < 1.0e-7);
        assert_eq!(observer.duty_c, pwm.duty_c);
        assert!((feedforward.duty_a - (pwm.duty_a + 0.75 * dead_time_duty)).abs() < 1.0e-7);
    }

    #[test]
    fn current_filter_and_device_drop_are_applied_before_polarity() {
        let config = InverterVoltageModelConfig {
            enabled: true,
            dead_time_s: 0.0,
            pwm_period_s: 1.0 / 12_000.0,
            compensation_gain: 1.0,
            current_zero_band_a: 0.01,
            current_sign_filter_alpha: 0.25,
            device_drop_v: 0.08,
            observer_voltage_correction_enabled: true,
            feedforward_enabled: true,
            source: ObserverVoltageSource::CommandModel,
        };
        let mut model = InverterVoltageModel::try_new(config).unwrap();
        model.update_phase_currents(PhaseCurrents {
            a: 0.02,
            b: -0.02,
            c: 0.0,
        });
        let filtered = model.update_phase_currents(PhaseCurrents {
            a: -0.02,
            b: 0.02,
            c: 0.0,
        });
        assert!((filtered.a - 0.01).abs() < 1.0e-7);
        assert!((filtered.b + 0.01).abs() < 1.0e-7);
        let loss = model.phase_voltage_loss_v(12.3);
        assert!((loss.a - 0.08).abs() < 1.0e-7);
        assert!((loss.b + 0.08).abs() < 1.0e-7);
    }

    #[test]
    fn invalid_or_unapproved_config_is_rejected_as_a_group() {
        assert!(InverterVoltageModel::try_new(InverterVoltageModelConfig {
            enabled: false,
            feedforward_enabled: true,
            ..InverterVoltageModelConfig::default()
        })
        .is_none());
        assert!(InverterVoltageModel::try_new(InverterVoltageModelConfig {
            source: ObserverVoltageSource::Hybrid,
            ..InverterVoltageModelConfig::default()
        })
        .is_none());
        assert!(InverterVoltageModel::try_new(InverterVoltageModelConfig {
            current_sign_filter_alpha: 0.0,
            ..InverterVoltageModelConfig::default()
        })
        .is_none());
    }
}
