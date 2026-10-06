//! EXP-B3 Ls(I) 辨识的硬件无关 PWM 注入计划。
//!
//! 本模块只把应用状态机给出的 `OFF/BIAS/+PULSE/-PULSE` 请求转换为三相
//! 占空比计划；它不认识寄存器、HAL、RTOS，也不能使能功率级。C 平台层仍是
//! 唯一的 PWM 所有者，并且必须在写 CCR 前再次检查硬件故障、采集窗口和输出范围。
//!
//! 当前 S4.4 使用保守的 `Rs * I_bias + V_perturb` 电压前馈，不在这里闭合电流环，
//! 也不增加尚未通过板端验证的死区电压前馈。辨识侧会从实际 CCR、同步 Vbus 和
//! 相电流重构实际施加电压，因此无需把“命令电压”等同于“施加电压”。

/// EXP-B3 当前固定的控制/采样频率 `[Hz]`。
pub const LSI_ACTUATION_SAMPLE_RATE_HZ: u32 = 12_000;
/// CCR preload 写入到成为 active compare 的名义控制拍延迟。
pub const LSI_ACTUATION_DELAY_CONTROL_TICKS: u32 = 1;

const MAX_STATOR_RESISTANCE_OHM: f32 = 20.0;
const MAX_BIAS_CURRENT_A: f32 = 0.2;
const MAX_PERTURBATION_VOLTAGE_V: f32 = 0.4;
const MAX_CURRENT_TRIP_A: f32 = 1.15;
const MIN_BUS_VOLTAGE_V: f32 = 7.0;
const MAX_BUS_VOLTAGE_V: f32 = 18.0;
const MIN_DUTY_FLOOR: f32 = 0.03;
const MAX_DUTY_CEILING: f32 = 0.97;

/// 应用状态机允许发出的四种请求。数值与 C 侧 ABI 固定一致。
#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LsiDriveRequest {
    Off = 0,
    Bias = 1,
    PulsePositive = 2,
    PulseNegative = 3,
}

/// 注入计划的冻结安全包络。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LsiActuationConfig {
    pub sample_rate_hz: u32,
    pub actuation_delay_control_ticks: u32,
    pub stator_resistance_ohm: f32,
    pub maximum_bias_current_a: f32,
    pub maximum_perturbation_voltage_v: f32,
    pub current_trip_a: f32,
    pub minimum_bus_voltage_v: f32,
    pub maximum_bus_voltage_v: f32,
    pub minimum_duty: f32,
    pub maximum_duty: f32,
}

impl Default for LsiActuationConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: LSI_ACTUATION_SAMPLE_RATE_HZ,
            actuation_delay_control_ticks: LSI_ACTUATION_DELAY_CONTROL_TICKS,
            // 三组端子 DC 电阻扣除 0.1 ohm 表笔后折算的每相筛查值。
            // 它尚未获批为 Production 参数，只服务 EXP-B3 受限辨识计划。
            stator_resistance_ohm: 4.966_666_7,
            maximum_bias_current_a: MAX_BIAS_CURRENT_A,
            maximum_perturbation_voltage_v: MAX_PERTURBATION_VOLTAGE_V,
            current_trip_a: MAX_CURRENT_TRIP_A,
            minimum_bus_voltage_v: MIN_BUS_VOLTAGE_V,
            maximum_bus_voltage_v: MAX_BUS_VOLTAGE_V,
            minimum_duty: MIN_DUTY_FLOOR,
            maximum_duty: MAX_DUTY_CEILING,
        }
    }
}

impl LsiActuationConfig {
    /// 只接受不超过 EXP-B3 冻结上限的配置；非法值不会被静默钳位。
    pub fn is_valid(self) -> bool {
        self.sample_rate_hz == LSI_ACTUATION_SAMPLE_RATE_HZ
            && self.actuation_delay_control_ticks == LSI_ACTUATION_DELAY_CONTROL_TICKS
            && self.stator_resistance_ohm.is_finite()
            && self.stator_resistance_ohm > 0.0
            && self.stator_resistance_ohm <= MAX_STATOR_RESISTANCE_OHM
            && self.maximum_bias_current_a.is_finite()
            && self.maximum_bias_current_a > 0.0
            && self.maximum_bias_current_a <= MAX_BIAS_CURRENT_A
            && self.maximum_perturbation_voltage_v.is_finite()
            && self.maximum_perturbation_voltage_v > 0.0
            && self.maximum_perturbation_voltage_v <= MAX_PERTURBATION_VOLTAGE_V
            && self.current_trip_a.is_finite()
            && self.current_trip_a > self.maximum_bias_current_a
            && self.current_trip_a <= MAX_CURRENT_TRIP_A
            && self.minimum_bus_voltage_v.is_finite()
            && self.maximum_bus_voltage_v.is_finite()
            && self.minimum_bus_voltage_v >= MIN_BUS_VOLTAGE_V
            && self.maximum_bus_voltage_v <= MAX_BUS_VOLTAGE_V
            && self.maximum_bus_voltage_v > self.minimum_bus_voltage_v
            && self.minimum_duty.is_finite()
            && self.maximum_duty.is_finite()
            && self.minimum_duty >= MIN_DUTY_FLOOR
            && self.maximum_duty <= MAX_DUTY_CEILING
            && self.maximum_duty > self.minimum_duty
    }
}

/// 一拍计划输入。所有布尔门由 C 平台从真实硬件状态生成。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LsiActuationInput {
    pub drive_request: LsiDriveRequest,
    pub force_safe_output: bool,
    pub capture_ready: bool,
    pub capture_full: bool,
    pub hardware_fault: bool,
    pub software_trip: bool,
    pub control_tick: u32,
    pub requested_bias_current_a: f32,
    pub requested_perturbation_voltage_v: f32,
    pub phase_u_current_a: f32,
    pub bus_voltage_v: f32,
}

/// 计划失败的分类；C ABI 会把它映射到统一 `foc_status_t`。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LsiActuationError {
    InvalidConfig,
    InvalidInput,
    NotReady,
    Fault,
}

/// 一拍 PWM 计划。`safe_output_required=true` 时 duty 字段必须被忽略并关断输出。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LsiActuationPlan {
    pub safe_output_required: bool,
    pub drive_active: bool,
    pub source_control_tick: u32,
    pub expected_active_control_tick: u32,
    pub phase_u_voltage_command_v: f32,
    pub duty_u: f32,
    pub duty_v: f32,
    pub duty_w: f32,
}

impl LsiActuationPlan {
    #[inline]
    pub fn safe(control_tick: u32) -> Self {
        Self {
            safe_output_required: true,
            drive_active: false,
            source_control_tick: control_tick,
            expected_active_control_tick: control_tick,
            phase_u_voltage_command_v: 0.0,
            duty_u: 0.0,
            duty_v: 0.0,
            duty_w: 0.0,
        }
    }
}

#[inline]
fn duty_is_in_window(duty: f32, config: LsiActuationConfig) -> bool {
    duty.is_finite() && duty >= config.minimum_duty && duty <= config.maximum_duty
}

/// 将状态机请求变成一个**尚未施加到硬件**的三相占空比计划。
///
/// `OFF` 必须同时带 `force_safe_output=true` 且两个请求量为 0；它在母线断电、
/// capture 未就绪或 fault 已锁存时仍返回确定的安全计划。任何 active 请求则必须
/// 通过全部硬件/采集/电压/电流门，且不做越界钳位。
pub fn plan_lsi_actuation(
    config: LsiActuationConfig,
    input: LsiActuationInput,
) -> Result<LsiActuationPlan, LsiActuationError> {
    if !config.is_valid() {
        return Err(LsiActuationError::InvalidConfig);
    }
    if !input.requested_bias_current_a.is_finite()
        || !input.requested_perturbation_voltage_v.is_finite()
        || !input.phase_u_current_a.is_finite()
        || !input.bus_voltage_v.is_finite()
    {
        return Err(LsiActuationError::InvalidInput);
    }

    if input.drive_request == LsiDriveRequest::Off {
        if input.force_safe_output
            && input.requested_bias_current_a == 0.0
            && input.requested_perturbation_voltage_v == 0.0
        {
            return Ok(LsiActuationPlan::safe(input.control_tick));
        }
        return Err(LsiActuationError::InvalidInput);
    }

    if input.force_safe_output {
        return Err(LsiActuationError::InvalidInput);
    }
    if input.hardware_fault
        || input.software_trip
        || input.capture_full
        || input.phase_u_current_a.abs() >= config.current_trip_a
    {
        return Err(LsiActuationError::Fault);
    }
    if !input.capture_ready {
        return Err(LsiActuationError::NotReady);
    }
    if input.bus_voltage_v < config.minimum_bus_voltage_v
        || input.bus_voltage_v > config.maximum_bus_voltage_v
    {
        return Err(LsiActuationError::Fault);
    }
    if input.requested_bias_current_a <= 0.0
        || input.requested_bias_current_a > config.maximum_bias_current_a
        || input.requested_perturbation_voltage_v.abs() > config.maximum_perturbation_voltage_v
    {
        return Err(LsiActuationError::InvalidInput);
    }

    let perturbation_v = match input.drive_request {
        LsiDriveRequest::Off => unreachable!(),
        LsiDriveRequest::Bias if input.requested_perturbation_voltage_v == 0.0 => 0.0,
        LsiDriveRequest::PulsePositive if input.requested_perturbation_voltage_v > 0.0 => {
            input.requested_perturbation_voltage_v
        }
        LsiDriveRequest::PulseNegative if input.requested_perturbation_voltage_v < 0.0 => {
            input.requested_perturbation_voltage_v
        }
        _ => return Err(LsiActuationError::InvalidInput),
    };

    let phase_u_voltage_command_v =
        config.stator_resistance_ohm * input.requested_bias_current_a + perturbation_v;
    // 本实验固定维持正向 DC bias；负电压会改变辨识拓扑，因此拒绝而非钳位。
    if !phase_u_voltage_command_v.is_finite() || phase_u_voltage_command_v <= 0.0 {
        return Err(LsiActuationError::InvalidInput);
    }

    // 平衡的静止 alpha 轴相电压：U=+V, V=W=-V/2，三相和为 0。
    // duty 的公共模固定为 0.5，C 平台稍后只负责换算为 CCR。
    let normalized = phase_u_voltage_command_v / input.bus_voltage_v;
    let duty_u = 0.5 + normalized;
    let duty_v = 0.5 - 0.5 * normalized;
    let duty_w = duty_v;
    if !duty_is_in_window(duty_u, config)
        || !duty_is_in_window(duty_v, config)
        || !duty_is_in_window(duty_w, config)
    {
        return Err(LsiActuationError::InvalidInput);
    }

    Ok(LsiActuationPlan {
        safe_output_required: false,
        drive_active: true,
        source_control_tick: input.control_tick,
        expected_active_control_tick: input
            .control_tick
            .wrapping_add(config.actuation_delay_control_ticks),
        phase_u_voltage_command_v,
        duty_u,
        duty_v,
        duty_w,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn active_input(request: LsiDriveRequest, perturbation_v: f32) -> LsiActuationInput {
        LsiActuationInput {
            drive_request: request,
            force_safe_output: false,
            capture_ready: true,
            capture_full: false,
            hardware_fault: false,
            software_trip: false,
            control_tick: 41,
            requested_bias_current_a: 0.2,
            requested_perturbation_voltage_v: perturbation_v,
            phase_u_current_a: 0.19,
            bus_voltage_v: 12.3,
        }
    }

    #[test]
    fn off_is_safe_even_with_zero_bus_and_latched_faults() {
        let plan = plan_lsi_actuation(
            LsiActuationConfig::default(),
            LsiActuationInput {
                drive_request: LsiDriveRequest::Off,
                force_safe_output: true,
                capture_ready: false,
                capture_full: true,
                hardware_fault: true,
                software_trip: true,
                control_tick: 7,
                requested_bias_current_a: 0.0,
                requested_perturbation_voltage_v: 0.0,
                phase_u_current_a: 0.0,
                bus_voltage_v: 0.0,
            },
        )
        .unwrap();
        assert!(plan.safe_output_required);
        assert!(!plan.drive_active);
        assert_eq!(plan.source_control_tick, 7);
        assert_eq!(plan.expected_active_control_tick, 7);
        assert_eq!(plan.duty_u, 0.0);
    }

    #[test]
    fn bias_and_pulses_have_balanced_voltage_and_one_tick_ledger() {
        let config = LsiActuationConfig::default();
        let bias = plan_lsi_actuation(config, active_input(LsiDriveRequest::Bias, 0.0)).unwrap();
        let positive =
            plan_lsi_actuation(config, active_input(LsiDriveRequest::PulsePositive, 0.4)).unwrap();
        let negative =
            plan_lsi_actuation(config, active_input(LsiDriveRequest::PulseNegative, -0.4)).unwrap();

        assert!(negative.phase_u_voltage_command_v < bias.phase_u_voltage_command_v);
        assert!(bias.phase_u_voltage_command_v < positive.phase_u_voltage_command_v);
        for plan in [bias, positive, negative] {
            assert!(!plan.safe_output_required);
            assert!(plan.drive_active);
            assert_eq!(plan.source_control_tick, 41);
            assert_eq!(plan.expected_active_control_tick, 42);
            assert!((plan.duty_u + plan.duty_v + plan.duty_w - 1.5).abs() < 1.0e-6);
            assert_eq!(plan.duty_v, plan.duty_w);
        }
    }

    #[test]
    fn request_sign_and_force_safe_contract_are_strict() {
        let config = LsiActuationConfig::default();
        assert_eq!(
            plan_lsi_actuation(config, active_input(LsiDriveRequest::PulsePositive, -0.4)),
            Err(LsiActuationError::InvalidInput)
        );
        let mut input = active_input(LsiDriveRequest::Bias, 0.0);
        input.force_safe_output = true;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::InvalidInput)
        );
    }

    #[test]
    fn capture_and_fault_gates_fail_closed() {
        let config = LsiActuationConfig::default();
        let mut input = active_input(LsiDriveRequest::Bias, 0.0);
        input.capture_ready = false;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::NotReady)
        );
        input.capture_ready = true;
        input.capture_full = true;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::Fault)
        );
        input.capture_full = false;
        input.hardware_fault = true;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::Fault)
        );
    }

    #[test]
    fn bus_current_and_non_finite_values_are_rejected_without_clamping() {
        let config = LsiActuationConfig::default();
        let mut input = active_input(LsiDriveRequest::Bias, 0.0);
        input.bus_voltage_v = 6.99;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::Fault)
        );
        input.bus_voltage_v = 12.3;
        input.phase_u_current_a = 1.15;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::Fault)
        );
        input.phase_u_current_a = f32::NAN;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::InvalidInput)
        );
    }

    #[test]
    fn widened_or_mistyped_envelopes_are_rejected() {
        let mut config = LsiActuationConfig {
            maximum_bias_current_a: 0.21,
            ..LsiActuationConfig::default()
        };
        assert!(!config.is_valid());
        config = LsiActuationConfig {
            maximum_perturbation_voltage_v: 0.401,
            ..LsiActuationConfig::default()
        };
        assert!(!config.is_valid());
        config = LsiActuationConfig {
            sample_rate_hz: 30_000,
            ..LsiActuationConfig::default()
        };
        assert!(!config.is_valid());
        config = LsiActuationConfig {
            minimum_duty: 0.0,
            ..LsiActuationConfig::default()
        };
        assert!(!config.is_valid());
    }

    #[test]
    fn h2_first_pulse_envelope_rejects_voltage_and_trip_boundaries() {
        let config = LsiActuationConfig {
            maximum_perturbation_voltage_v: 0.10,
            current_trip_a: 0.25,
            ..LsiActuationConfig::default()
        };
        assert!(config.is_valid());

        let mut input = active_input(LsiDriveRequest::PulsePositive, 0.10);
        input.phase_u_current_a = 0.249;
        assert!(plan_lsi_actuation(config, input).is_ok());

        input.requested_perturbation_voltage_v = 0.101;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::InvalidInput)
        );

        input.requested_perturbation_voltage_v = 0.10;
        input.phase_u_current_a = 0.25;
        assert_eq!(
            plan_lsi_actuation(config, input),
            Err(LsiActuationError::Fault)
        );
    }
}
