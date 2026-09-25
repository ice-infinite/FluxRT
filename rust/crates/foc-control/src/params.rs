//! FluxRT —— 电机与控制器参数集（含量纲与来源标注）。
//! FluxRT - motor and controller parameter sets with units and provenance.
//!
//! 本模块定义 `MotorParameters`（被控对象）、`ControlParameters`（PI 参数与
//! 频率）以及 ST MCSDK 6.4.1 参考工程的参数转录函数。
//! This module defines `MotorParameters` (the plant), `ControlParameters` (PI
//! gains and frequencies) and the transcription of the ST MCSDK 6.4.1 reference
//! project parameters.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   foc-rt-bridge -> `foc-control`（本模块）-> foc-algorithm
//!   `foc-rt-bridge` 通过 `foc_rust_configure()` 覆盖本模块的默认值，`foc-sim`
//!   则直接读同一份默认值，因此仿真与固件的起点完全一致。
//!   `foc-rt-bridge` overrides these defaults through `foc_rust_configure()` and
//!   `foc-sim` reads the same defaults, so simulation and firmware start equal.
//!
//! 来源标注 / Provenance tags: `[HW]` 台架实测、`[ST]` ST MCSDK 6.4.1 参考工程、
//!   `[FW]` 固件配置、`[VESC]` 借鉴 VESC 工程的公开控制策略。
//!   `[HW]` measured on the rig, `[ST]` from ST MCSDK 6.4.1, `[FW]` firmware
//!   config, `[VESC]` engineering strategy borrowed from VESC.
//!
//! 未辨识参数警告 / Un-identified parameter warning:
//!   定子电阻 `Rs`、电感 `Ls`、永磁磁链 `flux_linkage_wb` 与转动惯量
//!   `inertia_kg_m2` 全部来自 ST Workbench 数据库，**没有在实物电机上辨识过**。
//!   它们是仿真与首次上电的起点，不是最终整定值；在实机上完成辨识前，不要用
//!   它们下任何结论，也不要把观测器/电流环的整定建立在这些数值上。
//!   Stator resistance, inductance, flux linkage and inertia come from the ST
//!   Workbench database and are NOT identified on the physical motor. They are a
//!   starting point for simulation and first power-up, not final tuning values.
//!
//! 连续时间积分增益 / Continuous-time integral gain:
//!   `ki` 一律保持为**连续时间**增益，由算法按 `ki * ts * error` 离散化。因此
//!   改变控制频率只改变 `ts`，不需要重新整定 `ki`；`kp` 则与频率无关。这正是
//!   本项目能在 12 kHz 沿用 MCSDK 在 30 kHz 下整定出的积分增益的原因。
//!   `ki` stays a CONTINUOUS-TIME gain discretised by the algorithm as
//!   `ki * ts * error`, so changing the control frequency only changes `ts`;
//!   `kp` is frequency independent. That is how MCSDK's 30 kHz integral gains are
//!   reused at 12 kHz here.
//!
//! 量纲 / Units: 全部 SI 且为 `f32`（`[ohm] [H] [Wb] [A] [rpm] [V] [kg*m^2]`
//!   `[N*m*s] [s]`，PI 输出/积分限幅与 `kp`/`ki` 的量纲由其所属环决定）。
//!   本模块不使用 Q 格式：MCSDK 的定点增益在这里被换算成 SI/连续时间增益。
//!   All SI and `f32`; no Q formats here, the MCSDK fixed-point gains are
//!   converted to SI at this boundary.
//!
//! 参考 / Reference: docs/ST与VESC工程改进路线图.md,
//!   docs/架构与安全边界.md

use core::f32::consts::PI;

use foc_algorithm::PiParam;

/// 电机（被控对象）参数，全部 SI 单位。
/// Motor (plant) parameters, all in SI units.
///
/// 除 `pole_pairs`、`rated_current_a`、`max_speed_rpm` 与
/// `nominal_bus_voltage_v` 外，其余电气与机械参数均来自 ST Workbench 数据库，
/// **未经实物辨识**。其中 `stator_resistance_ohm`/`ld_h`/`lq_h` 直接决定观测器
/// 的 `rs`/`ls`，量级错误会让无感观测器无法收敛。
/// Except for `pole_pairs`, `rated_current_a`, `max_speed_rpm` and
/// `nominal_bus_voltage_v`, the electrical and mechanical values come from the ST
/// Workbench database and are NOT identified on the physical motor. In particular
/// `stator_resistance_ohm`/`ld_h`/`lq_h` feed the observer's `rs`/`ls`, so a wrong
/// order of magnitude prevents sensorless convergence.
///
/// `lq_h` 与 `ld_h` 分开保留是为了支持内嵌式（IPM）电机的凸极转矩与 MTPA；
/// 表贴式电机两者相等，`foc-rt-bridge` 配置时会把同一个值写进两者。
/// Keeping `lq_h` separate from `ld_h` supports IPM saliency and MTPA; for a
/// surface PMSM they are equal and the bridge writes one value into both.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorParameters {
    /// 极对数（不是极数）；电角度 = 机械角度 × 极对数。
    /// Pole pairs, not pole count; electrical angle = mechanical * pole pairs.
    pub pole_pairs: u8,
    /// 定子相电阻 `[ohm]`，未在实物上辨识；直接进入观测器的 `Rs*i` 前馈。
    /// Per-phase stator resistance `[ohm]`, not identified; feeds `Rs*i`.
    pub stator_resistance_ohm: f32,
    /// d 轴电感 `[H]`，未在实物上辨识。
    /// d-axis inductance `[H]`, not identified on hardware.
    pub ld_h: f32,
    /// q 轴电感 `[H]`；表贴式电机等于 `ld_h`，内嵌式用于凸极转矩。
    /// q-axis inductance `[H]`; equals `ld_h` for a surface PMSM.
    pub lq_h: f32,
    /// 永磁磁链 `[Wb]`，未在实物上辨识；决定反电势幅值与转矩常数。
    /// PM flux linkage `[Wb]`, not identified; sets BEMF and torque constant.
    pub flux_linkage_wb: f32,
    /// 额定相电流（幅值）`[A]`；用作速度环输出限幅与观测器边界层标度。
    /// Rated phase current amplitude `[A]`; scales the speed PI limit.
    pub rated_current_a: f32,
    /// 最高机械转速 `[rpm]`；观测器可靠性门限与目标转速校验都引用它。
    /// Maximum mechanical speed `[rpm]`; referenced by observer reliability.
    pub max_speed_rpm: f32,
    /// 标称直流母线电压 `[V]`；观测器滑模增益按它标度。
    /// Nominal DC bus voltage `[V]`; scales the SMO sliding gain.
    pub nominal_bus_voltage_v: f32,
    /// 转动惯量 `[kg*m^2]`，Workbench 估值，未在实物上辨识。
    /// Rotor inertia `[kg*m^2]`, a Workbench estimate, not identified.
    pub inertia_kg_m2: f32,
    /// 粘滞摩擦系数 `[N*m*s]`，Workbench 估值，未在实物上辨识。
    /// Viscous friction `[N*m*s]`, a Workbench estimate, not identified.
    pub viscous_friction_nm_s: f32,
}

/// 控制器参数集：被控对象 + 三组 PI + 频率与限幅策略。
/// Controller parameter set: plant, three PI gains, frequencies and limits.
///
/// `id_pi`/`iq_pi` 是电流环（d/q 各一个，通常同值），`speed_pi` 是外环速度环。
/// 两个频率字段决定多速率调度：电流环跑 `pwm_frequency_hz`（实机 12 kHz，与 ADC
/// ISR 同频），速度环跑 `speed_loop_frequency_hz`（1 kHz），两者必须整除。
/// `id_pi`/`iq_pi` are the d/q current loops (normally identical) and `speed_pi`
/// is the outer speed loop. The frequency fields drive the multi-rate schedule:
/// the current loop at `pwm_frequency_hz` (12 kHz on target, same as the ADC ISR)
/// and the speed loop at `speed_loop_frequency_hz` (1 kHz), which must divide it.
///
/// `voltage_utilization` 是电压圆限幅可用的母线比例（本项目 0.95）；留出的余量
/// 覆盖死区、管压降与 SVPWM 过调制。调高会提高转矩上限，但更容易进入限幅失真。
/// `voltage_utilization` is the usable bus fraction for the circle limiter (0.95
/// here); the margin covers dead time, drops and over-modulation. Raising it lifts
/// the torque ceiling but makes limiting distortion more likely.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlParameters {
    /// 被控对象参数（电阻、电感、磁链、惯量等）。
    /// Plant parameters (resistance, inductance, flux linkage, inertia).
    pub motor: MotorParameters,
    /// d 轴电流环 PI；输出与积分限幅是电压 `[V]`。
    /// d-axis current PI; output and integrator limits are volts `[V]`.
    pub id_pi: PiParam,
    /// q 轴电流环 PI；通常与 `id_pi` 完全相同。
    /// q-axis current PI; normally identical to `id_pi`.
    pub iq_pi: PiParam,
    /// 速度环 PI；输出与积分限幅是 q 轴电流 `[A]`。
    /// Speed PI; output and integrator limits are q-axis current `[A]`.
    pub speed_pi: PiParam,
    /// 电流环执行频率 `[Hz]`，实机与 PWM/ADC 同频（12 kHz）。
    /// Current-loop rate `[Hz]`, equal to PWM/ADC on target (12 kHz).
    pub pwm_frequency_hz: u32,
    /// 速度环执行频率 `[Hz]`（1 kHz）；必须整除 `pwm_frequency_hz`。
    /// Speed-loop rate `[Hz]` (1 kHz); must divide `pwm_frequency_hz`.
    pub speed_loop_frequency_hz: u32,
    /// 电压圆限幅可用的母线比例，无量纲，典型 0.95。
    /// Usable bus fraction for the voltage circle, dimensionless, typically 0.95.
    pub voltage_utilization: f32,
    /// 未指定目标转速时的默认机械转速 `[rpm]`。
    /// Default mechanical target speed `[rpm]` when none is given.
    pub default_target_speed_rpm: f32,
}

/// [ST] MCSDK 6.4.1 生成的电流环比例增益，原始定点值为 `3378/1024`。
/// [ST] MCSDK 6.4.1 generated current-loop proportional gain, raw `3378/1024`.
///
/// 量纲是 MCSDK 的"归一化电压 / ADC 计数"比值，不是 SI；换算过程见
/// [`st_gbm2804_reference_parameters`]。
/// The unit is MCSDK's normalized-voltage-per-ADC-count ratio, not SI; the
/// conversion lives in [`st_gbm2804_reference_parameters`].
pub const ST_RAW_CURRENT_KP: f32 = 3378.0 / 1024.0;
/// [ST] MCSDK 电流环**每周期**积分增量，原始定点值 `2252/4096`。
/// [ST] MCSDK current-loop integral increment PER TICK, raw `2252/4096`.
///
/// 注意它是每周期量而不是连续时间增益；转录时会乘回 MCSDK 的载波频率
/// （30 kHz）还原成连续时间 `ki`，再由算法用 `ki * ts` 在 12 kHz 下重新离散化。
/// This is a per-tick increment, not a continuous-time gain: the transcription
/// multiplies it back by MCSDK's 30 kHz carrier to recover `ki`, which the
/// algorithm then discretises at 12 kHz as `ki * ts`.
pub const ST_RAW_CURRENT_KI_PER_TICK: f32 = 2252.0 / 4096.0;
/// [ST] MCSDK 生成的速度环比例增益，原始定点值 `2730/256`。
/// [ST] MCSDK generated speed-loop proportional gain, raw `2730/256`.
///
/// 量纲是"每 `SPEED_UNIT` 的电流计数"，需乘 `speed_units_per_rad_s` 并除以电流
/// 换算比例才能得到 SI 的 `A/(rad/s)`。
/// Its unit is current counts per `SPEED_UNIT`; it needs the speed-unit factor and
/// the current conversion ratio to become SI `A/(rad/s)`.
pub const ST_RAW_SPEED_KP: f32 = 2730.0 / 256.0;
/// [ST] MCSDK 速度环**每周期**积分增量，原始定点值 `562/16384`。
/// [ST] MCSDK speed-loop integral increment PER TICK, raw `562/16384`.
///
/// 与电流环同理：先还原成连续时间增益，再在 1 kHz 下用 `ki * ts` 离散化。
/// Like the current loop, it is restored to continuous time and then discretised
/// at 1 kHz by `ki * ts`.
pub const ST_RAW_SPEED_KI_PER_TICK: f32 = 562.0 / 16384.0;
/// [ST] 电流换算比例：每安培对应的 Q15 ADC 计数。
/// [ST] Current conversion ratio: Q15 ADC counts per ampere.
///
/// 这个乘积把 MCSDK 的"每计数"增益换算成"每安培"增益，是下面所有电流环与
/// 速度环增益换算的公共因子。各因子的乘积顺序保持与 MCSDK 生成代码一致，
/// 便于逐项对照；各因子的物理出处（采样电阻、运放增益、偏置、参考电压）
/// 未在仓库内记录，修改前请回查 MCSDK Workbench 工程。
/// This product converts MCSDK's per-count gains into per-ampere gains and is the
/// common factor for every gain conversion below. The multiplication order is kept
/// identical to the generated MCSDK code for term-by-term comparison; the physical
/// origin of each factor is not recorded in this repository, so check the MCSDK
/// Workbench project before changing it.
pub const ST_CURRENT_CONVERSION_COUNTS_PER_AMP: f32 = 65536.0 * 0.33 * 1.53 / 3.3;

/// 转录自 MCSDK 6.4.1 为 NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T
/// 生成的工程参数。
/// Parameters transcribed from the generated MCSDK 6.4.1 project for
/// NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T.
///
/// MCSDK 把 PI 增益存成 ADC/电流与归一化电压的整数单位；下面的公式在保留这些
/// 增益数值不变的前提下把它们换算成 SI 单位。Workbench 的机械量（0.291 与
/// 0.937）是仿真估值，必须在实物电机/负载上重新辨识后才能用于整定。
/// MCSDK stores PI gains in ADC/current and normalized-voltage integer units.
/// The formulas below preserve those gains while converting them to SI units.
/// The Workbench mechanical values (0.291 and 0.937) are simulation estimates;
/// they must be identified again on the physical motor/load before tuning.
///
/// 未辨识警告 / Un-identified: `Rs`、`Ls`、磁链与转动惯量来自 Workbench 数据库，
/// **未在实物电机上辨识**，只是起点而非最终整定值。
/// `Rs`, `Ls`, flux linkage and inertia come from the Workbench database and are
/// NOT identified on the physical motor; they are a starting point only.
///
/// 量纲 / Units: 返回的电流环 PI 输出与积分限幅是电压 `[V]`，速度环是电流
/// `[A]`；`ts` 单位 `[s]`；`kp`/`ki` 为连续时间增益（`ki` 见本文件头部说明）。
/// Returned current-PI limits are volts `[V]` and speed-PI limits are amperes
/// `[A]`; `ts` is in `[s]`; `ki` is a continuous-time gain (see the module header).
pub fn st_gbm2804_reference_parameters() -> ControlParameters {
    // MCSDK 参考工程的载波频率，用于把"每周期"积分增量还原成连续时间增益。
    // MCSDK reference carrier frequency, used to restore continuous-time gains.
    const MCSDK_REFERENCE_PWM_HZ: f32 = 30_000.0;
    // 本工程实际控制频率：与 TIM1 中心对齐载波和 ADC 注入采样同频。
    // This project's control rate, matching the TIM1 carrier and ADC sampling.
    const CONTROL_PWM_HZ: f32 = 12_000.0;
    // 速度环频率：1 kHz，必须整除 CONTROL_PWM_HZ。
    // Speed-loop rate, 1 kHz; must divide CONTROL_PWM_HZ.
    const SPEED_HZ: f32 = 1_000.0;
    // 参考工程的母线电压 `[V]`，同时作为 MotorParameters 的标称母线电压。
    // Reference-project DC bus in `[V]`, also the nominal bus voltage.
    const BUS_V: f32 = 13.0;
    // 电压圆可用率：留 5% 余量给死区、管压降与过调制。
    // Usable bus fraction: 5 percent margin for dead time, drops, over-modulation.
    const VOLTAGE_UTILIZATION: f32 = 0.95;
    // 电压圆半径：SVPWM 线性区的最大相电压幅值是 Vbus/sqrt(3)，再乘可用率。
    // Voltage circle radius: the linear-range phase amplitude is Vbus/sqrt(3).
    let max_voltage = BUS_V * VOLTAGE_UTILIZATION / libm::sqrtf(3.0);
    // MCSDK 把电压归一化到 Q15 有符号满量程，所以除以 32767 得到每计数伏特数。
    // MCSDK normalizes voltage to signed Q15 full scale, hence the 32767 divisor.
    let volts_per_normalized_count = max_voltage / 32767.0;
    // 把"每 ADC 计数"的增益换算成"每安培"：两者相乘得到 V/A 量纲的公共因子。
    // Converts per-ADC-count gains into per-ampere ones: the common V/A factor.
    let current_gain_scale = ST_CURRENT_CONVERSION_COUNTS_PER_AMP * volts_per_normalized_count;
    // kp 与频率无关，直接按公共因子换算即可。
    // kp is frequency independent, so the common factor alone converts it.
    let current_kp = ST_RAW_CURRENT_KP * current_gain_scale;
    // 保留参考工程的连续时间积分增益，同时让本工程的电流环跑 12 kHz：原始值
    // 是 30 kHz 下的每周期增量，乘回 MCSDK_REFERENCE_PWM_HZ 还原成连续时间
    // 增益；算法随后用 ki * ts 在 12 kHz 下重新离散化，因此换频率无需重调整定。
    // Preserve the reference project's continuous-time integral gain while
    // running this project's current loop at 12 kHz.
    let current_ki = ST_RAW_CURRENT_KI_PER_TICK * current_gain_scale * MCSDK_REFERENCE_PWM_HZ;

    // SPEED_UNIT is U_01HZ: 10 units/Hz. Convert the MCSDK input to rad/s.
    // SPEED_UNIT 是 U_01HZ：10 个单位表示 1 Hz。换算成 rad/s：
    // 1 Hz = 2*pi rad/s，故 10 单位/Hz ÷ (2*pi) 得到 单位/(rad/s)。
    // 1 Hz = 2*pi rad/s, so 10 units/Hz divided by 2*pi gives units per rad/s.
    let speed_units_per_rad_s = 10.0 / (2.0 * PI);
    // 速度环的比例增益：原始单位是"每 SPEED_UNIT 的电流计数"，先换成每 rad/s，
    // 再除以电流换算比例换成安培，得到 A/(rad/s)。
    // Speed proportional gain: per-SPEED_UNIT counts converted to per-rad/s and
    // then to amperes, giving A/(rad/s).
    let speed_kp = ST_RAW_SPEED_KP * speed_units_per_rad_s / ST_CURRENT_CONVERSION_COUNTS_PER_AMP;
    // 速度环积分增益同理，但要乘回速度环自身频率（1 kHz）还原连续时间增益：
    // 原始值是 1 kHz 下的每周期增量。这里与电流环的 30 kHz 不同，容易抄错。
    // The speed integral gain is restored with the SPEED loop's own 1 kHz rate,
    // not the current loop's 30 kHz; that difference is easy to get wrong.
    let speed_ki = ST_RAW_SPEED_KI_PER_TICK * speed_units_per_rad_s
        / ST_CURRENT_CONVERSION_COUNTS_PER_AMP
        * SPEED_HZ;

    ControlParameters {
        motor: MotorParameters {
            // 极对数 7：电角度 = 机械角度 × 7。
            // Pole pairs 7: electrical angle = mechanical angle * 7.
            pole_pairs: 7,
            // 以下电气参数均来自 Workbench 数据库，未在实物电机上辨识。
            // The electrical values below come from the Workbench database and are
            // not identified on the physical motor.
            stator_resistance_ohm: 5.29,
            ld_h: 0.001_058,
            lq_h: 0.001_058,
            // Workbench 的 M1_MOTOR_RATED_FLUX 用它内部的角速度基准，所以这里除以
            // 2*pi 换成以 Wb 为单位的磁链。丢掉这个换算会让磁链偏大 2*pi 倍，
            // 观测器与转矩前馈随之整体失准。
            // Workbench M1_MOTOR_RATED_FLUX is in its internal angular basis.
            flux_linkage_wb: 0.034_739_897 / (2.0 * PI),
            rated_current_a: 0.8,
            max_speed_rpm: 1572.0,
            nominal_bus_voltage_v: BUS_V,
            // Workbench 原始值保留在文档里。这些 SI 换算只是主机仿真与被控对象
            // 模型的起点，不是辨识结果。
            // Workbench raw values are retained in documentation. These SI
            // conversions are only a host-plant starting point, not identified data.
            inertia_kg_m2: 0.291e-4,
            viscous_friction_nm_s: 0.937e-5,
        },
        id_pi: PiParam {
            kp: current_kp,
            ki: current_ki,
            ts: 1.0 / CONTROL_PWM_HZ,
            // 输出与积分限幅都是电压 [V]，用电压圆半径；积分器与输出同限幅，
            // 因此不会出现"积分绕死"式的饱和恢复延迟。
            // Output and integrator limits are the voltage circle radius in [V];
            // matching them avoids integrator wind-up recovery delay.
            out_min: -max_voltage,
            out_max: max_voltage,
            integrator_min: -max_voltage,
            integrator_max: max_voltage,
        },
        iq_pi: PiParam {
            kp: current_kp,
            ki: current_ki,
            ts: 1.0 / CONTROL_PWM_HZ,
            out_min: -max_voltage,
            out_max: max_voltage,
            integrator_min: -max_voltage,
            integrator_max: max_voltage,
        },
        speed_pi: PiParam {
            kp: speed_kp,
            ki: speed_ki,
            ts: 1.0 / SPEED_HZ,
            // 速度环的输出就是 q 轴电流给定，所以限幅单位是安培 [A]；
            // 0.8 A 即 rated_current_a，因此速度环天然受额定电流约束。
            // The speed loop outputs the q-axis current reference, so the limit is
            // in [A]; 0.8 A equals the rated current.
            out_min: -0.8,
            out_max: 0.8,
            integrator_min: -0.8,
            integrator_max: 0.8,
        },
        pwm_frequency_hz: CONTROL_PWM_HZ as u32,
        speed_loop_frequency_hz: SPEED_HZ as u32,
        voltage_utilization: VOLTAGE_UTILIZATION,
        // 默认目标转速 524 rpm，取自参考工程。注意它低于默认预启动终点
        //（RevUpConfig::final_speed_rpm = 582 rpm），而 foc_rust_start_realtime()
        // 要求请求转速不低于预启动终点，所以这个默认值不能直接用于实时启动。
        // Default 524 rpm from the reference project; it is BELOW the default rev-up
        // endpoint (582 rpm) which `foc_rust_start_realtime()` requires, so this
        // default is not directly usable for a realtime start.
        default_target_speed_rpm: 524.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 锁定"从 MCSDK 定点增益换算到 SI"这一步的数值与比例关系：频率字段、极对数
    /// 以及两个环的比例增益。它防止换算因子（电流换算、Q15 满量程、速度单位）
    /// 被无意改动，因为这类改动在实机上只表现为整定变差，很难归因。
    /// Pins the fixed-point-to-SI gain conversion: frequency fields, pole pairs and
    /// both proportional gains. It guards the conversion factors (current scaling,
    /// Q15 full scale, speed unit), whose corruption only shows up on hardware as
    /// mysteriously worse tuning.
    #[test]
    fn reference_parameters_preserve_generated_ratios() {
        let p = st_gbm2804_reference_parameters();
        assert_eq!(p.pwm_frequency_hz, 12_000);
        assert_eq!(p.speed_loop_frequency_hz, 1_000);
        assert_eq!(p.motor.pole_pairs, 7);
        assert!((p.id_pi.kp - 7.19).abs() < 0.6);
        assert_eq!(p.id_pi, p.iq_pi);
        assert!((p.speed_pi.kp - 0.001_7).abs() < 0.000_1);
    }
}
