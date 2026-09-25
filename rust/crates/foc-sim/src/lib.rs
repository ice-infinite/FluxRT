//! FluxRT —— PC 侧离线电机/逆变器仿真与实机相关性验证（`foc-sim`）。
//! FluxRT - offline PC motor/inverter simulation and hardware-correlation harness
//! (`foc-sim`).
//!
//! 职责 / Responsibility:
//!   - 两个仿真入口：`run_reference_simulation`（理想转子反馈的控制律回归）与
//!     `run_bringup_simulation`（走固件同一套 C ABI 快环的启动/观测器对标）。
//!   - PC 端 PMSM 被控对象、平均值逆变器（含死区压降）、`SimHardware` 假硬件，
//!     以及 CSV 导出与 RMSE/均值/标准差指标。
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   `foc-sim`（本 crate，仅 PC/`std`）-> `foc-control` -> `foc-algorithm`；
//!   `run_bringup_simulation` 另外调用 `foc-rt-bridge` 的固件 C ABI 符号。
//!   依赖单向：仿真依赖控制与桥接层；控制层不认识仿真，也不使用堆。
//!
//! 建模边界 / Modelling boundary（避免过度解读对标结果）:
//!   本 crate 只建模"控制器 + 平均值逆变器 + PMSM 被控对象"。两条入口的反馈都是
//!   **理想值**：三相电流由被控对象 dq 状态经逆 Park/逆 Clarke 反算，没有电流
//!   传感器模型，也没有 ADC 量化、零偏、增益误差；母线电压取常量，没有纹波；
//!   转子角与转速用的是对象真值。现在只建模 PWM 边界、整数控制分频、占空保持与
//!   可配置整拍生效延迟；仍不建模开关器件/纹波、ADC 量化与相间采样偏斜。因此
//!   对标的是控制律、启动时序与观测器行为，**不是**完整电流采样
//!   链路或 ADC/PWM 时序的验证。
//!   Only the controller, the average-value inverter and the plant are modelled:
//!   feedback is ideal (no current-sensor model, no ADC quantisation/offset, no
//!   sampling delay), so this is not a validation of the ADC/PWM chain.
//!
//! 实时约束 / Real-time constraints:
//!   本模块是离线 PC 代码：分配（`Vec`）、文件 I/O、`println!` 与 `?` 传播
//!   在这里都允许。真实固件快环运行在 12 kHz ADC 中断里，禁止动态分配、阻塞、
//!   日志与锁等待；这里的 `Vec`/CSV/打印**不得**照搬进实时路径。复用固件 C ABI
//!   的调用序列只保证控制行为一致，不携带任何实时性保证。
//!   This is offline PC code where `Vec`, CSV I/O and printing are fine, but they
//!   must never be copied into the 12 kHz ADC ISR realtime path.
//!
//! 量纲 / Units:
//!   全程 SI：电流 `[A]`、电压 `[V]`、角度 `[rad]`、角速度 `[rad/s]`、机械转速
//!   `[rpm]`、时间 `[s]`、转矩 `[N*m]`、电阻 `[ohm]`、电感 `[H]`、磁链 `[Wb]`；
//!   死区时间配置用 `[ns]`、参与计算时换算为 `[s]`。占空比、滤波系数与补偿增益
//!   无量纲（占空比范围 `[0,1]`），滑模增益 `[V]`、滑模边界 `[A]`、PLL 比例增益
//!   `[1/s]`、PLL 积分增益 `[1/s^2]`。
//!   SI units throughout; duty ratios and filter/compensation gains are
//!   dimensionless. CSV headers carry a unit suffix per column.
//!
//! 定点与 Q 格式 / Fixed point:
//!   本模块为纯 `f32`，不使用任何 Q 格式。定点与 CORDIC 缩放只存在于 C 平台
//!   适配器内部，本 crate 不做任何定点换算。
//!
//! 参考 / Reference: `docs/仿真实机相关性验证.md`,
//! `docs/ST与VESC工程改进路线图.md`（阶段 3 逆变器电压估计层）

use std::f32::consts::PI;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use foc_algorithm::{
    average_inverter_phase_voltage_loss_v, clarke, inverse_clarke, inverse_park, park, Abc,
    AlphaBeta, Dq, InverterLossParameters,
};
use foc_control::{
    pwm_to_alpha_beta, pwm_to_phase_voltage, FeedbackPort, FeedbackSnapshot, HardwareFault,
    MotorParameters, PhaseCurrents, PwmCommand, PwmPort, RotorFeedback, SafetyPort,
};

pub mod scheduler;
pub use scheduler::{MultiRateScheduler, MultiRateTimingConfig, TimingConfigError};

/// PC 平均值逆变器模型的死区配置（只描述平均压降，不含开关器件）。
/// Dead-time configuration of the PC average-value inverter model.
///
/// 量纲 / Units: `dead_time_s` 为 `[s]`；`dead_time_enabled` 是无量纲总开关。
///
/// 固定值来源 / Provenance: 默认 `550.0e-9` `[s]` 是功率板**实测**死区 `[HW]`
///   （STSPIN830 硬件死区 550 ns，TIM1 BDTR 死区寄存器为 0，见
///   `docs/2026-09-22实机烧录记录.md`）。它属于硬件/台架类常量，不是从
///   MCSDK Workbench 继承、也不是固件配置项。`dead_time_enabled` 默认为
///   `false`，即默认按理想逆变器计算，保证既有回归口径不变。
/// The 550 ns default is the rig-measured power-board dead time ([HW]); the model
/// is disabled by default so ideal-inverter baselines stay comparable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverterSimulationConfig {
    /// 死区模型总开关：`false` 时 `pwm_to_alpha_beta_with_inverter` 直接返回理想
    /// 结果，`dead_time_s` 被忽略。
    /// Master switch; `dead_time_s` is ignored when this is false.
    pub dead_time_enabled: bool,
    /// 单次换流的死区时间 `[s]`；功率板实测值 `550e-9` `[HW]`。
    /// Per-commutation dead time in `[s]`; the rig-measured value is 550 ns [HW].
    pub dead_time_s: f32,
    /// 等效单相器件导通压降 `[V]`；默认 0 保持已有对拍。
    pub device_drop_v: f32,
}

/// 默认值：死区模型关闭，死区时间取功率板实测 550 ns `[HW]`。
/// Defaults: model disabled, 550 ns measured power-board dead time [HW].
impl Default for InverterSimulationConfig {
    fn default() -> Self {
        Self {
            dead_time_enabled: false,
            dead_time_s: 550.0e-9,
            device_drop_v: 0.0,
        }
    }
}

/// 带死区压降的逆变器平均端电压（αβ），与 MATLAB 被控对象模型同口径。
/// Averaged inverter terminal voltage including polarity-dependent dead-time
/// loss. This matches the MATLAB plant convention used for correlation.
///
/// 模型边界（重要）/ Model scope (important):
///   这是**平均值模型，没有开关器件**：没有 PWM 载波、没有 MOSFET/二极管模型、
///   没有开关纹波，也没有电流纹波。它只从指令相电压里减去与电流极性相关的平均
///   压降 `loss_v = 2 * dead_time_s / pwm_period_s * dc_bus_voltage` `[V]`，
///   因此也不建模器件导通压降、母线纹波、ADC 量化与噪声。
///   Average-value model only: no carrier, no switching devices, no ripple.
///
/// 极性约定 / Polarity convention:
///   死区期间某相上下管全关，相电流经二极管续流，把端电压钳向与电流方向相反的
///   母线，平均效果是**削弱**指令电压幅值，即
///   `v_actual = v_commanded - loss_v * sign(i)`。系数 `2` 不是笔误：一个 PWM
///   周期内每相有两次换流（上下管各一次），平均损失是单次死区占比的两倍。
///   若把符号写成加号，模型不再是削弱而是**增强**指令电压，误差幅值变成
///   `2 * loss_v`（翻倍）且方向相反——这就是死区模型"符号写反比不写更糟"的原因。
///   The sign must subtract: a flipped sign doubles the error instead of
///   cancelling it, because the physical loss opposes the commanded voltage.
///
/// 参数 / Parameters:
///   pwm                 三相占空比指令，无量纲 `[0,1]`
///   currents            本拍三相电流 `[A]`，只用于取极性
///   dc_bus_voltage      母线电压 `[V]`
///   pwm_period_s        PWM 载波周期 `[s]`
///   inverter            死区配置，见 `InverterSimulationConfig`
///
/// 时间基准 / Time base: `pwm_period_s` 必须是 PWM 周期；即使控制链按整数分频
///   低于载波频率，也不能用控制周期代替，否则死区占比会按比例错算。
/// Dead time is normalised by the PWM period, not by the control-tick count.
///
/// 返回 / Returns: αβ 相电压 `[V]`；`inverter.dead_time_enabled == false` 时直接
///   返回 `pwm_to_alpha_beta()` 的理想值，不做任何减法。
///   Returns the αβ voltage in `[V]`, or the ideal value when the model is off.
///
/// 调用上下文 / Context: 仅离线仿真使用（`run_bringup_simulation` 的 plant 环）。
pub fn pwm_to_alpha_beta_with_inverter(
    pwm: PwmCommand,
    currents: PhaseCurrents,
    dc_bus_voltage: f32,
    pwm_period_s: f32,
    inverter: InverterSimulationConfig,
) -> AlphaBeta {
    if !inverter.dead_time_enabled {
        return pwm_to_alpha_beta(pwm, dc_bus_voltage);
    }
    let mut phase = pwm_to_phase_voltage(pwm, dc_bus_voltage);
    // Plant 使用同一份纯数学公式，但零带固定为 0，保留物理对象的硬符号跳变。
    let loss = average_inverter_phase_voltage_loss_v(
        InverterLossParameters {
            dead_time_s: inverter.dead_time_s,
            pwm_period_s,
            device_drop_v: inverter.device_drop_v,
            current_zero_band_a: 0.0,
        },
        dc_bus_voltage,
        Abc {
            a: currents.a,
            b: currents.b,
            c: currents.c,
        },
    );
    phase.a -= loss.a;
    phase.b -= loss.b;
    phase.c -= loss.c;
    clarke(Abc {
        a: phase.a,
        b: phase.b,
        c: phase.c,
    })
}

/// PMSM 被控对象状态快照（SI，全 `f32`）。
/// PMSM plant state snapshot in SI units.
///
/// 量纲 / Units: dq 电流 `[A]`、机械角速度 `[rad/s]`、机械角度 `[rad]`（环回
///   `[0, 2π)`）、电磁转矩 `[N*m]`。这里存的是**机械**量与电磁转矩，电角度由
///   `PmsmPlant::electrical_angle_rad()` 乘极对数导出。
/// Holds mechanical quantities; the electrical angle is derived via pole pairs.
#[derive(Clone, Copy, Debug, Default)]
pub struct PlantState {
    /// d 轴电流 `[A]`，励磁分量。
    /// d-axis (magnetising) current in `[A]`.
    pub id_a: f32,
    /// q 轴电流 `[A]`，转矩分量。
    /// q-axis (torque-producing) current in `[A]`.
    pub iq_a: f32,
    /// 机械角速度 `[rad/s]`；正方向由 dq/Park 约定与相序共同决定。
    /// Mechanical angular velocity in `[rad/s]`.
    pub mechanical_speed_rad_s: f32,
    /// 机械角度 `[rad]`，每次积分后环回 `[0, 2π)`。
    /// Mechanical angle in `[rad]`, wrapped into `[0, 2π)` after each step.
    pub mechanical_angle_rad: f32,
    /// 上一拍算出的电磁转矩 `[N*m]`，只作诊断/CSV 量，不参与状态积分。
    /// Last computed electromagnetic torque in `[N*m]`; diagnostic output only.
    pub electromagnetic_torque_nm: f32,
}

/// 理想 PMSM 模型 + 定步长前向欧拉积分（PC 被控对象）。
/// Ideal PMSM model integrated with fixed-step forward (explicit) Euler.
///
/// 方程 / Equations（dq 坐标系，SI 单位；`we` 为电角速度 `[rad/s]`）:
///   `Ld * did/dt = vd - Rs*id + we*Lq*iq`
///   `Lq * diq/dt = vq - Rs*iq - we*(Ld*id + flux_linkage)`
///   `Te = 1.5 * pole_pairs * (flux_linkage*iq + (Ld - Lq)*id*iq)` `[N*m]`
///   `J * dwe_m/dt = Te - T_load - B*we_m`
///   式中的 `1.5` 是在幅值不变 dq 约定下的 `3/2` 系数，与 `foc-algorithm`
///   的变换系数配套；换用功率不变约定会让转矩整体差 `2/3` 倍。
///   The `1.5` is the `3/2` factor of the amplitude-invariant dq convention.
///
/// 积分器与稳定性 / Integrator and stability:
///   显式前向欧拉，步长 = `dt_s`（本工程 `1/12000 s`）。它只有一阶精度且**条件
///   稳定**：电气时间常数 `Ld/Rs` 约 0.2 ms，步长必须明显小于它，否则
///   `Rs*dt/Ld` 接近 2 时电流状态数值发散；机械时间常数大得多，同一 `dt_s` 对
///   机械方程没有压力。本结构不校验参数与步长：`dt_s` 为 0/负数/`NaN`，或
///   `J`、`Ld`、`Lq` 为 0 时状态会静默变成 `inf`/`NaN`，由调用方保证输入合法。
///   Forward Euler is first order and conditionally stable; nothing is validated
///   here, so an illegal `dt_s` silently diverges the state.
///
/// 未辨识参数 / Unidentified parameters: 电机参数来自
///   `foc_control::st_gbm2804_reference_parameters()`，其 Workbench 机械量
///   （惯量/粘滞摩擦）**未经实物辨识**，只能作为台架起点。
#[derive(Clone, Copy, Debug)]
pub struct PmsmPlant {
    /// 电机电气/机械参数与额定值，构造后不再修改。
    parameters: MotorParameters,
    /// 当前 dq/机械状态。
    state: PlantState,
    /// 负载转矩 `[N*m]`；正值表示阻碍正转的阻力矩。
    load_torque_nm: f32,
}

impl PmsmPlant {
    /// 用给定电机参数构造被控对象：状态全零（零电流、零转速、零角度）、空载。
    /// Builds a plant with all-zero state and no load.
    ///
    /// 零初值意味着转子停在电角度 0（d 轴与 A 轴重合），因此本被控对象**没有**
    /// 初始位置不确定性，也没有对齐过程；这正是理想反馈仿真的前提。
    /// Zero initial state means no initial-position uncertainty and no alignment.
    pub fn new(parameters: MotorParameters) -> Self {
        Self {
            parameters,
            state: PlantState::default(),
            load_torque_nm: 0.0,
        }
    }

    /// 只读返回状态快照（`Copy`，不暴露内部可变引用）。
    /// Returns a copy of the current state snapshot.
    pub fn state(&self) -> PlantState {
        self.state
    }

    /// 设置离线场景的初始转速和电角度；只改机械状态，d/q 电流仍为零。
    /// Sets the initial mechanical speed and electrical angle for an offline
    /// scenario while leaving both d/q currents at zero.
    ///
    /// 调用方必须事先校验有限性和速度范围。电角度会环回到 `[0, 2π)`，
    /// 再按极对数换成内部机械角度。
    /// The caller validates finiteness and speed range; the electrical angle is
    /// wrapped to `[0, 2π)` before conversion to the internal mechanical angle.
    pub fn set_initial_mechanical_state(
        &mut self,
        mechanical_speed_rpm: f32,
        electrical_angle_rad: f32,
    ) {
        self.state.mechanical_speed_rad_s = mechanical_speed_rpm * PI / 30.0;
        self.state.mechanical_angle_rad =
            electrical_angle_rad.rem_euclid(2.0 * PI) / self.parameters.pole_pairs as f32;
    }

    /// 电角度 `[rad]`：`机械角度 × 极对数`，再 `rem_euclid(2π)` 环回 `[0, 2π)`。
    /// Electrical angle in `[rad]` from the mechanical angle and pole pairs.
    ///
    /// 用 `rem_euclid` 而不是取余 `%`，是为了让反转（负角度）也落在 `[0, 2π)`：
    /// `%` 会给出负角度，Park/逆 Park 数学上仍自洽，但与固件角度约定不一致，
    /// CSV 的角度列就无法与实机 trace 直接叠加比较。
    /// `rem_euclid` keeps reverse rotation in `[0, 2π)` like the firmware.
    pub fn electrical_angle_rad(&self) -> f32 {
        (self.state.mechanical_angle_rad * self.parameters.pole_pairs as f32).rem_euclid(2.0 * PI)
    }

    /// 由 dq 状态反算三相电流 `[A]`：先用当前电角度逆 Park 到 αβ，再逆 Clarke。
    /// Reconstructs the three phase currents in `[A]` from the dq state.
    ///
    /// 这是"理想传感器"读数：不含 ADC 偏置、增益误差、量化、噪声与采样延时，
    /// 因此 `SimHardware` 的反馈比实机干净得多，基于它的电流指标不能当作实机精度。
    /// This is the ideal sensor reading: no ADC offset, gain error, noise or delay.
    pub fn phase_currents(&self) -> PhaseCurrents {
        let alpha_beta = inverse_park(
            Dq {
                d: self.state.id_a,
                q: self.state.iq_a,
            },
            self.electrical_angle_rad(),
        );
        let abc = inverse_clarke(alpha_beta);
        PhaseCurrents {
            a: abc.a,
            b: abc.b,
            c: abc.c,
        }
    }

    /// 设置负载转矩 `[N*m]`；正值与电磁转矩反向（阻碍正转）。
    /// Sets the load torque in `[N*m]`.
    ///
    /// 本函数立即生效，但只在下一步积分时进入机械方程，所以负载阶跃存在一拍延时。
    /// The new torque only enters the mechanical equation on the next step.
    pub fn set_load_torque_nm(&mut self, load_torque_nm: f32) {
        self.load_torque_nm = load_torque_nm;
    }

    /// 读取当前负载转矩 `[N*m]`（CSV 用它标注阶跃时刻，便于对齐比较）。
    /// Returns the current load torque in `[N*m]`.
    pub fn load_torque_nm(&self) -> f32 {
        self.load_torque_nm
    }

    /// 前进一个固定步长：把 αβ 电压 Park 到 dq，再积分电气与机械方程。
    /// Advances one fixed step: Park αβ voltage to dq, then integrate dq and
    /// mechanical equations.
    ///
    /// 参数 / Parameters:
    ///   voltage_alpha_beta 本拍施加的 αβ 定子电压 `[V]`，通常来自逆变器模型
    ///   dt_s               积分步长 `[s]`，必须为正且远小于电气时间常数 `Ld/Rs`
    ///
    /// 量纲 / Units: 内部全部 SI —— `[V]` `[A]` `[H]` `[Wb]` `[ohm]` `[N*m]`
    ///   `[kg*m^2]` `[N*m*s/rad]` `[rad]` `[rad/s]` `[s]`。
    ///
    /// 顺序与陷阱 / Order and pitfalls:
    ///   先用**本拍开始时的**电角度做 Park，再用同一时刻的状态求导数，最后统一
    ///   更新，因此是显式欧拉的一拍延时结构（先算后写，绝不能用已被本拍修改的
    ///   状态继续求导，否则会变成半隐式格式而改变数值行为）。电角速度由机械角
    ///   速度乘极对数得到。
    ///   电压方程的交叉耦合项符号必须成对正确：`+we*Lq*iq` 与
    ///   `-we*(Ld*id + flux)`。写反在低速台架上几乎看不出来，只在高速时表现为
    ///   等效电感/磁链错误。
    ///   本函数不做任何限幅，也不检查 `dt_s`：非法步长会让状态静默变成
    ///   `inf`/`NaN`，并沿 CSV 指标扩散。
    /// Explicit Euler with a one-sample delay; no clamping and no validation, so
    /// an illegal `dt_s` silently diverges the state.
    pub fn step(&mut self, voltage_alpha_beta: foc_algorithm::AlphaBeta, dt_s: f32) {
        // 本拍电角度：Park 与状态更新必须使用同一个角度，混用会引入两倍角度误差。
        // The same angle must be used for the Park transform and the update.
        let electrical_angle = self.electrical_angle_rad();
        let voltage_dq = park(voltage_alpha_beta, electrical_angle);
        let electrical_speed =
            self.state.mechanical_speed_rad_s * self.parameters.pole_pairs as f32;

        // d 轴电压方程：`Ld*did/dt = vd - Rs*id + we*Lq*iq`，除以 `Ld` 得到电流导数。
        // d-axis voltage equation solved for `did/dt`.
        let did_dt = (voltage_dq.d - self.parameters.stator_resistance_ohm * self.state.id_a
            + electrical_speed * self.parameters.lq_h * self.state.iq_a)
            / self.parameters.ld_h;
        // q 轴电压方程：`Lq*diq/dt = vq - Rs*iq - we*(Ld*id + flux)`；磁链项是
        // 反电势，`(Ld - Lq)` 的凸极项在转矩方程里，不在电压方程里。
        // q-axis equation; the flux term is the back-EMF and saliency acts on torque.
        let diq_dt = (voltage_dq.q
            - self.parameters.stator_resistance_ohm * self.state.iq_a
            - electrical_speed
                * (self.parameters.ld_h * self.state.id_a + self.parameters.flux_linkage_wb))
            / self.parameters.lq_h;

        // 电流用显式欧拉推进（一阶）；`dt_s` 过大时 `Rs*dt/Ld` 接近 2 会发散。
        // Explicit Euler for the currents; too large a step diverges the loop.
        self.state.id_a += did_dt * dt_s;
        self.state.iq_a += diq_dt * dt_s;
        // 转矩方程：`Te = 1.5*pole_pairs*(flux*iq + (Ld-Lq)*id*iq)`，先算 Te 再算
        // 机械加速度，这样机械方程用的是本拍刚更新的电流与转矩。
        // Torque first, then the mechanical equation uses this sample's torque.
        self.state.electromagnetic_torque_nm = 1.5
            * self.parameters.pole_pairs as f32
            * (self.parameters.flux_linkage_wb * self.state.iq_a
                + (self.parameters.ld_h - self.parameters.lq_h)
                    * self.state.id_a
                    * self.state.iq_a);
        // 机械方程：`J*dwe_m/dt = Te - T_load - B*we_m`，粘滞摩擦按当前转速线性
        // 扣减；注意这里用的是**机械**角速度，电角速度只在电压方程里出现。
        // `J*dw/dt = Te - T_load - B*w`, with mechanical angular velocity.
        let acceleration = (self.state.electromagnetic_torque_nm
            - self.load_torque_nm
            - self.parameters.viscous_friction_nm_s * self.state.mechanical_speed_rad_s)
            / self.parameters.inertia_kg_m2;
        self.state.mechanical_speed_rad_s += acceleration * dt_s;
        // 机械角度用刚更新的转速积分（半隐式欧拉的一拍），并环回 `[0, 2π)` 防止
        // 长时间仿真累积出浮点精度损失。
        // The angle integrates the freshly updated speed and wraps to `[0, 2π)`.
        self.state.mechanical_angle_rad = (self.state.mechanical_angle_rad
            + self.state.mechanical_speed_rad_s * dt_s)
            .rem_euclid(2.0 * PI);
    }
}

/// 面向全部硬件端口的 PC 假实现（`FeedbackPort`/`PwmPort`/`SafetyPort`）。
/// PC fake for every hardware-facing control port. The ideal rotor feedback is
/// intentional: it isolates controller/plant behavior from observer tuning.
/// `BemfPllEstimator` can be inserted at this port in observer-specific tests.
///
/// 理想转子反馈（有意为之）/ Ideal rotor feedback (intentional):
///   反馈端口直接返回被控对象真值（电角度、机械角速度、电流、母线电压），
///   把控制器/被控对象行为与观测器整定解耦，因此
///   `run_reference_simulation` 只是控制律回归，不能用来验证无感观测器。
///   需要观测器场景时在本端口插入 `BemfPllEstimator`，或改用
///   `run_bringup_simulation` 走固件 SMO 路径。
///
/// 故障注入与安全语义 / Fault injection and safety:
///   `inject_fault` 模拟功率级硬件故障（`PowerStageFault`），
///   `set_feedback_available(false)` 模拟反馈丢失（`FeedbackUnavailable`），
///   `set_reject_output(true)` 模拟输出被拒绝（`OutputRejected`）。三条失败路径
///   都必须让 PWM 关断：`apply_pwm` 在任一拒绝条件下都先调用 `disable_pwm()`
///   再返回 `Err`，而 `disable_pwm` 把占空比复位为 `PwmCommand::default()`
///   （三路 0.5，差模电压为零）并清 `pwm_enabled`。**任何失败路径都不允许留下
///   使能的功率级**，这与实机 break/关断语义一致。
///   Every failure path must leave the power stage disabled; `apply_pwm` calls
///   `disable_pwm()` before returning `Err`.
///
/// 量纲 / Units: `dc_bus_voltage` `[V]`、`peak_phase_current_a` `[A]`、被控对象
///   状态见 `PlantState`。全 `f32`，无 Q 格式。
pub struct SimHardware {
    /// 被控对象（电机 + 负载），仿真的唯一物理状态来源。
    plant: PmsmPlant,
    /// 母线电压 `[V]`，默认取 `MotorParameters::nominal_bus_voltage_v`。
    dc_bus_voltage: f32,
    /// 最近一次被接受的占空比指令，用于重建施加电压。
    applied_pwm: PwmCommand,
    /// PWM 使能标志；`false` 时 `advance` 施加零电压而不是保持最后指令。
    pwm_enabled: bool,
    /// 功率级故障锁存：置位后不会自动清除，只能重建实例。
    faulted: bool,
    /// 反馈可用性开关，模拟 ADC/位置反馈丢失。
    feedback_available: bool,
    /// 输出拒绝开关，模拟功率级/保护逻辑拒绝占空比指令。
    reject_output: bool,
    /// 全程相电流峰值锁存 `[A]`，只增不减。
    peak_phase_current_a: f32,
}

impl SimHardware {
    /// 用给定电机参数构造假硬件：母线取额定值、PWM 关断、无故障、反馈可用。
    /// Builds the fake hardware with nominal bus voltage and PWM disabled.
    pub fn new(parameters: MotorParameters) -> Self {
        Self {
            plant: PmsmPlant::new(parameters),
            dc_bus_voltage: parameters.nominal_bus_voltage_v,
            applied_pwm: PwmCommand::default(),
            pwm_enabled: false,
            faulted: false,
            feedback_available: true,
            reject_output: false,
            peak_phase_current_a: 0.0,
        }
    }

    /// 推进被控对象一个固定步长，并更新相电流峰值锁存。
    /// Advances the plant by one fixed step and updates the peak-current latch.
    ///
    /// 参数 / Parameters: `dt_s` 积分步长 `[s]`，必须与控制器周期一致（本工程
    ///   `1/12000 s`），否则控制律与对象的时间基准不一致，指标失去意义。
    ///
    /// 安全语义 / Safety: `pwm_enabled == false` 时施加**零 αβ 电压**，而不是
    ///   保持最后占空比；这样关断后电机自然减速，方向与实机断栅极一致（但本模型
    ///   没有二极管续流路径，减速过程是理想的自由惰行）。
    /// When PWM is disabled a zero αβ voltage is applied instead of holding the
    /// last duty command.
    ///
    /// 峰值锁存 / Peak latch: 逐相取 `abs()` 的最大值，只增不减，并且**每步都
    ///   更新**（不是只在 CSV 抽点时刻），因此能捕获抽点 trace 漏掉的高频峰值，
    ///   与固件的高速峰值锁存同口径。
    pub fn advance(&mut self, dt_s: f32) {
        let voltage = if self.pwm_enabled {
            pwm_to_alpha_beta(self.applied_pwm, self.dc_bus_voltage)
        } else {
            foc_algorithm::AlphaBeta::default()
        };
        self.plant.step(voltage, dt_s);
        let currents = self.plant.phase_currents();
        self.peak_phase_current_a = self
            .peak_phase_current_a
            .max(currents.a.abs())
            .max(currents.b.abs())
            .max(currents.c.abs());
    }

    /// 只读访问被控对象（取真值、算指标、读负载转矩）。
    /// Read-only access to the plant.
    pub fn plant(&self) -> &PmsmPlant {
        &self.plant
    }

    /// 可变访问被控对象；仿真脚本用它注入负载转矩阶跃，测试用它改真值。
    /// Mutable access to the plant, used to inject load-torque steps.
    pub fn plant_mut(&mut self) -> &mut PmsmPlant {
        &mut self.plant
    }

    /// 机械转速 `[rpm]`：`[rad/s] × 30 / π`（每转 `2π` 弧度、每分钟 60 秒）。
    /// Mechanical speed in `[rpm]` converted from `[rad/s]`.
    pub fn speed_rpm(&self) -> f32 {
        self.plant.state().mechanical_speed_rad_s * 30.0 / PI
    }

    /// 全程相电流峰值 `[A]`（只增不减），对标固件的高速峰值锁存。
    /// Peak phase current in `[A]`, monotonically latched.
    pub fn peak_phase_current_a(&self) -> f32 {
        self.peak_phase_current_a
    }

    /// 注入功率级故障锁存：此后 `read_feedback` 返回 `PowerStageFault`，
    /// `apply_pwm` 拒绝并关断 PWM，`power_stage_faulted()` 为真。
    /// Latches a power-stage fault; every later port call fails and PWM is off.
    ///
    /// 该锁存**不可清除**：需要恢复正常行为必须新建实例，这与实机故障锁存的
    /// 单向性一致，避免测试里"清了故障但功率级仍在使能"的假通过。
    /// The latch is one-way, like the hardware fault latch.
    pub fn inject_fault(&mut self) {
        self.faulted = true;
    }

    /// 模拟反馈丢失：置 `false` 后 `read_feedback` 返回 `FeedbackUnavailable`，
    /// 控制器随后必须关断 PWM。
    /// Simulates lost feedback; the controller must then disable PWM.
    pub fn set_feedback_available(&mut self, available: bool) {
        self.feedback_available = available;
    }

    /// 模拟输出被拒绝（如硬件保护动作）：置位后任何占空比指令都返回
    /// `OutputRejected` 并关断 PWM。
    /// Simulates a rejected output command (for example a protective trip).
    pub fn set_reject_output(&mut self, reject: bool) {
        self.reject_output = reject;
    }

    /// 当前 PWM 使能状态；任何故障路径之后必须为 `false`，测试据此断言安全语义。
    /// Current PWM enable state; must be `false` after any fault.
    pub fn pwm_enabled(&self) -> bool {
        self.pwm_enabled
    }

    /// 最近一次被接受的占空比指令；被拒绝或关断后返回默认 `0.5/0.5/0.5`。
    /// Last accepted duty command; the default after a reject or disable.
    pub fn applied_pwm(&self) -> PwmCommand {
        self.applied_pwm
    }

    /// 仅供离线多速率调度器把“控制器刚写出的命令”替换成“本载波周期实际生效的
    /// 命令”。不会改变使能/故障状态；目标平台不得调用此接口。
    fn set_scheduled_pwm(&mut self, command: PwmCommand) {
        self.applied_pwm = command;
    }

    /// 母线电压 `[V]`，是逆变器模型与占空比换算的电压基准。
    /// DC bus voltage in `[V]`, the reference for duty-to-voltage conversion.
    pub fn dc_bus_voltage(&self) -> f32 {
        self.dc_bus_voltage
    }
}

/// `FeedbackPort` 的假实现：返回理想（真值）同步快照。
/// The fake `FeedbackPort`: one ideal, synchronised snapshot per call.
///
/// 量纲 / Units: 电流 `[A]`、母线电压 `[V]`、电角度 `[rad]`、机械角速度
///   `[rad/s]`。三个电流来自同一次 `plant.phase_currents()`，天然无相间时间偏斜
///   （实机的顺序采样会有偏斜）。
///
/// 失败语义 / Failure semantics: 先查故障锁存再查反馈可用性，所以两者同时成立时
///   返回 `PowerStageFault`。返回 `Err` 时**本实现不碰 PWM**，关断由控制器在错误
///   路径上调用 `PwmPort::disable_pwm()` 完成（`ControlRuntime::tick` 的每条失败
///   分支都会走到 `disable()`）。
/// Fault is checked before availability; disabling PWM is the caller's job.
impl FeedbackPort for SimHardware {
    fn read_feedback(&mut self) -> Result<FeedbackSnapshot, HardwareFault> {
        if self.faulted {
            return Err(HardwareFault::PowerStageFault);
        }
        if !self.feedback_available {
            return Err(HardwareFault::FeedbackUnavailable);
        }
        Ok(FeedbackSnapshot {
            currents: self.plant.phase_currents(),
            dc_bus_voltage: self.dc_bus_voltage,
            rotor: RotorFeedback {
                electrical_angle_rad: self.plant.electrical_angle_rad(),
                mechanical_speed_rad_s: self.plant.state().mechanical_speed_rad_s,
            },
        })
    }
}

/// `PwmPort` 的假实现：唯一的功率级写入口。
/// The fake `PwmPort`, the only entry that writes to the power stage.
///
/// 安全语义 / Safety: 成功才使能；故障、`reject_output`、或占空比非法
///   （`PwmCommand::is_valid()` 为假，即非有限或越出 `[0,1]`）三条拒绝路径都先
///   `disable_pwm()` 再返回 `Err(OutputRejected)`，因此不存在"报错但功率级仍在
///   使能"的中间态。
/// Rejection always disables the stage before returning `Err`.
impl PwmPort for SimHardware {
    fn apply_pwm(&mut self, command: PwmCommand) -> Result<(), HardwareFault> {
        if self.faulted || self.reject_output || !command.is_valid() {
            self.disable_pwm();
            return Err(HardwareFault::OutputRejected);
        }
        self.applied_pwm = command;
        self.pwm_enabled = true;
        Ok(())
    }

    /// 关断 PWM：占空比复位为 `PwmCommand::default()`（0.5/0.5/0.5，差模电压为
    /// 零）并清 `pwm_enabled`，保证关机后 `applied_pwm()` 不会残留旧指令。
    /// Disables PWM, restoring the default duty and clearing the enable flag.
    fn disable_pwm(&mut self) {
        self.applied_pwm = PwmCommand::default();
        self.pwm_enabled = false;
    }
}

/// `SafetyPort` 的假实现：异步保护状态查询（等价于实机 break/比较器故障标志）。
/// The fake `SafetyPort`: asynchronous protection status (break/comparator trip).
///
/// 返回的是锁存的 `faulted` 位，与 `read_feedback` 使用同一状态，因此控制器
///   在故障后的第一个 `tick` 就能在读取反馈之前直接进入关断路径。
/// Returns the same latched flag that `read_feedback` checks.
impl SafetyPort for SimHardware {
    fn power_stage_faulted(&self) -> bool {
        self.faulted
    }
}

/// `run_reference_simulation` 的场景配置（理想反馈控制律回归）。
/// Scenario configuration for `run_reference_simulation`.
///
/// 量纲 / Units: `duration_s` `[s]`、`target_speed_rpm` `[rpm]`、
///   `load_step_time_s` `[s]`、`load_torque_nm` `[N*m]`；`trace_decimation` 是
///   抽点用的控制拍数（无量纲计数，`1` 表示每拍都记录）。
///   SI/engineering units; `trace_decimation` counts control ticks.
#[derive(Clone, Copy, Debug)]
pub struct SimulationConfig {
    /// 仿真时长 `[s]`；必须有限且为正，步数由 `round(duration_s / dt)` 得到。
    /// Total simulated time in `[s]`; must be finite and positive.
    pub duration_s: f32,
    /// 速度环目标转速 `[rpm]`；必须有限。本入口不对目标做额外限幅，超出电机能力时
    /// 表现为电压限幅与跟踪误差变大，而不是报错。
    /// Speed-loop target in `[rpm]`; must be finite, not range-limited here.
    pub target_speed_rpm: f32,
    /// 负载阶跃施加时刻 `[s]`；必须有限且非负，按 `round(t / dt)` 取整到控制拍。
    /// Load-step instant in `[s]`, quantised to a control tick.
    pub load_step_time_s: f32,
    /// 阶跃后施加的负载转矩 `[N*m]`；必须有限，正值为阻力矩。
    /// Load torque in `[N*m]` applied from the step instant onwards.
    pub load_torque_nm: f32,
    /// trace 抽点间隔（控制拍数）；必须非零，否则取模与除法会直接 panic。
    /// Trace decimation in control ticks; zero would panic on `%` and `/`.
    pub trace_decimation: u32,
    /// PWM/控制分频与输出生效延迟；默认 12/12 kHz、零额外延迟，保持旧行为。
    pub timing: MultiRateTimingConfig,
}

/// 默认场景：3 s、目标 524 rpm、1 s 处 0.004 N*m 负载阶跃、每 10 拍记录一次。
/// Defaults: 3 s, 524 rpm, a 0.004 N*m load step at 1 s, decimation 10.
///
/// 来源 / Provenance: `524` 是 `[ST]` 参考工程默认目标转速
///   （`ControlParameters::default_target_speed_rpm`），不是台架辨识值；
///   `load_torque_nm = 0.004` 的来源未在 `docs/` 中标注。
///   `524` comes from the ST reference defaults; the load value is undocumented.
impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            duration_s: 3.0,
            target_speed_rpm: 524.0,
            load_step_time_s: 1.0,
            load_torque_nm: 0.004,
            trace_decimation: 10,
            timing: MultiRateTimingConfig::default(),
        }
    }
}

/// `run_reference_simulation` 的一行 trace，与 CSV 一行逐列对应。
/// One trace row of `run_reference_simulation`, one CSV line.
///
/// 量纲 / Units: 时间 `[s]`、转速 `[rpm]`、电流 `[A]`、电压 `[V]`、角度 `[rad]`、
///   转矩 `[N*m]`，占空比无量纲 `[0,1]`。全部是控制器/被控对象真值，不含 ADC
///   量化、噪声、通信截断或丢包，因此字段精度高于实机 trace。
/// All values are controller/plant values without ADC or link artifacts.
#[derive(Clone, Copy, Debug, Default)]
pub struct SimulationSample {
    /// 本样本对应的仿真时刻 `[s]`，由控制拍号乘步长得到（非实测时间）。
    /// Sample time in `[s]` from the tick index, not a measured timestamp.
    pub time_s: f32,
    /// 速度环目标转速 `[rpm]`（整段恒定，便于与实机目标对齐）。
    /// Speed target in `[rpm]`; constant over the run.
    pub target_speed_rpm: f32,
    /// 速度反馈 `[rpm]`；本入口是理想真值，因此等于被控对象实际转速。
    /// Measured speed in `[rpm]`; ideal ground truth in this entry point.
    pub measured_speed_rpm: f32,
    /// A 相电流 `[A]`。
    /// Phase A current in `[A]`.
    pub phase_current_a: f32,
    /// B 相电流 `[A]`。
    /// Phase B current in `[A]`.
    pub phase_current_b: f32,
    /// C 相电流 `[A]`。
    /// Phase C current in `[A]`.
    pub phase_current_c: f32,
    /// d 轴电流给定 `[A]`；本入口固定为 0（表贴式电机不弱磁）。
    /// d-axis current reference in `[A]`, zero in this entry point.
    pub id_ref_a: f32,
    /// q 轴电流给定 `[A]`，即速度环输出。
    /// q-axis current reference in `[A]`, the speed-loop output.
    pub iq_ref_a: f32,
    /// d 轴电流实测 `[A]`。
    /// Measured d-axis current in `[A]`.
    pub id_a: f32,
    /// q 轴电流实测 `[A]`。
    /// Measured q-axis current in `[A]`.
    pub iq_a: f32,
    /// 电流环 d 轴输出电压 `[V]`（限幅后）。
    /// Current-loop d-axis voltage command in `[V]`, after limiting.
    pub vd_v: f32,
    /// 电流环 q 轴输出电压 `[V]`（限幅后）。
    /// Current-loop q-axis voltage command in `[V]`, after limiting.
    pub vq_v: f32,
    /// 逆 Park 后的 α 轴电压 `[V]`，SVPWM 的输入。
    /// α-axis voltage in `[V]` after inverse Park, the SVPWM input.
    pub valpha_v: f32,
    /// 逆 Park 后的 β 轴电压 `[V]`。
    /// β-axis voltage in `[V]` after inverse Park.
    pub vbeta_v: f32,
    /// A 相占空比，无量纲 `[0,1]`。
    /// Phase A duty ratio in `[0,1]`.
    pub duty_a: f32,
    /// B 相占空比，无量纲 `[0,1]`。
    /// Phase B duty ratio in `[0,1]`.
    pub duty_b: f32,
    /// C 相占空比，无量纲 `[0,1]`。
    /// Phase C duty ratio in `[0,1]`.
    pub duty_c: f32,
    /// 本拍母线电压 `[V]`；本入口恒为额定值，实机会有纹波。
    /// DC bus voltage in `[V]`; constant here, rippled on hardware.
    pub dc_bus_voltage_v: f32,
    /// 被控对象当前负载转矩 `[N*m]`（标注阶跃生效的样本）。
    /// Plant load torque in `[N*m]`, marking where the step took effect.
    pub load_torque_nm: f32,
    /// 被控对象电磁转矩 `[N*m]`（被控对象真值，不是控制器估计）。
    /// Plant electromagnetic torque in `[N*m]`, a ground-truth value.
    pub electromagnetic_torque_nm: f32,
    /// 被控对象电角度 `[rad]`，范围 `[0, 2π)`。
    /// Plant electrical angle in `[rad]`, in `[0, 2π)`.
    pub electrical_angle_rad: f32,
    /// 电压圆限幅是否生效；`true` 表示本拍 PI 输出被限幅，跟踪会变差。
    /// Whether the voltage circle limit bound this sample.
    pub voltage_limited: bool,
    /// 本拍 PWM 是否使能；故障后为 `false`。
    /// Whether PWM was enabled for this sample.
    pub pwm_enabled: bool,
}

/// `run_reference_simulation` 的汇总指标（末端值 + 峰值 + 样本数）。
/// Summary of `run_reference_simulation`: final values, peak and sample count.
///
/// 量纲 / Units: 转速 `[rpm]`、峰值电流 `[A]`、负载转矩 `[N*m]`，样本数为计数。
///
/// 与 `BringupSimulationSummary` 的分工 / Division of labour: 这里只有末端量与
///   峰值，没有按时间窗的 RMSE；带时间窗的统计指标只在 bringup 入口里计算，
///   因此两条入口的汇总字段不可互换。
/// Only end-point and peak values here; windowed RMSE lives in the bringup summary.
#[derive(Clone, Copy, Debug, Default)]
pub struct SimulationSummary {
    /// 仿真最后一拍的转速 `[rpm]`。
    /// Speed at the last tick in `[rpm]`.
    pub final_speed_rpm: f32,
    /// 目标转速 `[rpm]`（回显配置，便于与 `final_speed_rpm` 直接求差）。
    /// Target speed in `[rpm]`, echoed for direct comparison.
    pub target_speed_rpm: f32,
    /// 全程相电流峰值 `[A]`（峰值锁存，见 `SimHardware::advance`）。
    /// Peak phase current in `[A]` over the whole run, from the peak latch.
    pub peak_phase_current_a: f32,
    /// 本场景使用的负载转矩 `[N*m]`（阶跃后的值）。
    /// Load torque in `[N*m]` used by the scenario.
    pub load_torque_nm: f32,
    /// 记录到的 trace 样本数（抽点后，不是控制拍数）。
    /// Number of trace samples after decimation, not control ticks.
    pub sample_count: usize,
    /// 实际推进的 PWM 拍数。
    pub pwm_tick_count: u64,
    /// 实际执行的控制拍数。
    pub control_tick_count: u64,
    /// 已在载波边界生效的占空更新数。
    pub applied_pwm_update_count: u64,
}

/// `run_reference_simulation` 的失败原因。
/// Failure reasons of `run_reference_simulation`.
///
/// 语义 / Semantics: `InvalidConfig` 只携带一条静态说明（哪一项配置非法），
///   `Hardware` 透传 `foc-control` 的端口故障。两种错误都发生在推进状态之前或
///   关断之后，因此不会留下"半推进"的仿真结果。
/// `InvalidConfig` carries a static reason string; `Hardware` forwards a port fault.
#[derive(Debug)]
pub enum SimulationError {
    /// 配置非法（时长/目标转速/负载时刻/负载转矩/抽点之一不满足约束）。
    /// Invalid configuration.
    InvalidConfig(&'static str),
    /// 硬件端口故障；此时 PWM 已被控制器关断。
    /// A hardware port fault; PWM has already been disabled.
    Hardware(HardwareFault),
}

/// 把端口故障转换为仿真错误，使 `runtime.tick(...)?` 能直接传播。
/// Converts a port fault so `runtime.tick(...)?` can propagate it.
impl From<HardwareFault> for SimulationError {
    fn from(value: HardwareFault) -> Self {
        Self::Hardware(value)
    }
}

/// 一次参考仿真的完整结果。
/// The full result of one reference simulation.
///
/// 实时约束 / Real-time note: `samples` 是 `Vec`，只在离线 PC 代码里合法；固件
///   trace 用定长环形缓冲，ISR 内不做分配。不要把这里的结构搬进快环。
/// `Vec` is fine here and must never appear in the realtime path.
#[derive(Debug)]
pub struct SimulationRun {
    /// 抽点后的 trace 样本，按时间升序。
    /// Decimated trace samples in ascending time order.
    pub samples: Vec<SimulationSample>,
    /// 汇总指标，见 `SimulationSummary`。
    /// Summary metrics, see `SimulationSummary`.
    pub summary: SimulationSummary,
}

impl SimulationRun {
    /// 把 trace 写成 CSV：一行表头 + 每样本一行，`,` 分隔，数值用 Rust 默认的
    /// `f32` 十进制格式（最短往返表示）。
    /// Writes the trace as CSV with one header row and one row per sample.
    ///
    /// 列定义 / Columns（顺序即 `SimulationSample` 的字段顺序）: `time_s` `[s]`、
    ///   `target_speed_rpm` 与 `measured_speed_rpm` `[rpm]`、三相电流 `[A]`、
    ///   `id_ref_a`/`iq_ref_a`/`id_a`/`iq_a` `[A]`、`vd_v`/`vq_v`/`valpha_v`/
    ///   `vbeta_v` `[V]`、三相占空比 `[0,1]`、`dc_bus_voltage_v` `[V]`、
    ///   `load_torque_nm`/`electromagnetic_torque_nm` `[N*m]`、
    ///   `electrical_angle_rad` `[rad]`，末尾 `voltage_limited` 与 `pwm_enabled`
    ///   写成 `0`/`1`。
    ///   The header mirrors the sample struct field order, ending with two flags.
    ///
    /// 契约 / Contract: 列名与顺序是 `simulation/compare_traces.py` 与 MATLAB
    ///   绘图脚本的输入契约，改名或换序会让对标脚本失配。
    ///   Column names and order are a contract for the comparison scripts.
    ///
    /// 失败语义 / Failure: 创建、写入与 `flush` 的 I/O 错误原样上抛，此时文件可能
    ///   只写了一部分（没有临时文件 + 原子改名）。
    /// I/O errors propagate as-is; a partially written file is possible.
    pub fn write_csv(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let file = File::create(path)?;
        let mut writer = BufWriter::new(file);
        writeln!(
            writer,
            "time_s,target_speed_rpm,measured_speed_rpm,phase_current_a,phase_current_b,phase_current_c,id_ref_a,iq_ref_a,id_a,iq_a,vd_v,vq_v,valpha_v,vbeta_v,duty_a,duty_b,duty_c,dc_bus_voltage_v,load_torque_nm,electromagnetic_torque_nm,electrical_angle_rad,voltage_limited,pwm_enabled"
        )?;
        for sample in &self.samples {
            writeln!(
                writer,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                sample.time_s,
                sample.target_speed_rpm,
                sample.measured_speed_rpm,
                sample.phase_current_a,
                sample.phase_current_b,
                sample.phase_current_c,
                sample.id_ref_a,
                sample.iq_ref_a,
                sample.id_a,
                sample.iq_a,
                sample.vd_v,
                sample.vq_v,
                sample.valpha_v,
                sample.vbeta_v,
                sample.duty_a,
                sample.duty_b,
                sample.duty_c,
                sample.dc_bus_voltage_v,
                sample.load_torque_nm,
                sample.electromagnetic_torque_nm,
                sample.electrical_angle_rad,
                u8::from(sample.voltage_limited),
                u8::from(sample.pwm_enabled),
            )?;
        }
        writer.flush()
    }
}

/// 理想转子反馈的控制律回归：`ControlRuntime` + `SimHardware`，被控对象真值直接
/// 当作转子反馈。
/// Runs the same Rust controller and PMSM plant used by unit tests while
/// collecting a decimated trace suitable for MATLAB, Python, or spreadsheets.
///
/// 定位（重要）/ Scope (important):
///   转子角度与转速来自 `SimHardware::read_feedback` 的**理想真值**：没有 SMO/PLL、
///   没有 Rev-Up、没有对齐过程。因此本入口验证的是 PI 增益、SVPWM、PMSM 方程与
///   故障接口，属于**控制律回归**；它**不是**无感/观测器相关性验证，不能用它的
///   结果评价观测器。要与固件逐步对标必须用 `run_bringup_simulation`。
///   Ideal rotor feedback makes this a control-law regression only, not a
///   sensorless/observer correlation run.
///
/// 参数 / Parameters: `config` 为场景配置。非法项各自返回一条
///   `SimulationError::InvalidConfig`：时长必须有限且为正、目标转速必须有限、
///   负载阶跃时刻必须有限且非负、负载转矩必须有限、抽点必须非零。校验全部在
///   构造运行时之前完成，所以非法配置不会推进或留下任何状态。
///
/// 时间基准与量纲 / Time base and units: plant 步长为 `1 / pwm_frequency_hz`，
///   控制链按 `control_frequency_hz` 的整数分频执行；默认两者都是 12 kHz。
///   负载阶跃量化到 PWM 拍，因此实际时刻存在最多半个 PWM 周期误差。
///   全部为 SI 单位，见 `SimulationSample`；无定点换算。
///
/// 循环顺序 / Loop order: 每个 PWM 拍先让到期命令生效并施加负载阶跃；控制分频点
///   才 `tick` 一次控制律并按控制拍抽点；最后推进被控对象。记录发生在对象推进之前，
///   所以样本里的电流/角度与同一拍的控制器输入是**同一时刻**的量，可以直接
///   与实机同一拍的 trace 对齐。
///
/// 实时约束 / Real-time note: 离线 PC 代码，`Vec::with_capacity` 与 `?` 传播都
///   可以用；目标固件的对应路径在 12 kHz ADC ISR 里，禁止分配与日志。
/// Offline only: the `Vec` trace and per-tick bookkeeping never belong in an ISR.
pub fn run_reference_simulation(
    config: SimulationConfig,
) -> Result<SimulationRun, SimulationError> {
    if !config.duration_s.is_finite() || config.duration_s <= 0.0 {
        return Err(SimulationError::InvalidConfig(
            "duration_s must be positive",
        ));
    }
    if !config.target_speed_rpm.is_finite() {
        return Err(SimulationError::InvalidConfig(
            "target_speed_rpm must be finite",
        ));
    }
    if !config.load_step_time_s.is_finite() || config.load_step_time_s < 0.0 {
        return Err(SimulationError::InvalidConfig(
            "load_step_time_s must be non-negative",
        ));
    }
    if !config.load_torque_nm.is_finite() {
        return Err(SimulationError::InvalidConfig(
            "load_torque_nm must be finite",
        ));
    }
    if config.trace_decimation == 0 {
        return Err(SimulationError::InvalidConfig(
            "trace_decimation must be non-zero",
        ));
    }

    let params = foc_control::st_gbm2804_reference_parameters();
    if config.timing.control_frequency_hz != params.pwm_frequency_hz {
        return Err(SimulationError::InvalidConfig(
            "control_frequency_hz must match the controller configuration",
        ));
    }
    let pwm_ticks_per_control = config
        .timing
        .validate()
        .map_err(|_| SimulationError::InvalidConfig("invalid multi-rate timing"))?;
    let pwm_dt = config.timing.pwm_period_s();
    let total_pwm_steps = (config.duration_s / pwm_dt).round() as u32;
    let total_control_steps = total_pwm_steps.div_ceil(pwm_ticks_per_control);
    let load_step = (config.load_step_time_s / pwm_dt).round() as u32;
    // 容量按抽点估算并多留 2 行（末拍无条件记录）；只是性能优化，超出会自动扩容。
    // Capacity is an estimate only; the final tick is always recorded.
    let capacity = (total_control_steps / config.trace_decimation + 2) as usize;
    let mut samples = Vec::with_capacity(capacity);
    let mut runtime = foc_control::ControlRuntime::new(
        foc_control::StReferenceController::new(params),
        SimHardware::new(params.motor),
    );
    runtime.enable()?;
    let mut scheduler = MultiRateScheduler::new(config.timing)
        .map_err(|_| SimulationError::InvalidConfig("invalid multi-rate timing"))?;

    for pwm_tick in 0..total_pwm_steps {
        let control_due = scheduler.begin_pwm_tick();
        // 负载阶跃只在恰好等于取整拍号时施加一次；由于是等值比较，若
        // `load_step >= total_steps`（阶跃晚于仿真结束）则整段都不会施加。
        // The load step is applied once, only when the tick index matches exactly.
        if pwm_tick == load_step {
            runtime
                .hardware_mut()
                .plant_mut()
                .set_load_torque_nm(config.load_torque_nm);
        }
        if control_due {
            let control_tick = scheduler.control_tick_count() as u32;
            let telemetry = runtime.tick(foc_control::SpeedCommand {
                target_rpm: config.target_speed_rpm,
                id_ref_a: 0.0,
            })?;
            let commanded_pwm = runtime.hardware().applied_pwm();
            scheduler.submit_control_output(commanded_pwm);
            runtime
                .hardware_mut()
                .set_scheduled_pwm(scheduler.active_pwm());

            // trace 按控制拍抽点；占空列记录本载波周期实际生效的值，而不是尚在等待
            // 延迟边界的命令。
            if control_tick.is_multiple_of(config.trace_decimation)
                || (control_tick + 1 == total_control_steps)
            {
                let hardware = runtime.hardware();
                let currents = hardware.plant().phase_currents();
                let plant_state = hardware.plant().state();
                let pwm = scheduler.active_pwm();
                samples.push(SimulationSample {
                    time_s: pwm_tick as f32 * pwm_dt,
                    target_speed_rpm: config.target_speed_rpm,
                    measured_speed_rpm: telemetry.measured_speed_rpm,
                    phase_current_a: currents.a,
                    phase_current_b: currents.b,
                    phase_current_c: currents.c,
                    id_ref_a: telemetry.current_reference.id_ref_a,
                    iq_ref_a: telemetry.current_reference.iq_ref_a,
                    id_a: telemetry.current_dq.d,
                    iq_a: telemetry.current_dq.q,
                    vd_v: telemetry.voltage_dq.d,
                    vq_v: telemetry.voltage_dq.q,
                    valpha_v: telemetry.voltage_alpha_beta.alpha,
                    vbeta_v: telemetry.voltage_alpha_beta.beta,
                    duty_a: pwm.duty_a,
                    duty_b: pwm.duty_b,
                    duty_c: pwm.duty_c,
                    dc_bus_voltage_v: hardware.dc_bus_voltage(),
                    load_torque_nm: hardware.plant().load_torque_nm(),
                    electromagnetic_torque_nm: plant_state.electromagnetic_torque_nm,
                    electrical_angle_rad: hardware.plant().electrical_angle_rad(),
                    voltage_limited: telemetry.voltage_limited,
                    pwm_enabled: hardware.pwm_enabled(),
                });
            }
        } else {
            runtime
                .hardware_mut()
                .set_scheduled_pwm(scheduler.active_pwm());
        }
        // 先记录再推进对象：这样一行 trace 里的控制器量与对象真值属于同一拍，
        // 不会出现"控制器量已更新、对象还停在上一拍"的错位。
        // Trace before advancing the plant so every row is one consistent tick.
        runtime.hardware_mut().advance(pwm_dt);
        scheduler.finish_pwm_tick();
    }

    Ok(SimulationRun {
        summary: SimulationSummary {
            final_speed_rpm: runtime.hardware().speed_rpm(),
            target_speed_rpm: config.target_speed_rpm,
            peak_phase_current_a: runtime.hardware().peak_phase_current_a(),
            load_torque_nm: config.load_torque_nm,
            sample_count: samples.len(),
            pwm_tick_count: scheduler.pwm_tick_count(),
            control_tick_count: scheduler.control_tick_count(),
            applied_pwm_update_count: scheduler.applied_update_count(),
        },
        samples,
    })
}

/// 实机相关性仿真的配置：与 `run_reference_simulation` 不同，这里逐步执行
/// STM32G431 ADC 中断使用的同一套 C ABI 控制器入口，含 Rev-Up 与所选无感观测器。
/// Configuration for the correlation simulation. Unlike
/// `run_reference_simulation`, this executes the exact C-ABI controller entry
/// points used by the ADC interrupt on the STM32G431, including rev-up and the
/// selected sensorless observer.
///
/// 量纲 / Units: `duration_s` `[s]`、`target_speed_rpm` `[rpm]`、
///   `dc_bus_voltage_v` `[V]`、`load_torque_nm` `[N*m]`、`dead_time_ns` `[ns]`
///   （进入计算前换算为 `[s]`）、`observer_smo_k_slide_v` `[V]`（滑模切换项幅值）、
///   `observer_smo_boundary_a` `[A]`（电流误差饱和边界）、
///   `dead_time_current_zero_band_a` `[A]`（极性软零带）。
///   `observer_pll_kp` 为 `[1/s]`、`observer_pll_ki` 为 `[1/s^2]`（相位误差是经
///   `sin` 的无量纲量）。
///
/// 无量纲字段 / Dimensionless fields: `closed_loop_enable`、`trace_decimation`
///   （控制拍计数）、`observer_emf_filter_alpha`（`[0,1]` 一阶低通系数）、
///   `dead_time_compensation_gain`（前馈增益）、以及 `dead_time_enabled`、
///   `dead_time_feedforward_enabled`、`observer_dead_time_compensation_enabled`
///   三个开关。
///
/// ABI 边界 / ABI boundary: 参数进入 V9 运行配置；Host 与目标固件
///   因而执行同一个版本化安装路径。默认总门与两个出口仍全部关闭。
///   The switches and parameters use the V9 runtime configuration on both host
///   and target; every gate remains disabled by default.
///
/// 取值范围 / Validation ranges: 由 `run_bringup_simulation` 校验，见该函数的说明。
#[derive(Clone, Copy, Debug)]
pub struct BringupSimulationConfig {
    /// 仿真时长 `[s]`；必须有限且为正。
    /// Total simulated time in `[s]`; finite and positive.
    pub duration_s: f32,
    /// 目标转速 `[rpm]`；必须有限且落在固件允许的应用速度范围内。
    /// Target speed in `[rpm]`, within the firmware's allowed range.
    pub target_speed_rpm: f32,
    /// 仿真开始时的转子机械转速 `[rpm]`；非零值用于滑行/飞行启动策略门。
    /// Initial rotor mechanical speed in `[rpm]`; non-zero values exercise
    /// coast/flying-start policy rather than a stationary cold start.
    pub initial_speed_rpm: f32,
    /// 仿真开始时的转子电角度 `[rad]`，运行前环回到 `[0, 2π)`。
    /// Initial rotor electrical angle in `[rad]`, wrapped to `[0, 2π)`.
    pub initial_electrical_angle_rad: f32,
    /// 是否启用闭环接管；固件上电默认为 `false`（`[FW]`），闭环还要求观测器已
    /// 使能，因此不能单独打开。
    /// Enables closed-loop handover; requires the observer to be enabled.
    pub closed_loop_enable: bool,
    /// 转子对齐时长 `[s]`；默认为 CM4.4 实机通过值 2.0 s。
    /// Rotor-alignment duration in `[s]`; defaults to the 2.0 s CM4.4 value.
    pub startup_alignment_s: f32,
    /// 强拖升速时长 `[s]`；默认为 CM4.4 实机通过值 5.0 s。
    /// Forced rev-up ramp duration in `[s]`; defaults to 5.0 s.
    pub startup_ramp_s: f32,
    /// 无感获取使用的 Rev-Up 终速幅值 `[rpm]`；可以高于最终目标。
    /// Rev-up endpoint used for sensorless acquisition; it may exceed the target.
    pub startup_final_speed_rpm: f32,
    /// 对齐结束 q 轴电流 `[A]`；升速时平滑过渡到 `startup_current_a`。
    /// Alignment-end q-axis current `[A]`; tapers to `startup_current_a` in Rev-Up.
    pub startup_alignment_current_a: f32,
    /// 强拖 q 轴电流 `[A]`；不得超过电机额定电流。
    /// Ramp-end and hold q-axis current in `[A]`; must not exceed rated current.
    pub startup_current_a: f32,
    /// 母线电压 `[V]`；必须有限且为正，用作逆变器模型与固件反馈的母线值。
    /// DC bus voltage in `[V]`; finite and positive.
    pub dc_bus_voltage_v: f32,
    /// 恒定负载转矩 `[N*m]`（本入口不做阶跃）；必须有限。
    /// Constant load torque in `[N*m]`; finite.
    pub load_torque_nm: f32,
    /// trace 抽点间隔（控制拍数）；必须非零。
    /// Trace decimation in control ticks; must be non-zero.
    pub trace_decimation: u32,
    /// PWM/控制分频与输出生效延迟；只属于 PC 仿真，不进入固件 C ABI。
    pub timing: MultiRateTimingConfig,
    /// 电流 Park 相对基础控制角的预测量 `[control ticks]`；范围 `[-2, 2]`。
    /// Current-Park prediction from the base angle `[control ticks]`.
    pub park_prediction_ticks: f32,
    /// 电压逆 Park 相对基础控制角的预测量 `[control ticks]`；范围 `[-2, 2]`。
    /// Voltage inverse-Park prediction from the base angle `[control ticks]`.
    pub reverse_park_prediction_ticks: f32,
    /// 滑模观测器切换项幅值 `[V]`；必须有限且大于 0，且要大于反电势幅值才可达。
    /// Sliding-mode switching amplitude in `[V]`; must be positive.
    pub observer_smo_k_slide_v: f32,
    /// 滑模电流误差饱和边界 `[A]`；必须有限且大于 0，越大越平滑但越"软"。
    /// Current-error saturation boundary in `[A]`; must be positive.
    pub observer_smo_boundary_a: f32,
    /// 反电势一阶低通系数，无量纲；校验区间 `[0.0001, 1.0]`，`1.0` 不滤波。
    /// EMF low-pass coefficient, dimensionless; validated in `[0.0001, 1.0]`.
    pub observer_emf_filter_alpha: f32,
    /// 角度环 PLL 比例增益 `[1/s]`；必须有限且不小于 0。
    /// PLL proportional gain in `[1/s]`; finite and non-negative.
    pub observer_pll_kp: f32,
    /// 强拖捕获期 PLL Kp/Ki 校正比例 `(0,1]`；不缩放终速前馈。
    /// Acquisition PLL Kp/Ki correction ratio `(0,1]`; speed feed-forward is kept.
    pub observer_acquisition_pll_kp_ratio: f32,
    /// 角度环 PLL 积分增益 `[1/s^2]`；必须有限且不小于 0。
    /// PLL integral gain in `[1/s^2]`; finite and non-negative.
    pub observer_pll_ki: f32,
    /// 启动获取窗允许的平均绝对包角相位误差 `[rad]`，范围 `(0,pi/2]`。
    /// Maximum mean absolute wrapped phase error over the acquisition window.
    pub observer_acquisition_maximum_phase_error_rad: f32,
    /// 闭环保持允许的最大原始包角相位误差 `[rad]`，范围 `(0,pi/2]`。
    /// Maximum raw wrapped phase error retained in closed loop `(0,pi/2]`.
    pub observer_run_maximum_phase_error_rad: f32,
    /// 接管前全部可靠性门连续成立的 1 kHz 窗口数（数值也就是毫秒数）。
    /// Consecutive 1 kHz reliability windows required before handover.
    pub observer_consecutive_samples: u32,
    /// 开环保持等待观测器收敛的超时 `[s]`。
    /// Open-loop-hold observer acquisition timeout `[s]`.
    pub observer_acquisition_timeout_s: f32,
    /// 观测器已接管角度后允许连续失锁的超时 `[s]`。
    /// Continuous observer-loss timeout after the estimator owns the angle `[s]`.
    pub observer_loss_timeout_s: f32,
    /// 强拖角切换到观察器角的渐变时长 `[s]`。
    /// Blend duration from forced angle to observer angle `[s]`.
    pub startup_transition_s: f32,
    /// 接管转矩支撑比例 `[--]`，0 使用观测器坐标系实测 Iq，1 保持开环转矩。
    /// Handoff torque-support ratio `[--]`.
    pub handoff_torque_support_ratio: f32,
    /// 速度 PI 积分器预装比例 `[--]`；与接管转矩支撑独立。
    /// Speed-PI integrator preload ratio `[--]`, independent of torque support.
    pub speed_pi_preload_ratio: f32,
    /// 闭环 Iq 给定限速 `[A/s]`。
    /// Closed-loop Iq command slew rate `[A/s]`.
    pub closed_loop_current_slew_a_per_s: f32,
    /// 死区模型总开关；关闭时两个补偿开关也必须关闭，否则配置非法。
    /// Master dead-time switch; both compensation switches require it.
    pub dead_time_enabled: bool,
    /// 死区时间 `[ns]`；校验区间 `[0, 10000]`，默认 550 ns `[HW]`。
    /// Dead time in `[ns]`; validated in `[0, 10000]`, default 550 ns [HW].
    pub dead_time_ns: f32,
    /// PWM 前馈死区补偿开关；必须与 `dead_time_enabled` 同时为真。
    /// Feed-forward dead-time compensation; requires `dead_time_enabled`.
    pub dead_time_feedforward_enabled: bool,
    /// 观测器电压重构死区补偿开关；必须与 `dead_time_enabled` 同时为真。
    /// Observer voltage-reconstruction compensation; requires `dead_time_enabled`.
    pub observer_dead_time_compensation_enabled: bool,
    /// 前馈补偿增益，无量纲；校验区间 `[0, 4]`，`1.0` 表示按名义死区全额补偿。
    /// Feed-forward gain, dimensionless; validated in `[0, 4]`.
    pub dead_time_compensation_gain: f32,
    /// 电流极性软零带 `[A]`；校验区间 `[0, 10]`，零带内极性按电流线性过渡。
    /// Soft polarity zero band in `[A]`; validated in `[0, 10]`.
    pub dead_time_current_zero_band_a: f32,
    /// 补偿电流极性的一阶滤波系数 `(0,1]`。
    pub inverter_current_sign_filter_alpha: f32,
    /// plant 与控制器模型共用的等效单相器件压降 `[V]`。
    pub inverter_device_drop_v: f32,
}

/// 默认场景，逐项标注来源：
/// Defaults, with provenance per field:
///
/// 量纲见 `BringupSimulationConfig`。`target_speed_rpm = 582.0` 与
///   `dc_bus_voltage_v = 12.3` 取自实机对标工况 `[HW]`（见
///   `docs/仿真实机相关性验证.md`），不是固件默认值；
///   `closed_loop_enable = false` 与固件上电默认一致 `[FW]`；
///   SMO/PLL 的 `4.0 V / 0.24 A / 0.015 / 40 / 1000` 是 CM4.4 空载双向
///   台架候选（`[HW]`）；`dead_time_ns = 550.0` 是功率板
///   实测死区 `[HW]`；`load_torque_nm = 0.0` 表示空载台架。
///   `trace_decimation = 320`：在 16 kHz 快环下等于 50 Hz 采样，当前 12 kHz 快环下
///   等效 37.5 Hz（详见 `docs/` 中的采样约定）。
///   582 rpm / 12.3 V are rig values [HW]; the SMO/PLL set is the tuned baseline.
impl Default for BringupSimulationConfig {
    fn default() -> Self {
        Self {
            duration_s: 5.0,
            target_speed_rpm: 582.0,
            initial_speed_rpm: 0.0,
            initial_electrical_angle_rad: 0.0,
            closed_loop_enable: false,
            startup_alignment_s: 2.0,
            startup_ramp_s: 5.0,
            startup_final_speed_rpm: 582.0,
            startup_alignment_current_a: 0.8,
            startup_current_a: 0.8,
            dc_bus_voltage_v: 12.3,
            load_torque_nm: 0.0,
            trace_decimation: 320,
            timing: MultiRateTimingConfig::default(),
            park_prediction_ticks: 0.0,
            reverse_park_prediction_ticks: 0.0,
            observer_smo_k_slide_v: 4.0,
            observer_smo_boundary_a: 0.24,
            observer_emf_filter_alpha: 0.015,
            observer_pll_kp: 40.0,
            observer_acquisition_pll_kp_ratio: 0.60,
            observer_pll_ki: 1_000.0,
            observer_acquisition_maximum_phase_error_rad: 0.65,
            observer_run_maximum_phase_error_rad: 0.65,
            observer_consecutive_samples: 20,
            observer_acquisition_timeout_s: 5.0,
            observer_loss_timeout_s: 0.20,
            startup_transition_s: 0.05,
            handoff_torque_support_ratio: 0.47,
            speed_pi_preload_ratio: 0.20,
            closed_loop_current_slew_a_per_s: 4.0,
            dead_time_enabled: false,
            dead_time_ns: 550.0,
            dead_time_feedforward_enabled: false,
            observer_dead_time_compensation_enabled: false,
            dead_time_compensation_gain: 1.0,
            dead_time_current_zero_band_a: 0.005,
            inverter_current_sign_filter_alpha: 1.0,
            inverter_device_drop_v: 0.0,
        }
    }
}

/// `run_bringup_simulation` 的一行 trace，与 bringup CSV 一行逐列对应。
/// One trace row of `run_bringup_simulation`, one CSV line.
///
/// 量纲 / Units: 时间 `[s]`、转速 `[rpm]`、电流 `[A]`、电压 `[V]`、角度 `[rad]`、
///   占空比无量纲 `[0,1]`；`step`/`state` 是计数与枚举值。
///
/// 真值分离（当前 ABI 0x000B0000）/ Ground-truth split: 三个角度字段刻意分开
///   （控制角、强制角、观测器角），避免把"控制角度"和"观测器角度"混成一个字段；
///   实机 trace 输出同样的字段，`true_*` 列在实机侧为空，因此仿真侧的真值列
///   只能单向用于对标，不能当作实机测量。
/// Three separate angle columns match the hardware trace; `true_*` columns have no
/// hardware counterpart and are simulation-only.
#[derive(Clone, Copy, Debug, Default)]
pub struct BringupSimulationSample {
    /// 仿真时刻 `[s]`，由控制拍号乘步长得到（非实测时间戳）。
    /// Simulation time in `[s]` from the tick index.
    pub time_s: f32,
    /// 已执行的控制拍数（从 1 开始，等于拍号 + 1）。
    /// Control ticks already executed, starting at 1.
    pub step: u32,
    /// 固件状态机的 `foc_rt_bridge::FocState` 数值：0 未初始化、1 关闭、2 运行、
    /// 3 对齐、4 开环加速、5 开环保持、6 观测器接管、7 闭环、8 故障。
    /// Numeric `FocState` value; 6 is observer handover and 7 is closed loop.
    pub state: u32,
    /// A 相电流 `[A]`（喂给控制器的同一拍采样值）。
    /// Phase A current in `[A]`, the same sample fed to the controller.
    pub phase_current_a: f32,
    /// B 相电流 `[A]`。
    /// Phase B current in `[A]`.
    pub phase_current_b: f32,
    /// C 相电流 `[A]`。
    /// Phase C current in `[A]`.
    pub phase_current_c: f32,
    /// 固件内部的目标转速 `[rpm]`（Rev-Up 期间是斜坡值，不是命令值）。
    /// Firmware target speed in `[rpm]`; it ramps during rev-up.
    pub target_speed_rpm: f32,
    /// **观测器估计**的转速 `[rpm]`，即固件遥测的 `measured_speed_rpm`。
    /// Observer-estimated speed in `[rpm]`, the firmware telemetry value.
    pub observer_speed_rpm: f32,
    /// 被控对象真值转速 `[rpm]`，只存在于仿真侧（实机没有轴端真值）。
    /// Plant ground-truth speed in `[rpm]`; simulation-only.
    pub true_speed_rpm: f32,
    /// d 轴电流给定 `[A]`。
    /// d-axis current reference in `[A]`.
    pub id_ref_a: f32,
    /// q 轴电流给定 `[A]`（闭环前是 Rev-Up 给定，闭环后来自速度环）。
    /// q-axis current reference in `[A]`.
    pub iq_ref_a: f32,
    /// d 轴电流实测 `[A]`。
    /// Measured d-axis current in `[A]`.
    pub id_a: f32,
    /// q 轴电流实测 `[A]`。
    /// Measured q-axis current in `[A]`.
    pub iq_a: f32,
    /// d 轴电压指令 `[V]`（固件遥测，限幅后）。
    /// d-axis voltage command in `[V]` from firmware telemetry.
    pub vd_v: f32,
    /// q 轴电压指令 `[V]`（固件遥测，限幅后）。
    /// q-axis voltage command in `[V]` from firmware telemetry.
    pub vq_v: f32,
    /// A 相占空比，无量纲 `[0,1]`（固件输出，未经死区补偿叠加）。
    /// Phase A duty ratio from the firmware output, in `[0,1]`.
    pub duty_a: f32,
    /// B 相占空比，无量纲 `[0,1]`。
    /// Phase B duty ratio in `[0,1]`.
    pub duty_b: f32,
    /// C 相占空比，无量纲 `[0,1]`。
    /// Phase C duty ratio in `[0,1]`.
    pub duty_c: f32,
    /// 本拍使用的母线电压 `[V]`（来自配置，恒定；实机会有纹波）。
    /// DC bus voltage in `[V]` from the config; constant here.
    pub dc_bus_voltage_v: f32,
    /// **本次 Park/逆 Park 延迟补偿前**的基础控制角度 `[rad]`。
    /// The control angle in `[rad]` actually used by Park/inverse Park.
    pub control_angle_rad: f32,
    /// Rev-Up 强制角度 `[rad]`（开环段使用的角度）。
    /// Rev-up forced angle in `[rad]`, used during open loop.
    pub forced_angle_rad: f32,
    /// SMO/PLL 估算角度 `[rad]`。
    /// Observer (SMO/PLL) estimated angle in `[rad]`.
    pub observer_angle_rad: f32,
    /// 被控对象真值电角度 `[rad]`，仿真侧才有。
    /// Plant ground-truth electrical angle in `[rad]`; simulation-only.
    pub true_angle_rad: f32,
    /// 固件观测器可靠标志（可靠性 FIFO / 方差门的输出）。
    /// The firmware observer-reliability flag.
    pub observer_reliable: bool,
}

/// 相关性仿真的汇总指标（RMSE / 均值 / 标准差），全部在**抽点后**的 trace 上计算。
/// Summary metrics of the correlation simulation, computed on the decimated trace.
///
/// 时间窗 / Time windows（按 `BringupSimulationSample::time_s` 过滤）:
///   保持段 hold 取 `time_s >= 2.5 s`；过渡段 transient 取 `4.0 <= time_s <= 5.0`；
///   稳态 steady 取 `time_s >= max(duration_s - 2.0, 2.5)`，即"结束前 2 秒"且不早于
///   2.5 s。
///   Hold is `>= 2.5 s`, transient is `4.0..=5.0 s`, and steady is the last 2 s but
///   never starting before 2.5 s.
///
/// 定界与陷阱 / Guards and pitfalls:
///   窗口为空时计数用 `.max(1)` 兜底，因此指标是 `0` 而不是 `NaN`——这会把"没有
///   样本"伪装成"误差为零"，改时长或抽点时必须检查样本数。标准差用样本分母
///   `n-1`（`n <= 1` 时按 1 算）；角度 RMSE 先把差值折到 `[-π, π)` 再平方，避免
///   `±π` 附近的环绕被算成 `2π` 误差。这些指标全部基于**仿真真值**，实机缺少
///   独立转速真值，因此不能反过来验证实机精度。
///   Empty windows return 0 rather than NaN, so short runs can look perfect; the
///   angle error is wrapped into `[-π, π)` before squaring.
#[derive(Clone, Copy, Debug, Default)]
pub struct BringupSimulationSummary {
    /// 结束时刻的被控对象真值转速 `[rpm]`。
    /// Final plant ground-truth speed in `[rpm]`.
    pub final_true_speed_rpm: f32,
    /// 结束时刻的观测器估计转速 `[rpm]`。
    /// Final observer-estimated speed in `[rpm]`.
    pub final_observer_speed_rpm: f32,
    /// 全程相电流峰值 `[A]`（每控制拍更新，不受抽点影响）。
    /// Peak phase current in `[A]`, updated every control tick.
    pub peak_phase_current_a: f32,
    /// `observer_reliable` 为真的样本数（计数，未抽点前的样本集合）。
    /// Number of samples with `observer_reliable == true`.
    pub observer_reliable_samples: usize,
    /// 保持段转速 RMSE `[rpm]`：`observer_speed_rpm - true_speed_rpm`，衡量观测器
    /// 在开环保持段的估速偏差。
    /// Hold-window speed RMSE in `[rpm]` between observed and true speed.
    pub hold_speed_rmse_rpm: f32,
    /// 保持段角度 RMSE `[rad]`：`observer_angle_rad - true_angle_rad` 折到
    /// `[-π, π)` 后求均方根。
    /// Hold-window angle RMSE in `[rad]`, wrapped into `[-π, π)`.
    pub hold_angle_rmse_rad: f32,
    /// 过渡段目标跟踪 RMSE `[rpm]`：`true_speed_rpm - target`，衡量真实转速对
    /// 目标的跟踪（不是观测器误差）。
    /// Transient-window target-tracking RMSE in `[rpm]` of the true speed.
    pub transient_speed_target_rmse_rpm: f32,
    /// 过渡段观测器 RMSE `[rpm]`：`observer_speed_rpm - true_speed_rpm`。
    /// Transient-window observer speed RMSE in `[rpm]`.
    pub transient_observer_rmse_rpm: f32,
    /// 稳态真值转速均值 `[rpm]`。
    /// Steady-window mean true speed in `[rpm]`.
    pub steady_true_speed_mean_rpm: f32,
    /// 稳态真值转速标准差 `[rpm]`（样本标准差，分母 `n-1`）。
    /// Steady-window true-speed standard deviation in `[rpm]` (`n-1` denominator).
    pub steady_true_speed_std_rpm: f32,
    /// 稳态观测器转速 RMSE `[rpm]`。
    /// Steady-window observer speed RMSE in `[rpm]`.
    pub steady_observer_rmse_rpm: f32,
    /// 稳态 q 轴电流跟踪 RMSE `[A]`：`iq_a - iq_ref_a`；12 kHz 下速度环会消除
    /// 大部分稳态机械转速偏差，因此死区畸变主要由这个指标反映。
    /// Steady-window q-axis current tracking RMSE in `[A]`, the dead-time metric.
    pub steady_iq_tracking_rmse_a: f32,
    /// 稳态 d 轴电流 RMSE `[A]`：按 `id_a` 相对 0 求均方根（d 轴给定恒为 0）。
    /// Steady-window d-axis current RMSE in `[A]` against a zero reference.
    pub steady_id_rmse_a: f32,
    /// 结束时刻的固件状态值，见 `BringupSimulationSample::state`。
    /// Final firmware state value, see `BringupSimulationSample::state`.
    pub final_state: u32,
    /// 抽点后的 trace 样本数（不是控制拍数）。
    /// Number of trace samples after decimation.
    pub sample_count: usize,
    /// 实际推进的 PWM 拍数。
    pub pwm_tick_count: u64,
    /// 实际调用固件快环 C ABI 的控制拍数。
    pub control_tick_count: u64,
    /// 已在载波边界生效的占空更新数。
    pub applied_pwm_update_count: u64,
}

/// `run_bringup_simulation` 的失败原因。
/// Failure reasons of `run_bringup_simulation`.
///
/// 语义 / Semantics: `InvalidConfig` 携带一条静态说明（只说明大类，逐项范围见
///   `run_bringup_simulation` 的校验代码）；`Bridge` 携带
///   `foc_rt_bridge::FocStatus`，表示固件侧某个入口返回非 `Ok`（例如目标转速
///   越界、未配置或硬件故障）。任一错误都在该拍立即返回，不再推进被控对象。
/// No state is advanced after a failure; the error is returned from that tick.
#[derive(Debug)]
pub enum BringupSimulationError {
    /// 配置非法。
    /// Invalid configuration.
    InvalidConfig(&'static str),
    /// 固件桥接层返回非 `Ok` 状态。
    /// The firmware bridge returned a non-`Ok` status.
    Bridge(foc_rt_bridge::FocStatus),
}

/// 一次 bringup 仿真的完整结果。
/// The full result of one bringup simulation.
///
/// 实时约束 / Real-time note: `samples` 是 `Vec`，只允许出现在离线 PC 代码；
///   固件侧同名字段由定长环形缓冲承载，ISR 不做分配。
/// `Vec` here is offline-only and must never appear in the realtime path.
#[derive(Debug)]
pub struct BringupSimulationRun {
    /// 抽点后的 trace 样本，按时间升序。
    /// Decimated trace samples in ascending time order.
    pub samples: Vec<BringupSimulationSample>,
    /// 汇总指标，见 `BringupSimulationSummary`。
    /// Summary metrics, see `BringupSimulationSummary`.
    pub summary: BringupSimulationSummary,
}

impl BringupSimulationRun {
    /// 把 bringup trace 写成 CSV：一行表头 + 每样本一行，`,` 分隔。
    /// Writes the bringup trace as CSV with one header row and one row per sample.
    ///
    /// 列定义 / Columns（顺序即 `BringupSimulationSample` 的字段顺序）: `time_s`
    ///   `[s]`、`step`（已执行控制拍数）、`state`（`FocState` 数值 0..8）、三相电流
    ///   `[A]`、目标/观测/真值转速 `[rpm]`、`id_ref_a`/`iq_ref_a`/`id_a`/`iq_a`
    ///   `[A]`、`vd_v`/`vq_v` `[V]`、三相占空比 `[0,1]`、`dc_bus_voltage_v` `[V]`、
    ///   `control_angle_rad`/`forced_angle_rad`/`observer_angle_rad`/`true_angle_rad`
    ///   `[rad]`、`observer_reliable`（`0`/`1`），最后还有一列 `flags`，本入口
    ///   **恒写 `0`**（列位与实机 trace 的 flags 对齐，仿真没有该遥测）。
    ///   The header mirrors the sample struct and ends with a constant `flags`
    ///   column that this entry point always writes as `0`.
    ///
    /// 契约 / Contract: 列名与顺序是 `simulation/compare_traces.py` 与 MATLAB 绘图
    ///   脚本的输入契约；实机 trace 由固件按同样字段输出，改名或换序会让两侧无法
    ///   按列名对齐。实机侧没有的 `true_*` 列留空，由脚本转成 `NaN`。
    ///   Column names and order are a contract for the comparison scripts.
    ///
    /// 失败语义 / Failure: I/O 错误原样上抛，文件可能是部分写入。
    /// I/O errors propagate as-is; a partially written file is possible.
    pub fn write_csv(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let file = File::create(path)?;
        let mut writer = BufWriter::new(file);
        writeln!(
            writer,
            "time_s,step,state,phase_current_a,phase_current_b,phase_current_c,target_speed_rpm,observer_speed_rpm,true_speed_rpm,id_ref_a,iq_ref_a,id_a,iq_a,vd_v,vq_v,duty_a,duty_b,duty_c,dc_bus_voltage_v,control_angle_rad,forced_angle_rad,observer_angle_rad,true_angle_rad,observer_reliable,flags"
        )?;
        for sample in &self.samples {
            // 格式串末尾的 `0` 是 `flags` 列：仿真没有实机的状态标志位，但列位必须
            // 保留，否则对比脚本按列名取数会整体错位。
            // The trailing literal `0` keeps the `flags` column aligned.
            writeln!(
                writer,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},0",
                sample.time_s,
                sample.step,
                sample.state,
                sample.phase_current_a,
                sample.phase_current_b,
                sample.phase_current_c,
                sample.target_speed_rpm,
                sample.observer_speed_rpm,
                sample.true_speed_rpm,
                sample.id_ref_a,
                sample.iq_ref_a,
                sample.id_a,
                sample.iq_a,
                sample.vd_v,
                sample.vq_v,
                sample.duty_a,
                sample.duty_b,
                sample.duty_c,
                sample.dc_bus_voltage_v,
                sample.control_angle_rad,
                sample.forced_angle_rad,
                sample.observer_angle_rad,
                sample.true_angle_rad,
                u8::from(sample.observer_reliable),
            )?;
        }
        writer.flush()
    }
}

/// 走固件同一套 Rev-Up/电流环/观测器路径，只把 ADC/PWM 适配与物理电机换成 PC 上的
/// PMSM 被控对象。
/// Runs the firmware-facing rev-up/current-loop/observer path against the
/// host PMSM plant. The command path is identical to target firmware; only the
/// ADC/PWM hardware adapter and physical motor are replaced by the plant.
///
/// 相关性为什么成立（本入口存在的理由）/ Why correlation is meaningful:
///   本函数逐步调用与 STM32G431 ADC 中断**完全相同**的 C ABI 入口：
///   `foc_rust_init`、`foc_rust_configure`、`foc_rust_start_realtime`、
///   `foc_rust_realtime_step`；逆变器模型也通过与目标固件相同的 V7 配置安装。
///   它不是"再写一遍近似控制器"，
///   所以状态机、Rev-Up 时序、SMO/PLL、可靠性门与电流环在 PC 与实机上来自同一份
///   代码；正因如此，仿真与实机的对比才有意义。
///   This is what makes simulation/hardware correlation meaningful: the same
///   C-ABI entry points used by the 12 kHz ADC ISR run step by step.
///
/// 与 `run_reference_simulation` 的关键区别 / Key difference:
///   这里**没有理想转子反馈**——`FocFeedback::electrical_angle_rad` 恒传 `0.0`，
///   控制角度完全由固件侧 Rev-Up 与观测器产生；被控对象的电角度只用于计算
///   `true_*` 真值列。传 `0.0` 是刻意的：`foc_rust_realtime_step` 只用该字段做
///   有限性校验，真正把它送进电流环的是旧的 `foc_rust_step` 兼容入口。
///   No ideal rotor feedback is fed to the controller: the zero angle is only
///   validated for finiteness, and rev-up/observer own the control angle.
///
/// 参数 / Parameters: `config` 见 `BringupSimulationConfig`。校验范围（任一项不满足
///   即 `InvalidConfig`）：`duration_s` 有限且为正、`target_speed_rpm` 有限、
///   `dc_bus_voltage_v` 有限且为正、`load_torque_nm` 有限、`trace_decimation` 非零、
///   `observer_smo_k_slide_v` 有限且为正、`observer_smo_boundary_a` 有限且为正、
///   `observer_emf_filter_alpha` 有限且落在 `[0.0001, 1.0]`、`observer_pll_kp` 与
///   `observer_pll_ki` 有限且非负、`dead_time_ns` 有限且落在 `[0, 10000]`、
///   `dead_time_compensation_gain` 有限且落在 `[0, 4]`、
///   `dead_time_current_zero_band_a` 有限且落在 `[0, 10]`；另外
///   `dead_time_enabled == false` 时禁止打开前馈或观测器死区补偿。
///   Invalid configuration returns `InvalidConfig` before any state is created.
///
/// 执行顺序 / Order of operations: 先校验参数；再构造被控对象并设置负载；然后
///   `foc_rust_init` 初始化上下文、下发 Host 死区补偿、用
///   `foc_rust_default_st_config` 读取 ST 默认档案并逐项覆盖（观测器使能、
///   闭环开关、`observer_update_divider = 1`、`voltage_utilization = 0.90`、
///   SMO/PLL 参数），再 `foc_rust_configure`，最后 `foc_rust_start_realtime` 进入
///   对齐态。主循环按 PWM 拍推进 plant，在控制分频点执行
///   "采被控对象 -> `foc_rust_realtime_step` -> 按抽点记录"，其余载波拍保持占空。
///   **被控对象的电流回馈是理想值**，本入口
///   不建模 ADC 采样延时、偏置与量化，因此误差只反映控制/观测器与逆变器模型。
///   No ADC delay, offset or quantisation is modelled; only the average inverter
///   model sits between the controller output and the plant.
///
/// 时间基准与量纲 / Time base and units: plant/死区使用 PWM 周期，固件快环、PI、
///   Rev-Up 与观测器使用控制周期；`dead_time_s = dead_time_ns * 1e-9` `[s]`；全程 SI，
///   被控对象为纯 `f32`，无 Q 格式。
///
/// 实时约束 / Real-time note: `Vec::with_capacity`、真值拷贝与末尾的 RMSE 统计都
///   只在离线 PC 代码里合法；目标固件对应路径运行在 12 kHz ADC ISR 中，禁止分配、
///   阻塞与日志。
/// The `Vec` trace and metric maths are offline-only and never belong in an ISR.
pub fn run_bringup_simulation(
    config: BringupSimulationConfig,
) -> Result<BringupSimulationRun, BringupSimulationError> {
    use foc_rt_bridge::{
        foc_rust_configure, foc_rust_default_st_config, foc_rust_init, foc_rust_realtime_step,
        foc_rust_start_realtime, FocFeedback, FocOutput, FocRuntimeConfig, FocRustContextStorage,
        FocStatus, FocTelemetry, FOC_RUST_CONTEXT_CAPACITY,
    };

    // 参数校验在构造任何状态之前完成，所以非法配置不会留下半个运行时。
    // All validation happens before any state is created.
    if !config.duration_s.is_finite() || config.duration_s <= 0.0 {
        return Err(BringupSimulationError::InvalidConfig(
            "duration_s must be positive",
        ));
    }
    // 观测器/死区参数必须落在观测器与补偿自身的有效区间内；最后一条约束要求
    // 打开任一补偿前必须先打开死区模型，避免"补偿了一个不存在的死区"。
    // Feed-forward/observer compensation requires the dead-time model to be on.
    if !config.target_speed_rpm.is_finite()
        || !config.initial_speed_rpm.is_finite()
        || !config.initial_electrical_angle_rad.is_finite()
        || !config.startup_alignment_s.is_finite()
        || !(0.001..=10.0).contains(&config.startup_alignment_s)
        || !config.startup_ramp_s.is_finite()
        || !(0.001..=10.0).contains(&config.startup_ramp_s)
        || !config.startup_final_speed_rpm.is_finite()
        || config.startup_final_speed_rpm <= 0.0
        || !config.startup_alignment_current_a.is_finite()
        || config.startup_alignment_current_a <= 0.0
        || !config.startup_current_a.is_finite()
        || config.startup_current_a <= 0.0
        || !config.dc_bus_voltage_v.is_finite()
        || config.dc_bus_voltage_v <= 0.0
        || !config.load_torque_nm.is_finite()
        || config.trace_decimation == 0
        || !config.park_prediction_ticks.is_finite()
        || !(-2.0..=2.0).contains(&config.park_prediction_ticks)
        || !config.reverse_park_prediction_ticks.is_finite()
        || !(-2.0..=2.0).contains(&config.reverse_park_prediction_ticks)
        || !config.observer_smo_k_slide_v.is_finite()
        || config.observer_smo_k_slide_v <= 0.0
        || !config.observer_smo_boundary_a.is_finite()
        || config.observer_smo_boundary_a <= 0.0
        || !config.observer_emf_filter_alpha.is_finite()
        || !(0.0001..=1.0).contains(&config.observer_emf_filter_alpha)
        || !config.observer_pll_kp.is_finite()
        || config.observer_pll_kp < 0.0
        || !config.observer_acquisition_pll_kp_ratio.is_finite()
        || !(0.0..=1.0).contains(&config.observer_acquisition_pll_kp_ratio)
        || config.observer_acquisition_pll_kp_ratio == 0.0
        || !config.observer_pll_ki.is_finite()
        || config.observer_pll_ki < 0.0
        || !config
            .observer_acquisition_maximum_phase_error_rad
            .is_finite()
        || config.observer_acquisition_maximum_phase_error_rad <= 0.0
        || config.observer_acquisition_maximum_phase_error_rad > core::f32::consts::FRAC_PI_2
        || !config.observer_run_maximum_phase_error_rad.is_finite()
        || config.observer_run_maximum_phase_error_rad <= 0.0
        || config.observer_run_maximum_phase_error_rad > core::f32::consts::FRAC_PI_2
        || !(1..=1_000).contains(&config.observer_consecutive_samples)
        || !config.observer_acquisition_timeout_s.is_finite()
        || !(0.001..=10.0).contains(&config.observer_acquisition_timeout_s)
        || !config.observer_loss_timeout_s.is_finite()
        || !(0.001..=5.0).contains(&config.observer_loss_timeout_s)
        || !config.startup_transition_s.is_finite()
        || !(0.001..=5.0).contains(&config.startup_transition_s)
        || !config.handoff_torque_support_ratio.is_finite()
        || !(0.0..=1.0).contains(&config.handoff_torque_support_ratio)
        || !config.speed_pi_preload_ratio.is_finite()
        || !(0.0..=1.0).contains(&config.speed_pi_preload_ratio)
        || !config.closed_loop_current_slew_a_per_s.is_finite()
        || !(0.01..=1_000.0).contains(&config.closed_loop_current_slew_a_per_s)
        || !config.dead_time_ns.is_finite()
        || !(0.0..=10_000.0).contains(&config.dead_time_ns)
        || !config.dead_time_compensation_gain.is_finite()
        || !(0.0..=4.0).contains(&config.dead_time_compensation_gain)
        || !config.dead_time_current_zero_band_a.is_finite()
        || !(0.0..=10.0).contains(&config.dead_time_current_zero_band_a)
        || !config.inverter_current_sign_filter_alpha.is_finite()
        || !(0.0001..=1.0).contains(&config.inverter_current_sign_filter_alpha)
        || !config.inverter_device_drop_v.is_finite()
        || !(0.0..=100.0).contains(&config.inverter_device_drop_v)
        || (!config.dead_time_enabled
            && (config.dead_time_feedforward_enabled
                || config.observer_dead_time_compensation_enabled))
    {
        return Err(BringupSimulationError::InvalidConfig(
            "target, initial state, bus, load and decimation must be valid",
        ));
    }

    let params = foc_control::st_gbm2804_reference_parameters();
    if config.startup_alignment_current_a > params.motor.rated_current_a
        || config.startup_current_a > params.motor.rated_current_a
        || config.initial_speed_rpm.abs() > params.motor.max_speed_rpm
    {
        return Err(BringupSimulationError::InvalidConfig(
            "startup currents must not exceed rated current",
        ));
    }
    if config.timing.control_frequency_hz != params.pwm_frequency_hz {
        return Err(BringupSimulationError::InvalidConfig(
            "control_frequency_hz must match the firmware controller configuration",
        ));
    }
    let pwm_ticks_per_control = config
        .timing
        .validate()
        .map_err(|_| BringupSimulationError::InvalidConfig("invalid multi-rate timing"))?;
    // 死区始终按 PWM 周期归一化；控制周期仍由固件参数保持 12 kHz。
    let pwm_dt = config.timing.pwm_period_s();
    let dead_time_s = config.dead_time_ns * 1.0e-9;
    let inverter = InverterSimulationConfig {
        dead_time_enabled: config.dead_time_enabled,
        dead_time_s,
        device_drop_v: config.inverter_device_drop_v,
    };
    let total_pwm_steps = (config.duration_s / pwm_dt).round() as u32;
    let total_control_steps = total_pwm_steps.div_ceil(pwm_ticks_per_control);
    let mut plant = PmsmPlant::new(params.motor);
    plant.set_initial_mechanical_state(
        config.initial_speed_rpm,
        config.initial_electrical_angle_rad,
    );
    plant.set_load_torque_nm(config.load_torque_nm);
    // 上下文缓冲区由 C ABI 约定为调用方持有：容量在编译期已断言容得下
    // `Controller`，这里用零初始化，`foc_rust_init` 会整体覆盖它。
    // Caller-owned ABI context storage; `foc_rust_init` overwrites it wholesale.
    let mut context = FocRustContextStorage {
        bytes: [0; FOC_RUST_CONTEXT_CAPACITY],
    };
    let mut runtime_config = FocRuntimeConfig::default();

    // 把 C ABI 的 `FocStatus` 收敛成 `Result`，让每个入口都能用 `?` 短路；任何一个
    // 入口失败都会立即返回，不会带着未配置的上下文继续跑。
    // Folds `FocStatus` into `Result` so any failing entry point short-circuits.
    let require_ok = |status: FocStatus| {
        if status == FocStatus::Ok {
            Ok(())
        } else {
            Err(BringupSimulationError::Bridge(status))
        }
    };
    // 中文：参与 FFI 调用的都是调用期内的局部对齐对象，且此处独占访问。
    // SAFETY: all values are local, aligned ABI objects with exclusive access.
    unsafe {
        require_ok(foc_rust_init(&mut context))?;
        require_ok(foc_rust_default_st_config(&mut runtime_config))?;
        // 读入 ST 默认档案后只覆盖下面这些字段：观测器强制打开（无感路径的必要
        // 条件），闭环开关由配置决定，`observer_update_divider = 1` 表示观测器与
        // 电流环同拍（12 kHz）执行，`voltage_utilization` 收紧到 0.90。
        // Only these fields are overridden on top of the ST default profile.
        runtime_config.observer_enable = 1;
        runtime_config.closed_loop_enable = u32::from(config.closed_loop_enable);
        runtime_config.alignment_duration_s = config.startup_alignment_s;
        runtime_config.open_loop_ramp_duration_s = config.startup_ramp_s;
        runtime_config.startup_final_speed_rpm = config.startup_final_speed_rpm;
        runtime_config.startup_alignment_current_a = config.startup_alignment_current_a;
        runtime_config.startup_current_a = config.startup_current_a;
        runtime_config.observer_update_divider = 1;
        runtime_config.voltage_utilization = 0.90;
        runtime_config.observer_smo_k_slide_v = config.observer_smo_k_slide_v;
        runtime_config.observer_smo_boundary_a = config.observer_smo_boundary_a;
        runtime_config.observer_emf_filter_alpha = config.observer_emf_filter_alpha;
        runtime_config.observer_pll_kp = config.observer_pll_kp;
        runtime_config.observer_acquisition_pll_kp_ratio = config.observer_acquisition_pll_kp_ratio;
        runtime_config.observer_pll_ki = config.observer_pll_ki;
        runtime_config.observer_acquisition_maximum_phase_error_rad =
            config.observer_acquisition_maximum_phase_error_rad;
        runtime_config
            .observer_run_reliability
            .maximum_phase_error_rad = config.observer_run_maximum_phase_error_rad;
        runtime_config.observer_consecutive_samples = config.observer_consecutive_samples;
        runtime_config.observer_acquisition_timeout_s = config.observer_acquisition_timeout_s;
        runtime_config.observer_loss_timeout_s = config.observer_loss_timeout_s;
        runtime_config.observer_transition_duration_s = config.startup_transition_s;
        runtime_config.handoff_torque_support_ratio = config.handoff_torque_support_ratio;
        runtime_config.speed_pi_preload_ratio = config.speed_pi_preload_ratio;
        runtime_config.closed_loop_current_slew_a_per_s = config.closed_loop_current_slew_a_per_s;
        runtime_config.angle_compensation.park_prediction_ticks = config.park_prediction_ticks;
        runtime_config
            .angle_compensation
            .reverse_park_prediction_ticks = config.reverse_park_prediction_ticks;
        runtime_config.inverter_voltage_model.enabled = u32::from(
            config.dead_time_feedforward_enabled || config.observer_dead_time_compensation_enabled,
        );
        runtime_config
            .inverter_voltage_model
            .observer_voltage_correction_enable =
            u32::from(config.observer_dead_time_compensation_enabled);
        runtime_config.inverter_voltage_model.pwm_feedforward_enable =
            u32::from(config.dead_time_feedforward_enabled);
        runtime_config
            .inverter_voltage_model
            .pwm_carrier_frequency_hz = config.timing.pwm_frequency_hz;
        runtime_config.inverter_voltage_model.dead_time_s = dead_time_s;
        runtime_config.inverter_voltage_model.compensation_gain =
            config.dead_time_compensation_gain;
        runtime_config.inverter_voltage_model.current_zero_band_a =
            config.dead_time_current_zero_band_a;
        runtime_config
            .inverter_voltage_model
            .current_sign_filter_alpha = config.inverter_current_sign_filter_alpha;
        runtime_config.inverter_voltage_model.device_drop_v = config.inverter_device_drop_v;
        require_ok(foc_rust_configure(&mut context, &runtime_config))?;
        require_ok(foc_rust_start_realtime(
            &mut context,
            1,
            config.target_speed_rpm,
        ))?;
    }

    let mut samples =
        Vec::with_capacity((total_control_steps / config.trace_decimation + 2) as usize);
    let mut peak_phase_current_a = 0.0_f32;
    let mut final_telemetry = FocTelemetry::default();
    let mut scheduler = MultiRateScheduler::new(config.timing)
        .map_err(|_| BringupSimulationError::InvalidConfig("invalid multi-rate timing"))?;
    for pwm_tick in 0..total_pwm_steps {
        let control_due = scheduler.begin_pwm_tick();
        // 被控对象每个 PWM 拍都推进并更新峰值；控制链只在整数分频点采样。
        let currents = plant.phase_currents();
        peak_phase_current_a = peak_phase_current_a
            .max(currents.a.abs())
            .max(currents.b.abs())
            .max(currents.c.abs());
        if control_due {
            let control_tick = scheduler.control_tick_count() as u32;
            let feedback = FocFeedback {
                phase_current_a: currents.a,
                phase_current_b: currents.b,
                phase_current_c: currents.c,
                dc_bus_voltage: config.dc_bus_voltage_v,
                electrical_angle_rad: 0.0,
            };
            let mut output = FocOutput::default();
            let mut telemetry = FocTelemetry::default();
            // SAFETY: context initialized; all pointers are valid and disjoint.
            let status = unsafe {
                foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry)
            };
            require_ok(status)?;
            final_telemetry = telemetry;
            scheduler.submit_control_output(PwmCommand {
                duty_a: output.duty_a,
                duty_b: output.duty_b,
                duty_c: output.duty_c,
            });

            // 只按控制拍记录；占空列是当前载波周期真正施加到对象的保持值。
            if control_tick.is_multiple_of(config.trace_decimation)
                || (control_tick + 1 == total_control_steps)
            {
                let plant_state = plant.state();
                let active_pwm = scheduler.active_pwm();
                samples.push(BringupSimulationSample {
                    time_s: pwm_tick as f32 * pwm_dt,
                    step: control_tick + 1,
                    state: telemetry.state,
                    phase_current_a: currents.a,
                    phase_current_b: currents.b,
                    phase_current_c: currents.c,
                    target_speed_rpm: telemetry.target_speed_rpm,
                    observer_speed_rpm: telemetry.measured_speed_rpm,
                    true_speed_rpm: plant_state.mechanical_speed_rad_s * 30.0 / PI,
                    id_ref_a: telemetry.id_reference_a,
                    iq_ref_a: telemetry.iq_reference_a,
                    id_a: telemetry.id_measured_a,
                    iq_a: telemetry.iq_measured_a,
                    vd_v: telemetry.vd_command_v,
                    vq_v: telemetry.vq_command_v,
                    duty_a: active_pwm.duty_a,
                    duty_b: active_pwm.duty_b,
                    duty_c: active_pwm.duty_c,
                    dc_bus_voltage_v: config.dc_bus_voltage_v,
                    control_angle_rad: telemetry.electrical_angle_rad,
                    forced_angle_rad: telemetry.forced_electrical_angle_rad,
                    observer_angle_rad: telemetry.observer_electrical_angle_rad,
                    true_angle_rad: plant.electrical_angle_rad(),
                    observer_reliable: telemetry.observer_reliable != 0,
                });
            }
        }

        // 每个载波周期都用保持后的占空推进对象；死区按载波周期而非控制周期归一化。
        plant.step(
            pwm_to_alpha_beta_with_inverter(
                scheduler.active_pwm(),
                currents,
                config.dc_bus_voltage_v,
                pwm_dt,
                inverter,
            ),
            pwm_dt,
        );
        scheduler.finish_pwm_tick();
    }

    let final_state = plant.state();
    // 指标全部在抽点后的 trace 上计算：窗口用时间（`[s]`）而不是拍号，因此与抽点
    // 间隔无关；`hold` 段取 2.5 s 之后，覆盖开环保持/接管阶段。
    // Metrics use time windows, so they are independent of the decimation setting.
    let hold_samples: Vec<&BringupSimulationSample> = samples
        .iter()
        .filter(|sample| sample.time_s >= 2.5)
        .collect();
    let hold_count = hold_samples.len().max(1) as f32;
    // 保持段估速误差 `[rpm]`：`max(1)` 兜底使空窗口返回 0 而不是 NaN。
    // Guarded count: an empty window yields 0 instead of NaN.
    let hold_speed_rmse_rpm = (hold_samples
        .iter()
        .map(|sample| {
            let error = sample.observer_speed_rpm - sample.true_speed_rpm;
            error * error
        })
        .sum::<f32>()
        / hold_count)
        .sqrt();
    // 角度误差先加 π、取模 `2π`、再减 π，把它折到 `[-π, π)`：否则 `+π` 与 `-π`
    // 附近的角度差会被当成接近 `2π` 的巨大误差。
    // Wraps the angle difference into `[-π, π)` before squaring.
    let hold_angle_rmse_rad = (hold_samples
        .iter()
        .map(|sample| {
            let error =
                (sample.observer_angle_rad - sample.true_angle_rad + PI).rem_euclid(2.0 * PI) - PI;
            error * error
        })
        .sum::<f32>()
        / hold_count)
        .sqrt();
    // 过渡段固定取 `4.0..=5.0 s`（与时长无关），所以时长小于 5 s 时该窗口会为空。
    // The transient window is fixed at `4.0..=5.0 s` regardless of duration.
    let transient_samples: Vec<&BringupSimulationSample> = samples
        .iter()
        .filter(|sample| (4.0..=5.0).contains(&sample.time_s))
        .collect();
    let transient_count = transient_samples.len().max(1) as f32;
    // 目标跟踪误差用真值转速，观测器误差用估速：两者不可混用，混用会把观测器
    // 偏差误判成控制跟踪变差。
    // True speed for tracking error, estimated speed for observer error.
    let transient_speed_target_rmse_rpm = (transient_samples
        .iter()
        .map(|sample| {
            let error = sample.true_speed_rpm - config.target_speed_rpm;
            error * error
        })
        .sum::<f32>()
        / transient_count)
        .sqrt();
    let transient_observer_rmse_rpm = (transient_samples
        .iter()
        .map(|sample| {
            let error = sample.observer_speed_rpm - sample.true_speed_rpm;
            error * error
        })
        .sum::<f32>()
        / transient_count)
        .sqrt();
    // 稳态窗口 = 结束前 2 秒，但不早于 2.5 s：短仿真因此会与保持段重叠，指标不再
    // 独立，读结果时必须同时看 `sample_count`。
    // Steady window is the last 2 s, never starting before 2.5 s.
    let steady_start_s = (config.duration_s - 2.0).max(2.5);
    let steady_samples: Vec<&BringupSimulationSample> = samples
        .iter()
        .filter(|sample| sample.time_s >= steady_start_s)
        .collect();
    let steady_count = steady_samples.len().max(1) as f32;
    let steady_true_speed_mean_rpm = steady_samples
        .iter()
        .map(|sample| sample.true_speed_rpm)
        .sum::<f32>()
        / steady_count;
    // 标准差用样本分母 `n-1`（不是总体分母 `n`），与 MATLAB/Python 的默认 `std`
    // 保持一致；`n <= 1` 时按 1 兜底，结果退化为 0。
    // Sample standard deviation with the `n-1` denominator, like MATLAB `std`.
    let steady_true_speed_std_rpm = (steady_samples
        .iter()
        .map(|sample| {
            let delta = sample.true_speed_rpm - steady_true_speed_mean_rpm;
            delta * delta
        })
        .sum::<f32>()
        / (steady_samples.len().saturating_sub(1).max(1) as f32))
        .sqrt();
    let steady_observer_rmse_rpm = (steady_samples
        .iter()
        .map(|sample| {
            let error = sample.observer_speed_rpm - sample.true_speed_rpm;
            error * error
        })
        .sum::<f32>()
        / steady_count)
        .sqrt();
    let steady_iq_tracking_rmse_a = (steady_samples
        .iter()
        .map(|sample| {
            let error = sample.iq_a - sample.iq_ref_a;
            error * error
        })
        .sum::<f32>()
        / steady_count)
        .sqrt();
    // Id 的给定恒为 0，所以 Id 误差就是 `id_a` 本身的均方根（不需要减参考值）。
    // The d-axis reference is zero, so the RMS of `id_a` is the tracking error.
    let steady_id_rmse_a = (steady_samples
        .iter()
        .map(|sample| sample.id_a * sample.id_a)
        .sum::<f32>()
        / steady_count)
        .sqrt();
    Ok(BringupSimulationRun {
        summary: BringupSimulationSummary {
            final_true_speed_rpm: final_state.mechanical_speed_rad_s * 30.0 / PI,
            final_observer_speed_rpm: final_telemetry.measured_speed_rpm,
            peak_phase_current_a,
            observer_reliable_samples: samples
                .iter()
                .filter(|sample| sample.observer_reliable)
                .count(),
            hold_speed_rmse_rpm,
            hold_angle_rmse_rad,
            transient_speed_target_rmse_rpm,
            transient_observer_rmse_rpm,
            steady_true_speed_mean_rpm,
            steady_true_speed_std_rpm,
            steady_observer_rmse_rpm,
            steady_iq_tracking_rmse_a,
            steady_id_rmse_a,
            final_state: final_telemetry.state,
            sample_count: samples.len(),
            pwm_tick_count: scheduler.pwm_tick_count(),
            control_tick_count: scheduler.control_tick_count(),
            applied_pwm_update_count: scheduler.applied_update_count(),
        },
        samples,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use foc_control::{
        st_gbm2804_reference_parameters, ControlRuntime, SpeedCommand, StReferenceController,
    };

    /// 以控制器周期为步长跑固定秒数：每拍先 `tick` 再 `advance`，复现真实快环里
    /// "控制律 -> 功率级 -> 电机" 的交替顺序。
    /// Runs the loop for a fixed time with tick-then-advance ordering.
    ///
    /// `.expect("closed-loop tick")` 意味着任何端口错误都会让测试直接失败，而不是
    ///   被静默忽略；`steps` 用 `as usize` 截断，非整数秒会少跑不到一拍。
    /// A port error panics here instead of being silently ignored.
    fn run_for(runtime: &mut ControlRuntime<SimHardware>, seconds: f32, target_rpm: f32) {
        let dt = runtime.controller_period_s();
        let steps = (seconds / dt) as usize;
        for _ in 0..steps {
            runtime
                .tick(SpeedCommand {
                    target_rpm,
                    id_ref_a: 0.0,
                })
                .expect("closed-loop tick");
            runtime.hardware_mut().advance(dt);
        }
    }

    /// 锁定闭环全链路：2 s 内转速进入目标 ±35 rpm `[rpm]`，且相电流峰值低于
    /// 1.2 A `[A]`——同时约束跟踪精度与不过流。
    /// Pins tracking accuracy and the current ceiling of the closed loop.
    #[test]
    fn speed_step_closes_controller_inverter_motor_sensor_loop() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        run_for(&mut runtime, 2.0, params.default_target_speed_rpm);
        let speed = runtime.hardware().speed_rpm();
        assert!(
            (speed - params.default_target_speed_rpm).abs() < 35.0,
            "speed={speed}"
        );
        assert!(runtime.hardware().peak_phase_current_a() < 1.2);
    }

    /// 锁定抗负载扰动能力：在 0.004 N*m `[N*m]` 负载阶跃后 2 s 内转速回到
    /// 524 rpm 的 ±15 rpm 内。
    /// Pins load-disturbance rejection within ±15 rpm after a 0.004 N*m step.
    #[test]
    fn load_step_is_rejected_and_speed_recovers() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        run_for(&mut runtime, 1.0, 524.0);
        runtime.hardware_mut().plant_mut().set_load_torque_nm(0.004);
        run_for(&mut runtime, 2.0, 524.0);
        assert!((runtime.hardware().speed_rpm() - 524.0).abs() < 15.0);
    }

    /// 锁定安全语义：功率级故障必须让 `tick` 返回 `PowerStageFault`，且 PWM 处于
    /// 关断状态（失败路径不得留下使能的功率级）。
    /// Pins that a power-stage fault returns an error AND disables PWM.
    #[test]
    fn injected_hardware_fault_disables_pwm() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        runtime.hardware_mut().inject_fault();
        assert_eq!(
            runtime.tick(SpeedCommand::default()),
            Err(HardwareFault::PowerStageFault)
        );
        assert!(!runtime.hardware().pwm_enabled());
    }

    /// 锁定反馈丢失的安全语义：先确认一拍照常使能 PWM，再断开反馈，要求下一拍
    /// 返回 `FeedbackUnavailable` 且 PWM 已关断。
    /// Pins that lost feedback errors out and leaves PWM disabled.
    #[test]
    fn lost_feedback_disables_pwm() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        runtime.tick(SpeedCommand::default()).unwrap();
        assert!(runtime.hardware().pwm_enabled());
        runtime.hardware_mut().set_feedback_available(false);
        assert_eq!(
            runtime.tick(SpeedCommand::default()),
            Err(HardwareFault::FeedbackUnavailable)
        );
        assert!(!runtime.hardware().pwm_enabled());
    }

    /// 锁定输出拒绝的安全语义：`OutputRejected` 之后 PWM 必须已关断。
    /// Pins that a rejected output command leaves PWM disabled.
    #[test]
    fn rejected_output_disables_pwm() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        runtime.hardware_mut().set_reject_output(true);
        assert_eq!(
            runtime.tick(SpeedCommand::default()),
            Err(HardwareFault::OutputRejected)
        );
        assert!(!runtime.hardware().pwm_enabled());
    }

    /// 锁定 trace 语义：负载阶跃确实生效（首样本 0.0、末样本 0.004 `[N*m]`），
    /// 样本数超过 10，且所有记录量都是有限值（无 `NaN`/`inf` 发散）。
    /// Pins the load step, sample count and finiteness of the reference trace.
    #[test]
    fn reference_trace_contains_load_step_and_finite_control_data() {
        let run = run_reference_simulation(SimulationConfig {
            duration_s: 0.05,
            load_step_time_s: 0.02,
            trace_decimation: 25,
            ..SimulationConfig::default()
        })
        .unwrap();
        assert!(run.samples.len() > 10);
        assert_eq!(run.samples.first().unwrap().load_torque_nm, 0.0);
        assert_eq!(run.samples.last().unwrap().load_torque_nm, 0.004);
        assert!(run
            .samples
            .iter()
            .all(|sample| sample.measured_speed_rpm.is_finite()
                && sample.id_a.is_finite()
                && sample.iq_a.is_finite()
                && sample.duty_a.is_finite()));
    }

    /// 锁定死区压降的**符号与量级**：`alpha` 轴电压恰好下降
    /// `2 * 550e-9 / (1/12000) * 12.3` `[V]`，容差 `1e-6`。写反号会让这个断言
    /// 以约两倍的差值失败，因此它是死区极性约定的回归点。
    /// Pins the sign and magnitude of the dead-time voltage loss.
    #[test]
    fn inverter_model_applies_expected_dead_time_voltage_loss() {
        let pwm = PwmCommand {
            duty_a: 0.6,
            duty_b: 0.4,
            duty_c: 0.5,
        };
        let currents = PhaseCurrents {
            a: 1.0,
            b: -1.0,
            c: 0.0,
        };
        let period_s = 1.0 / 12_000.0;
        let ideal = pwm_to_alpha_beta_with_inverter(
            pwm,
            currents,
            12.3,
            period_s,
            InverterSimulationConfig::default(),
        );
        let actual = pwm_to_alpha_beta_with_inverter(
            pwm,
            currents,
            12.3,
            period_s,
            InverterSimulationConfig {
                dead_time_enabled: true,
                dead_time_s: 550.0e-9,
                device_drop_v: 0.0,
            },
        );
        let expected_loss_v = 2.0 * 550.0e-9 / period_s * 12.3;
        assert!((ideal.alpha - actual.alpha - expected_loss_v).abs() < 1.0e-6);
    }

    /// 锁定死区补偿算法本身的有效性：用零额外执行延迟隔离补偿项，两组都在 10 s
    /// 内到达闭环（状态 7），补偿组的稳态
    /// Iq/Id 跟踪 RMSE 降到基线的 75% 以下，**并且**观测器 RMSE 也变好——不允许
    /// 用"电流误差换观测器误差"的方式通过。
    /// Pins that compensation improves both current and observer error.
    #[test]
    fn dead_time_compensation_reduces_closed_loop_error() {
        let baseline = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 10.0,
            closed_loop_enable: true,
            trace_decimation: 16,
            dead_time_enabled: true,
            timing: MultiRateTimingConfig {
                actuation_delay_pwm_ticks: 0,
                ..MultiRateTimingConfig::default()
            },
            ..BringupSimulationConfig::default()
        })
        .unwrap();
        let compensated = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 10.0,
            closed_loop_enable: true,
            trace_decimation: 16,
            dead_time_enabled: true,
            dead_time_feedforward_enabled: true,
            observer_dead_time_compensation_enabled: true,
            timing: MultiRateTimingConfig {
                actuation_delay_pwm_ticks: 0,
                ..MultiRateTimingConfig::default()
            },
            ..BringupSimulationConfig::default()
        })
        .unwrap();

        assert_eq!(baseline.summary.final_state, 7);
        assert_eq!(compensated.summary.final_state, 7);
        // 中文：12 kHz 下速度环会消掉大部分稳态机械转速偏差，所以相电流畸变才是
        // 死区补偿最直接的回归指标；同时要求观测器误差也改善，而不是被拿去换
        // 更低的电流误差。`2.5`/`4.0..=5.0` 这些窗口见
        // `BringupSimulationSummary` 的文档。
        // At 12 kHz the speed loop removes most steady mechanical-speed bias,
        // so phase-current distortion is the direct dead-time regression
        // metric. Observer error must still improve rather than being traded
        // for lower current error.
        assert!(
            compensated.summary.steady_iq_tracking_rmse_a
                < baseline.summary.steady_iq_tracking_rmse_a * 0.75
        );
        assert!(compensated.summary.steady_id_rmse_a < baseline.summary.steady_id_rmse_a * 0.75);
        assert!(
            compensated.summary.steady_observer_rmse_rpm
                < baseline.summary.steady_observer_rmse_rpm
        );
    }

    /// 锁定开环 bringup 链路可跑通：0.8 s 内至少 50 个抽点样本，峰值电流与真值
    /// 转速有限，且三相占空比始终落在 `[0,1]`（调制器不越界）。
    /// Pins that the open-loop firmware bridge runs and keeps duty in `[0,1]`.
    #[test]
    fn bringup_trace_runs_the_same_open_loop_bridge_as_firmware() {
        let run = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 0.8,
            trace_decimation: 160,
            ..BringupSimulationConfig::default()
        })
        .unwrap();
        assert!(run.samples.len() > 50);
        assert!(run.summary.peak_phase_current_a.is_finite());
        assert!(run.summary.final_true_speed_rpm.is_finite());
        assert!(run.samples.iter().all(|sample| {
            sample.phase_current_a.is_finite()
                && sample.iq_a.is_finite()
                && sample.duty_a.is_finite()
                && (0.0..=1.0).contains(&sample.duty_a)
        }));
    }

    /// 飞行启动预筛必须真正从指定转速/电角度起步，不能只修改 CSV
    /// 标签。首个控制拍在对象积分前采样，因此应精确保留初值。
    #[test]
    fn bringup_initial_rotor_state_is_applied_before_the_first_tick() {
        let run = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 0.01,
            initial_speed_rpm: -300.0,
            initial_electrical_angle_rad: 1.25,
            trace_decimation: 1,
            ..BringupSimulationConfig::default()
        })
        .unwrap();

        assert!((run.samples[0].true_speed_rpm + 300.0).abs() < 1.0e-4);
        assert!((run.samples[0].true_angle_rad - 1.25).abs() < 1.0e-6);
    }

    /// 锁定目标硬件候选：24 kHz 载波、12 kHz 控制、2 载波拍生效延迟。
    /// 0.25 s 恰好推进 6000 个 PWM 拍、执行 3000 次控制；末拍命令尚在 preload，
    /// 因此已生效更新为 2999 次。
    #[test]
    fn bringup_multirate_24k_12k_holds_and_applies_every_update() {
        let run = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 0.25,
            trace_decimation: 25,
            timing: MultiRateTimingConfig {
                pwm_frequency_hz: 24_000,
                control_frequency_hz: 12_000,
                actuation_delay_pwm_ticks: 2,
            },
            ..BringupSimulationConfig::default()
        })
        .unwrap();
        assert_eq!(run.summary.pwm_tick_count, 6_000);
        assert_eq!(run.summary.control_tick_count, 3_000);
        assert_eq!(run.summary.applied_pwm_update_count, 2_999);
        assert!(run.summary.final_true_speed_rpm.is_finite());
        assert!(run.samples.iter().all(|sample| sample.duty_a.is_finite()));
    }

    #[test]
    fn bringup_rejects_non_integer_multirate_plan() {
        let error = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 0.01,
            timing: MultiRateTimingConfig {
                pwm_frequency_hz: 25_000,
                control_frequency_hz: 12_000,
                actuation_delay_pwm_ticks: 0,
            },
            ..BringupSimulationConfig::default()
        })
        .unwrap_err();
        assert!(matches!(
            error,
            BringupSimulationError::InvalidConfig("invalid multi-rate timing")
        ));
    }

    /// 锁定闭环接管过程无阶跃：状态 6（观测器接管）与 7（闭环）都出现过、终态为 7，
    /// 且状态 6 之后的 `iq_ref` 单拍变化小于 0.02 A `[A]`、占空比单拍变化小于
    /// 0.05（无量纲）。这直接约束接管时的电流/电压斜率限制器。
    /// Pins a step-free closed-loop handover (current and duty slew limits).
    #[test]
    fn bringup_bridge_reaches_closed_loop_without_iq_step() {
        let run = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 10.0,
            closed_loop_enable: true,
            trace_decimation: 1,
            ..BringupSimulationConfig::default()
        })
        .unwrap();
        assert!(run.samples.iter().any(|sample| sample.state == 6));
        assert!(run.samples.iter().any(|sample| sample.state == 7));
        assert_eq!(run.summary.final_state, 7);

        let maximum_iq_step = run
            .samples
            .windows(2)
            .filter(|pair| pair[0].state >= 6 || pair[1].state >= 6)
            .map(|pair| (pair[1].iq_ref_a - pair[0].iq_ref_a).abs())
            .fold(0.0_f32, f32::max);
        assert!(maximum_iq_step < 0.02, "maximum_iq_step={maximum_iq_step}");

        let maximum_duty_step = run
            .samples
            .windows(2)
            .filter(|pair| pair[0].state >= 6 || pair[1].state >= 6)
            .map(|pair| {
                (pair[1].duty_a - pair[0].duty_a)
                    .abs()
                    .max((pair[1].duty_b - pair[0].duty_b).abs())
                    .max((pair[1].duty_c - pair[0].duty_c).abs())
            })
            .fold(0.0_f32, f32::max);
        assert!(
            maximum_duty_step < 0.05,
            "maximum_duty_step={maximum_duty_step}"
        );
    }

    /// 反向 BEMF 会把未经方向修正的 `atan2(-Ealpha,Ebeta)` 翻转 pi；这个端到端
    /// 回归锁定负目标从 Rev-Up、SMO/PLL 接管到速度环都保持同一方向。
    #[test]
    fn bringup_bridge_reaches_reverse_closed_loop() {
        let run = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 6.0,
            target_speed_rpm: -700.0,
            closed_loop_enable: true,
            startup_ramp_s: 1.8,
            trace_decimation: 12,
            observer_smo_k_slide_v: 4.0,
            observer_smo_boundary_a: 0.24,
            observer_emf_filter_alpha: 0.015,
            observer_pll_kp: 40.0,
            observer_acquisition_pll_kp_ratio: 0.5,
            observer_pll_ki: 1_000.0,
            observer_acquisition_maximum_phase_error_rad: 0.65,
            observer_run_maximum_phase_error_rad: 0.65,
            observer_consecutive_samples: 20,
            observer_acquisition_timeout_s: 5.0,
            observer_loss_timeout_s: 0.2,
            startup_transition_s: 0.05,
            handoff_torque_support_ratio: 0.47,
            speed_pi_preload_ratio: 0.20,
            closed_loop_current_slew_a_per_s: 4.0,
            dead_time_enabled: true,
            observer_dead_time_compensation_enabled: true,
            ..BringupSimulationConfig::default()
        })
        .unwrap();

        assert!(run.samples.iter().any(|sample| sample.state == 6));
        assert!(run.samples.iter().any(|sample| sample.state == 7));
        assert_eq!(run.summary.final_state, 7);
        assert!(run.summary.final_true_speed_rpm < -650.0);
        assert!(run.summary.final_observer_speed_rpm < -650.0);
        assert!(
            (run.summary.final_observer_speed_rpm - run.summary.final_true_speed_rpm).abs() < 15.0
        );
    }
}
