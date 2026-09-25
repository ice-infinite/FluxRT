//! FluxRT —— 电流环、速度环与串级控制器组合（MCSDK 参考拓扑）。
//! FluxRT - current loop, speed loop and the cascaded controller composition.
//!
//! `CurrentLoop` 是唯一每个 PWM 周期都运行的闭环：Clarke → Park → d/q PI →
//! 电压圆限幅 → 逆 Park → SVPWM。`SpeedLoop` 是 1 kHz 的外环，输出 q 轴电流
//! 给定，并支持无感切换时的积分预置（MCSDK `SWITCH_OVER` 的做法）。
//! `StReferenceController` 按分频比把两个环组合成多速率结构。
//! `CurrentLoop` is the only loop running every PWM period. `SpeedLoop` is the
//! 1 kHz outer loop producing the q-axis current reference and supporting the
//! sensorless handoff preload; `StReferenceController` composes them.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   foc-rt-bridge -> `foc-control`（本模块）-> foc-algorithm
//!   纯算法，不认识硬件：电压限幅用的是**反馈进来的**母线电压，因此本模块不需要
//!   任何端口，也不需要知道 PWM 寄存器。
//!   Pure algorithm with no hardware knowledge: the limiter uses the bus voltage
//!   from the feedback snapshot, so no port is needed here.
//!
//! 实时约束 / Real-time constraints:
//!   `CurrentLoop::update*()` 在 12 kHz ADC ISR 内执行：无动态分配、无阻塞、
//!   无日志、无 Mutex 等待。整个路径只调用一次 `sin_cos` 与一次 `magnitude`，
//!   且都走可替换的 [`ControlMath`] 后端（实机为 CORDIC，不在热路径用软件
//!   `sinf`/`cosf`）。速度环只在分频到点时运行，因此不增加热路径的固定开销。
//!   `CurrentLoop::update*` runs inside the 12 kHz ADC ISR with no allocation,
//!   blocking, logging or mutex wait, using exactly one `sin_cos` and one
//!   `magnitude` through the `ControlMath` backend.
//!
//! 量纲 / Units: 电流 `[A]`、电压 `[V]`、电角度 `[rad]`、机械角速度 `[rad/s]`、
//!   转速 `[rpm]`、占空比无量纲 `[0,1]`。全部为 `f32`，不使用 Q 格式。
//!   Currents `[A]`, voltages `[V]`, electrical angle `[rad]`, mechanical speed
//!   `[rad/s]` or `[rpm]`, duty dimensionless. Everything is `f32`.
//!
//! 参考 / Reference: docs/ST与VESC工程改进路线图.md

use core::f32::consts::PI;

use foc_algorithm::{clarke, svpwm_update, Abc, AlphaBeta, Dq, PiState, SvpwmParam};

use crate::{
    ControlMath, ControlParameters, ControlTelemetry, CpuMath, CurrentCommand, FeedbackSnapshot,
    PwmCommand, SpeedCommand,
};

/// `sqrt(3)`，用于把"SVPWM 线性区最大相电压幅值 = Vbus/sqrt(3)"写成显式常量。
/// `sqrt(3)`, making the linear-range phase amplitude Vbus/sqrt(3) explicit.
///
/// 这里用十进制字面量而不是 `3.0_f32.sqrt()`：目标板的热路径不允许额外的开方
/// 调用，而且常量折叠后的值与 C 侧参考实现逐位一致。
/// A decimal literal rather than `3.0_f32.sqrt()`: the hot path must not pay for a
/// square root, and the folded constant matches the C reference bit for bit.
const SQRT_3: f32 = 1.732_050_8;

/// Park 与逆 Park 相对“基础控制角”的独立偏移 `[rad]`。
/// Independent Park and inverse-Park offsets from the base control angle `[rad]`.
///
/// 这里使用绝对偏移而不是 MCSDK 的“第二个系数累加在第一个系数之后”的表示，
/// 避免配置时把逆 Park 的实际提前量算错。实时组合层负责把“控制拍数”乘以本拍
/// 电角速度和采样周期，换算成这两个弧度值。
/// These are absolute offsets. The realtime composition layer converts configured
/// control-tick predictions into radians from electrical speed and sample time.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlAngleOffsets {
    /// 电流采样做 Park 时的角度偏移 `[rad]`。
    /// Angle offset used by the current Park transform `[rad]`.
    pub park_rad: f32,
    /// 电压指令做逆 Park 时的角度偏移 `[rad]`。
    /// Angle offset used by the voltage inverse-Park transform `[rad]`.
    pub reverse_park_rad: f32,
}

/// 从一组基础 sin/cos 通过小角度旋转得到 `angle + offset` 的 sin/cos。
/// Rotates one base sin/cos pair to `angle + offset` using a small-angle series.
///
/// 配置把偏移限制在 ±2 个控制拍；即使按最高允许转速计算，偏移也处于小角度区。
/// 五阶 sin / 四阶 cos 在该范围内足够精确，同时避免每拍再发起一次 CORDIC 事务。
/// `offset == 0` 必须直接返回输入，保证默认 0/0 配置与改动前逐位一致。
#[inline]
fn shifted_sin_cos(sin: f32, cos: f32, offset: f32) -> (f32, f32) {
    if offset == 0.0 {
        return (sin, cos);
    }
    let offset2 = offset * offset;
    let offset4 = offset2 * offset2;
    let sin_offset = offset * (1.0 - offset2 / 6.0 + offset4 / 120.0);
    let cos_offset = 1.0 - offset2 / 2.0 + offset4 / 24.0;
    (
        sin * cos_offset + cos * sin_offset,
        cos * cos_offset - sin * sin_offset,
    )
}

/// d/q 双 PI 电流环：唯一在每个 PWM 周期都运行的闭环。
/// The d/q dual-PI current loop, the only loop running every PWM period.
///
/// 两个 PI 各自保存积分器状态，因此 `reset()` 必须在每次启动与停机时调用；
/// 否则残留积分会在下一次使能的第一个周期就把输出推到饱和电压。
/// Each PI owns its integrator, so `reset()` must run on every start and stop or
/// the leftover integral saturates the very first enabled sample.
#[derive(Clone, Copy, Debug, Default)]
pub struct CurrentLoop {
    /// d 轴电流 PI；单位：误差 `[A]` → 输出 `[V]`。
    /// d-axis current PI; `[A]` error in, `[V]` out.
    id_pi: PiState,
    /// q 轴电流 PI；通常与 `id_pi` 参数完全相同（表贴式电机 d/q 对称）。
    /// q-axis current PI; normally uses identical parameters to `id_pi`.
    iq_pi: PiState,
}

impl CurrentLoop {
    /// 清零两个 PI 的积分器与内部状态。
    /// Clears both PI integrators and internal state.
    ///
    /// 停机、故障恢复与使能前都必须调用。热复位（电机仍在旋转时复位）会让
    /// 电流环从零积分重新起跑，第一拍的电压完全由 `kp * error` 决定。
    /// Required on stop, fault recovery and before enabling. A hot reset restarts
    /// from zero integral, so the first sample's voltage is pure `kp * error`.
    pub fn reset(&mut self) {
        self.id_pi.reset();
        self.iq_pi.reset();
    }

    /// 用默认的 [`CpuMath`] 后端跑一个电流环采样。
    /// Runs one current-loop sample using the default [`CpuMath`] backend.
    ///
    /// 主机测试与不带数学加速外设的目标走这条路径；实机快环改为调用
    /// [`CurrentLoop::update_from_alpha_beta_with_math`]，以便与观测器共享同一个
    /// Clarke 结果。
    /// Host tests and targets without a math peripheral use this; the on-target
    /// fast loop calls `update_from_alpha_beta_with_math` to share one Clarke
    /// result with the observer.
    pub fn update(
        &mut self,
        params: &ControlParameters,
        feedback: &FeedbackSnapshot,
        reference: CurrentCommand,
    ) -> (PwmCommand, ControlTelemetry) {
        self.update_with_math(params, feedback, reference, &mut CpuMath)
    }

    /// 用一个可替换的数学后端跑一个电流环采样。
    /// Runs one current-loop sample using a replaceable math backend.
    ///
    /// 参数 / Parameters:
    ///   params    控制器参数（PI 增益、ts、限幅、母线可用率）
    ///   feedback  本拍反馈快照（电流 `[A]`、母线 `[V]`、电角度 `[rad]`）
    ///   reference d/q 电流给定 `[A]`
    ///   math      三角/开方后端；实机为 CORDIC，回退为 [`CpuMath`]
    ///
    /// 返回 / Returns: 三相占空比 `[0,1]` 与当拍遥测（含限幅标志）。
    /// Params are the PI gains, `ts` and limits; feedback is this sample's
    /// snapshot; reference is the d/q current in `[A]`; math is the trig backend.
    /// Returns the duty in `[0,1]` plus this sample's telemetry.
    pub fn update_with_math<M: ControlMath>(
        &mut self,
        params: &ControlParameters,
        feedback: &FeedbackSnapshot,
        reference: CurrentCommand,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        let current_alpha_beta = clarke(Abc {
            a: feedback.currents.a,
            b: feedback.currents.b,
            c: feedback.currents.c,
        });
        self.update_from_alpha_beta_with_math(params, feedback, current_alpha_beta, reference, math)
    }

    /// 用实时组合层已经算好的 Clarke 结果跑一个电流环采样。
    /// Runs one current-loop sample from a Clarke result already calculated by
    /// the realtime composition layer. This keeps Clarke a once-per-sample
    /// operation when the observer and current controller consume the same ADC
    /// snapshot.
    ///
    /// 之所以把 Clarke 拆出来，是因为观测器需要同一份 αβ 电流：重复做 Clarke 不但
    /// 浪费周期，更重要的是两次独立计算会让观测器与控制环看到"不同的"电流，在
    /// 电流环带宽附近引入不一致。
    /// Clarke is split out because the observer needs the same αβ current: a second
    /// transform wastes cycles and, more importantly, makes observer and controller
    /// see different currents, adding inconsistency near the loop bandwidth.
    pub fn update_from_alpha_beta_with_math<M: ControlMath>(
        &mut self,
        params: &ControlParameters,
        feedback: &FeedbackSnapshot,
        current_alpha_beta: AlphaBeta,
        reference: CurrentCommand,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        self.update_from_alpha_beta_with_angle_offsets_and_math(
            params,
            feedback,
            current_alpha_beta,
            reference,
            ControlAngleOffsets::default(),
            math,
        )
    }

    /// 在预计算 Clarke 路径上应用独立 Park/逆 Park 角度偏移。
    /// Applies independent Park/inverse-Park offsets on the precomputed-Clarke path.
    ///
    /// 只对基础角调用一次 `sin_cos`；两个偏移通过小角度旋转生成。默认偏移 0/0
    /// 会走直接返回分支，保持原有数值路径逐位不变。非零补偿只允许由已经过范围校验
    /// 的运行时配置进入，调用方不得传入任意大角度。
    /// Only one base `sin_cos` call is made. Zero offsets preserve the original
    /// numerical path exactly; non-zero offsets must come from validated runtime
    /// configuration.
    pub fn update_from_alpha_beta_with_angle_offsets_and_math<M: ControlMath>(
        &mut self,
        params: &ControlParameters,
        feedback: &FeedbackSnapshot,
        current_alpha_beta: AlphaBeta,
        reference: CurrentCommand,
        angle_offsets: ControlAngleOffsets,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        let (base_sin, base_cos) = math.sin_cos(feedback.rotor.electrical_angle_rad);
        let (park_sin, park_cos) = shifted_sin_cos(base_sin, base_cos, angle_offsets.park_rad);
        let current_dq = Dq {
            d: current_alpha_beta.alpha * park_cos + current_alpha_beta.beta * park_sin,
            q: -current_alpha_beta.alpha * park_sin + current_alpha_beta.beta * park_cos,
        };
        let mut voltage_dq = Dq {
            d: self
                .id_pi
                .update(&params.id_pi, reference.id_ref_a, current_dq.d),
            q: self
                .iq_pi
                .update(&params.iq_pi, reference.iq_ref_a, current_dq.q),
        };
        // 电压圆限幅半径：SVPWM 线性区的最大相电压幅值是 Vbus/sqrt(3)，再乘可用率
        //（本项目 0.95，余量留给死区、管压降与过调制）。用**实测**母线电压而不是
        // 标称值，母线跌落时才能自动收紧限幅。
        // Voltage circle radius: Vbus/sqrt(3) times the utilisation factor. Using
        // the MEASURED bus voltage lets the limit tighten automatically on sag.
        let limit = params.voltage_utilization * feedback.dc_bus_voltage / SQRT_3;
        let magnitude = math.magnitude(voltage_dq.d, voltage_dq.q);
        // `magnitude > 0.0` 防止零电压矢量被误判为"超限"后除以 0 产生 NaN。
        // The `magnitude > 0.0` guard stops the zero vector from being divided by
        // zero after being judged over-limit.
        let voltage_limited = magnitude > limit && magnitude > 0.0;
        if voltage_limited {
            // 等比缩放：只削幅值、不改方向，因此 d/q 指令的"方向"仍然可信，
            // 代价是幅值不再跟随给定（此时电流环已经失去线性度）。
            // Equal scaling clips magnitude only and preserves direction; the price
            // is that the magnitude no longer follows the reference.
            let scale = limit / magnitude;
            voltage_dq.d *= scale;
            voltage_dq.q *= scale;
        }
        let (reverse_park_sin, reverse_park_cos) =
            shifted_sin_cos(base_sin, base_cos, angle_offsets.reverse_park_rad);
        let voltage_alpha_beta = AlphaBeta {
            alpha: voltage_dq.d * reverse_park_cos - voltage_dq.q * reverse_park_sin,
            beta: voltage_dq.d * reverse_park_sin + voltage_dq.q * reverse_park_cos,
        };
        let pwm = svpwm_update(
            voltage_alpha_beta,
            &SvpwmParam {
                v_bus: feedback.dc_bus_voltage,
            },
        );
        (
            PwmCommand {
                duty_a: pwm.duty_a,
                duty_b: pwm.duty_b,
                duty_c: pwm.duty_c,
            },
            ControlTelemetry {
                current_reference: reference,
                current_dq,
                voltage_dq,
                voltage_alpha_beta,
                // 机械角速度 [rad/s] → 机械转速 [rpm]：乘 30/pi。
                // Mechanical speed [rad/s] to [rpm]: multiply by 30/pi.
                measured_speed_rpm: feedback.rotor.mechanical_speed_rad_s * 30.0 / PI,
                voltage_limited,
            },
        )
    }
}

/// 外环速度 PI：输入机械角速度 `[rad/s]`，输出 q 轴电流给定 `[A]`。
/// Outer speed PI: mechanical speed `[rad/s]` in, q-axis current `[A]` out.
///
/// 它的输出限幅**就是**电流限幅（`ControlParameters::speed_pi` 的
/// `out_min`/`out_max`），所以速度环天然受额定电流约束；母线电压能力由内环的
/// 电压圆限幅负责，速度环不需要知道母线电压。
/// The output limit IS the current limit, so the speed loop is inherently
/// current-limited; bus capability is handled by the inner loop's limiter.
#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedLoop {
    /// 速度 PI 的积分器与输出状态；单位：误差 `[rad/s]` → 输出 `[A]`。
    /// Speed PI state; `[rad/s]` error in, `[A]` out.
    pi: PiState,
}

impl SpeedLoop {
    /// 清零速度 PI 的积分器与输出状态；启动与停机时必须调用。
    /// Clears the speed PI state; required on start and stop.
    pub fn reset(&mut self) {
        self.pi.reset();
    }

    /// 把速度 PI 的输出预置到无感切换瞬间已经在流的 q 轴电流上，使切换无冲击。
    /// Matches the speed PI output to the torque-producing current already
    /// flowing at sensorless handoff. This mirrors MCSDK's speed-integral
    /// initialization in `SWITCH_OVER`.
    ///
    /// 预置只移动积分器/输出，不改变 `iq_ref` 的物理含义：切换前后电机看到的电流
    /// 是连续的，因此不出现转矩阶跃。若预置量算错（例如量纲弄混），表现就是切换
    /// 瞬间的电流跳变与机械冲击。
    /// The preload only moves the integrator/output, so the current the motor sees
    /// is continuous across the handoff and no torque step appears. A wrong preload
    /// shows up as a current jump and a mechanical shock.
    ///
    /// 参数 / Parameters: `mechanical_speed_rad_s` 单位 `[rad/s]`，
    /// `current_iq_a` 单位 `[A]`，`command.target_rpm` 单位 `[rpm]`。
    /// `mechanical_speed_rad_s` is `[rad/s]`, `current_iq_a` is `[A]` and
    /// `command.target_rpm` is `[rpm]`.
    ///
    /// 返回 / Returns: d/q 电流给定 `[A]`，其中 `id_ref_a` 原样透传。
    /// Returns the d/q current reference in `[A]`, with `id_ref_a` passed through.
    pub fn preload(
        &mut self,
        params: &ControlParameters,
        command: SpeedCommand,
        mechanical_speed_rad_s: f32,
        current_iq_a: f32,
    ) -> CurrentCommand {
        // 目标转速 [rpm] → [rad/s]，与反馈同量纲后才能相减。
        // Target speed [rpm] to [rad/s] so it is dimensionally comparable.
        let target = command.target_rpm * PI / 30.0;
        CurrentCommand {
            id_ref_a: command.id_ref_a,
            iq_ref_a: self.pi.preload_output(
                &params.speed_pi,
                target,
                mechanical_speed_rad_s,
                current_iq_a,
            ),
        }
    }

    /// 跑一个速度环采样，返回 d/q 电流给定。
    /// Runs one speed-loop sample and returns the d/q current reference.
    ///
    /// `target_rpm` 是机械转速 `[rpm]`，内部换算成 `[rad/s]` 后再与反馈相减，
    /// 因此速度 PI 的增益量纲是 `A/(rad/s)`（见 `params.rs` 的 `speed_kp`）。
    /// `target_rpm` is mechanical `[rpm]`, converted internally to `[rad/s]`, so the
    /// PI gain is in `A/(rad/s)` as produced by `params.rs`.
    ///
    /// `id_ref_a` 原样透传：速度环不参与 d 轴控制，d 轴给定由上层（弱磁/MTPA）
    /// 决定。
    /// `id_ref_a` is passed through: the speed loop does not control the d axis.
    pub fn update(
        &mut self,
        params: &ControlParameters,
        command: SpeedCommand,
        mechanical_speed_rad_s: f32,
    ) -> CurrentCommand {
        let target = command.target_rpm * PI / 30.0;
        CurrentCommand {
            id_ref_a: command.id_ref_a,
            iq_ref_a: self
                .pi
                .update(&params.speed_pi, target, mechanical_speed_rad_s),
        }
    }
}

/// 串级控制器：电流内环 + 速度外环，按固定分频比调度。
/// Cascaded controller: current inner loop plus speed outer loop, rate-divided.
///
/// 极对数、PI 参数与频率全部来自 [`ControlParameters`]；本类型不持有硬件，
/// 所以可以在 PC 仿真里与 `ControlRuntime` 组合，也可以由 `foc-rt-bridge` 直接
/// 持有并手动推进（C ABI 需要额外的启动状态机与观测器门控）。
/// Everything comes from `ControlParameters` and no hardware is held, so it pairs
/// with `ControlRuntime` in simulation or is driven directly by the bridge.
#[derive(Clone, Copy, Debug)]
pub struct StReferenceController {
    /// 完整的控制器参数集（PI 增益、频率、限幅、被控对象）。
    /// The full controller parameter set.
    params: ControlParameters,
    /// 电流内环（每拍运行）。
    /// Current inner loop, running every sample.
    current: CurrentLoop,
    /// 速度外环（按分频运行）。
    /// Speed outer loop, running at the divided rate.
    speed: SpeedLoop,
    /// 最近一次速度环输出的电流给定 `[A]`；电流环在非速度环拍上复用它。
    /// Last speed-loop current reference `[A]`, reused on non-speed samples.
    current_reference: CurrentCommand,
    /// 分频比 = 电流环频率 / 速度环频率，至少为 1。
    /// Divider = current rate / speed rate, at least 1.
    speed_divider: u32,
    /// 分频计数器，归零时才执行速度环。
    /// Divider counter; the speed loop runs when it reaches zero.
    speed_counter: u32,
}

impl StReferenceController {
    /// 由参数构建控制器；分频比取 `pwm/speed` 并至少为 1。
    /// Builds the controller; the divider is `pwm/speed`, clamped to at least 1.
    ///
    /// `.max(1)` 保证即使 C 侧把速度环频率配成等于或高于载波频率，速度环也只在
    /// 每个电流周期执行一次，而不会除零或跳拍。
    /// `.max(1)` keeps the speed loop at most once per current sample even if the C
    /// side configures a speed rate at or above the carrier, avoiding a divide by
    /// zero or skipped samples.
    pub fn new(params: ControlParameters) -> Self {
        let speed_divider = (params.pwm_frequency_hz / params.speed_loop_frequency_hz).max(1);
        Self {
            params,
            current: CurrentLoop::default(),
            speed: SpeedLoop::default(),
            current_reference: CurrentCommand::default(),
            speed_divider,
            speed_counter: 0,
        }
    }

    /// 只读访问参数集（遥测、仿真步长与分频比计算都用它）。
    /// Read-only access to the parameter set.
    pub fn parameters(&self) -> &ControlParameters {
        &self.params
    }

    /// 复位两个环的积分器、电流给定与分频计数器。
    /// Resets both integrators, the current reference and the divider counter.
    ///
    /// 必须同时清 PI 状态与分频计数器：只清积分会让多速率相位与切换前不一致，
    /// 表现为重新使能后速度环在错误的拍上运行。
    /// Both the PI state and the divider counter are cleared: clearing only the
    /// integrals would leave the multi-rate phase inconsistent.
    pub fn reset(&mut self) {
        self.current.reset();
        self.speed.reset();
        self.current_reference = CurrentCommand::default();
        self.speed_counter = 0;
    }

    /// 跑一拍完整控制（默认 [`CpuMath`] 后端）。
    /// Runs one full control tick using the default [`CpuMath`] backend.
    pub fn update(
        &mut self,
        feedback: &FeedbackSnapshot,
        command: SpeedCommand,
    ) -> (PwmCommand, ControlTelemetry) {
        self.update_with_math(feedback, command, &mut CpuMath)
    }

    /// 跑一拍完整控制：先按分频决定是否更新速度环，再跑电流环。
    /// Runs one full tick: update the speed loop when the divider says so, then
    /// run the current loop.
    ///
    /// `speed_counter == 0` 时才调用速度环，其余拍沿用上一次的电流给定——这就是
    /// 12 kHz 电流环 / 1 kHz 速度环的多速率实现。速度环因此天然带一个电流环周期
    /// 量级的零阶保持延迟，整定外环时必须把它算进相位裕度。
    /// The speed loop is called only when `speed_counter == 0`; other samples reuse
    /// the previous current reference. That is the 12 kHz/1 kHz multi-rate scheme,
    /// which adds a zero-order-hold delay the outer-loop tuning must account for.
    pub fn update_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        command: SpeedCommand,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        if self.speed_counter == 0 {
            self.current_reference =
                self.speed
                    .update(&self.params, command, feedback.rotor.mechanical_speed_rad_s);
        }
        self.speed_counter += 1;
        if self.speed_counter >= self.speed_divider {
            self.speed_counter = 0;
        }
        self.current
            .update_with_math(&self.params, feedback, self.current_reference, math)
    }

    /// 只跑电流环，电流给定由调用方直接给出（跳过速度环）。
    /// Runs only the current loop with a caller-supplied reference.
    ///
    /// 电流模式调试、给定阶跃实验与主机回归测试用它；速度外环的积分器不参与，
    /// 也不会被本函数修改。
    /// Used for current-mode bring-up, reference steps and host regression tests;
    /// the speed integrator is neither used nor modified.
    pub fn update_current(
        &mut self,
        feedback: &FeedbackSnapshot,
        command: CurrentCommand,
    ) -> (PwmCommand, ControlTelemetry) {
        self.current.update(&self.params, feedback, command)
    }

    /// 只跑电流环，使用可替换的数学后端。
    /// Runs only the current loop with a replaceable math backend.
    ///
    /// 参数 / Parameters: `command` 是 d/q 电流给定 `[A]`；返回三相占空比与遥测。
    /// `command` is the d/q current reference in `[A]`; returns duty and telemetry.
    pub fn update_current_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        command: CurrentCommand,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        self.current
            .update_with_math(&self.params, feedback, command, math)
    }
}

/// 把三相占空比重建成相电压 `[V]`：先减共模，再乘母线电压。
/// Reconstructs phase voltages `[V]` from duty: remove the common mode first,
/// then scale by the DC bus.
///
/// 必须减共模的原因是 SVPWM 刻意注入了零序分量（`(v_max+v_min)/2`，见
/// `foc-algorithm` 的 `svpwm_update`）来扩大线性区。共模电压不产生电机电流，
/// 不属于电机看到的线电压，所以重建时必须把它扣掉，只保留差模分量；否则重建
/// 电压会带上一个随扇区跳变的偏置，观测器的反电势与磁链估计会持续出错。
/// The common mode must go because SVPWM deliberately injects a zero sequence to
/// widen the linear range. Common mode drives no motor current and is not part of
/// the line-to-neutral voltage, so leaving it in adds a sector-dependent offset and
/// corrupts the observer's EMF and flux estimate.
///
/// 参数 / Parameters: `pwm` 占空比无量纲 `[0,1]`，`dc_bus_voltage` 单位 `[V]`。
/// `pwm` is dimensionless `[0,1]` and `dc_bus_voltage` is in `[V]`.
pub fn pwm_to_phase_voltage(pwm: PwmCommand, dc_bus_voltage: f32) -> Abc {
    let common = (pwm.duty_a + pwm.duty_b + pwm.duty_c) / 3.0;
    Abc {
        a: (pwm.duty_a - common) * dc_bus_voltage,
        b: (pwm.duty_b - common) * dc_bus_voltage,
        c: (pwm.duty_c - common) * dc_bus_voltage,
    }
}

/// 占空比 → 相电压 → Clarke，得到静止坐标系电压 `[V]`。
/// Duty to phase voltage to Clarke, giving the stationary-frame voltage `[V]`.
///
/// 观测器用它从**上一拍**的 PWM 命令重建实际施加的电压。必须用上一拍而不是当前
/// 拍：当前拍的命令还没经过载波周期，用它会在观测器里引入一拍超前，在高速下表现
/// 为角度估计超前、方向判断不稳。
/// The observer reconstructs the applied voltage from the PREVIOUS PWM command. Using
/// the current command would add a lead of one sample and destabilise the angle
/// estimate at speed.
pub fn pwm_to_alpha_beta(pwm: PwmCommand, dc_bus_voltage: f32) -> AlphaBeta {
    clarke(pwm_to_phase_voltage(pwm, dc_bus_voltage))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{st_gbm2804_reference_parameters, PhaseCurrents, RotorFeedback};

    /// 零误差、零角度时必须输出三相居中的零电压矢量：这条断言锁定 d/q PI 的
    /// 零输入行为与 SVPWM 的共模中点，两者任一偏移都会让电机在待机时带电。
    /// Zero error and zero angle must give the centred zero-voltage vector; this pins
    /// the PI zero-input behaviour and the SVPWM common-mode midpoint.
    #[test]
    fn zero_error_produces_centered_pwm() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback::default(),
        };
        let mut loop_ = CurrentLoop::default();
        let (pwm, telemetry) = loop_.update(&params, &feedback, CurrentCommand::default());
        assert!((pwm.duty_a - 0.5).abs() < 1e-6);
        assert!(!telemetry.voltage_limited);
    }

    /// 给定超过母线能力时必须触发电压圆限幅，且输出仍然合法（有限、`[0,1]`）。
    /// 这条断言防止"限幅被静默移除"——那会让占空比越界，实机上由 C 侧拒绝输出。
    /// An over-bus reference must engage the circle limiter while keeping the output
    /// legal, guarding against silently losing the limiter.
    #[test]
    fn circle_limit_bounds_voltage_vector() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback::default(),
        };
        let mut loop_ = CurrentLoop::default();
        let (pwm, telemetry) = loop_.update(
            &params,
            &feedback,
            CurrentCommand {
                id_ref_a: 0.8,
                iq_ref_a: 0.8,
            },
        );
        assert!(pwm.is_valid());
        assert!(telemetry.voltage_limited);
    }

    /// 计数用的假后端：`sin_cos` 返回定值 `(0,1)`，只统计调用次数。
    /// A counting fake backend: `sin_cos` returns a fixed `(0,1)` and only counts
    /// calls, so the test can assert the per-sample math budget.
    #[derive(Default)]
    struct CountingMath {
        sin_cos_calls: u32,
        magnitude_calls: u32,
    }

    impl ControlMath for CountingMath {
        fn sin_cos(&mut self, _angle_rad: f32) -> (f32, f32) {
            self.sin_cos_calls += 1;
            (0.0, 1.0)
        }

        fn magnitude(&mut self, x: f32, y: f32) -> f32 {
            self.magnitude_calls += 1;
            libm::sqrtf(x * x + y * y)
        }

        fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32 {
            foc_algorithm::atan2_angle_0_to_2pi(y, x)
        }
    }

    /// 每个电流控制周期只能各调用一次 `sin_cos` 与 `magnitude`：这是 12 kHz
    /// 热路径的预算上限，回归时最容易因为"顺手多算一次三角函数"而失效。
    /// Exactly one `sin_cos` and one `magnitude` per sample: the 12 kHz hot-path
    /// budget, which is easy to break by adding one more trig call.
    #[test]
    fn current_loop_uses_replaceable_math_backend_once_per_sample() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback::default(),
        };
        let mut loop_ = CurrentLoop::default();
        let mut math = CountingMath::default();
        let _ = loop_.update_with_math(&params, &feedback, CurrentCommand::default(), &mut math);
        assert_eq!(math.sin_cos_calls, 1);
        assert_eq!(math.magnitude_calls, 1);
    }

    /// 非零 Park/逆 Park 偏移也只能调用一次三角后端；否则目标板 12 kHz ISR 的
    /// 余量会被第二次 CORDIC 事务吃掉。
    #[test]
    fn angle_offsets_keep_one_sin_cos_transaction() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback {
                electrical_angle_rad: 1.0,
                mechanical_speed_rad_s: 40.0,
            },
        };
        let mut loop_ = CurrentLoop::default();
        let mut math = CountingMath::default();
        let _ = loop_.update_from_alpha_beta_with_angle_offsets_and_math(
            &params,
            &feedback,
            AlphaBeta::default(),
            CurrentCommand::default(),
            ControlAngleOffsets {
                park_rad: -0.12,
                reverse_park_rad: 0.18,
            },
            &mut math,
        );
        assert_eq!(math.sin_cos_calls, 1);
        assert_eq!(math.magnitude_calls, 1);
    }

    /// 小角度旋转在配置允许的 ±0.25 rad 范围内应与直接三角计算足够接近；该范围
    /// 覆盖最高转速下 ±2 控制拍，并给参数整定留出余量。
    #[test]
    fn shifted_sin_cos_matches_direct_trigonometry() {
        for base in [-2.7_f32, -0.4, 0.0, 1.3, 2.9] {
            let (sin, cos) = (libm::sinf(base), libm::cosf(base));
            for offset in [-0.25_f32, -0.1, 0.0, 0.1, 0.25] {
                let (shifted_sin, shifted_cos) = shifted_sin_cos(sin, cos, offset);
                assert!((shifted_sin - libm::sinf(base + offset)).abs() < 2.0e-6);
                assert!((shifted_cos - libm::cosf(base + offset)).abs() < 2.0e-6);
            }
        }
    }

    /// 0/0 偏移入口必须与旧入口逐位一致，确保默认关闭时没有隐含控制律变化。
    #[test]
    fn zero_angle_offsets_are_bit_identical() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents {
                a: 0.31,
                b: -0.08,
                c: -0.23,
            },
            dc_bus_voltage: 12.3,
            rotor: RotorFeedback {
                electrical_angle_rad: 2.2,
                mechanical_speed_rad_s: 60.0,
            },
        };
        let reference = CurrentCommand {
            id_ref_a: -0.03,
            iq_ref_a: 0.42,
        };
        let current_alpha_beta = clarke(Abc {
            a: feedback.currents.a,
            b: feedback.currents.b,
            c: feedback.currents.c,
        });
        let mut legacy = CurrentLoop::default();
        let mut compensated = CurrentLoop::default();
        let legacy_output = legacy.update_from_alpha_beta_with_math(
            &params,
            &feedback,
            current_alpha_beta,
            reference,
            &mut CpuMath,
        );
        let compensated_output = compensated.update_from_alpha_beta_with_angle_offsets_and_math(
            &params,
            &feedback,
            current_alpha_beta,
            reference,
            ControlAngleOffsets::default(),
            &mut CpuMath,
        );
        assert_eq!(compensated_output, legacy_output);
    }

    /// 预先算好的 Clarke 入口必须与常规入口逐位一致。实机快环走前者以共享
    /// αβ 电流，若两者语义漂移，仿真（通常走常规入口）就不再代表固件行为。
    /// The precomputed-Clarke entry must match the regular one exactly; otherwise
    /// simulation stops representing firmware behaviour.
    #[test]
    fn precomputed_clarke_path_matches_regular_current_loop() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents {
                a: 0.37,
                b: -0.22,
                c: -0.15,
            },
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback {
                electrical_angle_rad: 1.25,
                mechanical_speed_rad_s: 41.0,
            },
        };
        let reference = CurrentCommand {
            id_ref_a: 0.1,
            iq_ref_a: 0.6,
        };
        let current_alpha_beta = clarke(Abc {
            a: feedback.currents.a,
            b: feedback.currents.b,
            c: feedback.currents.c,
        });
        let mut regular = CurrentLoop::default();
        let mut precomputed = CurrentLoop::default();
        let regular_output = regular.update(&params, &feedback, reference);
        let precomputed_output = precomputed.update_from_alpha_beta_with_math(
            &params,
            &feedback,
            current_alpha_beta,
            reference,
            &mut CpuMath,
        );

        assert_eq!(precomputed_output, regular_output);
    }

    /// 预置后紧接的第一拍必须与预置输出一致（即切换点无电流阶跃）：这是
    /// "无感切换不产生转矩冲击"这一设计意图的回归保护。
    /// The first sample after preload must equal the preloaded output, i.e. no
    /// current step at handoff, protecting the bump-free switch-over intent.
    #[test]
    fn speed_loop_preload_is_bumpless_at_equal_speed() {
        let params = st_gbm2804_reference_parameters();
        let speed_rpm = 524.0;
        let speed_rad_s = speed_rpm * PI / 30.0;
        let command = SpeedCommand {
            target_rpm: speed_rpm,
            id_ref_a: 0.0,
        };
        let mut loop_ = SpeedLoop::default();
        let preloaded = loop_.preload(&params, command, speed_rad_s, 0.63);
        let first = loop_.update(&params, command, speed_rad_s);
        assert!((preloaded.iq_ref_a - 0.63).abs() < 1e-6);
        assert!((first.iq_ref_a - preloaded.iq_ref_a).abs() < 1e-6);
    }
}
