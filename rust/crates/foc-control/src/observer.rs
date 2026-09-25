//! FluxRT —— 无感转子观测器（SMO+PLL、浮点反电势+PLL）与可信门控。
//! FluxRT - sensorless rotor observers (SMO+PLL, float BEMF+PLL) and the
//! reliability gate.
//!
//! 本模块把 `foc-algorithm` 的观测器数学包装成板级后端：统一输入（αβ 电流、
//! 上一拍 PWM、母线电压）、统一输出（电角度 `[rad]` 与机械转速 `[rad/s]`），
//! 并额外提供一层"这个角度现在能不能信"的可靠性判定。
//! This wraps the `foc-algorithm` observer math into a board-level backend with a
//! uniform input (αβ current, previous PWM, DC bus), a uniform output (electrical
//! angle `[rad]`, mechanical speed `[rad/s]`) and a reliability decision.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   foc-rt-bridge -> `foc-control`（本模块）-> foc-algorithm
//!   `foc-rt-bridge` 用 `ConfigurableObserver` 按运行配置选择后端，并把
//!   `is_reliable()` 作为启动时序的切换门控；`foc-sim` 的
//!   `run_bringup_simulation` 走同一条 C ABI 路径，因此门控行为可离线复现。
//!   `foc-rt-bridge` selects a backend per runtime config and uses `is_reliable()`
//!   as the rev-up handoff gate; `foc-sim` runs the same path via the C ABI.
//!
//! 实时约束 / Real-time constraints:
//!   观测器在 12 kHz ADC ISR 内逐拍（或按 `observer_update_divider` 分频）执行：
//!   无动态分配、无阻塞、无日志。SMO-PLL 的反电势角走一次 CORDIC `atan2`，
//!   捕获期把最短路径包角误差夹到 `[-1,1]` 抑制速度方差，接管后恢复完整
//!   `[-pi,pi]` 误差；两条路径都不增加第二次 CORDIC 或软件三角函数。
//!   Runs in the 12 kHz ISR with no allocation, blocking or logging. One CORDIC
//!   `atan2` extracts the EMF angle; acquisition clamps the wrapped error to reduce
//!   variance and run mode restores the full error, without another CORDIC call.
//!
//! 量纲 / Units: 电流 `[A]`、电压 `[V]`、电角度 `[rad]`、电角速度 `[rad/s]`、
//!   机械角速度与转速分别为 `[rad/s]` 与 `[rpm]`、时间 `[s]`、电阻 `[ohm]`、
//!   电感 `[H]`。全部为 `f32`，不使用 Q 格式。
//!   Currents `[A]`, voltages `[V]`, electrical angle `[rad]`, electrical speed
//!   `[rad/s]`, mechanical speed `[rad/s]`/`[rpm]`, time `[s]`, `[ohm]`, `[H]`.
//!   Everything is `f32`; no Q formats.
//!
//! 可观性与门控 / Observability and gating:
//!   反电势幅值正比于电角速度，因此零速与低速下角度不可观。所有后端都必须靠
//!   [`RotorEstimator::is_reliable`] 明确告诉启动时序"现在还不能闭环"；绝不能
//!   因为"角度看起来是有限值"就接管闭环。
//!   The BEMF scales with electrical speed, so the angle is unobservable at low
//!   speed. Every backend must state through `is_reliable` that it cannot yet close
//!   the loop; a finite-looking angle is not evidence of convergence.
//!
//! 参考 / Reference: docs/ST与VESC工程改进路线图.md,
//!   docs/硬件数学加速与CPU回退.md

use foc_algorithm::{
    clamp, clarke, wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi, Abc, AlphaBeta, BemfInput,
    BemfParam, BemfPllParam, BemfPllState, PllParam, SmoInput, SmoParam, SmoPllParam, SmoPllState,
};

use crate::{
    command_model_observer_voltage, ControlMath, CpuMath, FeedbackSnapshot, MotorParameters,
    PwmCommand, RotorFeedback,
};

/// 可靠性窗口已经填满，均值与方差可用。
pub const OBSERVER_GATE_WINDOW_READY: u32 = 1 << 0;
/// 当前速度与窗口均值均为有限值。
pub const OBSERVER_GATE_SPEED_FINITE: u32 = 1 << 1;
/// 窗口平均机械转速高于配置的最低闭环转速。
pub const OBSERVER_GATE_SPEED_ABOVE_MINIMUM: u32 = 1 << 2;
/// 窗口平均机械转速低于电机最高转速的 110%。
pub const OBSERVER_GATE_SPEED_BELOW_MAXIMUM: u32 = 1 << 3;
/// 估计反电势幅值高于配置门限。
pub const OBSERVER_GATE_BEMF_ABOVE_MINIMUM: u32 = 1 << 4;
/// 速度窗口方差低于配置的归一化上限。
pub const OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM: u32 = 1 << 5;
/// 原始包角相位误差低于配置的锁定上限。
pub const OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM: u32 = 1 << 6;
/// 默认 SMO 可靠性门全部通过时的位掩码。
pub const OBSERVER_GATE_ALL: u32 = OBSERVER_GATE_WINDOW_READY
    | OBSERVER_GATE_SPEED_FINITE
    | OBSERVER_GATE_SPEED_ABOVE_MINIMUM
    | OBSERVER_GATE_SPEED_BELOW_MAXIMUM
    | OBSERVER_GATE_BEMF_ABOVE_MINIMUM
    | OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM
    | OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM;

/// 一拍只读观察器诊断。它不参与控制决策，只把已经计算出的内部量暴露给遥测。
/// Read-only observer diagnostics. These values never feed back into control.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ObserverDiagnostics {
    /// 滤波后的 α/β 反电势 `[V]`。
    pub bemf_alpha_v: f32,
    pub bemf_beta_v: f32,
    /// PLL 原始最短路径包角误差 `[rad]`；与相位可靠性门使用同一个量。
    /// Raw shortest-path wrapped PLL phase error `[rad]`, shared with the phase gate.
    pub pll_phase_error_rad: f32,
    /// 最近一次 1 kHz 可靠性窗口的平均机械转速 `[rpm]`。
    pub speed_mean_rpm: f32,
    /// 同一窗口的机械转速方差 `[rpm^2]`。
    pub speed_variance_rpm2: f32,
    /// `OBSERVER_GATE_*` 位，默认 SMO 只有等于 `OBSERVER_GATE_ALL` 才算本窗口稳定。
    pub reliability_flags: u32,
    /// 连续通过完整门控的 1 ms 窗口数。
    pub reliable_samples: u32,
}

/// 所有无感后端必须实现的统一接口。
/// The uniform interface every sensorless backend implements.
///
/// 输入统一为"本拍 αβ 电流 + 上一拍 PWM + 母线电压"，输出统一为电角度 `[rad]`
/// 与机械角速度 `[rad/s]`。把观测器抽象成 trait，是为了让编码器、旋变或将来
/// 位等价的 ST 定点 STO 都能替换进来而不改动电流环。
/// Encoder, resolver or a future bit-exact ST fixed-point STO can be substituted
/// without changing the current loop.
pub trait RotorEstimator {
    /// 复位到给定初始电角度 `[rad]`，并清零全部内部状态。
    /// Resets to the given initial electrical angle `[rad]` and clears all state.
    ///
    /// 强拖启动的对齐阶段结束时，用强制角度调用它，使 PLL 从已知角度起跑；
    /// 若从任意角度热复位，观测器需要额外时间重新收敛。
    /// The rev-up alignment phase calls it with the forced angle so the PLL starts
    /// from a known point.
    fn reset(&mut self, initial_electrical_angle_rad: f32);

    /// 为强拖后的第一次可观测更新准备状态。
    /// Prepares the estimator for its first observable update after forced alignment.
    ///
    /// 默认实现只复位角度；带电流模型的后端应同时把内部电流状态预置为当前实测
    /// `current_alpha_beta`。否则观察器在对齐期间保持 reset 后，会把升速首拍已经存在
    /// 的对齐电流误当成估计误差，产生一个与转子反电势无关的初始注入。
    /// The default only resets the angle. Current-model observers should also seed
    /// their internal current state from `current_alpha_beta`; otherwise the alignment
    /// current becomes an artificial first-sample estimation error.
    fn prepare_acquisition(
        &mut self,
        initial_electrical_angle_rad: f32,
        _current_alpha_beta: AlphaBeta,
    ) {
        self.reset(initial_electrical_angle_rad);
    }

    /// 在已知强拖速度处重新捕获，并用该电角速度预置 PLL 的速度前馈 `[rad/s]`。
    /// Reacquires at a known forced speed and presets the PLL speed feed-forward
    /// with that electrical angular speed `[rad/s]`.
    ///
    /// 默认后端仍退化为普通捕获；拥有 PLL 状态的后端应覆盖此方法。该提示只允许在
    /// 开环强拖仍掌握角度/速度时使用，不能拿速度指令伪装成闭环测量值。
    /// The default falls back to ordinary acquisition. PLL-backed estimators should
    /// override it. This hint is valid only while forced open-loop control still owns
    /// both angle and speed; a command must never masquerade as closed-loop feedback.
    fn prepare_acquisition_at_speed(
        &mut self,
        initial_electrical_angle_rad: f32,
        initial_electrical_speed_rad_s: f32,
        current_alpha_beta: AlphaBeta,
    ) {
        let _ = initial_electrical_speed_rad_s;
        self.prepare_acquisition(initial_electrical_angle_rad, current_alpha_beta);
    }

    /// 设置启动捕获阶段已知的机械旋转方向：`1` 正转、`-1` 反转、`0` 不约束。
    /// Sets the known mechanical direction during acquisition: `1` forward,
    /// `-1` reverse, `0` unconstrained.
    ///
    /// 这不是运行期速度指令；它只利用强拖轨迹已经知道的方向，阻止 PLL 在低速不可观
    /// 区进入相反方向的吸引域。闭环接管后调用方必须清零，让真实反转仍可被观测。
    /// This is not a speed command. It only keeps the PLL out of the opposite
    /// low-speed attractor while forced rotation provides a known direction, and
    /// must be cleared after handover so real reversal remains observable.
    fn set_acquisition_direction(&mut self, _direction: i8) {}

    /// 用调用方已经算好的 αβ 电流推进一步观测器。
    /// Advances the estimator using a caller-supplied αβ current.
    ///
    /// 参数 / Parameters:
    ///   currents_and_bus  反馈快照；本实现只取其中的母线电压 `[V]`
    ///   current_alpha_beta αβ 电流 `[A]`，与电流环共享同一次 Clarke
    ///   previous_pwm      上一拍的占空比命令，用于重建施加电压
    ///   math              三角/开方后端（实机为 CORDIC）
    ///
    /// 返回 / Returns: 电角度 `[rad]` 与机械角速度 `[rad/s]`。
    /// `currents_and_bus` supplies the bus voltage `[V]`, `current_alpha_beta` the
    /// shared Clarke result, `previous_pwm` the voltage reconstruction source.
    fn update_from_alpha_beta_with_math<M: ControlMath>(
        &mut self,
        currents_and_bus: &FeedbackSnapshot,
        current_alpha_beta: AlphaBeta,
        previous_pwm: PwmCommand,
        math: &mut M,
    ) -> RotorFeedback;

    /// 自行做 Clarke 后再推进一步（默认实现）。
    /// Runs Clarke itself and then advances one step (default implementation).
    ///
    /// 独立调用时会多算一次 Clarke，因此实机快环一律走
    /// [`RotorEstimator::update_from_alpha_beta_with_math`]。
    /// Calling this standalone pays for a second Clarke, so the fast loop always
    /// uses the precomputed entry point.
    fn update_with_math<M: ControlMath>(
        &mut self,
        currents_and_bus: &FeedbackSnapshot,
        previous_pwm: PwmCommand,
        math: &mut M,
    ) -> RotorFeedback {
        let current_alpha_beta = clarke(Abc {
            a: currents_and_bus.currents.a,
            b: currents_and_bus.currents.b,
            c: currents_and_bus.currents.c,
        });
        self.update_from_alpha_beta_with_math(
            currents_and_bus,
            current_alpha_beta,
            previous_pwm,
            math,
        )
    }

    /// 用默认 [`CpuMath`] 后端推进一步（默认实现）。
    /// Advances one step with the default [`CpuMath`] backend.
    fn update(
        &mut self,
        currents_and_bus: &FeedbackSnapshot,
        previous_pwm: PwmCommand,
    ) -> RotorFeedback {
        self.update_with_math(currents_and_bus, previous_pwm, &mut CpuMath)
    }

    /// The estimator may calculate an angle before it is trustworthy enough
    /// to close the speed loop. Startup logic must explicitly check this bit.
    ///
    /// 观测器可能已经给出角度，但该角度还不足以闭环；启动时序必须显式检查这一位。
    /// 这是本项目最重要的一条安全约定：任何"角度是有限值就闭环"的写法都会在
    /// 低速或观测器未收敛时把电流灌进错误的坐标轴。
    /// The startup logic must check this explicitly: treating a merely finite angle
    /// as convergence would drive current into the wrong axis.
    fn is_reliable(&self) -> bool;

    /// 观测器已经接管后，按运行期最低转速判断当前反馈是否仍可继续使用。
    /// Checks whether an already-acquired observer remains usable in closed loop.
    ///
    /// 启动接管门与运行保持门不能共用同一个最低转速：反电势观测器需要在较高转速
    /// 才能首次确认角度，但接管后的短时转速下探不应立刻撤销已经建立的角度轨迹。
    /// 运行保持不复用启动期 64 ms 速度方差窗：Kp 从捕获值切换到运行值会在该窗中
    /// 留下正常瞬态，若继续复用就会把“正在重新锁相”误判成失锁。运行期改由 BEMF、
    /// 有限性、速度边界和原始相位误差共同判断；最终持续时间仍由上层超时负责。
    /// Retention deliberately does not reuse the 64 ms acquisition-variance window,
    /// because the acquisition-to-run gain switch leaves a legitimate transient in
    /// that history. BEMF, finiteness, speed bounds and raw phase error guard run mode.
    fn is_reliable_for_run(&self, minimum_speed_rpm: f32, maximum_phase_error_rad: f32) -> bool;

    /// 返回最近一次只读诊断，不推进观测器，也不改变可靠性门。
    /// Returns the latest diagnostics without advancing or changing the estimator.
    fn diagnostics(&self) -> ObserverDiagnostics;
}

/// 可选的观测器后端标识，取值直接对应 C ABI 的 `observer_backend` 字段。
/// Selectable observer backend; values map onto the C ABI `observer_backend` field.
///
/// `#[repr(u32)]` 是 ABI 的一部分：数值即协议，改动就等于改 ABI。
/// `#[repr(u32)]` is part of the ABI: the discriminants are the wire protocol.
#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObserverBackend {
    /// Portable sliding-mode observer followed by a PLL. This is the default
    /// hardware-independent sensorless backend.
    ///
    /// 默认可移植后端：滑模观测器估计反电势，PLL 从反电势矢量提取角度与转速。
    /// 它是当前实机唯一真正参与闭环的无感后端。
    /// The default portable backend and the only one currently closing the loop on
    /// target hardware.
    SmoPll = 0,
    /// Portable floating-point BEMF + PLL, useful for host simulation and
    /// non-ST targets.
    ///
    /// 浮点反电势观测器 + PLL：适合主机仿真与非 ST 目标；模型是逐轴 RL 电路，
    /// 没有凸极修正，只在对齐良好的表贴式电机上准确。
    /// Float BEMF + PLL for host simulation and non-ST targets; the model is a
    /// per-axis RL circuit with no saliency correction.
    FloatBemfPll = 1,
    /// Reserved boundary for the exact ST MCSDK fixed-point STO-PLL backend.
    ///
    /// 为将来位等价的 ST MCSDK 定点 STO-PLL 预留的占位后端。目前它只是把请求
    /// 转发到浮点 BEMF 实现，且 [`RotorEstimator::is_reliable`] 恒为 `false`，
    /// 因此**不会**被用来闭环——这是刻意的安全选择，避免把"看起来像 STO"的
    /// 浮点结果当成 STO 的收敛结论。
    /// A placeholder for a future bit-exact STO. It currently forwards to the float
    /// BEMF path and reports `is_reliable() == false`, so it never closes the loop.
    StStoPll = 2,
}

/// SMO+PLL 后端的六个可整定量（可在运行时通过 C ABI 配置覆盖）。
/// The six tunables of the SMO+PLL backend, overridable at runtime through the
/// C ABI.
///
/// 两个"为什么锁不住"的典型整定错误（本题目的重点，实机调试时最先遇到）：
/// Two classic "cannot lock" tuning errors, the first things to check on hardware:
///
/// 反电势滤波系数 `emf_filter_alpha` 取**过小**：SMO 的等效反电势是靠低通滤掉
/// 开关项得到的，系数越小滤波越重、截止频率越低（约 `alpha/(2*pi*ts)` `[Hz]`），
/// 反电势矢量的相位滞后越大。这个滞后会被 PLL 的鉴相器读成"转子角度偏了"，
/// PLL 于是主动把角度往前推去追一个本来就滞后的矢量；滞后接近 90° 时鉴相增益
/// 趋零，PLL 失去回复力矩，表现为锁不住或锁在一个固定角度偏差上。此外滤波过重
/// 时反电势幅值建立很慢，可靠性门控的最小反电势条件长期不满足，切换永远不触发。
/// An EMF filter coefficient that is TOO SMALL over-smooths the equivalent control,
/// so the EMF vector lags. The PLL reads that lag as a rotor angle error and chases
/// it; as the lag approaches 90 degrees the phase-detector gain collapses and the
/// PLL cannot lock. The slow amplitude build-up also keeps the minimum-BEMF gate
/// unsatisfied, so the handoff never triggers.
///
/// 滑模增益 `k_slide_v` 取**过大**：滑模注入是开关量，其**等效平均值**才代表反电势。
/// 增益越大抖振幅值越大，而低通只能滤掉高频分量，残余抖振直接进入 `emf`，使矢量
/// 方向抖动、PLL 鉴相被噪声支配而无法建立稳定的锁定点；同时过大的注入叠加在显式
/// 欧拉电流积分上（增量约 `ts*k_slide/L`）会让估计电流过冲，滑模面本身建立不起来。
/// 反过来，`k_slide_v` 必须大于反电势峰值才可能建立滑动模态，所以它是"下限由
/// 反电势、上限由抖振与数值稳定"夹出来的窗口。
/// A sliding gain that is TOO LARGE amplifies chattering: because the EMF estimate
/// is just the low-pass of the switching term, residual chattering corrupts the
/// vector direction and the PLL cannot settle. A large injection also makes the
/// explicit Euler current estimate overshoot (`ts*k_slide/L`), so the sliding
/// surface never forms. The gain must still exceed the BEMF peak to exist at all,
/// which is why it is a window rather than a value to maximise.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmoPllTuning {
    /// 滑模增益 `[V]`，必须大于反电势峰值；过大会放大抖振（见上）。
    /// Sliding gain `[V]`; must exceed the BEMF peak, too large amplifies chattering.
    pub k_slide_v: f32,
    /// 边界层宽度 `[A]`，把 `sign` 换成线性饱和以抑制抖振；越大越平滑但等效反电势
    /// 幅值与带宽损失越大。
    /// Boundary-layer width `[A]`; smoothing trades against EMF amplitude and bandwidth.
    pub boundary_a: f32,
    /// 反电势一阶低通系数，无量纲 `(0, 1]`；过小会造成相位滞后并阻止锁定（见上）。
    /// EMF low-pass coefficient, dimensionless `(0, 1]`; too small prevents lock.
    pub emf_filter_alpha: f32,
    /// PLL 比例增益，量纲 `[rad/s per rad]`；决定锁定速度与相位裕度。
    /// PLL proportional gain; sets lock-in speed and phase margin.
    pub pll_kp: f32,
    /// 强拖捕获阶段的 PLL 校正增益比例 `[--]`，范围 `(0, 1]`；同时缩放 Kp 与 Ki，
    /// 但不缩放由强拖终速预置的速度前馈。
    /// Acquisition correction-gain ratio `(0,1]`; scales Kp and Ki but not the
    /// forced-speed feed-forward initial condition.
    pub acquisition_pll_kp_ratio: f32,
    /// PLL 积分增益，连续时间量纲 `[1/s^2]` 量级，按 `ki * ts` 离散化。
    /// PLL integral gain, discretised as `ki * ts` like every other `ki`.
    pub pll_ki: f32,
}

impl SmoPllTuning {
    /// 按电机参数给出一个"硬件相关"的起点值。
    /// Produces a hardware-correlated starting point from the machine parameters.
    ///
    /// 注意这是**起点**而不是整定结果：`k_slide_v` 按母线电压等比缩放，`boundary_a`
    /// 取额定电流的 20%，`emf_filter_alpha`/`pll_kp`/`pll_ki` 是针对高电阻
    /// GBM2804 类电机的固定值。五个值全部可以在运行时通过 C ABI 覆盖。
    /// This is a STARTING POINT, not a tuning result: the sliding gain scales with
    /// the bus, the boundary is 20 percent of rated current, and the remaining three
    /// are fixed values for the high-resistance GBM2804 class. All five are
    /// runtime-configurable.
    ///
    /// 来源标注 / Provenance: `k_slide_v` 与 `boundary_a` 由电机参数按比例导出，属于
    /// 固件默认配置（`[FW]`）并可运行时覆盖；`emf_filter_alpha`、`pll_kp`、`pll_ki`
    /// 是固定起点值，其来源类别（台架实测 `[HW]` 还是 MCSDK 参考 `[ST]`）**未在仓库内
    /// 记录**，因此不要当成实测整定值引用。整定这三个值时必须按上面的"锁不住"分析
    /// 在实物上重新扫频确认。
    /// `k_slide_v` and `boundary_a` are derived from machine parameters and are
    /// overridable firmware defaults (`[FW]`). The provenance class of the other
    /// three fixed values is not recorded in-tree, so do not cite them as `[HW]` or
    /// `[ST]`; retune them on hardware using the lock-failure analysis above.
    pub fn for_motor(motor: MotorParameters) -> Self {
        Self {
            // Hardware-correlated starting point for the high-resistance
            // GBM2804 class. All five values remain runtime-configurable.
            // 以 13 V 母线对应 4 V 滑模增益为基准按比例缩放。
            // Scaled from a 4 V gain at a 13 V bus.
            k_slide_v: motor.nominal_bus_voltage_v * (4.0 / 13.0),
            boundary_a: motor.rated_current_a * 0.20,
            // 实机重复启动表明 0.05 在少数初始条件下会把 SMO 抖振带进 PLL 的
            // 错误吸引域；0.03 在保持相位裕度的同时降低了速度窗口方差。
            // Repeated hardware starts showed that 0.05 can feed SMO chattering into
            // the wrong PLL attractor; 0.03 reduces the speed-window variance while
            // retaining adequate phase margin at the 582 rpm handover point.
            emf_filter_alpha: 0.03,
            pll_kp: 80.0,
            acquisition_pll_kp_ratio: 0.5,
            pll_ki: 1_000.0,
        }
    }

    /// 整定量自检：全部有限，增益与边界为正，滤波系数落在 `[1e-4, 1]`。
    /// Validates finiteness, positive gains and boundary, and a filter coefficient
    /// inside `[1e-4, 1]`.
    ///
    /// `emf_filter_alpha` 的下限 1e-4 就是为了挡住"滤波过重导致锁不住"这一类配置；
    /// `foc-rt-bridge` 在配置入口调用它，非法配置会被拒绝而不是半途生效。
    /// The 1e-4 lower bound exists to reject the over-filtered configurations that
    /// cannot lock; the bridge rejects invalid configs at its entry point.
    pub fn is_valid(&self) -> bool {
        self.k_slide_v.is_finite()
            && self.k_slide_v > 0.0
            && self.boundary_a.is_finite()
            && self.boundary_a > 0.0
            && self.emf_filter_alpha.is_finite()
            && (0.0001..=1.0).contains(&self.emf_filter_alpha)
            && self.pll_kp.is_finite()
            && self.pll_kp >= 0.0
            && self.acquisition_pll_kp_ratio.is_finite()
            && (0.0..=1.0).contains(&self.acquisition_pll_kp_ratio)
            && self.acquisition_pll_kp_ratio > 0.0
            && self.pll_ki.is_finite()
            && self.pll_ki >= 0.0
    }
}

/// 观测器可信门控的五类条件；只有全部满足并连续保持若干窗口，才允许无感切换。
/// The five condition groups of the observer reliability gate; all must hold for several
/// consecutive windows before the sensorless handoff is allowed.
///
/// 门控存在的理由：反电势幅值正比于转速，低速与零速下角度不可观。若不加门控就
/// 闭环，电流会被灌进错误的坐标轴，表现为电机抖动、堵转或过流，而不是"转得不好"。
/// The gate exists because the BEMF scales with speed and the angle is unobservable
/// at low speed; closing the loop without it drives current into the wrong axis.
///
/// 五类条件各自挡住的失败模式：
/// minimum_speed_rpm 挡住"低速不可观"——转速低于它时角度噪声远大于真实信息。
/// minimum_bemf_v 挡住"只有滤波器残差"——没有真实反电势时矢量方向是随机的，
/// 幅值门限确保估计的是旋转反电势而不是直流偏置或滤波残差。
/// speed_variance_ratio 挡住"PLL 还在振荡"——转速方差与均值平方之比过大说明
/// 估计尚未稳定，此时接管闭环会把振荡带进速度环。
/// maximum_phase_error_rad 挡住"速度看似稳定但角度仍未对齐"——必须检查原始包角
/// 误差，不能只看有界正弦鉴相值，否则接近 pi 时会被 `sin(pi)=0` 误判为锁定。
/// consecutive_samples 挡住"偶然通过"——启动过程中某一窗口可能碰巧满足全部条件，
/// 要求连续多个 1 kHz 窗口同时成立才算收敛。
/// Each field rejects a distinct failure mode: unobservable low speed, a flux/EMF
/// estimate that is only filter residue, a PLL still oscillating, a stable-speed but
/// phase-misaligned estimate, and a single lucky window during rev-up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ObserverReliabilityConfig {
    /// 允许闭环的最低机械转速 `[rpm]`。
    /// Minimum mechanical speed `[rpm]` at which closing the loop is allowed.
    pub minimum_speed_rpm: f32,
    /// 反电势矢量的最小幅值 `[V]`（按平方比较，避免开方）。
    /// Minimum BEMF vector magnitude `[V]`, compared squared to avoid a `sqrt`.
    pub minimum_bemf_v: f32,
    /// Maximum normalized variance: `variance / mean_speed^2`.
    ///
    /// 归一化转速方差上限 `variance / mean_speed^2`，无量纲。用比值而不是绝对值，
    /// 是为了让它与转速无关：同一条门限在 500 rpm 与 1500 rpm 下含义一致。
    /// A dimensionless ratio rather than an absolute variance, so one threshold
    /// means the same thing at 500 and 1500 rpm.
    pub speed_variance_ratio: f32,
    /// 启动获取窗允许的平均绝对包角相位误差 `[rad]`，范围 `(0, pi/2]`。
    /// Maximum mean absolute wrapped phase error over the acquisition window.
    pub maximum_phase_error_rad: f32,
    /// 需要连续满足条件的窗口个数；窗口长度为 1 ms。
    /// Number of consecutive windows that must hold; a window is 1 ms.
    pub consecutive_samples: u16,
}

impl ObserverReliabilityConfig {
    /// 按电机参数给出默认门控：最低转速取最高转速的 1/3，最小反电势 0.25 V，
    /// 归一化方差上限 0.01，需连续 2 个窗口。
    /// Defaults derived from the machine: minimum speed is one third of maximum,
    /// minimum BEMF 0.25 V, variance ratio 0.01 and two consecutive windows.
    ///
    /// 「最高转速的 1/3」是一个工程折中：太低则角度噪声主导，太高则强拖必须推到
    /// 很高的转速才能切换，启动电流与时间都会变大。
    /// One third of maximum speed is a compromise: lower lets noise dominate, higher
    /// forces a longer and more aggressive rev-up.
    ///
    /// 来源标注 / Provenance: 五个门限都是固件侧工程默认值（`[FW]`），**不是**台架
    /// 实测（`[HW]`）也不是 MCSDK 生成值（`[ST]`）。它们在 `foc-rt-bridge` 里可由
    /// `FocRuntimeConfig` 覆盖，其中 `minimum_speed_rpm` 的默认值被替换为
    /// `default_target_speed_rpm`（见该字段的说明与 `params.rs` 的警告）。
    /// All five thresholds are firmware-side engineering defaults (`[FW]`), neither
    /// rig-measured nor MCSDK-generated; the bridge can override them, and it
    /// replaces `minimum_speed_rpm` with `default_target_speed_rpm`.
    pub fn for_motor(motor: MotorParameters) -> Self {
        Self {
            minimum_speed_rpm: motor.max_speed_rpm / 3.0,
            minimum_bemf_v: 0.25,
            speed_variance_ratio: 0.01,
            maximum_phase_error_rad: 0.65,
            consecutive_samples: 2,
        }
    }

    /// 门控参数自检：全部有限，转速与反电势为正，方差比落在 `[1e-6, 1]`，
    /// 窗口个数至少为 1。
    /// Validates finiteness, positive speed and BEMF, a variance ratio inside
    /// `[1e-6, 1]` and at least one window.
    ///
    /// 方差比上限为 1 是有意义的：方差大于均值平方说明转速估计本身已经失去意义。
    /// A ratio above 1 would mean the speed estimate is meaningless.
    pub fn is_valid(&self) -> bool {
        self.minimum_speed_rpm.is_finite()
            && self.minimum_speed_rpm > 0.0
            && self.minimum_bemf_v.is_finite()
            && self.minimum_bemf_v > 0.0
            && self.speed_variance_ratio.is_finite()
            && (0.000_001..=1.0).contains(&self.speed_variance_ratio)
            && self.maximum_phase_error_rad.is_finite()
            && self.maximum_phase_error_rad > 0.0
            && self.maximum_phase_error_rad <= core::f32::consts::FRAC_PI_2
            && self.consecutive_samples > 0
    }
}

impl ObserverBackend {
    /// 把 C ABI 传进来的整数后端标识转成枚举；未知值返回 `None`。
    /// Converts the C ABI integer backend id into the enum; unknown values give
    /// `None`.
    ///
    /// 返回 `None` 而不是回退到默认后端，是刻意的：`foc_rust_configure()` 会因此
    /// 拒绝整个配置。静默回退会让"配了 A 却跑了 B"这种问题在实机上极难定位。
    /// Returning `None` instead of falling back is deliberate: the bridge then
    /// rejects the whole configuration, avoiding a silent A-configured/B-running bug.
    pub const fn from_raw(value: u32) -> Option<Self> {
        match value {
            0 => Some(Self::SmoPll),
            1 => Some(Self::FloatBemfPll),
            2 => Some(Self::StStoPll),
            _ => None,
        }
    }
}

/// 默认无感后端：滑模观测器（SMO）估计反电势，PLL 从中提取角度与转速。
/// The default sensorless backend: an SMO estimating the BEMF plus a PLL that
/// extracts angle and speed from it.
///
/// 它同时持有"观测器状态"和"可靠性统计窗口"：前者每拍更新，后者按 1 ms 抽稀更新，
/// 两者共同决定 [`RotorEstimator::is_reliable`]。
/// It holds both the observer state (per-sample) and the reliability statistics
/// (decimated to 1 ms), which together decide `is_reliable`.
#[derive(Clone, Copy, Debug)]
pub struct SmoPllEstimator {
    /// 电机参数（极对数、电阻、电感、磁链），用于建立观测器模型。
    /// Machine parameters used to build the observer model.
    motor: MotorParameters,
    /// 观测器与 PLL 的完整参数（电阻、电感、`ts`、滑模增益、边界层、滤波、PI）。
    /// The full SMO and PLL parameters.
    params: SmoPllParam,
    /// 可信门控的五类条件与窗口数。
    /// The five reliability condition groups and the window count.
    reliability: ObserverReliabilityConfig,
    /// 观测器状态：估计电流、滑模注入、滤波反电势与角度。
    /// Observer state: estimated current, sliding term, filtered EMF and angle.
    state: SmoPllState,
    /// 转速环形缓冲，长度 64；按 1 kHz 写入，因此覆盖 64 ms。
    /// Circular speed buffer of 64 entries; written at 1 kHz, covering 64 ms.
    speed_fifo: [f32; 64],
    /// 与速度窗同拍的相位误差绝对值窗口 `[rad]`；获取门使用其均值，避免单个
    /// SMO 纹波样本反复清零连续计数，同时反相锁定仍会保持接近 pi 而被拒绝。
    /// Absolute phase-error window `[rad]`; its mean rejects anti-phase lock while
    /// preventing single SMO ripple samples from resetting acquisition.
    phase_abs_fifo: [f32; 64],
    /// 环形缓冲写指针，用 `& 63` 回绕（长度是 2 的幂，避免取模除法）。
    /// Ring write index, wrapped with `& 63` since the length is a power of two.
    speed_index: usize,
    /// 已写入的样本总数（饱和累加）；小于 64 时窗口尚未填满。
    /// Total samples written (saturating); the window is incomplete below 64.
    valid_samples: u32,
    /// 连续满足全部门控条件的窗口计数，`is_reliable` 直接看它。
    /// Count of consecutive fully-satisfied windows; `is_reliable` reads it.
    reliable_samples: u16,
    /// 窗口内转速之和 `[rpm]`，用"加新减旧"递推维护。
    /// Running sum of window speeds `[rpm]`, maintained incrementally.
    speed_sum: f32,
    /// 窗口内转速平方和 `[rpm^2]`，同样递推维护，用于求方差。
    /// Running sum of squared window speeds `[rpm^2]`, used for the variance.
    speed_square_sum: f32,
    phase_abs_sum: f32,
    /// 1 ms 抽稀计数器：噪声统计不需要每拍评估。
    /// 1 ms decimation counter: the statistics need not be evaluated every sample.
    reliability_decimator: u8,
    /// 抽稀上限，等于 `1 ms / ts` 并夹到 `[1, 255]`；12 kHz 时为 12。
    /// Decimation limit, `1 ms / ts` clamped to `[1, 255]`; 12 at 12 kHz.
    reliability_decimator_limit: u8,
    /// 最近一次完整 64 ms 窗口的均值、方差和分解门控位，仅供诊断。
    diagnostic_speed_mean_rpm: f32,
    diagnostic_speed_variance_rpm2: f32,
    diagnostic_reliability_flags: u32,
    /// 最近一拍的原始包角鉴相误差 `[-pi, pi]`；运行保持门用它区分锁定点与反相点。
    /// Latest raw wrapped phase error; the run gate uses it to reject the anti-phase point.
    raw_phase_error_rad: f32,
    /// 捕获阶段 PLL Kp/Ki 相对运行增益的配置比例。
    /// Configured acquisition/run PLL correction-gain ratio.
    acquisition_pll_kp_ratio: f32,
    /// 启动捕获方向：-1/0/1；只限制 PLL 速度下/上界，不修改 BEMF 矢量。
    /// Acquisition direction (-1/0/1), applied only to the PLL speed bounds.
    acquisition_direction: i8,
}

/// 捕获期把原始包角误差夹到 `[-1,1]` rad：只有零相位一个零点，同时限制 SMO
/// 抖振造成的比例速度脉冲。不能使用 `sin(error)`，因为 `sin(+/-pi)=0` 会在
/// 正确角度的 180 度对面产生第二个平衡点。接管后不再调用本函数，而是恢复完整
/// `[-pi,pi]` 弧度误差，以保留运行 PLL 的既有增益与失锁恢复能力。
/// Acquisition clamps the wrapped error to `[-1,1]` to limit speed variance without
/// introducing a sine detector's false zero. Run mode restores the full wrapped error.
#[inline]
fn bounded_phase_error(angle_error_rad: f32) -> f32 {
    clamp(angle_error_rad, -1.0, 1.0)
}

/// `atan2(-Ealpha, Ebeta)` only equals rotor d-axis angle for positive electrical
/// speed. Reversing speed reverses the BEMF vector and adds pi to that raw angle;
/// remove the ambiguity with the known acquisition/PLL direction before handover.
#[inline]
fn bemf_angle_for_direction(raw_bemf_angle_rad: f32, direction: i8) -> f32 {
    if direction < 0 {
        wrap_angle_0_to_2pi(raw_bemf_angle_rad + core::f32::consts::PI)
    } else {
        raw_bemf_angle_rad
    }
}

/// 一阶 EMF 低通的低频群延迟近似，并把它换成随电角速度变化的相位超前。
///
/// 对 `emf += alpha * (sliding - emf)`，低频群延迟约为
/// `Ts * (1-alpha) / alpha`。离 0 Hz 越远，该线性近似会高估真实相移，因此使用
/// 0.8 的保守系数并限幅到 1.2 rad；这仍显著小于 pi/2，不会把错误方向翻转成
/// 看似正确的方向。`alpha=1` 时没有滤波延迟，补偿严格为 0。
#[inline]
fn emf_filter_phase_advance_rad(omega_rad_s: f32, alpha: f32, ts: f32) -> f32 {
    const DELAY_RATIO: f32 = 0.8;
    const MAXIMUM_ADVANCE_RAD: f32 = 1.2;
    if alpha >= 1.0 {
        return 0.0;
    }
    let delay_s = ts * (1.0 - alpha) / alpha * DELAY_RATIO;
    clamp(
        omega_rad_s * delay_s,
        -MAXIMUM_ADVANCE_RAD,
        MAXIMUM_ADVANCE_RAD,
    )
}

impl SmoPllEstimator {
    /// 用默认整定与默认门控构建；适用于快速上手与主机测试。
    /// Builds with default tuning and default gate; convenient for bring-up and
    /// host tests.
    pub fn new(motor: MotorParameters, sample_time_s: f32) -> Self {
        Self::new_with_tuning_and_reliability(
            motor,
            sample_time_s,
            SmoPllTuning::for_motor(motor),
            ObserverReliabilityConfig::for_motor(motor),
        )
    }

    /// 用指定整定与默认门控构建（实机通过 C ABI 走这条路径）。
    /// Builds with explicit tuning and the default gate; the C ABI path uses it.
    pub fn new_with_tuning(
        motor: MotorParameters,
        sample_time_s: f32,
        tuning: SmoPllTuning,
    ) -> Self {
        Self::new_with_tuning_and_reliability(
            motor,
            sample_time_s,
            tuning,
            ObserverReliabilityConfig::for_motor(motor),
        )
    }

    /// 完整构造：指定整定与门控。
    /// Full constructor with explicit tuning and reliability gate.
    ///
    /// `sample_time_s` 是**观测器的调用周期** `[s]`。注意它必须等于实际调用间隔：
    /// `foc-rt-bridge` 在 `observer_update_divider > 1` 时传入的是
    /// `divider / pwm_frequency_hz`，而不是载波周期，否则 SMO 的前向欧拉积分与 PLL
    /// 的 `ki * ts` 都会按错误的步长推进。
    /// `sample_time_s` must equal the real call interval: with a divider greater
    /// than 1 the bridge passes `divider / pwm_frequency_hz`, not the carrier period,
    /// or both the Euler integration and the PLL `ki * ts` use a wrong step.
    ///
    /// 非法整定或非法门控会**静默回退**到默认值（而不是拒绝构造），这是本模块与
    /// C ABI 配置入口的分工差别：C ABI 负责拒绝，构造函数负责给出可用起点。
    /// Invalid tuning or gate silently falls back to defaults here; rejecting is the
    /// C ABI configuration entry point's job.
    pub fn new_with_tuning_and_reliability(
        motor: MotorParameters,
        sample_time_s: f32,
        tuning: SmoPllTuning,
        reliability: ObserverReliabilityConfig,
    ) -> Self {
        let tuning = if tuning.is_valid() {
            tuning
        } else {
            SmoPllTuning::for_motor(motor)
        };
        // 额外要求最低转速门限不超过最高转速的 110%：超过它等于"永远不允许闭环"，
        // 会让电机一直停在开环保持阶段而看不出原因。
        // The extra 110 percent check rejects a gate that could never be satisfied,
        // which would silently pin the drive in open-loop hold forever.
        let reliability = if reliability.is_valid()
            && reliability.minimum_speed_rpm < motor.max_speed_rpm * 1.10
        {
            reliability
        } else {
            ObserverReliabilityConfig::for_motor(motor)
        };
        Self {
            motor,
            params: SmoPllParam {
                smo: SmoParam {
                    rs: motor.stator_resistance_ohm,
                    ls: motor.ld_h,
                    ts: sample_time_s,
                    k_slide: tuning.k_slide_v,
                    boundary: tuning.boundary_a,
                    emf_filter_alpha: tuning.emf_filter_alpha,
                },
                pll: PllParam {
                    kp: tuning.pll_kp,
                    ki: tuning.pll_ki,
                    ts: sample_time_s,
                    // 电角速度限幅 [rad/s]：±2000 rad/s 对应本机 7 对极约
                    // ±2730 rpm 机械转速，足以覆盖 1572 rpm 的上限，同时挡住
                    // 鉴相器在丢步瞬间产生的巨大虚假速度。
                    // Electrical speed clamp in [rad/s]: +/-2000 rad/s covers the
                    // 1572 rpm mechanical limit for 7 pole pairs while rejecting the
                    // huge spurious speeds a phase detector produces on loss of lock.
                    omega_min: -2_000.0,
                    omega_max: 2_000.0,
                },
            },
            reliability,
            state: SmoPllState::default(),
            speed_fifo: [0.0; 64],
            phase_abs_fifo: [0.0; 64],
            speed_index: 0,
            valid_samples: 0,
            reliable_samples: 0,
            speed_sum: 0.0,
            speed_square_sum: 0.0,
            phase_abs_sum: 0.0,
            reliability_decimator: 0,
            // 1 ms 抽稀，夹到 [1, 255]：`ts` 异常大或非法配置时至少每拍评估一次，
            // 且不会因为窄化转换而变成 0（那会让门控永不更新）。
            // 1 ms decimation clamped to [1, 255] so the gate still updates at all.
            reliability_decimator_limit: ((0.001 / sample_time_s) as u32).clamp(1, 255) as u8,
            diagnostic_speed_mean_rpm: 0.0,
            diagnostic_speed_variance_rpm2: 0.0,
            diagnostic_reliability_flags: 0,
            raw_phase_error_rad: 0.0,
            acquisition_pll_kp_ratio: tuning.acquisition_pll_kp_ratio,
            acquisition_direction: 0,
        }
    }

    /// 按 1 ms 抽稀评估一次可信门控（见 [`ObserverReliabilityConfig`] 的说明）。
    /// Evaluates the reliability gate once per 1 ms.
    ///
    /// 用 64 个样本的滑动窗口统计转速均值和方差，用"加新减旧"递推而不是每拍重算：
    /// 均值和方差需要窗口内全部样本，重算会带来 64 次乘加的固定开销。代价是递推
    /// 求和的浮点误差会随运行时间缓慢累积，所以这里每次都用 `.max(0.0)` 兜住
    /// `sqsum/64 - mean^2` 可能出现的极小负值，而不是让它变成 NaN。
    /// The 64-sample window uses incremental add-new/subtract-old sums instead of
    /// recomputing, at the cost of slowly accumulating float error; the `.max(0.0)`
    /// absorbs the small negative values that raises.
    fn update_reliability(&mut self) {
        self.reliability_decimator = self.reliability_decimator.wrapping_add(1);
        if self.reliability_decimator < self.reliability_decimator_limit {
            return;
        }
        self.reliability_decimator = 0;
        // 电角速度 [rad/s] → 机械转速 [rpm]：乘 30/pi 再除以极对数。
        // Electrical speed [rad/s] to mechanical [rpm]: 30/pi divided by pole pairs.
        let rpm =
            self.state.omega_rad_s * 30.0 / (core::f32::consts::PI * self.motor.pole_pairs as f32);
        let oldest = self.speed_fifo[self.speed_index];
        let oldest_phase_abs = self.phase_abs_fifo[self.speed_index];
        let phase_abs = self.raw_phase_error_rad.abs();
        self.speed_sum += rpm - oldest;
        self.speed_square_sum += rpm * rpm - oldest * oldest;
        self.phase_abs_sum += phase_abs - oldest_phase_abs;
        self.speed_fifo[self.speed_index] = rpm;
        self.phase_abs_fifo[self.speed_index] = phase_abs;
        self.speed_index = (self.speed_index + 1) & 63;
        self.valid_samples = self.valid_samples.saturating_add(1);
        // 窗口未填满前一律判为不可信：用不足 64 个样本算出的方差没有意义，
        // 而启动阶段恰好是"看起来已稳定"最容易误判的时候。
        // An unfilled window is never trustworthy, which matters most during rev-up.
        if self.valid_samples < 64 {
            self.reliable_samples = 0;
            self.diagnostic_speed_mean_rpm = 0.0;
            self.diagnostic_speed_variance_rpm2 = 0.0;
            self.diagnostic_reliability_flags = 0;
            return;
        }
        let mean = self.speed_sum / 64.0;
        // 方差用 E[x^2] - E[x]^2；理论上非负，浮点误差可能给出极小负值。
        // Variance as E[x^2] - E[x]^2, clamped because float error can go slightly
        // negative.
        let variance = (self.speed_square_sum / 64.0 - mean * mean).max(0.0);
        // 反电势幅值用平方比较，省掉一次每窗口的开方；量纲是 [V^2]。
        // The EMF magnitude is compared squared to save a sqrt per window.
        let emf_sq =
            self.state.emf.alpha * self.state.emf.alpha + self.state.emf.beta * self.state.emf.beta;
        // 分解保存每一道门，避免实机只留下一个 reliable=0 而无法判断失锁原因。
        let mut flags = OBSERVER_GATE_WINDOW_READY;
        if rpm.is_finite() && mean.is_finite() {
            flags |= OBSERVER_GATE_SPEED_FINITE;
        }
        // 获取期必须沿已知强拖方向超过门限；方向约束解除后用绝对值，使运行诊断
        // 对正反转保持对称。只写 `mean > minimum` 会让所有合法反转永远少一位门。
        // During acquisition the mean must exceed the threshold in the forced
        // direction. With no constraint, use magnitude for bidirectional diagnostics.
        let directed_mean = if self.acquisition_direction == 0 {
            mean.abs()
        } else {
            mean * self.acquisition_direction as f32
        };
        if directed_mean > self.reliability.minimum_speed_rpm {
            flags |= OBSERVER_GATE_SPEED_ABOVE_MINIMUM;
        }
        if mean.abs() < self.motor.max_speed_rpm * 1.10 {
            flags |= OBSERVER_GATE_SPEED_BELOW_MAXIMUM;
        }
        if emf_sq.is_finite()
            && emf_sq > self.reliability.minimum_bemf_v * self.reliability.minimum_bemf_v
        {
            flags |= OBSERVER_GATE_BEMF_ABOVE_MINIMUM;
        }
        if variance.is_finite() && variance < mean * mean * self.reliability.speed_variance_ratio {
            flags |= OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM;
        }
        let phase_abs_mean = self.phase_abs_sum / 64.0;
        if phase_abs_mean.is_finite() && phase_abs_mean <= self.reliability.maximum_phase_error_rad
        {
            flags |= OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM;
        }
        self.diagnostic_speed_mean_rpm = mean;
        self.diagnostic_speed_variance_rpm2 = variance;
        self.diagnostic_reliability_flags = flags;
        let stable = flags == OBSERVER_GATE_ALL;
        // 任何一个窗口不满足就清零连续计数：门控要求的是"连续稳定"，不是"累计稳定"。
        // Any failing window resets the streak: the gate wants consecutive stability.
        self.reliable_samples = if stable {
            self.reliable_samples.saturating_add(1)
        } else {
            0
        };
    }

    /// Clears only the statistical acquisition history, preserving the SMO and PLL
    /// dynamic states.  Full reset and BEMF-vector resynchronization share this path
    /// so stale windows can never authorize a handover.
    fn clear_reliability_history(&mut self) {
        self.speed_fifo = [0.0; 64];
        self.phase_abs_fifo = [0.0; 64];
        self.speed_index = 0;
        self.valid_samples = 0;
        self.reliable_samples = 0;
        self.speed_sum = 0.0;
        self.speed_square_sum = 0.0;
        self.phase_abs_sum = 0.0;
        self.reliability_decimator = 0;
        self.diagnostic_speed_mean_rpm = 0.0;
        self.diagnostic_speed_variance_rpm2 = 0.0;
        self.diagnostic_reliability_flags = 0;
        self.raw_phase_error_rad = 0.0;
    }
}

impl RotorEstimator for SmoPllEstimator {
    /// 复位滑模状态并把 PLL 相位预置到给定电角度 `[rad]`。
    /// Resets the SMO and presets the PLL phase to the given electrical angle `[rad]`.
    ///
    /// 同时清空可靠性窗口与计数：复用上一轮跑出来的 `valid_samples` 会让重启后
    /// 门控立刻"看起来已收敛"。清空 `speed_fifo` 是必要的，否则递推和里还留着
    /// 上一次运行的转速。
    /// The reliability window and counters are cleared too, otherwise the recursive
    /// sums would still hold speeds from the previous run.
    fn reset(&mut self, initial_electrical_angle_rad: f32) {
        self.state.reset();
        self.state.pll.reset(initial_electrical_angle_rad);
        self.clear_reliability_history();
    }

    fn prepare_acquisition(
        &mut self,
        initial_electrical_angle_rad: f32,
        current_alpha_beta: AlphaBeta,
    ) {
        self.reset(initial_electrical_angle_rad);
        // 对齐结束时相电流并不为零。把 SMO 的电流模型从同拍实测值起步，可使首拍
        // sliding=0；随后误差只由电压模型、反电势和真实动态建立，而不是由 reset
        // 的零电流与约 0.8 A 对齐电流之差随机决定符号。
        // Alignment ends with non-zero phase current. Seeding the current model from
        // the same sample makes the first sliding injection zero, so acquisition starts
        // from plant dynamics rather than an artificial reset-to-current mismatch.
        self.state.smo.current_est = current_alpha_beta;
    }

    fn prepare_acquisition_at_speed(
        &mut self,
        initial_electrical_angle_rad: f32,
        initial_electrical_speed_rad_s: f32,
        current_alpha_beta: AlphaBeta,
    ) {
        let emf_sq =
            self.state.emf.alpha * self.state.emf.alpha + self.state.emf.beta * self.state.emf.beta;
        let minimum_bemf_sq = self.reliability.minimum_bemf_v * self.reliability.minimum_bemf_v;
        if emf_sq.is_finite()
            && emf_sq >= minimum_bemf_sq
            && self.state.smo.theta_emf_rad.is_finite()
        {
            // Preserve an already-observable BEMF vector and realign only the PLL;
            // clearing the SMO can recreate the same capture basin on every retry.
            let phase_advance = emf_filter_phase_advance_rad(
                initial_electrical_speed_rad_s,
                self.params.smo.emf_filter_alpha,
                self.params.smo.ts,
            );
            let angle = wrap_angle_0_to_2pi(self.state.smo.theta_emf_rad + phase_advance);
            self.state.pll.theta_rad = angle;
            self.state.pll.omega_rad_s = initial_electrical_speed_rad_s;
            self.state.pll.integrator = initial_electrical_speed_rad_s;
            self.state.pll.phase_error = 0.0;
            self.state.theta_rad = angle;
            self.state.omega_rad_s = initial_electrical_speed_rad_s;
            self.clear_reliability_history();
            return;
        }

        self.prepare_acquisition(initial_electrical_angle_rad, current_alpha_beta);
        // 高速重捕获时若 PLL 仍从 0 rad/s 起步，首批大鉴相误差可能把积分器拉进
        // 次谐波吸引域。强拖阶段的电角速度是已知轨迹，像 MCSDK 的 STO_SetPLL
        // 一样把它只作为捕获初值；闭环后仍完全由 BEMF 鉴相更新。
        // Starting a high-speed reacquisition from zero lets the first large phase
        // errors select a sub-harmonic attractor. The forced speed is a valid initial
        // condition here (analogous to MCSDK STO_SetPLL), not measured feedback.
        // 调用者只能从已通过配置校验的 Rev-Up 终速生成这个内部提示，因此这里无需
        // 在 12 kHz 路径重复有限性与范围检查；PLL 后续更新仍保留正常限幅。
        // This internal hint comes only from a validated Rev-Up endpoint, so the
        // 12 kHz path need not repeat checks; normal PLL updates retain the clamp.
        let omega = initial_electrical_speed_rad_s;
        self.state.pll.omega_rad_s = omega;
        self.state.pll.integrator = omega;
        self.state.omega_rad_s = omega;
    }

    fn set_acquisition_direction(&mut self, direction: i8) {
        self.acquisition_direction = direction.signum();
    }

    /// 推进一步 SMO+PLL，并顺带更新可信门控。
    /// Advances the SMO+PLL one step and updates the reliability gate.
    ///
    /// 参数 / Parameters:
    ///   feedback  反馈快照，本实现只取 `dc_bus_voltage` `[V]`
    ///   current   与电流环共享的 αβ 电流 `[A]`
    ///   previous_pwm 上一拍占空比，用于重建施加电压 `[V]`
    ///   math      三角/开方后端；实机为 CORDIC
    ///
    /// 返回 / Returns: 电角度 `[rad]`（已归一化到 `[0, 2*pi)`）与机械角速度
    /// `[rad/s]`。
    fn update_from_alpha_beta_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        current: AlphaBeta,
        previous_pwm: PwmCommand,
        math: &mut M,
    ) -> RotorFeedback {
        // 用上一拍 PWM 与实测母线电压重建施加电压。占空比不知道死区与管压降，
        // 因此重建电压在轻载/过零附近误差最大，这部分误差会直接进入反电势估计。
        // The applied voltage is reconstructed from the previous duty and the
        // measured bus, so dead time and device drops appear as EMF error.
        let voltage =
            command_model_observer_voltage(previous_pwm, feedback.dc_bus_voltage).alpha_beta;
        self.state.emf = self
            .state
            .smo
            .update_vector(&self.params.smo, &SmoInput { voltage, current });
        // 反电势矢量旋转 -90° 即转子 d 轴角；实机由一次 CORDIC atan2 提取。
        let raw_bemf_angle = math.angle_0_to_2pi(-self.state.emf.alpha, self.state.emf.beta);
        // BEMF polarity flips with speed. During Rev-Up the direction is explicit;
        // after handover the previous PLL speed preserves the same branch. Without
        // this pi correction a reverse observer can look internally reliable while
        // handing the current loop an angle exactly opposite to the rotor.
        let bemf_direction = if self.acquisition_direction != 0 {
            self.acquisition_direction
        } else if self.state.pll.omega_rad_s < 0.0 {
            -1
        } else {
            1
        };
        self.state.smo.theta_emf_rad = bemf_angle_for_direction(raw_bemf_angle, bemf_direction);
        // `theta_emf_rad` 来自一阶低通后的反电势，天然滞后真实转子角。用 PLL 上一拍
        // 电速度估算该低通的群延迟相移并超前补偿，再交给 PLL；不做这一步时 582 rpm
        // 台架在 alpha=0.03 下会形成约 0.8..0.9 rad 固定偏差和双吸引域。
        // Compensate the first-order EMF-filter delay before phase detection. The
        // previous PLL speed keeps the calculation causal and adds only scalar math.
        let phase_advance = emf_filter_phase_advance_rad(
            self.state.pll.omega_rad_s,
            self.params.smo.emf_filter_alpha,
            self.params.smo.ts,
        );
        let compensated_emf_angle =
            wrap_angle_0_to_2pi(self.state.smo.theta_emf_rad + phase_advance);
        let raw_phase_error =
            wrap_angle_minus_pi_to_pi(compensated_emf_angle - self.state.pll.theta_rad);
        self.raw_phase_error_rad = raw_phase_error;
        // 捕获期限制大误差造成的速度方差；接管后恢复完整包角误差，避免改变运行
        // PLL 的既有增益与失锁恢复动态。两者都没有正弦鉴相器的 +/-pi 假零点。
        // Clamp large acquisition corrections, then restore the full wrapped error
        // after handover so run-mode PLL dynamics keep their established tuning.
        let bemf_sq =
            self.state.emf.alpha * self.state.emf.alpha + self.state.emf.beta * self.state.emf.beta;
        let minimum_bemf_sq = self.reliability.minimum_bemf_v * self.reliability.minimum_bemf_v;
        self.state.pll.phase_error = if bemf_sq >= minimum_bemf_sq {
            if self.acquisition_direction != 0 {
                bounded_phase_error(raw_phase_error)
            } else {
                raw_phase_error
            }
        } else {
            0.0
        };
        // 积分项按 ki * ts * error 离散化（ki 是连续时间增益，见 `params.rs`）。
        // The integral uses ki * ts * error with ki as a continuous-time gain.
        let acquisition_gain_ratio = if self.acquisition_direction != 0 {
            self.acquisition_pll_kp_ratio
        } else {
            1.0
        };
        self.state.pll.integrator += self.params.pll.ki
            * acquisition_gain_ratio
            * self.params.pll.ts
            * self.state.pll.phase_error;
        // 捕获时同时降低比例与积分校正，强拖终速预置的积分器基值不变；这样相位
        // 纹波不会被全 Ki 积成速度方差。接管前 bridge 清除 acquisition_direction，
        // 下一拍立即恢复完整 Kp/Ki。
        // Scale both correction branches during acquisition while retaining the
        // forced-speed integrator preload; full Kp/Ki return immediately at handover.
        let proportional_gain = self.params.pll.kp * acquisition_gain_ratio;
        let proportional = proportional_gain * self.state.pll.phase_error;
        let raw_speed = proportional + self.state.pll.integrator;
        let omega_min = if self.acquisition_direction > 0 {
            0.0
        } else {
            self.params.pll.omega_min
        };
        let omega_max = if self.acquisition_direction < 0 {
            0.0
        } else {
            self.params.pll.omega_max
        };
        self.state.pll.omega_rad_s = clamp(raw_speed, omega_min, omega_max);
        // 反算抗积分饱和：一旦限幅生效，把积分项回写成"限幅速度 - 比例项"，使积分器
        // 不再继续积累。没有这一步，限幅期间积分会一路涨到很大，限幅解除后 PLL
        // 需要很长时间才能把虚假速度吐出来，表现为角度长时间跑飞。
        // Back-calculation anti-windup: on clamping, the integrator is rewritten so it
        // cannot keep accumulating, which otherwise makes the PLL take a long time to
        // shed the spurious speed after the clamp releases.
        if raw_speed != self.state.pll.omega_rad_s {
            self.state.pll.integrator = self.state.pll.omega_rad_s - proportional;
        }
        // 角度按电角速度积分并归一化到 [0, 2*pi)。
        // The angle integrates the electrical speed and is wrapped.
        self.state.pll.theta_rad = wrap_angle_0_to_2pi(
            self.state.pll.theta_rad + self.state.pll.omega_rad_s * self.params.pll.ts,
        );
        self.state.theta_rad = self.state.pll.theta_rad;
        self.state.omega_rad_s = self.state.pll.omega_rad_s;
        self.update_reliability();
        RotorFeedback {
            electrical_angle_rad: self.state.theta_rad,
            // 电角速度 [rad/s] → 机械角速度 [rad/s]：除以极对数。
            // Electrical [rad/s] to mechanical [rad/s]: divide by pole pairs.
            mechanical_speed_rad_s: self.state.omega_rad_s / self.motor.pole_pairs as f32,
        }
    }

    fn is_reliable(&self) -> bool {
        // Reliability is evaluated at 1 kHz after the 64-sample speed window.
        // The required consecutive windows are runtime-configurable.
        //
        // 门控在 1 kHz 上评估（64 个 1 ms 窗口），因此 `is_reliable` 每拍读取的是
        // 最近一次评估的结果，最多滞后 1 ms；对毫秒级的启动时序来说可以接受，
        // 但不要用它做拍级保护。
        // Evaluated at 1 kHz, so this returns a value up to 1 ms stale; it is fine for
        // the millisecond-scale rev-up timing but is not a per-sample protection.
        self.reliable_samples >= self.reliability.consecutive_samples
    }

    fn is_reliable_for_run(&self, minimum_speed_rpm: f32, maximum_phase_error_rad: f32) -> bool {
        // `minimum_speed_rpm` in `self.reliability` is the acquisition threshold.
        // Once the observer owns the angle, retain every gate except that one and
        // apply the separately configured run threshold. This mirrors MCSDK's
        // `MinStartUpValidSpeed` versus `hMinReliableMecSpeedUnit` split.
        // Acquisition phase reliability is a 64 ms mean-absolute-error window.
        // Requiring that historical window again after handover defeats the
        // separately configured instantaneous run threshold: during a commanded
        // acceleration the raw error can already be back in range while the old
        // acquisition samples keep the window bit low for another 64 ms.  Keep the
        // finite/max-speed/BEMF safety gates, but replace acquisition speed,
        // variance *and phase* with their run-mode checks below.
        let required_for_run = OBSERVER_GATE_ALL
            & !OBSERVER_GATE_SPEED_ABOVE_MINIMUM
            & !OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM
            & !OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM;
        (self.diagnostic_reliability_flags & required_for_run) == required_for_run
            && self.diagnostic_speed_mean_rpm.is_finite()
            && self.diagnostic_speed_mean_rpm.abs() >= minimum_speed_rpm
            && self.raw_phase_error_rad.is_finite()
            && self.raw_phase_error_rad.abs() <= maximum_phase_error_rad
    }

    fn diagnostics(&self) -> ObserverDiagnostics {
        ObserverDiagnostics {
            bemf_alpha_v: self.state.emf.alpha,
            bemf_beta_v: self.state.emf.beta,
            // 遥测必须报告门控使用的原始包角，而不是控制内部的有界正弦鉴相量；
            // 否则接近 pi 时 `sin(pi)=0` 会让日志看起来像已经锁定。
            pll_phase_error_rad: self.raw_phase_error_rad,
            speed_mean_rpm: self.diagnostic_speed_mean_rpm,
            speed_variance_rpm2: self.diagnostic_speed_variance_rpm2,
            reliability_flags: self.diagnostic_reliability_flags,
            reliable_samples: self.reliable_samples as u32,
        }
    }
}

/// Floating-point BEMF + PLL adapter with the same observer topology as the
/// reference MCSDK STO-PLL. It is intentionally separate from the current
/// controller so an encoder, resolver, or a later bit-equivalent STO port can
/// be substituted without changing the FOC loop.
///
/// 浮点反电势观测器 + PLL 适配层，拓扑与参考 MCSDK STO-PLL 一致。它与电流环刻意
/// 解耦，因此编码器、旋变或将来位等价的 STO 移植都能替换进来而不改动 FOC 环。
/// The float BEMF + PLL adapter mirrors the MCSDK STO-PLL topology and is kept
/// separate from the current loop so another estimator can be substituted.
///
/// 它与 SMO 后端的差别：模型是逐轴 RL 电路直接求 `e = v - Rs*i - Ls*di/dt`，
/// 没有滑模切换项，因此没有抖振，但对参数误差与电流噪声更敏感（`di/dt` 用后向
/// 差分，噪声增益为 `Ls/ts`）。
/// Unlike the SMO backend it computes `e = v - Rs*i - Ls*di/dt` directly, so there
/// is no chattering, but it is more sensitive to parameter error and current noise.
#[derive(Clone, Copy, Debug)]
pub struct BemfPllEstimator {
    /// 电机参数，用于建立逐轴 RL 模型与电/机械换算。
    /// Machine parameters for the RL model and the electrical/mechanical conversion.
    motor: MotorParameters,
    /// 观测器与 PLL 参数（电阻、电感、`ts`、反电势滤波、PLL 增益与限幅）。
    /// Observer and PLL parameters.
    params: BemfPllParam,
    /// 观测器状态（上一拍电流、滤波反电势、角度、PLL）。
    /// Observer state: last current, filtered EMF, angle and PLL.
    state: BemfPllState,
    /// 已调用次数（饱和累加），仅用于 `is_reliable` 的初始瞬态保护。
    /// Call count (saturating), used only for the initial-transient protection.
    valid_samples: u32,
    /// 启动捕获方向：-1/0/1；闭环后由调用方清零。
    /// Acquisition direction (-1/0/1), cleared by the caller after handover.
    acquisition_direction: i8,
}

impl BemfPllEstimator {
    /// 构建浮点反电势观测器；`sample_time_s` 必须等于真实调用周期 `[s]`。
    /// Builds the estimator; `sample_time_s` must equal the real call period `[s]`.
    ///
    /// 来源标注 / Provenance: 电阻与电感取自 `MotorParameters`（Workbench 继承值，
    /// 未在实物上辨识）；`emf_filter_alpha = 0.08` 与 PLL 增益 `220`/`12000` 是固件
    /// 侧的浮点实验起点（`[FW]`），**不是** MCSDK 生成增益（`[ST]`）——后者是定点
    /// 量纲，不能直接抄成浮点，见下方注释。
    /// Resistance and inductance come from `MotorParameters` (Workbench-inherited,
    /// not identified); the filter coefficient and PLL gains are firmware-side float
    /// starting values (`[FW]`), not the MCSDK generated fixed-point gains (`[ST]`).
    pub fn new(motor: MotorParameters, sample_time_s: f32) -> Self {
        Self {
            motor,
            params: BemfPllParam {
                bemf: BemfParam {
                    rs: motor.stator_resistance_ohm,
                    ls: motor.ld_h,
                    ts: sample_time_s,
                    // 0.08 比 SMO 路径的 0.05 更轻：这条路径没有开关抖振需要压，
                    // 滤波越重相位滞后越大（见 `SmoPllTuning` 的说明）。
                    // 0.08 is lighter than the SMO path's 0.05: there is no switching
                    // chattering to suppress here, and heavier filtering only adds lag.
                    emf_filter_alpha: 0.08,
                },
                // Float-domain PLL bandwidth for host/target experimentation.
                // The generated fixed-point gains 195/16384 and 5/65535 are
                // retained in docs; they are not dimensionally interchangeable.
                //
                // 浮点域 PLL 带宽，供主机与目标实验用。MCSDK 生成的定点增益
                // 195/16384 与 5/65535 记录在文档里，但**量纲不可互换**：定点增益
                // 建立在 Q 格式与定点角度上，直接抄进浮点只会得到错误带宽。
                // Float-domain PLL gains for experimentation; the generated
                // fixed-point gains are not dimensionally interchangeable with these.
                pll: PllParam {
                    kp: 220.0,
                    ki: 12_000.0,
                    ts: sample_time_s,
                    omega_min: -2_000.0,
                    omega_max: 2_000.0,
                },
            },
            state: BemfPllState::default(),
            valid_samples: 0,
            acquisition_direction: 0,
        }
    }

    /// 返回最近一次估计的电角速度 `[rad/s]`（测试与遥测用）。
    /// Returns the last estimated electrical speed `[rad/s]` for tests and telemetry.
    pub fn electrical_speed_rad_s(&self) -> f32 {
        self.state.omega_rad_s
    }
}

impl RotorEstimator for BemfPllEstimator {
    /// 清空观测器状态并把 PLL 相位预置到给定电角度 `[rad]`。
    /// Clears the observer and presets the PLL phase to the given electrical angle.
    fn reset(&mut self, initial_electrical_angle_rad: f32) {
        self.state.reset();
        self.state.pll.reset(initial_electrical_angle_rad);
        self.valid_samples = 0;
    }

    fn prepare_acquisition(
        &mut self,
        initial_electrical_angle_rad: f32,
        current_alpha_beta: AlphaBeta,
    ) {
        self.reset(initial_electrical_angle_rad);
        // 避免后向差分在首拍把对齐电流与零初值相减，生成虚假的 L*di/dt。
        // Avoid a false L*di/dt term from differencing the alignment current against zero.
        self.state.bemf.last_current = current_alpha_beta;
        self.state.bemf.initialized = 1;
    }

    fn set_acquisition_direction(&mut self, direction: i8) {
        self.acquisition_direction = direction.signum();
    }

    /// 推进一步浮点反电势观测器。
    /// Advances the float BEMF observer one step.
    ///
    /// `math` 在本实现中未使用（角度由 `foc-algorithm` 内部计算），保留该参数是
    /// 为了让所有后端共享同一个 trait 签名，便于替换与对比。
    /// `math` is unused here because the angle is computed inside `foc-algorithm`;
    /// the parameter exists so all backends share one signature.
    fn update_from_alpha_beta_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        current: AlphaBeta,
        previous_pwm: PwmCommand,
        _math: &mut M,
    ) -> RotorFeedback {
        let voltage =
            command_model_observer_voltage(previous_pwm, feedback.dc_bus_voltage).alpha_beta;
        let mut params = self.params;
        if self.acquisition_direction > 0 {
            params.pll.omega_min = 0.0;
        } else if self.acquisition_direction < 0 {
            params.pll.omega_max = 0.0;
        }
        let electrical_angle_rad = self.state.update(&params, &BemfInput { voltage, current });
        self.valid_samples = self.valid_samples.saturating_add(1);
        RotorFeedback {
            electrical_angle_rad,
            // 电角速度 [rad/s] → 机械角速度 [rad/s]：除以极对数。
            // Electrical [rad/s] to mechanical [rad/s]: divide by pole pairs.
            mechanical_speed_rad_s: self.state.omega_rad_s / self.motor.pole_pairs as f32,
        }
    }

    fn is_reliable(&self) -> bool {
        // This deliberately is not an MCSDK convergence claim. It only keeps
        // the portable backend from being consumed during its initial FIFO/
        // PLL transient. The ST-compatible backend will supply its own
        // variance and BEMF-consistency decision.
        //
        // 这里刻意**不**声称达到了 MCSDK 的收敛判据：它只保证不在初始 FIFO/PLL
        // 瞬态期间被闭环使用（2048 拍 ≈ 170 ms）。真正的方差与反电势一致性判据
        // 由将来的 ST 兼容后端提供；在它到位之前，这条后端只用于主机验证。
        // This deliberately is not an MCSDK convergence claim: it only covers the
        // initial transient (2048 samples) and this backend stays host-only until a
        // real variance/BEMF-consistency decision exists.
        //
        // 来源标注 / Provenance: 2048 拍是固件侧选定的工程默认值（`[FW]`），不是
        // MCSDK 的收敛判据（`[ST]`）；在 12 kHz 下约 170 ms。
        // The 2048-sample threshold is a firmware-side default (`[FW]`), not an MCSDK
        // convergence criterion (`[ST]`); it is about 170 ms at 12 kHz.
        self.valid_samples >= 2_048 && self.state.omega_rad_s.is_finite()
    }

    fn is_reliable_for_run(&self, minimum_speed_rpm: f32, maximum_phase_error_rad: f32) -> bool {
        let speed_rpm =
            self.state.omega_rad_s * 30.0 / (core::f32::consts::PI * self.motor.pole_pairs as f32);
        self.is_reliable()
            && speed_rpm.abs() >= minimum_speed_rpm
            && self.state.pll.phase_error.is_finite()
            && self.state.pll.phase_error.abs() <= maximum_phase_error_rad
    }

    fn diagnostics(&self) -> ObserverDiagnostics {
        let speed_rpm =
            self.state.omega_rad_s * 30.0 / (core::f32::consts::PI * self.motor.pole_pairs as f32);
        let mut flags = 0;
        if self.valid_samples >= 2_048 {
            flags |= OBSERVER_GATE_WINDOW_READY;
        }
        if speed_rpm.is_finite() {
            flags |= OBSERVER_GATE_SPEED_FINITE;
        }
        ObserverDiagnostics {
            bemf_alpha_v: self.state.emf.alpha,
            bemf_beta_v: self.state.emf.beta,
            pll_phase_error_rad: self.state.pll.phase_error,
            speed_mean_rpm: speed_rpm,
            speed_variance_rpm2: 0.0,
            reliability_flags: flags,
            reliable_samples: self.is_reliable() as u32,
        }
    }
}

/// 按运行配置选择后端的包装器；两个后端都被原地持有，切换不需要分配。
/// A backend-selecting wrapper; both backends are held inline so switching needs
/// no allocation.
///
/// 这就是 C ABI 实际使用的类型：`foc_rt_bridge` 通过它把 `observer_backend`
/// 配置映射到具体实现，并把 `is_reliable()` 作为强拖切换门控。
/// This is the type the C ABI actually uses: the bridge maps its `observer_backend`
/// setting onto it and uses `is_reliable()` as the rev-up handoff gate.
#[derive(Clone, Copy, Debug)]
pub struct ConfigurableObserver {
    /// 当前选择的后端标识。
    /// The currently selected backend id.
    backend: ObserverBackend,
    /// SMO+PLL 后端实例（即使未选中也会被构造，换取无分配与 `const` 尺寸）。
    /// The SMO+PLL instance, always constructed to keep the type allocation-free.
    smo: SmoPllEstimator,
    /// 浮点反电势后端实例；`FloatBemfPll` 与 `StStoPll` 都路由到它。
    /// The float BEMF instance; both `FloatBemfPll` and `StStoPll` route to it.
    bemf: BemfPllEstimator,
}

impl ConfigurableObserver {
    /// 用默认 SMO 整定与默认门控构建指定后端。
    /// Builds the selected backend with default SMO tuning and gate.
    pub fn new(backend: ObserverBackend, motor: MotorParameters, sample_time_s: f32) -> Self {
        Self::new_with_smo_tuning(
            backend,
            motor,
            sample_time_s,
            SmoPllTuning::for_motor(motor),
        )
    }

    /// 用指定 SMO 整定与默认门控构建。
    /// Builds with explicit SMO tuning and the default gate.
    pub fn new_with_smo_tuning(
        backend: ObserverBackend,
        motor: MotorParameters,
        sample_time_s: f32,
        tuning: SmoPllTuning,
    ) -> Self {
        Self::new_with_smo_tuning_and_reliability(
            backend,
            motor,
            sample_time_s,
            tuning,
            ObserverReliabilityConfig::for_motor(motor),
        )
    }

    /// 完整构造：同时指定 SMO 整定与可信门控（即 C ABI 的配置路径）。
    /// Full constructor with explicit tuning and gate, i.e. the C ABI config path.
    ///
    /// 两个后端都会被构造，但只有 `backend` 选中的那个会被更新与查询；另一个
    /// 保持默认状态，不消耗运行时开销，代价是每实例多占一份状态。
    /// Both are constructed but only the selected one is updated or queried.
    pub fn new_with_smo_tuning_and_reliability(
        backend: ObserverBackend,
        motor: MotorParameters,
        sample_time_s: f32,
        tuning: SmoPllTuning,
        reliability: ObserverReliabilityConfig,
    ) -> Self {
        Self {
            backend,
            smo: SmoPllEstimator::new_with_tuning_and_reliability(
                motor,
                sample_time_s,
                tuning,
                reliability,
            ),
            bemf: BemfPllEstimator::new(motor, sample_time_s),
        }
    }

    /// 返回当前后端标识（遥测把它的数值直接上报给 C）。
    /// Returns the selected backend id, reported numerically to C in telemetry.
    pub const fn backend(&self) -> ObserverBackend {
        self.backend
    }
}

impl RotorEstimator for ConfigurableObserver {
    /// 复位当前选中的后端；未选中的后端保持不动。
    /// Resets the selected backend only.
    ///
    /// 只复位选中者是刻意的：切换配置必须经 `foc_rust_configure()`（它重建整个
    /// 观测器），因此这里不需要同时维护两条状态轨迹。
    /// Only the selected backend is reset: changing backends goes through
    /// `foc_rust_configure()`, which rebuilds the observer.
    fn reset(&mut self, initial_electrical_angle_rad: f32) {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.reset(initial_electrical_angle_rad),
            ObserverBackend::StStoPll | ObserverBackend::FloatBemfPll => {
                self.bemf.reset(initial_electrical_angle_rad)
            }
        }
    }

    fn prepare_acquisition(
        &mut self,
        initial_electrical_angle_rad: f32,
        current_alpha_beta: AlphaBeta,
    ) {
        match self.backend {
            ObserverBackend::SmoPll => self
                .smo
                .prepare_acquisition(initial_electrical_angle_rad, current_alpha_beta),
            ObserverBackend::StStoPll | ObserverBackend::FloatBemfPll => self
                .bemf
                .prepare_acquisition(initial_electrical_angle_rad, current_alpha_beta),
        }
    }

    fn set_acquisition_direction(&mut self, direction: i8) {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.set_acquisition_direction(direction),
            ObserverBackend::StStoPll | ObserverBackend::FloatBemfPll => {
                self.bemf.set_acquisition_direction(direction)
            }
        }
    }

    fn prepare_acquisition_at_speed(
        &mut self,
        initial_electrical_angle_rad: f32,
        initial_electrical_speed_rad_s: f32,
        current_alpha_beta: AlphaBeta,
    ) {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.prepare_acquisition_at_speed(
                initial_electrical_angle_rad,
                initial_electrical_speed_rad_s,
                current_alpha_beta,
            ),
            ObserverBackend::FloatBemfPll | ObserverBackend::StStoPll => self
                .bemf
                .prepare_acquisition(initial_electrical_angle_rad, current_alpha_beta),
        }
    }

    /// 把本拍更新路由到当前选中的后端。
    /// Routes this sample's update to the selected backend.
    fn update_from_alpha_beta_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        current_alpha_beta: AlphaBeta,
        previous_pwm: PwmCommand,
        math: &mut M,
    ) -> RotorFeedback {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.update_from_alpha_beta_with_math(
                feedback,
                current_alpha_beta,
                previous_pwm,
                math,
            ),
            ObserverBackend::StStoPll | ObserverBackend::FloatBemfPll => self
                .bemf
                .update_from_alpha_beta_with_math(feedback, current_alpha_beta, previous_pwm, math),
        }
    }

    /// 返回选中后端的可信判定。
    /// Returns the selected backend's reliability decision.
    fn is_reliable(&self) -> bool {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.is_reliable(),
            // The portable equations are not yet bit-equivalent to MCSDK's
            // fixed-point STO-PLL, so the ST selection is telemetry-only for
            // this first hardware milestone and must not close the loop.
            //
            // 可移植方程尚未与 MCSDK 定点 STO-PLL 位等价，因此在第一个硬件里程碑里
            // ST 选项只用于遥测对比，恒返回 false 以**禁止**闭环。把它改成 true
            // 之前必须先完成位等价移植与台架验证。
            // The portable equations are not bit-equivalent to the MCSDK fixed-point
            // STO-PLL yet, so the ST selection is telemetry-only and hard-wired to
            // false; enabling it requires a bit-exact port and rig validation first.
            ObserverBackend::StStoPll => false,
            ObserverBackend::FloatBemfPll => self.bemf.is_reliable(),
        }
    }

    fn is_reliable_for_run(&self, minimum_speed_rpm: f32, maximum_phase_error_rad: f32) -> bool {
        match self.backend {
            ObserverBackend::SmoPll => self
                .smo
                .is_reliable_for_run(minimum_speed_rpm, maximum_phase_error_rad),
            // ST selection remains telemetry-only until the bit-equivalent backend
            // is implemented; it must not gain a closed-loop path through this new
            // retention API.
            ObserverBackend::StStoPll => false,
            ObserverBackend::FloatBemfPll => self
                .bemf
                .is_reliable_for_run(minimum_speed_rpm, maximum_phase_error_rad),
        }
    }

    fn diagnostics(&self) -> ObserverDiagnostics {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.diagnostics(),
            ObserverBackend::StStoPll | ObserverBackend::FloatBemfPll => self.bemf.diagnostics(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{st_gbm2804_reference_parameters, PhaseCurrents};
    use foc_algorithm::{svpwm_update, AlphaBeta, SvpwmParam};

    /// SMO 捕获鉴相器必须保持在 `[-1,1]`，并且只能有 0 这一个
    /// 锁定点。特别锁定 +/-pi 端点的输出，避免退回到会产生 180 度假锁的
    /// 正弦鉴相。
    /// The realtime discriminator is bounded and has no false zero at +/-pi.
    #[test]
    fn smo_phase_discriminator_is_bounded_with_one_zero() {
        for index in -256..=256 {
            let angle = core::f32::consts::PI * index as f32 / 256.0;
            let actual = bounded_phase_error(angle);
            assert!((-1.000_001..=1.000_001).contains(&actual));
            if angle > 0.0 {
                assert!(actual > 0.0);
            } else if angle < 0.0 {
                assert!(actual < 0.0);
            } else {
                assert_eq!(actual, 0.0);
            }
        }
        assert_eq!(bounded_phase_error(core::f32::consts::PI), 1.0);
        assert_eq!(bounded_phase_error(-core::f32::consts::PI), -1.0);
    }

    /// 反转时 BEMF 矢量整体变号，裸 `atan2(-Ealpha,Ebeta)` 比真实转子角多/少 pi；
    /// 方向修正必须把正反向同一转子位置映射到同一个 d 轴角。
    #[test]
    fn bemf_angle_polarity_is_direction_aware() {
        let theta = 0.7_f32;
        let positive_raw = libm::atan2f(libm::sinf(theta), libm::cosf(theta));
        let negative_raw = libm::atan2f(-libm::sinf(theta), -libm::cosf(theta));
        let positive = bemf_angle_for_direction(wrap_angle_0_to_2pi(positive_raw), 1);
        let negative = bemf_angle_for_direction(wrap_angle_0_to_2pi(negative_raw), -1);
        assert!(wrap_angle_minus_pi_to_pi(positive - theta).abs() < 1.0e-6);
        assert!(wrap_angle_minus_pi_to_pi(negative - theta).abs() < 1.0e-6);
    }

    /// 用一个恒定电角速度 200 rad/s 的理想反电势矢量驱动浮点后端一整秒，
    /// 断言 PLL 收敛到该转速（容差 5 rad/s）。它锁定的是"鉴相极性 + PLL 增益
    /// 量纲 + 每秒 12000 拍的离散化"三者的一致性；符号写反会让估计转速变成
    /// -200 rad/s 而断言立刻失败，这正是该测试要挡住的最常见错误。
    /// Drives the float backend for one second with an ideal 200 rad/s BEMF vector
    /// and asserts the PLL converges within 5 rad/s, pinning phase-detector polarity,
    /// PLL gain dimensions and the 12 kHz discretisation.
    #[test]
    fn bemf_pll_tracks_a_rotating_nonzero_speed_vector() {
        let control = st_gbm2804_reference_parameters();
        let dt = 1.0 / control.pwm_frequency_hz as f32;
        let mut estimator = BemfPllEstimator::new(control.motor, dt);
        estimator.reset(0.0);
        let electrical_speed = 200.0;
        let mut theta: f32 = 0.0;
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: control.motor.nominal_bus_voltage_v,
            rotor: RotorFeedback::default(),
        };
        for _ in 0..control.pwm_frequency_hz {
            theta = (theta + electrical_speed * dt).rem_euclid(2.0 * core::f32::consts::PI);
            let voltage = AlphaBeta {
                alpha: -2.0 * libm::sinf(theta),
                beta: 2.0 * libm::cosf(theta),
            };
            let pwm = svpwm_update(
                voltage,
                &SvpwmParam {
                    v_bus: feedback.dc_bus_voltage,
                },
            );
            estimator.update(
                &feedback,
                PwmCommand {
                    duty_a: pwm.duty_a,
                    duty_b: pwm.duty_b,
                    duty_c: pwm.duty_c,
                },
            );
        }
        assert!((estimator.electrical_speed_rad_s() - electrical_speed).abs() < 5.0);
    }

    /// 升速首拍的实测电流应成为 SMO 电流模型初值，而不是被解释成滑模误差。
    /// The first acquisition sample seeds the SMO current model instead of becoming
    /// an artificial sliding error.
    #[test]
    fn smo_acquisition_seed_removes_alignment_current_step() {
        let control = st_gbm2804_reference_parameters();
        let dt = 1.0 / control.pwm_frequency_hz as f32;
        let mut estimator = SmoPllEstimator::new(control.motor, dt);
        let current = AlphaBeta {
            alpha: 0.8,
            beta: -0.2,
        };
        estimator.prepare_acquisition(0.0, current);
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: control.motor.nominal_bus_voltage_v,
            rotor: RotorFeedback::default(),
        };
        let mut math = CpuMath;
        let _ = estimator.update_from_alpha_beta_with_math(
            &feedback,
            current,
            PwmCommand {
                duty_a: 0.5,
                duty_b: 0.5,
                duty_c: 0.5,
            },
            &mut math,
        );

        let diagnostic = estimator.diagnostics();
        assert_eq!(diagnostic.bemf_alpha_v, 0.0);
        assert_eq!(diagnostic.bemf_beta_v, 0.0);
    }

    /// 高速开环重捕获必须同时带入已知强拖电速度；否则 PLL 角度虽然正确，速度 PI
    /// 仍从零起步，实机可能落入约半速的次谐波吸引域。
    /// High-speed reacquisition seeds both angle and the known forced electrical
    /// speed, so the PLL PI does not restart from zero at a spinning rotor.
    #[test]
    fn smo_forced_speed_reacquisition_seeds_pll_feedforward() {
        let control = st_gbm2804_reference_parameters();
        let dt = 1.0 / control.pwm_frequency_hz as f32;
        let mut estimator = SmoPllEstimator::new(control.motor, dt);
        let current = AlphaBeta {
            alpha: 0.4,
            beta: -0.6,
        };
        let forced_electrical_speed_rad_s =
            582.0 * core::f32::consts::PI / 30.0 * control.motor.pole_pairs as f32;

        estimator.prepare_acquisition_at_speed(1.25, forced_electrical_speed_rad_s, current);

        assert_eq!(estimator.state.smo.current_est, current);
        assert!((estimator.state.pll.theta_rad - 1.25).abs() < 1.0e-6);
        assert!((estimator.state.pll.omega_rad_s - forced_electrical_speed_rad_s).abs() < 1.0e-6);
        assert!((estimator.state.pll.integrator - forced_electrical_speed_rad_s).abs() < 1.0e-6);
        assert!((estimator.state.omega_rad_s - forced_electrical_speed_rad_s).abs() < 1.0e-6);
        assert_eq!(estimator.state.smo.emf, AlphaBeta::default());
        assert_eq!(estimator.state.emf, AlphaBeta::default());
    }

    /// A retry with a valid BEMF vector must preserve the learned SMO state and
    /// align only the PLL; otherwise every retry can recreate the same bad basin.
    #[test]
    fn smo_bemf_resynchronization_preserves_model_and_realigns_pll() {
        let control = st_gbm2804_reference_parameters();
        let dt = 1.0 / control.pwm_frequency_hz as f32;
        let mut estimator = SmoPllEstimator::new(control.motor, dt);
        let forced_speed = 582.0 * core::f32::consts::PI / 30.0 * control.motor.pole_pairs as f32;
        let current = AlphaBeta {
            alpha: 0.31,
            beta: -0.22,
        };
        let emf = AlphaBeta {
            alpha: 0.50,
            beta: -0.25,
        };
        estimator.state.smo.current_est = current;
        estimator.state.smo.theta_emf_rad = 1.0;
        estimator.state.emf = emf;
        estimator.valid_samples = 64;
        estimator.reliable_samples = 9;

        estimator.prepare_acquisition_at_speed(0.0, forced_speed, AlphaBeta::default());
        let expected_angle = wrap_angle_0_to_2pi(
            1.0 + emf_filter_phase_advance_rad(
                forced_speed,
                estimator.params.smo.emf_filter_alpha,
                estimator.params.smo.ts,
            ),
        );
        assert_eq!(estimator.state.smo.current_est, current);
        assert_eq!(estimator.state.emf, emf);
        assert!((estimator.state.pll.theta_rad - expected_angle).abs() < 1.0e-6);
        assert!((estimator.state.pll.integrator - forced_speed).abs() < 1.0e-6);
        assert_eq!(estimator.valid_samples, 0);
        assert_eq!(estimator.reliable_samples, 0);
    }

    /// 强拖已知正向时，PLL 捕获阶段不得从低速噪声积出负转速；解除约束后仍保留
    /// 双向估速能力。
    /// A known forward rev-up blocks negative PLL speed during acquisition, while
    /// clearing the constraint restores bidirectional estimation.
    #[test]
    fn smo_acquisition_direction_is_temporary_and_bidirectional_after_release() {
        let control = st_gbm2804_reference_parameters();
        let dt = 1.0 / control.pwm_frequency_hz as f32;
        let mut estimator = SmoPllEstimator::new(control.motor, dt);
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: control.motor.nominal_bus_voltage_v,
            rotor: RotorFeedback::default(),
        };
        let current = AlphaBeta::default();
        let pwm = PwmCommand {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
        };
        let mut math = CpuMath;

        estimator.state.pll.integrator = -100.0;
        estimator.set_acquisition_direction(1);
        let constrained =
            estimator.update_from_alpha_beta_with_math(&feedback, current, pwm, &mut math);
        assert_eq!(constrained.mechanical_speed_rad_s, 0.0);

        estimator.state.pll.integrator = -100.0;
        estimator.set_acquisition_direction(0);
        let released =
            estimator.update_from_alpha_beta_with_math(&feedback, current, pwm, &mut math);
        assert!(released.mechanical_speed_rad_s < 0.0);
    }

    #[test]
    fn smo_diagnostics_exposes_each_reliability_gate() {
        let control = st_gbm2804_reference_parameters();
        let dt = 1.0 / control.pwm_frequency_hz as f32;
        let reliability = ObserverReliabilityConfig {
            minimum_speed_rpm: 500.0,
            minimum_bemf_v: 0.25,
            speed_variance_ratio: 0.01,
            maximum_phase_error_rad: 0.5,
            consecutive_samples: 2,
        };
        let mut estimator = SmoPllEstimator::new_with_tuning_and_reliability(
            control.motor,
            dt,
            SmoPllTuning::for_motor(control.motor),
            reliability,
        );
        let speed_rpm = 600.0;
        estimator.speed_fifo = [speed_rpm; 64];
        estimator.valid_samples = 64;
        estimator.speed_sum = speed_rpm * 64.0;
        estimator.speed_square_sum = speed_rpm * speed_rpm * 64.0;
        estimator.state.omega_rad_s =
            speed_rpm * core::f32::consts::PI * control.motor.pole_pairs as f32 / 30.0;
        estimator.state.emf = AlphaBeta {
            alpha: 0.5,
            beta: 0.0,
        };
        estimator.state.pll.phase_error = 0.125;
        estimator.raw_phase_error_rad = 0.125;
        estimator.reliability_decimator = estimator.reliability_decimator_limit - 1;
        estimator.update_reliability();

        let diagnostic = estimator.diagnostics();
        assert_eq!(diagnostic.reliability_flags, OBSERVER_GATE_ALL);
        assert_eq!(diagnostic.reliable_samples, 1);
        assert!((diagnostic.speed_mean_rpm - speed_rpm).abs() < 1.0e-3);
        assert!(diagnostic.speed_variance_rpm2 < 1.0e-3);
        assert!((diagnostic.bemf_alpha_v - 0.5).abs() < 1.0e-6);
        assert!((diagnostic.pll_phase_error_rad - 0.125).abs() < 1.0e-6);

        // 捕获资格使用 64 ms 平均绝对包角误差，而不是有界正弦鉴相值。后者在
        // 接近 pi 的反相点也会趋近 0，若拿它做门限会把稳定反相误判成已锁定。
        // 单个相位尖峰不应清空资格计数；持续失配才应关闭相位门。
        estimator.raw_phase_error_rad = 0.75;
        estimator.reliability_decimator = estimator.reliability_decimator_limit - 1;
        estimator.update_reliability();
        let diagnostic = estimator.diagnostics();
        assert_ne!(
            diagnostic.reliability_flags & OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM,
            0
        );
        assert_eq!(diagnostic.reliable_samples, 2);

        estimator.raw_phase_error_rad = 0.75;
        estimator.phase_abs_fifo = [0.75; 64];
        estimator.phase_abs_sum = 0.75 * 64.0;
        estimator.reliability_decimator = estimator.reliability_decimator_limit - 1;
        estimator.update_reliability();
        let diagnostic = estimator.diagnostics();
        assert_eq!(
            diagnostic.reliability_flags & OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM,
            0
        );
        assert_eq!(diagnostic.reliable_samples, 0);

        estimator.raw_phase_error_rad = 0.0;
        estimator.speed_square_sum = (speed_rpm * speed_rpm + 10_000.0) * 64.0;
        estimator.reliability_decimator = estimator.reliability_decimator_limit - 1;
        estimator.update_reliability();
        let diagnostic = estimator.diagnostics();
        assert_eq!(
            diagnostic.reliability_flags & OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM,
            0
        );
        assert_eq!(diagnostic.reliable_samples, 0);
    }

    #[test]
    fn smo_run_gate_uses_separate_minimum_speed() {
        let control = st_gbm2804_reference_parameters();
        let dt = 1.0 / control.pwm_frequency_hz as f32;
        let reliability = ObserverReliabilityConfig {
            minimum_speed_rpm: 524.0,
            minimum_bemf_v: 0.25,
            speed_variance_ratio: 0.01,
            maximum_phase_error_rad: 0.5,
            consecutive_samples: 2,
        };
        let mut estimator = SmoPllEstimator::new_with_tuning_and_reliability(
            control.motor,
            dt,
            SmoPllTuning::for_motor(control.motor),
            reliability,
        );
        let speed_rpm = 488.0;
        estimator.speed_fifo = [speed_rpm; 64];
        estimator.valid_samples = 64;
        estimator.speed_sum = speed_rpm * 64.0;
        estimator.speed_square_sum = speed_rpm * speed_rpm * 64.0;
        estimator.state.omega_rad_s =
            speed_rpm * core::f32::consts::PI * control.motor.pole_pairs as f32 / 30.0;
        estimator.state.emf = AlphaBeta {
            alpha: 0.5,
            beta: 0.0,
        };
        estimator.reliability_decimator = estimator.reliability_decimator_limit - 1;
        estimator.update_reliability();

        assert_eq!(
            estimator.diagnostics().reliability_flags,
            OBSERVER_GATE_ALL & !OBSERVER_GATE_SPEED_ABOVE_MINIMUM
        );
        assert!(!estimator.is_reliable());
        assert!(estimator.is_reliable_for_run(0.0, 0.5));
        assert!(estimator.is_reliable_for_run(450.0, 0.5));
        assert!(!estimator.is_reliable_for_run(500.0, 0.5));

        // The acquisition window may still contain an earlier transition spike.
        // Once the instantaneous run error is back below its own threshold, that
        // stale window must not extend the loss timer by another 64 ms.
        estimator.phase_abs_fifo = [0.75; 64];
        estimator.phase_abs_sum = 0.75 * 64.0;
        estimator.raw_phase_error_rad = 0.25;
        estimator.reliability_decimator = estimator.reliability_decimator_limit - 1;
        estimator.update_reliability();
        assert_eq!(
            estimator.diagnostics().reliability_flags & OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM,
            0
        );
        assert!(estimator.is_reliable_for_run(450.0, 0.5));

        // 运行保持门不应把捕获->运行 Kp 切换遗留在 64 ms 窗中的方差当成失锁；
        // 但原始相位误差超限必须拒绝，且不能在 sin(pi)=0 的反相点误判可靠。
        estimator.speed_square_sum = (speed_rpm * speed_rpm + 10_000.0) * 64.0;
        estimator.reliability_decimator = estimator.reliability_decimator_limit - 1;
        estimator.update_reliability();
        assert_eq!(
            estimator.diagnostics().reliability_flags & OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM,
            0
        );
        estimator.raw_phase_error_rad = 0.25;
        assert!(estimator.is_reliable_for_run(450.0, 0.5));
        estimator.raw_phase_error_rad = core::f32::consts::PI - 0.01;
        assert!(!estimator.is_reliable_for_run(450.0, 0.5));
    }
}
