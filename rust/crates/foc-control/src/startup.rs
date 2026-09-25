//! FluxRT —— 无感启动的强拖时序与切换到观测器的角度渐变。
//! FluxRT - the forced (open-loop) rev-up sequence and the blend to the observer.
//!
//! 反电势幅值正比于转速，因此观测器在零速与低速不可观。本模块先用"对齐 → 开环
//! 升速 → 保持 → 角度渐变 → 闭环"把电机推到可观速度，再把角度与转速控制权交给
//! 观测器；渐变阶段对角度做最短路径插值，避免切换瞬间的角度跳变。
//! The BEMF is proportional to speed, so the observer is unobservable at standstill.
//! This module forces the motor up to an observable speed and then hands angle and
//! speed to the observer, blending the angle over the shortest path.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   foc-rt-bridge -> `foc-control`（本模块）-> foc-algorithm
//!   本模块是纯状态机：不认识硬件，也不认识观测器的内部实现，只消费"观测器角度"
//!   与"观测器是否可信"两个输入。
//!   A pure state machine: it sees only the observer angle and the observer's
//!   reliability flag, never the observer internals.
//!
//! 实时约束 / Real-time constraints:
//!   `update()` 每拍在 12 kHz ADC ISR 内调用：无动态分配、无阻塞、无日志。
//!   只有算术与 `wrap_angle_*`，没有三角函数调用。
//!   `update()` runs every sample inside the 12 kHz ISR with no allocation, blocking
//!   or logging, and no trig calls.
//!
//! 量纲 / Units: 时间 `[s]`、转速 `[rpm]`（内部转 `[rad/s]`）、电角度 `[rad]`、
//!   电流 `[A]`。全部为 `f32`，不使用 Q 格式。
//!   Time `[s]`, speed `[rpm]` (converted internally to `[rad/s]`), electrical angle
//!   `[rad]`, current `[A]`. Everything is `f32`; no Q formats.
//!
//! 参考 / Reference: docs/ST与VESC工程改进路线图.md

use core::f32::consts::PI;

use foc_algorithm::{clamp, wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi};

use crate::CurrentCommand;

/// 强拖启动的五个阶段。
/// The five phases of the forced rev-up.
///
/// 顺序是固定的，状态机只向前推进；任何回退都必须通过 [`RevUpSequencer::reset`]
/// 重新开始，因为强制角度已不再对应转子位置。
/// The order is fixed and the machine only moves forward; going back requires
/// `reset()`, since the forced angle no longer matches the rotor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RevUpPhase {
    /// 对齐：把电流缓慢加到 `final_current_a`，让转子被拉到已知电角度 0。
    /// Alignment: ramps current up so the rotor is pulled to electrical angle 0.
    Alignment,
    /// 开环升速：转速线性升到 `final_speed_rpm`，电流保持 `final_current_a`。
    /// Open-loop ramp: speed rises linearly to `final_speed_rpm`.
    OpenLoopRamp,
    /// 开环保持：转速保持在 `final_speed_rpm`，等待观测器通过可信门控。
    /// Open-loop hold: speed held while waiting for the observer reliability gate.
    OpenLoopHold,
    /// 角度渐变：在 `transition_s` 内把控制角度从强制角度插值到观测器角度，
    /// 同时把 q 轴电流从开环值过渡到观测器参考值。
    /// Observer transition: blends angle and q-axis current to the observer.
    ObserverTransition,
    /// 闭环：角度与转速完全由观测器提供，速度外环接管电流给定。
    /// Closed loop: angle and speed come from the observer; the speed loop owns iq.
    ClosedLoop,
}

/// 强拖时序一拍输出。
/// One sample of rev-up sequencer output.
///
/// 调用方用 `phase` 与 `use_observer_speed` 决定后续行为，用
/// `angle_for_control_rad` 做 Park 变换，用 `current_reference` 做电流给定。
/// The caller uses `phase` and `use_observer_speed` to branch, `angle_for_control_rad`
/// for Park and `current_reference` for the current loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RevUpOutput {
    /// 本拍所处阶段。
    /// The phase this sample is in.
    pub phase: RevUpPhase,
    /// 本拍 d/q 电流给定 `[A]`；强拖期间 `id_ref_a` 恒为 0。
    /// This sample's d/q current reference in `[A]`; `id_ref_a` is always 0 here.
    pub current_reference: CurrentCommand,
    /// 内部强拖角度 `[rad]`，归一化在 `[0, 2*pi)`；用于遥测观察"强制角度"轨迹。
    /// The internal forced electrical angle `[rad]`; for telemetry.
    pub forced_electrical_angle_rad: f32,
    /// 本拍强拖轨迹对应的机械转速 `[rpm]`。开环角度补偿用它换算电角速度；
    /// 闭环控制仍使用观测器速度，不把这个值当成实测量。
    /// Mechanical speed of the forced trajectory `[rpm]`; used only to derive
    /// open-loop electrical-speed compensation, never as measured feedback.
    pub forced_speed_rpm: f32,
    /// 实际用于 Park 变换的电角度 `[rad]`：强拖阶段等于强制角度，渐变阶段是两者
    /// 的最短路径插值，闭环阶段等于观测器角度。
    /// The electrical angle actually used for Park `[rad]`.
    pub angle_for_control_rad: f32,
    /// 是否应当改用观测器给出的转速（仅闭环阶段为真）。
    /// Whether the observer speed should be used (true only in closed loop).
    pub use_observer_speed: bool,
}

/// 强拖时序的时间与终点参数。
/// Timing and end-point parameters of the rev-up sequence.
///
/// 全部时间单位 `[s]`，转速 `[rpm]`（机械），电流 `[A]`。
/// Times are `[s]`, speed is mechanical `[rpm]` and current is `[A]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RevUpConfig {
    /// 对齐时长 `[s]`；太短会让转子未到位就开始升速，表现为启动时来回抖动。
    /// Alignment duration `[s]`; too short starts the ramp before the rotor settles.
    pub alignment_s: f32,
    /// 开环升速时长 `[s]`；升速斜率过高会让转子跟不上强制角度（失步）。
    /// Open-loop ramp duration `[s]`; a steeper ramp makes the rotor lose step.
    pub ramp_s: f32,
    /// 角度渐变时长 `[s]`；为 0 时直接跳到观测器角度（仅用于测试）。
    /// Angle blend duration `[s]`; 0 jumps straight to the observer angle.
    pub transition_s: f32,
    /// 开环终点转速 `[rpm]`；必须高到观测器可信门控能被满足。
    /// Open-loop end speed `[rpm]`; high enough for the reliability gate.
    pub final_speed_rpm: f32,
    /// 对齐结束时的 q 轴电流 `[A]`；可高于升速终点电流，用于克服启动死区。
    /// Q-axis current at the end of alignment `[A]`; it may exceed the ramp-end
    /// current so breakaway torque does not force a voltage-limited final hold.
    pub alignment_current_a: f32,
    /// 升速终点与开环保持段的 q 轴电流 `[A]`。
    /// Q-axis current at the ramp endpoint and during open-loop hold `[A]`.
    pub final_current_a: f32,
}

impl RevUpConfig {
    /// 参数自检：全部有限、时间为正、转速与电流为正。
    /// Validates that everything is finite and the times, speed and current are
    /// positive; `transition_s` may be zero.
    ///
    /// `foc-rt-bridge` 在 `foc_rust_configure()` 里调用它，非法配置会被拒绝而不是
    /// 半途生效。`transition_s` 允许为 0（瞬时切换）是刻意保留的测试能力。
    /// `foc-rt-bridge` calls this from `foc_rust_configure()` so a bad config is
    /// rejected instead of partially applied.
    pub fn is_valid(&self) -> bool {
        self.alignment_s.is_finite()
            && self.ramp_s.is_finite()
            && self.transition_s.is_finite()
            && self.final_speed_rpm.is_finite()
            && self.alignment_current_a.is_finite()
            && self.final_current_a.is_finite()
            && self.alignment_s > 0.0
            && self.ramp_s > 0.0
            && self.transition_s >= 0.0
            && self.final_speed_rpm > 0.0
            && self.alignment_current_a > 0.0
            && self.final_current_a > 0.0
    }
}

impl Default for RevUpConfig {
    /// MCSDK 参考工程的强拖时序（`[ST]`）：对齐 1.0 s、开环升速 1.164 s 到
    /// 582 rpm、再用 25 ms 把角度渐变到观测器。
    /// MCSDK-reference rev-up timing (`[ST]`): 1.0 s alignment, 1.164 s open-loop
    /// ramp to 582 rpm, then a 25 ms angle transition.
    ///
    /// 这些是参考工程的整定结果，不是本台架的实测最优值；换电机或换负载后必须
    /// 重新确认升速斜率与终点转速仍能进入观测器可信区间。
    /// These are reference-project values, not rig-measured optima; they must be
    /// re-checked after a motor or load change.
    fn default() -> Self {
        Self {
            alignment_s: 1.0,
            ramp_s: 1.164,
            transition_s: 0.025,
            final_speed_rpm: 582.0,
            alignment_current_a: 0.8,
            final_current_a: 0.8,
        }
    }
}

/// MCSDK-reference rev-up timing: 1000 ms alignment, 1164 ms open-loop ramp
/// to 582 rpm, then 25 ms angle transition to the observer.
///
/// 强拖时序状态机：推进强制角度、生成电流给定，并在观测器可信后开始切换。
/// The rev-up state machine: advances the forced angle, produces the current
/// reference and starts the switch-over once the observer is trustworthy.
///
/// 它是纯状态机，不持有硬件，也不读时间：`dt_s` 由调用方每拍给出（实机为
/// `1/12000` `[s]`），因此时序精度取决于调用周期的一致性。
/// It holds no hardware and reads no clock: the caller supplies `dt_s` (12 kHz on
/// target), so timing accuracy depends on a consistent call period.
#[derive(Clone, Copy, Debug)]
pub struct RevUpSequencer {
    /// 极对数；用于把机械转速 `[rad/s]` 换成电角度增量。
    /// Pole pairs, converting mechanical `[rad/s]` into an electrical increment.
    pole_pairs: u8,
    /// 时间与终点参数。
    /// Timing and end-point parameters.
    config: RevUpConfig,
    /// 本次启动方向：`1` 正转，`-1` 反转。配置里的转速和电流保持为正的幅值，
    /// 避免同一套 Rev-Up 参数为两个方向复制两份。
    /// Direction of this start: `1` forward, `-1` reverse. Configured speed and
    /// current remain positive magnitudes shared by both directions.
    direction: i8,
    /// 自启动以来的累计时间 `[s]`（含渐变与闭环阶段）。
    /// Accumulated time `[s]` since start.
    elapsed_s: f32,
    /// 渐变阶段已经过的时间 `[s]`。
    /// Time elapsed inside the transition `[s]`.
    transition_elapsed_s: f32,
    /// 是否已经进入渐变阶段（一旦为真不再回退）。
    /// Whether the transition has started (latching).
    transition_started: bool,
    /// 是否已经完成渐变进入闭环。
    /// Whether the blend completed and closed loop was entered.
    closed_loop: bool,
    /// 开始渐变那一刻的开环转速 `[rpm]`；渐变与闭环期间强制角度按它继续积分。
    /// Open-loop speed at handoff `[rpm]`; the forced angle keeps integrating at it.
    handoff_speed_rpm: f32,
    /// 渐变起点 q 轴电流 `[A]`（即开环电流），保证切换无阶跃。
    /// Transition start q-axis current `[A]`, i.e. the open-loop current.
    handoff_start_iq_a: f32,
    /// 渐变终点 q 轴电流 `[A]`，来自观测器参考并被限幅到 `final_current_a`。
    /// Transition end q-axis current `[A]`, clamped to `final_current_a`.
    handoff_end_iq_a: f32,
    /// 内部强制电角度 `[rad]`，按开环转速积分。
    /// Internal forced electrical angle `[rad]`.
    forced_electrical_angle_rad: f32,
}

impl RevUpSequencer {
    /// 用给定极对数和默认时序构建时序器。
    /// Builds the sequencer with the default timing for the given pole pairs.
    pub fn new(pole_pairs: u8) -> Self {
        Self::with_config(pole_pairs, RevUpConfig::default())
    }

    /// 用显式配置构建时序器；所有内部状态从零开始。
    /// Builds the sequencer with an explicit configuration and zeroed state.
    ///
    /// 调用方必须先自行确认 `config.is_valid()`：本函数不做校验，也不回退到默认
    /// 值（`foc-rt-bridge` 在配置入口处校验并拒绝非法配置）。
    /// The caller must validate the config first; this function neither checks nor
    /// falls back to defaults, the bridge validates at its config entry point.
    pub fn with_config(pole_pairs: u8, config: RevUpConfig) -> Self {
        Self {
            pole_pairs,
            config,
            direction: 1,
            elapsed_s: 0.0,
            transition_elapsed_s: 0.0,
            transition_started: false,
            closed_loop: false,
            handoff_speed_rpm: 0.0,
            handoff_start_iq_a: 0.0,
            handoff_end_iq_a: 0.0,
            forced_electrical_angle_rad: 0.0,
        }
    }

    /// 复位到时序起点：强制角度归零、阶段回到 [`RevUpPhase::Alignment`]。
    /// Resets to the start of the sequence with the forced angle at zero.
    ///
    /// 会在每次重新启动和故障恢复时被调用。注意强制角度归零意味着"转子被重新
    /// 对齐到电角度 0"，若此时电机仍在旋转，这一步会产生制动转矩。
    /// Resetting the forced angle means the rotor is re-aligned to electrical zero;
    /// if the motor is still spinning this produces braking torque.
    pub fn reset(&mut self) {
        self.reset_with_direction(1);
    }

    /// 复位并选择本次启动方向。零按正转处理；公开启动入口已经拒绝零目标，
    /// 这个回退只防止独立复用状态机时留下无方向的强拖轨迹。
    /// Resets and selects the start direction. Zero falls back to forward; the
    /// public bridge rejects a zero target, while this keeps standalone use safe.
    pub fn reset_with_direction(&mut self, direction: i8) {
        self.direction = if direction < 0 { -1 } else { 1 };
        self.elapsed_s = 0.0;
        self.transition_elapsed_s = 0.0;
        self.transition_started = false;
        self.closed_loop = false;
        self.handoff_speed_rpm = 0.0;
        self.handoff_start_iq_a = 0.0;
        self.handoff_end_iq_a = 0.0;
        self.forced_electrical_angle_rad = 0.0;
    }

    /// 推进一步开环强拖；只有在观测器通过收敛门控后才开始无感切换。
    /// Advances the open-loop rev-up and starts switch-over only after the
    /// observer has passed its convergence gate. `observer_iq_a` is captured
    /// in the observer reference frame and becomes the end point of the MCSDK-
    /// style torque-current ramp.
    ///
    /// 参数 / Parameters:
    ///   dt_s               距上一拍的时间 `[s]`；负值会被夹到 0
    ///   observer_angle_rad 观测器电角度 `[rad]`，仅用于渐变插值
    ///   handoff_ready      观测器是否可信（由可靠性门控给出）
    ///   observer_iq_a      观测器参考系下的 q 轴电流 `[A]`，作为渐变终点
    ///   current_slew_a_per_s 接管电流允许的下降斜率 `[A/s]`；角度融合时长保持独立
    ///
    /// 返回 / Returns: 本拍阶段、电流给定 `[A]` 与控制用电角度 `[rad]`。
    ///
    /// 时间参数为负会把时序推回过去，因此先夹到 0；`transition_just_started`
    /// 则保证开始渐变的那一拍不会把 `dt_s` 计两次。
    /// A negative `dt_s` would rewind the sequence, so it is clamped to zero, and
    /// `transition_just_started` keeps the start tick from counting `dt_s` twice.
    pub fn update(
        &mut self,
        dt_s: f32,
        observer_angle_rad: f32,
        handoff_ready: bool,
        observer_iq_a: f32,
        current_slew_a_per_s: f32,
    ) -> RevUpOutput {
        let dt_s = dt_s.max(0.0);
        self.elapsed_s += dt_s;
        let alignment_complete = self.elapsed_s >= self.config.alignment_s;
        // 升速进度：对齐结束后在 ramp_s 内从 0 线性升到 1，并夹到 [0,1]。
        // Ramp progress: linear 0 to 1 over ramp_s after alignment, clamped.
        let ramp_progress = clamp(
            (self.elapsed_s - self.config.alignment_s) / self.config.ramp_s,
            0.0,
            1.0,
        );
        let direction = self.direction as f32;
        let open_loop_speed_rpm = self.config.final_speed_rpm * ramp_progress * direction;
        let open_loop_phase = if !alignment_complete {
            RevUpPhase::Alignment
        } else if ramp_progress < 1.0 {
            RevUpPhase::OpenLoopRamp
        } else {
            RevUpPhase::OpenLoopHold
        };
        // 对齐电流先从 0 线性上升；开环升速时再从对齐电流平滑过渡到终点
        // 电流。这样可用较大静止转矩克服某些初始转子位置，又避免高电阻电机
        // 在终速因 `I*R + BEMF` 顶到母线电压限幅。
        // Ramp alignment current from zero, then taper it linearly to the hold
        // current. This separates breakaway torque from the endpoint voltage budget.
        let open_loop_current_magnitude_a = if !alignment_complete {
            self.config.alignment_current_a * self.elapsed_s / self.config.alignment_s
        } else {
            self.config.alignment_current_a
                + ramp_progress * (self.config.final_current_a - self.config.alignment_current_a)
        };
        // 反转不仅反向积分强制角，也反向给出 Iq。这样 Rev-Up、接管支撑和闭环
        // 速度 PI 在同一个转矩符号约定下工作，不会在交接边界先制动再反向加速。
        // Reverse start flips both forced-angle velocity and Iq, keeping one torque
        // sign convention through rev-up, handover and the speed loop.
        let open_loop_current_a = open_loop_current_magnitude_a * direction;

        let mut transition_just_started = false;
        // 切换只在"已对齐 + 观测器可信 + 尚未开始"时触发一次。通过完整获取门后，
        // 渐变必须单向完成：SMO 原始相位在滑模谐波下可能短时抖出运行门，若每拍
        // 反转进度，会在强制角与观测角之间来回摆动，电流环永远无法完成无扰接管。
        // 持续失锁由上层 `observer_loss_timeout_s` 计时并故障停机，本时序层不再用单拍
        // `handoff_ready` 回滚已启动的坐标系渐变。
        // Once the full acquisition gate has started the blend, progress is monotonic.
        // Brief raw-phase dropouts must not rock the current controller between two
        // reference frames; sustained loss is handled by the caller's timed fault.
        if !self.transition_started && !self.closed_loop && alignment_complete && handoff_ready {
            self.transition_started = true;
            transition_just_started = true;
            self.transition_elapsed_s = 0.0;
            self.handoff_speed_rpm = open_loop_speed_rpm;
            self.handoff_start_iq_a = open_loop_current_a;
            // 渐变终点的电流被限幅到强拖电流：观测器此刻给出的 iq 可能因为角度尚
            // 未收敛而偏大，直接采用会让切换瞬间过流。
            // The end current is clamped to the forced current: an unconverged
            // observer angle can report an over-large iq.
            self.handoff_end_iq_a = clamp(
                observer_iq_a,
                -self.config.final_current_a,
                self.config.final_current_a,
            );
        }

        let (phase, current_a, forced_speed_rpm, transition) = if self.closed_loop {
            (
                RevUpPhase::ClosedLoop,
                self.handoff_end_iq_a,
                self.handoff_speed_rpm,
                1.0,
            )
        } else if self.transition_started {
            if !transition_just_started {
                self.transition_elapsed_s += dt_s;
            }
            // transition_s 为 0 时直接视为完成，避免除以 0。
            // A zero transition_s completes immediately instead of dividing by zero.
            let progress = if self.config.transition_s > 0.0 {
                clamp(
                    self.transition_elapsed_s / self.config.transition_s,
                    0.0,
                    1.0,
                )
            } else {
                1.0
            };
            if progress >= 1.0 {
                self.closed_loop = true;
                (
                    RevUpPhase::ClosedLoop,
                    self.handoff_end_iq_a,
                    self.handoff_speed_rpm,
                    1.0,
                )
            } else {
                // 角度需要较慢融合来保证观测器不掉出吸引域，但 q 轴电流不应被迫在
                // 同样长的时间里维持额外加速转矩。电流按显式 A/s 限速允许提前到达
                // 终点；若配置斜率较慢，则退回角度融合时长，确保闭环边界没有跳变。
                // Angle ownership may need a slow blend while torque current can
                // reach its endpoint earlier. A slow configured slew falls back to
                // the angle duration so the closed-loop boundary stays bumpless.
                let current_delta_a = (self.handoff_end_iq_a - self.handoff_start_iq_a).abs();
                let slew_duration_s =
                    if current_slew_a_per_s.is_finite() && current_slew_a_per_s > 0.0 {
                        current_delta_a / current_slew_a_per_s
                    } else {
                        self.config.transition_s
                    };
                let current_transition_s = slew_duration_s.min(self.config.transition_s);
                let current_progress = if current_transition_s > 0.0 {
                    clamp(self.transition_elapsed_s / current_transition_s, 0.0, 1.0)
                } else {
                    1.0
                };
                let current_a = self.handoff_start_iq_a
                    + current_progress * (self.handoff_end_iq_a - self.handoff_start_iq_a);
                (
                    RevUpPhase::ObserverTransition,
                    current_a,
                    self.handoff_speed_rpm,
                    progress,
                )
            }
        } else {
            (
                open_loop_phase,
                open_loop_current_a,
                open_loop_speed_rpm,
                0.0,
            )
        };

        // 强制角度按开环转速积分；转速 [rpm] → [rad/s]（乘 pi/30），再乘极对数得到
        // 电角速度。渐变与闭环期间仍按 handoff 转速继续积分而不是冻结，这样遥测里的
        // 强制角度始终是一条连续的参考轨迹，可以直接与观测器角度对比。注意闭环阶段
        // 插值权重恒为 1，`angle_for_control_rad` 此时就等于观测器角度，强制角度不再
        // 参与控制。
        // The forced angle integrates at the open-loop speed and keeps integrating
        // through and after the handoff so telemetry shows a continuous reference
        // trajectory. In ClosedLoop the blend weight is 1, so the control angle is the
        // observer angle and the forced angle no longer affects control.
        let mechanical_speed_rad_s = forced_speed_rpm * PI / 30.0;
        self.forced_electrical_angle_rad = wrap_angle_0_to_2pi(
            self.forced_electrical_angle_rad
                + mechanical_speed_rad_s * self.pole_pairs as f32 * dt_s,
        );
        // 角度误差取最短路径（wrap 到 [-pi, pi)），否则 2*pi 附近会走"长边"，
        // 渐变期间表现为角度反向绕一圈。
        // The error uses the shortest path, otherwise the blend takes the long way
        // round near the 2*pi wrap.
        let angle_error =
            wrap_angle_minus_pi_to_pi(observer_angle_rad - self.forced_electrical_angle_rad);
        // transition = 0 时用强制角度，= 1 时正好等于观测器角度，中间线性插值。
        // transition 0 uses the forced angle and 1 lands exactly on the observer angle.
        let angle_for_control_rad =
            wrap_angle_0_to_2pi(self.forced_electrical_angle_rad + transition * angle_error);
        RevUpOutput {
            phase,
            current_reference: CurrentCommand {
                id_ref_a: 0.0,
                iq_ref_a: current_a,
            },
            forced_electrical_angle_rad: self.forced_electrical_angle_rad,
            forced_speed_rpm,
            angle_for_control_rad,
            use_observer_speed: phase == RevUpPhase::ClosedLoop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 按 MCSDK 参考时序逐拍走完五个阶段，锁定时间点与阶段判定：对齐结束才升速、
    /// 升速结束才保持、观测器可信才进入渐变、渐变满 25 ms 才闭环，且闭环电流等于
    /// 观测器参考值。时间边界一旦漂移，实机上表现为启动时间或切换点变化。
    /// Walks all five phases at the MCSDK reference timing and pins the boundaries
    /// and the final closed-loop current.
    #[test]
    fn follows_generated_rev_up_timing() {
        let mut startup = RevUpSequencer::new(7);
        let mut output = startup.update(0.999, 0.0, false, 0.0, 32.0);
        assert_eq!(output.phase, RevUpPhase::Alignment);
        assert!(output.current_reference.iq_ref_a < 0.8);
        output = startup.update(0.002, 0.0, false, 0.0, 32.0);
        assert_eq!(output.phase, RevUpPhase::OpenLoopRamp);
        output = startup.update(1.163, 0.0, false, 0.0, 32.0);
        assert_eq!(output.phase, RevUpPhase::OpenLoopHold);
        output = startup.update(0.001, 0.25, true, 0.35, 32.0);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        output = startup.update(0.012, 0.25, false, 0.0, 32.0);
        assert!(output.current_reference.iq_ref_a < 0.8);
        output = startup.update(0.014, 0.25, true, 0.0, 32.0);
        assert_eq!(output.phase, RevUpPhase::ClosedLoop);
        assert!(output.use_observer_speed);
        assert!((output.current_reference.iq_ref_a - 0.35).abs() < 1e-6);
    }

    /// 观测器提前可信时，渐变必须能早于升速终点开始（此处 `elapsed = 2.05 s`，
    /// 已完成对齐但升速未满），并且此刻仍用强拖满电流而不是观测器电流。
    /// 这保证"观测器先收敛"不会导致强制角度停在半路。
    /// A trustworthy observer may start the transition before the ramp endpoint, and
    /// the forced current is still used; the forced angle must not stall mid-ramp.
    #[test]
    fn observer_can_start_transition_before_ramp_endpoint() {
        let mut startup = RevUpSequencer::new(7);
        let output = startup.update(2.05, 1.0, true, 0.5, 32.0);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        assert!(output.forced_electrical_angle_rad.is_finite());
        assert!((output.current_reference.iq_ref_a - 0.8).abs() < 1e-6);
    }

    /// 通过完整获取门后，渐变进度必须单向前进；短时可靠性掉线由上层失锁
    /// 超时处理，不能让本层在两个坐标系之间反复摇摆。
    #[test]
    fn observer_transition_is_monotonic_after_acquisition() {
        let mut startup = RevUpSequencer::new(7);
        let mut output = startup.update(2.2, 0.4, true, 0.3, 32.0);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        let initial_iq = output.current_reference.iq_ref_a;

        output = startup.update(0.020, 0.5, true, 0.0, 32.0);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        assert!(output.current_reference.iq_ref_a < initial_iq);

        output = startup.update(0.002, 0.6, false, 0.0, 32.0);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        assert!(output.current_reference.iq_ref_a < initial_iq);
        output = startup.update(0.004, 0.7, false, 0.0, 32.0);
        assert_eq!(output.phase, RevUpPhase::ClosedLoop);
        assert!((output.current_reference.iq_ref_a - 0.3).abs() < 1e-6);
    }

    /// 电流斜率可以让转矩在角度渐变结束前先到达支撑终点，但角度状态仍保持在
    /// ObserverTransition；这样减少启动超调而不缩短无感角接管窗口。
    #[test]
    fn current_slew_is_independent_from_angle_transition() {
        let mut startup = RevUpSequencer::new(7);
        let mut output = startup.update(2.2, 0.4, true, 0.28, 52.0);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        assert!((output.current_reference.iq_ref_a - 0.8).abs() < 1e-6);

        output = startup.update(0.011, 0.5, false, 0.0, 52.0);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        assert!((output.current_reference.iq_ref_a - 0.28).abs() < 1e-4);

        output = startup.update(0.014, 0.6, false, 0.0, 52.0);
        assert_eq!(output.phase, RevUpPhase::ClosedLoop);
    }

    /// 对齐电流与终速保持电流可独立配置：对齐段从 0 升到 0.8 A，
    /// 随后在升速段线性降到 0.6 A，过程不能在阶段边界产生电流跳变。
    /// Alignment and hold currents are independent and joined by a continuous ramp.
    #[test]
    fn alignment_current_tapers_to_hold_current() {
        let config = RevUpConfig {
            alignment_s: 1.0,
            ramp_s: 1.0,
            transition_s: 0.05,
            final_speed_rpm: 582.0,
            alignment_current_a: 0.8,
            final_current_a: 0.6,
        };
        let mut startup = RevUpSequencer::with_config(7, config);

        let alignment = startup.update(0.5, 0.0, false, 0.0, 4.0);
        assert_eq!(alignment.phase, RevUpPhase::Alignment);
        assert!((alignment.current_reference.iq_ref_a - 0.4).abs() < 1.0e-6);

        let middle = startup.update(1.0, 0.0, false, 0.0, 4.0);
        assert_eq!(middle.phase, RevUpPhase::OpenLoopRamp);
        assert!((middle.current_reference.iq_ref_a - 0.7).abs() < 1.0e-6);

        let hold = startup.update(0.5, 0.0, false, 0.0, 4.0);
        assert_eq!(hold.phase, RevUpPhase::OpenLoopHold);
        assert!((hold.current_reference.iq_ref_a - 0.6).abs() < 1.0e-6);
    }

    /// 正反转共用同一套幅值参数，但强制速度、Iq 和电角轨迹必须严格镜像；
    /// 否则负目标会在 Rev-Up 或接管处出现转矩符号翻转。
    #[test]
    fn reverse_rev_up_mirrors_speed_current_and_angle() {
        let mut forward = RevUpSequencer::new(7);
        let mut reverse = RevUpSequencer::new(7);
        reverse.reset_with_direction(-1);

        let forward_output = forward.update(1.5, 0.0, false, 0.0, 32.0);
        let reverse_output = reverse.update(1.5, 0.0, false, 0.0, 32.0);

        assert_eq!(forward_output.phase, RevUpPhase::OpenLoopRamp);
        assert_eq!(reverse_output.phase, forward_output.phase);
        assert!((reverse_output.forced_speed_rpm + forward_output.forced_speed_rpm).abs() < 1.0e-5);
        assert!(
            (reverse_output.current_reference.iq_ref_a + forward_output.current_reference.iq_ref_a)
                .abs()
                < 1.0e-6
        );
        assert!(
            wrap_angle_minus_pi_to_pi(
                reverse_output.forced_electrical_angle_rad
                    + forward_output.forced_electrical_angle_rad
            )
            .abs()
                < 1.0e-5
        );
    }
}
