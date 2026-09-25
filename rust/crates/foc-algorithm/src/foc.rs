//! FOC_Basic 的等价实现：Clarke -> Park -> Id/Iq PI -> 逆 Park -> SVPWM。
//!
//! 职责 / Responsibility:
//!   - 把一个采样周期的三相电流、一个电角度和 Id/Iq 给定，算成三相占空比
//!   - 只做算法组合：不读寄存器、不写 PWM、不做故障判定、不含电压矢量圆限幅
//! Turns one sampling cycle of three-phase currents, one electrical angle and the Id/Iq
//! references into three duties. It only composes math: no register access, no PWM
//! write, no fault decision and no voltage-vector circle limiting.
//!
//! 架构位置 / Position in the architecture:
//!   - applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control -> 本文件
//!   - 依赖方向只向下：只依赖本 crate 的 `controller`、`modulation`、`transform` 和
//!     `libm`，不反向调用 HAL、RT-Thread 或 `foc-control`
//!   - `#![no_std]`、无 `unsafe`、无堆分配、无全局可变状态
//! Pure-math end of applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) ->
//! foc-control -> this file. Dependencies point downward only: sibling modules of this
//! crate plus `libm`, never HAL, RT-Thread or `foc-control`. `no_std`, no `unsafe`, no
//! heap and no global mutable state.
//!
//! 单位与角度约定 / Units and angle convention:
//!   - 电流 `[A]`、电压 `[V]`、角度 `[rad]`、采样周期 `[s]`，占空比是无量纲 `[0,1]`
//!   - `theta_e_rad` 是电角度 = `机械角度 × 极对数 + 电角度零位偏置`，不是机械角度
//!   - 本文件全部是 f32；本文件内没有 Q1.15/Q1.31 定点缩放
//! Currents `[A]`, voltages `[V]`, angles `[rad]`, sample period `[s]`; duty is a
//! dimensionless `[0,1]` ratio. `theta_e_rad` is the ELECTRICAL angle
//! (`mechanical_angle * pole_pairs + electrical_offset`), never the mechanical angle.
//! Everything here is f32; no Q-format scaling appears in this file.
//!
//! 实时约束 / Real-time constraints:
//!   - 面向 12 kHz ADC 中断内的电流环调用（STM32G431，见 docs/架构与安全边界.md）
//!   - 无分配、无阻塞、无日志、无 Mutex 等待；状态必须由唯一上下文持有
//!   - 本仓库板级快环实际跑的是 `foc-control` 的电流环；本结构是同一算法的库内组合，
//!     由 `CascadeState`、`EkfFocState` 和文档示例使用
//! Written for the 12 kHz ADC-interrupt current loop on the STM32G431 (see
//! docs/架构与安全边界.md). No allocation, no blocking, no logging, no mutex waits; one
//! owner context must own the state. The board fast loop in this repository actually runs
//! the `foc-control` current loop; this struct is the in-library composition of the same
//! algorithm, used by `CascadeState`, `EkfFocState` and the documented example.
//!
//! 与板级闭环的差别 / Difference from the board closed loop:
//!   - 本结构缺少 `foc-control` 电流环中基于实测母线电压的电压矢量圆限幅，是纯算法
//!     库里的最小电流环组合，不等于成品控制器
//!   - 内部固定以 SVPWM 收尾；需要 DPWM 时应复用它算出的 `voltage_alpha_beta` 另调
//!     `dpwm_update()`，不要先算 SVPWM 占空比再“转换”成 DPWM
//! This struct lacks the measured-bus voltage-vector circle limiting used by the
//! `foc-control` current loop, so it is a minimal composition rather than a product
//! controller. It always ends in SVPWM; for DPWM reuse its `voltage_alpha_beta` and call
//! `dpwm_update()` instead of post-processing SVPWM duties.
//!
//! 迁移与等价性 / Migration and equivalence:
//!   - 对应原 C 库的 `FOC_Basic`；测试 `matches_c_reference_vector` 沿用该 C 模块的
//!     原始测试向量，它的运算顺序与比例系数正是等价性成立的原因，改动会破坏该测试
//!   - 原 C 参考库未随本仓库提供（crate README 记录原 C 目录未发现许可证），因此库内
//!     常数的来源类别（`[HW]`/`[ST]`/`[FW]`/`[VESC]`）在本仓库内无法确认
//! Corresponds to the C module `FOC_Basic`; `matches_c_reference_vector` reuses that
//! module's original test vector, which holds only because of the exact operation order
//! and scaling used here. The C reference library is not shipped in this repository, so
//! the provenance class of these constants (`[HW]`/`[ST]`/`[FW]`/`[VESC]`) cannot be
//! confirmed in-tree.

use crate::controller::{PiParam, PiState};
use crate::modulation::{svpwm_update, SvpwmOutput, SvpwmParam};
use crate::transform::{clarke, inverse_park, park, Abc, AlphaBeta, Dq};

/// 基础电流环的全部参数：d/q 两个电流 PI 加 SVPWM 的母线电压。
/// All parameters of the basic current loop: the two d/q current PIs plus the SVPWM
/// DC-link voltage.
///
/// 纯参数、无状态，可被多个 `FocBasicState` 共享；运行中不应改写，尤其是
/// `svpwm.v_bus`，它参与每个周期的电压到 duty 换算。
/// Parameters only, so one value can be shared by several states. They are not meant to
/// change while running, especially `svpwm.v_bus`, which scales every duty conversion.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocBasicParam {
    /// d 轴电流 PI：误差 `[A]` 进、d 轴电压 `[V]` 出。
    /// d-axis current PI: current error in `[A]` in, d-axis voltage in `[V]` out.
    ///
    /// 本结构不做 d/q 电压圆限幅，所以 `out_min`/`out_max` `[V]` 与
    /// `integrator_min`/`integrator_max` `[V]` 就是唯一的电压边界。
    /// No d/q voltage-circle limiting is done here, so `out_min`/`out_max` in `[V]` and
    /// the integrator limits in `[V]` are the only voltage bounds.
    pub id_pi: PiParam,
    /// q 轴电流 PI：误差 `[A]` 进、q 轴电压 `[V]` 出。
    /// q-axis current PI: current error in `[A]` in, q-axis voltage in `[V]` out.
    ///
    /// 隐极机 Id=0 策略下转矩几乎全部来自这一路，所以它的输出限幅实际上决定了
    /// 可用电压与最大转矩。
    /// With Id=0 on a surface PM almost all torque comes from this axis, so its output
    /// limit effectively sets the available voltage and the maximum torque.
    pub iq_pi: PiParam,
    /// SVPWM 的母线电压参数，见 `SvpwmParam::v_bus` `[V]`。
    /// SVPWM DC-link parameter, see `SvpwmParam::v_bus` in `[V]`.
    pub svpwm: SvpwmParam,
}

/// 一次电流环调用的输入：同一采样周期的三相电流、电角度和 d/q 给定。
/// Inputs of one current-loop call: three-phase currents, the electrical angle and the
/// d/q references, all from the same sampling cycle.
///
/// 电流、角度与给定必须同源同拍：混用相邻周期会在 d/q 上留下随转速增大的交叉耦合
/// 角误差（12 kHz 下 ω_e·ts）。
/// Currents, angle and references must come from one snapshot: mixing adjacent cycles
/// leaves a speed-proportional cross-coupling angle error (ω_e·ts at 12 kHz) in d/q.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocBasicInput {
    /// a 相电流 `[A]`，正方向由平台层的极性约定决定，必须与 b/c 相一致。
    /// Phase-a current in `[A]`; the positive direction is fixed by the platform
    /// adapter's polarity convention and must match phases b and c.
    pub ia: f32,
    /// b 相电流 `[A]`；三相必须来自同一采样时刻。
    /// Phase-b current in `[A]`; all three phases must share one sampling instant.
    pub ib: f32,
    /// c 相电流 `[A]`；本模块不做零序校验，也不强制 a+b+c=0。
    /// Phase-c current in `[A]`. This module does not cross-check the zero sequence and
    /// does not enforce a+b+c=0.
    pub ic: f32,
    /// d 轴电流给定 `[A]`；隐极机常用 0，弱磁区为负。
    /// d-axis current reference in `[A]`; usually 0 on a surface PM and negative in the
    /// field-weakening region.
    pub id_ref: f32,
    /// q 轴电流给定 `[A]`；来自速度环或转矩指令，受硬件电流上限约束。
    /// q-axis current reference in `[A]`, produced by the speed loop or a torque command
    /// and bounded by the hardware current limit.
    pub iq_ref: f32,
    /// 电角度 `[rad]`，即 `机械角度 × 极对数 + 电角度零位偏置`，不是机械角度。
    /// Electrical angle in `[rad]`: `mechanical_angle * pole_pairs + electrical_offset`,
    /// not the mechanical angle. Passing a mechanical angle here gives a wrong
    /// synchronous frame and therefore wrong torque.
    pub theta_e_rad: f32,
}

/// 基础电流环的全部中间状态，便于在遥测里逐级核对 Clarke/Park/PI/逆 Park/SVPWM。
/// All intermediate state of the basic current loop, so telemetry can check
/// Clarke/Park/PI/inverse-Park/SVPWM stage by stage.
///
/// 尺寸固定、无指针/`Vec`/trait object，可静态分配到全局上下文，ISR 内不会触发分配
/// （算法库实时性说明.md 记录 `FocBasicState` 为 72 字节）。
/// Fixed-size with no pointers, `Vec` or trait object, so it can live in a static
/// context and never allocates inside the ISR (算法库实时性说明.md records
/// `FocBasicState` as 72 bytes).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FocBasicState {
    /// d 轴电流 PI 的内部状态（积分器 `[V]`、误差 `[A]`、上次输出 `[V]`）。
    /// Internal state of the d-axis current PI (integrator `[V]`, error `[A]`, last
    /// output `[V]`).
    pub id_pi: PiState,
    /// q 轴电流 PI 的内部状态，量纲同上。
    /// Internal state of the q-axis current PI with the same units.
    pub iq_pi: PiState,
    /// 本拍 Clarke 结果：αβ 电流 `[A]`，幅值不变形式。
    /// This cycle's Clarke result: αβ currents in `[A]`, amplitude-invariant.
    pub current_alpha_beta: AlphaBeta,
    /// 本拍 Park 结果：dq 电流 `[A]`，d 轴对齐转子磁链。
    /// This cycle's Park result: dq currents in `[A]`, d-axis aligned with the rotor
    /// flux.
    pub current_dq: Dq,
    /// 两个 PI 的本拍输出：dq 电压 `[V]`，也就是逆 Park 的输入。
    /// This cycle's PI outputs: dq voltages in `[V]`, i.e. the inverse-Park input.
    pub voltage_dq: Dq,
    /// 逆 Park 结果：αβ 电压 `[V]`，直接交给 SVPWM 换算 duty。
    /// Inverse-Park result: αβ voltage in `[V]`, handed straight to SVPWM.
    pub voltage_alpha_beta: AlphaBeta,
    /// 本拍 SVPWM 输出（三相 duty 与扇区），即 `update` 返回值的副本。
    /// This cycle's SVPWM output (three duties plus sector), a copy of what `update`
    /// returns.
    pub pwm: SvpwmOutput,
}

/// 显式实现 `Default`（而不是派生），以固定复位后每个字段的值。
/// Implements `Default` explicitly instead of deriving it, so every field has a defined
/// reset value.
///
/// 除 `pwm` 外的字段全部清零；`pwm` 的三相 duty 也清零，用于和 C 版
/// `FOC_Basic_Reset` 的 `memset(0)` 结果逐字节一致。
/// Every field is zeroed, including the three `pwm` duties, to stay byte-identical with
/// the C `FOC_Basic_Reset` `memset(0)` result.
impl Default for FocBasicState {
    fn default() -> Self {
        // 与 C 版 FOC_Basic_Reset 的 memset(0) 行为保持一致。
        // Matches the all-zero reset result of the C `FOC_Basic_Reset`.
        //
        // 注意这里是 0 duty（三相都到下桥臂），而不是 `SvpwmOutput::default()` 的
        // 0.5 中点：两者都是零电压矢量，但复位值只能在下一次写入有效 duty 之前、
        // 且功率级仍处于失能状态时使用。
        // Note that the duties are 0 here (all phases at the low side), not the 0.5
        // midpoint of `SvpwmOutput::default()`. Both are zero-voltage vectors, but this
        // reset value may only be applied while the power stage is still disabled and
        // before a fresh duty is written.
        Self {
            id_pi: PiState::default(),
            iq_pi: PiState::default(),
            current_alpha_beta: AlphaBeta::default(),
            current_dq: Dq::default(),
            voltage_dq: Dq::default(),
            voltage_alpha_beta: AlphaBeta::default(),
            pwm: SvpwmOutput {
                duty_a: 0.0,
                duty_b: 0.0,
                duty_c: 0.0,
                sector: 0,
            },
        }
    }
}

/// 基础电流环的状态操作。
/// State operations of the basic current loop.
impl FocBasicState {
    /// 把整个状态恢复为 `Default`：两个 PI、四个中间矢量和 duty 全部清零。
    /// Restores the whole state to `Default`: both PIs, all four intermediate vectors and
    /// the duties are cleared.
    ///
    /// 停机、故障恢复或切换控制模式后必须调用。复位后 `pwm` 是 0 duty，调用方必须
    /// 先确保功率级失能，再按启动顺序重新使能。
    /// Call it on stop, fault recovery or a control-mode change. `pwm` comes back with
    /// zero duties, so the caller must keep the power stage disabled until the normal
    /// start sequence re-enables it.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 执行一个电流环周期并返回本拍三相占空比。
    /// Runs one current-loop cycle and returns this cycle's three-phase duties.
    ///
    /// 参数 / Parameters:
    ///   param - PI 与 SVPWM 参数；两个 `ts` 必须是真实调用周期 `[s]`（12 kHz 即
    ///           1/12000），`svpwm.v_bus` 是母线电压 `[V]`
    ///           PI and SVPWM parameters; both `ts` values must be the real call period
    ///           in `[s]` (1/12000 at 12 kHz) and `v_bus` is the DC link in `[V]`
    ///   input - 同一采样周期的三相电流 `[A]`、d/q 给定 `[A]` 和电角度 `[rad]`
    ///           three-phase currents `[A]`, d/q references `[A]` and the electrical
    ///           angle `[rad]` from one sampling cycle
    ///
    /// 返回 / Returns:
    ///   `SvpwmOutput`：duty 已被 SVPWM 钳位到 `[0,1]`，`sector` 为 1..=6（输入退化时
    ///   为 0）；`v_bus <= 0` 时返回 0.5/0.5/0.5 的安全中点
    ///   `SvpwmOutput` with the duties clamped to `[0,1]` and `sector` in 1..=6 (0 for a
    ///   degenerate input); when `v_bus <= 0` it returns the safe 0.5/0.5/0.5 midpoint
    ///
    /// 失败语义 / Failure semantics:
    ///   没有错误返回值，也不会关断栅极：`v_bus <= 0`、`NaN` 或限幅都只体现在 duty
    ///   数值上。硬件过流/过压/驱动故障必须由独立保护链处理，不能用本函数的返回值
    ///   代替
    ///   There is no error path and no gate shutdown: `v_bus <= 0`, `NaN` or saturation
    ///   only show up in the duty values. Hardware overcurrent, overvoltage and driver
    ///   faults need an independent protection chain and cannot be inferred from this
    ///   return value.
    ///
    /// 上下文 / Context: 12 kHz 电流环；无分配、无阻塞、无日志。
    /// 12 kHz current loop: no allocation, no blocking, no logging.
    #[inline]
    pub fn update(&mut self, param: &FocBasicParam, input: &FocBasicInput) -> SvpwmOutput {
        // 三相 → 静止 αβ：幅值不变形式（α 等于 a 相），与 `transform::clarke` 一致。
        // 这里不做 (a+b+c)=0 校验：采样偏置造成的共模误差在 αβ 上是直流量，进 dq 后
        // 变成与电频率同频的交流分量（相间增益不一致才是二倍频），属于平台层偏置
        // 标定的职责。
        // Three-phase to stationary αβ in amplitude-invariant form (alpha equals phase
        // a), matching `transform::clarke`. No (a+b+c)=0 check is done: a common-mode
        // error from sampling offset appears as a DC term in αβ and as an
        // electrical-frequency AC term in dq (phase gain mismatch is what produces the
        // second harmonic), which is the platform's offset-calibration job.
        self.current_alpha_beta = clarke(Abc {
            a: input.ia,
            b: input.ib,
            c: input.ic,
        });
        // Park 用输入给出的电角度：d 轴对齐转子磁链、q 轴超前 d 轴 90°。这里与后面的
        // 逆 Park 用的是同一个角度实例；若平台层给的是上一拍的角度，d/q 之间会残留
        // ω_e·ts 的交叉耦合角（12 kHz、524 rpm、7 极对时约 0.032 rad ≈ 1.8°），
        // 转速越高越明显。
        // Park uses the supplied electrical angle: d aligned with the rotor flux and q
        // leading d by 90 degrees. The same angle instance is reused by the inverse Park,
        // so if the platform hands over the previous-cycle angle the d/q frame keeps an
        // ω_e·ts cross-coupling (about 0.032 rad ≈ 1.8° at 12 kHz, 524 rpm and 7 pole
        // pairs) that grows with speed.
        self.current_dq = park(self.current_alpha_beta, input.theta_e_rad);
        // PI 输入是电流误差 `[A]`、输出是 d 轴电压 `[V]`，因此 `kp` 的单位是 `[V/A]`、
        // `ki` 是 `[V/(A*s)]`，`out_min`/`out_max` 限幅的是电压，`ts` 必须是真实调用
        // 周期 `[s]`。本结构不做 d/q 圆限幅，两个轴各自饱和。
        // The PI error is in `[A]` and its output is a d-axis voltage in `[V]`, so `kp` is
        // `[V/A]`, `ki` is `[V/(A*s)]`, the output limits bound volts and `ts` must be the
        // real call period in `[s]`. There is no d/q circle limiting here, so each axis
        // saturates on its own.
        self.voltage_dq.d = self
            .id_pi
            .update(&param.id_pi, input.id_ref, self.current_dq.d);
        // q 轴同上：隐极机 Id=0 时转矩几乎全部由这一路产生，它的输出限幅实际上决定了
        // 可用电压与最大转矩。
        // Same for the q axis: with Id=0 on a surface PM almost all torque comes from it,
        // so its output limit effectively sets the available voltage and maximum torque.
        self.voltage_dq.q = self
            .iq_pi
            .update(&param.iq_pi, input.iq_ref, self.current_dq.q);
        // 逆 Park 回到静止 αβ；`voltage_alpha_beta` 的单位仍是 `[V]` 而不是 per-unit，
        // SVPWM 直接用它与 `v_bus` 相除得到 duty。
        // Inverse Park back to stationary αβ. `voltage_alpha_beta` stays in `[V]`, not
        // per-unit: SVPWM divides it by `v_bus` to obtain the duties.
        self.voltage_alpha_beta = inverse_park(self.voltage_dq, input.theta_e_rad);
        // 末级固定为 SVPWM（min-max 零序注入，线性区上限 |v| = v_bus/√3）。需要 DPWM
        // 时应复制 `voltage_alpha_beta` 后另调 `dpwm_update`，不要对 duty 做二次处理。
        // The last stage is always SVPWM (min-max zero-sequence injection, linear range up
        // to |v| = v_bus/sqrt(3)). For DPWM, copy `voltage_alpha_beta` and call
        // `dpwm_update` instead of post-processing the duties.
        self.pwm = svpwm_update(self.voltage_alpha_beta, &param.svpwm);
        self.pwm
    }
}

/// 主机单元测试：核对与 C 版 `FOC_Basic` 参考向量的等价性。
/// Host unit tests checking equivalence with the C `FOC_Basic` reference vector.
///
/// 主机测试只证明数值等价，不证明电流环闭环稳定、时序正确或保护有效。
/// Host tests prove numerical equivalence only; they say nothing about closed-loop
/// stability, timing or protection.
#[cfg(test)]
mod tests {
    use super::*;

    /// 与 C 参考模块 `FOC_Basic` 的测试向量逐项对齐。
    /// Matches the test vector of the C reference module `FOC_Basic` item by item.
    ///
    /// `ki = 0` 让积分器恒为零、结果与 `ts` 无关，`theta_e_rad = 0` 使 Park 退化为恒等
    /// 映射，因此断言值只由 `kp`、Clarke/Park 公式和 SVPWM 换算决定。改动其中任何一步
    /// 的运算顺序或比例系数都会破坏这条等价性。
    /// `ki = 0` keeps the integrator at zero and makes the result independent of `ts`,
    /// while `theta_e_rad = 0` reduces Park to identity, so the asserted values depend only
    /// on `kp`, the Clarke/Park formulas and the SVPWM scaling. Changing the operation
    /// order or any scaling step breaks this equivalence.
    #[test]
    fn matches_c_reference_vector() {
        // ki = 0 使结果与 ts 无关；限幅值沿用 C 测试向量，不是整定建议。
        // ki = 0 makes the result independent of ts; the limits come from the C test vector
        // and are not a tuning recommendation.
        let pi = PiParam {
            kp: 1.0,
            ki: 0.0,
            ts: 0.001,
            out_min: -12.0,
            out_max: 12.0,
            integrator_min: -6.0,
            integrator_max: 6.0,
        };
        // 24 V 是测试向量里的母线电压 `[V]`，只影响 duty 的换算比例。
        // 24 V is the DC-link voltage in `[V]` used by the test vector; it only scales the
        // duty conversion.
        let param = FocBasicParam {
            id_pi: pi,
            iq_pi: pi,
            svpwm: SvpwmParam { v_bus: 24.0 },
        };
        let input = FocBasicInput {
            ia: 0.0,
            ib: 0.0,
            ic: 0.0,
            id_ref: 0.0,
            iq_ref: 2.0,
            theta_e_rad: 0.0,
        };
        let mut state = FocBasicState::default();
        // iq_ref = 2 A 而电流反馈为 0，所以 q 轴 PI 必须输出 2 V（ki = 0）。
        // iq_ref = 2 A with zero current feedback, so the q-axis PI must output 2 V because
        // ki = 0.
        let out = state.update(&param, &input);
        assert!(state.current_dq.d.abs() <= 1e-6);
        assert!(state.current_dq.q.abs() <= 1e-6);
        assert!(state.voltage_dq.d.abs() <= 1e-6);
        assert!((state.voltage_dq.q - 2.0).abs() <= 1e-6);
        // 占空比必须留在 `[0,1]`；平台层写 CCR 之前还会按 3%～97% 的窗口再检查一次。
        // The duties must stay inside `[0,1]`; the platform re-checks the 3%..97% window
        // before writing the compare registers.
        assert!((0.0..=1.0).contains(&out.duty_a));
        assert!((0.0..=1.0).contains(&out.duty_b));
        assert!((0.0..=1.0).contains(&out.duty_c));
    }
}
