//! 高频注入、旋转高频注入、脉冲注入及 HF/BEMF 融合。
//! High-frequency injection, rotating HF injection, pulse injection and HF/BEMF
//! fusion.
//!
//! FluxRT —— 高频注入观测器组（`foc-algorithm` 的 `observer` 子模块）。
//! FluxRT - the high-frequency injection observer group of `foc-algorithm`.
//!
//! 职责 / Responsibility:
//!   - `HfInjectionState`：固定（静止）轴高频电压注入 + 同步解调，输出与转子
//!     凸极位置相关的解调量，用于零速/极低速无感。
//!   - `RotatingHfState`：旋转高频电压注入，由 αβ 电流响应的凸极图像取角度。
//!   - `PulseInjectionState`：沿固定轴正负交替脉冲注入，比较两次电流响应之差，
//!     用于初始位置与磁极极性辨识；它不是连续运行的观测器。
//!   - `HfBemfState`：按 |速度| 对低速高频注入角度和中高速 BEMF 角度加权融合，
//!     得到全速域电角度。
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!     -> foc-algorithm -> `observer::injection`（本模块）
//!   向下依赖 `crate::math`（角度环绕与限幅）、`crate::transform::AlphaBeta`
//!   以及同层的 `observer::bemf`（`HfBemfState` 内嵌 `BemfState`）。
//!   本模块不认识 HAL、RTOS、堆，也没有全局可变状态，且不得反向调用上层。
//!   Depends only downwards on `crate::math`, `crate::transform` and the sibling
//!   `observer::bemf`; it never reaches up into `foc-control` or the C platform.
//!
//! 实时约束 / Real-time constraints:
//!   可能被 12 kHz 的 ADC 注入中断逐拍调用。全部为纯计算：不分配、不阻塞、
//!   不打日志、无锁。固定轴注入每拍调用两次 `libm` 超越函数，旋转注入另加
//!   `atan2`，是快环里最贵的几处调用；是否放进 ISR 必须由目标板 WCET 实测决定，
//!   也可换成芯片 CORDIC 或查表正余弦（见 README 的 RT-Thread 接入边界一节）。
//!   May be called from the 12 kHz ADC ISR. Pure computation, but every call uses
//!   `libm` `sinf`/`cosf` (plus `atan2` for rotating injection), so in-ISR
//!   placement must be decided by measured WCET.
//!
//! 量纲约定 / Unit conventions:
//!   电压统一按正弦**峰值** `[V]`（不是有效值），电流 `[A]`，角度 `[rad]`，
//!   载波频率 `[Hz]`，采样周期 `[s]`；`demod_alpha` 是无量纲的一阶低通系数。
//!   `HfBemfParam::speed_low_abs` / `speed_high_abs` 与 `HfBemfInput::speed_abs`
//!   的量纲由调用方自定（`[rpm]` 或 `[rad/s]` 均可），本模块只用它们的比值，
//!   不做任何换算，但三处必须同量纲。本模块没有任何定点 Q 格式，全部为 `f32`。
//!   Voltages are sine peak values in `[V]`, currents in `[A]`, angles in `[rad]`,
//!   frequency in `[Hz]`, periods in `[s]`. The speed thresholds are caller-defined
//!   units and are only ever used as a ratio.
//!
//! 边界说明 / Scope note:
//!   单独调用 `HfBemfState::update()` 不等于完成全速域无感。工程侧还必须实现注入
//!   电压叠加、基波/载波分离、凸极性判定、角度 π 模糊消除、切换滞环、可信度和
//!   失败回退；`Ld == Lq` 的 SPM 无法由高频注入给出可靠角度。
//!   Calling `update()` does not by itself deliver sensorless full-speed
//!   operation; superposition, carrier separation, π ambiguity removal, hand-over
//!   hysteresis, confidence and fallback stay in the project layer.
//!
//! 常数来源 / Constant provenance:
//!   本模块不含任何带 `[HW]`/`[ST]`/`[FW]`/`[VESC]` 标签的标定常数：注入幅值、载波
//!   频率、解调系数和速度阈值全部由调用方给定，仓库内没有记录这些数值的来源。测试
//!   里的 2 V/3 V 与 25 Hz/100 Hz 只是参考向量的输入，不是可直接使用的整定值。
//!   No calibrated constants with provenance tags appear here; the numbers used by the
//!   tests are reference-vector inputs, not tuned machine settings.
//!
//! 参考 / Reference: `算法库移植状态.md` 第 39～42 行、`算法库总览与对接指南.md`
//!   §4.5、docs/FOC算法组合与应用场景.md §4.7 组合 G。

use super::{BemfInput, BemfParam, BemfState};
use crate::math::{
    atan2_angle_0_to_2pi, clamp, wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi, TWO_PI,
};
use crate::transform::AlphaBeta;

/// 固定（静止）轴高频电压注入参数。
/// Stationary-axis high-frequency voltage injection parameters.
///
/// 参数 / Parameters:
///   amplitude      注入电压正弦峰值 `[V]`。它同时决定解调信噪比、注入电流峰值
///                  `[A]`（约 `amplitude / (ω·L)`）和可听噪声：幅值越大低速角度
///                  越稳，但声噪、转矩纹波与铜损也同步上升，必须受功率级和电机
///                  温升限制。具体取值来源未在仓库内标注。
///                  Peak injected voltage in `[V]`; it sets the demodulated SNR,
///                  the injected current `amplitude / (ω·L)` in `[A]`, the audible
///                  noise and the extra copper loss at the same time.
///   freq_hz        载波频率 `[Hz]`。必须远高于电流环带宽和基波频率（否则基波与
///                  载波无法分离），同时避开人耳最敏感频段以降低声噪；具体取值
///                  来源未在仓库内标注。
///                  Carrier frequency in `[Hz]`, well above the current-loop
///                  bandwidth and the fundamental; its origin is not identified
///                  in-tree.
///   ts             调用周期 `[s]`，必须等于真实调用节拍；`<= 0` 时 `update`
///                  返回零电压且完全不改状态。
///                  Sampling period in `[s]`; `<= 0` returns zero voltage and
///                  leaves the state untouched.
///   axis_angle_rad 静止 αβ 平面上的注入轴方向 `[rad]`。注入方向由这个常量决定
///                  而不是由转子角度决定，所以本观测器不需要初始角度、也不怕
///                  反向旋转。
///                  Injection axis in the stationary αβ frame, in `[rad]`.
///   demod_alpha    同步解调后一阶低通的系数，无量纲，函数内部再夹到 `[0,1]`。
///                  等效截止频率约 `alpha / (2π·ts)` `[Hz]`（`alpha` 较小时）：
///                  太小则角度滞后、动态差，太大则载波残余与噪声直接进入解调量。
///                  First-order demodulation LPF coefficient, clamped to `[0,1]`;
///                  corner frequency is about `alpha / (2π·ts)` in `[Hz]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfInjectionParam {
    pub amplitude: f32,
    pub freq_hz: f32,
    pub ts: f32,
    pub axis_angle_rad: f32,
    pub demod_alpha: f32,
}

/// 固定轴高频注入的运行状态。
/// Runtime state of the stationary-axis HF injection.
///
/// 字段 / Fields:
///   carrier_angle_rad 载波相位 `[rad]`。每拍按 `2π·freq_hz·ts` 递增后环绕到
///                     `[0, 2π)`，避免长期累加把 `f32` 的有效位吃掉。
///                     Carrier phase in `[rad]`, advanced by `2π·freq_hz·ts` and
///                     wrapped every call so long runs keep `f32` resolution.
///   carrier           当前拍载波值，无量纲，范围 `[-1, 1]`；单独保存便于诊断，
///                     也便于上层判断注入是否真的在工作。
///                     Current carrier sample, dimensionless in `[-1, 1]`.
///   voltage           本拍应叠加到电压指令上的 αβ 注入电压 `[V]`（也是 `update`
///                     的返回值）。它必须由调用方**相加**到基波电压上，函数本身
///                     不修改任何调制输出。
///                     The αβ injection voltage in `[V]` to be added to the
///                     fundamental by the caller.
///   demod_response    同步解调并低通后的响应 `[A]`，保留 0.5 倍解调增益，
///                     详见 `update`。
///                     Demodulated, low-passed response in `[A]`; keeps the 0.5
///                     demodulation gain (see `update`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfInjectionState {
    pub carrier_angle_rad: f32,
    pub carrier: f32,
    pub voltage: AlphaBeta,
    pub demod_response: f32,
}

/// 固定轴高频注入的状态机（`Observer_HFInjection`）。
/// The stationary-axis injection state machine (`Observer_HFInjection`).
impl HfInjectionState {
    /// 复位注入状态：载波相位、载波值、输出电压和解调量全部清零。
    /// Resets the injection state: phase, carrier, voltage and demodulated value.
    ///
    /// 默认载波相位为 0，因此复位后的第一拍注入方向为 `+amplitude`；切换控制模式、
    /// 重新使能功率级或改变注入轴之后必须先复位，否则解调量的旧值会以低通时间
    /// 常数慢慢泄放，过渡期角度不可信。
    /// The carrier phase restarts at 0, so the first call after a reset injects
    /// along `+amplitude`; reset before re-enabling the power stage or changing the
    /// injection axis.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 推进一拍固定轴高频注入并做同步解调。
    /// Advances one step of stationary-axis HF injection with synchronous
    /// demodulation.
    ///
    /// 参数 / Parameters:
    ///   param            注入参数（见 `HfInjectionParam`）
    ///   current_response 本拍测得的电流响应 `[A]`。它必须与**上一拍**发出的注入
    ///                    电压同相（典型做法是取 αβ 电流在注入轴上的投影，且电流
    ///                    采样发生在本次注入电压生效之前）；相位关系搞错时解调量
    ///                    会随载波相位摆动，角度信息被抵消。
    ///                    Measured current response in `[A]`, phase-aligned with the
    ///                    previously issued injection voltage.
    ///
    /// 返回 / Returns: 本拍需要叠加到基波电压上的 αβ 注入电压 `[V]`，峰值不超过
    ///   `|amplitude|`；`ts <= 0` 时返回零电压 `AlphaBeta::default()`，下游 SVPWM
    ///   在电压为零时退化为 `0.5/0.5/0.5` 安全中点。
    ///   The αβ injection voltage in `[V]` to add to the fundamental; zero when
    ///   `ts <= 0`.
    ///
    /// 行为与陷阱 / Behaviour and pitfalls:
    ///   - `ts <= 0` 属于"停拍"而不是报错：函数直接返回，载波相位与解调量都保持
    ///     原值，调用方不能据此判断注入是否已经停止。
    ///   - 解调是"电流响应乘回同相载波再低通"。对 `demod_alpha` 较小的平均解调，
    ///     `i·sin(ωt)` 的直流分量只有注入同相分量的一半，所以 `demod_response`
    ///     比真实同相电流小 2 倍：它适合做比较、判符号或闭环，不能当作电流测量值
    ///     使用。这个系数与 C 参考实现一致，改动会让
    ///     `tests::hf_injection_matches_c_reference` 失配。
    ///   - 解调用的是本拍新算出的载波，而 `current_response` 来自上一拍电压，两者
    ///     之间存在 `2π·freq_hz·ts` 的固定相位偏差；载波频率越高偏差越大，这是
    ///     载波频率不能取得过高的原因之一。
    ///   - 解调量初值为 0，需要若干拍（约 `1/demod_alpha`）才收敛；收敛前不应把
    ///     它当作有效角度信息使用。
    ///   - 每拍调用 `sinf` 与 `cosf`，其中 `cosf(axis_angle_rad)` 对固定注入轴是
    ///     常量却每拍重算，是快环 WCET 的可见开销，可考虑由调用方预计算。
    pub fn update(&mut self, param: &HfInjectionParam, current_response: f32) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        self.carrier_angle_rad =
            wrap_angle_0_to_2pi(self.carrier_angle_rad + TWO_PI * param.freq_hz * param.ts);
        self.carrier = libm::sinf(self.carrier_angle_rad);
        self.voltage.alpha = param.amplitude * self.carrier * libm::cosf(param.axis_angle_rad);
        self.voltage.beta = param.amplitude * self.carrier * libm::sinf(param.axis_angle_rad);
        let response = current_response * self.carrier;
        let alpha = clamp(param.demod_alpha, 0.0, 1.0);
        self.demod_response += alpha * (response - self.demod_response);
        self.voltage
    }
}

/// 旋转高频电压注入参数。
/// Rotating high-frequency voltage injection parameters.
///
/// 凸极性前提 / Saliency requirement:
///   本策略只在 `Ld != Lq` 的凸极电机（IPMSM）上有效。`Ld == Lq` 时电流响应里
///   不含转子位置信息，注入幅值再大也估不出角度；上机前应先用受控角度扫描测出
///   `Ld/Lq` 差异，工程文档要求"有独立角度真值、凸极证据后再研究 HFI"。
///   Only meaningful when `Ld != Lq`: an SPMSM response carries no rotor position
///   information no matter how large the injection is.
///
/// 参数 / Parameters:
///   amplitude   旋转电压矢量幅值 `[V]`（正弦峰值）。与固定轴注入相同，幅值同时
///               决定解调信噪比、注入电流峰值 `[A]`、声噪和温升；具体取值来源未在
///               仓库内标注。
///               Peak rotating voltage magnitude in `[V]`; origin not identified
///               in-tree.
///   freq_hz     载波频率 `[Hz]`。需要高于电流环带宽和基波频率数个量级，才能让
///               解调后的低通把基波与载波残余分开。
///               Carrier frequency in `[Hz]`.
///   ts          调用周期 `[s]`；`<= 0` 时 `update` 返回零电压且不改状态。
///               Sampling period in `[s]`.
///   demod_alpha 解调低通系数，无量纲，内部夹到 `[0,1]`；等效截止频率约
///               `alpha / (2π·ts)` `[Hz]`，必须远低于 `freq_hz`，否则载波残余直接
///               变成角度抖动。
///               Demodulation LPF coefficient; its corner must sit far below
///               `freq_hz`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RotatingHfParam {
    pub amplitude: f32,
    pub freq_hz: f32,
    pub ts: f32,
    pub demod_alpha: f32,
}

/// 旋转高频注入的运行状态。
/// Runtime state of the rotating HF injection.
///
/// 字段 / Fields:
///   carrier_angle_rad 载波相位 `[rad]`，每拍环绕到 `[0, 2π)`。
///                     Carrier phase in `[rad]`, wrapped into `[0, 2π)`.
///   voltage           本拍要叠加的旋转 αβ 注入电压 `[V]`，模长等于 `amplitude`。
///                     Rotating αβ injection voltage in `[V]`.
///   demod             解调后低通的 αβ 分量 `[A]`，保留 0.5 倍解调增益。
///                     Low-passed demodulated αβ components in `[A]`.
///   theta_est_rad     估计电角度 `[rad]`，落在 `[0, π)`。本式对磁极 N/S 不敏感，
///                     只给出 mod π 的角度，极性必须由脉冲注入或其它手段消除后
///                     才能与 BEMF 角度融合。
///                     Estimated electrical angle in `[rad]` inside `[0, π)`,
///                     i.e. only defined modulo π.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RotatingHfState {
    pub carrier_angle_rad: f32,
    pub voltage: AlphaBeta,
    pub demod: AlphaBeta,
    pub theta_est_rad: f32,
}

/// 旋转高频注入的状态机（`Observer_RotatingHFInjection`）。
/// The rotating HF injection state machine (`Observer_RotatingHFInjection`).
impl RotatingHfState {
    /// 复位旋转注入状态：载波相位、注入电压、解调量、估计角度全部清零。
    /// Resets the rotating injection state: carrier phase, voltage, demodulated
    /// values and the estimated angle.
    ///
    /// 复位后 `theta_est_rad` 为 0，但这是"没有信息"的 0 而不是"转子在 0 rad"；
    /// 解调量收敛到有效值之前，调用方不得把这个角度送入角度环。
    /// After a reset `theta_est_rad` is 0, which means "no information" rather than
    /// "the rotor is at 0 rad".
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 推进一拍旋转高频注入，解调出凸极角度。
    /// Advances one step of rotating HF injection and extracts the saliency angle.
    ///
    /// 参数 / Parameters:
    ///   param            旋转注入参数（见 `RotatingHfParam`）
    ///   current_response 本拍 αβ 电流响应 `[A]`。它必须已经扣除基波电流：基波、
    ///                    死区与续流造成的逆变器非线性都会直接叠在解调量上，变成
    ///                    与负载电流相关的角度误差。工程侧仍需自行完成基波/载波
    ///                    分离与可信度判定。
    ///                    αβ current response in `[A]` with the fundamental already
    ///                    removed; inverter non-linearity shows up directly as
    ///                    load-dependent angle error.
    ///
    /// 返回 / Returns: 本拍 αβ 注入电压 `[V]`（模长 `|amplitude|`）；`ts <= 0` 时
    ///   返回零电压。
    ///   The αβ injection voltage in `[V]`; zero when `ts <= 0`.
    ///
    /// 行为与陷阱 / Behaviour and pitfalls:
    ///   - 每拍重新累加并环绕载波相位，保证长期运行时相位分辨率不退化。
    ///   - 解调是两轴**各自**与同相载波相乘后低通（`i_alpha·cos` 与 `i_beta·sin`），
    ///     没有交叉项，与经典旋转注入所用的复解调（需要 `i_alpha·cos - i_beta·sin`
    ///     和 `i_alpha·sin + i_beta·cos` 才能同时取出正/负序分量）不同。因此这里
    ///     取出的两个量只承载凸极图像的一部分信息，其符号与相位完全依赖调用方的
    ///     相序、极性和电流符号约定；换相序或改电流符号会让估计角度整体错位。
    ///     运算顺序与 C 参考实现一致，改动会让
    ///     `tests::rotating_hf_matches_c_reference` 失配。
    ///   - 取角时乘 0.5 是因为凸极图像以 `2·theta` 的周期出现；这也意味着结果
    ///     天然存在 π 模糊，必须由工程侧消除极性后才能与 BEMF 角度融合。
    ///   - 解调量初值为 0 时 `atan2(0, 0)` 返回 0，`theta_est_rad` 也是 0；在低通
    ///     收敛前该角度没有物理意义，需要约 `1/demod_alpha` 拍才可用。
    ///   - 本函数不注入、不调制，也不检查峰值电流：把返回值叠加到电压指令、并限制
    ///     注入引起的电流峰值，是调用方的责任。
    pub fn update(&mut self, param: &RotatingHfParam, current_response: AlphaBeta) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        self.carrier_angle_rad =
            wrap_angle_0_to_2pi(self.carrier_angle_rad + TWO_PI * param.freq_hz * param.ts);
        let c = libm::cosf(self.carrier_angle_rad);
        let s = libm::sinf(self.carrier_angle_rad);
        self.voltage.alpha = param.amplitude * c;
        self.voltage.beta = param.amplitude * s;
        let alpha = clamp(param.demod_alpha, 0.0, 1.0);
        self.demod.alpha += alpha * (current_response.alpha * c - self.demod.alpha);
        self.demod.beta += alpha * (current_response.beta * s - self.demod.beta);
        self.theta_est_rad =
            wrap_angle_0_to_2pi(0.5 * atan2_angle_0_to_2pi(self.demod.beta, self.demod.alpha));
        self.voltage
    }
}

/// 正负脉冲注入参数（初始位置与磁极极性辨识用）。
/// Positive/negative pulse injection parameters for initial position and magnet
/// polarity identification.
///
/// 用途边界 / Scope:
///   它不是连续运行的主观测器，只回答"转子大致在哪个方向、磁极朝哪一边"；
///   辨识完成后必须交回旋转高频或 BEMF 路径（见 docs/FOC算法组合与应用场景.md §4.7 组合 G）。
///   This is not a continuous main observer; the project must hand over to the
///   rotating-HF or BEMF path once identification is done.
///
/// 参数 / Parameters:
///   amplitude      脉冲电压幅值 `[V]`。幅值越大，正向与反向脉冲的饱和差异越明显
///                  （`saliency` 越大），但脉冲结束时的电流约 `amplitude/L · T_pulse`
///                  `[A]` 也越大，必须受电机与功率级峰值电流限制。具体取值来源
///                  未在仓库内标注。
///                  Pulse amplitude in `[V]`; the current it drives is roughly
///                  `amplitude/L · T_pulse` in `[A]` and must stay inside the
///                  ratings. Origin not identified in-tree.
///   axis_angle_rad 脉冲施加方向 `[rad]`（静止 αβ 平面）。要对多个候选方向各发一对
///                  脉冲，才能同时确定转子所在扇区与磁极极性。
///                  Pulse direction in `[rad]` in the stationary αβ frame; several
///                  directions are needed to resolve both sector and polarity.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PulseInjectionParam {
    pub amplitude: f32,
    pub axis_angle_rad: f32,
}

/// 正负脉冲注入的运行状态。
/// Runtime state of the positive/negative pulse injection.
///
/// 字段 / Fields:
///   polarity           本拍脉冲的符号（`+1` 或 `-1`）。每次 `update` 先翻转再
///                      使用，所以它始终表示**本拍**即将施加的极性。
///                      Sign of the pulse about to be applied, `+1` or `-1`.
///   voltage            本拍 αβ 注入电压 `[V]`：方向 `axis_angle_rad`，符号取翻转
///                      后的 `polarity`。
///                      The αβ pulse voltage in `[V]`.
///   positive_response  最近一次正向脉冲的电流响应 `[A]`。
///                      Current response of the most recent positive pulse in `[A]`.
///   negative_response  最近一次负向脉冲的电流响应 `[A]`。
///                      Current response of the most recent negative pulse in `[A]`.
///   saliency           两者之差 `[A]`。铁芯饱和使正反方向的等效电感不同，这个差值
///                      因此带磁极极性信息：符号反映 N/S 方向，幅值反映凸极与饱和
///                      强弱。它来自**相邻的两个不同脉冲**，所以两次脉冲的幅值、
///                      方向与持续时间必须完全一致，差值才有物理意义。
///                      Difference in `[A]` between the two most recent pulses; its
///                      sign carries the magnet polarity, so the two pulses must be
///                      identical in amplitude, direction and duration.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PulseInjectionState {
    pub polarity: i32,
    pub voltage: AlphaBeta,
    pub positive_response: f32,
    pub negative_response: f32,
    pub saliency: f32,
}

/// 手写 `Default`（而不是 derive）：`polarity` 必须是 `+1` 而不是 0，这样复位后的
/// 第一次 `update` 必然发出**负向**脉冲，正负脉冲序列从确定状态开始，极性判据的
/// 符号才可复现。
/// Hand-written `Default` rather than derived: `polarity` starts at `+1` so the
/// first `update` after a reset always emits the negative pulse and the sign of the
/// polarity criterion stays reproducible.
impl Default for PulseInjectionState {
    fn default() -> Self {
        Self {
            polarity: 1,
            voltage: AlphaBeta::default(),
            positive_response: 0.0,
            negative_response: 0.0,
            saliency: 0.0,
        }
    }
}

/// 正负脉冲注入的状态机（`Observer_PulseInjection`）。
/// The positive/negative pulse injection state machine
/// (`Observer_PulseInjection`).
impl PulseInjectionState {
    /// 复位脉冲注入状态：极性回到 `+1`，响应与差值为 0。
    /// Resets the pulse injection state: polarity back to `+1`, responses and
    /// saliency to 0.
    ///
    /// `saliency` 归零表示"还没有一对有效脉冲"，调用方不能用 0 当作"没有凸极"。
    /// A zero `saliency` means "no valid pulse pair yet", not "no saliency".
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 推进一拍脉冲注入：翻转极性、记录电流响应、更新凸极差值。
    /// Advances one pulse step: flips polarity, stores the current response and
    /// updates the saliency difference.
    ///
    /// 参数 / Parameters:
    ///   param            脉冲参数（见 `PulseInjectionParam`）
    ///   current_response 本拍电流响应 `[A]`。必须在脉冲已经建立（电流到达稳态）
    ///                    之后采样，否则差值里混入上升沿信息，磁极判据可能翻转。
    ///                    Current response in `[A]`, sampled after the pulse has
    ///                    settled, otherwise the rise transient can flip the
    ///                    polarity decision.
    ///
    /// 返回 / Returns: 本拍 αβ 注入电压 `[V]`，方向为 `axis_angle_rad`，符号取
    ///   **已经翻转过的** `polarity`；因此复位后的第一拍总是负脉冲。
    ///   The αβ pulse voltage in `[V]` with the already-flipped polarity, so the
    ///   first call after a reset always emits the negative pulse.
    ///
    /// 行为与陷阱 / Behaviour and pitfalls:
    ///   - 函数没有时间参数、也不判超时：脉冲宽度、间隔和"等电流建立"的等待完全由
    ///     调用方按拍数控制。每调用一次就换向一次，误调用会把脉冲对拆散，使
    ///     `saliency` 变成两个不同工况响应之差。
    ///   - `positive_response` 与 `negative_response` 是原始电流量 `[A]`，没有按脉冲
    ///     宽度或电压归一化；改变幅值、方向或脉宽后，先前的差值不再可比，必须复位
    ///     并重新成对测量。
    ///   - 运算顺序与符号与 C 参考实现一致，改动会让
    ///     `tests::pulse_injection_matches_c_reference` 失配。
    pub fn update(&mut self, param: &PulseInjectionParam, current_response: f32) -> AlphaBeta {
        if self.polarity >= 0 {
            self.positive_response = current_response;
            self.polarity = -1;
        } else {
            self.negative_response = current_response;
            self.polarity = 1;
        }
        self.saliency = self.positive_response - self.negative_response;
        let polarity = self.polarity as f32;
        self.voltage.alpha = param.amplitude * polarity * libm::cosf(param.axis_angle_rad);
        self.voltage.beta = param.amplitude * polarity * libm::sinf(param.axis_angle_rad);
        self.voltage
    }
}

/// 全速域 HF/BEMF 融合参数。
/// Full-speed HF/BEMF fusion parameters.
///
/// 参数 / Parameters:
///   hf             低速段使用的旋转高频注入参数（见 `RotatingHfParam`）。
///                  Rotating HF parameters used in the low-speed region.
///   bemf           中高速段使用的反电势观测器参数：`rs` `[ohm]`、`ls` `[H]`、
///                  `ts` `[s]`、`emf_filter_alpha` 无量纲。它的时间常数决定了
///                  `theta_bemf_rad` 的滞后。
///                  BEMF observer parameters: `rs` in `[ohm]`, `ls` in `[H]`, `ts`
///                  in `[s]`, `emf_filter_alpha` dimensionless.
///   speed_low_abs  融合下限 |速度|：低于此值时权重完全给高频注入。单位由调用方
///                  自定（`[rpm]` 或 `[rad/s]` 均可），但必须与 `speed_high_abs`
///                  和 `HfBemfInput::speed_abs` 一致；本模块只做比值。
///                  Lower blend threshold on |speed| in caller-defined units.
///   speed_high_abs 融合上限 |速度|：高于此值时权重完全给 BEMF。它应当取"反电势
///                  已经足够可信"的速度，而不是反电势刚开始出现的速度。
///                  Upper blend threshold on |speed|; pick it where the BEMF is
///                  already trustworthy.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfBemfParam {
    pub hf: RotatingHfParam,
    pub bemf: BemfParam,
    pub speed_low_abs: f32,
    pub speed_high_abs: f32,
}

/// 全速域融合的一拍输入。
/// One-step input of the full-speed fusion.
///
/// 字段 / Fields:
///   bemf                反电势观测器输入：αβ 电压 `[V]`、αβ 电流 `[A]`。
///                       BEMF observer input: αβ voltage in `[V]`, αβ current in
///                       `[A]`.
///   hf_current_response 高频注入的 αβ 电流响应 `[A]`，直接送给旋转注入解调。
///                       αβ current response of the HF injection in `[A]`.
///   speed_abs           当前速度的绝对值，单位由调用方自定，但必须与
///                       `speed_low_abs`、`speed_high_abs` 同量纲。取绝对值是为了
///                       让融合在正反转两个方向对称工作：换向时不应重新触发切换。
///                       |speed| in caller-defined units; the absolute value keeps
///                       the blend symmetric for both rotation directions.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfBemfInput {
    pub bemf: BemfInput,
    pub hf_current_response: AlphaBeta,
    pub speed_abs: f32,
}

/// 全速域 HF/BEMF 融合状态（`Observer_HF_BEMF`）。
/// Full-speed HF/BEMF fusion state (`Observer_HF_BEMF`).
///
/// 字段 / Fields:
///   hf             旋转高频注入子状态。
///                  Rotating HF sub-state.
///   bemf           反电势观测器子状态。
///                  BEMF observer sub-state.
///   hf_voltage     本拍高频注入电压 `[V]`。函数只做保存，需要由调用方叠加到电压
///                  指令上，注入才会真正发生。
///                  This step's HF injection voltage in `[V]`; the caller still has
///                  to add it to the voltage command.
///   bemf_emf       本拍估出的 αβ 反电势 `[V]`。
///                  Estimated αβ back-EMF in `[V]`.
///   theta_hf_rad   高频注入角度 `[rad]`，只有 mod π 的有效性。
///                  HF injection angle in `[rad]`, valid only modulo π.
///   theta_bemf_rad BEMF 角度 `[rad]`，范围 `[0, 2π)`。
///                  BEMF angle in `[rad]` inside `[0, 2π)`.
///   theta_rad      融合后的输出电角度 `[rad]`，范围 `[0, 2π)`。
///                  Blended output electrical angle in `[rad]`.
///   bemf_weight    BEMF 通道权重，无量纲 `[0,1]`：0 表示纯高频注入，1 表示纯
///                  BEMF。它同时是诊断"当前使用哪一路角度"的依据。
///                  Weight of the BEMF channel in `[0,1]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfBemfState {
    pub hf: RotatingHfState,
    pub bemf: BemfState,
    pub hf_voltage: AlphaBeta,
    pub bemf_emf: AlphaBeta,
    pub theta_hf_rad: f32,
    pub theta_bemf_rad: f32,
    pub theta_rad: f32,
    pub bemf_weight: f32,
}

/// 全速域 HF/BEMF 融合的状态机。
/// The full-speed HF/BEMF fusion state machine.
impl HfBemfState {
    /// 复位融合状态：高频注入、BEMF 观测器、两路角度、权重和输出全部清零。
    /// Resets the fusion: HF injection, BEMF observer, both angles, the weight and
    /// the output.
    ///
    /// 复位后 `bemf_weight` 为 0，即默认回到"低速、纯高频注入"一侧；重新使能前
    /// 必须先让两路各自收敛，否则融合输出会在过渡期跳变。
    /// `bemf_weight` returns to 0 (pure HF injection); let both paths settle again
    /// before re-enabling.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 推进一拍全速域融合，返回融合后的电角度。
    /// Advances one fusion step and returns the blended electrical angle.
    ///
    /// 参数 / Parameters:
    ///   param 融合参数（见 `HfBemfParam`）
    ///   input 本拍输入（见 `HfBemfInput`）
    ///
    /// 返回 / Returns: 融合电角度 `[rad]`，落在 `[0, 2π)`，可直接作为
    ///   `FocBasicInput::theta_e_rad`（是电角度，不是机械角度）。
    ///   Blended electrical angle in `[rad]` inside `[0, 2π)`, usable directly as
    ///   `FocBasicInput::theta_e_rad`.
    ///
    /// 行为与陷阱 / Behaviour and pitfalls:
    ///   - 两个子观测器**每拍都被推进**，与当前权重无关。这样权重变化时两路状态
    ///     都是连续的，交接瞬间不会因为"重新起步"而跳变；代价是低速段也在计算
    ///     BEMF，且每拍都要喂真实电流/电压，WCET 与最低速段的开销由两路之和决定。
    ///   - 权重按 |速度| 线性插值。`span <= 0`（阈值写反或相等）时退化为硬切换，
    ///     会产生角度阶跃；正常整定应留出足够宽的过渡带，并保证过渡带下沿处 BEMF
    ///     角度已经可信。
    ///   - 插值用 `wrap_angle_minus_pi_to_pi` 求两路角度差的最短表示，避免它们分别
    ///     跨过 0/2π 时算出接近 2π 的假差值把输出甩向错误方向。
    ///   - 本函数**没有滞环、没有驻留时间、也没有可信度判断**：速度在阈值附近抖动
    ///     时 `bemf_weight` 会逐拍来回变化。滞环、驻留、失败回退和角度真值校核都
    ///     必须由工程层实现（docs/FOC算法组合与应用场景.md §4.7）。
    ///   - `theta_hf_rad` 只有 mod π 的有效性：如果调用方没有先消除磁极极性模糊，
    ///     融合会把 BEMF 角度拉偏最多 π，方向可能完全相反。
    ///   - 高频解调低通与 BEMF 滤波的时间常数必须匹配：前者太快会让低速角度抖动，
    ///     后者太慢会让高速角度滞后，两者都会在交接点留下可观测的角度误差。
    ///   - 函数不检查 `NaN/Inf`，也不限幅输出；把返回值送入 Park/SVPWM 之前应确认
    ///     它有限，否则会污染整个电流环。
    pub fn update(&mut self, param: &HfBemfParam, input: &HfBemfInput) -> f32 {
        self.hf_voltage = self.hf.update(&param.hf, input.hf_current_response);
        self.bemf_emf = self.bemf.update(&param.bemf, &input.bemf);
        self.theta_hf_rad = self.hf.theta_est_rad;
        self.theta_bemf_rad = self.bemf.theta_emf_rad;
        let span = param.speed_high_abs - param.speed_low_abs;
        self.bemf_weight = if span <= 0.0 {
            if input.speed_abs >= param.speed_high_abs {
                1.0
            } else {
                0.0
            }
        } else {
            clamp((input.speed_abs - param.speed_low_abs) / span, 0.0, 1.0)
        };
        let delta = wrap_angle_minus_pi_to_pi(self.theta_bemf_rad - self.theta_hf_rad);
        self.theta_rad = wrap_angle_0_to_2pi(self.theta_hf_rad + self.bemf_weight * delta);
        self.theta_rad
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::PI;

    /// 浮点近似比较辅助。容差按被比较量的量纲选取：电压/电流用 `1e-5`～`1e-6`，
    /// 角度/相位用 `1e-5` `[rad]`，只为吸收 `f32` 的舍入差异。
    /// Approximate float comparison helper; the tolerance absorbs `f32` rounding
    /// only and is chosen per quantity.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// C 参考向量：固定轴注入的相位推进、注入电压幅值与同步解调结果。
    /// C reference vector for the stationary-axis injection: phase advance,
    /// injected voltage and the demodulated value.
    ///
    /// 断言的是与 C 参考实现逐值一致。改动 `update` 的运算顺序、`2π·f·ts` 的
    /// 累加方式、0.5 解调增益或角度环绕都会让本测试失配，因此这些形式不是可以
    /// 随意"优化"的实现细节。
    /// The assertion is numerical equivalence with the C reference; changing the
    /// operation order, the demodulation gain or the wrapping breaks it.
    #[test]
    fn hf_injection_matches_c_reference() {
        let mut param = HfInjectionParam {
            amplitude: 2.0,
            freq_hz: 25.0,
            ts: 0.01,
            axis_angle_rad: 0.0,
            demod_alpha: 1.0,
        };
        let mut state = HfInjectionState::default();
        let voltage = state.update(&param, 3.0);
        near(state.carrier_angle_rad, PI * 0.5, 1e-5);
        near(voltage.alpha, 2.0, 1e-5);
        near(voltage.beta, 0.0, 1e-5);
        near(state.demod_response, 3.0, 1e-5);
        param.axis_angle_rad = PI * 0.5;
        let voltage = state.update(&param, 1.0);
        near(state.carrier, 0.0, 1e-5);
        near(voltage.alpha, 0.0, 1e-5);
    }

    /// C 参考向量：旋转注入的电压矢量、两轴解调量与 `0.5·atan2` 取角结果。
    /// C reference vector for the rotating injection: voltage vector, per-axis
    /// demodulated values and the `0.5·atan2` angle extraction.
    ///
    /// 这里的期望值是 `π/4`——它验证的是"半角 + 解调量比值"这套运算本身，而不是
    /// 真实电机的角度精度；实机角度正确性还要靠独立角度真值验证。
    /// The expected `π/4` validates the half-angle arithmetic, not real-machine
    /// angle accuracy.
    #[test]
    fn rotating_hf_matches_c_reference() {
        let param = RotatingHfParam {
            amplitude: 3.0,
            freq_hz: 25.0,
            ts: 0.01,
            demod_alpha: 1.0,
        };
        let mut state = RotatingHfState::default();
        let voltage = state.update(
            &param,
            AlphaBeta {
                alpha: 4.0,
                beta: 2.0,
            },
        );
        near(state.carrier_angle_rad, PI * 0.5, 1e-5);
        near(voltage.alpha, 0.0, 1e-5);
        near(voltage.beta, 3.0, 1e-5);
        near(state.demod.alpha, 0.0, 1e-5);
        near(state.demod.beta, 2.0, 1e-5);
        near(state.theta_est_rad, PI * 0.25, 1e-5);
    }

    /// C 参考向量：脉冲注入的极性翻转顺序、响应记录与 `saliency` 差值。
    /// C reference vector for the pulse injection: polarity flip order, stored
    /// responses and the `saliency` difference.
    ///
    /// 测试确认了"复位后第一拍是负脉冲"这一符号约定；重排翻转顺序或把差值取反
    /// 都会失配。
    /// Confirms the sign convention that the first pulse after a reset is negative.
    #[test]
    fn pulse_injection_matches_c_reference() {
        let param = PulseInjectionParam {
            amplitude: 2.0,
            axis_angle_rad: 0.0,
        };
        let mut state = PulseInjectionState::default();
        let voltage = state.update(&param, 5.0);
        assert_eq!(state.polarity, -1);
        near(state.positive_response, 5.0, 1e-6);
        near(voltage.alpha, -2.0, 1e-6);
        let voltage = state.update(&param, 3.0);
        assert_eq!(state.polarity, 1);
        near(state.negative_response, 3.0, 1e-6);
        near(state.saliency, 2.0, 1e-6);
        near(voltage.alpha, 2.0, 1e-6);
    }

    /// C 参考向量：全速域融合的权重计算与角度插值端点。
    /// C reference vector for the full-speed fusion: weight computation and the
    /// interpolation endpoints.
    ///
    /// 测试只覆盖"低于下沿给纯 HF、高于上沿给纯 BEMF"两个端点与权重值，不覆盖
    /// 过渡带、反转、阈值写反（`span <= 0`）等工况；这些边界需要在实机或仿真中
    /// 另行验证。
    /// Only the two endpoints are covered; the transition band, reversal and the
    /// `span <= 0` degenerate case are not.
    #[test]
    fn hf_bemf_matches_c_reference() {
        let param = HfBemfParam {
            hf: RotatingHfParam {
                amplitude: 2.0,
                freq_hz: 100.0,
                ts: 0.001,
                demod_alpha: 1.0,
            },
            bemf: BemfParam {
                rs: 0.0,
                ls: 0.0,
                ts: 0.001,
                emf_filter_alpha: 1.0,
            },
            speed_low_abs: 10.0,
            speed_high_abs: 100.0,
        };
        let mut input = HfBemfInput {
            bemf: BemfInput {
                voltage: AlphaBeta {
                    alpha: -10.0,
                    beta: 0.0,
                },
                current: AlphaBeta::default(),
            },
            hf_current_response: AlphaBeta {
                alpha: 1.0,
                beta: 0.5,
            },
            speed_abs: 0.0,
        };
        let mut state = HfBemfState::default();
        let low = state.update(&param, &input);
        near(state.bemf_weight, 0.0, 1e-6);
        near(low, state.theta_hf_rad, 1e-6);
        assert!(state.hf_voltage.alpha.abs() > 0.0 || state.hf_voltage.beta.abs() > 0.0);
        input.speed_abs = 150.0;
        let high = state.update(&param, &input);
        near(state.bemf_weight, 1.0, 1e-6);
        near(high, state.theta_bemf_rad, 1e-6);
    }
}
