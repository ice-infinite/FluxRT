#![cfg_attr(target_os = "none", no_std)]

//! FluxRT —— C 固件与纯 Rust FOC 算法之间的唯一 C ABI 边界。
//! RT-Thread C firmware and the pure Rust FOC algorithms meet at this crate.
//!
//! 职责 / Responsibility:
//!   - 在 C 提供的静态存储中原位构造 `Controller`，不分配任何内存。
//!   - 承载一次快环的全部步骤：Clarke、观测器、Rev-Up 启动时序、电流环、
//!     可选速度环，最后把三相占空比交回 C。
//!   - 把 C 侧指针转成 Rust 引用，并把所有失败路径统一为 `FocStatus`。
//!
//! 不安全代码的范围 / Scope of unsafe code: 仅限对调用方所拥有的 C 存储做**带校验**
//! 的指针转换。
//! Unsafe code is restricted to checked conversion of caller-owned C storage.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge（本 crate，`#[no_mangle]`
//!   C ABI）-> foc-control -> foc-algorithm。依赖方向单向向下：本 crate 认识
//!   `foc-control`，反之不成立；`foc-control` 不知道 HAL、RTOS 或全局状态。
//!   The dependency direction is strictly downward: this crate knows
//!   `foc-control`, never the other way round.
//!
//! 实时约束 / Real-time constraints:
//!   `foc_rust_realtime_step()` 在 12 kHz 的 ADC 中断里执行，是整条路径上唯一
//!   的实时入口。热路径内禁止：动态分配、阻塞、日志、Mutex 等待、Panic 路径，
//!   以及需要软件浮点 `sinf`/`cosf` 的三角函数——目标板上 `sin/cos`、`atan2`
//!   走 CORDIC，模长走 FPU（见 `docs/硬件数学加速与CPU回退.md`）。
//!   `foc_rust_realtime_step()` runs inside the 12 kHz ADC ISR. The hot path must
//!   not allocate, block, log, wait on a mutex, panic, or fall back to software
//!   `sinf`/`cosf`; the target uses the hybrid CORDIC/FPU adapter instead.
//!
//! 量纲 / Units: 电流 `[A]`、电压 `[V]`、电角度 `[rad]`、机械转速 `[rpm]`（部分
//!   内部状态用 `[rad/s]`）、时间 `[s]`、占空比无量纲 `[0,1]`。结构体字段名带
//!   单位后缀，跨边界时以 `foc/include/foc_rust_bridge.h` 的注释为准。
//!   Field name suffixes carry the unit across the boundary.
//!
//! 定点与 Q 格式 / Fixed point and Q formats:
//!   本文件全程使用 `f32`，不存在 Q 格式。CORDIC 的定/浮边界位于 C 适配器
//!   （`foc_math_accel_*`），其内部 Q 格式由 C 侧决定，本侧只看到 `f32` 出参。
//!   Everything here is `f32`; the only fixed-point boundary is the CORDIC adapter,
//!   whose Q format stays on the C side.
//!
//! ABI 稳定性 / ABI stability:
//!   本文件的 `#[repr(C)]` 结构体字段顺序与函数签名**就是** ABI。任何字段、
//!   签名或语义改动都必须同时提升 `FOC_RUST_ABI_VERSION`、更新
//!   `foc/include/foc_rust_bridge.h`，并复核其中的 `_Static_assert` 尺寸断言；
//!   `main.c` 启动时会比对版本并拒绝不匹配的组合（这是设计意图）。
//!   Field order and signatures ARE the ABI: any change must bump
//!   `FOC_RUST_ABI_VERSION`, update `foc/include/foc_rust_bridge.h`, and re-check
//!   its `_Static_assert` sizes, or the boot self-check refuses to run.
//!
//! 并发模型 / Concurrency model:
//!   不做锁。C 的 ADC 中断是可变状态的唯一所有者；管理线程若要读遥测只能调用
//!   `foc_rust_get_telemetry()`，而且**互斥必须由调用方提供**：该函数内部没有临界区，
//!   只是逐字段拷贝 `FocTelemetry`，与中断并发时会读到跨拍的混合值。同一上下文不得
//!   被线程与中断并发修改，且必须先 `foc_rust_init()` 再调用其他任何函数。
//!   No locks: the ADC ISR owns mutable state. A management thread may read telemetry
//!   only through `foc_rust_get_telemetry()`, and the CALLER must supply the mutual
//!   exclusion: that function has no critical section and copies `FocTelemetry` field
//!   by field, so it can observe a torn, cross-sample mixture if it races the ISR.
//!   One context must never be mutated concurrently by a thread and an ISR.
//!
//! 以本文件为准 / This file is authoritative: `foc/include/foc_rust_bridge.h` 里
//!   "内部短临界区"的说法目前**未在本层实现**，需要一致快照时请由 C 侧临界区包裹。
//!   The header's "internal short critical section" wording is NOT implemented here;
//!   wrap the call in a C-side critical section when a consistent snapshot is needed.
//!
//! 参考 / Reference: `docs/C与Rust混合架构.md`, `docs/架构与安全边界.md`,
//! `docs/无感闭环接管.md`, `foc/include/foc_rust_bridge.h`

use core::f32::consts::PI;
use core::mem::{align_of, size_of};
use core::ptr;
use foc_algorithm::{
    clarke, svpwm_update, wrap_angle_0_to_2pi, Abc, AlphaBeta, PiParam, SvpwmParam,
};
#[cfg(any(feature = "fast-math-benchmark", feature = "fast-math-candidate"))]
use foc_control::FastApproxMath;
use foc_control::{
    st_gbm2804_reference_parameters, ConfigurableObserver, ControlAngleOffsets, ControlMath,
    ControlParameters, CpuMath, CurrentCommand, CurrentLoop, FeedbackSnapshot,
    InverterVoltageModel, InverterVoltageModelConfig, ObserverBackend, ObserverReliabilityConfig,
    ObserverVoltageSource, PhaseCurrents, PwmCommand, RevUpConfig, RevUpPhase, RevUpSequencer,
    RotorEstimator, RotorFeedback, SmoPllTuning, SpeedCommand, SpeedLoop,
};

/// ABI 版本：主版本占高 16 位，`0x000E_0000` 表示第 14 代；必须与 C 侧宏逐位
/// 一致，否则 `main.c` 启动自检会拒绝运行。
/// ABI version packed as 16-bit halves; must match the C macro bit for bit.
pub const FOC_RUST_ABI_VERSION: u32 = 0x0011_0000;
/// `FocRuntimeConfig` 自身的版本，与 ABI 版本独立演进；C 侧填错会被直接拒绝。
/// Version of `FocRuntimeConfig`; it evolves independently of the ABI version.
pub const FOC_RUST_CONFIG_VERSION: u32 = 11;
/// C 提供的控制器存储容量 `[bytes]`；编译期断言保证 `Controller` 装得下并留有余量。
/// Controller storage capacity supplied by C [bytes]; a compile-time assertion
/// below proves `Controller` fits with headroom.
pub const FOC_RUST_CONTEXT_CAPACITY: usize = 2048;
/// 算法算出的占空比非有限或超出 `[0,1]`；已停止输出。
/// Algorithm output was non-finite or outside `[0,1]`; output stopped.
pub const FOC_FAULT_ALGORITHM_OUTPUT: u32 = 1 << 0;
/// 反馈含非有限值或母线电压 `[V]` 不为正；本拍被拒绝。
/// Feedback was non-finite or the bus voltage was not positive.
pub const FOC_FAULT_INVALID_FEEDBACK: u32 = 1 << 1;
/// 启动期在 `observer_acquisition_timeout_s` 内观测器始终未过可靠性门。
/// The observer never passed its reliability gate during acquisition.
pub const FOC_FAULT_OBSERVER_STARTUP: u32 = 1 << 2;
/// 观测器已接管角度后，连续失锁超过 `observer_loss_timeout_s`。
/// The observer stayed unlocked longer than `observer_loss_timeout_s`.
pub const FOC_FAULT_OBSERVER_LOST: u32 = 1 << 3;

/// `Controller` 的哨兵值，取自 "FOCR"。C 在 `foc_rust_init()` 之前把存储清零，
/// 于是"未初始化/被踩坏"与"已初始化"可区分：`magic` 不匹配时所有入口都返回
/// `InvalidArgument`，绝不把调用方内存当作 `Controller` 解释。
/// Sentinel marking an initialized `Controller`; zeroed C storage never matches,
/// so an uninitialized or corrupted context is rejected instead of reinterpreted.
/// It is not a substitute for the single-caller discipline: a half-written
/// context is still undefined behaviour, which is why C must call
/// `foc_rust_init()` before anything else.
const CONTEXT_MAGIC: u32 = 0x464F_4352; // "FOCR"

/// 快环使用的 `ControlMath` 实现：默认目标板优先走 CORDIC/FPU，显式候选可改用
/// CPU 快速近似；主机默认走软件基线。
/// The fast-loop `ControlMath`: CORDIC/FPU by default on target, with an explicit
/// CPU fast-approximation candidate; the host keeps the software reference.
///
/// `cargo clippy` 带 `--all-features` 时 CORDIC 分支不会参与编译（它同时要求
/// `target_os = "none"`），因此主机测试永远走 `CpuMath`。反过来，目标固件启用
/// `stm32g4-cordic` 特性后，若 C 侧没有提供 `foc_math_accel_*` 符号，问题会在
/// 链接阶段暴露，而不是运行期。
/// With `--all-features` clippy the CORDIC branch is still excluded because it
/// also requires `target_os = "none"`, so host tests always use `CpuMath`.
///
/// 本结构体每拍在栈上新建（`PlatformMath::default()`），不含任何跨拍状态；这些
/// 适配器是无状态的，所以"CORDIC 不可用"可以逐次调用地回退。
/// It is rebuilt on the stack every tick and holds no cross-tick state, which is
/// what makes the per-call software fallback safe.
#[derive(Default)]
struct PlatformMath {
    cpu: CpuMath,
    #[cfg(feature = "fast-math-candidate")]
    fast: FastApproxMath,
}

impl ControlMath for PlatformMath {
    /// 返回 `(sin, cos)`，输入电角度 `[rad]`；CORDIC 不可用或结果非有限时静默回退。
    /// Returns `(sin, cos)` for an angle in `[rad]`, silently falling back to
    /// software math when CORDIC is unavailable or produced non-finite values.
    fn sin_cos(&mut self, angle_rad: f32) -> (f32, f32) {
        #[cfg(feature = "fast-math-candidate")]
        {
            let result = self.fast.sin_cos(angle_rad);
            if result.0.is_finite() && result.1.is_finite() {
                return result;
            }
        }
        #[cfg(all(feature = "stm32g4-cordic", target_os = "none"))]
        {
            let mut sin = 0.0;
            let mut cos = 0.0;
            // SAFETY: the C adapter writes only the two valid stack outputs. It
            // returns zero when CORDIC is unavailable, which selects CpuMath.
            if unsafe { foc_math_accel_sin_cos(angle_rad, &mut sin, &mut cos) } != 0
                && sin.is_finite()
                && cos.is_finite()
            {
                return (sin, cos);
            }
        }
        self.cpu.sin_cos(angle_rad)
    }

    /// 返回 `sqrt(x^2 + y^2)`，与输入同量纲（通常是电压 `[V]` 或电流 `[A]`）。
    /// Returns `sqrt(x^2 + y^2)` in the same unit as the inputs.
    ///
    /// 这里额外要求结果 `>= 0`：C 侧若返回负值会导致反电动势幅值门限判断反向，
    /// 观测器会在零速噪声上被判"可靠"。语法上这个条件恒真，保留它是为了把
    /// "适配器返回值不可信"这一契约写在代码里。
    /// The extra `>= 0` check documents that adapter output is untrusted; a
    /// negative magnitude would invert the BEMF gate comparison.
    fn magnitude(&mut self, x: f32, y: f32) -> f32 {
        #[cfg(all(feature = "stm32g4-cordic", target_os = "none"))]
        {
            let mut magnitude = 0.0;
            // SAFETY: the C adapter writes only the valid stack output and
            // reports failure instead of returning an unchecked value.
            if unsafe { foc_math_accel_magnitude(x, y, &mut magnitude) } != 0
                && magnitude.is_finite()
                && magnitude >= 0.0
            {
                return magnitude;
            }
        }
        self.cpu.magnitude(x, y)
    }

    /// 返回 `atan2(y, x)` 并按 `[0, 2π)` 归一，参数顺序是 `(y, x)`。
    /// Returns `atan2(y, x)` wrapped into `[0, 2π)`; note the `(y, x)` order.
    ///
    /// 角度单位 `[rad]`。交换 x/y 不会报错但会让观测器角度整体旋转 90°，表现为
    /// 转矩方向错误——这类错误在实机上只能靠示波器或已知转向发现。
    /// Swapping the arguments is silent but rotates the estimated angle by 90°.
    fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32 {
        #[cfg(feature = "fast-math-candidate")]
        {
            let angle = self.fast.angle_0_to_2pi(y, x);
            if angle.is_finite() {
                return angle;
            }
        }
        #[cfg(all(feature = "stm32g4-cordic", target_os = "none"))]
        {
            let mut angle = 0.0;
            // SAFETY: the C adapter writes one valid stack output and returns
            // zero when CORDIC is unavailable.
            if unsafe { foc_math_accel_atan2(y, x, &mut angle) } != 0 && angle.is_finite() {
                return angle;
            }
        }
        self.cpu.angle_0_to_2pi(y, x)
    }
}

// CORDIC/FPU 适配器，由 C 平台层实现（见 `docs/硬件数学加速与CPU回退.md`）。
// CORDIC/FPU adapters implemented by the C platform layer.
//
// 调用契约（对每个 `unsafe` 调用点都成立）/ Caller contract for every call:
//   返回非 0 表示"出参已写入且可信"；返回 0 表示不可用，调用方必须回退。
//   出参指针必须是有效、对齐、独占的可写 `f32` 栈位置；本文件只传 `&mut` 局部
//   变量，因此天然满足，不存在悬垂或别名。
//   这些适配器不得阻塞、不得分配、不得回调 Rust：它们在 ADC ISR 上下文被调用。
//   A non-zero return means the out-parameter is valid; the pointers must be
//   valid, aligned, exclusively writable `f32` stack slots; and the adapters must
//   not block, allocate or call back into Rust because they run in the ISR.
#[cfg(all(feature = "stm32g4-cordic", target_os = "none"))]
extern "C" {
    fn foc_math_accel_sin_cos(angle_rad: f32, sin_out: *mut f32, cos_out: *mut f32) -> u32;
    fn foc_math_accel_magnitude(x: f32, y: f32, magnitude_out: *mut f32) -> u32;
    fn foc_math_accel_atan2(y: f32, x: f32, angle_out: *mut f32) -> u32;
}

/// Diagnostic-only entry used by the stopped-state C benchmark to measure the
/// portable fast `sin/cos` implementation in the same firmware image as CORDIC.
///
/// # Safety
/// `sin_out` and `cos_out` must be valid, aligned, independently writable `f32`
/// pointers. They are cleared before any invalid-input return.
#[cfg(feature = "fast-math-benchmark")]
#[no_mangle]
pub unsafe extern "C" fn foc_rust_fast_math_sin_cos(
    angle_rad: f32,
    sin_out: *mut f32,
    cos_out: *mut f32,
) -> u32 {
    if sin_out.is_null() || cos_out.is_null() {
        return 0;
    }
    // SAFETY: pointer validity and exclusive writability are the caller's ABI
    // contract, documented above. Both outputs are initialised on every path.
    unsafe {
        ptr::write(sin_out, 0.0);
        ptr::write(cos_out, 0.0);
    }
    if !angle_rad.is_finite() {
        return 0;
    }
    let mut math = FastApproxMath;
    let (sin, cos) = math.sin_cos(angle_rad);
    if !sin.is_finite() || !cos.is_finite() {
        return 0;
    }
    unsafe {
        ptr::write(sin_out, sin);
        ptr::write(cos_out, cos);
    }
    1
}

/// Diagnostic-only entry used by the stopped-state C benchmark to measure the
/// portable fast `atan2` implementation beside CORDIC.
///
/// # Safety
/// `angle_out` must be a valid, aligned, exclusively writable `f32` pointer. It
/// is cleared before any invalid-input return.
#[cfg(feature = "fast-math-benchmark")]
#[no_mangle]
pub unsafe extern "C" fn foc_rust_fast_math_atan2(y: f32, x: f32, angle_out: *mut f32) -> u32 {
    if angle_out.is_null() {
        return 0;
    }
    // SAFETY: pointer validity and exclusive writability are the caller's ABI
    // contract, documented above. The output is initialised on every path.
    unsafe { ptr::write(angle_out, 0.0) };
    if !x.is_finite() || !y.is_finite() {
        return 0;
    }
    let mut math = FastApproxMath;
    let angle = math.angle_0_to_2pi(y, x);
    if !angle.is_finite() {
        return 0;
    }
    unsafe { ptr::write(angle_out, angle) };
    1
}

/// 跨边界的统一状态码，值为固定 `u32`（禁止 C 短枚举），数值本身属于 ABI。
/// The single status code crossing the boundary as a fixed `u32`; the values are
/// part of the ABI.
///
/// 失败语义 / Failure semantics: `foc_rust_realtime_step()` 在任何错误返回之前
/// 都已把 `output` 清零，调用方不得沿用上一周期的占空比；但本 crate **不接触
/// 硬件**，栅极关断必须由 C 平台层完成。
/// Every error return from the realtime step has already zeroed `output`, but gate
/// shutdown itself is the C platform layer's job.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FocStatus {
    /// 调用完成，出参可信。
    /// The call completed and its outputs are valid.
    Ok = 0,
    /// 尚未进入运行态（含 `FocState::Disabled` 与配置前的状态）。
    /// Not in a runnable state yet.
    Disabled = 1,
    /// 算法参数或平台就绪标志缺失，拒绝启动。
    /// Algorithm parameters or the platform-ready flag are missing.
    NotConfigured = 2,
    /// 入参非法：空指针、非有限值、越界或未初始化上下文，控制器状态不变。
    /// Invalid input; controller state is left unchanged.
    InvalidArgument = 3,
    /// 已进入故障态并锁存故障位，调用方必须停止 PWM 并处理硬件。
    /// Entered the fault state with latched fault bits.
    HardwareFault = 4,
}

/// 控制器逻辑状态机。数值属于 ABI，只能追加不能重排。
/// The controller state machine. The values are part of the ABI; only append.
///
/// 正常路径 / Normal path:
///   启动链由 `RevUpSequencer` 的相位驱动，`foc_rust_realtime_step()` 每次把它
///   映射到本枚举：`Alignment` -> `OpenLoopRamp` -> `OpenLoopHold` ->
///   `ObserverTransition` -> `ClosedLoop`。每个状态都对应一个真实的物理阶段，
///   不是装饰：`Alignment` 期间转子被强制对齐（电流按秒线性建立），
///   `OpenLoopRamp` 是开环升速，`OpenLoopHold` 是"升速到位后等观测器收敛"的
///   驻留段，`ObserverTransition` 用最短路径把强制角平滑过渡到观测角，
///   `ClosedLoop` 才把角度所有权交给观测器。
///   `foc_rust_start_realtime()` 进入 `Alignment`；兼容入口
///   `foc_rust_request_start()` 直接进入 `Running`（`fast_step`/`open_loop_step`/
///   `speed_step` 只认这一个状态），两者不混用。
///
/// 故障路径 / Fault path: 任一故障（`InvalidFeedback`、观测器启动超时、观测器
/// 失锁、算法输出非法）都直接切到 `Fault` 并锁存 `fault_flags`。`Fault` 是**粘
/// 滞**的：只有 `foc_rust_clear_fault()` 能离开，且它不检查硬件是否真的恢复，
/// 所以 C 必须先确认硬件已安全。
/// `Fault` is sticky: only `foc_rust_clear_fault()` leaves it, and that call does
/// not verify the hardware, so C must do so first.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FocState {
    /// 未初始化：空指针或 `CONTEXT_MAGIC` 不匹配时由查询入口返回，不写入状态。
    /// Not initialized: returned by the query entry points for a null or
    /// magic-mismatched context.
    Uninitialized = 0,
    /// 停机且已复位全部算法状态；只有此状态允许 `foc_rust_configure()`。
    /// Stopped with all algorithm state reset; only this state accepts
    /// `foc_rust_configure()`.
    Disabled = 1,
    /// 兼容入口 `foc_rust_request_start()` 使用的遗留运行态，不含启动时序与观测器。
    /// Legacy run state used by the compatibility `foc_rust_request_start()` path.
    Running = 2,
    /// 强制对齐：电角度固定，电流按 `alignment_s` 线性建立。
    /// Forced alignment: fixed electrical angle, current ramps over `alignment_s`.
    Alignment = 3,
    /// 开环升速：电角度按设定转速积分，电流保持在升速电流。
    /// Open-loop ramp: the electrical angle integrates the commanded speed.
    OpenLoopRamp = 4,
    /// 开环保持：转速已达升速终点，等待观测器过可靠性门。
    /// Open-loop hold: at the rev-up endpoint, waiting for the observer gate.
    OpenLoopHold = 5,
    /// 交接段：角度在强制角与观测角之间按最短路径混合。
    /// Handover: the angle blends from the forced angle to the observer angle.
    ObserverTransition = 6,
    /// 闭环：角度由观测器提供，速度环接管 Iq 给定。
    /// Closed loop: the observer owns the angle and the speed loop owns Iq.
    ClosedLoop = 7,
    /// 故障：输出已清零、故障位已锁存，只能由 `foc_rust_clear_fault()` 退出。
    /// Fault: output zeroed, fault bits latched, sticky until cleared.
    Fault = 8,
}

/// 一拍 ADC 采样得到的反馈快照，量纲见字段名后缀。
/// One ADC feedback snapshot; the field-name suffixes carry the units.
///
/// 三相电流必须来自**同一采样时刻**，且电流极性约定为 `(offset - raw)`；相位
/// 或极性写反会让 Id/Iq 同时变号，电流环看起来仍然"稳定"，但转矩方向相反。
/// All three currents must come from the same sampling instant with the
/// `(offset - raw)` polarity convention, or torque direction inverts.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocFeedback {
    /// A 相电流 `[A]` / Phase A current `[A]`.
    pub phase_current_a: f32,
    /// B 相电流 `[A]` / Phase B current `[A]`.
    pub phase_current_b: f32,
    /// C 相电流 `[A]` / Phase C current `[A]`.
    pub phase_current_c: f32,
    /// 母线电压 `[V]`，必须为正；`<= 0` 或非有限会被判为非法反馈并锁存故障。
    /// DC bus voltage `[V]`; non-positive or non-finite values are rejected.
    pub dc_bus_voltage: f32,
    /// 本次 Park/逆 Park 使用的电角度 `[rad]`，不是机械角度。
    /// Electrical angle used by the Park transforms `[rad]`, not mechanical.
    pub electrical_angle_rad: f32,
}

/// 电流环的 dq 给定 `[A]`，由 `foc_rust_fast_step()` 逐拍传入。
/// dq current references `[A]` fed per tick into `foc_rust_fast_step()`.
///
/// `id_ref` 是励磁分量，本工程通常取 0（表贴式电机）；`iq_ref` 是转矩分量，其
/// 幅值受电流环 `out_min`/`out_max` 限制，越界由 PI 内部限幅而不是这里兜底。
/// `id_ref` is the magnetising component (normally 0 here); `iq_ref` is the torque
/// component, clamped inside the PI rather than by this struct.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocReference {
    /// d 轴（励磁）电流给定 `[A]` / d-axis current reference `[A]`.
    pub id_ref: f32,
    /// q 轴（转矩）电流给定 `[A]` / q-axis current reference `[A]`.
    pub iq_ref: f32,
}

/// 三相占空比，无量纲，有效范围 `[0,1]`（0 = 全下管，1 = 全上管）。
/// Three-phase duty cycle, dimensionless, valid range `[0,1]`.
///
/// 这里**不含**死区、最小脉宽或栅极使能处理：那些由 C 平台层在写 TIM1 比较寄存
/// 器时完成（见 `foc/platform/stm32g431/foc_platform_stm32g431.c`）。本结构体的
/// 值已被算法侧检查过有限性与范围，C 侧不得把 `0.0/1.0` 直接当作 0%/100% 脉宽
/// 使用而不加占空比窗口限幅。
/// Dead time, minimum pulse width and gate enabling are the C platform's job; this
/// struct carries only the checked algorithm output.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocOutput {
    /// A 相占空比 `[--]` / Phase A duty.
    pub duty_a: f32,
    /// B 相占空比 `[--]` / Phase B duty.
    pub duty_b: f32,
    /// C 相占空比 `[--]` / Phase C duty.
    pub duty_c: f32,
}

/// 主机仿真覆盖逆变器补偿的便捷入口参数，**不属于 C ABI**。
/// Host-side installation contract for the shared inverter model.
///
/// 补偿公式和滤波状态已迁到可同时供 Host/Cortex-M 编译的
/// `foc-control::InverterVoltageModel`；这个结构体只保留 Host 参数覆盖入口。
/// 目标固件走版本化的 `FocRuntimeConfig::inverter_voltage_model`，不调用此接口。
///
/// 量纲 / Units: `dead_time_s` 与 `pwm_period_s` `[s]`，`current_zero_band_a` `[A]`，
/// `feedforward_gain` 无量纲比例 `[--]`。
/// 物理背景 / Physics: 死区期间相电流由续流二极管决定，实际相电压相对指令会偏
/// 移约 `sign(i) * dead_time / T_pwm` 的占空比；电流过零附近的符号不可辨，所以
/// 用线性过渡带 `current_zero_band_a` 而不是硬 `sign()`，否则零附近会抖动。
/// Near zero current the polarity is unknowable, hence the linear band instead of
/// a hard `sign()`, which would chatter.
#[cfg(not(target_os = "none"))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HostDeadTimeCompensation {
    /// 逆变器死区时间 `[s]`；范围 `0..=0.01`（10 ms 已远超任何正常死区）。
    /// Inverter dead time `[s]`, validated to `0..=0.01`.
    pub dead_time_s: f32,
    /// PWM 载波周期 `[s]`；与控制 ISR 周期分开，24/12 kHz 多速率时取 `1/24000`。
    pub pwm_period_s: f32,
    /// 电流极性线性过渡带 `[A]`；0 表示退化为硬 `sign()`。
    /// Current-polarity transition band `[A]`; 0 degenerates to a hard sign.
    pub current_zero_band_a: f32,
    /// 电流极性滤波系数 `(0,1]`；1.0 保持旧 Host 路径的逐拍原值。
    pub current_sign_filter_alpha: f32,
    /// 等效单相器件导通压降 `[V]`；0.0 保持旧死区模型。
    pub device_drop_v: f32,
    /// 前馈补偿增益，无量纲比例 `[--]`；1.0 表示补偿全部死区占空比损失。
    /// Feed-forward gain `[--]`; 1.0 compensates the full dead-time duty loss.
    pub feedforward_gain: f32,
    /// 是否向 PWM 叠加前馈补偿。
    /// Whether to add the feed-forward correction to the PWM command.
    pub feedforward_enabled: bool,
    /// 是否在观测器电压重构里扣除死区损失（只影响观测器，不影响实际输出）。
    /// Whether to subtract the dead-time loss in the observer voltage
    /// reconstruction only.
    pub observer_voltage_correction_enabled: bool,
}

#[cfg(not(target_os = "none"))]
impl Default for HostDeadTimeCompensation {
    fn default() -> Self {
        Self {
            dead_time_s: 0.0,
            pwm_period_s: 1.0 / 12_000.0,
            current_zero_band_a: 0.0,
            current_sign_filter_alpha: 1.0,
            device_drop_v: 0.0,
            feedforward_gain: 0.0,
            feedforward_enabled: false,
            observer_voltage_correction_enabled: false,
        }
    }
}

#[cfg(not(target_os = "none"))]
impl HostDeadTimeCompensation {
    /// 校验补偿参数；任一项非有限或越界即整组拒绝，避免部分生效造成不对称补偿。
    /// Validates the whole group; a partial application would produce asymmetric
    /// compensation, so one bad field rejects everything.
    fn is_valid(self) -> bool {
        self.model_config().is_valid()
    }

    /// 映射到可在目标端编译的公共模型，不产生 C ABI 变化。
    fn model_config(self) -> InverterVoltageModelConfig {
        InverterVoltageModelConfig {
            enabled: self.feedforward_enabled || self.observer_voltage_correction_enabled,
            dead_time_s: self.dead_time_s,
            pwm_period_s: self.pwm_period_s,
            compensation_gain: self.feedforward_gain,
            current_zero_band_a: self.current_zero_band_a,
            current_sign_filter_alpha: self.current_sign_filter_alpha,
            device_drop_v: self.device_drop_v,
            observer_voltage_correction_enabled: self.observer_voltage_correction_enabled,
            feedforward_enabled: self.feedforward_enabled,
            source: ObserverVoltageSource::CommandModel,
        }
    }
}

/// 单个 PI 调节器的配置，直接映射到 `foc_algorithm::PiParam`。
/// One PI regulator configuration, mapped field-by-field onto
/// `foc_algorithm::PiParam`.
///
/// `ts` 是**该环**的采样周期 `[s]`：电流环用 `1/pwm_frequency_hz`，速度环用
/// `1/speed_loop_frequency_hz`，两者不同，填错会让积分增益的物理含义整体改变
/// （典型表现是电流环响应"看起来正常"但速度环振荡）。
/// `ts` is the per-loop sample period in `[s]`; the current and speed loops use
/// different values, and swapping them silently rescales every integral gain.
///
/// 限幅语义 / Limits: `out_min`/`out_max` 是输出饱和（电流环为 `[V]`，速度环为
/// `[A]`），`integrator_min`/`integrator_max` 是积分器抗饱和钳位，两者相互独立；
/// 只改输出限幅而不改积分器钳位会让退饱和变慢。
/// Output saturation and integrator anti-windup clamps are independent.
///
/// `speed_pi_preload_ratio` 出现在 `FocRuntimeConfig` 而不是这里，见那边的说明。
/// `speed_pi_preload_ratio` lives in `FocRuntimeConfig`, not here.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocPiConfig {
    /// 比例增益；电流环 `[V/A]`，速度环 `[A*s/rad]`；`[ST]` 转写自 MCSDK 6.4.1。
    /// Proportional gain; `[V/A]` for the current loop, `[A*s/rad]` for the speed
    /// loop. `[ST]` transcribed from MCSDK 6.4.1.
    pub kp: f32,
    /// 积分增益（连续时间形式）；`ki * ts` 是每个采样周期的增量。
    /// Integral gain in continuous-time form; `ki * ts` is the per-sample step.
    pub ki: f32,
    /// 本环采样周期 `[s]`；必须 > 0 且非有限值会被拒绝。
    /// Sample period of this loop `[s]`; must be finite and positive.
    pub ts: f32,
    /// 输出下限 / Output lower saturation limit.
    pub out_min: f32,
    /// 输出上限 / Output upper saturation limit.
    pub out_max: f32,
    /// 积分器下限（抗饱和）/ Integrator lower anti-windup clamp.
    pub integrator_min: f32,
    /// 积分器上限（抗饱和）/ Integrator upper anti-windup clamp.
    pub integrator_max: f32,
}

/// 观测器接管后的运行期保持门。V17 已把启动获取相位门拆成顶层独立字段，
/// 使运行纹波容差不再隐式放宽接管资格。
/// Observer retention settings after handover. V17 separates the acquisition
/// phase gate so runtime ripple tolerance cannot relax handover.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocObserverRunReliabilityConfig {
    /// 接管后允许的最低估计转速 `[rpm]`，可低于但不能高于启动接管门限。
    /// Minimum estimated speed after handover; it may not exceed the acquisition
    /// threshold. The ST reference maps this to `MIN_APPLICATION_SPEED_RPM`.
    pub minimum_speed_rpm: f32,
    /// 接管后保持允许的最大原始包角相位误差 `[rad]`，范围 `(0, pi/2]`。
    /// Maximum raw wrapped phase error retained after handover.
    pub maximum_phase_error_rad: f32,
}

/// Park/逆 Park 的执行延迟补偿，以“当前控制周期数”为单位。
/// Park/inverse-Park delay compensation in current-control ticks.
///
/// 两个值都是相对基础控制角的**绝对预测拍数**，不是 MCSDK 那种第二项累加在
/// 第一项之后的系数。合法范围是 `-2.0..=2.0`；0/0 与实际 ST 参考工程的默认值
/// 一致，并保持原控制律不变。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocAngleCompensationConfig {
    /// 电流 Park 角预测量 `[control ticks]`。
    /// Current-Park angle prediction `[control ticks]`.
    pub park_prediction_ticks: f32,
    /// 电压逆 Park 角预测量 `[control ticks]`，相对基础角独立定义。
    /// Voltage inverse-Park prediction `[control ticks]`, absolute from base angle.
    pub reverse_park_prediction_ticks: f32,
}

/// 目标端逆变器平均电压模型的版本化配置。
/// Versioned target-side average inverter-voltage model configuration.
///
/// 三层开关刻意分开：`enabled` 是总门，另外两个开关分别控制观测器电压修正与
/// 实际 PWM 前馈。子开关为 1 而总门为 0 的组合会被整组拒绝。载波频率单独保存，
/// 不能借用 `FocRuntimeConfig::pwm_frequency_hz`，因为后者历史上表示控制 ISR 频率，
/// 在 24/12 kHz 多速率配置中两者并不相等。
/// The master gate and the two consumers are independent. The carrier frequency
/// is explicit because the historical `pwm_frequency_hz` field is the control
/// rate and differs from the carrier in a multi-rate build.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocInverterVoltageModelConfig {
    /// 模型总门（0/1）/ Master model gate (0/1).
    pub enabled: u32,
    /// 修正观测器 CommandModel 电压（0/1）/ Correct observer voltage (0/1).
    pub observer_voltage_correction_enable: u32,
    /// 向实际 PWM 叠加前馈（0/1）/ Add feed-forward to issued PWM (0/1).
    pub pwm_feedforward_enable: u32,
    /// 实际 PWM 载波频率 `[Hz]`，不是控制 ISR 频率。
    /// Applied PWM carrier frequency `[Hz]`, not the control ISR rate.
    pub pwm_carrier_frequency_hz: u32,
    /// 单次换流死区 `[s]` / Per-commutation dead time `[s]`.
    pub dead_time_s: f32,
    /// PWM 前馈增益 `[--]`；观测器修正始终使用全量物理损失。
    /// PWM feed-forward gain; observer correction uses the full physical loss.
    pub compensation_gain: f32,
    /// 电流极性软零带 `[A]` / Current-polarity soft zero band `[A]`.
    pub current_zero_band_a: f32,
    /// 电流极性一阶滤波系数 `(0,1]` / Current-polarity filter alpha `(0,1]`.
    pub current_sign_filter_alpha: f32,
    /// 等效单相器件导通压降 `[V]` / Equivalent per-phase device drop `[V]`.
    pub device_drop_v: f32,
}

/// 兼容入口 `foc_rust_configure_basic()` 使用的最小配置。
/// Minimal configuration used by the compatibility
/// `foc_rust_configure_basic()` entry point.
///
/// 这是早期主机测试与独立调用者的旧接口：它只覆盖两个电流环 PI 与母线电压，
/// 其余参数全部回落到 ST 参考默认值。**实时路径不使用它**——真实路径是
/// `foc_rust_configure()` + `foc_rust_start_realtime()` +
/// `foc_rust_realtime_step()`。
/// This is the legacy interface for early host tests; the realtime path uses
/// `foc_rust_configure()` instead.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocBasicConfig {
    /// d 轴电流环 / d-axis current loop.
    pub id_pi: FocPiConfig,
    /// q 轴电流环 / q-axis current loop.
    pub iq_pi: FocPiConfig,
    /// 标称母线电压 `[V]`，必须为正有限值；用于初始化观测器与调制限幅。
    /// Nominal DC bus voltage `[V]`, must be finite and positive.
    pub nominal_dc_bus_voltage: f32,
}

/// 完整运行时配置：C 侧唯一需要理解的"业务级"结构体，字段顺序即 ABI 顺序。
/// Full runtime configuration; the only "business level" struct C must know.
///
/// 前两个字段是自检字段：`struct_size` 必须等于 `sizeof(FocRuntimeConfig)`，
/// `config_version` 必须等于 `FOC_RUST_CONFIG_VERSION`；不匹配直接拒绝，避免把
/// 旧结构体的字节按新布局解释。**任何字段增删改都必须提升 ABI 版本并同步修改
/// `foc/include/foc_rust_bridge.h`**，否则 C 侧 `_Static_assert` 或启动自检会失败。
/// The first two fields are self-check fields; any field change must bump the ABI
/// version and update the C header.
///
/// 参数来源标注 / Provenance of the defaults:
///   `[ST]` 转写自 MCSDK 6.4.1 参考工程；`[FW]` 本工程固件默认值；`[HW]` 本套件
///   实测。Workbench 继承的电机参数**未经实物辨识**，只能当起点而不是整定结果。
///   `[ST]` from the MCSDK 6.4.1 reference project, `[FW]` firmware defaults, `[HW]`
///   measured on this kit. Workbench-inherited motor values are NOT identified on
///   the physical machine and are starting points only.
///
/// 校验 / Validation: 全部字段由 `runtime_config_is_valid()` 检查（有限性、区间、
/// 频率整除关系、枚举取值、启动时序自洽），失败时 `foc_rust_configure()` 返回
/// `InvalidArgument` 且控制器状态完全不变。
/// Everything is checked by `runtime_config_is_valid()`; a failure leaves the
/// controller untouched.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocRuntimeConfig {
    /// 结构体尺寸自检 `[bytes]`，必须等于 `size_of::<FocRuntimeConfig>()`。
    /// Struct size self-check `[bytes]`.
    pub struct_size: u32,
    /// 配置版本自检，必须等于 `FOC_RUST_CONFIG_VERSION`。
    /// Config version self-check.
    pub config_version: u32,
    /// 观测器后端；取值见 `ObserverBackend::from_raw`（0 = SMO+PLL，1 = 浮点
    /// BEMF+PLL，2 = ST 定点 STO-PLL 预留）。未知取值会被拒绝。
    /// Observer backend; unknown values are rejected.
    pub observer_backend: u32,
    /// 是否每拍更新观测器（0/1）。关掉后纯开环，观测器遥测也不再刷新。
    /// Whether to update the observer every tick (0/1).
    pub observer_enable: u32,
    /// 是否允许观测器接管角度（0/1）。上电默认 0：首次上板必须先在开环下确认
    /// 观测器遥测通过交接门限，再打开闭环。
    /// Whether the observer may take over the angle (0/1); defaults to 0 so that
    /// first bring-up stays current-controlled open loop.
    pub closed_loop_enable: u32,
    /// 观测器分频：1 = 每拍更新，N > 1 = 每 N 拍更新一次。
    /// Observer divider: 1 = every tick, N > 1 = one update per N ticks.
    ///
    /// 多率路径只影响观测器的**执行频率**，观测器内部 `ts` 仍按单拍周期传入，
    /// 见 `configure_runtime()`；分频 > 1 时占空比会在观测器时隙被"保持"，这一点
    /// 写在 `foc_rust_realtime_step()` 的相应分支里。
    /// The divider only changes how often the observer runs; its internal `ts`
    /// stays at one PWM period.
    pub observer_update_divider: u32,
    /// d 轴电流环 / d-axis current loop.
    pub id_pi: FocPiConfig,
    /// q 轴电流环 / q-axis current loop.
    pub iq_pi: FocPiConfig,
    /// 速度环，执行频率由 `speed_loop_frequency_hz` 决定 `[A]` 输出。
    /// Speed loop, running at `speed_loop_frequency_hz`.
    pub speed_pi: FocPiConfig,
    /// 极对数 `[--]`，决定电角度与机械转速的换算；范围 `1..=32`。
    /// Pole pairs; validated to `1..=32`.
    pub pole_pairs: u32,
    /// 电流环执行频率 `[Hz]`，必须与平台 ISR 频率一致，且是速度环频率的整数倍。
    /// Current-loop rate `[Hz]`; must match the ISR and be an integer multiple of
    /// the speed-loop rate.
    pub pwm_frequency_hz: u32,
    /// 速度环执行频率 `[Hz]`，范围 `10..=pwm_frequency_hz`。
    /// Speed-loop rate `[Hz]`.
    pub speed_loop_frequency_hz: u32,
    /// 定子电阻 `[ohm]` `[ST]`，**未在本电机上辨识**。
    /// Stator resistance `[ohm]` `[ST]`, not identified on this motor.
    pub stator_resistance_ohm: f32,
    /// 定子电感 `[H]` `[ST]`，未辨识；本工程按 `Ld = Lq` 处理（表贴式近似）。
    /// Stator inductance `[H]` `[ST]`, not identified; `Ld = Lq` is assumed.
    pub stator_inductance_h: f32,
    /// 永磁磁链 `[Wb]` `[ST]`，由 Workbench 内部角基准换算而来。
    /// PM flux linkage `[Wb]` `[ST]`.
    pub flux_linkage_wb: f32,
    /// 额定电流 `[A]` `[ST]`；同时是速度环输出上限和 SMO 边界层基准。
    /// Rated current `[A]` `[ST]`; also caps the speed loop and scales the SMO
    /// boundary layer.
    pub rated_current_a: f32,
    /// 最高机械转速 `[rpm]` `[ST]`，用于入参合法性检查。
    /// Maximum mechanical speed `[rpm]` `[ST]`.
    pub max_speed_rpm: f32,
    /// 标称母线电压 `[V]` / Nominal bus voltage `[V]`.
    pub nominal_bus_voltage_v: f32,
    /// 电压利用率 `[--]`，范围 `0.05..=0.98`；圆限幅按
    /// `utilization * Vbus / sqrt(3)` 计算，取 1.0 会让调制进入非线性区。
    /// Voltage utilisation `[--]`; the circle limit is
    /// `utilization * Vbus / sqrt(3)`.
    pub voltage_utilization: f32,
    /// 默认目标机械转速 `[rpm]`；必须 `<= max_speed_rpm`。
    /// Default target mechanical speed `[rpm]`.
    pub default_target_speed_rpm: f32,
    /// 定向（强制对齐）时长 `[s]` `[ST]`，> 0。
    /// Alignment duration `[s]` `[ST]`, must be positive.
    pub alignment_duration_s: f32,
    /// 开环升速时长 `[s]` `[ST]`，> 0。
    /// Open-loop ramp duration `[s]` `[ST]`, must be positive.
    pub open_loop_ramp_duration_s: f32,
    /// 观测器交接时长 `[s]` `[ST]`，可为 0（等于立即切换角度所有权）。
    /// Observer handover duration `[s]` `[ST]`; 0 switches immediately.
    pub observer_transition_duration_s: f32,
    /// 升速终点转速 `[rpm]` `[ST]`；`foc_rust_start_realtime()` 要求闭环目标转速
    /// 不低于它，否则观测器还没到可靠工作区就被要求闭环。
    /// Rev-up endpoint speed `[rpm]` `[ST]`; the realtime start requires a target
    /// at or above this value.
    pub startup_final_speed_rpm: f32,
    /// 对齐结束时的 q 轴电流 `[A]`；必须 `<= rated_current_a`。升速段从该值
    /// 线性过渡到 `startup_current_a`。
    /// Alignment-end q-axis current `[A]`; the ramp tapers from this value to
    /// `startup_current_a` and both are bounded by the rated current.
    pub startup_alignment_current_a: f32,
    /// 升速与保持段电流 `[A]` `[ST]`，必须 `<= rated_current_a`。
    /// Rev-up and hold current `[A]` `[ST]`.
    pub startup_current_a: f32,
    /// SMO 滑模增益 `[V]`。ST 参考形式是 `nominal_bus * 0.9`，对低压小电流电机
    /// 偏大；实测过大时观测器无法锁定，本工程采用重新整定的较小值。
    /// SMO sliding gain `[V]`. The reference form is nominal_bus * 0.9, which is
    /// far too large for a low-voltage low-current machine and prevents lock.
    pub observer_smo_k_slide_v: f32,
    /// SMO 边界层 `[A]`，决定滑模项等效增益 `k / boundary`。
    /// SMO boundary layer `[A]`; sets the effective gain `k / boundary`.
    pub observer_smo_boundary_a: f32,
    /// 反电动势提取低通系数 `[--]`。过小会让 SMO 输出相位滞后超过电频率，是观测
    /// 器失锁的主要诱因之一；过大则保留更多高频抖振。
    /// BEMF extraction low-pass coefficient; too small a value is a leading cause
    /// of observer loss of lock.
    pub observer_emf_filter_alpha: f32,
    /// PLL 比例增益 `[rad/s per rad]` / PLL proportional gain.
    pub observer_pll_kp: f32,
    /// 强拖捕获阶段的 PLL Kp/Ki 校正比例 `[--]`，范围 `(0, 1]`；不缩放终速前馈。
    /// Acquisition PLL Kp/Ki correction ratio `(0,1]`; forced-speed feed-forward
    /// remains unscaled. The field name is retained for ABI source compatibility.
    pub observer_acquisition_pll_kp_ratio: f32,
    /// PLL 积分增益 `[rad/s^2 per rad]` / PLL integral gain.
    pub observer_pll_ki: f32,
    /// 可靠性门限：最低估计转速 `[rpm]`，低于此值不判可靠。
    /// Reliability gate: minimum estimated speed `[rpm]`.
    pub observer_minimum_speed_rpm: f32,
    /// 启动获取窗允许的平均绝对包角相位误差 `[rad]`，范围 `(0, pi/2]`。
    /// Maximum mean absolute wrapped phase error for acquisition `(0, pi/2]`.
    pub observer_acquisition_maximum_phase_error_rad: f32,
    /// 可靠性门限：最低反电动势幅值 `[V]`，避免在零速噪声上误判可靠。
    /// Reliability gate: minimum BEMF magnitude `[V]`.
    pub observer_minimum_bemf_v: f32,
    /// 可靠性门限：转速方差上限，以均方值的比例表示 `[--]`。
    /// Reliability gate: speed-variance ceiling as a fraction of the mean square.
    pub observer_speed_variance_ratio: f32,
    /// 可靠性门限：需要连续通过的评估窗口数，范围 `1..=1000`。
    /// Reliability gate: consecutive passing windows required.
    pub observer_consecutive_samples: u32,
    /// 启动期观测器获取超时 `[s]`，范围 `0.001..=10.0`；超时锁存
    /// `FOC_FAULT_OBSERVER_STARTUP` 并进入 `Fault`。它只在开环保持段计时。
    /// Acquisition timeout `[s]`; only counts during the open-loop hold phase.
    pub observer_acquisition_timeout_s: f32,
    /// 闭环期失锁超时 `[s]`，范围 `0.001..=5.0`；超时锁存
    /// `FOC_FAULT_OBSERVER_LOST` 并进入 `Fault`。短暂失锁不会立刻故障，靠这个
    /// 时限区分"瞬态抖动"和"真的丢了转子"。
    /// Loss-of-lock timeout `[s]`; a transient reliability drop does not fault
    /// immediately, which is exactly what this deadline distinguishes.
    pub observer_loss_timeout_s: f32,
    /// 闭环接管时目标转速斜坡速率 `[rpm/s]`，避免单周期阶跃造成电流冲击；
    /// 斜坡步长按 `rate / speed_loop_frequency_hz` 计算。
    /// Closed-loop target speed ramp rate `[rpm/s]`; the per-update step is
    /// `rate / speed_loop_frequency_hz`.
    pub closed_loop_speed_ramp_rpm_per_s: f32,
    /// 无感接管转矩支撑比例 `[--]`，范围 `0.0..=1.0`。非零时，交接电流终点固定为
    /// `开环强拖 Iq * ratio`；0.0 保留 ST 的观测器坐标系实测 Iq 终点。
    /// Sensorless handoff torque-support ratio `[--]`; zero keeps the ST-style
    /// measured-Iq endpoint, non-zero selects `rev-up Iq * ratio`.
    pub handoff_torque_support_ratio: f32,
    /// 速度 PI 积分器预装比例 `[--]`，范围 `0.0..=1.0`；只作用于闭环首拍的 PI
    /// 积分状态，不再改变接管电流终点。0.0 与 ST 参考工程一致。
    /// Speed-PI integrator preload ratio `[--]`; it affects only the PI state and
    /// no longer changes the handoff torque endpoint. Zero matches the reference.
    pub speed_pi_preload_ratio: f32,
    /// 闭环时 Iq 指令限速器 `[A/s]`，范围 `0.01..=1000.0`；每拍最大变化量为
    /// `slew * (1 / pwm_frequency_hz)`，抑制电流给定阶跃引起的转矩冲击。
    /// Iq command slew limiter in closed loop `[A/s]`.
    pub closed_loop_current_slew_a_per_s: f32,
    /// 观测器接管后的运行期保持门 / Closed-loop observer-retention gate.
    pub observer_run_reliability: FocObserverRunReliabilityConfig,
    /// Park/逆 Park 执行延迟补偿 / Park/inverse-Park execution-delay compensation.
    pub angle_compensation: FocAngleCompensationConfig,
    /// 逆变器平均电压模型；默认总门和两个出口均关闭。
    /// Average inverter-voltage model; all gates default off.
    pub inverter_voltage_model: FocInverterVoltageModelConfig,
}

/// 一拍快环结束时的遥测快照 `[A] [V] [rad] [rpm]`，供 Shell 与 trace 读取。
/// Telemetry snapshot of one fast-loop call, for the shell and trace.
///
/// 三个角度字段刻意分开：`electrical_angle_rad` 是补偿前的基础控制角（可能来自
/// 强制角、观测角或两者的交接混合），`forced_electrical_angle_rad`
/// 是开环强制角，`observer_electrical_angle_rad` 是观测器原始输出。混成一个字段
/// 会让"交接是否平滑"无法从遥测判断。
/// The three angle fields are deliberately distinct so that handover smoothness can
/// be judged from telemetry.
///
/// 注意 `measured_speed_rpm` 是观测器**估计值**，不是真值测量。
/// `measured_speed_rpm` is an observer estimate, not a ground-truth measurement.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocTelemetry {
    /// 控制器逻辑状态，取值见 `FocState` / Controller state.
    pub state: u32,
    /// 本次生效的观测器后端 / Observer backend in effect.
    pub observer_backend: u32,
    /// 观测器是否已过可靠性门（0/1）/ Observer passed its reliability gate.
    pub observer_reliable: u32,
    /// 观测器是否正在提供控制角（0/1）/ Observer currently owns the angle.
    pub closed_loop_active: u32,
    /// 目标机械转速 `[rpm]` / Target mechanical speed `[rpm]`.
    pub target_speed_rpm: f32,
    /// 观测器估计的机械转速 `[rpm]`（估计值，不是实测值）。
    /// Observer-estimated mechanical speed `[rpm]`.
    pub measured_speed_rpm: f32,
    /// 本次 Park/逆 Park 延迟补偿前的基础电角度 `[rad]`。
    /// Base electrical angle before Park/inverse-Park delay compensation `[rad]`.
    pub electrical_angle_rad: f32,
    /// Id 给定 `[A]` / Id reference `[A]`.
    pub id_reference_a: f32,
    /// Iq 给定 `[A]` / Iq reference `[A]`.
    pub iq_reference_a: f32,
    /// 实测 Id `[A]`；在观测器时隙早返回的那一拍不会被更新（保持上一拍值）。
    /// Measured Id `[A]`; not refreshed on an early-return observer slot.
    pub id_measured_a: f32,
    /// 实测 Iq `[A]`；同 `id_measured_a` 的更新语义。
    /// Measured Iq `[A]`, with the same update semantics as `id_measured_a`.
    pub iq_measured_a: f32,
    /// 电流环输出的 d 轴电压指令 `[V]`（圆限幅之后）。
    /// d-axis voltage command out of the current loop, after the circle limit [V].
    pub vd_command_v: f32,
    /// 电流环输出的 q 轴电压指令 `[V]`（圆限幅之后）。
    /// q-axis voltage command out of the current loop [V].
    pub vq_command_v: f32,
    /// Rev-Up 强制电角度 `[rad]` / Rev-up forced electrical angle [rad].
    pub forced_electrical_angle_rad: f32,
    /// 观测器估计电角度 `[rad]` / Observer-estimated electrical angle [rad].
    pub observer_electrical_angle_rad: f32,
    /// 滤波后的 α/β 反电势 `[V]`；主机离线计算幅值，避免 ISR 再做开方。
    pub observer_bemf_alpha_v: f32,
    pub observer_bemf_beta_v: f32,
    /// PLL 最短路径鉴相误差 `[rad]`。
    pub observer_pll_phase_error_rad: f32,
    /// 最近一次可靠性窗口的平均机械转速 `[rpm]` 与方差 `[rpm^2]`。
    pub observer_speed_mean_rpm: f32,
    pub observer_speed_variance_rpm2: f32,
    /// 分解的 `OBSERVER_GATE_*` 位与连续通过窗口数。
    pub observer_reliability_flags: u32,
    pub observer_reliable_samples: u32,
    /// 等待收敛和闭环连续失锁计时 `[s]`。
    pub observer_wait_elapsed_s: f32,
    pub observer_loss_elapsed_s: f32,
    /// 本拍是否触发 dq 电压圆限幅（0/1）。
    pub voltage_limited: u32,
}

const _: () = assert!(size_of::<FocObserverRunReliabilityConfig>() == 8);
const _: () = assert!(size_of::<FocAngleCompensationConfig>() == 8);
const _: () = assert!(size_of::<FocInverterVoltageModelConfig>() == 36);
const _: () = assert!(size_of::<FocPiConfig>() == 28);
const _: () = assert!(align_of::<FocRuntimeConfig>() == 4);
const _: () = assert!(size_of::<FocRuntimeConfig>() == 296);
const _: () = assert!(size_of::<FocTelemetry>() == 100);

/// C 提供的控制器存储容器 `[bytes]`，对齐 8 字节。
/// Controller storage container supplied by C `[bytes]`, 8-byte aligned.
///
/// 用 `#[repr(C, align(8))]` 而不是裸数组，是为了让 Rust 侧能在其中原位构造对齐
/// 要求更高的 `Controller`（`align_of` 断言在下方的 `const _` 处检查）。
/// 容器本身不含任何状态：内容在 `foc_rust_init()` 之前是未定义的填充值。
/// The container holds no state; its bytes are undefined until `foc_rust_init()`.
#[repr(C, align(8))]
pub struct FocRustContextStorage {
    /// 原始字节；只有 Rust 侧知道其中 `Controller` 的布局。
    /// Raw bytes; only the Rust side knows the `Controller` layout inside.
    pub bytes: [u8; FOC_RUST_CONTEXT_CAPACITY],
}

/// Rust 侧真正的控制器状态；C 只看到包着它的 `FocRustContextStorage` 字节数组。
/// The real Rust controller state; C only sees the opaque storage bytes around it.
///
/// 布局与尺寸属于本 crate 的私有实现，**不是** C ABI：C 只知道容器容量 2048 字节
/// 与 8 字节对齐。因此新增字段只要仍装得下就无需改 C 侧，但仍然受下方两个编译期
/// 断言约束。
/// The layout is private, not ABI: C knows only the 2048-byte, 8-byte-aligned
/// container, so new fields need no C change as long as the assertions hold.
///
/// 所有权 / Ownership: 本结构体由 C 的 ADC 中断独占可变访问（功率级 arm 期间），
/// 不做内部加锁。`magic` 是唯一的初始化凭证，见 `CONTEXT_MAGIC`。
/// It is exclusively owned by the ADC ISR while armed; `magic` is the only
/// initialization proof.
///
/// 单位 / Units: 角度 `[rad]`，机械转速 `[rpm]`（`observer_feedback` 内部用
/// `[rad/s]`），时间 `[s]`，电流 `[A]`，电压 `[V]`，计数值 `[cycles]`。
/// Angles in `[rad]`, speeds in `[rpm]` outside the observer, times in `[s]`.
struct Controller {
    /// `CONTEXT_MAGIC` 哨兵；只有它匹配才允许把本内存当作 `Controller` 使用。
    /// `CONTEXT_MAGIC` sentinel gating every access to this memory.
    magic: u32,
    /// 逻辑状态，驱动 `foc_rust_realtime_step()` 的整条分支树。
    /// Logical state driving the whole step branch tree.
    state: FocState,
    /// 锁存的逻辑故障位（`FOC_FAULT_*` 的按位或）。`foc_rust_latch_fault()` 可以
    /// 置入 C 侧的位（例如平台过流），因此这里可能包含本 crate 未定义的高位。
    /// Latched logical fault bits; C may OR in its own bits via `latch_fault`.
    fault_flags: u32,
    /// 是否已成功配置过算法。为假时所有运行入口返回 `NotConfigured`，防止在默认
    /// 参数上"意外开转"。
    /// Whether the algorithm was configured; false blocks every run entry point.
    algorithm_configured: bool,
    /// 当前生效的控制参数（电机 + 三个 PI + 频率 + 电压利用率）。
    /// Control parameters in effect.
    params: ControlParameters,
    /// dq 电流环（含两个 PI 的积分状态）。
    /// The dq current loop.
    current_loop: CurrentLoop,
    /// 速度环（1 kHz 级），只在 `ClosedLoop` 相位被调用。
    /// The speed loop, only invoked in the closed-loop phase.
    speed_loop: SpeedLoop,
    /// 兼容入口 `foc_rust_open_loop_step()` 用的开环试转电角度 `[rad]`。
    /// Trial electrical angle `[rad]` used by the compatibility open-loop step.
    trial_angle_rad: f32,
    /// 当前生效的运行时配置（含观测器门限、超时与斜坡速率）。
    /// Runtime configuration in effect.
    runtime_config: FocRuntimeConfig,
    /// 观测器实例；后端在 `configure_runtime()` 时选定，运行期不再变。
    /// Observer instance; its backend is fixed at configuration time.
    observer: ConfigurableObserver,
    /// 观测器分频计数器 `[cycles]`：归零的那一拍才更新观测器。
    /// Observer divider counter [cycles]; the observer updates when it reaches 0.
    observer_counter: u32,
    /// 升速阶段是否已用同拍电流初始化观察器。对齐阶段保持为 false。
    /// Whether acquisition was seeded from the same-sample current after alignment.
    observer_acquisition_prepared: bool,
    /// 开环保持段距离下一次有界重捕获的控制拍倒计数；总获取超时仍是最终上限。
    /// Ticks until the next bounded hold-stage reacquisition; the global acquisition
    /// timeout remains the hard limit.
    observer_hold_recovery_countdown: u32,
    /// 最近一次观测器输出（电角度 `[rad]` 与机械转速 `[rad/s]`）。
    /// Last observer output (electrical angle `[rad]`, mechanical speed `[rad/s]`).
    observer_feedback: RotorFeedback,
    /// 最近一次采样时刻观测器是否过可靠性门；失锁判断依赖它。
    /// Whether the observer passed the acquisition gate at the last update.
    observer_reliable: bool,
    /// 最近一次采样时刻观测器是否过运行期保持门；只在已经接管角度后用于失锁判断。
    /// Whether the observer passed the closed-loop retention gate at the last update.
    observer_run_reliable: bool,
    /// Rev-Up 启动时序（对齐 -> 升速 -> 保持 -> 交接 -> 闭环）。
    /// The rev-up start-up sequencer.
    startup: RevUpSequencer,
    /// 上一拍真正下发的 PWM，观测器重构相电压时使用；也用于多率路径保持输出。
    /// Last PWM actually issued; used for observer voltage reconstruction and for
    /// holding the output on multi-rate observer slots.
    previous_pwm: PwmCommand,
    /// 本拍实际施加给电流环的 dq 给定 `[A]`（可能被限速器整形过）。
    /// dq reference actually applied to the current loop this tick.
    current_reference: CurrentCommand,
    /// 速度环最近一次算出的 dq 给定 `[A]`，是限速器的目标值。
    /// Latest speed-loop dq reference; the slew limiter's target.
    speed_current_command: CurrentCommand,
    /// 闭环目标机械转速 `[rpm]`，来自 `foc_rust_start_realtime()`。
    /// Closed-loop target mechanical speed `[rpm]`.
    target_speed_rpm: f32,
    /// 被转速斜坡整形后的目标 `[rpm]`，是速度环的真正输入。
    /// Ramp-shaped speed reference `[rpm]`, the speed loop's actual input.
    speed_reference_rpm: f32,
    /// 速度环分频计数器 `[cycles]`：`(电流环频率 / 速度环频率)` 循环，归零那拍才
    /// 执行速度 PI 并推进转速斜坡。
    /// Speed-loop divider counter [cycles].
    speed_counter: u32,
    /// 开环保持段已等待观测器的时间 `[s]`；只在 `OpenLoopHold` 累加。
    /// Time spent waiting for the observer during open-loop hold `[s]`.
    observer_wait_elapsed_s: f32,
    /// 观测器已连续失锁的时间 `[s]`；只要观测器提供控制角就一直计时。
    /// Continuous observer-unreliable time `[s]` while the observer owns the angle.
    observer_loss_elapsed_s: f32,
    /// 闭环首拍是否已初始化（转速斜坡起点与速度 PI 预装只做一次）。
    /// Whether the first closed-loop tick ran; the ramp seed and speed-PI preload
    /// happen exactly once.
    closed_loop_initialized: bool,
    /// 公共逆变器模型；目标与 Host 使用同一状态和同一组合顺序。
    /// Shared inverter model; target and host use the same state and ordering.
    inverter_voltage_model: InverterVoltageModel,
    /// 最近一次快环结束时对外发布的遥测快照。
    /// Latest telemetry snapshot published to the outside world.
    telemetry: FocTelemetry,
}

impl Controller {
    /// 用 `[ST]` 参考参数构造一个"停机、未配置完成但参数已就绪"的控制器。
    /// Builds a stopped controller preloaded with the `[ST]` reference parameters.
    ///
    /// 注意 `state` 初值是 `Disabled` 而非 `Uninitialized`：`Uninitialized` 只作为
    /// 查询入口对空/坏指针的返回值，永不写进真实状态。`algorithm_configured`
    /// 初值为假，所以即使参数已就绪也必须显式配置才能启动——这是有意的双重门。
    /// `state` starts at `Disabled`, never `Uninitialized`, and the configuration
    /// gate stays closed on purpose.
    fn new() -> Self {
        let params = st_gbm2804_reference_parameters();
        let runtime_config = default_st_runtime_config();
        Self {
            magic: CONTEXT_MAGIC,
            state: FocState::Disabled,
            fault_flags: 0,
            algorithm_configured: false,
            params,
            current_loop: CurrentLoop::default(),
            speed_loop: SpeedLoop::default(),
            trial_angle_rad: 0.0,
            runtime_config,
            observer: ConfigurableObserver::new(
                ObserverBackend::SmoPll,
                params.motor,
                1.0 / params.pwm_frequency_hz as f32,
            ),
            observer_counter: 0,
            observer_acquisition_prepared: false,
            observer_hold_recovery_countdown: 0,
            observer_feedback: RotorFeedback::default(),
            observer_reliable: false,
            observer_run_reliable: false,
            startup: RevUpSequencer::with_config(params.motor.pole_pairs, RevUpConfig::default()),
            previous_pwm: PwmCommand::default(),
            current_reference: CurrentCommand::default(),
            speed_current_command: CurrentCommand::default(),
            target_speed_rpm: params.default_target_speed_rpm,
            speed_reference_rpm: 0.0,
            speed_counter: 0,
            observer_wait_elapsed_s: 0.0,
            observer_loss_elapsed_s: 0.0,
            closed_loop_initialized: false,
            inverter_voltage_model: InverterVoltageModel::default(),
            telemetry: FocTelemetry::default(),
        }
    }

    /// 停机：状态回 `Disabled`，复位两个环、观测器、启动时序、限速器与全部计时。
    /// Stops and resets both loops, the observer, the sequencer, the limiters and
    /// every timer; the state returns to `Disabled`.
    ///
    /// **不清除 `fault_flags`**：故障位是粘滞的，只有 `foc_rust_clear_fault()`
    /// 才能清零。这样"故障发生过"这一事实不会被一次例行停机抹掉。
    /// `fault_flags` is deliberately NOT cleared here: fault bits are sticky and
    /// only `foc_rust_clear_fault()` resets them.
    ///
    /// `observer.reset(0.0)` 把估计角度归零而不是保留：重启启动链时观测器必须从
    /// 确定的初始角出发，否则交接角会带着上一次运行的残留。
    /// The observer is reset to a known angle so a restart cannot inherit a stale
    /// handover angle.
    fn stop(&mut self) {
        self.state = FocState::Disabled;
        self.current_loop.reset();
        self.speed_loop.reset();
        self.trial_angle_rad = 0.0;
        self.observer.reset(0.0);
        self.observer_counter = 0;
        self.observer_acquisition_prepared = false;
        self.observer_hold_recovery_countdown = 0;
        self.observer_feedback = RotorFeedback::default();
        self.observer_reliable = false;
        self.observer_run_reliable = false;
        self.startup.reset();
        self.previous_pwm = PwmCommand::default();
        self.current_reference = CurrentCommand::default();
        self.speed_current_command = CurrentCommand::default();
        self.speed_reference_rpm = 0.0;
        self.speed_counter = 0;
        self.observer_wait_elapsed_s = 0.0;
        self.observer_loss_elapsed_s = 0.0;
        self.closed_loop_initialized = false;
        self.inverter_voltage_model.reset();
        self.telemetry = FocTelemetry::default();
    }
}

/// 编译期证明：`Controller` 装得进 C 提供的 2048 字节容器，且对齐要求不超过容器。
/// Compile-time proof that `Controller` fits the C container and is not
/// over-aligned for it.
///
/// 这两条断言是"改结构体不用改 C"这一便利的安全网：一旦超限，构建就失败，而不是
/// 在运行期越界写入调用方的静态存储。
/// These two assertions are the safety net behind that convenience: exceeding the
/// budget breaks the build instead of corrupting C's static storage at runtime.
const _: () = assert!(size_of::<Controller>() <= FOC_RUST_CONTEXT_CAPACITY);
const _: () = assert!(align_of::<Controller>() <= align_of::<FocRustContextStorage>());

/// 把输出占空比归零。所有失败返回路径都会先调用它。
/// Zeroes the output duties; every failure return calls it first.
///
/// 语义 / Semantics: 清零只表示"本拍不施加电压"，**不等于关断栅极**。C 平台层
/// 仍需在 `HardwareFault` 时执行硬件关断；调用方也不得因为看到 0 就跳过故障处理。
/// A zeroed output means "no voltage this tick", not "gates are off".
fn zero_output(output: &mut FocOutput) {
    *output = FocOutput::default();
}

/// 以不超过 `maximum_step` 的步长把 `current` 向 `target` 移动，用于转速/电流限速。
/// Moves `current` toward `target` by at most `maximum_step` per call.
///
/// `copysign` 保证方向由 `delta` 决定而不是由步长符号决定，因此 `maximum_step`
/// 必须非负且有限（调用方传入的都是配置里校验过的正数）。`target - current` 在
/// 极端量纲下可能溢出为 `inf`，此时 `|inf| > maximum_step` 仍成立，结果退化为
/// `current ± maximum_step`，不会产生 NaN。
/// The direction comes from `delta`, so `maximum_step` must be non-negative; an
/// overflowed `delta` still degrades to `current ± maximum_step` instead of NaN.
fn move_towards(current: f32, target: f32, maximum_step: f32) -> f32 {
    let delta = target - current;
    if delta.abs() <= maximum_step {
        target
    } else {
        current + maximum_step.copysign(delta)
    }
}

/// Selects the MCSDK-compatible measured switch-over current at ratio zero, or a
/// deterministic bounded torque target when explicitly enabled.
#[inline]
fn supported_handoff_iq(measured_iq_a: f32, rev_up_iq_a: f32, ratio: f32) -> f32 {
    if ratio == 0.0 {
        measured_iq_a
    } else {
        rev_up_iq_a * ratio
    }
}

/// 校验单个 PI 配置：所有字段有限、`ts > 0`、上下限顺序正确。
/// Validates one PI configuration: finite fields, positive `ts`, ordered limits.
///
/// `NaN` 是这里的主要防守目标：`NaN` 参与比较恒为假，若不显式 `is_finite()`，一个
/// `NaN` 增益会静默穿过所有区间检查，让控制器输出 `NaN` 占空比。
/// `NaN` never satisfies a comparison, so `is_finite()` is the only reliable guard.
fn pi_is_valid(pi: &FocPiConfig) -> bool {
    pi.kp.is_finite()
        && pi.ki.is_finite()
        && pi.ts.is_finite()
        && pi.out_min.is_finite()
        && pi.out_max.is_finite()
        && pi.integrator_min.is_finite()
        && pi.integrator_max.is_finite()
        && pi.ts > 0.0
        && pi.out_min <= pi.out_max
        && pi.integrator_min <= pi.integrator_max
}

/// 校验兼容入口的基础配置：两个电流环合法且母线电压为正有限值 `[V]`。
/// Validates the compatibility basic configuration.
fn config_is_valid(config: &FocBasicConfig) -> bool {
    pi_is_valid(&config.id_pi)
        && pi_is_valid(&config.iq_pi)
        && config.nominal_dc_bus_voltage.is_finite()
        && config.nominal_dc_bus_voltage > 0.0
}

/// 校验一拍反馈：三相电流、母线电压、电角度有限，且母线电压 `[V]` 为正。
/// Validates one feedback snapshot; the bus voltage must be finite and positive.
///
/// 母线电压为 0 会让所有电压归一化（占空比）除法溢出或产生 `inf`，因此这里就
/// 拦掉，而不是等算法算出非法占空比再报故障。
/// A zero bus voltage would make every duty normalization blow up, so it is
/// rejected here rather than being caught later as an invalid output.
fn feedback_is_valid(feedback: &FocFeedback) -> bool {
    feedback.phase_current_a.is_finite()
        && feedback.phase_current_b.is_finite()
        && feedback.phase_current_c.is_finite()
        && feedback.dc_bus_voltage.is_finite()
        && feedback.dc_bus_voltage > 0.0
        && feedback.electrical_angle_rad.is_finite()
}

/// 校验兼容入口的 dq 电流给定 `[A]` 有限。
/// Validates the compatibility dq current reference `[A]`.
fn reference_is_valid(reference: &FocReference) -> bool {
    reference.id_ref.is_finite() && reference.iq_ref.is_finite()
}

/// C ABI 的 `FocPiConfig` -> 算法的 `PiParam`。两侧字段逐一同名同量纲，本函数只是
/// 打破依赖方向（`foc-algorithm` 不能认识 C ABI 类型）的适配层，不做任何换算。
/// Adapts the C ABI `FocPiConfig` to the algorithm `PiParam`; no unit conversion
/// happens here, the two are field-for-field identical.
fn to_pi_param(config: FocPiConfig) -> PiParam {
    PiParam {
        kp: config.kp,
        ki: config.ki,
        ts: config.ts,
        out_min: config.out_min,
        out_max: config.out_max,
        integrator_min: config.integrator_min,
        integrator_max: config.integrator_max,
    }
}

/// 与 `to_pi_param()` 相反，把 ST 参考参数回填进可编辑的 C 配置结构体。
/// Inverse of `to_pi_param()`: fills the editable C configuration struct.
fn from_pi_param(param: PiParam) -> FocPiConfig {
    FocPiConfig {
        kp: param.kp,
        ki: param.ki,
        ts: param.ts,
        out_min: param.out_min,
        out_max: param.out_max,
        integrator_min: param.integrator_min,
        integrator_max: param.integrator_max,
    }
}

/// 由 `[ST]` MCSDK 6.4.1 参考参数导出的一份完整、可直接使用的运行时配置默认值。
/// Derives a complete, directly usable runtime configuration from the `[ST]`
/// MCSDK 6.4.1 reference parameters.
///
/// 基线硬件 / Baseline hardware: NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T，
/// 12 kHz 电流环、1 kHz 速度环、母线 13 V `[ST]`。电机参数（Rs、Ls、磁链、Kp/Ki）
/// 来自 Workbench 转写，**未在实物上辨识**，必须视为起点。
/// The motor parameters are Workbench transcriptions, NOT identified on the real
/// machine; treat them as starting points.
///
/// 两个刻意的保守默认值 / Two deliberately conservative defaults:
/// `closed_loop_enable = 0` —— 首次上板保持电流控制的开环，只有在实机遥测通过交接
/// 门限后才打开闭环；`observer_update_divider = 1` —— 12 kHz 下 STM32G431 的预算
/// 足以在同一次 PWM 周期内跑完观测器与电流环。
/// `closed_loop_enable = 0` keeps first bring-up in current-controlled open loop;
/// `divider = 1` is affordable at 12 kHz on this part.
///
/// 其余 `[HW]` 板级默认值来自 2026-09-25 CM4.4 空载双向重复实机门：
/// 对齐 2.0 s、强拖 5.0 s、获取超时 5.0 s、失锁超时 0.20 s、
/// 转矩支撑 0.47、速度 PI 预装 0.20、Iq 限速 4 A/s。这些是启动与接管候选，
/// 不代表电机参数已辨识，也不改变上电闭环和逆变器门均关闭的策略。
/// The board-level start/handover defaults are the CM4.4 unloaded bidirectional
/// bench candidate; closed-loop and inverter gates still remain off at boot.
fn default_st_runtime_config() -> FocRuntimeConfig {
    let params = st_gbm2804_reference_parameters();
    // Keep the reusable control crate's generic MCSDK starting point intact;
    // this bridge owns the board/motor-specific candidate proved by CM4.4.
    let startup = RevUpConfig {
        alignment_s: 2.0,
        ramp_s: 5.0,
        transition_s: 0.05,
        final_speed_rpm: 582.0,
        alignment_current_a: 0.8,
        final_current_a: 0.8,
    };
    let observer = SmoPllTuning {
        k_slide_v: 4.0,
        boundary_a: 0.24,
        emf_filter_alpha: 0.015,
        pll_kp: 40.0,
        acquisition_pll_kp_ratio: 0.60,
        pll_ki: 1_000.0,
    };
    let reliability = ObserverReliabilityConfig::for_motor(params.motor);
    FocRuntimeConfig {
        struct_size: size_of::<FocRuntimeConfig>() as u32,
        config_version: FOC_RUST_CONFIG_VERSION,
        observer_backend: ObserverBackend::SmoPll as u32,
        observer_enable: 1,
        // First board runs stay in current-controlled open loop. This field is
        // explicitly enabled only after observer telemetry passes the handoff
        // gates on the actual motor.
        closed_loop_enable: 0,
        // At 12 kHz the STM32G431 has enough budget to execute the selected
        // observer and current controller in the same PWM period.
        observer_update_divider: 1,
        id_pi: from_pi_param(params.id_pi),
        iq_pi: from_pi_param(params.iq_pi),
        speed_pi: from_pi_param(params.speed_pi),
        pole_pairs: params.motor.pole_pairs as u32,
        pwm_frequency_hz: params.pwm_frequency_hz,
        speed_loop_frequency_hz: params.speed_loop_frequency_hz,
        stator_resistance_ohm: params.motor.stator_resistance_ohm,
        stator_inductance_h: params.motor.ld_h,
        flux_linkage_wb: params.motor.flux_linkage_wb,
        rated_current_a: params.motor.rated_current_a,
        max_speed_rpm: params.motor.max_speed_rpm,
        nominal_bus_voltage_v: params.motor.nominal_bus_voltage_v,
        voltage_utilization: params.voltage_utilization,
        default_target_speed_rpm: params.default_target_speed_rpm,
        alignment_duration_s: startup.alignment_s,
        open_loop_ramp_duration_s: startup.ramp_s,
        observer_transition_duration_s: startup.transition_s,
        startup_final_speed_rpm: startup.final_speed_rpm,
        startup_alignment_current_a: startup.alignment_current_a,
        startup_current_a: startup.final_current_a,
        observer_smo_k_slide_v: observer.k_slide_v,
        observer_smo_boundary_a: observer.boundary_a,
        observer_emf_filter_alpha: observer.emf_filter_alpha,
        observer_pll_kp: observer.pll_kp,
        observer_acquisition_pll_kp_ratio: observer.acquisition_pll_kp_ratio,
        observer_pll_ki: observer.pll_ki,
        observer_minimum_speed_rpm: params.default_target_speed_rpm,
        observer_acquisition_maximum_phase_error_rad: 0.65,
        observer_minimum_bemf_v: reliability.minimum_bemf_v,
        observer_speed_variance_ratio: reliability.speed_variance_ratio,
        observer_consecutive_samples: 20,
        observer_acquisition_timeout_s: 5.0,
        observer_loss_timeout_s: 0.20,
        closed_loop_speed_ramp_rpm_per_s: 500.0,
        // The actual MCSDK project has PID_SPEED_INTEGRAL_INIT_DIV=0, so its
        // speed PI integral starts at zero. The applied Iq still crosses over
        // continuously through the configurable current slew below.
        handoff_torque_support_ratio: 0.47,
        speed_pi_preload_ratio: 0.20,
        closed_loop_current_slew_a_per_s: 4.0,
        // MCSDK separates OBS_MINIMUM_SPEED_RPM=524 (acquisition) from
        // MIN_APPLICATION_SPEED_RPM=0 (run-time retention). BEMF, raw phase error
        // and the 100 ms loss timer guard a genuinely lost observer; the 64 ms
        // acquisition variance history is intentionally not reused after handover.
        observer_run_reliability: FocObserverRunReliabilityConfig {
            minimum_speed_rpm: 0.0,
            maximum_phase_error_rad: 0.65,
        },
        // The actual P-IHM03-Potentiometer project defines both MCSDK angle
        // compensation factors as zero. Keep that as the safe, bit-compatible
        // baseline; non-zero values are opt-in tuning candidates.
        angle_compensation: FocAngleCompensationConfig {
            park_prediction_ticks: 0.0,
            reverse_park_prediction_ticks: 0.0,
        },
        // Physical candidates are preloaded for an explicit staged experiment,
        // but all three gates stay off. 550 ns is the IHM16M1/STSPIN830 hardware
        // dead time; device drop remains unknown and therefore zero.
        inverter_voltage_model: FocInverterVoltageModelConfig {
            enabled: 0,
            observer_voltage_correction_enable: 0,
            pwm_feedforward_enable: 0,
            pwm_carrier_frequency_hz: params.pwm_frequency_hz,
            dead_time_s: 550.0e-9,
            compensation_gain: 1.0,
            current_zero_band_a: 0.005,
            current_sign_filter_alpha: 1.0,
            device_drop_v: 0.0,
        },
    }
}

/// 用 CRC-32/ISO-HDLC 把一个 32 位字按小端字节顺序追加到规范化配置流。
/// Appends one little-endian 32-bit word to a CRC-32/ISO-HDLC stream.
fn runtime_config_crc32_word(mut crc: u32, word: u32) -> u32 {
    for byte in word.to_le_bytes() {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let reflected_polynomial = 0xEDB8_8320 & (0u32.wrapping_sub(crc & 1));
            crc = (crc >> 1) ^ reflected_polynomial;
        }
    }
    crc
}

/// 对运行时配置做与内存布局无关的规范化 CRC-32。
/// Computes a canonical CRC-32 over a runtime configuration without reading
/// structure padding or native-endian memory.
///
/// 结构体仅由 69 个 `u32/f32` ABI 字构成；上方的 size/alignment 断言与
/// C 侧 `_Static_assert` 共同证明其间没有填充。本函数逐字按本机布局读取，再转成
/// 小端字节输入 CRC-32/ISO-HDLC，因此 Host 与 MCU 结果一致。新增或重排
/// 字段时必须同时提升 ABI 并更新已审批配置档的 CRC。
fn runtime_config_crc32(config: &FocRuntimeConfig) -> u32 {
    const CONFIG_WORDS: usize = size_of::<FocRuntimeConfig>() / size_of::<u32>();
    let mut crc = 0xFFFF_FFFF;

    // SAFETY: FocRuntimeConfig is repr(C), exactly 296 bytes and composed only of
    // repr(C) four-byte u32/f32 leaves. The compile-time layout assertions above
    // prove there are exactly 74 initialized words and no padding. Byte views may
    // alias any initialized object; from_ne_bytes then recovers each logical word.
    let bytes = unsafe {
        core::slice::from_raw_parts(
            core::ptr::from_ref(config).cast::<u8>(),
            size_of::<FocRuntimeConfig>(),
        )
    };
    for index in 0..CONFIG_WORDS {
        let offset = index * 4;
        let word = u32::from_ne_bytes([
            bytes[offset],
            bytes[offset + 1],
            bytes[offset + 2],
            bytes[offset + 3],
        ]);
        crc = runtime_config_crc32_word(crc, word);
    }
    !crc
}

/// 把 ABI 配置映射成控制层模型配置。电压来源在 CM3 仍固定为 CommandModel；
/// PhaseVoltage/Hybrid 要等 A23～A25 的物理量 ABI、质量门与失效回退完成后才可开放。
/// Maps the ABI group into the control-layer model. CM3 deliberately fixes the
/// source to CommandModel; measured/hybrid sources remain behind later gates.
fn inverter_voltage_model_config(config: &FocRuntimeConfig) -> InverterVoltageModelConfig {
    let inverter = config.inverter_voltage_model;
    InverterVoltageModelConfig {
        enabled: inverter.enabled != 0,
        dead_time_s: inverter.dead_time_s,
        pwm_period_s: 1.0 / inverter.pwm_carrier_frequency_hz as f32,
        compensation_gain: inverter.compensation_gain,
        current_zero_band_a: inverter.current_zero_band_a,
        current_sign_filter_alpha: inverter.current_sign_filter_alpha,
        device_drop_v: inverter.device_drop_v,
        observer_voltage_correction_enabled: inverter.observer_voltage_correction_enable != 0,
        feedforward_enabled: inverter.pwm_feedforward_enable != 0,
        source: ObserverVoltageSource::CommandModel,
    }
}

/// 校验完整运行时配置。任何一项不满足即拒绝整份配置，控制器状态不变。
/// Validates the whole runtime configuration; one failure rejects all of it and
/// leaves the controller untouched.
///
/// 本函数同时是"编译期结构体兼容性"的运行期补充：`struct_size` 与
/// `config_version` 必须与当前二进制一致，否则旧结构体的字节会被按新布局解释。
/// Along with `struct_size` and `config_version` this prevents old bytes from
/// being reinterpreted under a new layout.
///
/// 三条关键不变量 / Three key invariants:
/// `speed_loop_frequency_hz` 必须整除 `pwm_frequency_hz` —— 速度环分频器是整数
/// 计数器，不整除会让速度环平均周期与写入 `ts` 的周期不一致，PI 增益随之偏差。
/// `closed_loop_enable != 0` 时要求 `observer_enable != 0` —— 没有观测器就没有
/// 可用的控制角，闭环无从谈起。
/// `observer_update_divider` 限制在 `1..=32` —— 分频过大时观测器时隙之间的角度
/// 保持时间变长，观测器带宽相对变低。
/// The speed rate must divide the PWM rate, closed loop requires the observer, and
/// the divider is capped at 32.
///
/// 注意 `startup.is_valid()` 在最后：它检查的是由本结构体字段临时拼出的
/// `RevUpConfig`，所以启动时序的自洽性（各时长 > 0、终点转速 > 0）也在校验范围内。
/// The final `startup.is_valid()` covers the start-up sequence built from these
/// fields.
fn runtime_config_is_valid(config: &FocRuntimeConfig) -> bool {
    let startup = RevUpConfig {
        alignment_s: config.alignment_duration_s,
        ramp_s: config.open_loop_ramp_duration_s,
        transition_s: config.observer_transition_duration_s,
        final_speed_rpm: config.startup_final_speed_rpm,
        alignment_current_a: config.startup_alignment_current_a,
        final_current_a: config.startup_current_a,
    };
    let observer = SmoPllTuning {
        k_slide_v: config.observer_smo_k_slide_v,
        boundary_a: config.observer_smo_boundary_a,
        emf_filter_alpha: config.observer_emf_filter_alpha,
        pll_kp: config.observer_pll_kp,
        acquisition_pll_kp_ratio: config.observer_acquisition_pll_kp_ratio,
        pll_ki: config.observer_pll_ki,
    };
    let reliability = ObserverReliabilityConfig {
        minimum_speed_rpm: config.observer_minimum_speed_rpm,
        minimum_bemf_v: config.observer_minimum_bemf_v,
        speed_variance_ratio: config.observer_speed_variance_ratio,
        maximum_phase_error_rad: config.observer_acquisition_maximum_phase_error_rad,
        consecutive_samples: config.observer_consecutive_samples.min(u16::MAX as u32) as u16,
    };
    let inverter = config.inverter_voltage_model;
    config.struct_size == size_of::<FocRuntimeConfig>() as u32
        && config.config_version == FOC_RUST_CONFIG_VERSION
        && ObserverBackend::from_raw(config.observer_backend).is_some()
        && config.observer_enable <= 1
        && config.closed_loop_enable <= 1
        && (config.closed_loop_enable == 0 || config.observer_enable != 0)
        && (1..=32).contains(&config.observer_update_divider)
        && pi_is_valid(&config.id_pi)
        && pi_is_valid(&config.iq_pi)
        && pi_is_valid(&config.speed_pi)
        && (1..=32).contains(&config.pole_pairs)
        && config.pwm_frequency_hz >= 1_000
        && config.speed_loop_frequency_hz >= 10
        && config.speed_loop_frequency_hz <= config.pwm_frequency_hz
        && config
            .pwm_frequency_hz
            .is_multiple_of(config.speed_loop_frequency_hz)
        && config.stator_resistance_ohm.is_finite()
        && config.stator_resistance_ohm > 0.0
        && config.stator_inductance_h.is_finite()
        && config.stator_inductance_h > 0.0
        && config.flux_linkage_wb.is_finite()
        && config.flux_linkage_wb > 0.0
        && config.rated_current_a.is_finite()
        && config.rated_current_a > 0.0
        && config.max_speed_rpm.is_finite()
        && config.max_speed_rpm > 0.0
        && config.nominal_bus_voltage_v.is_finite()
        && config.nominal_bus_voltage_v > 0.0
        && config.voltage_utilization.is_finite()
        && (0.05..=0.98).contains(&config.voltage_utilization)
        && config.default_target_speed_rpm.is_finite()
        && config.default_target_speed_rpm > 0.0
        && config.default_target_speed_rpm <= config.max_speed_rpm
        && config.startup_alignment_current_a <= config.rated_current_a
        && config.startup_current_a <= config.rated_current_a
        && observer.is_valid()
        && reliability.is_valid()
        && config.observer_minimum_speed_rpm < config.max_speed_rpm * 1.10
        && config
            .observer_acquisition_maximum_phase_error_rad
            .is_finite()
        && config.observer_acquisition_maximum_phase_error_rad > 0.0
        && config.observer_acquisition_maximum_phase_error_rad <= core::f32::consts::FRAC_PI_2
        && config
            .observer_run_reliability
            .minimum_speed_rpm
            .is_finite()
        && config.observer_run_reliability.minimum_speed_rpm >= 0.0
        && config.observer_run_reliability.minimum_speed_rpm <= config.observer_minimum_speed_rpm
        && config
            .observer_run_reliability
            .maximum_phase_error_rad
            .is_finite()
        && config.observer_run_reliability.maximum_phase_error_rad > 0.0
        && config.observer_run_reliability.maximum_phase_error_rad <= core::f32::consts::FRAC_PI_2
        && config.angle_compensation.park_prediction_ticks.is_finite()
        && (-2.0..=2.0).contains(&config.angle_compensation.park_prediction_ticks)
        && config
            .angle_compensation
            .reverse_park_prediction_ticks
            .is_finite()
        && (-2.0..=2.0).contains(&config.angle_compensation.reverse_park_prediction_ticks)
        && inverter.enabled <= 1
        && inverter.observer_voltage_correction_enable <= 1
        && inverter.pwm_feedforward_enable <= 1
        && (inverter.enabled != 0
            || (inverter.observer_voltage_correction_enable == 0
                && inverter.pwm_feedforward_enable == 0))
        && (1_000..=1_000_000).contains(&inverter.pwm_carrier_frequency_hz)
        && inverter.pwm_carrier_frequency_hz >= config.pwm_frequency_hz
        && inverter
            .pwm_carrier_frequency_hz
            .is_multiple_of(config.pwm_frequency_hz)
        && inverter.dead_time_s.is_finite()
        && (0.0..=0.01).contains(&inverter.dead_time_s)
        && inverter.compensation_gain.is_finite()
        && (0.0..=4.0).contains(&inverter.compensation_gain)
        && inverter.current_zero_band_a.is_finite()
        && (0.0..=10.0).contains(&inverter.current_zero_band_a)
        && inverter.current_sign_filter_alpha.is_finite()
        && (0.0001..=1.0).contains(&inverter.current_sign_filter_alpha)
        && inverter.device_drop_v.is_finite()
        && (0.0..=100.0).contains(&inverter.device_drop_v)
        && (1..=1_000).contains(&config.observer_consecutive_samples)
        && config.observer_acquisition_timeout_s.is_finite()
        && (0.001..=10.0).contains(&config.observer_acquisition_timeout_s)
        && config.observer_loss_timeout_s.is_finite()
        && (0.001..=5.0).contains(&config.observer_loss_timeout_s)
        && config.closed_loop_speed_ramp_rpm_per_s.is_finite()
        && config.closed_loop_speed_ramp_rpm_per_s > 0.0
        && config.handoff_torque_support_ratio.is_finite()
        && (0.0..=1.0).contains(&config.handoff_torque_support_ratio)
        && config.speed_pi_preload_ratio.is_finite()
        && (0.0..=1.0).contains(&config.speed_pi_preload_ratio)
        && config.closed_loop_current_slew_a_per_s.is_finite()
        && (0.01..=1_000.0).contains(&config.closed_loop_current_slew_a_per_s)
        && startup.is_valid()
}

/// 把一份**已通过校验**的运行时配置整体装入控制器，并复位全部运行期状态。
/// Installs an already-validated runtime configuration and resets all run state.
///
/// 调用约束 / Caller contract: 只有 `foc_rust_configure()` 在校验通过且控制器处于
/// `Disabled` 时调用它；直接调用会绕过校验，把非法参数装进观察器与 PI。
/// Only `foc_rust_configure()` calls this, after validation and only while
/// `Disabled`; calling it directly bypasses every check.
///
/// 观测器采样周期 / Observer sample period: 传入的是
/// `observer_update_divider / pwm_frequency_hz`，即观测器**两次更新之间的实际时间**
/// `[s]`。分频 > 1 时这个值必须随分频放大，否则观测器会以为自己跑在 12 kHz 而实
/// 际每 N 拍才更新一次，滑模项与 PLL 增益的等效带宽全部偏大 N 倍。运行时
/// `foc_rust_realtime_step()` 只在分频计数归零那一拍调用观测器，与这里的 `ts`
/// 保持一致。
/// The observer is told the real interval between its updates
/// (`divider / pwm_frequency_hz`); the realtime step must update it on exactly the
/// same schedule or every observer gain is effectively scaled by the divider.
///
/// `unwrap_or(ObserverBackend::SmoPll)` 只是兜底：非法取值已在
/// `runtime_config_is_valid()` 处被拒绝，因此运行时不会走到这个默认分支。
/// The `unwrap_or` fallback is unreachable in practice: `runtime_config_is_valid()`
/// already rejected unknown backend values.
///
/// 副作用 / Side effects: 这里把状态强制回 `Disabled`，所以配置过程中不会带着旧
/// 积分状态重新启动；`fault_flags` 不被清除，粘滞故障位需要显式
/// `foc_rust_clear_fault()`。
/// The state is forced back to `Disabled`; latched fault bits are left untouched.
fn configure_runtime(controller: &mut Controller, config: FocRuntimeConfig) {
    let backend =
        ObserverBackend::from_raw(config.observer_backend).unwrap_or(ObserverBackend::SmoPll);
    let mut params = st_gbm2804_reference_parameters();
    params.id_pi = to_pi_param(config.id_pi);
    params.iq_pi = to_pi_param(config.iq_pi);
    params.speed_pi = to_pi_param(config.speed_pi);
    params.motor.pole_pairs = config.pole_pairs as u8;
    params.motor.stator_resistance_ohm = config.stator_resistance_ohm;
    params.motor.ld_h = config.stator_inductance_h;
    params.motor.lq_h = config.stator_inductance_h;
    params.motor.flux_linkage_wb = config.flux_linkage_wb;
    params.motor.rated_current_a = config.rated_current_a;
    params.motor.max_speed_rpm = config.max_speed_rpm;
    params.motor.nominal_bus_voltage_v = config.nominal_bus_voltage_v;
    params.pwm_frequency_hz = config.pwm_frequency_hz;
    params.speed_loop_frequency_hz = config.speed_loop_frequency_hz;
    params.voltage_utilization = config.voltage_utilization;
    params.default_target_speed_rpm = config.default_target_speed_rpm;
    let startup_config = RevUpConfig {
        alignment_s: config.alignment_duration_s,
        ramp_s: config.open_loop_ramp_duration_s,
        transition_s: config.observer_transition_duration_s,
        final_speed_rpm: config.startup_final_speed_rpm,
        alignment_current_a: config.startup_alignment_current_a,
        final_current_a: config.startup_current_a,
    };
    controller.params = params;
    controller.runtime_config = config;
    controller.observer = ConfigurableObserver::new_with_smo_tuning_and_reliability(
        backend,
        params.motor,
        config.observer_update_divider as f32 / params.pwm_frequency_hz as f32,
        SmoPllTuning {
            k_slide_v: config.observer_smo_k_slide_v,
            boundary_a: config.observer_smo_boundary_a,
            emf_filter_alpha: config.observer_emf_filter_alpha,
            pll_kp: config.observer_pll_kp,
            acquisition_pll_kp_ratio: config.observer_acquisition_pll_kp_ratio,
            pll_ki: config.observer_pll_ki,
        },
        ObserverReliabilityConfig {
            minimum_speed_rpm: config.observer_minimum_speed_rpm,
            minimum_bemf_v: config.observer_minimum_bemf_v,
            speed_variance_ratio: config.observer_speed_variance_ratio,
            maximum_phase_error_rad: config.observer_acquisition_maximum_phase_error_rad,
            consecutive_samples: config.observer_consecutive_samples as u16,
        },
    );
    controller.observer_counter = 0;
    controller.observer_acquisition_prepared = false;
    controller.observer_hold_recovery_countdown = 0;
    controller.observer_feedback = RotorFeedback::default();
    controller.observer_reliable = false;
    controller.observer_run_reliable = false;
    controller.startup = RevUpSequencer::with_config(params.motor.pole_pairs, startup_config);
    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.algorithm_configured = true;
    controller.state = FocState::Disabled;
    controller.target_speed_rpm = params.default_target_speed_rpm;
    controller.previous_pwm = PwmCommand::default();
    controller.current_reference = CurrentCommand::default();
    controller.speed_current_command = CurrentCommand::default();
    controller.speed_reference_rpm = 0.0;
    controller.speed_counter = 0;
    controller.observer_wait_elapsed_s = 0.0;
    controller.observer_loss_elapsed_s = 0.0;
    controller.closed_loop_initialized = false;
    // The configuration has already passed `runtime_config_is_valid()`. The
    // control-layer constructor keeps a Host/debug assertion for rule drift but
    // avoids duplicating the entire validator in the release image.
    controller.inverter_voltage_model =
        InverterVoltageModel::from_validated_config(inverter_voltage_model_config(&config));
    controller.telemetry = FocTelemetry::default();
}

/// 把 C 的上下文指针转成 `&mut Controller`，并校验哨兵；失败返回 `None`。
/// Converts the C context pointer into `&mut Controller` and checks the sentinel.
///
/// 返回 / Returns: 指针为空、或 `magic != CONTEXT_MAGIC` 时返回 `None`（表示未初
/// 始化或内存已被破坏）；所有 FFI 入口据此返回 `InvalidArgument`，绝不把调用方
/// 内存当作 `Controller` 解释。
/// Returns `None` for a null pointer or a mismatched magic, which every FFI entry
/// point maps to `InvalidArgument`.
///
/// # Safety
/// 调用方必须保证 / The caller must ensure:
///   1. `context` 要么为空，要么指向**对齐且有效**的 `FocRustContextStorage` 存储，
///      其大小至少为 `FOC_RUST_CONTEXT_CAPACITY` 字节。
///   2. 该存储具有静态（或至少覆盖整个控制会话的）生命周期；控制器持有的状态在
///      容器被回收后不再可用。
///   3. 在本次借用的生命周期内，同一上下文没有被线程与中断并发修改：本函数不做
///      任何同步，`&mut` 的唯一性完全依赖"C 侧单一调用者"这一纪律。
///   4. 存储已由 `foc_rust_init()` 初始化（否则 `magic` 校验必然失败）。
///
/// The caller guarantees alignment, validity, lifetime, exclusive access and prior
/// initialization; this function performs no synchronization at all.
///
/// 关于别名的说明 / Aliasing note: 返回引用的生命周期 `'a` 是未约束的，因此 Rust
/// 编译器无法证明唯一性——这正是本函数是 `unsafe fn` 的原因，也是为什么同一上下文
/// 不得被并发使用。
/// The unconstrained `'a` is exactly why this is an `unsafe fn`.
unsafe fn controller_mut<'a>(context: *mut FocRustContextStorage) -> Option<&'a mut Controller> {
    if context.is_null() {
        return None;
    }

    // SAFETY: the C contract requires aligned, zero/static storage initialized by
    // foc_rust_init before every other call. The size/alignment assertions above
    // guarantee that Controller fits in that storage.
    let controller = unsafe { &mut *context.cast::<Controller>() };
    (controller.magic == CONTEXT_MAGIC).then_some(controller)
}

/// 用主机仿真参数覆盖当前模型，**不改变目标机的 C ABI**。
/// Overrides the current model for host simulation without changing the target C ABI.
///
/// 目标固件里本函数不存在（整体被 `#[cfg(not(target_os = "none"))]` 排除），目标端
/// 只能经版本化运行配置安装模型。必须在 `foc_rust_init()` 之后调用；补偿参数先校验，
/// 校验失败返回 `InvalidArgument` 且不写入任何字段。
/// Target firmware never calls this function and therefore keeps the disabled
/// default. The setting must be installed after `foc_rust_init`.
#[cfg(not(target_os = "none"))]
pub fn foc_rust_set_host_dead_time_compensation(
    context: &mut FocRustContextStorage,
    compensation: HostDeadTimeCompensation,
) -> FocStatus {
    if !compensation.is_valid() {
        return FocStatus::InvalidArgument;
    }
    // SAFETY: the mutable reference proves exclusive, aligned storage access.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(model) = InverterVoltageModel::try_new(compensation.model_config()) else {
        return FocStatus::InvalidArgument;
    };
    controller.inverter_voltage_model = model;
    FocStatus::Ok
}

/// 返回编译期 ABI 版本 `[--]`；C 侧启动时与 `FOC_RUST_ABI_VERSION` 比对。
/// Returns the compiled ABI version; C compares it at boot.
///
/// 版本不匹配时 `main.c` 会拒绝运行——这是设计意图，避免新旧组合静默错配。
/// A mismatch makes `main.c` refuse to run; that refusal is intentional.
#[no_mangle]
pub extern "C" fn foc_rust_abi_version() -> u32 {
    FOC_RUST_ABI_VERSION
}

/// 返回 `Controller` 的实际尺寸 `[bytes]`，供 C 确认容器容量够用。
/// Returns the actual controller size `[bytes]` so C can confirm its capacity.
#[no_mangle]
pub extern "C" fn foc_rust_context_required_size() -> u32 {
    size_of::<Controller>() as u32
}

/// 返回 `Controller` 的对齐要求 `[bytes]`；应不超过 C 容器的 8 字节对齐。
/// Returns the controller alignment `[bytes]`; must not exceed the container's.
#[no_mangle]
pub extern "C" fn foc_rust_context_required_align() -> u32 {
    align_of::<Controller>() as u32
}

#[no_mangle]
/// 导出 `[ST]` MCSDK 参考工程的默认配置，供 C 侧取默认值后再改字段。
/// Writes the editable configuration profile derived from the working ST
/// MCSDK project. Nothing in the realtime path reads global constants after
/// `foc_rust_configure`; the caller owns and may version this value.
///
/// 这是 C 侧获取默认值的正确方式：先取回整份默认配置再覆盖需要改的字段，避免手工
/// 填写 30 多个字段时漏填而导致合法性检查失败。配置是值语义的，调用方拥有它并可以
/// 自行版本化。
/// This is how C should obtain defaults: take them, then override only what needs
/// changing.
///
/// 返回 / Returns: `Ok`，或 `config` 为空指针时 `InvalidArgument`。
///
/// # Safety
/// `config` must be null or point to writable `FocRuntimeConfig` storage.
pub unsafe extern "C" fn foc_rust_default_st_config(config: *mut FocRuntimeConfig) -> FocStatus {
    // SAFETY: the caller promises writable C ABI storage or passes null.
    let Some(config) = (unsafe { config.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    *config = default_st_runtime_config();
    FocStatus::Ok
}

#[no_mangle]
/// 计算一份完整、合法运行时配置的规范化 CRC-32/ISO-HDLC。
/// Computes the canonical CRC-32/ISO-HDLC of a complete valid runtime config.
///
/// 这是 Production 参数档的完整性边界：它先复用 `runtime_config_is_valid()`
/// 拒绝无效配置，再按 ABI 字段顺序和小端字节序计算 CRC，因此不依赖
/// C 编译器的填充或 MCU 本地字节序。它只证明字节完整性，不代表参数已
/// 辨识或已通过实机审批。
/// This proves byte-level identity, not motor identification or approval.
///
/// # Safety
/// `config` must be null or point to a readable `FocRuntimeConfig`; `crc_out` must
/// be null or point to writable `u32` storage.
pub unsafe extern "C" fn foc_rust_runtime_config_crc32(
    config: *const FocRuntimeConfig,
    crc_out: *mut u32,
) -> FocStatus {
    let Some(config) = (unsafe { config.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(crc_out) = (unsafe { crc_out.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    if !runtime_config_is_valid(config) {
        return FocStatus::InvalidArgument;
    }
    *crc_out = runtime_config_crc32(config);
    FocStatus::Ok
}

#[no_mangle]
/// 应用一份完整、带版本号的运行时配置；PWM 必须处于关闭状态。
/// Applies a complete versioned runtime configuration while PWM is disabled.
///
/// 校验范围 / Validation: `struct_size`/`config_version`、后端枚举、所有 PI 参数、
/// 频率整除关系、电机参数为正、观测器可靠性门限、两个超时、斜坡与限速速率，以及
/// 启动时序自洽（见 `runtime_config_is_valid()`）。
/// 失败时返回 `InvalidArgument` 且**控制器状态完全不变**。
/// On failure nothing about the controller changes.
///
/// 调用约束 / Constraint: 只能在 `FocState::Disabled` 时调用；运行中改配置会被拒绝
/// （`InvalidArgument`），因为观测器与两个 PI 的 `ts` 都绑在频率上，热切换会让积分
/// 状态失去意义。
/// Must be called while stopped: the observer and both PIs bind their `ts` to the
/// configured frequencies.
///
/// # Safety
/// `context` must be initialized and exclusively accessible; `config` must be
/// null or point to a readable `FocRuntimeConfig`.
pub unsafe extern "C" fn foc_rust_configure(
    context: *mut FocRustContextStorage,
    config: *const FocRuntimeConfig,
) -> FocStatus {
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(config) = (unsafe { config.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state != FocState::Disabled || !runtime_config_is_valid(config) {
        return FocStatus::InvalidArgument;
    }
    configure_runtime(controller, *config);
    FocStatus::Ok
}

#[no_mangle]
/// 在调用方提供的存储中**原位构造**控制器，不做任何内存分配。
/// Initializes caller-owned controller storage without allocating memory.
///
/// 上下文 / Context: 初始化阶段调用一次，必须先于其他任何函数。未调用它时
/// `CONTEXT_MAGIC` 校验失败，所有入口都返回 `InvalidArgument`，不会读到未初始化内存。
/// Call once during initialization, before any other function.
///
/// 返回 / Returns: `Ok`，或 `context` 为空指针时 `InvalidArgument`。
///
/// # Safety
/// `context` must be null or point to writable, correctly aligned
/// `FocRustContextStorage`. The caller must provide exclusive access during the call.
pub unsafe extern "C" fn foc_rust_init(context: *mut FocRustContextStorage) -> FocStatus {
    if context.is_null() {
        return FocStatus::InvalidArgument;
    }

    // SAFETY: context is non-null, properly aligned by its C/Rust ABI type, and
    // Controller is compile-time checked to fit in the caller-owned storage.
    unsafe { ptr::write(context.cast::<Controller>(), Controller::new()) };
    FocStatus::Ok
}

#[no_mangle]
/// 兼容入口：应用旧版基础配置（`FocBasicConfig`）。
/// Copies and validates the parameters used by the Rust `FocBasic` algorithm.
///
/// **仅供兼容与主机测试，实时路径不使用本函数**：它只覆盖两个电流环 PI 与母线
/// 电压，其余参数全部回落到 `[ST]` 默认值，并把观测器强制重建为 `SmoPll` +
/// 单拍 `ts`。真实路径是 `foc_rust_configure()` + `foc_rust_start_realtime()` +
/// `foc_rust_realtime_step()`。
/// NOT used by the realtime path: it only covers the two current PIs and the bus
/// voltage and resets the observer to the default SMO backend.
///
/// 失败语义 / Failure semantics: 参数非法时不仅返回 `InvalidArgument`，还会把
/// `algorithm_configured` 清成假并执行 `controller.stop()`。这是有意的"失效安全"：
/// 配置坏的参数不能留下一个仍可运行、但用着旧参数的控制器。
/// An invalid configuration additionally clears `algorithm_configured` and stops
/// the controller, so a failed reconfiguration cannot leave a runnable controller
/// behind.
///
/// # Safety
/// `context` must have been initialized by `foc_rust_init` and be exclusively
/// accessible. `config` must be null or point to a readable `FocBasicConfig`.
pub unsafe extern "C" fn foc_rust_configure_basic(
    context: *mut FocRustContextStorage,
    config: *const FocBasicConfig,
) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    // SAFETY: the caller promises config points to a readable C ABI value.
    let Some(config) = (unsafe { config.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    if !config_is_valid(config) {
        controller.algorithm_configured = false;
        controller.stop();
        return FocStatus::InvalidArgument;
    }

    controller.params = st_gbm2804_reference_parameters();
    controller.params.id_pi = to_pi_param(config.id_pi);
    controller.params.iq_pi = to_pi_param(config.iq_pi);
    controller.params.motor.nominal_bus_voltage_v = config.nominal_dc_bus_voltage;
    controller.runtime_config = default_st_runtime_config();
    controller.runtime_config.id_pi = config.id_pi;
    controller.runtime_config.iq_pi = config.iq_pi;
    controller.runtime_config.nominal_bus_voltage_v = config.nominal_dc_bus_voltage;
    controller.observer = ConfigurableObserver::new(
        ObserverBackend::SmoPll,
        controller.params.motor,
        1.0 / controller.params.pwm_frequency_hz as f32,
    );
    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.algorithm_configured = true;
    controller.state = FocState::Disabled;
    FocStatus::Ok
}

#[no_mangle]
/// 兼容入口：装入 `[ST]` MCSDK 6.4.1 参考参数并保持停机。
/// Selects the parameters and cascaded-loop topology transcribed from the ST
/// MCSDK 6.4.1 reference project.
///
/// 等价于 `foc_rust_default_st_config()` + `foc_rust_configure()`，保留是为了让既有
/// 测试不退化、并让"只想要参考参数"的调用者少写一步；**实时路径不使用本函数**。
/// Equivalent to `default_st_config()` plus `configure()`; the realtime path uses
/// `foc_rust_configure()` instead.
///
/// 与 `foc_rust_configure()` 的一个差异：本函数不做显式的合法性前置检查，直接走
/// `configure_runtime()`，而当前这份默认配置在 `runtime_config_is_valid()` 下是合法
/// 的。修改默认值时必须保持这一点。
/// It skips the explicit validation step that `foc_rust_configure()` performs, so
/// the built-in defaults must stay valid.
///
/// # Safety
/// `context` must have been initialized and be exclusively accessible.
pub unsafe extern "C" fn foc_rust_configure_st_reference(
    context: *mut FocRustContextStorage,
) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    configure_runtime(controller, default_st_runtime_config());
    FocStatus::Ok
}

#[no_mangle]
/// 兼容入口：进入遗留的 `Running` 状态（不含启动时序）。
/// Enters the running state only when both Rust parameters and the C platform are ready.
///
/// **仅供兼容与主机测试，实时路径不使用本函数**：它不复位观测器与启动时序，直接
/// 进入 `FocState::Running`——只有 `foc_rust_fast_step()`、
/// `foc_rust_open_loop_step()`、`foc_rust_speed_step()` 这三个兼容步进入口认这个
/// 状态。真实路径是 `foc_rust_start_realtime()`（进入 `Alignment`）+
/// `foc_rust_realtime_step()`。
/// NOT used by the realtime path; only the three compatibility step entry points
/// accept the `Running` state.
///
/// `platform_ready` 不是装饰：为 0 时即使 Rust 参数完整也拒绝启动并回到
/// `Disabled`。两道门（`algorithm_configured` 与 `platform_ready`）必须同时满足。
/// Both gates must pass: configuration and the C platform's readiness.
///
/// # Safety
/// `context` must have been initialized by `foc_rust_init` and be exclusively
/// accessible for the duration of the call.
pub unsafe extern "C" fn foc_rust_request_start(
    context: *mut FocRustContextStorage,
    platform_ready: u32,
) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured || platform_ready == 0 {
        controller.state = FocState::Disabled;
        return FocStatus::NotConfigured;
    }

    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.state = FocState::Running;
    FocStatus::Ok
}

#[no_mangle]
/// 启动由 ISR 持有的"启动时序 + 电流控制"运行实例，是**实时路径的启动入口**。
/// Starts the ISR-owned startup/current-control runtime. The requested speed is
/// kept separate from the configured rev-up endpoint so both can be tuned.
///
/// 与 `foc_rust_request_start()` 的区别 / Difference from `request_start`:
/// 本函数复位观测器、启动时序、电流环、速度环与全部计时器，把状态置为
/// `Alignment`，随后由 `foc_rust_realtime_step()` 推进整条启动链。它不进入
/// `Running`，因此三个兼容步进入口在本函数之后都会返回 `Disabled`。
/// It resets the observer, sequencer, current loop, speed loop and every timer and
/// enters `Alignment`; the compatibility step entry points therefore return
/// `Disabled` after it.
///
/// 参数 / Parameters: `target_speed_rpm` 是带方向的闭环目标机械转速 `[rpm]`，必须
/// 有限，且其绝对值落在 `[default_target_speed_rpm, max_speed_rpm]`。Rev-Up 终速是
/// 独立的无感捕获速度，可以高于最终目标；接管后速度斜坡再回到请求值。目标符号同时
/// 决定 Rev-Up、Iq 支撑和观察器捕获方向。
/// `target_speed_rpm` is signed and its magnitude must lie within the application
/// speed range. The independent rev-up endpoint may be higher than the requested
/// speed; the closed-loop ramp returns to the request after handover.
///
/// 调用约束 / Constraint: 必须在 ADC ISR 使能**之前**调用，且调用期间不得有其他
/// 上下文访问该控制器，否则 `configure` 之后的复位序列会与 ISR 竞争。
/// Must be called before the ADC ISR is enabled.
///
/// # Safety
/// `context` must be initialized, exclusively accessible and not concurrently
/// used by the ADC ISR until this function returns.
pub unsafe extern "C" fn foc_rust_start_realtime(
    context: *mut FocRustContextStorage,
    platform_ready: u32,
    target_speed_rpm: f32,
) -> FocStatus {
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured || platform_ready == 0 {
        controller.state = FocState::Disabled;
        return FocStatus::NotConfigured;
    }
    let target_speed_magnitude_rpm = target_speed_rpm.abs();
    if !target_speed_rpm.is_finite()
        || target_speed_magnitude_rpm < controller.params.default_target_speed_rpm
        || target_speed_magnitude_rpm > controller.params.motor.max_speed_rpm
    {
        return FocStatus::InvalidArgument;
    }

    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.observer.reset(0.0);
    controller.observer_counter = 0;
    controller.observer_acquisition_prepared = false;
    controller.observer_hold_recovery_countdown = 0;
    controller.observer_feedback = RotorFeedback::default();
    controller.observer_reliable = false;
    controller.observer_run_reliable = false;
    controller
        .startup
        .reset_with_direction(if target_speed_rpm < 0.0 { -1 } else { 1 });
    controller.previous_pwm = PwmCommand::default();
    controller.current_reference = CurrentCommand::default();
    controller.speed_current_command = CurrentCommand::default();
    controller.target_speed_rpm = target_speed_rpm;
    controller.speed_reference_rpm = 0.0;
    controller.speed_counter = 0;
    controller.observer_wait_elapsed_s = 0.0;
    controller.observer_loss_elapsed_s = 0.0;
    controller.closed_loop_initialized = false;
    controller.telemetry = FocTelemetry::default();
    controller.state = FocState::Alignment;
    FocStatus::Ok
}

#[no_mangle]
/// 执行**恰好一次**完整快环：观测器 -> 启动时序 -> 电流环 -> 可选速度环 -> SVPWM。
/// Executes observer, startup, current loop and optional speed loop exactly
/// once. The C ADC interrupt is the sole caller and therefore the sole owner
/// of mutable controller state while the power stage is armed.
///
/// 这是实时路径的唯一入口（配合 `foc_rust_start_realtime()`），在 **12 kHz 的 ADC
/// 中断**里执行。实时约束：无动态分配、无阻塞、无日志、无 Mutex 等待；三角运算走
/// `PlatformMath`（目标板 CORDIC），`mat` 对象每拍在栈上新建。实测单次调用是 ISR
/// 的主要开销（约 7,300 cycles @170 MHz `[HW]`）。
/// Runs inside the 12 kHz ADC ISR with no allocation, blocking or logging; the
/// math object is rebuilt on the stack every tick.
///
/// 关键语义 / Key semantics: 任何错误返回之前 `output` 都已被清零，调用方不得沿用
/// 上一周期的占空比；本函数不接触硬件，栅极关断仍由 C 平台层负责。`telemetry` 可以
/// 为 NULL，非 NULL 时在每条正常返回路径上被**完整覆盖**（多率时隙那一条除外，见
/// 下文该分支）。`feedback` 与 `output`、`telemetry` 不得指向同一内存。
/// Every error return has already zeroed `output`; `telemetry` may be NULL and is
/// fully overwritten on every normal return path.
///
/// 并发 / Concurrency: 本函数是控制器可变状态的唯一所有者，不做任何加锁或临界区。
/// 若 C 侧的管理线程需要读遥测，只能走 `foc_rust_get_telemetry()`。
/// No locking at all: single-caller discipline is the concurrency model here.
///
/// # Safety
/// `context` must be initialized and exclusively owned by the caller;
/// `feedback` must be readable, `output` writable, and optional `telemetry`
/// writable. The pointed-to objects must not overlap.
///
/// 具体契约 / Concrete contract: `context` 必须已由 `foc_rust_init()` 初始化且非空；
/// 在功率级 arm 期间不得有线程与 ISR 并发修改同一上下文；三个指针要么为 NULL（仅
/// `telemetry` 允许），要么指向有效、对齐、互不重叠的 C ABI 对象。
/// `context` must have been initialized by `foc_rust_init()`, and no thread may
/// mutate it concurrently with the ISR while the power stage is armed.
pub unsafe extern "C" fn foc_rust_realtime_step(
    context: *mut FocRustContextStorage,
    feedback: *const FocFeedback,
    output: *mut FocOutput,
    telemetry: *mut FocTelemetry,
) -> FocStatus {
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    zero_output(output);
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(feedback) = (unsafe { feedback.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    // 先判故障再判配置：粘滞故障优先，即使配置被清掉也要保持 `HardwareFault`，
    // 避免调用方把故障态误读成"只是没配置"。
    // Fault is checked before configuration so a latched fault is never masked.
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    // `Running` 是兼容路径的状态，不属于本实时启动链；只有启动链的五个状态才允许
    // 推进，其余（含 `Disabled`/`Running`）一律返回 `Disabled` 且输出保持为 0。
    // Only the five startup-chain states are accepted here.
    if !matches!(
        controller.state,
        FocState::Alignment
            | FocState::OpenLoopRamp
            | FocState::OpenLoopHold
            | FocState::ObserverTransition
            | FocState::ClosedLoop
    ) {
        return FocStatus::Disabled;
    }
    if !feedback_is_valid(feedback) {
        controller.fault_flags |= FOC_FAULT_INVALID_FEEDBACK;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    let mut snapshot = FeedbackSnapshot {
        currents: PhaseCurrents {
            a: feedback.phase_current_a,
            b: feedback.phase_current_b,
            c: feedback.phase_current_c,
        },
        dc_bus_voltage: feedback.dc_bus_voltage,
        rotor: RotorFeedback::default(),
    };
    let mut math = PlatformMath::default();
    // One ADC current snapshot has exactly one Clarke transform. The observer,
    // handoff initializer and current loop all consume this shared result.
    let current_alpha_beta = clarke(Abc {
        a: feedback.phase_current_a,
        b: feedback.phase_current_b,
        c: feedback.phase_current_c,
    });
    // 模型总门打开时，每个控制拍只更新一次电流极性滤波；观测器修正和 PWM 前馈
    // 共享这一状态。默认总门为 0，所以既不更新滤波，也不改变任何数值输出。
    // With the master gate on, update polarity filtering exactly once per control
    // tick and share it between observer correction and PWM feed-forward.
    if controller.inverter_voltage_model.config().enabled {
        controller
            .inverter_voltage_model
            .update_phase_currents(snapshot.currents);
    }
    // 电流环周期 `[s]`，即 ADC ISR 周期（12 kHz -> 约 83.3 us）。所有"按时间积分"
    // 的量（启动时序、斜坡、超时）都用它，所以它必须与平台 ISR 频率逐位一致，
    // 配置校验里有对应检查。
    // The ADC ISR period [s]; every time-integrated quantity uses it.
    let dt_s = 1.0 / controller.params.pwm_frequency_hz as f32;
    let observer_enabled = controller.runtime_config.observer_enable != 0;
    // 反电势观测器在静止对齐段没有可观测信息。若仍逐拍运行，ADC 零点噪声与
    // command-model 残差会在整整一个 alignment 周期内积进 SMO/PLL；实机已经观察到
    // 升速前出现约 +/-170 rpm 的虚假估速，并会随机锁进反向解。对齐期间保持启动时
    // 的 reset 状态，进入 OpenLoopRamp 后再从已知强制角开始建立 BEMF。
    // A back-EMF observer has no observable information during standstill alignment.
    // Running it there lets current-offset noise and command-model residuals seed the
    // SMO/PLL for a full alignment interval and can select the reverse-speed solution.
    // Keep the start-time reset state until OpenLoopRamp begins.
    let observer_active_phase = controller.state != FocState::Alignment;
    // 强拖方向由带符号目标确定。把它作为捕获约束交给 PLL，防止低速 BEMF 尚不稳定时
    // 锁入相反方向；ClosedLoop 接管后清零约束，使运行反馈仍保留双向能力。
    // The signed target constrains PLL acquisition to the forced direction. Clear
    // the constraint after handover so normal feedback remains bidirectional.
    let acquisition_direction = if matches!(
        controller.state,
        FocState::OpenLoopRamp | FocState::OpenLoopHold | FocState::ObserverTransition
    ) {
        if controller.target_speed_rpm < 0.0 {
            -1
        } else {
            1
        }
    } else {
        0
    };
    controller
        .observer
        .set_acquisition_direction(acquisition_direction);
    if observer_enabled
        && controller.state == FocState::OpenLoopHold
        && !controller.observer_reliable
    {
        if controller.observer_hold_recovery_countdown == 0 {
            // 终速 BEMF 已可观时，从同拍强制角/电速度/电流重新捕获。实机存在稳定的
            // 错误吸引域，所以在总获取超时内有界重试；总超时仍累计，最终失败
            // 一定故障停机。可信度门本身需要 64 ms 统计窗口，原 200 ms 重试只留下
            // 约 136 ms 相位收敛时间，实机会在即将收敛时反复清空 PLL。单零点鉴相器
            // 消除了 180 度假锁后，约 333 ms 既能保留重试，也给每次捕获完整收敛时间；
            // 同时不会与默认 500 ms 的总获取超时撞在同一拍。
            // The 64 ms reliability window left only about 136 ms to converge with
            // the former 200 ms retry. The single-zero discriminator removes the
            // 180-degree false lock, so an approximately 333 ms interval retains
            // bounded retries without colliding with the default 500 ms deadline.
            let forced_electrical_speed_rad_s = controller.runtime_config.startup_final_speed_rpm
                * controller.target_speed_rpm.signum()
                * PI
                / 30.0
                * controller.params.motor.pole_pairs as f32;
            // The SMO backend preserves an already-usable BEMF vector and realigns
            // only its PLL; with weak BEMF the same API falls back to a clean reset.
            controller.observer.prepare_acquisition_at_speed(
                controller.telemetry.forced_electrical_angle_rad,
                forced_electrical_speed_rad_s,
                current_alpha_beta,
            );
            controller.observer_counter = 0;
            controller.observer_feedback = RotorFeedback::default();
            controller.observer_reliable = false;
            controller.observer_run_reliable = false;
            controller.observer_hold_recovery_countdown =
                (controller.params.pwm_frequency_hz / 3).max(1);
        } else {
            controller.observer_hold_recovery_countdown -= 1;
        }
    }
    if observer_enabled && observer_active_phase && !controller.observer_acquisition_prepared {
        // 上一拍已经由 startup 把状态推进到 OpenLoopRamp；此时 telemetry 中保存的
        // forced angle 是对齐结束边界的已知角度。必须在第一次 update 前用同拍实测
        // 电流预置内部电流模型，避免 reset=0 与对齐电流之间的人为阶跃。
        // The previous tick advanced startup into OpenLoopRamp. Before the first
        // observer update, seed its current model from this exact ADC sample and the
        // known forced angle at the alignment boundary.
        controller.observer.prepare_acquisition(
            controller.telemetry.forced_electrical_angle_rad,
            current_alpha_beta,
        );
        controller.observer_acquisition_prepared = true;
        // Keep the ramp-trained observer alive for one full retry interval before
        // forcing a clean high-speed acquisition. With the old immediate reset the
        // useful phase/variance state collected during Rev-Up was always discarded at
        // the hold boundary, even when it was already close to the acquisition gate.
        controller.observer_hold_recovery_countdown =
            (controller.params.pwm_frequency_hz / 3).max(1);
    }
    let observer_due =
        observer_enabled && observer_active_phase && controller.observer_counter == 0;
    if observer_due {
        let observer_pwm = controller
            .inverter_voltage_model
            .observer_equivalent_pwm(controller.previous_pwm, snapshot.dc_bus_voltage);
        // 观测器吃的是**上一拍**真正下发的 PWM：本拍的占空比要到本函数末尾才算出来，
        // 用本拍值会造成代数环。分频 > 1 时 `previous_pwm` 仍是最近一次实际下发值，
        // 只是它对应的周期已经过去了 N 个，观测器的电压项因此偏旧——这是多率路径的
        // 已知代价，也是分频上限设为 32 的原因之一。
        // The observer consumes the previous tick's PWM; its voltage term is
        // therefore stale by `divider` periods on the multi-rate path.
        controller.observer_feedback = controller.observer.update_from_alpha_beta_with_math(
            &snapshot,
            current_alpha_beta,
            observer_pwm,
            &mut math,
        );
        controller.observer_reliable = controller.observer.is_reliable();
        controller.observer_run_reliable = controller.observer.is_reliable_for_run(
            controller
                .runtime_config
                .observer_run_reliability
                .minimum_speed_rpm,
            controller
                .runtime_config
                .observer_run_reliability
                .maximum_phase_error_rad,
        );
    }
    if observer_enabled && observer_active_phase {
        controller.observer_counter += 1;
        if controller.observer_counter >= controller.runtime_config.observer_update_divider {
            controller.observer_counter = 0;
        }
    }
    let observer_diagnostics = controller.observer.diagnostics();
    // 本拍内部统一用这两个局部量：观测器输出与可靠性在同一拍内不会再变，取局部副本
    // 可以避免中途误读被后续分支改写的字段。
    // Local copies keep the observer result consistent across the branches below.
    let observer_feedback = controller.observer_feedback;
    let observer_acquisition_reliable = controller.observer_reliable;
    let observer_run_reliable = controller.observer_run_reliable;
    /* 进入交接前必须通过带 64 ms 方差与连续确认计数的严格获取门；一旦已经进入
     * ObserverTransition，则改用运行保持门。若渐变期间继续要求“重新获取”，任一
     * 方差窗口抖动都会把已经开始的渐变反复打回开环，实机永远无法完成接管。
     * Acquisition requires the strict window-and-streak gate. Once transition has
     * started, retain it with the run gate; otherwise a single variance-window
     * wobble repeatedly rolls an otherwise valid handover back to open loop. */
    let handoff_gate_reliable = if controller.state == FocState::ObserverTransition {
        observer_run_reliable
    } else {
        observer_acquisition_reliable
    };
    // The public SMO can briefly overestimate speed during the final part of the
    // forced ramp and satisfy its own 524 rpm gate before the commanded trajectory
    // actually reaches that speed. Keep learning throughout the ramp, but arm the
    // handoff only from OpenLoopHold; once started, ObserverTransition retains it
    // with the run gate above.
    let handoff_phase_ready = matches!(
        controller.state,
        FocState::OpenLoopHold | FocState::ObserverTransition
    );
    let handoff_ready = controller.runtime_config.closed_loop_enable != 0
        && handoff_phase_ready
        && handoff_gate_reliable;
    // 交接电流 `[A]`：只在"即将进入交接"的那一拍、在**观测器坐标系**下把当前三相电流
    // 投影成 Iq，并按显式配置选择与目标方向一致的确定转矩目标，作为转矩电流斜坡的终点。比例 0
    // 是 MCSDK 原行为；非零比例为公开 SMO 保留有界且可复现的激励。交接后不再重算，
    // 覆盖速度环给定，速度环永远无法积累积分。
    // Capture observer-frame Iq once and optionally replace it by a deterministic,
    // bounded direction-aware torque target. Ratio zero is the exact MCSDK path.
    let observer_iq_a = if handoff_ready
        && !matches!(
            controller.state,
            FocState::ObserverTransition | FocState::ClosedLoop
        ) {
        // 这是整个启动过程中只执行一次的预装投影。若再走 PlatformMath，会在同一
        // 接管拍与观察器 atan2、随后电流环 sin/cos 串行占用三次 CORDIC，实机最坏
        // 拍已越过 12,500-cycle 软件截止。快速近似的角误差远小于 ADC/观测误差，
        // 只影响渐变终点数 mA，不进入常态 Park/逆 Park。
        // This one-shot preload projection uses the regression-bounded fast
        // approximation so the handover tick does not serialize a third CORDIC
        // transaction. Steady-state Park/inverse-Park remain hardware accelerated.
        let (sin, cos) = foc_algorithm::fast_sin_cos(observer_feedback.electrical_angle_rad);
        let measured_iq_a = -current_alpha_beta.alpha * sin + current_alpha_beta.beta * cos;
        // ST's switch-over endpoint is the measured observer-frame Iq. A public SMO
        // can need bounded torque excitation through the first handoff, especially
        // at no load where that projection may be nearly zero or even braking.
        // Reuse the existing preload ratio as an explicit opt-in: ratio 0 keeps the
        // exact reference behaviour; ratio 1 keeps the already-safe rev-up current.
        supported_handoff_iq(
            measured_iq_a,
            controller.runtime_config.startup_current_a * controller.target_speed_rpm.signum(),
            controller.runtime_config.handoff_torque_support_ratio,
        )
    } else {
        controller.current_reference.iq_ref_a
    };
    let startup = controller.startup.update(
        dt_s,
        observer_feedback.electrical_angle_rad,
        handoff_ready,
        observer_iq_a,
        controller.runtime_config.closed_loop_current_slew_a_per_s,
    );
    let observer_controls = matches!(
        startup.phase,
        RevUpPhase::ObserverTransition | RevUpPhase::ClosedLoop
    );
    // 524 rpm is the acquisition threshold inherited from OBS_MINIMUM_SPEED_RPM.
    // Once the observer owns the angle, the separately configured retention gate
    // applies (0 rpm by default, matching MIN_APPLICATION_SPEED_RPM in the ST
    // reference). BEMF, variance and finite-value gates remain mandatory.
    let observer_reliable = if observer_controls {
        observer_run_reliable
    } else {
        observer_acquisition_reliable
    };

    // 获取超时：只在"转速已到升速终点、正在等观测器过门"的开环保持段计时。此时转子
    // 已被开环拖动、电流仍在施加，若观测器始终不收敛就必须故障停机，否则会一直空转
    // 发热。超时后锁存 `FOC_FAULT_OBSERVER_STARTUP` 并进入 `Fault`。
    // Acquisition timeout: counts only while waiting for the observer in
    // open-loop hold, so a non-converging observer cannot spin forever.
    if controller.runtime_config.closed_loop_enable != 0
        && matches!(
            startup.phase,
            RevUpPhase::OpenLoopHold | RevUpPhase::ObserverTransition
        )
    {
        controller.observer_wait_elapsed_s += dt_s;
        if controller.observer_wait_elapsed_s
            >= controller.runtime_config.observer_acquisition_timeout_s
        {
            controller.fault_flags |= FOC_FAULT_OBSERVER_STARTUP;
            controller.state = FocState::Fault;
            return FocStatus::HardwareFault;
        }
    } else {
        controller.observer_wait_elapsed_s = 0.0;
    }

    // 失锁超时：只在观测器**已经在提供控制角**之后计时。允许短暂失锁而不立刻故障，
    // 靠 `observer_loss_timeout_s` 区分瞬态抖动与真的丢转子；一旦恢复即清零，所以正常
    // 运行时这个计时器几乎恒为 0。超时后锁存 `FOC_FAULT_OBSERVER_LOST`。
    // Loss-of-lock timeout: only while the observer owns the angle; a transient
    // drop resets the timer, so normal operation keeps it near zero.
    if observer_controls {
        if observer_reliable {
            controller.observer_loss_elapsed_s = 0.0;
        } else {
            controller.observer_loss_elapsed_s += dt_s;
            if controller.observer_loss_elapsed_s
                >= controller.runtime_config.observer_loss_timeout_s
            {
                controller.fault_flags |= FOC_FAULT_OBSERVER_LOST;
                controller.state = FocState::Fault;
                return FocStatus::HardwareFault;
            }
        }
    } else {
        controller.observer_loss_elapsed_s = 0.0;
    }

    // 控制角的选择是本函数最关键的一处：交接/闭环阶段用 `angle_for_control_rad`
    // （交接段是强制角到观测角的最短路径混合，闭环段就是观测角），开环阶段用强制角
    // 且机械转速置 0（开环没有可信转速，速度环也不该被调用）。
    // 注意闭环阶段即使观测器**当拍**失锁，这里仍然用 `angle_for_control_rad` 而不是
    // 立刻退回强制角：保持角度连续，失锁由上面的定时故障处理，避免一次抖动就产生
    // 角度跳变而失步。
    // The control angle stays continuous on a transient observer dropout: the timed
    // observer-loss fault handles it instead of jumping back to the forced angle.
    snapshot.rotor = if observer_controls {
        RotorFeedback {
            // During ObserverTransition this is the shortest-path blend from
            // forced angle to observer angle. In ClosedLoop it equals the
            // observer angle. A transient reliability drop keeps this continuous
            // and is handled by the timed observer-loss fault above.
            electrical_angle_rad: startup.angle_for_control_rad,
            mechanical_speed_rad_s: observer_feedback.mechanical_speed_rad_s,
        }
    } else {
        RotorFeedback {
            electrical_angle_rad: startup.forced_electrical_angle_rad,
            mechanical_speed_rad_s: 0.0,
        }
    };

    // 电流给定的产生 / Producing the current reference:
    // 闭环相位的处理与开环不同，分三种情况。
    // In closed loop the reference is produced in three cases.
    //
    // 闭环首拍 / First closed-loop tick: 用观测器估计的机械转速 `[rad/s]` 换算出
    // `[rpm]` 作为转速斜坡的起点（`rad/s * 30 / π`），并用 MCSDK 的
    // `SWITCH_OVER` 思路给速度 PI 做一次预装，让积分器一开始就对应"交接瞬间正在流动
    // 的转矩电流"，避免速度环从零积分重新爬升导致转速跌落。
    // The first tick seeds the speed ramp from the estimated speed and preloads the
    // speed PI, mirroring MCSDK's `SWITCH_OVER`.
    //
    // 预装量 = `交接电流 * speed_pi_preload_ratio` `[A]`。本工程 `[FW]` 默认比例
    // 0.0（参考工程 `PID_SPEED_INTEGRAL_INIT_DIV = 0`），所以实际预装为 0；比例是
    // 留给"换成使用积分预装的参考工程"的开关。
    // The preload is `Iq_at_handover * ratio`; the `[FW]` ratio is 0.0 here.
    //
    // 随后立刻把 `current_reference` 覆盖成 `startup.current_reference`，也就是
    // **保留交接最后一拍的电流不变**，否则首拍就会出现一个电流阶跃（转矩冲击）。
    // 真正向速度 PI 输出靠拢由下面的限速器逐拍完成。
    // The applied reference is then held exactly at the switch-over value so the
    // first tick has no torque step; the slew limiter closes the gap afterwards.
    //
    // 之后的每一拍 / Later ticks: 速度环只在分频计数归零时执行一次，其余拍沿用上一
    // 次的 `speed_current_command`。转速斜坡步长按 `rate / speed_loop_frequency_hz`
    // 计算 `[rpm]`，保证斜坡速率与速度环频率无关。
    // Later ticks run the speed loop only when the divider counter is zero.
    //
    // 无论是否执行速度环，**每一拍**都推进 Iq/Id 限速器：步长
    // `closed_loop_current_slew_a_per_s * dt_s` `[A]`，把实际施加的给定平滑地推向
    // 速度环输出。这是"1 kHz 速度环 + 12 kHz 电流环"之间不产生转矩阶跃的关键。
    // The Iq/Id slew runs every tick, which is what keeps a 1 kHz speed loop from
    // stepping a 12 kHz current loop.
    controller.current_reference = if startup.phase == RevUpPhase::ClosedLoop {
        let divider =
            (controller.params.pwm_frequency_hz / controller.params.speed_loop_frequency_hz).max(1);
        if !controller.closed_loop_initialized {
            controller.speed_reference_rpm = observer_feedback.mechanical_speed_rad_s * 30.0 / PI;
            controller.speed_current_command = controller.speed_loop.preload(
                &controller.params,
                SpeedCommand {
                    target_rpm: controller.speed_reference_rpm,
                    id_ref_a: 0.0,
                },
                observer_feedback.mechanical_speed_rad_s,
                startup.current_reference.iq_ref_a
                    * controller.runtime_config.speed_pi_preload_ratio,
            );
            // Preserve the final switch-over current exactly on the first
            // closed-loop sample. The applied reference then slews toward the
            // speed PI output without a one-tick torque step.
            controller.current_reference = startup.current_reference;
            controller.speed_counter = 0;
            controller.closed_loop_initialized = true;
        } else if controller.speed_counter == 0 {
            let maximum_step = controller.runtime_config.closed_loop_speed_ramp_rpm_per_s
                / controller.params.speed_loop_frequency_hz as f32;
            controller.speed_reference_rpm = move_towards(
                controller.speed_reference_rpm,
                controller.target_speed_rpm,
                maximum_step,
            );
            controller.speed_current_command = controller.speed_loop.update(
                &controller.params,
                SpeedCommand {
                    target_rpm: controller.speed_reference_rpm,
                    id_ref_a: 0.0,
                },
                observer_feedback.mechanical_speed_rad_s,
            );
        }
        controller.speed_counter = (controller.speed_counter + 1) % divider;
        let maximum_current_step =
            controller.runtime_config.closed_loop_current_slew_a_per_s * dt_s;
        controller.current_reference.id_ref_a = move_towards(
            controller.current_reference.id_ref_a,
            controller.speed_current_command.id_ref_a,
            maximum_current_step,
        );
        controller.current_reference.iq_ref_a = move_towards(
            controller.current_reference.iq_ref_a,
            controller.speed_current_command.iq_ref_a,
            maximum_current_step,
        );
        controller.state = FocState::ClosedLoop;
        controller.current_reference
    } else {
        // 开环阶段：给定完全来自启动时序（对齐/升速/保持/交接各自的电流与角度），
        // 状态机的外部状态随之跟随 `startup.phase`。这里不调用速度环，所以开环段
        // 不会因为观测器不可信而被速度 PI 带偏。
        // Open-loop phases take their reference from the sequencer only.
        controller.state = match startup.phase {
            RevUpPhase::Alignment => FocState::Alignment,
            RevUpPhase::OpenLoopRamp => FocState::OpenLoopRamp,
            RevUpPhase::OpenLoopHold => FocState::OpenLoopHold,
            RevUpPhase::ObserverTransition => FocState::ObserverTransition,
            RevUpPhase::ClosedLoop => FocState::ClosedLoop,
        };
        startup.current_reference
    };

    // The observer owns one scheduled slot and the power stage holds the last
    // valid PWM command for that single carrier period. This optional
    // multi-rate path is retained for future higher-frequency board profiles;
    // divider=1 executes observer and current controller in the same slot.
    //
    // 多率时隙 / Multi-rate slot: 分频 > 1 时，观测器所在那一拍**跳过电流环**，直接
    // 复用 `previous_pwm` 作为本拍输出。原因是同一拍既要跑观测器（含 PLL 与滑模项）
    // 又要跑电流环会超出高 PWM 频率下的时间预算；用保持一拍占空比换取确定性执行时间。
    // 代价与注意事项 / Cost and caveats: 这一拍没有新的电流环输出，因此遥测里的
    // `id_measured_a`/`iq_measured_a`/`vd_command_v`/`vq_command_v` **不会被刷新**
    // （保持上一拍的值），而 `observer_*` 与 `target_speed_rpm` 等字段会被更新——遥测
    // 快照在同一拍内因此是字段级"新旧混合"的，读侧不要假设所有字段同源。
    // On a skipped slot the measured-current and voltage telemetry fields keep their
    // previous values while the observer fields are refreshed; the snapshot is
    // therefore mixed-age by field.
    //
    // 分频为 1（本工程 `[FW]` 默认）时这个分支永不进入，观测器与电流环在同一时隙内
    // 完成，`previous_pwm` 的语义退化为"上一拍的输出"。
    // With divider = 1 this branch never runs.
    if observer_due && controller.runtime_config.observer_update_divider > 1 {
        let pwm = controller.previous_pwm;
        output.duty_a = pwm.duty_a;
        output.duty_b = pwm.duty_b;
        output.duty_c = pwm.duty_c;
        controller.telemetry.state = controller.state as u32;
        controller.telemetry.observer_backend = controller.observer.backend() as u32;
        controller.telemetry.observer_reliable = observer_reliable as u32;
        controller.telemetry.closed_loop_active = (controller.state == FocState::ClosedLoop) as u32;
        controller.telemetry.target_speed_rpm = controller.target_speed_rpm;
        controller.telemetry.measured_speed_rpm =
            observer_feedback.mechanical_speed_rad_s * 30.0 / PI;
        controller.telemetry.electrical_angle_rad = snapshot.rotor.electrical_angle_rad;
        controller.telemetry.forced_electrical_angle_rad = startup.forced_electrical_angle_rad;
        controller.telemetry.observer_electrical_angle_rad = observer_feedback.electrical_angle_rad;
        controller.telemetry.observer_bemf_alpha_v = observer_diagnostics.bemf_alpha_v;
        controller.telemetry.observer_bemf_beta_v = observer_diagnostics.bemf_beta_v;
        controller.telemetry.observer_pll_phase_error_rad =
            observer_diagnostics.pll_phase_error_rad;
        controller.telemetry.observer_speed_mean_rpm = observer_diagnostics.speed_mean_rpm;
        controller.telemetry.observer_speed_variance_rpm2 =
            observer_diagnostics.speed_variance_rpm2;
        controller.telemetry.observer_reliability_flags = observer_diagnostics.reliability_flags;
        controller.telemetry.observer_reliable_samples = observer_diagnostics.reliable_samples;
        controller.telemetry.observer_wait_elapsed_s = controller.observer_wait_elapsed_s;
        controller.telemetry.observer_loss_elapsed_s = controller.observer_loss_elapsed_s;
        controller.telemetry.id_reference_a = controller.current_reference.id_ref_a;
        controller.telemetry.iq_reference_a = controller.current_reference.iq_ref_a;
        if let Some(telemetry) = unsafe { telemetry.as_mut() } {
            *telemetry = controller.telemetry;
        }
        return FocStatus::Ok;
    }

    // 角度延迟补偿以本拍电角速度换算：机械 `[rad/s] * 极对数 * dt [s]` 得到
    // “一个控制拍”的电角增量 `[rad]`，再乘配置的预测拍数。开环没有可信估计转速，
    // 使用强拖轨迹速度；交接/闭环使用观测器速度。0/0 默认配置会产生精确的 0.0，
    // 电流环因此走逐位兼容的零偏移路径。
    // One control-tick electrical advance is mechanical speed * pole pairs * dt.
    // Open loop uses the forced trajectory speed; handover/closed loop uses the
    // observer estimate. The 0/0 default reaches the bit-compatible zero path.
    let angle_compensation = controller.runtime_config.angle_compensation;
    let angle_offsets = if angle_compensation.park_prediction_ticks == 0.0
        && angle_compensation.reverse_park_prediction_ticks == 0.0
    {
        // 默认 0/0 不做速度换算，避免为一个关闭的能力消耗热路径乘法。
        ControlAngleOffsets::default()
    } else {
        let compensation_mechanical_speed_rad_s = if observer_controls {
            observer_feedback.mechanical_speed_rad_s
        } else {
            startup.forced_speed_rpm * PI / 30.0
        };
        let electrical_angle_per_control_tick_rad =
            compensation_mechanical_speed_rad_s * controller.params.motor.pole_pairs as f32 * dt_s;
        ControlAngleOffsets {
            park_rad: electrical_angle_per_control_tick_rad
                * angle_compensation.park_prediction_ticks,
            reverse_park_rad: electrical_angle_per_control_tick_rad
                * angle_compensation.reverse_park_prediction_ticks,
        }
    };

    // 电流环：Park -> dq PI（含抗饱和与圆限幅）-> 逆 Park -> SVPWM，全部由
    // `foc-control` 完成，本层只负责把共享的 Clarke 结果与当前给定传进去。
    // 传入 `current_alpha_beta` 而不是让电流环自己再算一次 Clarke，是为了保证观测器与
    // 电流环看到**完全同一组** αβ，否则两者之间的微小相位差会表现为转矩纹波。
    // The current loop reuses the shared Clarke result so that the observer and the
    // loop see exactly the same αβ.
    let (pwm, control) = controller
        .current_loop
        .update_from_alpha_beta_with_angle_offsets_and_math(
            &controller.params,
            &snapshot,
            current_alpha_beta,
            controller.current_reference,
            angle_offsets,
            &mut math,
        );
    // 输出合法性检查：非有限或超出 `[0,1]` 一律锁存 `FOC_FAULT_ALGORITHM_OUTPUT`
    // 并进入 `Fault`。这是"最后一道算法侧防线"——C 侧会把非法占空比直接写进比较
    // 寄存器，所以这里必须在写输出之前拦住。
    // A non-finite or out-of-range duty latches a fault before anything reaches C.
    if !pwm.is_valid() {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    // 可选的死区/器件压降前馈。补偿会把占空比推近边界，`feedforward_pwm()` 内部虽已钳到 `[0,1]`，
    // 但第二道检查仍然保留，防止将来补偿逻辑改动后越界值被静默下发。
    // The second validity check guards the compensated duty before it reaches C.
    let pwm = controller
        .inverter_voltage_model
        .feedforward_pwm(pwm, snapshot.dc_bus_voltage);
    if !pwm.is_valid() {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    // `previous_pwm` 必须保存**实际下发**的值（补偿之后），因为下一拍观测器用它重构
    // 相电压；保存补偿前的值会让观测器的电压项与真实相电压系统性偏差。
    // `previous_pwm` stores the value actually issued so the observer reconstructs
    // the real phase voltage next tick.
    controller.previous_pwm = pwm;
    output.duty_a = pwm.duty_a;
    output.duty_b = pwm.duty_b;
    output.duty_c = pwm.duty_c;
    // 遥测整体赋值（而不是逐字段改），保证除下面字段外的内容不会残留旧值；`measured_`
    // 转速由 `rad/s` 换算成 `[rpm]`（`* 30 / π`），与目标转速同量纲便于对比。
    // Telemetry is assigned as a whole; the estimated speed is converted from
    // `[rad/s]` to `[rpm]` so it can be compared with the target directly.
    controller.telemetry = FocTelemetry {
        state: controller.state as u32,
        observer_backend: controller.observer.backend() as u32,
        observer_reliable: observer_reliable as u32,
        closed_loop_active: (controller.state == FocState::ClosedLoop) as u32,
        target_speed_rpm: controller.target_speed_rpm,
        measured_speed_rpm: observer_feedback.mechanical_speed_rad_s * 30.0 / PI,
        electrical_angle_rad: snapshot.rotor.electrical_angle_rad,
        id_reference_a: controller.current_reference.id_ref_a,
        iq_reference_a: controller.current_reference.iq_ref_a,
        id_measured_a: control.current_dq.d,
        iq_measured_a: control.current_dq.q,
        vd_command_v: control.voltage_dq.d,
        vq_command_v: control.voltage_dq.q,
        forced_electrical_angle_rad: startup.forced_electrical_angle_rad,
        observer_electrical_angle_rad: observer_feedback.electrical_angle_rad,
        observer_bemf_alpha_v: observer_diagnostics.bemf_alpha_v,
        observer_bemf_beta_v: observer_diagnostics.bemf_beta_v,
        observer_pll_phase_error_rad: observer_diagnostics.pll_phase_error_rad,
        observer_speed_mean_rpm: observer_diagnostics.speed_mean_rpm,
        observer_speed_variance_rpm2: observer_diagnostics.speed_variance_rpm2,
        observer_reliability_flags: observer_diagnostics.reliability_flags,
        observer_reliable_samples: observer_diagnostics.reliable_samples,
        observer_wait_elapsed_s: controller.observer_wait_elapsed_s,
        observer_loss_elapsed_s: controller.observer_loss_elapsed_s,
        voltage_limited: control.voltage_limited as u32,
    };
    // `telemetry` 允许为 NULL；非 NULL 时才写回调用方，写的是本拍刚算出的快照。
    // A NULL `telemetry` is legal; the snapshot is only copied when it is present.
    if let Some(telemetry) = unsafe { telemetry.as_mut() } {
        *telemetry = controller.telemetry;
    }
    FocStatus::Ok
}

#[no_mangle]
/// 复制最近一次实时遥测快照，供管理线程读取。
/// Copies the last realtime snapshot.
///
/// 语义 / Semantics: 返回的是**上一拍**写下的 `controller.telemetry`，不是即时重算
/// 的结果，因此字段之间可能来自不同拍（多率时隙尤其明显）。
/// It returns the snapshot written on a previous tick, not a fresh computation.
///
/// 实时约束 / Real-time constraints: 本函数本身不在 ISR 里执行，但它与 ISR 共享
/// `telemetry` 字段，而本层不做任何加锁或临界区（头文件写的"内部短临界区"在本层
/// 并未实现），因此 C 侧**必须**自己用短临界区包裹调用，否则会与 12 kHz 中断竞争并
/// 读到跨拍混合值。仍依赖调用方的单一写者纪律。
/// The C side MUST wrap this call in a short critical section: this layer takes no
/// lock and implements no critical section, so an unwrapped call races the 12 kHz ISR
/// and can read a torn mixture of samples.
///
/// 返回 / Returns: `Ok`，或 `context`/`telemetry` 为空指针、未初始化时
/// `InvalidArgument`。
///
/// # Safety
/// `context` must be initialized and not concurrently mutated; `telemetry`
/// must be null or point to writable C ABI storage.
pub unsafe extern "C" fn foc_rust_get_telemetry(
    context: *mut FocRustContextStorage,
    telemetry: *mut FocTelemetry,
) -> FocStatus {
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(telemetry) = (unsafe { telemetry.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    *telemetry = controller.telemetry;
    FocStatus::Ok
}

#[no_mangle]
/// 停机：复位电流环、速度环、观测器与启动时序，状态回到 `Disabled`。
/// Stops control and resets all algorithm state.
///
/// 本函数**不接触硬件**：栅极使能与 PWM 通道由 C 平台层负责关闭。它也不清除
/// `fault_flags`（故障位粘滞），并且对空指针/未初始化上下文是**静默无操作**——
/// 因此可以在清理路径上无条件调用。
/// It touches no hardware, does not clear fault bits, and is a silent no-op for a
/// null or uninitialized context so it can be called unconditionally during cleanup.
///
/// # Safety
/// `context` must be null or an initialized context with exclusive access.
pub unsafe extern "C" fn foc_rust_stop(context: *mut FocRustContextStorage) {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    if let Some(controller) = unsafe { controller_mut(context) } {
        controller.stop();
    }
}

#[no_mangle]
/// 兼容入口：执行一次不含观测器与速度环的纯电流控制步，**实时路径不使用本函数**。
/// Executes one allocation-free current-control step.
///
/// 与 `foc_rust_realtime_step()` 的差异 / Difference from the realtime step:
/// 它只认 `FocState::Running`（由 `foc_rust_request_start()` 设置），需要调用方在
/// `FocReference` 里显式给出 Id/Iq 给定，不跑观测器、不跑启动时序、不跑速度环，也
/// **不更新遥测**。`FeedbackSnapshot.rotor.mechanical_speed_rad_s` 被强制填 0。
/// It only accepts `Running`, takes explicit Id/Iq references, runs no observer,
/// sequencer or speed loop, and updates no telemetry.
///
/// 失败语义 / Failure semantics: `output` 在任何错误返回前已清零；电机参数非法或
/// 输出越界会锁存 `FOC_FAULT_ALGORITHM_OUTPUT` 并进入 `Fault`。
///
/// # Safety
/// `context` must be initialized and exclusively accessible. `feedback` and
/// `reference` must be null or readable, and `output` must be null or writable.
/// No pointed-to object may alias another mutable object during the call.
pub unsafe extern "C" fn foc_rust_fast_step(
    context: *mut FocRustContextStorage,
    feedback: *const FocFeedback,
    reference: *const FocReference,
    output: *mut FocOutput,
) -> FocStatus {
    // SAFETY: output must point to writable C ABI storage.
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    zero_output(output);

    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    if controller.state != FocState::Running {
        return FocStatus::Disabled;
    }

    // SAFETY: the caller promises readable C ABI inputs for this call.
    let Some(feedback) = (unsafe { feedback.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    // SAFETY: the caller promises readable C ABI inputs for this call.
    let Some(reference) = (unsafe { reference.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    if !feedback_is_valid(feedback) || !reference_is_valid(reference) {
        return FocStatus::InvalidArgument;
    }

    let mut math = PlatformMath::default();
    let (pwm, _) = controller.current_loop.update_with_math(
        &controller.params,
        &FeedbackSnapshot {
            currents: PhaseCurrents {
                a: feedback.phase_current_a,
                b: feedback.phase_current_b,
                c: feedback.phase_current_c,
            },
            dc_bus_voltage: feedback.dc_bus_voltage,
            rotor: RotorFeedback {
                electrical_angle_rad: feedback.electrical_angle_rad,
                mechanical_speed_rad_s: 0.0,
            },
        },
        CurrentCommand {
            id_ref_a: reference.id_ref,
            iq_ref_a: reference.iq_ref,
        },
        &mut math,
    );

    if !pwm.duty_a.is_finite()
        || !pwm.duty_b.is_finite()
        || !pwm.duty_c.is_finite()
        || !(0.0..=1.0).contains(&pwm.duty_a)
        || !(0.0..=1.0).contains(&pwm.duty_b)
        || !(0.0..=1.0).contains(&pwm.duty_c)
    {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    output.duty_a = pwm.duty_a;
    output.duty_b = pwm.duty_b;
    output.duty_c = pwm.duty_c;
    FocStatus::Ok
}

#[no_mangle]
/// 兼容入口：为板上首次调试生成一个**刻意限压**的开环电压矢量，实时路径不使用本函数。
/// Generates one deliberately voltage-limited open-loop vector for board
/// bring-up. C owns arming, current trips, PWM registers and gate enables.
///
/// 与实时路径的区别 / Difference from the realtime path: 角度由本函数自己积分
/// （`trial_angle_rad`），与观测器和启动时序无关；只认 `FocState::Running`。
/// 它可用于验证 PWM 通道、相序与电流采样方向，而不必先把观测器调通。
/// 本函数是兼容/测试入口，**实时路径不使用它**。
/// The angle is integrated locally, independent of the observer and sequencer, so
/// it can validate the PWM path, phase order and current sensing first.
///
/// 参数与限幅 / Parameters and limits:
///   `electrical_speed_rad_s` 电角速度 `[rad/s]`；`voltage_magnitude_v` 相电压幅值
///   `[V]`；`dc_bus_voltage_v` 母线电压 `[V]`；`sample_time_s` 本步时长 `[s]`，
///   范围 `0 < ts <= 0.01`。
///   `voltage_magnitude_v` 硬限制为母线的 **8%**：这是调试矢量，不是控制通道，
///   占空比上限 8% 保证即使相序/极对配置错误也不会产生大电流。
///   The magnitude is hard-limited to 8% of the bus because this vector is a
///   bring-up aid, not a control channel.
///
/// 职责边界 / Ownership: 使能、过流跳闸、PWM 寄存器与栅极使能全部由 C 拥有；本函数
/// 只返回占空比，且失败路径同样先清输出。
/// C owns arming, current trips, PWM registers and gate enables.
///
/// 返回 / Returns: `Ok`；非有限值、幅值为负、母线非正、幅值超 8% 母线、`ts` 越界时
/// `InvalidArgument`（输出保持为 0）。
///
/// # Safety
/// Pointer and exclusive-access requirements are identical to
/// `foc_rust_fast_step`.
pub unsafe extern "C" fn foc_rust_open_loop_step(
    context: *mut FocRustContextStorage,
    electrical_speed_rad_s: f32,
    voltage_magnitude_v: f32,
    dc_bus_voltage_v: f32,
    sample_time_s: f32,
    output: *mut FocOutput,
) -> FocStatus {
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    zero_output(output);

    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    if controller.state != FocState::Running {
        return FocStatus::Disabled;
    }
    if !electrical_speed_rad_s.is_finite()
        || !voltage_magnitude_v.is_finite()
        || !dc_bus_voltage_v.is_finite()
        || !sample_time_s.is_finite()
        || voltage_magnitude_v < 0.0
        || dc_bus_voltage_v <= 0.0
        || voltage_magnitude_v > dc_bus_voltage_v * 0.08
        || !(0.0..=0.01).contains(&sample_time_s)
        || sample_time_s == 0.0
    {
        return FocStatus::InvalidArgument;
    }

    // 开环试转的角度积分：`θ += ω * ts` 后按 `[0, 2π)` 归一，防止长时间运行后 float
    // 精度随角度增大而下降。注意这里用的是**电角速度** `[rad/s]`，机械量要乘极对数。
    // The trial angle integrates electrical speed and is wrapped to `[0, 2π)` so
    // that float precision does not degrade as the angle grows.
    controller.trial_angle_rad =
        wrap_angle_0_to_2pi(controller.trial_angle_rad + electrical_speed_rad_s * sample_time_s);
    let mut math = PlatformMath::default();
    let (sin, cos) = math.sin_cos(controller.trial_angle_rad);
    // 逆 Park：把"幅值固定、方向由试转角决定"的电压矢量写成 αβ，再交给 SVPWM 换成
    // 三相占空比。α 用 `cos`、β 用 `sin`，与 `park()` 的方向约定一致。
    // Inverse Park from a fixed-magnitude vector rotating at the trial angle.
    let pwm = svpwm_update(
        AlphaBeta {
            alpha: voltage_magnitude_v * cos,
            beta: voltage_magnitude_v * sin,
        },
        &SvpwmParam {
            v_bus: dc_bus_voltage_v,
        },
    );
    if !pwm.duty_a.is_finite()
        || !pwm.duty_b.is_finite()
        || !pwm.duty_c.is_finite()
        || !(0.0..=1.0).contains(&pwm.duty_a)
        || !(0.0..=1.0).contains(&pwm.duty_b)
        || !(0.0..=1.0).contains(&pwm.duty_c)
    {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    output.duty_a = pwm.duty_a;
    output.duty_b = pwm.duty_b;
    output.duty_c = pwm.duty_c;
    FocStatus::Ok
}

#[no_mangle]
/// 兼容入口：执行一次 1 kHz 等效的速度 PI，返回快电流环需要的 Id/Iq 给定，
/// **实时路径不使用本函数**（实时路径的速度环内嵌在 `foc_rust_realtime_step()` 里）。
/// Executes the 1 kHz-equivalent speed PI and returns Id/Iq references for the
/// fast current loop. C decides when to call it and still owns ISR scheduling.
/// NOT used by the realtime path.
///
/// 调用方式 / How it is meant to be used: C 决定何时调用它并继续拥有 ISR 调度权——
/// 由 C 自己按 `speed_loop_frequency_hz` 分频调用本函数，再把结果喂给
/// `foc_rust_fast_step()`。本函数不做分频、不做斜坡、不做限速。
/// C schedules it and feeds the result to `foc_rust_fast_step()`; this function
/// performs no divider, ramp or slew limiting itself.
///
/// 单位换算 / Unit conversion: 入参 `target_speed_rpm` 与 `measured_speed_rpm` 都是
/// **机械**转速 `[rpm]`，内部按 `rpm * π / 30` 换成 `[rad/s]`（速度 PI 工作在 rad/s）。
/// 极对数不参与这一步：速度环的受控量就是机械转速。
/// Both inputs are mechanical speeds in `[rpm]`, converted internally to `[rad/s]`;
/// pole pairs do not enter here because the speed loop regulates mechanical speed.
///
/// 失败语义 / Failure semantics: `reference` 在函数开头就被清零，因此任何错误返回都
/// 让调用方看到 `(0, 0)`；`id_ref` 恒为 0（表贴式电机不弱磁）。
///
/// # Safety
/// `context` and `reference` follow the same ownership rules as the fast step.
pub unsafe extern "C" fn foc_rust_speed_step(
    context: *mut FocRustContextStorage,
    target_speed_rpm: f32,
    measured_speed_rpm: f32,
    reference: *mut FocReference,
) -> FocStatus {
    // SAFETY: reference must point to writable C ABI storage.
    let Some(reference) = (unsafe { reference.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    *reference = FocReference::default();
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    if controller.state != FocState::Running {
        return FocStatus::Disabled;
    }
    if !target_speed_rpm.is_finite() || !measured_speed_rpm.is_finite() {
        return FocStatus::InvalidArgument;
    }
    let current = controller.speed_loop.update(
        &controller.params,
        SpeedCommand {
            target_rpm: target_speed_rpm,
            id_ref_a: 0.0,
        },
        measured_speed_rpm * PI / 30.0,
    );
    reference.id_ref = current.id_ref_a;
    reference.iq_ref = current.iq_ref_a;
    FocStatus::Ok
}

#[no_mangle]
/// 由 C 侧主动锁存故障位并把控制器置入 `Fault`。
/// Latches fault bits and places the controller in the fault state.
///
/// 用途 / Use: 平台层检测到硬件故障（过流比较器、过温、驱动 FAULT 引脚）时调用，
/// 把 C 自己的位与 Rust 的逻辑位放进**同一个** `fault_flags` 空间。注意该位域因此
/// 可能包含本 crate 未定义的高位，读侧不要假定只有 `FOC_FAULT_*` 那四位。
/// The C platform ORs its own bits into the same `fault_flags` space, so readers
/// must not assume only the four `FOC_FAULT_*` bits can be set.
///
/// 返回 / Returns: 恒为 `HardwareFault`（即使上下文非法时返回 `InvalidArgument` 的
/// 分支除外），因为调用本身就是"现在有故障"的声明。
/// # Safety
/// `context` must be initialized and exclusively accessible.
pub unsafe extern "C" fn foc_rust_latch_fault(
    context: *mut FocRustContextStorage,
    fault_flags: u32,
) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    controller.fault_flags |= fault_flags;
    controller.state = FocState::Fault;
    FocStatus::HardwareFault
}

#[no_mangle]
/// 清除逻辑故障状态并回到 `Disabled`。
/// Clears logical fault state and returns to disabled state.
///
/// 安全语义 / Safety semantics: 本函数**只清软件状态**，不检查也不清除任何硬件故障。
/// 调用方必须先确认硬件原因已消失（过流比较器复位、驱动使能安全、母线正常），否则
/// 下一次启动会在同一个硬件条件下重新上电。清 `fault_flags` 后立刻 `stop()`，因此
/// 两个 PI 的积分与启动时序都是干净的。
/// It clears only software state; the C platform layer must already have checked
/// and cleared the hardware condition.
///
/// 返回 / Returns: `Ok`，或上下文非法时 `InvalidArgument`。
///
/// # Safety
/// `context` must be initialized and exclusively accessible. Hardware faults
/// must already have been checked and cleared by the C platform layer.
pub unsafe extern "C" fn foc_rust_clear_fault(context: *mut FocRustContextStorage) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    controller.fault_flags = 0;
    controller.stop();
    FocStatus::Ok
}

#[no_mangle]
/// 查询当前逻辑状态；这是**只读**入口，可在管理线程里调用。
/// Returns the current logical control state.
///
/// 返回 / Returns: 空指针或未初始化上下文时返回 `FocState::Uninitialized`（该值不会
/// 被写进真实控制器状态，只作为查询失败的信号）。
/// Returns `Uninitialized` for a null or uninitialized context; that value is only
/// ever a query-failure signal.
///
/// # Safety
/// `context` must be null or an initialized context without concurrent mutation.
pub unsafe extern "C" fn foc_rust_state(context: *mut FocRustContextStorage) -> FocState {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    unsafe { controller_mut(context) }
        .map(|controller| controller.state)
        .unwrap_or(FocState::Uninitialized)
}

#[no_mangle]
/// 查询锁存的逻辑故障位（`FOC_FAULT_*` 与 C 侧自身的位）。
/// Returns the latched logical fault bits.
///
/// 返回 / Returns: 位掩码；空指针或未初始化上下文时返回 0——**注意 0 与"无故障"不
/// 可区分**，因此判断是否存在故障时应先确认上下文有效（例如先查 `foc_rust_state()`
/// 是否不是 `Uninitialized`）。
/// Returns 0 for an invalid context, which is indistinguishable from "no fault";
/// confirm the context is valid first.
///
/// # Safety
/// `context` must be null or an initialized context without concurrent mutation.
pub unsafe extern "C" fn foc_rust_fault_flags(context: *mut FocRustContextStorage) -> u32 {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    unsafe { controller_mut(context) }
        .map(|controller| controller.fault_flags)
        .unwrap_or(0)
}

// 平台层的紧急关断钩子，由 C 实现。
// The platform emergency-shutdown hook, implemented in C.
//
// 这是本 crate 中唯一面向硬件的调用，且只在 **panic 处理器**里使用：Rust 侧出现
// panic 时没有其他可用的安全路径（分配、日志、恢复都不允许），只能请求 C 关断
// 功率级，然后停在这里。
// The only hardware-facing call in this crate, used solely by the panic handler:
// once Rust panics there is no safe recovery path, so C is asked to shut down.
#[cfg(all(not(test), target_os = "none"))]
extern "C" {
    fn foc_platform_emergency_stop();
}

/// `no_std` 目标板的 panic 处理器：请求硬件紧急关断后死循环。
/// The `no_std` target panic handler: request an emergency stop, then spin.
///
/// 为什么是死循环而不是复位 / Why spin instead of reset: 复位会让功率级在无人处理
/// 故障的情况下重新进入启动链；死循环 + 硬件关断把系统停在一个明确的安全状态，交给
/// 看门狗或操作者决定下一步。
/// A reset would re-enter the start-up chain unattended; spinning with the power
/// stage shut down leaves the system in one explicit safe state.
#[cfg(all(not(test), target_os = "none"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    // SAFETY: this is the C platform's non-returning-safe hardware shutdown hook.
    // It is the only hardware-facing call allowed in the Rust bridge.
    unsafe { foc_platform_emergency_stop() };
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handoff_torque_support_preserves_default_and_selects_a_fixed_target() {
        assert!((supported_handoff_iq(-0.2, 0.8, 0.0) + 0.2).abs() < f32::EPSILON);
        assert!((supported_handoff_iq(-0.2, 0.8, 0.15) - 0.12).abs() < 1.0e-6);
        assert!((supported_handoff_iq(0.4, 0.8, 0.15) - 0.12).abs() < 1.0e-6);
        assert!((supported_handoff_iq(0.2, -0.8, 0.15) + 0.12).abs() < 1.0e-6);
    }

    /// 构造一块清零的控制器存储，模拟 C 侧提供的静态缓冲。
    /// Builds a zeroed controller buffer, mimicking the static storage C provides.
    ///
    /// 全零是有意的：它与 `CONTEXT_MAGIC` 不匹配，因此未初始化上下文的行为（返回
    /// `InvalidArgument` / `Uninitialized`）可以在测试里被验证。
    /// All-zero bytes never match `CONTEXT_MAGIC`, so the uninitialized-context
    /// behaviour is exercised too.
    fn context() -> FocRustContextStorage {
        FocRustContextStorage {
            bytes: [0; FOC_RUST_CONTEXT_CAPACITY],
        }
    }

    /// 构造一份兼容入口用的最小合法配置：只有比例增益、无积分、母线 24 V。
    /// Builds a minimal valid legacy configuration: proportional only, 24 V bus.
    ///
    /// `ki = 0` 让输出只取决于本拍误差，测试结果与调用次数无关；`ts = 0.0001` 与
    /// 电流环周期不同量级，但因为 `ki = 0` 不影响结果。
    /// `ki = 0` makes the result independent of how many times the loop has run.
    fn config() -> FocBasicConfig {
        let pi = FocPiConfig {
            kp: 1.0,
            ki: 0.0,
            ts: 0.000_1,
            out_min: -12.0,
            out_max: 12.0,
            integrator_min: -6.0,
            integrator_max: 6.0,
        };
        FocBasicConfig {
            id_pi: pi,
            iq_pi: pi,
            nominal_dc_bus_voltage: 24.0,
        }
    }

    /// 锁定启动的双门语义：未配置时即使平台就绪也必须拒绝，配置后平台未就绪仍拒绝，
    /// 两门同时满足才进入 `Running`（`platform_ready` 不是装饰参数）。
    /// Pins the two-gate startup rule: configuration and platform readiness are both
    /// required before `Running`.
    #[test]
    fn starts_safe_and_requires_both_gates() {
        let mut context = context();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_state(&mut context), FocState::Disabled);
            assert_eq!(
                foc_rust_request_start(&mut context, 1),
                FocStatus::NotConfigured
            );
            assert_eq!(
                foc_rust_configure_basic(&mut context, &config()),
                FocStatus::Ok
            );
            assert_eq!(
                foc_rust_request_start(&mut context, 0),
                FocStatus::NotConfigured
            );
            assert_eq!(foc_rust_request_start(&mut context, 1), FocStatus::Ok);
            assert_eq!(foc_rust_state(&mut context), FocState::Running);
        }
    }

    /// 锁定兼容电流环路径能跑通并给出三相都合法、且互不相同的占空比。
    /// Pins that the legacy current-loop path produces three distinct in-range duties.
    ///
    /// `assert_ne!(duty_a, duty_b)` 是关键：若 SVPWM 或 Park 退化成常数输出，三相会
    /// 全部相等（纯共模），电机不产生任何转矩，而"占空比在 [0,1] 内"的检查仍然通过。
    /// The distinctness check is what catches a degenerated SVPWM/Park that would
    /// still pass the range assertions.
    #[test]
    fn executes_imported_foc_basic_algorithm() {
        let mut context = context();
        let feedback = FocFeedback {
            dc_bus_voltage: 24.0,
            ..FocFeedback::default()
        };
        let reference = FocReference {
            id_ref: 0.0,
            iq_ref: 2.0,
        };
        let mut output = FocOutput::default();

        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(
                foc_rust_configure_basic(&mut context, &config()),
                FocStatus::Ok
            );
            assert_eq!(foc_rust_request_start(&mut context, 1), FocStatus::Ok);
            assert_eq!(
                foc_rust_fast_step(&mut context, &feedback, &reference, &mut output),
                FocStatus::Ok
            );
        }

        assert!((0.0..=1.0).contains(&output.duty_a));
        assert!((0.0..=1.0).contains(&output.duty_b));
        assert!((0.0..=1.0).contains(&output.duty_c));
        assert_ne!(output.duty_a, output.duty_b);
    }

    /// 锁定 ST 参考配置确实同时具备速度环与电流环：零速下给 524 rpm 目标应产生正的
    /// Iq，且不超过额定电流 0.8 A（速度 PI 输出限幅即额定电流）。
    /// Pins that the ST reference configuration exposes both loops and clamps Iq to
    /// the rated current at zero measured speed.
    #[test]
    fn st_reference_config_exposes_speed_and_current_loops() {
        let mut context = context();
        let mut reference = FocReference::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_configure_st_reference(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_request_start(&mut context, 1), FocStatus::Ok);
            assert_eq!(
                foc_rust_speed_step(&mut context, 524.0, 0.0, &mut reference),
                FocStatus::Ok
            );
        }
        assert_eq!(reference.id_ref, 0.0);
        assert!(reference.iq_ref > 0.0 && reference.iq_ref <= 0.8);
    }

    /// 锁定"任何非成功返回都先清零输出"这一安全语义：停机后与进入 `Fault` 后调用快环
    /// 都必须把之前写满 1.0 的输出清零，而不是沿用上一周期的占空比。
    /// Pins the fail-safe invariant that every non-OK return zeroes the output,
    /// even when the caller's buffer previously held full-scale duties.
    #[test]
    fn stop_and_fault_paths_always_zero_output() {
        let mut context = context();
        let feedback = FocFeedback {
            dc_bus_voltage: 24.0,
            ..FocFeedback::default()
        };
        let reference = FocReference::default();
        let mut output = FocOutput {
            duty_a: 1.0,
            duty_b: 1.0,
            duty_c: 1.0,
        };

        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(
                foc_rust_configure_basic(&mut context, &config()),
                FocStatus::Ok
            );
            foc_rust_stop(&mut context);
            assert_eq!(
                foc_rust_fast_step(&mut context, &feedback, &reference, &mut output),
                FocStatus::Disabled
            );
            assert_eq!(output.duty_a, 0.0);
            assert_eq!(
                foc_rust_latch_fault(&mut context, 0x10),
                FocStatus::HardwareFault
            );
            assert_eq!(
                foc_rust_fast_step(&mut context, &feedback, &reference, &mut output),
                FocStatus::HardwareFault
            );
            assert_eq!(output.duty_b, 0.0);
            assert_eq!(foc_rust_fault_flags(&mut context), 0x10);
        }
    }

    /// 锁定开环试转受 `Running` 门控且受 8% 母线电压硬限幅：未启动返回 `Disabled`
    /// 且不写输出；启动后合法幅值（0.5 V < 13 V * 8% = 1.04 V）给出约 50% 占空比；
    /// 幅值越界（2.0 V）返回 `InvalidArgument` 且输出被清零。
    /// Pins the `Running` gate, the 8%-of-bus magnitude limit and the zeroing on the
    /// rejected-magnitude path.
    #[test]
    fn open_loop_trial_is_running_gated_and_voltage_limited() {
        let mut context = context();
        let mut output = FocOutput::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_configure_st_reference(&mut context), FocStatus::Ok);
            assert_eq!(
                foc_rust_open_loop_step(&mut context, 1.0, 0.5, 13.0, 0.001, &mut output),
                FocStatus::Disabled
            );
            assert_eq!(foc_rust_request_start(&mut context, 1), FocStatus::Ok);
            assert_eq!(
                foc_rust_open_loop_step(&mut context, 10.0, 0.5, 13.0, 0.001, &mut output),
                FocStatus::Ok
            );
            assert!((0.44..=0.56).contains(&output.duty_a));
            assert_eq!(
                foc_rust_open_loop_step(&mut context, 10.0, 2.0, 13.0, 0.001, &mut output),
                FocStatus::InvalidArgument
            );
            assert_eq!(output.duty_a, 0.0);
        }
    }

    /// 锁定 Production 参数档使用的规范化 CRC：相同的完整配置必须在
    /// Host 与 MCU 上得到相同结果，改动任一字段都必须改变 CRC；空指针或
    /// 非法配置必须拒绝且不覆写输出。
    #[test]
    fn runtime_config_crc_is_canonical_sensitive_and_guarded() {
        let mut runtime = default_st_runtime_config();
        runtime.observer_backend = ObserverBackend::SmoPll as u32;
        runtime.observer_enable = 1;
        runtime.closed_loop_enable = 0;
        runtime.observer_update_divider = 1;
        runtime.voltage_utilization = 0.90;
        let mut crc = 0;

        unsafe {
            assert_eq!(
                foc_rust_runtime_config_crc32(&runtime, &mut crc),
                FocStatus::Ok
            );
        }
        assert_eq!(crc, 0xEDF4_F6CA);

        let mut changed = runtime;
        changed.observer_pll_kp = f32::from_bits(changed.observer_pll_kp.to_bits() + 1);
        let mut changed_crc = 0;
        unsafe {
            assert_eq!(
                foc_rust_runtime_config_crc32(&changed, &mut changed_crc),
                FocStatus::Ok
            );
        }
        assert_ne!(changed_crc, crc);

        let mut invalid = runtime;
        invalid.voltage_utilization = f32::NAN;
        let mut untouched = 0xA5A5_5A5A;
        unsafe {
            assert_eq!(
                foc_rust_runtime_config_crc32(&invalid, &mut untouched),
                FocStatus::InvalidArgument
            );
            assert_eq!(
                foc_rust_runtime_config_crc32(core::ptr::null(), &mut untouched),
                FocStatus::InvalidArgument
            );
            assert_eq!(
                foc_rust_runtime_config_crc32(&runtime, core::ptr::null_mut()),
                FocStatus::InvalidArgument
            );
        }
        assert_eq!(untouched, 0xA5A5_5A5A);
    }

    /// 锁定实时路径的版本化 SMO 默认参数与启动门：断言 `[ST]`/`[FW]` 默认值本身
    /// （后端、分频、闭环默认关闭、SMO 与 PLL 增益、可靠性门限、电流限速），再确认
    /// 300 拍内实时步始终返回 `Ok`、占空比有限、且闭环默认关闭时状态停在 `Alignment`。
    /// Pins the versioned SMO defaults and the default-closed-loop start-up gate, and
    /// that 300 realtime ticks stay finite and in `Alignment`.
    ///
    /// 这些数值断言是**有意的**：默认参数改动必须显式改这里，从而不能在无人注意的
    /// 情况下改变上电行为。
    /// These numeric assertions are intentional: changing a default must be a
    /// deliberate, visible edit.
    #[test]
    fn realtime_path_uses_versioned_smo_configuration_and_startup_gate() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            assert_eq!(runtime.observer_backend, ObserverBackend::SmoPll as u32);
            assert_eq!(runtime.observer_enable, 1);
            assert_eq!(runtime.observer_update_divider, 1);
            assert_eq!(runtime.closed_loop_enable, 0);
            assert!((runtime.observer_smo_k_slide_v - 4.0).abs() < 1e-6);
            assert!((runtime.alignment_duration_s - 2.0).abs() < 1e-6);
            assert!((runtime.open_loop_ramp_duration_s - 5.0).abs() < 1e-6);
            assert!((runtime.observer_transition_duration_s - 0.05).abs() < 1e-6);
            assert!((runtime.startup_final_speed_rpm - 582.0).abs() < 1e-6);
            assert!((runtime.observer_smo_boundary_a - 0.24).abs() < 1e-6);
            assert!((runtime.observer_emf_filter_alpha - 0.015).abs() < 1e-6);
            assert!((runtime.observer_pll_kp - 40.0).abs() < 1e-6);
            assert!((runtime.observer_acquisition_pll_kp_ratio - 0.60).abs() < 1e-6);
            assert!((runtime.observer_pll_ki - 1_000.0).abs() < 1e-6);
            assert!((runtime.observer_minimum_speed_rpm - 524.0).abs() < 1e-6);
            assert_eq!(runtime.observer_run_reliability.minimum_speed_rpm, 0.0);
            assert!((runtime.observer_acquisition_maximum_phase_error_rad - 0.65).abs() < 1e-6);
            assert!((runtime.observer_run_reliability.maximum_phase_error_rad - 0.65).abs() < 1e-6);
            assert_eq!(runtime.angle_compensation.park_prediction_ticks, 0.0);
            assert_eq!(
                runtime.angle_compensation.reverse_park_prediction_ticks,
                0.0
            );
            assert_eq!(runtime.inverter_voltage_model.enabled, 0);
            assert_eq!(
                runtime
                    .inverter_voltage_model
                    .observer_voltage_correction_enable,
                0
            );
            assert_eq!(runtime.inverter_voltage_model.pwm_feedforward_enable, 0);
            assert_eq!(
                runtime.inverter_voltage_model.pwm_carrier_frequency_hz,
                12_000
            );
            assert!((runtime.inverter_voltage_model.dead_time_s - 550.0e-9).abs() < 1.0e-12);
            assert!((runtime.inverter_voltage_model.compensation_gain - 1.0).abs() < 1e-6);
            assert!((runtime.inverter_voltage_model.current_zero_band_a - 0.005).abs() < 1e-6);
            assert_eq!(
                runtime.inverter_voltage_model.current_sign_filter_alpha,
                1.0
            );
            assert_eq!(runtime.inverter_voltage_model.device_drop_v, 0.0);
            assert_eq!(runtime.observer_consecutive_samples, 20);
            assert!((runtime.observer_acquisition_timeout_s - 5.0).abs() < 1e-6);
            assert!((runtime.observer_loss_timeout_s - 0.20).abs() < 1e-6);
            assert!((runtime.handoff_torque_support_ratio - 0.47).abs() < 1e-6);
            assert!((runtime.speed_pi_preload_ratio - 0.20).abs() < 1e-6);
            assert!((runtime.closed_loop_current_slew_a_per_s - 4.0).abs() < 1e-6);
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );
            for _ in 0..300 {
                assert_eq!(
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry,),
                    FocStatus::Ok
                );
                assert!(output.duty_a.is_finite());
                assert!(output.duty_b.is_finite());
                assert!(output.duty_c.is_finite());
            }
        }
        assert_eq!(telemetry.state, FocState::Alignment as u32);
        assert_eq!(telemetry.observer_backend, ObserverBackend::SmoPll as u32);
        assert_eq!(telemetry.closed_loop_active, 0);
        assert!(
            (output.duty_a - 0.5).abs() > 1e-6
                || (output.duty_b - 0.5).abs() > 1e-6
                || (output.duty_c - 0.5).abs() > 1e-6
        );
    }

    /// 负目标按幅值通过同一安全窗口，并把 Rev-Up 速度与 Iq 同时翻转；Rev-Up
    /// 获取速度可以高于最终目标，而低于应用最低速度和超过机械上限仍必须拒绝。
    #[test]
    fn realtime_start_accepts_bounded_reverse_and_mirrors_rev_up_torque() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.alignment_duration_s = 0.001;
            runtime.open_loop_ramp_duration_s = 0.001;
            runtime.startup_final_speed_rpm = 700.0;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, 0.0),
                FocStatus::InvalidArgument
            );
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, -(runtime.default_target_speed_rpm - 1.0),),
                FocStatus::InvalidArgument
            );
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, -(runtime.max_speed_rpm + 1.0)),
                FocStatus::InvalidArgument
            );
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, -runtime.default_target_speed_rpm),
                FocStatus::Ok
            );
            for _ in 0..32 {
                assert_eq!(
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry,),
                    FocStatus::Ok
                );
            }
        }
        assert!(matches!(
            telemetry.state,
            value if value == FocState::OpenLoopRamp as u32
                || value == FocState::OpenLoopHold as u32
        ));
        // A short negative-angle integration wraps just below 2*pi; the mirrored
        // positive trajectory would still be close to zero.
        assert!(telemetry.forced_electrical_angle_rad > PI);
        assert!(telemetry.iq_reference_a < 0.0);
        assert!(telemetry.target_speed_rpm < 0.0);
    }

    /// 反电势在静止对齐段不可观，因此 SMO/PLL 必须保持刚启动时的 reset 状态；
    /// `Alignment -> OpenLoopRamp` 的边界拍也不能提前消费一次噪声输入。进入升速后的
    /// 下一拍先用同拍 αβ 电流预置 SMO 电流模型，再允许观测器更新，使每次启动都从
    /// 同一个已知状态建立 BEMF，而不是由一秒对齐期里的 ADC 残差或 0→对齐电流阶跃
    /// 随机选择正/反向锁定解。
    /// The BEMF observer stays reset throughout standstill alignment, including the
    /// boundary tick, and begins updating on the following OpenLoopRamp tick.
    #[test]
    fn bemf_observer_stays_reset_until_open_loop_ramp() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            phase_current_a: 0.20,
            phase_current_b: -0.10,
            phase_current_c: -0.10,
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.alignment_duration_s = 0.001;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );

            let mut reached_ramp = false;
            for _ in 0..32 {
                assert_eq!(
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry),
                    FocStatus::Ok
                );
                assert_eq!(telemetry.observer_electrical_angle_rad, 0.0);
                assert_eq!(telemetry.observer_bemf_alpha_v, 0.0);
                assert_eq!(telemetry.observer_bemf_beta_v, 0.0);
                assert_eq!(telemetry.observer_reliability_flags, 0);
                if telemetry.state == FocState::OpenLoopRamp as u32 {
                    reached_ramp = true;
                    break;
                }
                assert_eq!(telemetry.state, FocState::Alignment as u32);
            }
            assert!(reached_ramp);

            assert_eq!(
                foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry),
                FocStatus::Ok
            );
            // 首个升速拍只建立电流模型；由于估计电流等于同拍实测电流，首拍
            // sliding 与 BEMF 必须仍为零。
            assert_eq!(telemetry.observer_bemf_alpha_v, 0.0);
            assert_eq!(telemetry.observer_bemf_beta_v, 0.0);
            assert_eq!(
                foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry),
                FocStatus::Ok
            );
        }
        assert_eq!(telemetry.state, FocState::OpenLoopRamp as u32);
        assert!(telemetry.observer_bemf_alpha_v != 0.0 || telemetry.observer_bemf_beta_v != 0.0);
    }

    /// 观察器到达可观的终速保持段后先保留 Rev-Up 期间已学到的状态；只有在
    /// 500 ms 内仍未通过获取门时，才用同拍强制角、已知终速和电流干净重捕获。
    #[test]
    fn open_loop_hold_reacquires_observer_from_observable_speed() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            phase_current_a: 0.20,
            phase_current_b: -0.10,
            phase_current_c: -0.10,
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.alignment_duration_s = 0.001;
            runtime.open_loop_ramp_duration_s = 0.001;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );

            for _ in 0..64 {
                assert_eq!(
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry),
                    FocStatus::Ok
                );
                if telemetry.state == FocState::OpenLoopHold as u32 {
                    break;
                }
            }
            assert_eq!(telemetry.state, FocState::OpenLoopHold as u32);
            let countdown = controller_mut(&mut context)
                .expect("controller")
                .observer_hold_recovery_countdown;
            assert_eq!(countdown, (runtime.pwm_frequency_hz / 3).max(1));

            assert_eq!(
                foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry),
                FocStatus::Ok
            );
            assert_eq!(
                controller_mut(&mut context)
                    .expect("controller")
                    .observer_hold_recovery_countdown,
                (runtime.pwm_frequency_hz / 3).max(1) - 1
            );
        }
        assert!(telemetry.observer_bemf_alpha_v.is_finite());
        assert!(telemetry.observer_bemf_beta_v.is_finite());
    }

    #[test]
    fn run_minimum_speed_must_not_exceed_acquisition_threshold() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.observer_run_reliability.minimum_speed_rpm =
                runtime.observer_minimum_speed_rpm + 1.0;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );
        }
    }

    #[test]
    fn observer_gain_schedule_and_run_phase_gate_must_be_bounded() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.observer_acquisition_pll_kp_ratio = 0.0;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );
            runtime.observer_acquisition_pll_kp_ratio = 0.5;
            runtime.observer_acquisition_maximum_phase_error_rad =
                core::f32::consts::FRAC_PI_2 + 0.001;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );
            runtime.observer_acquisition_maximum_phase_error_rad = 0.65;
            runtime.observer_run_reliability.maximum_phase_error_rad =
                core::f32::consts::FRAC_PI_2 + 0.001;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );
        }
    }

    #[test]
    fn angle_compensation_ticks_must_be_finite_and_bounded() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.angle_compensation.park_prediction_ticks = 2.01;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );
            runtime.angle_compensation.park_prediction_ticks = 0.0;
            runtime.angle_compensation.reverse_park_prediction_ticks = f32::NAN;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );
        }
    }

    #[test]
    fn inverter_model_gates_and_carrier_are_validated_as_one_group() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);

            runtime
                .inverter_voltage_model
                .observer_voltage_correction_enable = 1;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );

            runtime.inverter_voltage_model.enabled = 1;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);

            runtime.inverter_voltage_model.pwm_carrier_frequency_hz = 24_000;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);

            runtime.inverter_voltage_model.pwm_carrier_frequency_hz = 18_000;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );
            runtime.inverter_voltage_model.pwm_carrier_frequency_hz = 12_000;
            runtime.inverter_voltage_model.current_sign_filter_alpha = 0.0;
            assert_eq!(
                foc_rust_configure(&mut context, &runtime),
                FocStatus::InvalidArgument
            );
        }
    }

    #[test]
    fn observer_correction_and_pwm_feedforward_are_independent_runtime_layers() {
        fn one_step(observer_correction: u32, feedforward: u32) -> FocOutput {
            let mut context = context();
            let mut runtime = FocRuntimeConfig::default();
            let feedback = FocFeedback {
                phase_current_a: 0.20,
                phase_current_b: -0.10,
                phase_current_c: -0.10,
                dc_bus_voltage: 13.0,
                ..FocFeedback::default()
            };
            let mut output = FocOutput::default();
            unsafe {
                assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
                assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
                runtime.inverter_voltage_model.enabled =
                    (observer_correction != 0 || feedforward != 0) as u32;
                runtime
                    .inverter_voltage_model
                    .observer_voltage_correction_enable = observer_correction;
                runtime.inverter_voltage_model.pwm_feedforward_enable = feedforward;
                assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
                assert_eq!(
                    foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                    FocStatus::Ok
                );
                assert_eq!(
                    foc_rust_realtime_step(
                        &mut context,
                        &feedback,
                        &mut output,
                        core::ptr::null_mut(),
                    ),
                    FocStatus::Ok
                );
            }
            output
        }

        let baseline = one_step(0, 0);
        let observer_only = one_step(1, 0);
        let feedforward_only = one_step(0, 1);
        assert_eq!(baseline.duty_a, observer_only.duty_a);
        assert_eq!(baseline.duty_b, observer_only.duty_b);
        assert_eq!(baseline.duty_c, observer_only.duty_c);
        assert!(
            (baseline.duty_a - feedforward_only.duty_a).abs() > 1.0e-7
                || (baseline.duty_b - feedforward_only.duty_b).abs() > 1.0e-7
                || (baseline.duty_c - feedforward_only.duty_c).abs() > 1.0e-7
        );
    }

    /// 锁定观测器**获取**超时语义：打开闭环、把获取超时压到 1 ms 后，若观测器始终
    /// 不收敛，实时步必须在有限的拍数内返回 `HardwareFault`，状态进入 `Fault`，锁存
    /// `FOC_FAULT_OBSERVER_STARTUP`，并把输出清零（而不是一直空转）。
    /// Pins the observer acquisition-timeout fault: a never-converging observer ends
    /// in `Fault` with `FOC_FAULT_OBSERVER_STARTUP` and a zeroed output.
    #[test]
    fn closed_loop_request_faults_when_observer_never_converges() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.closed_loop_enable = 1;
            runtime.alignment_duration_s = 0.001;
            runtime.open_loop_ramp_duration_s = 0.001;
            runtime.observer_acquisition_timeout_s = 0.001;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );
            let mut status = FocStatus::Ok;
            for _ in 0..100 {
                status =
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry);
                if status != FocStatus::Ok {
                    break;
                }
            }
            assert_eq!(status, FocStatus::HardwareFault);
            assert_eq!(foc_rust_state(&mut context), FocState::Fault);
            assert_ne!(
                foc_rust_fault_flags(&mut context) & FOC_FAULT_OBSERVER_STARTUP,
                0
            );
            assert_eq!(output.duty_a, 0.0);
        }
    }

    /// 锁定观测器**失锁**超时语义（与获取超时是两条独立路径）：先用两次
    /// `startup.update()` 手工把时序推进到闭环交接完成，再强制 `ClosedLoop` 且标记
    /// 观测器不可靠，失锁超时 1 ms；实时步必须在有限拍数内返回 `HardwareFault`、
    /// 锁存 `FOC_FAULT_OBSERVER_LOST` 并清零输出。
    /// Pins the separate loss-of-lock timeout path, driven from an already
    /// handed-over `ClosedLoop` state with a deliberately unreliable observer.
    ///
    /// 直接操作 `controller.startup`/`state` 是因为该路径只有在交接完成后才可能触发，
    /// 靠真实的观测器输入在单元测试里复现代价过高；这里依赖的是"已经在线性区"的假设，
    /// 而不是完整的物理仿真。
    /// The sequencer is driven directly because reproducing a post-handover dropout
    /// through the real observer inputs is impractical in a unit test.
    #[test]
    fn closed_loop_latches_fault_after_observer_loss_timeout() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.closed_loop_enable = 1;
            runtime.observer_loss_timeout_s = 0.001;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );

            let controller = controller_mut(&mut context).unwrap();
            let _ = controller.startup.update(2.05, 0.0, true, 0.2, 32.0);
            let _ = controller.startup.update(0.026, 0.0, true, 0.0, 32.0);
            controller.state = FocState::ClosedLoop;
            controller.observer_reliable = false;

            let mut status = FocStatus::Ok;
            for _ in 0..32 {
                status =
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry);
                if status != FocStatus::Ok {
                    break;
                }
            }
            assert_eq!(status, FocStatus::HardwareFault);
            assert_ne!(
                foc_rust_fault_flags(&mut context) & FOC_FAULT_OBSERVER_LOST,
                0
            );
            assert_eq!(output.duty_b, 0.0);
        }
    }
}
