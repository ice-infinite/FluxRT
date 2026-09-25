//! Utility_LimitRamp 的等价实现。
//!
//! FluxRT —— 输出限幅与目标斜坡（`foc-algorithm` 的安全边界层）。
//! FluxRT - output limiting and target ramping, the safety-bound layer of
//! `foc-algorithm`.
//!
//! 职责 / Responsibility:
//!   - `limit()` 给任何控制量提供统一的上下界入口。
//!   - `RampState` 限制目标值每拍的升降幅度，避免给功率级和电流环施加阶跃。
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!     -> foc-algorithm -> `utility`（本模块，只依赖 `crate::math::clamp`）
//!   依赖方向单一，本模块不认识 HAL、RTOS、堆或全局状态。
//!   Depends only on `crate::math::clamp`; no HAL, RTOS, heap or global state.
//!
//! 实时约束 / Real-time constraints:
//!   `update()` 每拍只有一次减法、两次乘法和常数次比较，可安全放在 12 kHz
//!   电流环/速度环里；不分配、不阻塞、不打日志。
//!   `update()` costs one subtraction, two multiplies and a constant number of
//!   comparisons, so it is safe in the 12 kHz current/speed loop.
//!
//! 与 C 版的关系 / Relation to the C version:
//!   本模块是 C 版 `Utility_LimitRamp` 的逐行为等价移植，`limit()`/`update()`
//!   的返回值与参数顺序都保持原样，`matches_c_reference_vectors` 直接复用原
//!   C 测试向量。改掉"先算增量再限幅"的顺序会让该测试失配。
//!   A behaviour-equivalent port of the C `Utility_LimitRamp`; the test replays
//!   the original C vectors, so changing the order of operations breaks it.
//!
//! 参考 / Reference: `算法库移植状态.md`（Utility_LimitRamp 行）

use crate::math::clamp;

/// 限幅：把 `input` 夹到 `[min_value, max_value]`。
/// Clamps `input` into `[min_value, max_value]`.
///
/// 参数 / Parameters:
///   input     待限幅量（量纲由调用方决定，如电流 `[A]`、电压 `[V]`、
///             占空比 per-mille 或 `[0, 1]`）
///   min_value 下界（含）
///   max_value 上界（含）
///
/// 返回 / Returns: 限幅后的值，与 `input` 同量纲。
/// The limited value, in the same unit as `input`.
///
/// 与 `clamp` 的关系 / Relation to `clamp`:
///   只是一层命名包装，保留 C 版 `Utility_LimitRamp` 的 API 名字以便逐项对照；
///   上下界写反时内部同样会自动交换，`NaN` 同样会穿透。
///   A thin naming wrapper kept for one-to-one comparison with the C API. Swapped
///   bounds are tolerated and `NaN` passes through, exactly like `clamp`.
///
/// 上下文 / Context: 调用者可在初始化或 12 kHz 快环中使用，无状态、无分配。
/// Stateless and allocation-free; usable at init time and in the fast loop.
#[inline]
pub fn limit(input: f32, min_value: f32, max_value: f32) -> f32 {
    clamp(input, min_value, max_value)
}

/// 斜坡参数（调用方持有，`update()` 只读）。
/// Ramp parameters, owned by the caller and only read by `update()`.
///
/// `#[repr(C)]` 用于稳定布局，便于后续与 C 侧结构体对齐；当前 crate 未导出
/// C ABI 符号。`Copy` 让参数可以按值传递而无需借用检查开销。
/// `#[repr(C)]` prepares a stable layout for future C interoperability; no C ABI
/// symbol is exported today.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RampParam {
    /// 上坡速率 `[目标量单位/s]`，必须非负。
    /// Rising slew rate in target-units per second; must be non-negative.
    ///
    /// 每拍最大增量 = `rate_up * ts`。负值会让 `delta > max_up` 的比较失去意义，
    /// 输出可能反向冲过目标。
    /// The per-sample step is `rate_up * ts`; a negative value breaks the
    /// comparison and can overshoot the target.
    pub rate_up: f32,
    /// 下坡速率 `[目标量单位/s]`，用正数表示下降速度，必须非负。
    /// Falling slew rate in target-units per second, expressed as a positive
    /// magnitude; must be non-negative.
    ///
    /// 上升和下降分开配置，是为了让"加速快、减速慢"这类功率级保护策略可表达。
    /// Separate up/down rates let a strategy such as "accelerate fast, decelerate
    /// slowly" be expressed directly.
    pub rate_down: f32,
    /// 采样周期 `[s]`。必须与调用本状态的真实周期一致。
    /// Sample period in `[s]`; must match the real call period of this state.
    ///
    /// 多速率系统必须为每个环单独设置：把 12 kHz 电流环的 `ts` 用在 1 kHz
    /// 速度环上，实际斜坡速率会放大 12 倍。
    /// Multi-rate systems need one value per loop; reusing the 12 kHz current-loop
    /// `ts` in a 1 kHz loop makes the ramp 12x too fast.
    pub ts: f32,
}

/// 斜坡状态：只保存当前输出，定长且无堆指针。
/// Ramp state: only the current output, fixed size, no heap pointer.
///
/// 由调用方持有并在初始化阶段静态创建；在 ISR 中不允许动态创建对象。
/// Owned by the caller and created statically at init time; never allocate one
/// inside the ISR.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RampState {
    /// 当前斜坡输出值，量纲与传入的 `target` 相同。
    /// Current ramp output, in the same unit as the incoming `target`.
    ///
    /// 它是本状态唯一的记忆：`update()` 只在这个值上叠加受速率限制的增量。
    /// It is the only memory this state has; `update()` only adds a rate-limited
    /// increment to it.
    pub output: f32,
}

impl RampState {
    /// 以给定初始输出构造斜坡状态，`initial_output` 的量纲与后续 `target` 相同。
    /// Builds a ramp state with the given initial output, in the same unit as the
    /// later `target`.
    ///
    /// `const fn` 的目的是支持在静态变量/`static` 初始化中使用，从而满足"状态在
    /// 初始化阶段创建、ISR 内不分配"的约束。
    /// `const fn` allows static initialisation, which is how the "create state at
    /// init time, never allocate in the ISR" rule is met.
    #[inline]
    pub const fn new(initial_output: f32) -> Self {
        Self {
            output: initial_output,
        }
    }

    /// 复位：把输出直接跳到 `output`。量纲与 `update()` 的 `target` 相同。
    /// Reset: jumps the output straight to `output`, in the same unit as the
    /// `update()` target.
    ///
    /// 这是唯一的无条件跳变入口，用于使能、模式切换、故障恢复和停机；
    /// 正常运行期间应让 `update()` 逐步逼近目标，避免给功率级阶跃。
    /// This is the only unconditional jump and belongs to enable, mode-switch,
    /// fault-recovery and stop paths. During normal operation let `update()` walk
    /// towards the target instead of stepping the power stage.
    #[inline]
    pub fn reset(&mut self, output: f32) {
        self.output = output;
    }

    /// 单步斜坡：返回本拍输出，量纲与 `target` 相同。
    /// One ramp step: returns this sample's output, in the same unit as `target`.
    ///
    /// 参数 / Parameters:
    ///   param  斜坡速率 `[目标量单位/s]` 与采样周期 `[s]`
    ///   target 目标值（与 `output` 同量纲）
    ///
    /// 返回 / Returns: 更新后的 `self.output`；上限为 `target`，不会过冲。
    /// The updated `self.output`, which never overshoots `target`.
    ///
    /// 实时约束 / Real-time constraints: 定长常数次运算，可在 12 kHz 快环调用；
    ///   不分配、不阻塞、不打日志、不加锁。
    /// Constant work, safe for the 12 kHz fast loop; no allocation, blocking,
    /// logging or locking.
    ///
    /// 陷阱 / Pitfalls:
    ///   - `rate_up`/`rate_down` 必须非负：负速率下增量会被"限幅"成负值，
    ///     输出朝目标的反方向走。
    ///   - `ts == 0` 时输出被冻结，看起来像控制失效而不是报错；`ts` 为 `NaN`
    ///     则所有比较为假，输出会直接跟到目标。
    ///   - `output` 自身为 `NaN` 时 `delta` 变成 `NaN`，所有比较为假，状态会
    ///     永久污染，调用方需保证参数与初值有限。
    ///   - Negative rates drive the output away from the target; `ts == 0` freezes
    ///     the output silently; a `NaN` output poisons the state permanently.
    #[inline]
    pub fn update(&mut self, param: &RampParam, target: f32) -> f32 {
        let mut delta = target - self.output;
        let max_up = param.rate_up * param.ts;
        let max_down = param.rate_down * param.ts;
        if delta > max_up {
            delta = max_up;
        } else if delta < -max_down {
            delta = -max_down;
        }
        self.output += delta;
        self.output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 与 C 版 `Utility_LimitRamp` 原测试向量逐项对照。
    /// Replays the original C `Utility_LimitRamp` test vectors.
    ///
    /// 第一段用 `rate_up = 10.0`、`ts = 0.1` 得到每拍 1.0 的上升步长，第二段用
    /// 相反的 `rate_down` 验证下降方向；`limit()` 一项验证包装仍然限幅。
    /// The first step exercises the 1.0-per-sample rise, the second the fall, and
    /// the last one that the `limit()` wrapper still clamps.
    #[test]
    fn matches_c_reference_vectors() {
        let p = RampParam {
            rate_up: 10.0,
            rate_down: 20.0,
            ts: 0.1,
        };
        let mut s = RampState::new(0.0);
        assert!((s.update(&p, 5.0) - 1.0).abs() <= 1e-6);
        assert!((s.update(&p, -5.0) + 1.0).abs() <= 1e-6);
        assert!((limit(5.0, -2.0, 2.0) - 2.0).abs() <= 1e-6);
    }
}
