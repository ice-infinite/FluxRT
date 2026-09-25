//! Transform_ClarkePark 的等价实现。
//!
//! FluxRT —— 三相/两相/旋转坐标系变换（`foc-algorithm` 的坐标变换层）。
//! FluxRT - three-phase / two-phase / rotating-frame transforms, the coordinate
//! transform layer of `foc-algorithm`.
//!
//! 职责 / Responsibility:
//!   - 提供 `abc <-> αβ`（Clarke/逆 Clarke）与 `αβ <-> dq`（Park/逆 Park）。
//!   - 这是电流环最热的四步运算：采样后立即 Clarke，随后 Park 到 dq 做 PI，
//!     再逆 Park 回 αβ 交给 SVPWM。
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!     -> foc-algorithm -> `transform`（本模块，只依赖 `crate::math`）
//!   本模块不认识 HAL、RTOS、堆或全局状态；`foc-control` 在快环里逐拍调用
//!   `clarke()`/`park()`，`foc-sim` 在 PC 仿真里调用全部四个函数。
//!
//! 实时约束 / Real-time constraints:
//!   全部为纯计算，无分配、无阻塞、无日志、无锁。`park`/`inverse_park` 各需要
//!   一组 sin/cos：真实快环由 `foc-control` 只算一次 sin/cos 并共享（见
//!   `docs/硬件数学加速与CPU回退.md`），不要把这里当成"每拍两次三角"的
//!   接口设计。
//!   Pure computation with no allocation, blocking or logging. `park` and
//!   `inverse_park` each need a sin/cos pair; the real fast loop computes one pair
//!   and shares it in `foc-control`.
//!
//! 量纲 / Units:
//!   三个结构体本身无量纲，量纲由调用方约定：`foc-control` 用 `[A]` 表示电流、
//!   用 `[V]` 表示电压（`pwm_to_phase_voltage()` 先减去共模再乘母线电压）。
//!   变换是线性且保幅值的，所以输入输出的量纲必须一致，混用会让 PI 增益失去
//!   意义。
//!   The structs are dimensionless; `foc-control` feeds `[A]` currents and `[V]`
//!   voltages. The transforms are linear and amplitude-preserving, so the input
//!   and output units must match or the PI gains become meaningless.
//!
//! 定点与 Q 格式 / Fixed point: 本模块全为 `f32`，不使用任何 Q 格式；定点缩放
//!   只存在于 CORDIC 平台适配器内部。
//!
//! 参考 / Reference: `算法库移植状态.md`（Transform_ClarkePark 行）,
//! `docs/C与Rust混合架构.md`

use crate::math::{INV_SQRT_3, SQRT_3};

/// 三相量（电流 `[A]` 或电压 `[V]`，由调用方约定）。
/// A three-phase quantity (currents in `[A]` or voltages in `[V]`, by convention
/// of the caller).
///
/// 三个分量必须来自**同一采样时刻**：分别读取再拼装会引入相间时间偏斜，
/// 表现为 dq 上出现与转速同频的纹波，而电流环看起来仍然"稳定"。
/// All three components must come from the same sampling instant; reading them
/// separately adds inter-phase skew that shows up as speed-synchronous ripple on
/// dq while the loop still looks stable.
///
/// `#[repr(C)]` 只为稳定布局，当前 crate 不导出 C ABI 符号。
/// `#[repr(C)]` only prepares a stable layout; no C ABI symbol is exported today.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Abc {
    /// A 相分量（电流 `[A]` 或电压 `[V]`）。
    /// Phase A component, in `[A]` or `[V]`.
    pub a: f32,
    /// B 相分量（电流 `[A]` 或电压 `[V]`）。
    /// Phase B component, in `[A]` or `[V]`.
    pub b: f32,
    /// C 相分量（电流 `[A]` 或电压 `[V]`）。
    /// Phase C component, in `[A]` or `[V]`.
    pub c: f32,
}

/// 静止两相量 αβ，与三相量同量纲。
/// Stationary two-phase quantity αβ, in the same unit as the three-phase input.
///
/// α 轴与 A 相轴重合；β 轴在 α 轴逆时针 90° 处，正方向由相序决定。
/// The α axis coincides with phase A; β leads α by 90° in the direction fixed by
/// the phase order.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AlphaBeta {
    /// α 轴分量，与输入同量纲。
    /// Alpha-axis component, in the same unit as the input.
    pub alpha: f32,
    /// β 轴分量，与输入同量纲。
    /// Beta-axis component, in the same unit as the input.
    pub beta: f32,
}

/// 转子旋转坐标系量 dq，与三相量同量纲。
/// Rotor rotating-frame quantity dq, in the same unit as the three-phase input.
///
/// d 轴对准转子磁链（电流即励磁分量），q 轴超前 d 轴 90°（电流即转矩分量）。
/// 因此这两个分量的物理解释依赖 `theta_rad` 的正方向约定和编码器零位。
/// The d axis follows the rotor flux, so its current is the magnetising component
/// and the q current is the torque component. Both meanings depend on the
/// `theta_rad` direction convention and the encoder zero offset.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Dq {
    /// d 轴分量（励磁），与输入同量纲。
    /// d-axis (magnetising) component, in the same unit as the input.
    pub d: f32,
    /// q 轴分量（转矩），与输入同量纲。
    /// q-axis (torque) component, in the same unit as the input.
    pub q: f32,
}

/// Clarke 变换：`abc -> αβ`，输入输出同量纲（通常 `[A]`）。
/// Clarke transform: `abc -> αβ`, same unit in and out (usually `[A]`).
///
/// 参数 / Parameters:
///   input 三相瞬时量 `Abc`；隐含假设三相之和为零（三线制、无中线电流）。
///
/// 返回 / Returns: `AlphaBeta { alpha = a, beta = (a + 2b)/√3 }`。
///
/// 形式与等价性（重要）/ Form and equivalence (important):
///   这是**幅值不变**形式，不是功率不变形式。系数 `1/√3` 与固件 C 平台适配
///   路径共用同一常量，两侧换算必须逐位可对照；改写为 `2/3` 系数（功率不变）
///   会让 `matches_c_reference_vectors` 直接失配，并让所有 PI 增益的物理含义
///   整体改变。
///   This is the amplitude-invariant form, not the power-invariant one. The
///   `1/√3` factor is shared with the C platform adapter, so rewriting it with a
///   `2/3` factor breaks `matches_c_reference_vectors` and changes the physical
///   meaning of every PI gain.
///
/// 只用了 A、B 两个相：第三相由 `a + b + c = 0` 在三线制下消去，因此传入的
/// `c` 字段不参与运算。现场若采用"两相重构"（ADC 只采两相，第三相由固件算），
/// 直接走本函数即可。
/// Only phases A and B are used; `c` disappears through `a + b + c = 0` in a
/// three-wire system, so `input.c` does not participate. A two-shunt
/// reconstruction path can call this directly.
#[inline]
pub fn clarke(input: Abc) -> AlphaBeta {
    AlphaBeta {
        alpha: input.a,
        beta: (input.a + 2.0 * input.b) * INV_SQRT_3,
    }
}

/// 两相输入版 Clarke：由 A、B 两相直接得到 αβ。
/// Two-input Clarke: builds αβ from phases A and B alone.
///
/// 参数 / Parameters:
///   ia A 相分量（通常 `[A]`）
///   ib B 相分量（通常 `[A]`）
///
/// 返回 / Returns: 与 `clarke()` 相同量纲的 `AlphaBeta`。
///
/// 用途与陷阱 / Use and pitfalls:
///   专为只采两相电流的硬件准备（如 `foc-platform` 中按 `c = -(a + b)` 重构
///   第三相）。传入的 `c` 只是形式占位，会被 `clarke()` 忽略，所以"重构"不会
///   引入额外误差；但**三相不平衡**（有中线电流、采样偏置不同）时这个假设不
///   成立，dq 上会出现基频纹波。
///   Meant for hardware that samples only two phase currents. The synthesised `c`
///   is a formal placeholder that `clarke()` ignores, so the reconstruction adds
///   no error; an unbalanced three-phase system breaks the assumption.
#[inline]
pub fn clarke_two_phase(ia: f32, ib: f32) -> AlphaBeta {
    clarke(Abc {
        a: ia,
        b: ib,
        c: -(ia + ib),
    })
}

/// 逆 Clarke 变换：`αβ -> abc`，输入输出同量纲。
/// Inverse Clarke transform: `αβ -> abc`, same unit in and out.
///
/// 参数 / Parameters:
///   input αβ 分量
///
/// 返回 / Returns: 三相分量，满足 `a + b + c = 0`。
///
/// 与 `clarke()` 的关系 / Relation to `clarke()`:
///   采用与正变换一致的幅值不变约定；`a` 直接取 `α`，b/c 由 `β` 的 `±√3/2`
///   分量构成，两者互为逆运算（测试用 `clarke` 后 `inverse_clarke` 验证往返）。
///   仿真里也用它把 αβ 电压还原成三相电压。
///   Uses the same amplitude-invariant convention as `clarke()`, so the two are
///   exact inverses; the simulator also uses it to turn αβ voltages back into
///   phase voltages.
#[inline]
pub fn inverse_clarke(input: AlphaBeta) -> Abc {
    Abc {
        a: input.alpha,
        b: -0.5 * input.alpha + 0.5 * SQRT_3 * input.beta,
        c: -0.5 * input.alpha - 0.5 * SQRT_3 * input.beta,
    }
}

/// Park 变换：`αβ -> dq`，把静止坐标系量投到转子上。
/// Park transform: `αβ -> dq`, projecting the stationary quantity onto the rotor.
///
/// 参数 / Parameters:
///   input     αβ 分量（电流 `[A]` 或电压 `[V]`）
///   theta_rad 转子**电角度** `[rad]`，不是机械角度
///
/// 返回 / Returns: dq 分量，与输入同量纲。
///
/// 角度约定（最容易出错的地方）/ Angle convention (the usual source of bugs):
///   `theta_rad` 必须是 `机械角度 × 极对数 + 电角度零位偏置`。传机械角度会让
///   极对数大于 1 的电机在高转速下完全失控。低速小角度下用错角度往往仍能"闭环
///   稳定"，因此必须用示波器或已知转向验证符号和零位，而不是只看能否转起来。
///   `theta_rad` must be the electrical angle (mechanical angle x pole pairs plus
///   the zero offset). Feeding the mechanical angle makes any motor with more than
///   one pole pair uncontrollable at speed; low-speed tests can still look stable,
///   so verify sign and zero offset against a scope or a known direction.
///
/// 实时成本 / Real-time cost: 需要一组 `sinf`/`cosf`（约数百周期软件实现）。
///   真实快环由 `foc-control` 算一次并让 Park/逆 Park 共享，不要在此重复计算。
///   Needs one sin/cos pair; the real fast loop shares a single pair between Park
///   and inverse Park.
#[inline]
pub fn park(input: AlphaBeta, theta_rad: f32) -> Dq {
    let s = libm::sinf(theta_rad);
    let c = libm::cosf(theta_rad);
    Dq {
        d: input.alpha * c + input.beta * s,
        q: -input.alpha * s + input.beta * c,
    }
}

/// 逆 Park 变换：`dq -> αβ`，把控制输出转回静止坐标系。
/// Inverse Park transform: `dq -> αβ`, turning the control output back into the
/// stationary frame.
///
/// 参数 / Parameters:
///   input     dq 分量（通常是 PI 输出的电压 `[V]`）
///   theta_rad 与 `park()` **同一次**采样使用的电角度 `[rad]`
///
/// 返回 / Returns: αβ 分量，与输入同量纲，可直接交给 SVPWM。
///
/// 等价性与实现说明 / Equivalence and implementation note:
///   数学上等于用 `-theta_rad` 调用 `park()`，这里单独实现是为了避免多一次
///   符号约定转换和一次额外的三角函数调用；两者必须与 `park()` 用**同一个**
///   `theta_rad`，否则 dq 旋转会引入两倍角度误差，表现为转矩与电流指令不符。
///   Mathematically it is `park(-theta_rad)`; the separate implementation avoids an
///   extra sign convention and trig call. It must use the same `theta_rad` as the
///   matching `park()`, otherwise the dq rotation picks up twice the angle error.
#[inline]
pub fn inverse_park(input: Dq, theta_rad: f32) -> AlphaBeta {
    let s = libm::sinf(theta_rad);
    let c = libm::cosf(theta_rad);
    AlphaBeta {
        alpha: input.d * c - input.q * s,
        beta: input.d * s + input.q * c,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::PI;

    /// 带容差的浮点比较辅助函数；容差由每个断言显式给出。
    /// Float comparison helper with an explicit per-assertion tolerance.
    fn assert_near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 与 C 版 `Transform_ClarkePark` 原测试向量逐项对照。
    /// Replays the original C `Transform_ClarkePark` test vectors.
    ///
    /// 四组断言各自锁定一件事：`(1, -0.5, -0.5)` 经幅值不变 Clarke 后
    /// `β = 0` 且 `α = 1`（锁定 `1/√3` 系数与 β 轴符号）、`inverse_clarke` 能
    /// 往返、`park(αβ, π/2)` 给出 `q = -1`（锁定 dq 旋转方向）、`inverse_park`
    /// 能往返。任何一项改动都会让本测试失配，而符号错误在实机上只表现为转矩
    /// 方向相反。
    /// Four groups pin the `1/√3` factor and β sign, the `inverse_clarke` round
    /// trip, the dq rotation direction, and the `inverse_park` round trip. A sign
    /// error here only shows up on hardware as reversed torque.
    #[test]
    fn matches_c_reference_vectors() {
        let abc = Abc {
            a: 1.0,
            b: -0.5,
            c: -0.5,
        };
        let ab = clarke(abc);
        assert_near(ab.alpha, 1.0, 1e-6);
        assert_near(ab.beta, 0.0, 1e-6);

        let abc_back = inverse_clarke(ab);
        assert_near(abc_back.a, abc.a, 1e-6);
        assert_near(abc_back.b, abc.b, 1e-6);
        assert_near(abc_back.c, abc.c, 1e-6);

        let dq = park(ab, PI * 0.5);
        assert_near(dq.d, 0.0, 1e-5);
        assert_near(dq.q, -1.0, 1e-5);
        let ab_back = inverse_park(dq, PI * 0.5);
        assert_near(ab_back.alpha, ab.alpha, 1e-5);
        assert_near(ab_back.beta, ab.beta, 1e-5);
    }
}
