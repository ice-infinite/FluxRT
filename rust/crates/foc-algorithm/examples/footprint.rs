//! 主机侧状态对象尺寸报告示例（`cargo run --release --example footprint`）。
//! Host-side state-object size report example.
//!
//! 这是示例二进制（crate root），不是 `foc-algorithm` 的库模块，因此不参与
//! `no_std` 目标构建，只在开发主机上运行。
//! This is an example binary (crate root), not a library module: it is never built for
//! the `no_std` target and only runs on the development host.
//!
//! 职责 / Responsibility:
//!   - 打印每个状态类型的 `core::mem::size_of`，用于记录本库"定长、无分配"的
//!     实时状态尺寸。这些数字就是 `算法库实时性说明.md` 表格里记录的内容。
//!   - 按固定顺序逐行输出，便于与文档表格逐行比对。
//!
//! 架构位置 / Position in the architecture:
//!   开发主机工具；不进入 applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI)
//!   -> foc-control -> foc-algorithm 这条目标侧实时链路。
//!
//! 边界与限制 / Limits:
//!   - 数值来自 Windows x86_64 release，与 `算法库实时性说明.md` 的记录一致。
//!     本库状态只含 `f32`/`i32` 定长字段，但目标 ABI 的对齐与填充仍应以
//!     `thumbv7em` 交叉构建出的 map 为准。
//!   - 对象大小只说明内存占用，不证明实时行为：周期时间与栈水位必须用 DWT
//!     周期计数器或定时器实测（见 `算法库实时性说明.md`）。
//!   - 这里只调用 `core::mem::size_of`，不涉及定点格式：`i16` Q1.15 与
//!     `i32` Q1.31 的换算是本库其他文件的职责，与尺寸报告无关。
//!   - `println!` 只能在主机工具里用，目标固件的实时路径禁止日志。
//!   - 不要改动打印的名称或顺序：文档表格按这些名称记录。

use core::mem::size_of;

// 只导入状态类型本身：这里不做任何算法计算，因此不需要引入数学函数或参数类型。
// Only state types are imported; no algorithm is executed here.
use foc_algorithm::{
    AdaptiveSmoState, AdaptiveState, AdrcFastTdState, AdrcState, CascadeState, EkfFocState,
    EkfState, FocBasicState, FuzzyState, HfBemfState, HfInjectionState, HigherOrderSmoState,
    KalmanState, LuenbergerState, MpcState, NeuralNetworkState, PulseInjectionState,
    ReinforcementLearningState, RotatingHfState, SuperTwistingSmoState, UkfState,
};

/// 打印一个状态类型的字节大小，格式为 `名字=字节数`。
/// Prints one state type's size in bytes as `name=bytes`.
///
/// 泛型参数只在编译期决定单态化实例，运行时只是取一个常量，因此这个函数自身
/// 没有测量开销，也不反映任何时间特性。
/// The generic parameter only selects a monomorphised instantiation; this function
/// measures nothing about timing.
///
/// 上下文 / Context: 仅主机工具；目标固件里没有 `println!`。
fn print_size<T>(name: &str) {
    println!("{name}={}", size_of::<T>());
}

/// 逐个打印本库实时状态对象的尺寸。
/// Prints the size of each real-time state object in turn.
///
/// 顺序刻意保持固定：`算法库实时性说明.md` 的表格按同一组名称记录，便于逐行核对。
/// 这里不做排序、不做差值、不做统计；类型名缺失会在编译期报错，而不是在运行时
/// 静默少打印一行。
/// The order is deliberately fixed so the output can be compared line by line against
/// `算法库实时性说明.md`; a missing type is a compile error, not a silent gap.
///
/// 注意 / Note: 打印出的字节数只描述内存占用，不证明这些状态能在 12 kHz 控制
/// 周期内算完；周期时间必须另行实测。
/// The byte counts describe memory only, not whether the state can be updated inside a
/// 12 kHz period.
fn main() {
    print_size::<FocBasicState>("FocBasicState");
    print_size::<CascadeState>("CascadeState");
    print_size::<AdaptiveSmoState>("AdaptiveSmoState");
    print_size::<HigherOrderSmoState>("HigherOrderSmoState");
    print_size::<SuperTwistingSmoState>("SuperTwistingSmoState");
    print_size::<HfInjectionState>("HfInjectionState");
    print_size::<RotatingHfState>("RotatingHfState");
    print_size::<PulseInjectionState>("PulseInjectionState");
    print_size::<HfBemfState>("HfBemfState");
    print_size::<KalmanState>("KalmanState");
    print_size::<LuenbergerState>("LuenbergerState");
    print_size::<EkfState>("EkfState");
    print_size::<EkfFocState>("EkfFocState");
    print_size::<UkfState>("UkfState");
    print_size::<AdrcState>("AdrcState");
    print_size::<AdrcFastTdState>("AdrcFastTdState");
    print_size::<AdaptiveState>("AdaptiveState");
    print_size::<FuzzyState>("FuzzyState");
    print_size::<MpcState>("MpcState");
    print_size::<NeuralNetworkState>("NeuralNetworkState");
    // 最大的一个：内部是 8x4 个 `f32` 的 Q 表（128 字节）加两个簿记字段，
    // 合计 136 字节，与 `算法库实时性说明.md` 的记录一致。
    // The largest one: 128 bytes of Q values plus two bookkeeping fields.
    print_size::<ReinforcementLearningState>("ReinforcementLearningState");
}
