#![no_std]
#![forbid(unsafe_code)]

//! `lib` 目录中 C 版 FOC 算法库的 Rust `no_std` 等价实现。
//!
//! 当前版本完整覆盖相邻 C 库的 63 个生产模块，包括基础 FOC、调制、
//! 观测器、状态估计器、高级控制器和优化辅助算法。所有实时状态由调用方持有，
//! 不使用堆、不使用全局可变状态，也不包含芯片 HAL 依赖。

pub mod adrc;
pub mod advanced_control;
pub mod angle;
pub mod commutation;
pub mod controller;
pub mod filter;
pub mod foc;
pub mod genetic;
pub mod learning;
pub mod math;
pub mod modulation;
pub mod observer;
pub mod optimization;
pub mod transform;
pub mod utility;

pub use adrc::*;
pub use advanced_control::*;
pub use angle::*;
pub use commutation::*;
pub use controller::*;
pub use filter::*;
pub use foc::*;
pub use genetic::*;
pub use learning::*;
pub use math::*;
pub use modulation::*;
pub use observer::*;
pub use optimization::*;
pub use transform::*;
pub use utility::*;
