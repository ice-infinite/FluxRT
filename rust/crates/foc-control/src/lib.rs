#![no_std]
#![forbid(unsafe_code)]

//! Board-independent motor-control composition and hardware ports.
//!
//! This crate contains the control topology shared by the STM32 target and the
//! host simulation. It deliberately has no RTOS, HAL, allocator, or MCU types.

pub mod controller;
pub mod math;
pub mod observer;
pub mod params;
pub mod ports;
pub mod runtime;
pub mod startup;
pub mod types;

pub use controller::*;
pub use math::*;
pub use observer::*;
pub use params::*;
pub use ports::*;
pub use runtime::*;
pub use startup::*;
pub use types::*;
