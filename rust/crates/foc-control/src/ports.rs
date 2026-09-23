use crate::{FeedbackSnapshot, HardwareFault, PwmCommand};

/// Provides one synchronized ADC/position snapshot for a PWM period.
pub trait FeedbackPort {
    fn read_feedback(&mut self) -> Result<FeedbackSnapshot, HardwareFault>;
}

/// The only port allowed to pass a duty command to a power-stage adapter.
pub trait PwmPort {
    fn apply_pwm(&mut self, command: PwmCommand) -> Result<(), HardwareFault>;
    fn disable_pwm(&mut self);
}

/// Represents asynchronous hardware protection (break, comparator, gate fault).
pub trait SafetyPort {
    fn power_stage_faulted(&self) -> bool;
}

pub trait MotorHardware: FeedbackPort + PwmPort + SafetyPort {}

impl<T> MotorHardware for T where T: FeedbackPort + PwmPort + SafetyPort {}
