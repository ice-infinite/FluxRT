use foc_algorithm::{AlphaBeta, Dq};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// One PWM-period current snapshot in amperes.
pub struct PhaseCurrents {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RotorFeedback {
    pub electrical_angle_rad: f32,
    pub mechanical_speed_rad_s: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedbackSnapshot {
    pub currents: PhaseCurrents,
    pub dc_bus_voltage: f32,
    pub rotor: RotorFeedback,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpeedCommand {
    pub target_rpm: f32,
    pub id_ref_a: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CurrentCommand {
    pub id_ref_a: f32,
    pub iq_ref_a: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PwmCommand {
    pub duty_a: f32,
    pub duty_b: f32,
    pub duty_c: f32,
}

impl Default for PwmCommand {
    fn default() -> Self {
        Self {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
        }
    }
}

impl PwmCommand {
    pub fn is_valid(&self) -> bool {
        self.duty_a.is_finite()
            && self.duty_b.is_finite()
            && self.duty_c.is_finite()
            && (0.0..=1.0).contains(&self.duty_a)
            && (0.0..=1.0).contains(&self.duty_b)
            && (0.0..=1.0).contains(&self.duty_c)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlTelemetry {
    pub current_reference: CurrentCommand,
    pub current_dq: Dq,
    pub voltage_dq: Dq,
    pub voltage_alpha_beta: AlphaBeta,
    pub measured_speed_rpm: f32,
    pub voltage_limited: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HardwareFault {
    FeedbackUnavailable,
    InvalidFeedback,
    PowerStageFault,
    OutputRejected,
}
