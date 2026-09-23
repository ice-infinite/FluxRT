use crate::{
    ControlTelemetry, HardwareFault, MotorHardware, PwmCommand, SpeedCommand, StReferenceController,
};

pub struct ControlRuntime<H> {
    controller: StReferenceController,
    hardware: H,
    enabled: bool,
}

impl<H: MotorHardware> ControlRuntime<H> {
    pub fn new(controller: StReferenceController, hardware: H) -> Self {
        Self {
            controller,
            hardware,
            enabled: false,
        }
    }

    pub fn enable(&mut self) -> Result<(), HardwareFault> {
        if self.hardware.power_stage_faulted() {
            self.hardware.disable_pwm();
            return Err(HardwareFault::PowerStageFault);
        }
        self.controller.reset();
        self.enabled = true;
        Ok(())
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.controller.reset();
        self.hardware.disable_pwm();
    }

    pub fn tick(&mut self, command: SpeedCommand) -> Result<ControlTelemetry, HardwareFault> {
        if !self.enabled || self.hardware.power_stage_faulted() {
            self.disable();
            return Err(HardwareFault::PowerStageFault);
        }
        let feedback = match self.hardware.read_feedback() {
            Ok(feedback) => feedback,
            Err(fault) => {
                self.disable();
                return Err(fault);
            }
        };
        if !feedback.dc_bus_voltage.is_finite()
            || feedback.dc_bus_voltage <= 0.0
            || !feedback.currents.a.is_finite()
            || !feedback.currents.b.is_finite()
            || !feedback.currents.c.is_finite()
            || !feedback.rotor.electrical_angle_rad.is_finite()
            || !feedback.rotor.mechanical_speed_rad_s.is_finite()
        {
            self.disable();
            return Err(HardwareFault::InvalidFeedback);
        }
        let (pwm, telemetry) = self.controller.update(&feedback, command);
        if !pwm.is_valid() {
            self.disable();
            return Err(HardwareFault::OutputRejected);
        }
        if let Err(fault) = self.hardware.apply_pwm(pwm) {
            self.disable();
            return Err(fault);
        }
        Ok(telemetry)
    }

    pub fn hardware(&self) -> &H {
        &self.hardware
    }

    pub fn hardware_mut(&mut self) -> &mut H {
        &mut self.hardware
    }

    pub fn controller_period_s(&self) -> f32 {
        1.0 / self.controller.parameters().pwm_frequency_hz as f32
    }

    pub fn last_safe_pwm() -> PwmCommand {
        PwmCommand::default()
    }
}
