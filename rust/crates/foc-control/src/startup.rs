use core::f32::consts::PI;

use foc_algorithm::{clamp, wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi};

use crate::CurrentCommand;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RevUpPhase {
    Alignment,
    OpenLoopRamp,
    OpenLoopHold,
    ObserverTransition,
    ClosedLoop,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RevUpOutput {
    pub phase: RevUpPhase,
    pub current_reference: CurrentCommand,
    pub forced_electrical_angle_rad: f32,
    pub angle_for_control_rad: f32,
    pub use_observer_speed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RevUpConfig {
    pub alignment_s: f32,
    pub ramp_s: f32,
    pub transition_s: f32,
    pub final_speed_rpm: f32,
    pub final_current_a: f32,
}

impl RevUpConfig {
    pub fn is_valid(&self) -> bool {
        self.alignment_s.is_finite()
            && self.ramp_s.is_finite()
            && self.transition_s.is_finite()
            && self.final_speed_rpm.is_finite()
            && self.final_current_a.is_finite()
            && self.alignment_s > 0.0
            && self.ramp_s > 0.0
            && self.transition_s >= 0.0
            && self.final_speed_rpm > 0.0
            && self.final_current_a > 0.0
    }
}

impl Default for RevUpConfig {
    fn default() -> Self {
        Self {
            alignment_s: 1.0,
            ramp_s: 1.164,
            transition_s: 0.025,
            final_speed_rpm: 582.0,
            final_current_a: 0.8,
        }
    }
}

/// MCSDK-reference rev-up timing: 1000 ms alignment, 1164 ms open-loop ramp
/// to 582 rpm, then 25 ms angle transition to the observer.
#[derive(Clone, Copy, Debug)]
pub struct RevUpSequencer {
    pole_pairs: u8,
    config: RevUpConfig,
    elapsed_s: f32,
    transition_elapsed_s: f32,
    transition_started: bool,
    closed_loop: bool,
    handoff_speed_rpm: f32,
    handoff_start_iq_a: f32,
    handoff_end_iq_a: f32,
    forced_electrical_angle_rad: f32,
}

impl RevUpSequencer {
    pub fn new(pole_pairs: u8) -> Self {
        Self::with_config(pole_pairs, RevUpConfig::default())
    }

    pub fn with_config(pole_pairs: u8, config: RevUpConfig) -> Self {
        Self {
            pole_pairs,
            config,
            elapsed_s: 0.0,
            transition_elapsed_s: 0.0,
            transition_started: false,
            closed_loop: false,
            handoff_speed_rpm: 0.0,
            handoff_start_iq_a: 0.0,
            handoff_end_iq_a: 0.0,
            forced_electrical_angle_rad: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.elapsed_s = 0.0;
        self.transition_elapsed_s = 0.0;
        self.transition_started = false;
        self.closed_loop = false;
        self.handoff_speed_rpm = 0.0;
        self.handoff_start_iq_a = 0.0;
        self.handoff_end_iq_a = 0.0;
        self.forced_electrical_angle_rad = 0.0;
    }

    /// Advances the open-loop rev-up and starts switch-over only after the
    /// observer has passed its convergence gate. `observer_iq_a` is captured
    /// in the observer reference frame and becomes the end point of the MCSDK-
    /// style torque-current ramp.
    pub fn update(
        &mut self,
        dt_s: f32,
        observer_angle_rad: f32,
        handoff_ready: bool,
        observer_iq_a: f32,
    ) -> RevUpOutput {
        let dt_s = dt_s.max(0.0);
        self.elapsed_s += dt_s;
        let alignment_complete = self.elapsed_s >= self.config.alignment_s;
        let ramp_progress = clamp(
            (self.elapsed_s - self.config.alignment_s) / self.config.ramp_s,
            0.0,
            1.0,
        );
        let open_loop_speed_rpm = self.config.final_speed_rpm * ramp_progress;
        let open_loop_phase = if !alignment_complete {
            RevUpPhase::Alignment
        } else if ramp_progress < 1.0 {
            RevUpPhase::OpenLoopRamp
        } else {
            RevUpPhase::OpenLoopHold
        };
        let open_loop_current_a = if !alignment_complete {
            self.config.final_current_a * self.elapsed_s / self.config.alignment_s
        } else {
            self.config.final_current_a
        };

        let mut transition_just_started = false;
        if !self.transition_started && !self.closed_loop && alignment_complete && handoff_ready {
            self.transition_started = true;
            transition_just_started = true;
            self.transition_elapsed_s = 0.0;
            self.handoff_speed_rpm = open_loop_speed_rpm;
            self.handoff_start_iq_a = open_loop_current_a;
            self.handoff_end_iq_a = clamp(
                observer_iq_a,
                -self.config.final_current_a,
                self.config.final_current_a,
            );
        }

        let (phase, current_a, forced_speed_rpm, transition) = if self.closed_loop {
            (
                RevUpPhase::ClosedLoop,
                self.handoff_end_iq_a,
                self.handoff_speed_rpm,
                1.0,
            )
        } else if self.transition_started {
            if !transition_just_started {
                self.transition_elapsed_s += dt_s;
            }
            let progress = if self.config.transition_s > 0.0 {
                clamp(
                    self.transition_elapsed_s / self.config.transition_s,
                    0.0,
                    1.0,
                )
            } else {
                1.0
            };
            let current_a = self.handoff_start_iq_a
                + progress * (self.handoff_end_iq_a - self.handoff_start_iq_a);
            if progress >= 1.0 {
                self.closed_loop = true;
                (
                    RevUpPhase::ClosedLoop,
                    self.handoff_end_iq_a,
                    self.handoff_speed_rpm,
                    1.0,
                )
            } else {
                (
                    RevUpPhase::ObserverTransition,
                    current_a,
                    self.handoff_speed_rpm,
                    progress,
                )
            }
        } else {
            (
                open_loop_phase,
                open_loop_current_a,
                open_loop_speed_rpm,
                0.0,
            )
        };

        let mechanical_speed_rad_s = forced_speed_rpm * PI / 30.0;
        self.forced_electrical_angle_rad = wrap_angle_0_to_2pi(
            self.forced_electrical_angle_rad
                + mechanical_speed_rad_s * self.pole_pairs as f32 * dt_s,
        );
        let angle_error =
            wrap_angle_minus_pi_to_pi(observer_angle_rad - self.forced_electrical_angle_rad);
        let angle_for_control_rad =
            wrap_angle_0_to_2pi(self.forced_electrical_angle_rad + transition * angle_error);
        RevUpOutput {
            phase,
            current_reference: CurrentCommand {
                id_ref_a: 0.0,
                iq_ref_a: current_a,
            },
            forced_electrical_angle_rad: self.forced_electrical_angle_rad,
            angle_for_control_rad,
            use_observer_speed: phase == RevUpPhase::ClosedLoop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn follows_generated_rev_up_timing() {
        let mut startup = RevUpSequencer::new(7);
        let mut output = startup.update(0.999, 0.0, false, 0.0);
        assert_eq!(output.phase, RevUpPhase::Alignment);
        assert!(output.current_reference.iq_ref_a < 0.8);
        output = startup.update(0.002, 0.0, false, 0.0);
        assert_eq!(output.phase, RevUpPhase::OpenLoopRamp);
        output = startup.update(1.163, 0.0, false, 0.0);
        assert_eq!(output.phase, RevUpPhase::OpenLoopHold);
        output = startup.update(0.001, 0.25, true, 0.35);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        output = startup.update(0.012, 0.25, false, 0.0);
        assert!(output.current_reference.iq_ref_a < 0.8);
        output = startup.update(0.014, 0.25, false, 0.0);
        assert_eq!(output.phase, RevUpPhase::ClosedLoop);
        assert!(output.use_observer_speed);
        assert!((output.current_reference.iq_ref_a - 0.35).abs() < 1e-6);
    }

    #[test]
    fn observer_can_start_transition_before_ramp_endpoint() {
        let mut startup = RevUpSequencer::new(7);
        let output = startup.update(2.05, 1.0, true, 0.5);
        assert_eq!(output.phase, RevUpPhase::ObserverTransition);
        assert!(output.forced_electrical_angle_rad.is_finite());
        assert!((output.current_reference.iq_ref_a - 0.8).abs() < 1e-6);
    }
}
