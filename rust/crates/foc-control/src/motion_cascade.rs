//! Board-independent Position P -> Velocity PI -> Torque/Iq cascade.
//!
//! P4.2B consumes the bounded [`MotionReference`] emitted by P4.2A.  The module
//! deliberately remains behind the default-off `motion-control` feature. P4.2C
//! added persisted-configuration mapping and an independent ABI, but the core
//! is still not wired to the target ISR or production enable path.

use foc_algorithm::{PiParam, PiState};

use crate::{
    input_is_compatible, ControlMode, CurrentCommand, InputMode, MotionReference,
    MOTION_REFERENCE_FLAG_KNOWN_MASK, MOTION_REFERENCE_FLAG_TRANSITION,
};

/// The position loop contributed to the velocity reference.
pub const MOTION_CASCADE_FLAG_POSITION_LOOP_ACTIVE: u32 = 1 << 0;
/// The velocity loop contributed to the torque reference.
pub const MOTION_CASCADE_FLAG_VELOCITY_LOOP_ACTIVE: u32 = 1 << 1;
/// Controller state was preloaded from measured q-axis current.
pub const MOTION_CASCADE_FLAG_TRANSITION_PRELOADED: u32 = 1 << 2;
/// The combined position-loop/feed-forward velocity was limited.
pub const MOTION_CASCADE_FLAG_VELOCITY_LIMITED: u32 = 1 << 3;
/// The combined PI/feed-forward torque was limited.
pub const MOTION_CASCADE_FLAG_TORQUE_LIMITED: u32 = 1 << 4;
/// The current-derived torque ceiling was tighter than the torque ceiling.
pub const MOTION_CASCADE_FLAG_CURRENT_LIMITED: u32 = 1 << 5;

/// Cascade gains and motor conversion in SI units.
///
/// `torque_constant_nm_per_a` is a motor parameter supplied by the future
/// configuration mapper.  It is intentionally not a board or motor constant in
/// this module.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionCascadeConfig {
    /// Cascade update period `[s]`.
    pub control_period_s: f32,
    /// Position proportional gain `(rad/s)/rad = 1/s`.
    pub position_kp_per_s: f32,
    /// Velocity proportional gain `N*m/(rad/s)`.
    pub velocity_kp_nm_per_rad_s: f32,
    /// Velocity integral gain `N*m/rad`.
    pub velocity_ki_nm_per_rad: f32,
    /// Motor torque constant `N*m/A`.
    pub torque_constant_nm_per_a: f32,
}

impl MotionCascadeConfig {
    pub fn validate(&self) -> Result<(), MotionCascadeError> {
        let finite = [
            self.control_period_s,
            self.position_kp_per_s,
            self.velocity_kp_nm_per_rad_s,
            self.velocity_ki_nm_per_rad,
            self.torque_constant_nm_per_a,
        ]
        .iter()
        .all(|value| value.is_finite());
        if !finite
            || self.control_period_s <= 0.0
            || self.control_period_s > 1.0
            || self.position_kp_per_s <= 0.0
            || self.velocity_kp_nm_per_rad_s < 0.0
            || self.velocity_ki_nm_per_rad < 0.0
            || self.velocity_kp_nm_per_rad_s + self.velocity_ki_nm_per_rad <= 0.0
            || self.torque_constant_nm_per_a <= 0.0
        {
            return Err(MotionCascadeError::InvalidConfig);
        }
        Ok(())
    }
}

/// Mechanical feedback and actually applied q-axis current for one cascade tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotionCascadeFeedback {
    pub position_valid: bool,
    pub velocity_valid: bool,
    pub current_q_valid: bool,
    pub mechanical_position_rad: f32,
    pub mechanical_velocity_rad_s: f32,
    pub current_q_a: f32,
}

/// One deterministic cascade result in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionCascadeOutput {
    pub control_mode: ControlMode,
    pub input_mode: InputMode,
    pub position_error_rad: f32,
    pub velocity_reference_rad_s: f32,
    pub velocity_error_rad_s: f32,
    pub torque_reference_nm: f32,
    pub current_reference: CurrentCommand,
    /// Original P4.2A planner flags, kept separate for traceability.
    pub reference_flags: u32,
    /// Flags produced by this cascade.
    pub flags: u32,
}

impl Default for MotionCascadeOutput {
    fn default() -> Self {
        Self {
            control_mode: ControlMode::Inactive,
            input_mode: InputMode::Inactive,
            position_error_rad: 0.0,
            velocity_reference_rad_s: 0.0,
            velocity_error_rad_s: 0.0,
            torque_reference_nm: 0.0,
            current_reference: CurrentCommand::default(),
            reference_flags: 0,
            flags: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionCascadeError {
    Disabled,
    InvalidConfig,
    InvalidReference,
    UnknownReferenceFlags,
    UnsupportedControlMode,
    UnsupportedInputMode,
    MissingPositionFeedback,
    MissingVelocityFeedback,
    MissingCurrentFeedback,
    InvalidFeedback,
    TransitionRequired,
    NonFiniteComputation,
}

/// Stateful cascade with transactional updates and explicit no-bump handoff.
#[derive(Clone, Copy, Debug)]
pub struct MotionCascadeController {
    enabled: bool,
    initialized: bool,
    control_mode: ControlMode,
    input_mode: InputMode,
    velocity_pi: PiState,
    output: MotionCascadeOutput,
}

impl Default for MotionCascadeController {
    fn default() -> Self {
        Self {
            enabled: false,
            initialized: false,
            control_mode: ControlMode::Inactive,
            input_mode: InputMode::Inactive,
            velocity_pi: PiState::default(),
            output: MotionCascadeOutput::default(),
        }
    }
}

impl MotionCascadeController {
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Enabling and disabling both discard prior controller state.
    pub fn set_enabled(&mut self, enabled: bool) {
        *self = Self {
            enabled,
            ..Self::default()
        };
    }

    pub fn current_output(&self) -> MotionCascadeOutput {
        self.output
    }

    /// Velocity-PI integral in torque units `[N*m]`, exposed for diagnostics.
    pub fn velocity_integrator_nm(&self) -> f32 {
        self.velocity_pi.integrator
    }

    /// Run one cascade tick.  Every error leaves the complete controller state
    /// unchanged, including the PI integrator.
    pub fn step(
        &mut self,
        config: &MotionCascadeConfig,
        reference: &MotionReference,
        feedback: MotionCascadeFeedback,
    ) -> Result<MotionCascadeOutput, MotionCascadeError> {
        if !self.enabled {
            return Err(MotionCascadeError::Disabled);
        }
        config.validate()?;
        validate_reference(reference)?;
        validate_feedback(reference.control_mode, reference.flags, feedback)?;

        let state_changed = !self.initialized
            || self.control_mode != reference.control_mode
            || self.input_mode != reference.input_mode;
        let transition = reference.flags & MOTION_REFERENCE_FLAG_TRANSITION != 0;
        if state_changed && !transition {
            return Err(MotionCascadeError::TransitionRequired);
        }

        let mut next = *self;
        let output = next.step_inner(config, reference, feedback, transition)?;
        next.initialized = true;
        next.control_mode = reference.control_mode;
        next.input_mode = reference.input_mode;
        next.output = output;
        *self = next;
        Ok(output)
    }

    /// Realtime inner path for a configuration/reference already validated by
    /// the owning motion transaction.
    ///
    /// Live feedback and every finite-computation gate remain active.  The
    /// caller owns the outer shadow/commit, so this method avoids a second
    /// controller copy and publishes into `self` only inside that shadow.
    #[inline]
    pub fn step_prevalidated_realtime(
        &mut self,
        config: &MotionCascadeConfig,
        reference: &MotionReference,
        feedback: MotionCascadeFeedback,
    ) -> Result<MotionCascadeOutput, MotionCascadeError> {
        if !self.enabled {
            return Err(MotionCascadeError::Disabled);
        }
        validate_feedback(reference.control_mode, reference.flags, feedback)?;

        let state_changed = !self.initialized
            || self.control_mode != reference.control_mode
            || self.input_mode != reference.input_mode;
        let transition = reference.flags & MOTION_REFERENCE_FLAG_TRANSITION != 0;
        if state_changed && !transition {
            return Err(MotionCascadeError::TransitionRequired);
        }

        let output = self.step_inner(config, reference, feedback, transition)?;
        self.initialized = true;
        self.control_mode = reference.control_mode;
        self.input_mode = reference.input_mode;
        self.output = output;
        Ok(output)
    }

    fn step_inner(
        &mut self,
        config: &MotionCascadeConfig,
        reference: &MotionReference,
        feedback: MotionCascadeFeedback,
        transition: bool,
    ) -> Result<MotionCascadeOutput, MotionCascadeError> {
        let current_torque_limit = reference.current_limit_a * config.torque_constant_nm_per_a;
        if !current_torque_limit.is_finite() {
            return Err(MotionCascadeError::NonFiniteComputation);
        }
        let effective_torque_limit = reference.torque_limit_nm.min(current_torque_limit);
        let mut flags = 0;
        if current_torque_limit < reference.torque_limit_nm {
            flags |= MOTION_CASCADE_FLAG_CURRENT_LIMITED;
        }

        let position_error = if reference.control_mode == ControlMode::Position {
            reference.position_ref_rad - feedback.mechanical_position_rad
        } else {
            0.0
        };
        if !position_error.is_finite() {
            return Err(MotionCascadeError::NonFiniteComputation);
        }

        let mut velocity_reference = 0.0;
        let mut velocity_error = 0.0;
        let torque_reference = match reference.control_mode {
            ControlMode::Torque => {
                self.velocity_pi.reset();
                if transition {
                    flags |= MOTION_CASCADE_FLAG_TRANSITION_PRELOADED;
                    clamp_with_flag(
                        feedback.current_q_a * config.torque_constant_nm_per_a,
                        effective_torque_limit,
                        &mut flags,
                        MOTION_CASCADE_FLAG_TORQUE_LIMITED,
                    )
                } else {
                    clamp_with_flag(
                        reference.torque_ref_nm,
                        effective_torque_limit,
                        &mut flags,
                        MOTION_CASCADE_FLAG_TORQUE_LIMITED,
                    )
                }
            }
            ControlMode::Velocity | ControlMode::Position => {
                flags |= MOTION_CASCADE_FLAG_VELOCITY_LOOP_ACTIVE;
                velocity_reference = if reference.control_mode == ControlMode::Position {
                    flags |= MOTION_CASCADE_FLAG_POSITION_LOOP_ACTIVE;
                    let raw = reference.velocity_ref_rad_s
                        + reference.velocity_feedforward_rad_s
                        + config.position_kp_per_s * position_error;
                    if !raw.is_finite() {
                        return Err(MotionCascadeError::NonFiniteComputation);
                    }
                    clamp_with_flag(
                        raw,
                        reference.velocity_limit_rad_s,
                        &mut flags,
                        MOTION_CASCADE_FLAG_VELOCITY_LIMITED,
                    )
                } else {
                    clamp_with_flag(
                        reference.velocity_ref_rad_s,
                        reference.velocity_limit_rad_s,
                        &mut flags,
                        MOTION_CASCADE_FLAG_VELOCITY_LIMITED,
                    )
                };
                velocity_error = velocity_reference - feedback.mechanical_velocity_rad_s;
                if !velocity_error.is_finite() {
                    return Err(MotionCascadeError::NonFiniteComputation);
                }

                let feedforward = clamp_with_flag(
                    reference.torque_feedforward_nm,
                    effective_torque_limit,
                    &mut flags,
                    MOTION_CASCADE_FLAG_TORQUE_LIMITED,
                );
                let pi_param = PiParam {
                    kp: config.velocity_kp_nm_per_rad_s,
                    ki: config.velocity_ki_nm_per_rad,
                    ts: config.control_period_s,
                    out_min: -effective_torque_limit - feedforward,
                    out_max: effective_torque_limit - feedforward,
                    integrator_min: -effective_torque_limit - feedforward,
                    integrator_max: effective_torque_limit - feedforward,
                };
                let pi_torque = if transition {
                    flags |= MOTION_CASCADE_FLAG_TRANSITION_PRELOADED;
                    let applied_torque = clamp_symmetric(
                        feedback.current_q_a * config.torque_constant_nm_per_a,
                        effective_torque_limit,
                    );
                    self.velocity_pi.preload_output(
                        &pi_param,
                        velocity_reference,
                        feedback.mechanical_velocity_rad_s,
                        applied_torque - feedforward,
                    )
                } else {
                    self.velocity_pi.update(
                        &pi_param,
                        velocity_reference,
                        feedback.mechanical_velocity_rad_s,
                    )
                };
                if pi_torque <= pi_param.out_min || pi_torque >= pi_param.out_max {
                    flags |= MOTION_CASCADE_FLAG_TORQUE_LIMITED;
                }
                clamp_with_flag(
                    pi_torque + feedforward,
                    effective_torque_limit,
                    &mut flags,
                    MOTION_CASCADE_FLAG_TORQUE_LIMITED,
                )
            }
            _ => return Err(MotionCascadeError::UnsupportedControlMode),
        };

        let raw_iq = torque_reference / config.torque_constant_nm_per_a;
        if !raw_iq.is_finite() {
            return Err(MotionCascadeError::NonFiniteComputation);
        }
        let iq_ref = clamp_symmetric(raw_iq, reference.current_limit_a);
        if iq_ref != raw_iq {
            flags |= MOTION_CASCADE_FLAG_CURRENT_LIMITED | MOTION_CASCADE_FLAG_TORQUE_LIMITED;
        }
        let final_torque = iq_ref * config.torque_constant_nm_per_a;
        let output = MotionCascadeOutput {
            control_mode: reference.control_mode,
            input_mode: reference.input_mode,
            position_error_rad: position_error,
            velocity_reference_rad_s: velocity_reference,
            velocity_error_rad_s: velocity_error,
            torque_reference_nm: final_torque,
            current_reference: CurrentCommand {
                id_ref_a: 0.0,
                iq_ref_a: iq_ref,
            },
            reference_flags: reference.flags,
            flags,
        };
        if !output_is_finite(output) {
            return Err(MotionCascadeError::NonFiniteComputation);
        }
        Ok(output)
    }
}

fn validate_reference(reference: &MotionReference) -> Result<(), MotionCascadeError> {
    if reference.flags & !MOTION_REFERENCE_FLAG_KNOWN_MASK != 0 {
        return Err(MotionCascadeError::UnknownReferenceFlags);
    }
    if !matches!(
        reference.control_mode,
        ControlMode::Torque | ControlMode::Velocity | ControlMode::Position
    ) {
        return Err(MotionCascadeError::UnsupportedControlMode);
    }
    if !input_is_compatible(reference.control_mode, reference.input_mode) {
        return Err(MotionCascadeError::UnsupportedInputMode);
    }
    let finite = [
        reference.position_ref_rad,
        reference.velocity_ref_rad_s,
        reference.torque_ref_nm,
        reference.velocity_feedforward_rad_s,
        reference.torque_feedforward_nm,
        reference.current_limit_a,
        reference.torque_limit_nm,
        reference.velocity_limit_rad_s,
    ]
    .iter()
    .all(|value| value.is_finite());
    if !finite
        || reference.current_limit_a <= 0.0
        || reference.torque_limit_nm <= 0.0
        || reference.velocity_limit_rad_s <= 0.0
    {
        return Err(MotionCascadeError::InvalidReference);
    }
    Ok(())
}

fn validate_feedback(
    mode: ControlMode,
    reference_flags: u32,
    feedback: MotionCascadeFeedback,
) -> Result<(), MotionCascadeError> {
    if feedback.position_valid && !feedback.mechanical_position_rad.is_finite()
        || feedback.velocity_valid && !feedback.mechanical_velocity_rad_s.is_finite()
        || feedback.current_q_valid && !feedback.current_q_a.is_finite()
    {
        return Err(MotionCascadeError::InvalidFeedback);
    }
    if mode == ControlMode::Position && !feedback.position_valid {
        return Err(MotionCascadeError::MissingPositionFeedback);
    }
    if matches!(mode, ControlMode::Velocity | ControlMode::Position) && !feedback.velocity_valid {
        return Err(MotionCascadeError::MissingVelocityFeedback);
    }
    if reference_flags & MOTION_REFERENCE_FLAG_TRANSITION != 0 && !feedback.current_q_valid {
        return Err(MotionCascadeError::MissingCurrentFeedback);
    }
    Ok(())
}

fn clamp_with_flag(value: f32, limit: f32, flags: &mut u32, flag: u32) -> f32 {
    let clamped = clamp_symmetric(value, limit);
    if clamped != value {
        *flags |= flag;
    }
    clamped
}

/// Panic-free symmetric clamp. The configuration/reference validators prove
/// `limit` finite and positive; preserving NaN here lets the existing
/// finite-output gate reject the transaction without pulling float formatting
/// and panic machinery into the target image.
#[inline]
fn clamp_symmetric(value: f32, limit: f32) -> f32 {
    if value < -limit {
        -limit
    } else if value > limit {
        limit
    } else {
        value
    }
}

fn output_is_finite(output: MotionCascadeOutput) -> bool {
    [
        output.position_error_rad,
        output.velocity_reference_rad_s,
        output.velocity_error_rad_s,
        output.torque_reference_nm,
        output.current_reference.id_ref_a,
        output.current_reference.iq_ref_a,
    ]
    .iter()
    .all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        FeedbackMode, MotionFeedback, MotionPlannerConfig, MotionReferencePlanner, ProductCommand,
        ProductCommandKind,
    };

    fn config() -> MotionCascadeConfig {
        MotionCascadeConfig {
            control_period_s: 0.001,
            position_kp_per_s: 4.0,
            velocity_kp_nm_per_rad_s: 0.2,
            velocity_ki_nm_per_rad: 2.0,
            torque_constant_nm_per_a: 0.2,
        }
    }

    fn reference(mode: ControlMode, input: InputMode, transition: bool) -> MotionReference {
        MotionReference {
            control_mode: mode,
            input_mode: input,
            current_limit_a: 2.0,
            torque_limit_nm: 1.0,
            velocity_limit_rad_s: 10.0,
            flags: if transition {
                MOTION_REFERENCE_FLAG_TRANSITION
            } else {
                0
            },
            ..MotionReference::default()
        }
    }

    fn feedback(position: f32, velocity: f32, current_q: f32) -> MotionCascadeFeedback {
        MotionCascadeFeedback {
            position_valid: true,
            velocity_valid: true,
            current_q_valid: true,
            mechanical_position_rad: position,
            mechanical_velocity_rad_s: velocity,
            current_q_a: current_q,
        }
    }

    fn enabled_controller() -> MotionCascadeController {
        let mut controller = MotionCascadeController::default();
        controller.set_enabled(true);
        controller
    }

    #[test]
    fn default_is_disabled_and_enable_resets_state() {
        let mut controller = MotionCascadeController::default();
        let reference = reference(ControlMode::Torque, InputMode::Passthrough, true);
        assert_eq!(
            controller.step(&config(), &reference, feedback(0.0, 0.0, 0.0)),
            Err(MotionCascadeError::Disabled)
        );
        controller.set_enabled(true);
        assert!(controller.is_enabled());
        controller.set_enabled(false);
        assert!(!controller.is_enabled());
        assert_eq!(controller.current_output(), MotionCascadeOutput::default());
    }

    #[test]
    fn invalid_input_is_transactional() {
        let mut controller = enabled_controller();
        let before = controller.current_output();
        let mut bad_config = config();
        bad_config.torque_constant_nm_per_a = f32::NAN;
        let reference = reference(ControlMode::Torque, InputMode::Passthrough, true);
        assert_eq!(
            controller.step(&bad_config, &reference, feedback(0.0, 0.0, 0.0)),
            Err(MotionCascadeError::InvalidConfig)
        );
        assert_eq!(controller.current_output(), before);

        let mut bad_reference = reference;
        bad_reference.flags |= 1 << 31;
        assert_eq!(
            controller.step(&config(), &bad_reference, feedback(0.0, 0.0, 0.0)),
            Err(MotionCascadeError::UnknownReferenceFlags)
        );
        assert_eq!(controller.current_output(), before);
    }

    #[test]
    fn torque_mode_converts_to_iq_and_obeys_the_tighter_current_limit() {
        let mut controller = enabled_controller();
        let seed = reference(ControlMode::Torque, InputMode::Passthrough, true);
        controller
            .step(&config(), &seed, feedback(0.0, 0.0, 0.0))
            .unwrap();
        let mut command = reference(ControlMode::Torque, InputMode::Passthrough, false);
        command.torque_ref_nm = 0.8;
        let output = controller
            .step(&config(), &command, feedback(0.0, 0.0, 0.0))
            .unwrap();
        assert!((output.torque_reference_nm - 0.4).abs() < 1.0e-6);
        assert!((output.current_reference.iq_ref_a - 2.0).abs() < 1.0e-6);
        assert_ne!(output.flags & MOTION_CASCADE_FLAG_CURRENT_LIMITED, 0);
        assert_ne!(output.flags & MOTION_CASCADE_FLAG_TORQUE_LIMITED, 0);
    }

    #[test]
    fn transition_preload_preserves_applied_torque_in_every_mode() {
        for (mode, input) in [
            (ControlMode::Torque, InputMode::Passthrough),
            (ControlMode::Velocity, InputMode::VelocityRamp),
            (ControlMode::Position, InputMode::PositionFilter),
        ] {
            let mut controller = enabled_controller();
            let mut reference = reference(mode, input, true);
            if matches!(mode, ControlMode::Velocity | ControlMode::Position) {
                reference.velocity_ref_rad_s = 1.2;
            }
            if mode == ControlMode::Position {
                reference.position_ref_rad = 0.4;
            }
            let output = controller
                .step(&config(), &reference, feedback(0.4, 1.2, 0.75))
                .unwrap();
            assert!((output.torque_reference_nm - 0.15).abs() < 1.0e-6);
            assert!((output.current_reference.iq_ref_a - 0.75).abs() < 1.0e-6);
            assert_ne!(output.flags & MOTION_CASCADE_FLAG_TRANSITION_PRELOADED, 0);
        }
    }

    #[test]
    fn velocity_pi_adds_feedforward_and_tracks_error() {
        let mut controller = enabled_controller();
        let seed = reference(ControlMode::Velocity, InputMode::Passthrough, true);
        controller
            .step(&config(), &seed, feedback(0.0, 0.0, 0.0))
            .unwrap();
        let mut command = reference(ControlMode::Velocity, InputMode::Passthrough, false);
        command.velocity_ref_rad_s = 1.0;
        command.torque_feedforward_nm = 0.05;
        let output = controller
            .step(&config(), &command, feedback(0.0, 0.0, 0.0))
            .unwrap();
        assert!((output.torque_reference_nm - 0.252).abs() < 1.0e-6);
        assert_eq!(output.velocity_error_rad_s, 1.0);
        assert_ne!(output.flags & MOTION_CASCADE_FLAG_VELOCITY_LOOP_ACTIVE, 0);
    }

    #[test]
    fn position_loop_combines_planner_velocity_and_feedforward_then_limits() {
        let mut controller = enabled_controller();
        let seed = reference(ControlMode::Position, InputMode::PositionFilter, true);
        controller
            .step(&config(), &seed, feedback(0.0, 0.0, 0.0))
            .unwrap();
        let mut command = reference(ControlMode::Position, InputMode::PositionFilter, false);
        command.position_ref_rad = 3.0;
        command.velocity_ref_rad_s = 1.0;
        command.velocity_feedforward_rad_s = 1.0;
        let output = controller
            .step(&config(), &command, feedback(0.0, 0.0, 0.0))
            .unwrap();
        assert_eq!(output.velocity_reference_rad_s, 10.0);
        assert_ne!(output.flags & MOTION_CASCADE_FLAG_POSITION_LOOP_ACTIVE, 0);
        assert_ne!(output.flags & MOTION_CASCADE_FLAG_VELOCITY_LIMITED, 0);
    }

    #[test]
    fn state_change_without_transition_is_rejected() {
        let mut controller = enabled_controller();
        let seed = reference(ControlMode::Torque, InputMode::Passthrough, true);
        controller
            .step(&config(), &seed, feedback(0.0, 0.0, 0.0))
            .unwrap();
        let changed = reference(ControlMode::Velocity, InputMode::VelocityRamp, false);
        let before = controller.current_output();
        assert_eq!(
            controller.step(&config(), &changed, feedback(0.0, 0.0, 0.0)),
            Err(MotionCascadeError::TransitionRequired)
        );
        assert_eq!(controller.current_output(), before);
    }

    #[test]
    fn transition_requires_applied_current_and_active_loop_feedback() {
        let mut controller = enabled_controller();
        let position = reference(ControlMode::Position, InputMode::PositionFilter, true);
        assert_eq!(
            controller.step(&config(), &position, MotionCascadeFeedback::default()),
            Err(MotionCascadeError::MissingPositionFeedback)
        );
        let mut no_current = feedback(0.0, 0.0, 0.0);
        no_current.current_q_valid = false;
        assert_eq!(
            controller.step(&config(), &position, no_current),
            Err(MotionCascadeError::MissingCurrentFeedback)
        );
    }

    #[test]
    fn saturation_back_calculates_integrator_and_releases_without_windup() {
        let mut controller = enabled_controller();
        let seed = reference(ControlMode::Velocity, InputMode::Passthrough, true);
        controller
            .step(&config(), &seed, feedback(0.0, 0.0, 0.0))
            .unwrap();
        let mut positive = reference(ControlMode::Velocity, InputMode::Passthrough, false);
        positive.velocity_ref_rad_s = 100.0;
        for _ in 0..10_000 {
            let output = controller
                .step(&config(), &positive, feedback(0.0, 0.0, 0.0))
                .unwrap();
            assert!((output.torque_reference_nm - 0.4).abs() < 1.0e-6);
        }
        assert!(controller.velocity_integrator_nm().abs() <= 0.4 + 1.0e-6);

        let mut negative = positive;
        negative.velocity_ref_rad_s = -100.0;
        let output = controller
            .step(&config(), &negative, feedback(0.0, 0.0, 0.0))
            .unwrap();
        assert!((output.torque_reference_nm + 0.4).abs() < 1.0e-6);
    }

    #[test]
    fn feedforward_is_inside_pi_headroom_and_cannot_bypass_final_limit() {
        let mut controller = enabled_controller();
        let seed = reference(ControlMode::Velocity, InputMode::Passthrough, true);
        controller
            .step(&config(), &seed, feedback(0.0, 0.0, 0.0))
            .unwrap();
        let mut command = reference(ControlMode::Velocity, InputMode::Passthrough, false);
        command.velocity_ref_rad_s = 100.0;
        command.torque_feedforward_nm = 0.35;
        let output = controller
            .step(&config(), &command, feedback(0.0, 0.0, 0.0))
            .unwrap();
        assert!((output.torque_reference_nm - 0.4).abs() < 1.0e-6);
        assert!(controller.velocity_integrator_nm() <= 0.05 + 1.0e-6);
        assert_ne!(output.flags & MOTION_CASCADE_FLAG_TORQUE_LIMITED, 0);
    }

    #[test]
    fn nonfinite_feedback_is_rejected_without_mutating_pi_state() {
        let mut controller = enabled_controller();
        let seed = reference(ControlMode::Velocity, InputMode::Passthrough, true);
        controller
            .step(&config(), &seed, feedback(0.0, 0.0, 0.0))
            .unwrap();
        let before_output = controller.current_output();
        let before_integrator = controller.velocity_integrator_nm();
        let command = reference(ControlMode::Velocity, InputMode::Passthrough, false);
        assert_eq!(
            controller.step(&config(), &command, feedback(0.0, f32::NAN, 0.0)),
            Err(MotionCascadeError::InvalidFeedback)
        );
        assert_eq!(controller.current_output(), before_output);
        assert_eq!(controller.velocity_integrator_nm(), before_integrator);
    }

    #[test]
    fn planner_transition_connects_to_cascade_preload_without_field_remapping() {
        let planner_config = MotionPlannerConfig {
            control_period_s: 0.001,
            maximum_current_a: 2.0,
            maximum_torque_nm: 1.0,
            maximum_velocity_rad_s: 10.0,
            torque_ramp_rate_nm_s: 2.0,
            velocity_ramp_rate_rad_s2: 5.0,
            position_filter_bandwidth_rad_s: 8.0,
            trajectory_acceleration_rad_s2: 4.0,
            trajectory_deceleration_rad_s2: 5.0,
            soft_limit_min_rad: -2.0,
            soft_limit_max_rad: 3.0,
        };
        let command = ProductCommand {
            command_kind: ProductCommandKind::Setpoint as u32,
            control_mode: ControlMode::Velocity as u32,
            input_mode: InputMode::VelocityRamp as u32,
            feedback_mode: FeedbackMode::IncrementalEncoder as u32,
            velocity_ref_rad_s: 4.0,
            ..ProductCommand::default()
        };
        let planner_feedback = MotionFeedback {
            position_valid: true,
            velocity_valid: true,
            mechanical_position_rad: 0.3,
            mechanical_velocity_rad_s: 2.0,
        };
        let cascade_feedback = feedback(0.3, 2.0, 0.5);
        let mut planner = MotionReferencePlanner::default();
        planner.set_enabled(true);
        let mut controller = enabled_controller();

        let transition = planner
            .step(&planner_config, &command, planner_feedback)
            .unwrap();
        let first = controller
            .step(&config(), &transition, cascade_feedback)
            .unwrap();
        assert_eq!(transition.flags, MOTION_REFERENCE_FLAG_TRANSITION);
        assert_eq!(first.reference_flags, transition.flags);
        assert!((first.current_reference.iq_ref_a - 0.5).abs() < 1.0e-6);

        let planned = planner
            .step(&planner_config, &command, planner_feedback)
            .unwrap();
        let second = controller
            .step(&config(), &planned, cascade_feedback)
            .unwrap();
        assert_eq!(planned.flags & MOTION_REFERENCE_FLAG_TRANSITION, 0);
        assert!(second.current_reference.iq_ref_a > first.current_reference.iq_ref_a);
        assert_ne!(second.flags & MOTION_CASCADE_FLAG_VELOCITY_LOOP_ACTIVE, 0);
    }
}
