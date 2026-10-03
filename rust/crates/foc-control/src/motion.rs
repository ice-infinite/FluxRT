//! Product-level motion-reference planning for Torque/Velocity/Position modes.
//!
//! P4.2A deliberately stops at the reference boundary: it validates one
//! [`ProductCommand`], applies input-mode shaping and product limits, and emits a
//! deterministic SI-unit [`MotionReference`].  It does not run the position,
//! velocity or current loops and it is not connected to the target ISR yet.
//! Keeping this boundary explicit lets the later cascade consume the same
//! planner in target firmware and PC simulation without letting a protocol,
//! RTOS or board type leak into the control core.

use libm::sqrtf;

use crate::{
    ContractError, ControlMode, InputMode, ProductCommand, ProductCommandKind,
    PRODUCT_COMMAND_FLAG_CURRENT_LIMIT, PRODUCT_COMMAND_FLAG_TORQUE_LIMIT,
    PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT,
};

/// Planner output was seeded for a control/input-mode transition.
pub const MOTION_REFERENCE_FLAG_TRANSITION: u32 = 1 << 0;
/// The requested torque was reduced by the effective torque limit.
pub const MOTION_REFERENCE_FLAG_TORQUE_LIMITED: u32 = 1 << 1;
/// The requested velocity was reduced by the effective velocity limit.
pub const MOTION_REFERENCE_FLAG_VELOCITY_LIMITED: u32 = 1 << 2;
/// The requested position was clamped to the lower software limit.
pub const MOTION_REFERENCE_FLAG_SOFT_LIMIT_MIN: u32 = 1 << 3;
/// The requested position was clamped to the upper software limit.
pub const MOTION_REFERENCE_FLAG_SOFT_LIMIT_MAX: u32 = 1 << 4;
/// All currently defined planner-output flags.
pub const MOTION_REFERENCE_FLAG_KNOWN_MASK: u32 = MOTION_REFERENCE_FLAG_TRANSITION
    | MOTION_REFERENCE_FLAG_TORQUE_LIMITED
    | MOTION_REFERENCE_FLAG_VELOCITY_LIMITED
    | MOTION_REFERENCE_FLAG_SOFT_LIMIT_MIN
    | MOTION_REFERENCE_FLAG_SOFT_LIMIT_MAX;

/// Board-independent planner parameters in SI units.
///
/// These are inputs to the pure core, not hard-coded motor tuning. P4.2C maps
/// them from the persisted [`crate::ConfigBundle`] in one shared location. All
/// limits are absolute magnitudes and must be positive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionPlannerConfig {
    /// Planner sample period `[s]`.
    pub control_period_s: f32,
    /// Product hard current limit `[A]`; a command may only tighten it.
    pub maximum_current_a: f32,
    /// Product hard torque limit `[N*m]`; a command may only tighten it.
    pub maximum_torque_nm: f32,
    /// Product hard mechanical velocity limit `[rad/s]`; a command may tighten it.
    pub maximum_velocity_rad_s: f32,
    /// Maximum TorqueRamp slope `[N*m/s]`.
    pub torque_ramp_rate_nm_s: f32,
    /// Maximum VelocityRamp slope `[rad/s^2]`.
    pub velocity_ramp_rate_rad_s2: f32,
    /// First-order PositionFilter bandwidth `[rad/s]`.
    pub position_filter_bandwidth_rad_s: f32,
    /// TrapezoidalTrajectory acceleration magnitude `[rad/s^2]`.
    pub trajectory_acceleration_rad_s2: f32,
    /// TrapezoidalTrajectory braking magnitude `[rad/s^2]`.
    pub trajectory_deceleration_rad_s2: f32,
    /// Lower multi-turn software position bound `[rad]`.
    pub soft_limit_min_rad: f32,
    /// Upper multi-turn software position bound `[rad]`.
    pub soft_limit_max_rad: f32,
}

impl MotionPlannerConfig {
    pub fn validate(&self) -> Result<(), MotionPlannerError> {
        let positive = [
            self.control_period_s,
            self.maximum_current_a,
            self.maximum_torque_nm,
            self.maximum_velocity_rad_s,
            self.torque_ramp_rate_nm_s,
            self.velocity_ramp_rate_rad_s2,
            self.position_filter_bandwidth_rad_s,
            self.trajectory_acceleration_rad_s2,
            self.trajectory_deceleration_rad_s2,
        ];
        if !positive
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
            || self.control_period_s > 1.0
            || !self.soft_limit_min_rad.is_finite()
            || !self.soft_limit_max_rad.is_finite()
            || self.soft_limit_min_rad >= self.soft_limit_max_rad
        {
            return Err(MotionPlannerError::InvalidConfig);
        }
        Ok(())
    }
}

/// Mechanical feedback needed to seed reference generators without a jump.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotionFeedback {
    pub position_valid: bool,
    pub velocity_valid: bool,
    pub mechanical_position_rad: f32,
    pub mechanical_velocity_rad_s: f32,
}

/// Planner output consumed by the future position/velocity/torque cascade.
///
/// Feed-forward stays separate from the planned reference so telemetry can show
/// its origin.  The cascade must clamp the combined value again after its PI/P
/// contribution; the planner cannot enforce a downstream controller output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionReference {
    pub control_mode: ControlMode,
    pub input_mode: InputMode,
    pub position_ref_rad: f32,
    pub velocity_ref_rad_s: f32,
    pub torque_ref_nm: f32,
    pub velocity_feedforward_rad_s: f32,
    pub torque_feedforward_nm: f32,
    pub current_limit_a: f32,
    pub torque_limit_nm: f32,
    pub velocity_limit_rad_s: f32,
    pub flags: u32,
}

impl Default for MotionReference {
    fn default() -> Self {
        Self {
            control_mode: ControlMode::Inactive,
            input_mode: InputMode::Inactive,
            position_ref_rad: 0.0,
            velocity_ref_rad_s: 0.0,
            torque_ref_nm: 0.0,
            velocity_feedforward_rad_s: 0.0,
            torque_feedforward_nm: 0.0,
            current_limit_a: 0.0,
            torque_limit_nm: 0.0,
            velocity_limit_rad_s: 0.0,
            flags: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionPlannerError {
    Disabled,
    InvalidConfig,
    InvalidCommand(ContractError),
    NotSetpoint,
    UnsupportedControlMode,
    MissingPositionFeedback,
    MissingVelocityFeedback,
    InvalidFeedback,
}

/// Allocation-free, board-independent input-mode state machine.
///
/// `Default` is disabled.  Enabling clears all prior state; the first valid
/// command seeds velocity/position from measured feedback and emits a
/// `TRANSITION` sample rather than immediately jumping to a remote target.  The
/// later cascade uses that sample to preload its state before it accepts normal
/// references on following ticks.
#[derive(Clone, Copy, Debug)]
pub struct MotionReferencePlanner {
    enabled: bool,
    initialized: bool,
    control_mode: ControlMode,
    input_mode: InputMode,
    reference: MotionReference,
    trajectory_velocity_rad_s: f32,
}

impl Default for MotionReferencePlanner {
    fn default() -> Self {
        Self {
            enabled: false,
            initialized: false,
            control_mode: ControlMode::Inactive,
            input_mode: InputMode::Inactive,
            reference: MotionReference::default(),
            trajectory_velocity_rad_s: 0.0,
        }
    }
}

impl MotionReferencePlanner {
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Explicit feature gate.  Both enable and disable discard prior references.
    pub fn set_enabled(&mut self, enabled: bool) {
        *self = Self {
            enabled,
            ..Self::default()
        };
    }

    pub fn current_reference(&self) -> MotionReference {
        self.reference
    }

    /// Plan one management/control tick transactionally.
    ///
    /// On any error the complete planner state remains unchanged.  This matters
    /// for malformed or stale commands: a rejected packet must not partially
    /// advance a ramp or trajectory.
    pub fn step(
        &mut self,
        config: &MotionPlannerConfig,
        command: &ProductCommand,
        feedback: MotionFeedback,
    ) -> Result<MotionReference, MotionPlannerError> {
        if !self.enabled {
            return Err(MotionPlannerError::Disabled);
        }
        config.validate()?;
        command
            .validate()
            .map_err(MotionPlannerError::InvalidCommand)?;
        if ProductCommandKind::try_from(command.command_kind)
            .map_err(MotionPlannerError::InvalidCommand)?
            != ProductCommandKind::Setpoint
        {
            return Err(MotionPlannerError::NotSetpoint);
        }

        let control = ControlMode::try_from(command.control_mode)
            .map_err(MotionPlannerError::InvalidCommand)?;
        let input =
            InputMode::try_from(command.input_mode).map_err(MotionPlannerError::InvalidCommand)?;
        if !matches!(
            control,
            ControlMode::Torque | ControlMode::Velocity | ControlMode::Position
        ) {
            return Err(MotionPlannerError::UnsupportedControlMode);
        }
        validate_feedback_for_mode(control, feedback)?;

        let mut next = *self;
        let transition =
            !next.initialized || next.control_mode != control || next.input_mode != input;
        if transition {
            next.seed_transition(config, command, feedback, control, input);
        } else {
            next.advance(config, command, control, input);
        }
        let output = next.reference;
        *self = next;
        Ok(output)
    }

    /// Realtime inner path for a command/configuration already validated by the
    /// owning ABI transaction.
    ///
    /// This deliberately keeps the live feedback checks and enum conversion,
    /// but does not repeat `config.validate()` or `command.validate()`.  The
    /// caller must mutate an outer shadow and publish it only after the complete
    /// control transaction succeeds; consequently this method may update `self`
    /// in place without another planner-sized shadow copy.
    #[inline]
    pub fn step_prevalidated_realtime(
        &mut self,
        config: &MotionPlannerConfig,
        command: &ProductCommand,
        feedback: MotionFeedback,
    ) -> Result<MotionReference, MotionPlannerError> {
        if !self.enabled {
            return Err(MotionPlannerError::Disabled);
        }
        let control = ControlMode::try_from(command.control_mode)
            .map_err(MotionPlannerError::InvalidCommand)?;
        let input =
            InputMode::try_from(command.input_mode).map_err(MotionPlannerError::InvalidCommand)?;
        if !matches!(
            control,
            ControlMode::Torque | ControlMode::Velocity | ControlMode::Position
        ) {
            return Err(MotionPlannerError::UnsupportedControlMode);
        }
        validate_feedback_for_mode(control, feedback)?;

        let transition =
            !self.initialized || self.control_mode != control || self.input_mode != input;
        if transition {
            self.seed_transition(config, command, feedback, control, input);
        } else {
            self.advance(config, command, control, input);
        }
        Ok(self.reference)
    }

    fn seed_transition(
        &mut self,
        config: &MotionPlannerConfig,
        command: &ProductCommand,
        feedback: MotionFeedback,
        control: ControlMode,
        input: InputMode,
    ) {
        let (current_limit, torque_limit, velocity_limit) = effective_limits(config, command);
        let torque_seed = clamp_symmetric(
            if self.initialized {
                self.reference.torque_ref_nm
            } else {
                0.0
            },
            torque_limit,
        );
        let velocity_seed = clamp_symmetric(
            if feedback.velocity_valid {
                feedback.mechanical_velocity_rad_s
            } else {
                0.0
            },
            velocity_limit,
        );
        let position_seed = if feedback.position_valid {
            feedback.mechanical_position_rad
        } else {
            0.0
        };

        self.reference = MotionReference {
            control_mode: control,
            input_mode: input,
            position_ref_rad: position_seed,
            velocity_ref_rad_s: velocity_seed,
            torque_ref_nm: torque_seed,
            velocity_feedforward_rad_s: 0.0,
            torque_feedforward_nm: 0.0,
            current_limit_a: current_limit,
            torque_limit_nm: torque_limit,
            velocity_limit_rad_s: velocity_limit,
            flags: MOTION_REFERENCE_FLAG_TRANSITION,
        };
        self.trajectory_velocity_rad_s = velocity_seed;
        self.control_mode = control;
        self.input_mode = input;
        self.initialized = true;
    }

    fn advance(
        &mut self,
        config: &MotionPlannerConfig,
        command: &ProductCommand,
        control: ControlMode,
        input: InputMode,
    ) {
        let (current_limit, torque_limit, velocity_limit) = effective_limits(config, command);
        let mut flags = 0;
        let mut output = MotionReference {
            control_mode: control,
            input_mode: input,
            current_limit_a: current_limit,
            torque_limit_nm: torque_limit,
            velocity_limit_rad_s: velocity_limit,
            ..MotionReference::default()
        };

        match control {
            ControlMode::Torque => {
                let target = clamp_with_flag(
                    command.torque_ref_nm,
                    torque_limit,
                    &mut flags,
                    MOTION_REFERENCE_FLAG_TORQUE_LIMITED,
                );
                output.torque_ref_nm = match input {
                    InputMode::TorqueRamp => move_towards(
                        self.reference.torque_ref_nm,
                        target,
                        config.torque_ramp_rate_nm_s * config.control_period_s,
                    ),
                    InputMode::Passthrough | InputMode::ExternalSynchronized => target,
                    _ => unreachable!("ProductCommand validation rejects this pair"),
                };
            }
            ControlMode::Velocity => {
                let target = clamp_with_flag(
                    command.velocity_ref_rad_s,
                    velocity_limit,
                    &mut flags,
                    MOTION_REFERENCE_FLAG_VELOCITY_LIMITED,
                );
                output.velocity_ref_rad_s = match input {
                    InputMode::VelocityRamp => move_towards(
                        self.reference.velocity_ref_rad_s,
                        target,
                        config.velocity_ramp_rate_rad_s2 * config.control_period_s,
                    ),
                    InputMode::Passthrough | InputMode::ExternalSynchronized => target,
                    _ => unreachable!("ProductCommand validation rejects this pair"),
                };
                output.torque_feedforward_nm = clamp_with_flag(
                    command.torque_feedforward_nm,
                    torque_limit,
                    &mut flags,
                    MOTION_REFERENCE_FLAG_TORQUE_LIMITED,
                );
            }
            ControlMode::Position => {
                let target = clamp_position(config, command.position_ref_rad, &mut flags);
                match input {
                    InputMode::Passthrough | InputMode::ExternalSynchronized => {
                        output.position_ref_rad = target;
                        self.trajectory_velocity_rad_s = 0.0;
                    }
                    InputMode::PositionFilter => {
                        let bandwidth_step =
                            config.position_filter_bandwidth_rad_s * config.control_period_s;
                        let alpha = bandwidth_step / (1.0 + bandwidth_step);
                        let desired_delta = (target - self.reference.position_ref_rad) * alpha;
                        let maximum_delta = velocity_limit * config.control_period_s;
                        let delta = clamp_symmetric(desired_delta, maximum_delta);
                        if delta != desired_delta {
                            flags |= MOTION_REFERENCE_FLAG_VELOCITY_LIMITED;
                        }
                        output.position_ref_rad = self.reference.position_ref_rad + delta;
                        output.velocity_ref_rad_s = delta / config.control_period_s;
                        self.trajectory_velocity_rad_s = output.velocity_ref_rad_s;
                    }
                    InputMode::TrapezoidalTrajectory => {
                        let (position, velocity) = step_trapezoidal(
                            self.reference.position_ref_rad,
                            self.trajectory_velocity_rad_s,
                            target,
                            velocity_limit,
                            config.trajectory_acceleration_rad_s2,
                            config.trajectory_deceleration_rad_s2,
                            config.control_period_s,
                        );
                        output.position_ref_rad = position;
                        output.velocity_ref_rad_s = velocity;
                        self.trajectory_velocity_rad_s = velocity;
                    }
                    _ => unreachable!("ProductCommand validation rejects this pair"),
                }

                output.velocity_feedforward_rad_s = clamp_feedforward(
                    output.velocity_ref_rad_s,
                    command.velocity_feedforward_rad_s,
                    velocity_limit,
                    &mut flags,
                    MOTION_REFERENCE_FLAG_VELOCITY_LIMITED,
                );
                output.torque_feedforward_nm = clamp_with_flag(
                    command.torque_feedforward_nm,
                    torque_limit,
                    &mut flags,
                    MOTION_REFERENCE_FLAG_TORQUE_LIMITED,
                );
            }
            _ => unreachable!("unsupported modes are rejected before mutation"),
        }
        output.flags = flags;
        self.reference = output;
    }
}

fn validate_feedback_for_mode(
    control: ControlMode,
    feedback: MotionFeedback,
) -> Result<(), MotionPlannerError> {
    if feedback.position_valid && !feedback.mechanical_position_rad.is_finite()
        || feedback.velocity_valid && !feedback.mechanical_velocity_rad_s.is_finite()
    {
        return Err(MotionPlannerError::InvalidFeedback);
    }
    match control {
        ControlMode::Velocity if !feedback.velocity_valid => {
            Err(MotionPlannerError::MissingVelocityFeedback)
        }
        ControlMode::Position if !feedback.position_valid => {
            Err(MotionPlannerError::MissingPositionFeedback)
        }
        ControlMode::Position if !feedback.velocity_valid => {
            Err(MotionPlannerError::MissingVelocityFeedback)
        }
        _ => Ok(()),
    }
}

fn effective_limits(config: &MotionPlannerConfig, command: &ProductCommand) -> (f32, f32, f32) {
    let current = if command.flags & PRODUCT_COMMAND_FLAG_CURRENT_LIMIT != 0 {
        config.maximum_current_a.min(command.current_limit_a)
    } else {
        config.maximum_current_a
    };
    let torque = if command.flags & PRODUCT_COMMAND_FLAG_TORQUE_LIMIT != 0 {
        config.maximum_torque_nm.min(command.torque_limit_nm)
    } else {
        config.maximum_torque_nm
    };
    let velocity = if command.flags & PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT != 0 {
        config
            .maximum_velocity_rad_s
            .min(command.velocity_limit_rad_s)
    } else {
        config.maximum_velocity_rad_s
    };
    (current, torque, velocity)
}

fn clamp_position(config: &MotionPlannerConfig, value: f32, flags: &mut u32) -> f32 {
    if value < config.soft_limit_min_rad {
        *flags |= MOTION_REFERENCE_FLAG_SOFT_LIMIT_MIN;
        config.soft_limit_min_rad
    } else if value > config.soft_limit_max_rad {
        *flags |= MOTION_REFERENCE_FLAG_SOFT_LIMIT_MAX;
        config.soft_limit_max_rad
    } else {
        value
    }
}

/// Panic-free symmetric clamp for the realtime path. Configuration validation
/// proves `limit` finite and positive before these helpers run; a non-finite
/// computed value is deliberately preserved so the caller's finite-output gate
/// rejects the complete transaction instead of formatting a panic message.
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

fn clamp_with_flag(value: f32, limit: f32, flags: &mut u32, flag: u32) -> f32 {
    let clamped = clamp_symmetric(value, limit);
    if clamped != value {
        *flags |= flag;
    }
    clamped
}

fn clamp_feedforward(base: f32, feedforward: f32, limit: f32, flags: &mut u32, flag: u32) -> f32 {
    let combined = clamp_symmetric(base + feedforward, limit);
    if combined != base + feedforward {
        *flags |= flag;
    }
    combined - base
}

fn move_towards(current: f32, target: f32, maximum_delta: f32) -> f32 {
    let delta = target - current;
    if delta > maximum_delta {
        current + maximum_delta
    } else if delta < -maximum_delta {
        current - maximum_delta
    } else {
        target
    }
}

fn step_trapezoidal(
    position: f32,
    velocity: f32,
    target: f32,
    maximum_velocity: f32,
    acceleration: f32,
    deceleration: f32,
    dt: f32,
) -> (f32, f32) {
    let error = target - position;
    let stopping_velocity = sqrtf(2.0 * deceleration * error.abs());
    let direction = if error > 0.0 {
        1.0
    } else if error < 0.0 {
        -1.0
    } else {
        0.0
    };
    let desired_velocity = direction * maximum_velocity.min(stopping_velocity);
    let rate = if velocity * desired_velocity < 0.0 || desired_velocity.abs() < velocity.abs() {
        deceleration
    } else {
        acceleration
    };
    let next_velocity = clamp_symmetric(
        move_towards(velocity, desired_velocity, rate * dt),
        maximum_velocity,
    );
    let next_position = position + next_velocity * dt;
    if error == 0.0 || error * (target - next_position) <= 0.0 {
        (target, 0.0)
    } else {
        (next_position, next_velocity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        FeedbackMode, PRODUCT_COMMAND_FLAG_CURRENT_LIMIT, PRODUCT_COMMAND_FLAG_TORQUE_LIMIT,
        PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT,
    };

    fn config() -> MotionPlannerConfig {
        MotionPlannerConfig {
            control_period_s: 0.01,
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
        }
    }

    fn feedback(position: f32, velocity: f32) -> MotionFeedback {
        MotionFeedback {
            position_valid: true,
            velocity_valid: true,
            mechanical_position_rad: position,
            mechanical_velocity_rad_s: velocity,
        }
    }

    fn setpoint(control: ControlMode, input: InputMode) -> ProductCommand {
        ProductCommand {
            command_kind: ProductCommandKind::Setpoint as u32,
            control_mode: control as u32,
            input_mode: input as u32,
            feedback_mode: FeedbackMode::IncrementalEncoder as u32,
            ..ProductCommand::default()
        }
    }

    fn enabled_planner() -> MotionReferencePlanner {
        let mut planner = MotionReferencePlanner::default();
        planner.set_enabled(true);
        planner
    }

    #[test]
    fn default_is_disabled_and_enable_clears_old_state() {
        let mut planner = MotionReferencePlanner::default();
        let command = setpoint(ControlMode::Torque, InputMode::TorqueRamp);
        assert_eq!(
            planner.step(&config(), &command, MotionFeedback::default()),
            Err(MotionPlannerError::Disabled)
        );
        planner.set_enabled(true);
        assert!(planner.is_enabled());
        planner.set_enabled(false);
        assert!(!planner.is_enabled());
        assert_eq!(planner.current_reference(), MotionReference::default());
    }

    #[test]
    fn invalid_config_and_command_do_not_mutate_state() {
        let mut planner = enabled_planner();
        let mut command = setpoint(ControlMode::Torque, InputMode::TorqueRamp);
        command.torque_ref_nm = 0.5;
        let before = planner.current_reference();
        let mut bad_config = config();
        bad_config.control_period_s = f32::NAN;
        assert_eq!(
            planner.step(&bad_config, &command, MotionFeedback::default()),
            Err(MotionPlannerError::InvalidConfig)
        );
        assert_eq!(planner.current_reference(), before);

        command.torque_ref_nm = f32::INFINITY;
        assert_eq!(
            planner.step(&config(), &command, MotionFeedback::default()),
            Err(MotionPlannerError::InvalidCommand(ContractError::NonFinite))
        );
        assert_eq!(planner.current_reference(), before);
    }

    #[test]
    fn first_sample_is_a_transition_seed() {
        let mut planner = enabled_planner();
        let mut command = setpoint(ControlMode::Velocity, InputMode::VelocityRamp);
        command.velocity_ref_rad_s = 8.0;
        let output = planner
            .step(&config(), &command, feedback(1.2, 2.5))
            .unwrap();
        assert_eq!(output.velocity_ref_rad_s, 2.5);
        assert_eq!(output.flags, MOTION_REFERENCE_FLAG_TRANSITION);
        assert_eq!(output.current_limit_a, 2.0);
        assert_eq!(output.torque_limit_nm, 1.0);
        assert_eq!(output.velocity_limit_rad_s, 10.0);
    }

    #[test]
    fn torque_ramp_is_bounded_and_command_can_only_tighten_limits() {
        let mut planner = enabled_planner();
        let mut command = setpoint(ControlMode::Torque, InputMode::TorqueRamp);
        command.flags = PRODUCT_COMMAND_FLAG_CURRENT_LIMIT | PRODUCT_COMMAND_FLAG_TORQUE_LIMIT;
        command.current_limit_a = 1.5;
        command.torque_limit_nm = 0.3;
        command.torque_ref_nm = 0.8;
        planner
            .step(&config(), &command, MotionFeedback::default())
            .unwrap();
        let output = planner
            .step(&config(), &command, MotionFeedback::default())
            .unwrap();
        assert!((output.torque_ref_nm - 0.02).abs() < 1.0e-6);
        assert_eq!(output.current_limit_a, 1.5);
        assert_eq!(output.torque_limit_nm, 0.3);
        assert_ne!(output.flags & MOTION_REFERENCE_FLAG_TORQUE_LIMITED, 0);
    }

    #[test]
    fn velocity_ramp_starts_from_measured_speed_and_obeys_limit() {
        let mut planner = enabled_planner();
        let mut command = setpoint(ControlMode::Velocity, InputMode::VelocityRamp);
        command.flags = PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT;
        command.velocity_limit_rad_s = 3.0;
        command.velocity_ref_rad_s = 9.0;
        planner
            .step(&config(), &command, feedback(0.0, 1.0))
            .unwrap();
        let output = planner
            .step(&config(), &command, feedback(0.0, 1.0))
            .unwrap();
        assert!((output.velocity_ref_rad_s - 1.05).abs() < 1.0e-6);
        assert_ne!(output.flags & MOTION_REFERENCE_FLAG_VELOCITY_LIMITED, 0);
    }

    #[test]
    fn position_filter_respects_soft_and_velocity_limits() {
        let mut planner = enabled_planner();
        let mut command = setpoint(ControlMode::Position, InputMode::PositionFilter);
        command.flags = PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT;
        command.position_ref_rad = 8.0;
        command.velocity_limit_rad_s = 0.5;
        planner
            .step(&config(), &command, feedback(1.0, 0.0))
            .unwrap();
        let output = planner
            .step(&config(), &command, feedback(1.0, 0.0))
            .unwrap();
        assert!((output.position_ref_rad - 1.005).abs() < 1.0e-6);
        assert!((output.velocity_ref_rad_s - 0.5).abs() < 1.0e-6);
        assert_ne!(output.flags & MOTION_REFERENCE_FLAG_SOFT_LIMIT_MAX, 0);
        assert_ne!(output.flags & MOTION_REFERENCE_FLAG_VELOCITY_LIMITED, 0);
    }

    #[test]
    fn position_feedforward_is_trimmed_against_effective_velocity_limit() {
        let mut planner = enabled_planner();
        let mut command = setpoint(ControlMode::Position, InputMode::Passthrough);
        command.flags = PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT;
        command.position_ref_rad = 1.0;
        command.velocity_feedforward_rad_s = 4.0;
        command.velocity_limit_rad_s = 1.5;
        planner
            .step(&config(), &command, feedback(0.0, 0.0))
            .unwrap();
        let output = planner
            .step(&config(), &command, feedback(0.0, 0.0))
            .unwrap();
        assert_eq!(output.position_ref_rad, 1.0);
        assert_eq!(output.velocity_feedforward_rad_s, 1.5);
        assert_ne!(output.flags & MOTION_REFERENCE_FLAG_VELOCITY_LIMITED, 0);
    }

    #[test]
    fn trapezoidal_trajectory_stops_without_overshoot_in_both_directions() {
        for target in [0.2, -0.2] {
            let mut planner = enabled_planner();
            let mut command = setpoint(ControlMode::Position, InputMode::TrapezoidalTrajectory);
            command.position_ref_rad = target;
            planner
                .step(&config(), &command, feedback(0.0, 0.0))
                .unwrap();
            let mut output = MotionReference::default();
            for _ in 0..1_000 {
                output = planner
                    .step(&config(), &command, feedback(0.0, 0.0))
                    .unwrap();
                assert!(output.velocity_ref_rad_s.abs() <= 10.0);
            }
            assert!((output.position_ref_rad - target).abs() < 1.0e-6);
            assert_eq!(output.velocity_ref_rad_s, 0.0);
        }
    }

    #[test]
    fn switching_mode_reseeds_from_feedback_and_marks_transition() {
        let mut planner = enabled_planner();
        let mut torque = setpoint(ControlMode::Torque, InputMode::Passthrough);
        torque.torque_ref_nm = 0.4;
        planner
            .step(&config(), &torque, MotionFeedback::default())
            .unwrap();
        planner
            .step(&config(), &torque, MotionFeedback::default())
            .unwrap();

        let mut position = setpoint(ControlMode::Position, InputMode::PositionFilter);
        position.position_ref_rad = 2.0;
        let output = planner
            .step(&config(), &position, feedback(0.7, -0.2))
            .unwrap();
        assert_eq!(output.position_ref_rad, 0.7);
        assert_eq!(output.velocity_ref_rad_s, -0.2);
        assert_eq!(output.torque_ref_nm, 0.4);
        assert_eq!(output.flags, MOTION_REFERENCE_FLAG_TRANSITION);
    }

    #[test]
    fn missing_or_nonfinite_feedback_is_rejected_transactionally() {
        let mut planner = enabled_planner();
        let command = setpoint(ControlMode::Position, InputMode::PositionFilter);
        let before = planner.current_reference();
        assert_eq!(
            planner.step(&config(), &command, MotionFeedback::default()),
            Err(MotionPlannerError::MissingPositionFeedback)
        );
        assert_eq!(planner.current_reference(), before);

        assert_eq!(
            planner.step(
                &config(),
                &command,
                MotionFeedback {
                    position_valid: true,
                    velocity_valid: true,
                    mechanical_position_rad: f32::NAN,
                    mechanical_velocity_rad_s: 0.0,
                },
            ),
            Err(MotionPlannerError::InvalidFeedback)
        );
        assert_eq!(planner.current_reference(), before);
    }

    #[test]
    fn non_motion_setpoints_are_rejected_without_guessing_a_fallback() {
        let mut planner = enabled_planner();
        let mut command = setpoint(ControlMode::Current, InputMode::Passthrough);
        command.current_q_ref_a = 0.2;
        assert_eq!(
            planner.step(&config(), &command, MotionFeedback::default()),
            Err(MotionPlannerError::UnsupportedControlMode)
        );
    }
}
