//! Fixed-capacity product-command mailbox and deterministic source arbiter.
//!
//! This module is deliberately pure Rust and independent from protocols, the
//! RTOS and the target platform. Each configured source owns one persistent
//! setpoint slot plus one one-shot action slot. This is important because the
//! Product Contract V1 expresses the legacy speed start as two commands:
//! `Setpoint` followed by `AxisRequest(ClosedLoopControl)`.

use crate::{AxisRequest, ContractError, ProductCommand, ProductCommandKind};

const WRAPPING_HALF_RANGE: u32 = 0x8000_0000;

/// Per-source command authorization. This is policy data, not caller data.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandPermissions(u32);

impl CommandPermissions {
    pub const RELEASE: Self = Self(1 << ProductCommandKind::Release as u32);
    pub const AXIS_REQUEST: Self = Self(1 << ProductCommandKind::AxisRequest as u32);
    pub const SETPOINT: Self = Self(1 << ProductCommandKind::Setpoint as u32);
    pub const CLEAR_FAULT: Self = Self(1 << ProductCommandKind::ClearFault as u32);
    pub const EMERGENCY_STOP: Self = Self(1 << ProductCommandKind::EmergencyStop as u32);
    pub const ALL: Self = Self(
        Self::RELEASE.0
            | Self::AXIS_REQUEST.0
            | Self::SETPOINT.0
            | Self::CLEAR_FAULT.0
            | Self::EMERGENCY_STOP.0,
    );

    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    pub const fn allows(self, kind: ProductCommandKind) -> bool {
        (self.0 & (1 << kind as u32)) != 0
    }
}

/// Static policy for one command source. Larger priorities win.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandSourcePolicy {
    pub source_id: u32,
    pub priority: u8,
    pub permissions: CommandPermissions,
    /// Maximum time ownership may be held without a newly selected command.
    pub lease_ms: u32,
    /// Maximum accepted command age, measured from `created_at_ms`.
    pub command_timeout_ms: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArbiterConfigError {
    EmptySourceSet,
    SourceCountExceedsCapacity,
    DuplicateSourceId,
    ZeroDuration,
    AmbiguousDuration,
    LeaseExceedsCommandTimeout,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommandSubmitError {
    InvalidContract(ContractError),
    UnknownSource,
    UnauthorizedCommand,
    InvalidTimeWindow,
    FutureTimestamp,
    Expired,
    TimedOut,
    DuplicateSequence,
    OutOfOrderSequence,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommandSubmitOutcome {
    Queued,
    Released,
}

/// Why the arbiter intentionally emitted no command.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommandSafeReason {
    NoCommand,
    FaultActive,
    EmergencyStopLatched,
    /// The active command or its ownership lease timed out. All queued command
    /// payloads are discarded, so recovery requires a fresh sequence number.
    CommandTimeout,
}

/// Commands selected for one management tick.
///
/// `setpoint` persists while the source owns a valid lease. `action` is
/// consumed after one decision. Either may be absent, but never both.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArbitratedCommand {
    pub source_id: u32,
    pub setpoint: Option<ProductCommand>,
    pub action: Option<ProductCommand>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
// Both ProductCommand copies intentionally stay inline: this management-path
// value is fixed-size, allocation-free and must also work in `no_std` target
// builds where boxing is not an acceptable way to silence this lint.
#[allow(clippy::large_enum_variant)]
pub enum CommandDecision {
    Safe(CommandSafeReason),
    Selected(ArbitratedCommand),
}

#[derive(Clone, Copy, Debug)]
struct SourceMailbox {
    policy: CommandSourcePolicy,
    setpoint: Option<ProductCommand>,
    action: Option<ProductCommand>,
    // Emergency stop has a dedicated lane so a later normal action from the
    // same source cannot overwrite it before the management tick polls.
    emergency_stop: Option<ProductCommand>,
    last_sequence: u32,
    has_sequence: bool,
}

impl SourceMailbox {
    fn new(policy: CommandSourcePolicy) -> Self {
        Self {
            policy,
            setpoint: None,
            action: None,
            emergency_stop: None,
            last_sequence: 0,
            has_sequence: false,
        }
    }

    fn has_normal_command(&self) -> bool {
        if self.setpoint.is_some() {
            return true;
        }
        self.action.is_some_and(|command| {
            command.command_kind == ProductCommandKind::AxisRequest as u32
                && command.axis_request != AxisRequest::ClosedLoopControl as u32
        })
    }

    fn clear_control_payloads(&mut self) {
        self.setpoint = None;
        self.action = None;
    }

    fn clear_all_payloads(&mut self) {
        self.clear_control_payloads();
        self.emergency_stop = None;
    }
}

#[derive(Clone, Copy, Debug)]
struct ActiveLease {
    source_index: usize,
    selected_sequence: u32,
    acquired_at_ms: u32,
}

/// Fixed-capacity, allocation-free command arbiter.
///
/// `N` is the complete source capacity; sources cannot be added at runtime.
/// A held lease blocks equal/lower-priority sources. A higher-priority source
/// can preempt immediately. When no lease is held, equal priority is resolved
/// by the smaller `source_id`, independent of arrival order.
pub struct CommandArbiter<const N: usize> {
    sources: [SourceMailbox; N],
    active_source_count: usize,
    active: Option<ActiveLease>,
    emergency_stop_latched: bool,
}

impl<const N: usize> CommandArbiter<N> {
    pub fn new(policies: [CommandSourcePolicy; N]) -> Result<Self, ArbiterConfigError> {
        Self::new_with_active_count(policies, N)
    }

    /// Construct a fixed-capacity arbiter with only the leading policy slots
    /// active. This lets FFI callers use one code-generated capacity instead
    /// of monomorphizing the complete arbitration logic once per source count.
    /// Inactive tail policies are storage only and are never searched, polled
    /// or emitted.
    pub fn new_with_active_count(
        policies: [CommandSourcePolicy; N],
        active_source_count: usize,
    ) -> Result<Self, ArbiterConfigError> {
        if active_source_count == 0 {
            return Err(ArbiterConfigError::EmptySourceSet);
        }
        if active_source_count > N {
            return Err(ArbiterConfigError::SourceCountExceedsCapacity);
        }
        for (index, policy) in policies[..active_source_count].iter().enumerate() {
            if policy.lease_ms == 0 || policy.command_timeout_ms == 0 {
                return Err(ArbiterConfigError::ZeroDuration);
            }
            if policy.lease_ms >= WRAPPING_HALF_RANGE
                || policy.command_timeout_ms >= WRAPPING_HALF_RANGE
            {
                return Err(ArbiterConfigError::AmbiguousDuration);
            }
            if policy.lease_ms > policy.command_timeout_ms {
                return Err(ArbiterConfigError::LeaseExceedsCommandTimeout);
            }
            if policies[..index]
                .iter()
                .any(|earlier| earlier.source_id == policy.source_id)
            {
                return Err(ArbiterConfigError::DuplicateSourceId);
            }
        }

        Ok(Self {
            sources: policies.map(SourceMailbox::new),
            active_source_count,
            active: None,
            emergency_stop_latched: false,
        })
    }

    /// Validate and enqueue one command received at `now_ms`.
    ///
    /// Contract validation always runs first. Sequence and time comparisons use
    /// the standard wrapping half-range rule; exactly half a `u32` range is
    /// rejected as ambiguous. Commands are valid through `valid_until_ms`
    /// with an exclusive `valid_until_ms` bound, and future-created commands
    /// are rejected rather than held.
    pub fn submit(
        &mut self,
        command: ProductCommand,
        now_ms: u32,
    ) -> Result<CommandSubmitOutcome, CommandSubmitError> {
        command
            .validate()
            .map_err(CommandSubmitError::InvalidContract)?;

        let source_index = self.sources[..self.active_source_count]
            .iter()
            .position(|source| source.policy.source_id == command.source_id)
            .ok_or(CommandSubmitError::UnknownSource)?;
        let policy = self.sources[source_index].policy;
        validate_command_time(&command, now_ms, policy.command_timeout_ms)?;

        let kind = ProductCommandKind::try_from(command.command_kind)
            .map_err(CommandSubmitError::InvalidContract)?;
        if !policy.permissions.allows(kind) {
            return Err(CommandSubmitError::UnauthorizedCommand);
        }

        let source = &mut self.sources[source_index];
        if source.has_sequence {
            let distance = command.sequence.wrapping_sub(source.last_sequence);
            if distance == 0 {
                return Err(CommandSubmitError::DuplicateSequence);
            }
            if distance >= WRAPPING_HALF_RANGE {
                return Err(CommandSubmitError::OutOfOrderSequence);
            }
        }
        source.last_sequence = command.sequence;
        source.has_sequence = true;

        match kind {
            ProductCommandKind::Release => {
                // A pending emergency stop deliberately survives Release and
                // wins when the tick is resolved.
                source.clear_control_payloads();
                if self
                    .active
                    .is_some_and(|lease| lease.source_index == source_index)
                {
                    self.active = None;
                }
                Ok(CommandSubmitOutcome::Released)
            }
            ProductCommandKind::Setpoint => {
                source.setpoint = Some(command);
                Ok(CommandSubmitOutcome::Queued)
            }
            ProductCommandKind::AxisRequest => {
                if command.axis_request != AxisRequest::ClosedLoopControl as u32 {
                    // Calibration, test and disable requests must not inherit a
                    // stale running setpoint. ClosedLoopControl is the one V1
                    // request explicitly paired with a persistent setpoint.
                    source.setpoint = None;
                }
                source.action = Some(command);
                Ok(CommandSubmitOutcome::Queued)
            }
            ProductCommandKind::ClearFault => {
                source.action = Some(command);
                Ok(CommandSubmitOutcome::Queued)
            }
            ProductCommandKind::EmergencyStop => {
                source.emergency_stop = Some(command);
                Ok(CommandSubmitOutcome::Queued)
            }
        }
    }

    /// Submit all commands collected for one management tick.
    ///
    /// Commands belonging to the same source are applied in their wrapping
    /// sequence order, so a `Setpoint`/`AxisRequest` pair has the same result
    /// regardless of queue traversal order. Results retain the caller's input
    /// order. A single `submit` intentionally remains strict and rejects late
    /// packets; transports should use this method only for one bounded tick
    /// batch, never to reorder commands across time.
    pub fn submit_batch<const M: usize>(
        &mut self,
        commands: [ProductCommand; M],
        now_ms: u32,
    ) -> [Result<CommandSubmitOutcome, CommandSubmitError>; M] {
        let mut processed = [false; M];
        let mut results = [None; M];

        for _ in 0..M {
            let first = processed
                .iter()
                .position(|done| !done)
                .expect("one unprocessed batch entry must remain");
            let source_id = commands[first].source_id;
            let source_watermark = self.sources[..self.active_source_count]
                .iter()
                .find(|source| source.policy.source_id == source_id)
                .and_then(|source| source.has_sequence.then_some(source.last_sequence));
            let selected =
                select_next_batch_entry(&commands, &processed, source_id, source_watermark, first);

            results[selected] = Some(self.submit(commands[selected], now_ms));
            processed[selected] = true;
        }

        results.map(|result| result.expect("every batch entry must be processed exactly once"))
    }

    /// Resolve one management tick.
    ///
    /// An external active fault wins before every queued command and discards
    /// all old payloads. Emergency stop is the next global priority and is
    /// latched internally. ClearFault is a one-shot recovery action and also
    /// discards old control payloads, requiring an explicit new start sequence.
    pub fn poll(&mut self, now_ms: u32, fault_active: bool) -> CommandDecision {
        if fault_active {
            self.clear_all_payloads();
            self.active = None;
            return CommandDecision::Safe(CommandSafeReason::FaultActive);
        }

        let active_command_timed_out = self.prune_timed_out_payloads(now_ms);

        if let Some(source_index) = self.best_action(ProductCommandKind::EmergencyStop) {
            let emergency = self.sources[source_index]
                .emergency_stop
                .expect("selected emergency action must exist");
            self.clear_all_payloads();
            self.active = None;
            self.emergency_stop_latched = true;
            return CommandDecision::Selected(ArbitratedCommand {
                source_id: emergency.source_id,
                setpoint: None,
                action: Some(emergency),
            });
        }

        if self.emergency_stop_latched {
            if let Some(source_index) = self.best_action(ProductCommandKind::ClearFault) {
                let clear = self.sources[source_index]
                    .action
                    .expect("selected clear-fault action must exist");
                self.clear_all_payloads();
                self.active = None;
                self.emergency_stop_latched = false;
                return CommandDecision::Selected(ArbitratedCommand {
                    source_id: clear.source_id,
                    setpoint: None,
                    action: Some(clear),
                });
            }
            return CommandDecision::Safe(CommandSafeReason::EmergencyStopLatched);
        }

        if active_command_timed_out {
            // No automatic fallback to an older queued source after timeout.
            // This produces an observable safe event and demands fresh input.
            self.clear_all_payloads();
            self.active = None;
            return CommandDecision::Safe(CommandSafeReason::CommandTimeout);
        }

        if let Some(source_index) = self.best_action(ProductCommandKind::ClearFault) {
            let clear = self.sources[source_index]
                .action
                .expect("selected clear-fault action must exist");
            self.clear_all_payloads();
            self.active = None;
            return CommandDecision::Selected(ArbitratedCommand {
                source_id: clear.source_id,
                setpoint: None,
                action: Some(clear),
            });
        }

        if let Some(lease) = self.active {
            let policy = self.sources[lease.source_index].policy;
            if elapsed_reaches(now_ms, lease.acquired_at_ms, policy.lease_ms) {
                self.clear_all_payloads();
                self.active = None;
                return CommandDecision::Safe(CommandSafeReason::CommandTimeout);
            }

            if let Some(best_index) = self.best_normal_source() {
                let active_priority = policy.priority;
                let best_priority = self.sources[best_index].policy.priority;
                if best_priority > active_priority {
                    return self.select_source(best_index, now_ms);
                }
            }

            if self.sources[lease.source_index].last_sequence != lease.selected_sequence {
                return self.select_source(lease.source_index, now_ms);
            }
            return self.emit_source(lease.source_index);
        }

        match self.best_normal_source() {
            Some(source_index) => self.select_source(source_index, now_ms),
            None => CommandDecision::Safe(CommandSafeReason::NoCommand),
        }
    }

    pub fn active_source_id(&self) -> Option<u32> {
        self.active
            .map(|lease| self.sources[lease.source_index].policy.source_id)
    }

    pub fn emergency_stop_latched(&self) -> bool {
        self.emergency_stop_latched
    }

    fn prune_timed_out_payloads(&mut self, now_ms: u32) -> bool {
        let mut active_timed_out = false;
        for (source_index, source) in self.sources[..self.active_source_count]
            .iter_mut()
            .enumerate()
        {
            let timeout_ms = source.policy.command_timeout_ms;
            if source
                .setpoint
                .is_some_and(|command| !command_is_current(&command, now_ms, timeout_ms))
            {
                source.setpoint = None;
                active_timed_out |= self
                    .active
                    .is_some_and(|lease| lease.source_index == source_index);
            }
            if source
                .action
                .is_some_and(|command| !command_is_current(&command, now_ms, timeout_ms))
            {
                source.action = None;
            }
            if source
                .emergency_stop
                .is_some_and(|command| !command_is_current(&command, now_ms, timeout_ms))
            {
                source.emergency_stop = None;
            }
        }
        active_timed_out
    }

    fn best_action(&self, kind: ProductCommandKind) -> Option<usize> {
        self.sources[..self.active_source_count]
            .iter()
            .enumerate()
            .filter(|(_, source)| {
                let action = if kind == ProductCommandKind::EmergencyStop {
                    source.emergency_stop
                } else {
                    source.action
                };
                action.is_some_and(|command| command.command_kind == kind as u32)
            })
            .map(|(index, _)| index)
            .reduce(|best, candidate| self.preferred_source(best, candidate))
    }

    fn best_normal_source(&self) -> Option<usize> {
        self.sources[..self.active_source_count]
            .iter()
            .enumerate()
            .filter(|(_, source)| source.has_normal_command())
            .map(|(index, _)| index)
            .reduce(|best, candidate| self.preferred_source(best, candidate))
    }

    fn preferred_source(&self, left: usize, right: usize) -> usize {
        let left_policy = self.sources[left].policy;
        let right_policy = self.sources[right].policy;
        if right_policy.priority > left_policy.priority
            || (right_policy.priority == left_policy.priority
                && right_policy.source_id < left_policy.source_id)
        {
            right
        } else {
            left
        }
    }

    fn select_source(&mut self, source_index: usize, now_ms: u32) -> CommandDecision {
        self.active = Some(ActiveLease {
            source_index,
            selected_sequence: self.sources[source_index].last_sequence,
            acquired_at_ms: now_ms,
        });
        self.emit_source(source_index)
    }

    fn emit_source(&mut self, source_index: usize) -> CommandDecision {
        let source = &mut self.sources[source_index];
        let action = source.action.take();
        let setpoint = source.setpoint;
        if setpoint.is_none() && action.is_none() {
            self.active = None;
            return CommandDecision::Safe(CommandSafeReason::NoCommand);
        }
        if setpoint.is_none() {
            // A one-shot request alone must not retain ownership after it has
            // been consumed.
            self.active = None;
        }
        CommandDecision::Selected(ArbitratedCommand {
            source_id: source.policy.source_id,
            setpoint,
            action,
        })
    }

    fn clear_all_payloads(&mut self) {
        for source in &mut self.sources[..self.active_source_count] {
            source.clear_all_payloads();
        }
    }
}

fn select_next_batch_entry<const M: usize>(
    commands: &[ProductCommand; M],
    processed: &[bool; M],
    source_id: u32,
    watermark: Option<u32>,
    fallback: usize,
) -> usize {
    if let Some(watermark) = watermark {
        let mut selected = None;
        let mut selected_distance = u32::MAX;
        for (index, command) in commands.iter().enumerate() {
            if processed[index] || command.source_id != source_id {
                continue;
            }
            let distance = command.sequence.wrapping_sub(watermark);
            if distance != 0 && distance < WRAPPING_HALF_RANGE && distance < selected_distance {
                selected = Some(index);
                selected_distance = distance;
            }
        }
        return selected.unwrap_or(fallback);
    }

    // Without a previous watermark, choose the entry that is serially before
    // every other distinct sequence in this source's bounded batch.
    for (candidate_index, candidate) in commands.iter().enumerate() {
        if processed[candidate_index] || candidate.source_id != source_id {
            continue;
        }
        let precedes_all = commands.iter().enumerate().all(|(other_index, other)| {
            if processed[other_index]
                || other.source_id != source_id
                || other.sequence == candidate.sequence
            {
                return true;
            }
            let distance = other.sequence.wrapping_sub(candidate.sequence);
            distance != 0 && distance < WRAPPING_HALF_RANGE
        });
        if precedes_all {
            return candidate_index;
        }
    }
    fallback
}

fn validate_command_time(
    command: &ProductCommand,
    now_ms: u32,
    command_timeout_ms: u32,
) -> Result<(), CommandSubmitError> {
    let window_ms = command.valid_until_ms.wrapping_sub(command.created_at_ms);
    if window_ms == 0 || window_ms >= WRAPPING_HALF_RANGE {
        return Err(CommandSubmitError::InvalidTimeWindow);
    }
    let age_ms = now_ms.wrapping_sub(command.created_at_ms);
    if age_ms >= WRAPPING_HALF_RANGE {
        return Err(CommandSubmitError::FutureTimestamp);
    }
    if age_ms >= window_ms {
        return Err(CommandSubmitError::Expired);
    }
    if age_ms >= command_timeout_ms {
        return Err(CommandSubmitError::TimedOut);
    }
    Ok(())
}

fn command_is_current(command: &ProductCommand, now_ms: u32, timeout_ms: u32) -> bool {
    let age_ms = now_ms.wrapping_sub(command.created_at_ms);
    let window_ms = command.valid_until_ms.wrapping_sub(command.created_at_ms);
    age_ms < WRAPPING_HALF_RANGE && age_ms < window_ms && age_ms < timeout_ms
}

fn elapsed_reaches(now_ms: u32, since_ms: u32, limit_ms: u32) -> bool {
    let elapsed_ms = now_ms.wrapping_sub(since_ms);
    elapsed_ms >= WRAPPING_HALF_RANGE || elapsed_ms >= limit_ms
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ControlMode, FeedbackMode, InputMode};

    fn policy(source_id: u32, priority: u8) -> CommandSourcePolicy {
        CommandSourcePolicy {
            source_id,
            priority,
            permissions: CommandPermissions::ALL,
            lease_ms: 10,
            command_timeout_ms: 20,
        }
    }

    fn canonical(source_id: u32, sequence: u32, now_ms: u32) -> ProductCommand {
        ProductCommand {
            source_id,
            sequence,
            created_at_ms: now_ms,
            valid_until_ms: now_ms.wrapping_add(20),
            ..ProductCommand::default()
        }
    }

    fn velocity_setpoint(
        source_id: u32,
        sequence: u32,
        now_ms: u32,
        velocity_rad_s: f32,
    ) -> ProductCommand {
        ProductCommand {
            command_kind: ProductCommandKind::Setpoint as u32,
            control_mode: ControlMode::Velocity as u32,
            input_mode: InputMode::Passthrough as u32,
            feedback_mode: FeedbackMode::Sensorless as u32,
            velocity_ref_rad_s: velocity_rad_s,
            ..canonical(source_id, sequence, now_ms)
        }
    }

    fn axis_request(
        source_id: u32,
        sequence: u32,
        now_ms: u32,
        request: AxisRequest,
    ) -> ProductCommand {
        ProductCommand {
            command_kind: ProductCommandKind::AxisRequest as u32,
            axis_request: request as u32,
            ..canonical(source_id, sequence, now_ms)
        }
    }

    fn action(
        source_id: u32,
        sequence: u32,
        now_ms: u32,
        kind: ProductCommandKind,
    ) -> ProductCommand {
        ProductCommand {
            command_kind: kind as u32,
            ..canonical(source_id, sequence, now_ms)
        }
    }

    fn selected(decision: CommandDecision) -> ArbitratedCommand {
        match decision {
            CommandDecision::Selected(command) => command,
            CommandDecision::Safe(reason) => panic!("expected selected command, got {reason:?}"),
        }
    }

    #[test]
    fn configuration_rejects_ambiguous_or_duplicate_sources() {
        assert!(matches!(
            CommandArbiter::<0>::new([]),
            Err(ArbiterConfigError::EmptySourceSet)
        ));
        assert!(matches!(
            CommandArbiter::new([policy(1, 1), policy(1, 2)]),
            Err(ArbiterConfigError::DuplicateSourceId)
        ));
        assert!(matches!(
            CommandArbiter::new_with_active_count([policy(1, 1)], 2),
            Err(ArbiterConfigError::SourceCountExceedsCapacity)
        ));
        let mut prefix_only =
            CommandArbiter::new_with_active_count([policy(1, 1), policy(1, 2)], 1).unwrap();
        assert_eq!(
            prefix_only.submit(velocity_setpoint(2, 1, 100, 1.0), 100),
            Err(CommandSubmitError::UnknownSource)
        );
        let mut zero = policy(1, 1);
        zero.lease_ms = 0;
        assert!(matches!(
            CommandArbiter::new([zero]),
            Err(ArbiterConfigError::ZeroDuration)
        ));
        let mut too_long = policy(1, 1);
        too_long.command_timeout_ms = WRAPPING_HALF_RANGE;
        assert!(matches!(
            CommandArbiter::new([too_long]),
            Err(ArbiterConfigError::AmbiguousDuration)
        ));
    }

    #[test]
    fn source_permissions_are_policy_owned_and_fail_closed() {
        let mut setpoint_only = policy(1, 1);
        setpoint_only.permissions = CommandPermissions::SETPOINT.union(CommandPermissions::RELEASE);
        let mut arbiter = CommandArbiter::new([setpoint_only]).unwrap();

        assert_eq!(
            arbiter.submit(velocity_setpoint(1, 1, 100, 1.0), 100),
            Ok(CommandSubmitOutcome::Queued)
        );
        assert_eq!(
            arbiter.submit(action(1, 2, 101, ProductCommandKind::EmergencyStop), 101),
            Err(CommandSubmitError::UnauthorizedCommand)
        );

        // A rejected command must not advance the anti-replay watermark.
        assert_eq!(
            arbiter.submit(action(1, 2, 101, ProductCommandKind::Release), 101),
            Ok(CommandSubmitOutcome::Released)
        );
    }

    #[test]
    fn setpoint_and_closed_loop_request_are_emitted_together_once() {
        let mut arbiter = CommandArbiter::new([policy(7, 1)]).unwrap();
        arbiter
            .submit(velocity_setpoint(7, 10, 100, 54.873_15), 100)
            .unwrap();
        arbiter
            .submit(
                axis_request(7, 11, 101, AxisRequest::ClosedLoopControl),
                101,
            )
            .unwrap();

        let first = selected(arbiter.poll(101, false));
        assert_eq!(first.setpoint.unwrap().sequence, 10);
        assert_eq!(first.action.unwrap().sequence, 11);
        assert_eq!(arbiter.active_source_id(), Some(7));

        let second = selected(arbiter.poll(102, false));
        assert_eq!(second.setpoint.unwrap().sequence, 10);
        assert_eq!(second.action, None);
    }

    #[test]
    fn same_tick_batch_is_independent_of_input_traversal_order() {
        let setpoint = velocity_setpoint(7, 10, 100, 54.873_15);
        let request = axis_request(7, 11, 100, AxisRequest::ClosedLoopControl);

        let mut forward = CommandArbiter::new([policy(7, 1)]).unwrap();
        let forward_results = forward.submit_batch([setpoint, request], 100);
        assert!(forward_results.iter().all(Result::is_ok));
        let forward_decision = selected(forward.poll(100, false));

        let mut reverse = CommandArbiter::new([policy(7, 1)]).unwrap();
        let reverse_results = reverse.submit_batch([request, setpoint], 100);
        assert!(reverse_results.iter().all(Result::is_ok));
        let reverse_decision = selected(reverse.poll(100, false));

        assert_eq!(forward_decision, reverse_decision);
        assert_eq!(forward_decision.setpoint.unwrap().sequence, 10);
        assert_eq!(forward_decision.action.unwrap().sequence, 11);
    }

    #[test]
    fn same_tick_batch_orders_sequences_across_u32_wrap() {
        let setpoint = velocity_setpoint(7, u32::MAX, 100, 54.873_15);
        let request = axis_request(7, 0, 100, AxisRequest::ClosedLoopControl);
        let mut arbiter = CommandArbiter::new([policy(7, 1)]).unwrap();

        let results = arbiter.submit_batch([request, setpoint], 100);
        assert!(results.iter().all(Result::is_ok));
        let decision = selected(arbiter.poll(100, false));
        assert_eq!(decision.setpoint.unwrap().sequence, u32::MAX);
        assert_eq!(decision.action.unwrap().sequence, 0);
    }

    #[test]
    fn priority_preemption_and_equal_priority_are_deterministic() {
        let mut arbiter =
            CommandArbiter::new([policy(30, 1), policy(20, 1), policy(40, 2)]).unwrap();
        arbiter
            .submit(velocity_setpoint(30, 1, 100, 30.0), 100)
            .unwrap();
        arbiter
            .submit(velocity_setpoint(20, 1, 100, 20.0), 100)
            .unwrap();
        assert_eq!(selected(arbiter.poll(100, false)).source_id, 20);

        // Equal-priority source 30 cannot disturb source 20's held lease.
        arbiter
            .submit(velocity_setpoint(30, 2, 101, 31.0), 101)
            .unwrap();
        assert_eq!(selected(arbiter.poll(101, false)).source_id, 20);

        // Larger priority preempts immediately.
        arbiter
            .submit(velocity_setpoint(40, 1, 102, 40.0), 102)
            .unwrap();
        assert_eq!(selected(arbiter.poll(102, false)).source_id, 40);
    }

    #[test]
    fn duplicate_out_of_order_and_wrapped_sequences_are_rejected_or_accepted() {
        let mut arbiter = CommandArbiter::new([policy(1, 1)]).unwrap();
        arbiter
            .submit(velocity_setpoint(1, u32::MAX, 10, 1.0), 10)
            .unwrap();
        assert_eq!(
            arbiter.submit(velocity_setpoint(1, u32::MAX, 11, 1.0), 11),
            Err(CommandSubmitError::DuplicateSequence)
        );
        assert_eq!(
            arbiter.submit(velocity_setpoint(1, u32::MAX - 1, 11, 1.0), 11),
            Err(CommandSubmitError::OutOfOrderSequence)
        );
        assert_eq!(
            arbiter.submit(velocity_setpoint(1, 0, 11, 1.0), 11),
            Ok(CommandSubmitOutcome::Queued)
        );
        assert_eq!(
            arbiter.submit(velocity_setpoint(1, WRAPPING_HALF_RANGE, 12, 1.0), 12),
            Err(CommandSubmitError::OutOfOrderSequence)
        );
    }

    #[test]
    fn closed_loop_request_waits_for_a_same_source_setpoint() {
        let mut arbiter = CommandArbiter::new([policy(1, 1), policy(2, 2)]).unwrap();
        arbiter
            .submit(axis_request(2, 1, 100, AxisRequest::ClosedLoopControl), 100)
            .unwrap();
        assert_eq!(
            arbiter.poll(100, false),
            CommandDecision::Safe(CommandSafeReason::NoCommand)
        );

        // A setpoint owned by another source must not arm source 2.
        arbiter
            .submit(velocity_setpoint(1, 1, 101, 10.0), 101)
            .unwrap();
        assert_eq!(selected(arbiter.poll(101, false)).source_id, 1);

        arbiter
            .submit(velocity_setpoint(2, 2, 102, 20.0), 102)
            .unwrap();
        let armed = selected(arbiter.poll(102, false));
        assert_eq!(armed.source_id, 2);
        assert!(armed.setpoint.is_some());
        assert_eq!(
            armed.action.unwrap().axis_request,
            AxisRequest::ClosedLoopControl as u32
        );
    }

    #[test]
    fn time_window_boundaries_and_u32_wrap_are_explicit() {
        let mut arbiter = CommandArbiter::new([policy(1, 1)]).unwrap();
        let before_deadline = velocity_setpoint(1, 1, 100, 1.0);
        assert_eq!(
            arbiter.submit(before_deadline, 119),
            Ok(CommandSubmitOutcome::Queued)
        );

        let exact_deadline = velocity_setpoint(1, 2, 100, 1.0);
        assert_eq!(
            arbiter.submit(exact_deadline, 120),
            Err(CommandSubmitError::Expired)
        );
        let future = velocity_setpoint(1, 2, 200, 1.0);
        assert_eq!(
            arbiter.submit(future, 199),
            Err(CommandSubmitError::FutureTimestamp)
        );

        let wrapped = velocity_setpoint(1, 2, u32::MAX - 5, 2.0);
        assert_eq!(arbiter.submit(wrapped, 2), Ok(CommandSubmitOutcome::Queued));
        assert_eq!(selected(arbiter.poll(2, false)).source_id, 1);

        let mut timed_out = velocity_setpoint(1, 3, 0, 1.0);
        timed_out.valid_until_ms = 100;
        assert_eq!(
            arbiter.submit(timed_out, 21),
            Err(CommandSubmitError::TimedOut)
        );

        let mut ambiguous = velocity_setpoint(1, 3, 10, 1.0);
        ambiguous.valid_until_ms = ambiguous.created_at_ms.wrapping_add(WRAPPING_HALF_RANGE);
        assert_eq!(
            arbiter.submit(ambiguous, 10),
            Err(CommandSubmitError::InvalidTimeWindow)
        );

        let mut zero_window = velocity_setpoint(1, 3, 10, 1.0);
        zero_window.valid_until_ms = zero_window.created_at_ms;
        assert_eq!(
            arbiter.submit(zero_window, 10),
            Err(CommandSubmitError::InvalidTimeWindow)
        );
    }

    #[test]
    fn active_command_expiry_is_exclusive_and_emits_timeout_event() {
        let mut arbiter = CommandArbiter::new([policy(1, 1)]).unwrap();
        let mut short = velocity_setpoint(1, 1, 100, 1.0);
        short.valid_until_ms = 103;
        arbiter.submit(short, 100).unwrap();

        assert!(matches!(
            arbiter.poll(100, false),
            CommandDecision::Selected(_)
        ));
        assert_eq!(
            arbiter.poll(103, false),
            CommandDecision::Safe(CommandSafeReason::CommandTimeout)
        );
        assert_eq!(arbiter.active_source_id(), None);
    }

    #[test]
    fn lease_holds_release_transfers_and_timeout_requires_fresh_input() {
        let mut arbiter = CommandArbiter::new([policy(2, 1), policy(1, 1)]).unwrap();
        arbiter
            .submit(velocity_setpoint(2, 1, 100, 2.0), 100)
            .unwrap();
        assert_eq!(selected(arbiter.poll(100, false)).source_id, 2);

        // Source 1 would win an unowned tie, but cannot steal a held lease.
        arbiter
            .submit(velocity_setpoint(1, 1, 101, 1.0), 101)
            .unwrap();
        assert_eq!(selected(arbiter.poll(101, false)).source_id, 2);

        let release = action(2, 2, 102, ProductCommandKind::Release);
        assert_eq!(
            arbiter.submit(release, 102),
            Ok(CommandSubmitOutcome::Released)
        );
        assert_eq!(selected(arbiter.poll(102, false)).source_id, 1);

        // An old competing command must not become active automatically after
        // the current owner's lease timeout.
        arbiter
            .submit(velocity_setpoint(2, 3, 103, 4.0), 103)
            .unwrap();

        // The held lease is valid before t=112 and expires at t=112.
        assert!(matches!(
            arbiter.poll(111, false),
            CommandDecision::Selected(_)
        ));
        assert_eq!(
            arbiter.poll(112, false),
            CommandDecision::Safe(CommandSafeReason::CommandTimeout)
        );
        assert_eq!(arbiter.active_source_id(), None);
        assert_eq!(
            arbiter.poll(113, false),
            CommandDecision::Safe(CommandSafeReason::NoCommand)
        );

        arbiter
            .submit(velocity_setpoint(2, 4, 113, 3.0), 113)
            .unwrap();
        assert_eq!(selected(arbiter.poll(113, false)).source_id, 2);
    }

    #[test]
    fn emergency_stop_and_fault_beat_normal_commands_in_the_same_tick() {
        let mut arbiter = CommandArbiter::new([policy(1, 9), policy(2, 1)]).unwrap();
        arbiter
            .submit(velocity_setpoint(1, 1, 100, 10.0), 100)
            .unwrap();
        arbiter
            .submit(action(2, 1, 100, ProductCommandKind::EmergencyStop), 100)
            .unwrap();
        // A later normal action from the same source must not overwrite the
        // dedicated emergency lane before this tick is resolved.
        arbiter
            .submit(action(2, 2, 100, ProductCommandKind::ClearFault), 100)
            .unwrap();

        let stopped = selected(arbiter.poll(100, false));
        assert_eq!(
            stopped.action.unwrap().command_kind,
            ProductCommandKind::EmergencyStop as u32
        );
        assert_eq!(stopped.setpoint, None);
        assert!(arbiter.emergency_stop_latched());
        assert_eq!(
            arbiter.poll(101, false),
            CommandDecision::Safe(CommandSafeReason::EmergencyStopLatched)
        );

        arbiter
            .submit(action(1, 2, 102, ProductCommandKind::ClearFault), 102)
            .unwrap();
        let cleared = selected(arbiter.poll(102, false));
        assert_eq!(
            cleared.action.unwrap().command_kind,
            ProductCommandKind::ClearFault as u32
        );
        assert!(!arbiter.emergency_stop_latched());
        assert_eq!(
            arbiter.poll(103, false),
            CommandDecision::Safe(CommandSafeReason::NoCommand)
        );

        arbiter
            .submit(velocity_setpoint(1, 3, 104, 10.0), 104)
            .unwrap();
        arbiter
            .submit(action(2, 3, 104, ProductCommandKind::ClearFault), 104)
            .unwrap();
        assert_eq!(
            arbiter.poll(104, true),
            CommandDecision::Safe(CommandSafeReason::FaultActive)
        );
        assert_eq!(arbiter.active_source_id(), None);
        assert_eq!(
            arbiter.poll(105, false),
            CommandDecision::Safe(CommandSafeReason::NoCommand)
        );
    }

    #[test]
    fn disable_request_clear_fault_release_and_invalid_contract_clear_correctly() {
        let mut arbiter = CommandArbiter::new([policy(1, 1)]).unwrap();
        arbiter
            .submit(velocity_setpoint(1, 1, 100, 10.0), 100)
            .unwrap();
        arbiter
            .submit(axis_request(1, 2, 101, AxisRequest::Disabled), 101)
            .unwrap();
        let disable = selected(arbiter.poll(101, false));
        assert_eq!(disable.setpoint, None);
        assert_eq!(
            disable.action.unwrap().axis_request,
            AxisRequest::Disabled as u32
        );
        assert_eq!(
            arbiter.poll(102, false),
            CommandDecision::Safe(CommandSafeReason::NoCommand)
        );

        arbiter
            .submit(velocity_setpoint(1, 3, 103, 10.0), 103)
            .unwrap();
        arbiter
            .submit(action(1, 4, 104, ProductCommandKind::ClearFault), 104)
            .unwrap();
        let clear = selected(arbiter.poll(104, false));
        assert_eq!(clear.setpoint, None);
        assert_eq!(
            arbiter.poll(105, false),
            CommandDecision::Safe(CommandSafeReason::NoCommand)
        );

        let mut invalid = velocity_setpoint(1, 5, 106, 10.0);
        invalid.control_mode = u32::MAX;
        assert_eq!(
            arbiter.submit(invalid, 106),
            Err(CommandSubmitError::InvalidContract(
                ContractError::UnknownEnum
            ))
        );
    }

    #[test]
    fn empty_arbiter_output_is_explicitly_safe() {
        let mut arbiter = CommandArbiter::new([policy(1, 1)]).unwrap();
        assert_eq!(
            arbiter.poll(0, false),
            CommandDecision::Safe(CommandSafeReason::NoCommand)
        );
    }
}
