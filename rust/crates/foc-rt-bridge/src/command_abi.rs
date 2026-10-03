//! Fixed-capacity C ABI for the device-side command arbiter.
//!
//! This is a management-plane interface.  Protocol and input adapters submit
//! canonical `ProductCommand` values here; they never call the realtime FOC or
//! power stage directly.  The context is caller-owned, allocation-free and must
//! be serialized by one management task.

use core::mem::{align_of, size_of};
use core::ptr;
use foc_control::{
    ArbiterConfigError, ArbitratedCommand, CommandArbiter, CommandDecision, CommandSafeReason,
    CommandSourceConfig, CommandSubmitError, ContractError, ProductCommand,
};

pub const FOC_COMMAND_ABI_VERSION: u32 = 0x0001_0000;
pub const FOC_COMMAND_CONTEXT_CAPACITY: usize = 4096;
pub const FOC_COMMAND_MAX_SOURCES: usize = 8;

const COMMAND_CONTEXT_MAGIC: u32 = 0x4643_4D44; // "FCMD"

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FocCommandStatus {
    Ok = 0,
    InvalidArgument = 1,
    NotInitialized = 2,
    NotConfigured = 3,
    InvalidConfig = 4,
    InvalidCommand = 5,
    UnknownSource = 6,
    Unauthorized = 7,
    InvalidTime = 8,
    DuplicateSequence = 9,
    OutOfOrderSequence = 10,
}

pub const FOC_COMMAND_DECISION_SAFE: u32 = 0;
pub const FOC_COMMAND_DECISION_SELECTED: u32 = 1;
pub const FOC_COMMAND_SAFE_NO_COMMAND: u32 = 0;
pub const FOC_COMMAND_SAFE_FAULT_ACTIVE: u32 = 1;
pub const FOC_COMMAND_SAFE_EMERGENCY_STOP_LATCHED: u32 = 2;
pub const FOC_COMMAND_SAFE_COMMAND_TIMEOUT: u32 = 3;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocCommandSourcePolicyAbi {
    pub source_id: u32,
    pub priority: u32,
    pub permissions: u32,
    pub lease_ms: u32,
    pub command_timeout_ms: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FocCommandDecisionAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub decision: u32,
    pub safe_reason: u32,
    pub source_id: u32,
    pub has_setpoint: u32,
    pub has_action: u32,
    pub reserved: u32,
    pub setpoint: ProductCommand,
    pub action: ProductCommand,
}

impl Default for FocCommandDecisionAbi {
    fn default() -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_COMMAND_ABI_VERSION,
            decision: FOC_COMMAND_DECISION_SAFE,
            safe_reason: FOC_COMMAND_SAFE_NO_COMMAND,
            source_id: 0,
            has_setpoint: 0,
            has_action: 0,
            reserved: 0,
            setpoint: ProductCommand::default(),
            action: ProductCommand::default(),
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocCommandArbiterStatusAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub configured_source_count: u32,
    pub submitted: u32,
    pub accepted: u32,
    pub rejected: u32,
    pub polls: u32,
    pub last_result: u32,
    pub last_detail: u32,
}

#[repr(C, align(8))]
pub struct FocCommandContextStorage {
    pub bytes: [u8; FOC_COMMAND_CONTEXT_CAPACITY],
}

// The largest fixed-capacity variant intentionally stays inline. Heap-backed
// indirection is unavailable on the no_std target and would weaken deterministic
// ownership; the caller-provided 4 KiB context already budgets the maximum.
#[allow(clippy::large_enum_variant)]
enum ActiveArbiter {
    Disabled,
    Enabled(CommandArbiter<FOC_COMMAND_MAX_SOURCES>),
}

impl ActiveArbiter {
    fn submit(&mut self, command: ProductCommand, now_ms: u32) -> Result<(), CommandSubmitError> {
        match self {
            Self::Disabled => Err(CommandSubmitError::UnknownSource),
            Self::Enabled(value) => value.submit(command, now_ms).map(|_| ()),
        }
    }

    fn poll(&mut self, now_ms: u32, fault_active: bool) -> CommandDecision {
        match self {
            Self::Disabled => CommandDecision::Safe(CommandSafeReason::NoCommand),
            Self::Enabled(value) => value.poll(now_ms, fault_active),
        }
    }
}

struct CommandAbiContext {
    magic: u32,
    configured_source_count: u32,
    submitted: u32,
    accepted: u32,
    rejected: u32,
    polls: u32,
    last_result: FocCommandStatus,
    last_detail: u32,
    arbiter: ActiveArbiter,
}

impl CommandAbiContext {
    fn new() -> Self {
        Self {
            magic: COMMAND_CONTEXT_MAGIC,
            configured_source_count: 0,
            submitted: 0,
            accepted: 0,
            rejected: 0,
            polls: 0,
            last_result: FocCommandStatus::NotConfigured,
            last_detail: 0,
            arbiter: ActiveArbiter::Disabled,
        }
    }

    fn record(&mut self, status: FocCommandStatus, detail: u32) -> FocCommandStatus {
        self.last_result = status;
        self.last_detail = detail;
        status
    }
}

const _: () = assert!(size_of::<FocCommandSourcePolicyAbi>() == 20);
const _: () = assert!(size_of::<FocCommandDecisionAbi>() == 240);
const _: () = assert!(size_of::<FocCommandArbiterStatusAbi>() == 36);
const _: () = assert!(size_of::<CommandAbiContext>() <= FOC_COMMAND_CONTEXT_CAPACITY);
const _: () = assert!(align_of::<CommandAbiContext>() <= align_of::<FocCommandContextStorage>());

fn address_is_aligned<T>(address: usize) -> bool {
    address & (align_of::<T>() - 1) == 0
}

unsafe fn context_mut<'a>(
    storage: *mut FocCommandContextStorage,
) -> Result<&'a mut CommandAbiContext, FocCommandStatus> {
    if storage.is_null() || !address_is_aligned::<FocCommandContextStorage>(storage as usize) {
        return Err(FocCommandStatus::InvalidArgument);
    }
    let context = unsafe { &mut *storage.cast::<CommandAbiContext>() };
    if context.magic != COMMAND_CONTEXT_MAGIC {
        return Err(FocCommandStatus::NotInitialized);
    }
    Ok(context)
}

unsafe fn read_copy<T: Copy>(value: *const T) -> Result<T, FocCommandStatus> {
    if value.is_null() || !address_is_aligned::<T>(value as usize) {
        Err(FocCommandStatus::InvalidArgument)
    } else {
        Ok(unsafe { ptr::read(value) })
    }
}

unsafe fn write_copy<T: Copy>(output: *mut T, value: T) -> Result<(), FocCommandStatus> {
    if output.is_null() || !address_is_aligned::<T>(output as usize) {
        Err(FocCommandStatus::InvalidArgument)
    } else {
        unsafe { ptr::write(output, value) };
        Ok(())
    }
}

fn policy_from_abi(
    value: FocCommandSourcePolicyAbi,
) -> Result<foc_control::CommandSourcePolicy, FocCommandStatus> {
    CommandSourceConfig {
        source_id: value.source_id,
        priority: value.priority,
        permissions: value.permissions,
        lease_ms: value.lease_ms,
        command_timeout_ms: value.command_timeout_ms,
    }
    .policy()
    .map_err(|_| FocCommandStatus::InvalidConfig)
}

unsafe fn read_policies(
    sources: *const FocCommandSourcePolicyAbi,
    source_count: usize,
) -> Result<[foc_control::CommandSourcePolicy; FOC_COMMAND_MAX_SOURCES], FocCommandStatus> {
    if sources.is_null() || !address_is_aligned::<FocCommandSourcePolicyAbi>(sources as usize) {
        return Err(FocCommandStatus::InvalidArgument);
    }
    let first = policy_from_abi(unsafe { ptr::read(sources) })?;
    let mut policies = [first; FOC_COMMAND_MAX_SOURCES];
    for (index, policy) in policies[..source_count].iter_mut().enumerate().skip(1) {
        *policy = policy_from_abi(unsafe { ptr::read(sources.add(index)) })?;
    }
    Ok(policies)
}

fn map_config_error(error: ArbiterConfigError) -> u32 {
    match error {
        ArbiterConfigError::EmptySourceSet => 1,
        ArbiterConfigError::SourceCountExceedsCapacity => 2,
        ArbiterConfigError::DuplicateSourceId => 3,
        ArbiterConfigError::ZeroDuration => 4,
        ArbiterConfigError::AmbiguousDuration => 5,
        ArbiterConfigError::LeaseExceedsCommandTimeout => 6,
    }
}

fn map_submit_error(error: CommandSubmitError) -> (FocCommandStatus, u32) {
    match error {
        CommandSubmitError::InvalidContract(value) => (
            FocCommandStatus::InvalidCommand,
            contract_error_detail(value),
        ),
        CommandSubmitError::UnknownSource => (FocCommandStatus::UnknownSource, 0),
        CommandSubmitError::UnauthorizedCommand => (FocCommandStatus::Unauthorized, 0),
        CommandSubmitError::InvalidTimeWindow => (FocCommandStatus::InvalidTime, 1),
        CommandSubmitError::FutureTimestamp => (FocCommandStatus::InvalidTime, 2),
        CommandSubmitError::Expired => (FocCommandStatus::InvalidTime, 3),
        CommandSubmitError::TimedOut => (FocCommandStatus::InvalidTime, 4),
        CommandSubmitError::DuplicateSequence => (FocCommandStatus::DuplicateSequence, 0),
        CommandSubmitError::OutOfOrderSequence => (FocCommandStatus::OutOfOrderSequence, 0),
    }
}

fn contract_error_detail(error: ContractError) -> u32 {
    match error {
        ContractError::Header => 1,
        ContractError::UnsupportedAxis => 2,
        ContractError::UnknownEnum => 3,
        ContractError::UnknownFlags => 4,
        ContractError::NonFinite => 5,
        ContractError::OutOfRange => 6,
        ContractError::UnsupportedCombination => 7,
        ContractError::NonCanonical => 8,
    }
}

fn safe_reason(value: CommandSafeReason) -> u32 {
    match value {
        CommandSafeReason::NoCommand => FOC_COMMAND_SAFE_NO_COMMAND,
        CommandSafeReason::FaultActive => FOC_COMMAND_SAFE_FAULT_ACTIVE,
        CommandSafeReason::EmergencyStopLatched => FOC_COMMAND_SAFE_EMERGENCY_STOP_LATCHED,
        CommandSafeReason::CommandTimeout => FOC_COMMAND_SAFE_COMMAND_TIMEOUT,
    }
}

fn selected_output(value: ArbitratedCommand) -> FocCommandDecisionAbi {
    let mut output = FocCommandDecisionAbi {
        decision: FOC_COMMAND_DECISION_SELECTED,
        source_id: value.source_id,
        ..FocCommandDecisionAbi::default()
    };
    if let Some(setpoint) = value.setpoint {
        output.has_setpoint = 1;
        output.setpoint = setpoint;
    }
    if let Some(action) = value.action {
        output.has_action = 1;
        output.action = action;
    }
    output
}

#[no_mangle]
pub extern "C" fn foc_rust_command_abi_version() -> u32 {
    FOC_COMMAND_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn foc_rust_command_context_required_size() -> u32 {
    size_of::<CommandAbiContext>() as u32
}

#[no_mangle]
pub extern "C" fn foc_rust_command_context_required_align() -> u32 {
    align_of::<CommandAbiContext>() as u32
}

#[no_mangle]
/// Initialize a caller-owned command-arbiter context in the disabled state.
///
/// # Safety
/// `storage` must be aligned, writable and valid for the duration of the call.
pub unsafe extern "C" fn foc_rust_command_init(
    storage: *mut FocCommandContextStorage,
) -> FocCommandStatus {
    if storage.is_null() || !address_is_aligned::<FocCommandContextStorage>(storage as usize) {
        return FocCommandStatus::InvalidArgument;
    }
    unsafe {
        ptr::write(
            storage.cast::<CommandAbiContext>(),
            CommandAbiContext::new(),
        )
    };
    FocCommandStatus::Ok
}

#[no_mangle]
/// Atomically replace all trusted source policies.  A zero count disables the arbiter.
///
/// # Safety
/// `storage` must be initialized.  For a non-zero count, `sources` must point
/// to `source_count` aligned readable policies.
pub unsafe extern "C" fn foc_rust_command_configure(
    storage: *mut FocCommandContextStorage,
    sources: *const FocCommandSourcePolicyAbi,
    source_count: u32,
) -> FocCommandStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if source_count == 0 {
        context.arbiter = ActiveArbiter::Disabled;
        context.configured_source_count = 0;
        return context.record(FocCommandStatus::Ok, 0);
    }
    if source_count as usize > FOC_COMMAND_MAX_SOURCES {
        return context.record(FocCommandStatus::InvalidArgument, source_count);
    }

    let policies = match unsafe { read_policies(sources, source_count as usize) } {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let replacement = match CommandArbiter::new_with_active_count(policies, source_count as usize) {
        Ok(value) => ActiveArbiter::Enabled(value),
        Err(error) => {
            return context.record(FocCommandStatus::InvalidConfig, map_config_error(error));
        }
    };
    context.arbiter = replacement;
    context.configured_source_count = source_count;
    context.record(FocCommandStatus::Ok, 0)
}

#[no_mangle]
/// Validate and queue one canonical product command.
///
/// # Safety
/// `storage` must be initialized and `command` must be aligned and readable.
pub unsafe extern "C" fn foc_rust_command_submit(
    storage: *mut FocCommandContextStorage,
    command: *const ProductCommand,
    now_ms: u32,
) -> FocCommandStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if context.configured_source_count == 0 {
        context.rejected = context.rejected.wrapping_add(1);
        return context.record(FocCommandStatus::NotConfigured, 0);
    }
    let command = match unsafe { read_copy(command) } {
        Ok(value) => value,
        Err(status) => {
            context.rejected = context.rejected.wrapping_add(1);
            return context.record(status, 0);
        }
    };
    context.submitted = context.submitted.wrapping_add(1);
    match context.arbiter.submit(command, now_ms) {
        Ok(()) => {
            context.accepted = context.accepted.wrapping_add(1);
            context.record(FocCommandStatus::Ok, 0)
        }
        Err(error) => {
            context.rejected = context.rejected.wrapping_add(1);
            let (status, detail) = map_submit_error(error);
            context.record(status, detail)
        }
    }
}

#[no_mangle]
/// Resolve one management tick.  `fault_active` must be canonical zero/one.
///
/// # Safety
/// `storage` must be initialized and `output` must be aligned and writable.
pub unsafe extern "C" fn foc_rust_command_poll(
    storage: *mut FocCommandContextStorage,
    now_ms: u32,
    fault_active: u32,
    output: *mut FocCommandDecisionAbi,
) -> FocCommandStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if fault_active > 1 {
        return context.record(FocCommandStatus::InvalidArgument, fault_active);
    }
    let decision = match context.arbiter.poll(now_ms, fault_active != 0) {
        CommandDecision::Safe(reason) => FocCommandDecisionAbi {
            safe_reason: safe_reason(reason),
            ..FocCommandDecisionAbi::default()
        },
        CommandDecision::Selected(value) => selected_output(value),
    };
    context.polls = context.polls.wrapping_add(1);
    match unsafe { write_copy(output, decision) } {
        Ok(()) => context.record(FocCommandStatus::Ok, 0),
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Return management-path counters and the last result.
///
/// # Safety
/// `storage` must be initialized and `output` must be aligned and writable.
pub unsafe extern "C" fn foc_rust_command_get_status(
    storage: *mut FocCommandContextStorage,
    output: *mut FocCommandArbiterStatusAbi,
) -> FocCommandStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let status = FocCommandArbiterStatusAbi {
        struct_size: size_of::<FocCommandArbiterStatusAbi>() as u32,
        abi_version: FOC_COMMAND_ABI_VERSION,
        configured_source_count: context.configured_source_count,
        submitted: context.submitted,
        accepted: context.accepted,
        rejected: context.rejected,
        polls: context.polls,
        last_result: context.last_result as u32,
        last_detail: context.last_detail,
    };
    match unsafe { write_copy(output, status) } {
        Ok(()) => FocCommandStatus::Ok,
        Err(status) => context.record(status, 0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use foc_control::{
        AxisRequest, ControlMode, FeedbackMode, InputMode, ProductCommandKind,
        COMMAND_SOURCE_PERMISSION_AXIS_REQUEST, COMMAND_SOURCE_PERMISSION_RELEASE,
        COMMAND_SOURCE_PERMISSION_SETPOINT,
    };

    fn storage() -> FocCommandContextStorage {
        FocCommandContextStorage {
            bytes: [0; FOC_COMMAND_CONTEXT_CAPACITY],
        }
    }

    fn source(source_id: u32, priority: u32, permissions: u32) -> FocCommandSourcePolicyAbi {
        FocCommandSourcePolicyAbi {
            source_id,
            priority,
            permissions,
            lease_ms: 50,
            command_timeout_ms: 100,
        }
    }

    fn setpoint(source_id: u32, sequence: u32) -> ProductCommand {
        ProductCommand {
            source_id,
            sequence,
            created_at_ms: 10,
            valid_until_ms: 100,
            command_kind: ProductCommandKind::Setpoint as u32,
            control_mode: ControlMode::Velocity as u32,
            input_mode: InputMode::Passthrough as u32,
            feedback_mode: FeedbackMode::Sensorless as u32,
            velocity_ref_rad_s: 20.0,
            ..ProductCommand::default()
        }
    }

    fn start(source_id: u32, sequence: u32) -> ProductCommand {
        ProductCommand {
            source_id,
            sequence,
            created_at_ms: 10,
            valid_until_ms: 100,
            command_kind: ProductCommandKind::AxisRequest as u32,
            axis_request: AxisRequest::ClosedLoopControl as u32,
            ..ProductCommand::default()
        }
    }

    #[test]
    fn fixed_capacity_abi_configures_submits_polls_and_disables() {
        let mut context = storage();
        let policies = [
            source(
                11,
                10,
                COMMAND_SOURCE_PERMISSION_RELEASE
                    | COMMAND_SOURCE_PERMISSION_SETPOINT
                    | COMMAND_SOURCE_PERMISSION_AXIS_REQUEST,
            ),
            source(22, 20, COMMAND_SOURCE_PERMISSION_RELEASE),
        ];
        assert_eq!(
            unsafe { foc_rust_command_init(&mut context) },
            FocCommandStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_command_configure(&mut context, policies.as_ptr(), 2) },
            FocCommandStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_command_submit(&mut context, &setpoint(11, 1), 20) },
            FocCommandStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_command_submit(&mut context, &start(11, 2), 20) },
            FocCommandStatus::Ok
        );
        let mut decision = FocCommandDecisionAbi::default();
        assert_eq!(
            unsafe { foc_rust_command_poll(&mut context, 20, 0, &mut decision) },
            FocCommandStatus::Ok
        );
        assert_eq!(decision.decision, FOC_COMMAND_DECISION_SELECTED);
        assert_eq!(decision.source_id, 11);
        assert_eq!(decision.has_setpoint, 1);
        assert_eq!(decision.has_action, 1);

        assert_eq!(
            unsafe { foc_rust_command_submit(&mut context, &setpoint(22, 1), 20) },
            FocCommandStatus::Unauthorized
        );
        assert_eq!(
            unsafe { foc_rust_command_poll(&mut context, 21, 1, &mut decision) },
            FocCommandStatus::Ok
        );
        assert_eq!(decision.safe_reason, FOC_COMMAND_SAFE_FAULT_ACTIVE);

        assert_eq!(
            unsafe { foc_rust_command_configure(&mut context, ptr::null(), 0) },
            FocCommandStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_command_submit(&mut context, &setpoint(11, 3), 22) },
            FocCommandStatus::NotConfigured
        );
    }

    #[test]
    fn configure_is_fail_closed_and_preserves_previous_policy() {
        let mut context = storage();
        let valid = [source(
            11,
            10,
            COMMAND_SOURCE_PERMISSION_RELEASE | COMMAND_SOURCE_PERMISSION_SETPOINT,
        )];
        assert_eq!(
            unsafe { foc_rust_command_init(&mut context) },
            FocCommandStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_command_configure(&mut context, valid.as_ptr(), 1) },
            FocCommandStatus::Ok
        );
        let duplicate = [valid[0], valid[0]];
        assert_eq!(
            unsafe { foc_rust_command_configure(&mut context, duplicate.as_ptr(), 2) },
            FocCommandStatus::InvalidConfig
        );
        assert_eq!(
            unsafe { foc_rust_command_submit(&mut context, &setpoint(11, 1), 20) },
            FocCommandStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_command_configure(&mut context, valid.as_ptr(), 9) },
            FocCommandStatus::InvalidArgument
        );
    }
}
