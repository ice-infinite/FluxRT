//! Independent C ABI for the default-off P4.2 motion planner and cascade.
//!
//! Configuration and enable/disable are management-plane calls. `step` is the
//! allocation-free realtime boundary; it advances planner and cascade as one
//! transaction, so a rejected cascade sample cannot leave a ramp half advanced.

use core::mem::{align_of, size_of};
use core::ptr;

use foc_control::{
    ControlMode, InputMode, MotionCascadeController, MotionCascadeError, MotionCascadeFeedback,
    MotionConfigError, MotionFeedback, MotionPlannerError, MotionReferencePlanner,
    MotionRuntimeConfig, ProductCommand, ProductCommandKind,
};

use crate::FocConfigBundleAbi;

pub const FOC_MOTION_ABI_VERSION: u32 = 0x0002_0000;
pub const FOC_MOTION_FEEDBACK_VERSION: u32 = 1;
pub const FOC_MOTION_OUTPUT_VERSION: u32 = 1;
pub const FOC_MOTION_REALTIME_REQUEST_VERSION: u32 = 1;
pub const FOC_MOTION_CONTEXT_CAPACITY: usize = 512;

pub const FOC_MOTION_REALTIME_REQUEST_COMMAND_VALID: u32 = 1 << 0;
pub const FOC_MOTION_REALTIME_REQUEST_POSITION_VALID: u32 = 1 << 1;
pub const FOC_MOTION_REALTIME_REQUEST_STOP: u32 = 1 << 2;
pub const FOC_MOTION_REALTIME_REQUEST_FAULT: u32 = 1 << 3;
pub const FOC_MOTION_REALTIME_REQUEST_KNOWN_MASK: u32 = FOC_MOTION_REALTIME_REQUEST_COMMAND_VALID
    | FOC_MOTION_REALTIME_REQUEST_POSITION_VALID
    | FOC_MOTION_REALTIME_REQUEST_STOP
    | FOC_MOTION_REALTIME_REQUEST_FAULT;

pub(crate) const FOC_MOTION_DETAIL_INVALID_REQUEST: u32 = 301;
pub(crate) const FOC_MOTION_DETAIL_FAULT_REQUEST: u32 = 302;
pub(crate) const FOC_MOTION_DETAIL_COMMAND_TIME: u32 = 303;
pub(crate) const FOC_MOTION_DETAIL_PUBLICATION_SEQUENCE: u32 = 304;
pub(crate) const FOC_MOTION_DETAIL_COMMAND_SEQUENCE: u32 = 305;

pub const FOC_MOTION_FEEDBACK_VALID_POSITION: u32 = 1 << 0;
pub const FOC_MOTION_FEEDBACK_VALID_VELOCITY: u32 = 1 << 1;
pub const FOC_MOTION_FEEDBACK_VALID_CURRENT_Q: u32 = 1 << 2;
pub const FOC_MOTION_FEEDBACK_VALID_KNOWN_MASK: u32 = FOC_MOTION_FEEDBACK_VALID_POSITION
    | FOC_MOTION_FEEDBACK_VALID_VELOCITY
    | FOC_MOTION_FEEDBACK_VALID_CURRENT_Q;

const MOTION_CONTEXT_MAGIC: u32 = 0x464D_4F54; // "FMOT"

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FocMotionStatus {
    Ok = 0,
    InvalidArgument = 1,
    NotInitialized = 2,
    NotConfigured = 3,
    Disabled = 4,
    InvalidConfig = 5,
    InvalidCommand = 6,
    InvalidFeedback = 7,
    TransitionRequired = 8,
    ControlFailure = 9,
    InvalidState = 10,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocMotionFeedbackAbi {
    pub struct_size: u32,
    pub version: u32,
    pub sequence: u32,
    pub valid_flags: u32,
    pub mechanical_position_rad: f32,
    pub mechanical_velocity_rad_s: f32,
    pub current_q_a: f32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocMotionOutputAbi {
    pub struct_size: u32,
    pub version: u32,
    pub config_revision: u32,
    pub source_sequence: u32,
    pub control_mode: u32,
    pub input_mode: u32,
    pub reference_flags: u32,
    pub cascade_flags: u32,
    pub detail: u32,
    pub position_error_rad: f32,
    pub velocity_reference_rad_s: f32,
    pub velocity_error_rad_s: f32,
    pub torque_reference_nm: f32,
    pub current_d_reference_a: f32,
    pub current_q_reference_a: f32,
    pub reserved: u32,
}

impl FocMotionOutputAbi {
    pub(crate) fn safe(config_revision: u32, source_sequence: u32) -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            version: FOC_MOTION_OUTPUT_VERSION,
            config_revision,
            source_sequence,
            control_mode: ControlMode::Inactive as u32,
            input_mode: InputMode::Inactive as u32,
            ..Self::default()
        }
    }
}

/// Fixed management-task to ADC-ISR request consumed by the combined realtime
/// entry.  The command begins at byte 32 by ABI contract.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocMotionRealtimeRequest {
    pub struct_size: u32,
    pub version: u32,
    pub publication_sequence: u32,
    pub request_flags: u32,
    pub consumer_now_ms: u32,
    pub fault_detail: u32,
    pub mechanical_position_rad: f32,
    pub position_sampled_at_ms: u32,
    pub command: ProductCommand,
}

#[repr(C, align(8))]
pub struct FocMotionContextStorage {
    pub bytes: [u8; FOC_MOTION_CONTEXT_CAPACITY],
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct MotionAbiContext {
    magic: u32,
    configured: bool,
    enabled: bool,
    config: Option<MotionRuntimeConfig>,
    planner: MotionReferencePlanner,
    cascade: MotionCascadeController,
    outer_loop_divider: u32,
    outer_loop_counter: u32,
    has_publication_sequence: bool,
    last_publication_sequence: u32,
    has_command_sequence: bool,
    last_command_sequence: u32,
    last_current_reference: foc_control::CurrentCommand,
    last_output: FocMotionOutputAbi,
}

impl MotionAbiContext {
    fn new() -> Self {
        Self {
            magic: MOTION_CONTEXT_MAGIC,
            configured: false,
            enabled: false,
            config: None,
            planner: MotionReferencePlanner::default(),
            cascade: MotionCascadeController::default(),
            outer_loop_divider: 1,
            outer_loop_counter: 0,
            has_publication_sequence: false,
            last_publication_sequence: 0,
            has_command_sequence: false,
            last_command_sequence: 0,
            last_current_reference: foc_control::CurrentCommand::default(),
            last_output: FocMotionOutputAbi::safe(0, 0),
        }
    }

    pub(crate) fn disable(&mut self) {
        self.enabled = false;
        self.planner.set_enabled(false);
        self.cascade.set_enabled(false);
        self.outer_loop_counter = 0;
        self.has_publication_sequence = false;
        self.last_publication_sequence = 0;
        self.has_command_sequence = false;
        self.last_command_sequence = 0;
        self.last_current_reference = foc_control::CurrentCommand::default();
        self.last_output =
            FocMotionOutputAbi::safe(self.config.map_or(0, |config| config.config_revision), 0);
    }

    pub(crate) fn realtime_tick_due(&self) -> bool {
        self.outer_loop_counter == 0
    }

    pub(crate) fn safe_output(&self, source_sequence: u32) -> FocMotionOutputAbi {
        FocMotionOutputAbi::safe(
            self.config.map_or(0, |config| config.config_revision),
            source_sequence,
        )
    }

    pub(crate) fn prepare_realtime_request(
        &mut self,
        request: &FocMotionRealtimeRequest,
        output: &mut FocMotionOutputAbi,
    ) -> Result<(), (FocMotionStatus, u32)> {
        let config = self.config.filter(|_| self.configured).ok_or((
            FocMotionStatus::NotConfigured,
            FOC_MOTION_DETAIL_INVALID_REQUEST,
        ))?;
        *output = FocMotionOutputAbi::safe(config.config_revision, request.command.sequence);
        if !self.enabled {
            return Err((FocMotionStatus::Disabled, FOC_MOTION_DETAIL_INVALID_REQUEST));
        }
        if request.struct_size != size_of::<FocMotionRealtimeRequest>() as u32
            || request.version != FOC_MOTION_REALTIME_REQUEST_VERSION
            || request.request_flags & !FOC_MOTION_REALTIME_REQUEST_KNOWN_MASK != 0
            || request.request_flags & FOC_MOTION_REALTIME_REQUEST_COMMAND_VALID == 0
            || request.request_flags
                & (FOC_MOTION_REALTIME_REQUEST_STOP | FOC_MOTION_REALTIME_REQUEST_FAULT)
                != 0
            || (request.request_flags & FOC_MOTION_REALTIME_REQUEST_POSITION_VALID != 0
                && !request.mechanical_position_rad.is_finite())
            || (request.request_flags & FOC_MOTION_REALTIME_REQUEST_POSITION_VALID == 0
                && request.position_sampled_at_ms != 0)
            || (request.request_flags & FOC_MOTION_REALTIME_REQUEST_FAULT == 0
                && request.fault_detail != 0)
            || request.command.validate().is_err()
            || request.command.command_kind != ProductCommandKind::Setpoint as u32
        {
            return Err((
                FocMotionStatus::InvalidCommand,
                FOC_MOTION_DETAIL_INVALID_REQUEST,
            ));
        }

        let window_ms = request
            .command
            .valid_until_ms
            .wrapping_sub(request.command.created_at_ms);
        let age_ms = request
            .consumer_now_ms
            .wrapping_sub(request.command.created_at_ms);
        if window_ms == 0
            || window_ms >= 0x8000_0000
            || age_ms >= 0x8000_0000
            || age_ms >= window_ms
        {
            return Err((
                FocMotionStatus::InvalidCommand,
                FOC_MOTION_DETAIL_COMMAND_TIME,
            ));
        }
        if request.request_flags & FOC_MOTION_REALTIME_REQUEST_POSITION_VALID != 0 {
            let position_age_ms = request
                .consumer_now_ms
                .wrapping_sub(request.position_sampled_at_ms);
            if position_age_ms >= 0x8000_0000 || position_age_ms >= window_ms {
                return Err((
                    FocMotionStatus::InvalidFeedback,
                    FOC_MOTION_DETAIL_COMMAND_TIME,
                ));
            }
        }

        if self.has_publication_sequence {
            let distance = request
                .publication_sequence
                .wrapping_sub(self.last_publication_sequence);
            if distance >= 0x8000_0000 {
                return Err((
                    FocMotionStatus::InvalidCommand,
                    FOC_MOTION_DETAIL_PUBLICATION_SEQUENCE,
                ));
            }
            if distance == 0 && request.command.sequence != self.last_command_sequence {
                return Err((
                    FocMotionStatus::InvalidCommand,
                    FOC_MOTION_DETAIL_COMMAND_SEQUENCE,
                ));
            }
        }
        if self.has_command_sequence {
            let distance = request
                .command
                .sequence
                .wrapping_sub(self.last_command_sequence);
            if distance >= 0x8000_0000 {
                return Err((
                    FocMotionStatus::InvalidCommand,
                    FOC_MOTION_DETAIL_COMMAND_SEQUENCE,
                ));
            }
        }

        self.has_publication_sequence = true;
        self.last_publication_sequence = request.publication_sequence;
        self.has_command_sequence = true;
        self.last_command_sequence = request.command.sequence;
        Ok(())
    }

    pub(crate) fn realtime_reference(
        &mut self,
        request: &FocMotionRealtimeRequest,
        mechanical_velocity_rad_s: f32,
        current_q_a: f32,
        output: &mut FocMotionOutputAbi,
    ) -> Result<foc_control::CurrentCommand, (FocMotionStatus, u32)> {
        if self.outer_loop_counter != 0 {
            self.outer_loop_counter = (self.outer_loop_counter + 1) % self.outer_loop_divider;
            *output = self.last_output;
            return Ok(self.last_current_reference);
        }

        let config = self.config.filter(|_| self.configured).ok_or((
            FocMotionStatus::NotConfigured,
            FOC_MOTION_DETAIL_INVALID_REQUEST,
        ))?;
        let position_valid =
            request.request_flags & FOC_MOTION_REALTIME_REQUEST_POSITION_VALID != 0;
        let planner_feedback = MotionFeedback {
            position_valid,
            velocity_valid: true,
            mechanical_position_rad: request.mechanical_position_rad,
            mechanical_velocity_rad_s,
        };
        let cascade_feedback = MotionCascadeFeedback {
            position_valid,
            velocity_valid: true,
            current_q_valid: true,
            mechanical_position_rad: request.mechanical_position_rad,
            mechanical_velocity_rad_s,
            current_q_a,
        };
        /* `foc_rust_realtime_step_with_motion_impl` already executes on a full
         * MotionAbiContext shadow and commits it only after the current loop also
         * succeeds.  Repeating planner/cascade shadows and immutable validation
         * here inflated both MSP depth and the 1 kHz outer-loop WCET. */
        let reference = self
            .planner
            .step_prevalidated_realtime(&config.planner, &request.command, planner_feedback)
            .map_err(map_planner_error)?;
        let result = self
            .cascade
            .step_prevalidated_realtime(&config.cascade, &reference, cascade_feedback)
            .map_err(map_cascade_error)?;
        let final_output = FocMotionOutputAbi {
            struct_size: size_of::<FocMotionOutputAbi>() as u32,
            version: FOC_MOTION_OUTPUT_VERSION,
            config_revision: config.config_revision,
            source_sequence: request.command.sequence,
            control_mode: result.control_mode as u32,
            input_mode: result.input_mode as u32,
            reference_flags: result.reference_flags,
            cascade_flags: result.flags,
            detail: 0,
            position_error_rad: result.position_error_rad,
            velocity_reference_rad_s: result.velocity_reference_rad_s,
            velocity_error_rad_s: result.velocity_error_rad_s,
            torque_reference_nm: result.torque_reference_nm,
            current_d_reference_a: result.current_reference.id_ref_a,
            current_q_reference_a: result.current_reference.iq_ref_a,
            reserved: 0,
        };
        self.last_current_reference = result.current_reference;
        self.last_output = final_output;
        self.outer_loop_counter = (self.outer_loop_counter + 1) % self.outer_loop_divider;
        *output = final_output;
        Ok(result.current_reference)
    }
}

pub(crate) fn realtime_request_envelope_is_valid(request: &FocMotionRealtimeRequest) -> bool {
    request.struct_size == size_of::<FocMotionRealtimeRequest>() as u32
        && request.version == FOC_MOTION_REALTIME_REQUEST_VERSION
        && request.request_flags & !FOC_MOTION_REALTIME_REQUEST_KNOWN_MASK == 0
        && (request.request_flags & FOC_MOTION_REALTIME_REQUEST_POSITION_VALID == 0
            || request.mechanical_position_rad.is_finite())
}

const _: () = assert!(size_of::<ProductCommand>() == 104);
const _: () = assert!(size_of::<FocMotionFeedbackAbi>() == 32);
const _: () = assert!(size_of::<FocMotionOutputAbi>() == 64);
const _: () = assert!(size_of::<FocMotionRealtimeRequest>() == 136);
const _: () = assert!(core::mem::offset_of!(FocMotionRealtimeRequest, command) == 32);
const _: () = assert!(size_of::<MotionAbiContext>() <= FOC_MOTION_CONTEXT_CAPACITY);
const _: () = assert!(align_of::<MotionAbiContext>() <= align_of::<FocMotionContextStorage>());

#[no_mangle]
pub extern "C" fn foc_rust_motion_abi_version() -> u32 {
    FOC_MOTION_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn foc_rust_motion_context_required_size() -> u32 {
    size_of::<MotionAbiContext>() as u32
}

#[no_mangle]
pub extern "C" fn foc_rust_motion_context_required_align() -> u32 {
    align_of::<MotionAbiContext>() as u32
}

#[no_mangle]
/// Initialize caller-owned motion context storage in place.
///
/// # Safety
/// `storage` must be writable, correctly aligned, and valid for the full
/// [`FocMotionContextStorage`] size. It must not be accessed concurrently.
pub unsafe extern "C" fn foc_rust_motion_init(
    storage: *mut FocMotionContextStorage,
) -> FocMotionStatus {
    if storage.is_null() || !pointer_is_aligned(storage.cast::<MotionAbiContext>()) {
        return FocMotionStatus::InvalidArgument;
    }
    unsafe { ptr::write(storage.cast::<MotionAbiContext>(), MotionAbiContext::new()) };
    FocMotionStatus::Ok
}

#[no_mangle]
/// Validate and copy a persisted configuration while motion is disabled.
///
/// # Safety
/// Both pointers must be valid and correctly aligned for their pointee types.
/// The context must not be accessed concurrently.
pub unsafe extern "C" fn foc_rust_motion_configure(
    storage: *mut FocMotionContextStorage,
    config: *const FocConfigBundleAbi,
) -> FocMotionStatus {
    unsafe { foc_rust_motion_configure_realtime(storage, config, 1) }
}

#[no_mangle]
/// Configure an ISR-owned motion context whose outer loop is divided from the
/// bundle's fast control frequency.
///
/// # Safety
/// Both pointers must be valid and the context must be stopped and exclusively
/// owned for the duration of the call.
pub unsafe extern "C" fn foc_rust_motion_configure_realtime(
    storage: *mut FocMotionContextStorage,
    config: *const FocConfigBundleAbi,
    outer_loop_divider: u32,
) -> FocMotionStatus {
    if config.is_null() {
        return FocMotionStatus::InvalidArgument;
    }
    let config = unsafe { *config };
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if context.enabled {
        return FocMotionStatus::InvalidState;
    }
    let bundle = match config.to_bundle() {
        Ok(value) => value,
        Err(_) => return FocMotionStatus::InvalidArgument,
    };
    if outer_loop_divider == 0 || bundle.board.control_frequency_hz % outer_loop_divider != 0 {
        return FocMotionStatus::InvalidConfig;
    }
    let mut runtime = match MotionRuntimeConfig::from_bundle(&bundle) {
        Ok(value) => value,
        Err(MotionConfigError::Disabled) => return FocMotionStatus::Disabled,
        Err(_) => return FocMotionStatus::InvalidConfig,
    };
    let outer_period_s = outer_loop_divider as f32 / bundle.board.control_frequency_hz as f32;
    runtime.planner.control_period_s = outer_period_s;
    runtime.cascade.control_period_s = outer_period_s;
    if runtime.planner.validate().is_err() || runtime.cascade.validate().is_err() {
        return FocMotionStatus::InvalidConfig;
    }
    context.config = Some(runtime);
    context.configured = true;
    context.outer_loop_divider = outer_loop_divider;
    context.disable();
    FocMotionStatus::Ok
}

#[no_mangle]
/// Enable a previously configured motion context and reset its dynamic state.
///
/// # Safety
/// `storage` must name a context initialized by [`foc_rust_motion_init`] and
/// must not be accessed concurrently.
pub unsafe extern "C" fn foc_rust_motion_enable(
    storage: *mut FocMotionContextStorage,
) -> FocMotionStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if !context.configured || context.config.is_none() {
        return FocMotionStatus::NotConfigured;
    }
    context.planner.set_enabled(true);
    context.cascade.set_enabled(true);
    context.outer_loop_counter = 0;
    context.has_publication_sequence = false;
    context.last_publication_sequence = 0;
    context.has_command_sequence = false;
    context.last_command_sequence = 0;
    context.last_current_reference = foc_control::CurrentCommand::default();
    context.last_output =
        FocMotionOutputAbi::safe(context.config.map_or(0, |config| config.config_revision), 0);
    context.enabled = true;
    FocMotionStatus::Ok
}

#[no_mangle]
/// Disable motion and discard planner/controller dynamic state.
///
/// # Safety
/// `storage` must name a context initialized by [`foc_rust_motion_init`] and
/// must not be accessed concurrently.
pub unsafe extern "C" fn foc_rust_motion_disable(
    storage: *mut FocMotionContextStorage,
) -> FocMotionStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    context.disable();
    FocMotionStatus::Ok
}

#[no_mangle]
/// Advance the planner and cascade by one transactional realtime tick.
///
/// # Safety
/// All pointers must be valid and correctly aligned for their pointee types.
/// `output` must be writable and may not alias the context or inputs. The
/// context must have one exclusive owner for the duration of the call.
pub unsafe extern "C" fn foc_rust_motion_step(
    storage: *mut FocMotionContextStorage,
    command: *const ProductCommand,
    feedback: *const FocMotionFeedbackAbi,
    output: *mut FocMotionOutputAbi,
) -> FocMotionStatus {
    if command.is_null() || feedback.is_null() || output.is_null() {
        return FocMotionStatus::InvalidArgument;
    }
    let command = unsafe { *command };
    let feedback = unsafe { *feedback };
    let mut safe_output = FocMotionOutputAbi::safe(0, command.sequence);
    unsafe { ptr::write(output, safe_output) };

    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let config = match context.config {
        Some(value) if context.configured => value,
        _ => return FocMotionStatus::NotConfigured,
    };
    safe_output.config_revision = config.config_revision;
    unsafe { ptr::write(output, safe_output) };
    if !context.enabled {
        return FocMotionStatus::Disabled;
    }
    if feedback.struct_size != size_of::<FocMotionFeedbackAbi>() as u32
        || feedback.version != FOC_MOTION_FEEDBACK_VERSION
        || feedback.valid_flags & !FOC_MOTION_FEEDBACK_VALID_KNOWN_MASK != 0
        || feedback.reserved != 0
    {
        return FocMotionStatus::InvalidFeedback;
    }

    let position_valid = feedback.valid_flags & FOC_MOTION_FEEDBACK_VALID_POSITION != 0;
    let velocity_valid = feedback.valid_flags & FOC_MOTION_FEEDBACK_VALID_VELOCITY != 0;
    let current_q_valid = feedback.valid_flags & FOC_MOTION_FEEDBACK_VALID_CURRENT_Q != 0;
    let planner_feedback = MotionFeedback {
        position_valid,
        velocity_valid,
        mechanical_position_rad: feedback.mechanical_position_rad,
        mechanical_velocity_rad_s: feedback.mechanical_velocity_rad_s,
    };
    let cascade_feedback = MotionCascadeFeedback {
        position_valid,
        velocity_valid,
        current_q_valid,
        mechanical_position_rad: feedback.mechanical_position_rad,
        mechanical_velocity_rad_s: feedback.mechanical_velocity_rad_s,
        current_q_a: feedback.current_q_a,
    };

    let mut next = *context;
    let reference = match next
        .planner
        .step(&config.planner, &command, planner_feedback)
    {
        Ok(value) => value,
        Err(error) => {
            let (status, detail) = map_planner_error(error);
            safe_output.detail = detail;
            unsafe { ptr::write(output, safe_output) };
            return status;
        }
    };
    let result = match next
        .cascade
        .step(&config.cascade, &reference, cascade_feedback)
    {
        Ok(value) => value,
        Err(error) => {
            let (status, detail) = map_cascade_error(error);
            safe_output.detail = detail;
            unsafe { ptr::write(output, safe_output) };
            return status;
        }
    };

    let final_output = FocMotionOutputAbi {
        struct_size: size_of::<FocMotionOutputAbi>() as u32,
        version: FOC_MOTION_OUTPUT_VERSION,
        config_revision: config.config_revision,
        source_sequence: command.sequence,
        control_mode: result.control_mode as u32,
        input_mode: result.input_mode as u32,
        reference_flags: result.reference_flags,
        cascade_flags: result.flags,
        detail: 0,
        position_error_rad: result.position_error_rad,
        velocity_reference_rad_s: result.velocity_reference_rad_s,
        velocity_error_rad_s: result.velocity_error_rad_s,
        torque_reference_nm: result.torque_reference_nm,
        current_d_reference_a: result.current_reference.id_ref_a,
        current_q_reference_a: result.current_reference.iq_ref_a,
        reserved: 0,
    };
    *context = next;
    unsafe { ptr::write(output, final_output) };
    FocMotionStatus::Ok
}

pub(crate) unsafe fn context_mut<'a>(
    storage: *mut FocMotionContextStorage,
) -> Result<&'a mut MotionAbiContext, FocMotionStatus> {
    if storage.is_null() || !pointer_is_aligned(storage.cast::<MotionAbiContext>()) {
        return Err(FocMotionStatus::InvalidArgument);
    }
    let context = unsafe { &mut *storage.cast::<MotionAbiContext>() };
    if context.magic != MOTION_CONTEXT_MAGIC {
        return Err(FocMotionStatus::NotInitialized);
    }
    Ok(context)
}

fn pointer_is_aligned<T>(pointer: *const T) -> bool {
    (pointer as usize).is_multiple_of(align_of::<T>())
}

fn map_planner_error(error: MotionPlannerError) -> (FocMotionStatus, u32) {
    match error {
        MotionPlannerError::Disabled => (FocMotionStatus::Disabled, 101),
        MotionPlannerError::InvalidConfig => (FocMotionStatus::InvalidConfig, 102),
        MotionPlannerError::InvalidCommand(_)
        | MotionPlannerError::NotSetpoint
        | MotionPlannerError::UnsupportedControlMode => (FocMotionStatus::InvalidCommand, 103),
        MotionPlannerError::MissingPositionFeedback
        | MotionPlannerError::MissingVelocityFeedback
        | MotionPlannerError::InvalidFeedback => (FocMotionStatus::InvalidFeedback, 104),
    }
}

fn map_cascade_error(error: MotionCascadeError) -> (FocMotionStatus, u32) {
    match error {
        MotionCascadeError::Disabled => (FocMotionStatus::Disabled, 201),
        MotionCascadeError::InvalidConfig => (FocMotionStatus::InvalidConfig, 202),
        MotionCascadeError::MissingPositionFeedback
        | MotionCascadeError::MissingVelocityFeedback
        | MotionCascadeError::MissingCurrentFeedback
        | MotionCascadeError::InvalidFeedback => (FocMotionStatus::InvalidFeedback, 203),
        MotionCascadeError::TransitionRequired => (FocMotionStatus::TransitionRequired, 204),
        _ => (FocMotionStatus::ControlFailure, 205),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        FocConfigAppAbi, FocConfigAxisAbi, FocConfigBoardAbi, FocConfigBundleAbi,
        FocConfigCalibrationAbi, FocConfigCommandSourceAbi, FocConfigInverterAbi,
        FocConfigMotorAbi, FocOutput, FocRealtimeInput, FocRuntimeConfig, FocRustContextStorage,
        FocState, FocStatus, FocTelemetry, FOC_CONFIG_ABI_VERSION, FOC_FAULT_MOTION_CONTROL,
        FOC_RUST_CONTEXT_CAPACITY,
    };
    use foc_control::{
        AxisRequest, ExternalIoConfig, FeedbackMode, ProductCommandKind, RotorFeedback,
        BOARD_CAP_THREE_SHUNT_CURRENT, COMMAND_SOURCE_PERMISSION_KNOWN_MASK, CONFIG_GROUP_VERSION,
        MOTION_REFERENCE_FLAG_TRANSITION,
    };

    fn bundle(motion_enabled: bool) -> FocConfigBundleAbi {
        FocConfigBundleAbi {
            struct_size: size_of::<FocConfigBundleAbi>() as u32,
            abi_version: FOC_CONFIG_ABI_VERSION,
            bundle_revision: 9,
            generated_by_version: 1,
            board: FocConfigBoardAbi {
                version: CONFIG_GROUP_VERSION,
                board_id: 1,
                pwm_frequency_hz: 12_000,
                control_frequency_hz: 12_000,
                capability_flags: BOARD_CAP_THREE_SHUNT_CURRENT,
                adc_reference_v: 3.3,
                current_gain_a_per_count: 0.001,
                bus_voltage_v_per_count: 0.01,
            },
            motor: FocConfigMotorAbi {
                version: CONFIG_GROUP_VERSION,
                motor_id: 2,
                pole_pairs: 7,
                phase_resistance_ohm: 5.0,
                d_inductance_h: 0.001,
                q_inductance_h: 0.001,
                flux_linkage_v_s: 0.025,
                continuous_current_a: 0.8,
                maximum_speed_rad_s: 200.0,
            },
            inverter: FocConfigInverterAbi {
                version: CONFIG_GROUP_VERSION,
                inverter_id: 3,
                maximum_phase_current_a: 2.0,
                minimum_bus_voltage_v: 7.0,
                maximum_bus_voltage_v: 18.0,
                dead_time_s: 550.0e-9,
                transistor_drop_v: 0.35,
                brake_resistance_ohm: 0.0,
                maximum_duty: 0.95,
            },
            axis: FocConfigAxisAbi {
                version: CONFIG_GROUP_VERSION,
                axis_id: 0,
                feedback_mode: FeedbackMode::Sensorless as u32,
                direction: 1,
                current_kp: 3.0,
                current_ki: 100.0,
                velocity_kp: 0.02,
                velocity_ki: 0.2,
                position_kp: 1.0,
                soft_limit_min_rad: -100.0,
                soft_limit_max_rad: 100.0,
                motion_control_enabled: u32::from(motion_enabled),
                torque_ramp_rate_nm_s: if motion_enabled { 1.0 } else { 0.0 },
                velocity_ramp_rate_rad_s2: if motion_enabled { 20.0 } else { 0.0 },
                position_filter_bandwidth_rad_s: if motion_enabled { 40.0 } else { 0.0 },
                trajectory_acceleration_rad_s2: if motion_enabled { 30.0 } else { 0.0 },
                trajectory_deceleration_rad_s2: if motion_enabled { 35.0 } else { 0.0 },
            },
            app: FocConfigAppAbi {
                version: CONFIG_GROUP_VERSION,
                can_node_id: 1,
                uart_baud: 115_200,
                command_source_count: 1,
                command_sources: [
                    FocConfigCommandSourceAbi {
                        source_id: 1,
                        priority: 10,
                        permissions: COMMAND_SOURCE_PERMISSION_KNOWN_MASK,
                        lease_ms: 100,
                        command_timeout_ms: 250,
                    },
                    FocConfigCommandSourceAbi::default(),
                    FocConfigCommandSourceAbi::default(),
                    FocConfigCommandSourceAbi::default(),
                ],
            },
            external_io: ExternalIoConfig::default(),
            calibration: FocConfigCalibrationAbi {
                version: CONFIG_GROUP_VERSION,
                board_id: 1,
                motor_id: 2,
                valid_flags: 0,
                current_offset_counts: [0.0; 3],
                phase_voltage_gain: [0.0; 3],
                phase_voltage_offset_v: [0.0; 3],
                encoder_offset_rad: 0.0,
                encoder_counts_per_revolution: 0,
                hall_sequence_packed: 0,
            },
        }
    }

    fn storage() -> FocMotionContextStorage {
        FocMotionContextStorage {
            bytes: [0; FOC_MOTION_CONTEXT_CAPACITY],
        }
    }

    fn position_command(sequence: u32) -> ProductCommand {
        ProductCommand {
            sequence,
            command_kind: ProductCommandKind::Setpoint as u32,
            axis_request: AxisRequest::None as u32,
            control_mode: ControlMode::Position as u32,
            input_mode: InputMode::Passthrough as u32,
            feedback_mode: FeedbackMode::Sensorless as u32,
            position_ref_rad: 2.0,
            ..ProductCommand::default()
        }
    }

    fn realtime_request(sequence: u32, publication_sequence: u32) -> FocMotionRealtimeRequest {
        FocMotionRealtimeRequest {
            struct_size: size_of::<FocMotionRealtimeRequest>() as u32,
            version: FOC_MOTION_REALTIME_REQUEST_VERSION,
            publication_sequence,
            request_flags: FOC_MOTION_REALTIME_REQUEST_COMMAND_VALID,
            consumer_now_ms: 100,
            fault_detail: 0,
            mechanical_position_rad: 0.0,
            position_sampled_at_ms: 0,
            command: ProductCommand {
                sequence,
                created_at_ms: 90,
                valid_until_ms: 200,
                command_kind: ProductCommandKind::Setpoint as u32,
                axis_request: AxisRequest::None as u32,
                control_mode: ControlMode::Velocity as u32,
                input_mode: InputMode::Passthrough as u32,
                feedback_mode: FeedbackMode::Sensorless as u32,
                velocity_ref_rad_s: 0.0,
                ..ProductCommand::default()
            },
        }
    }

    fn controller_storage() -> FocRustContextStorage {
        FocRustContextStorage {
            bytes: [0; FOC_RUST_CONTEXT_CAPACITY],
        }
    }

    fn started_alignment_controller() -> FocRustContextStorage {
        let mut storage = controller_storage();
        let mut runtime = FocRuntimeConfig::default();
        unsafe {
            assert_eq!(crate::foc_rust_init(&mut storage), FocStatus::Ok);
            assert_eq!(
                crate::foc_rust_default_st_config(&mut runtime),
                FocStatus::Ok
            );
            assert_eq!(
                crate::foc_rust_configure(&mut storage, &runtime),
                FocStatus::Ok
            );
            assert_eq!(
                crate::foc_rust_start_realtime(&mut storage, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );
        }
        storage
    }

    fn started_closed_loop_controller() -> FocRustContextStorage {
        let mut storage = controller_storage();
        let mut runtime = FocRuntimeConfig::default();
        unsafe {
            assert_eq!(crate::foc_rust_init(&mut storage), FocStatus::Ok);
            assert_eq!(
                crate::foc_rust_default_st_config(&mut runtime),
                FocStatus::Ok
            );
            assert_eq!(
                crate::foc_rust_configure(&mut storage, &runtime),
                FocStatus::Ok
            );
            assert_eq!(
                crate::foc_rust_start_realtime(&mut storage, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );
            let controller = crate::controller_mut(&mut storage).unwrap();
            let _ = controller.startup.update(10.0, 0.0, true, 0.0, 10.0);
            let _ = controller.startup.update(1.0, 0.0, true, 0.0, 10.0);
            controller.state = FocState::ClosedLoop;
            controller.observer_feedback = RotorFeedback {
                electrical_angle_rad: 0.0,
                mechanical_speed_rad_s: 0.0,
            };
            controller.observer_reliable = true;
            controller.observer_run_reliable = true;
        }
        storage
    }

    #[test]
    fn abi_is_default_off_and_disabled_config_cannot_be_enabled() {
        let mut storage = storage();
        let disabled = bundle(false);
        assert_eq!(
            unsafe { foc_rust_motion_init(&mut storage) },
            FocMotionStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_motion_configure(&mut storage, &disabled) },
            FocMotionStatus::Disabled
        );
        assert_eq!(
            unsafe { foc_rust_motion_enable(&mut storage) },
            FocMotionStatus::NotConfigured
        );
    }

    #[test]
    fn planner_and_cascade_commit_as_one_transaction() {
        let mut storage = storage();
        let enabled = bundle(true);
        assert_eq!(
            unsafe { foc_rust_motion_init(&mut storage) },
            FocMotionStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_motion_configure(&mut storage, &enabled) },
            FocMotionStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_motion_enable(&mut storage) },
            FocMotionStatus::Ok
        );

        let command = position_command(42);
        let mut feedback = FocMotionFeedbackAbi {
            struct_size: size_of::<FocMotionFeedbackAbi>() as u32,
            version: FOC_MOTION_FEEDBACK_VERSION,
            sequence: 7,
            valid_flags: FOC_MOTION_FEEDBACK_VALID_POSITION | FOC_MOTION_FEEDBACK_VALID_VELOCITY,
            mechanical_position_rad: 1.0,
            mechanical_velocity_rad_s: 0.0,
            current_q_a: 0.0,
            reserved: 0,
        };
        let mut output = FocMotionOutputAbi::default();
        assert_eq!(
            unsafe { foc_rust_motion_step(&mut storage, &command, &feedback, &mut output) },
            FocMotionStatus::InvalidFeedback
        );
        assert_eq!(output.control_mode, ControlMode::Inactive as u32);
        assert_eq!(output.current_q_reference_a, 0.0);

        feedback.valid_flags |= FOC_MOTION_FEEDBACK_VALID_CURRENT_Q;
        assert_eq!(
            unsafe { foc_rust_motion_step(&mut storage, &command, &feedback, &mut output) },
            FocMotionStatus::Ok
        );
        assert_ne!(output.reference_flags & MOTION_REFERENCE_FLAG_TRANSITION, 0);
        assert_eq!(output.config_revision, 9);
        assert_eq!(output.source_sequence, 42);
    }

    #[test]
    fn realtime_abi_layout_divider_and_wrapping_windows_are_pinned() {
        assert_eq!(FOC_MOTION_ABI_VERSION, 0x0002_0000);
        assert_eq!(size_of::<FocMotionRealtimeRequest>(), 136);
        assert_eq!(core::mem::offset_of!(FocMotionRealtimeRequest, command), 32);

        let mut storage = storage();
        let enabled = bundle(true);
        assert_eq!(
            unsafe { foc_rust_motion_init(&mut storage) },
            FocMotionStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_motion_configure_realtime(&mut storage, &enabled, 7) },
            FocMotionStatus::InvalidConfig
        );
        assert_eq!(
            unsafe { foc_rust_motion_configure_realtime(&mut storage, &enabled, 12) },
            FocMotionStatus::Ok
        );
        let context = unsafe { context_mut(&mut storage) }.unwrap();
        let config = context.config.unwrap();
        assert!((config.planner.control_period_s - 0.001).abs() < 1.0e-9);
        assert_eq!(
            config.planner.control_period_s,
            config.cascade.control_period_s
        );
        assert_eq!(context.outer_loop_divider, 12);

        assert_eq!(
            unsafe { foc_rust_motion_enable(&mut storage) },
            FocMotionStatus::Ok
        );
        let context = unsafe { context_mut(&mut storage) }.unwrap();
        let mut output = FocMotionOutputAbi::default();
        let mut request = realtime_request(u32::MAX, u32::MAX);
        request.command.created_at_ms = u32::MAX - 5;
        request.command.valid_until_ms = 10;
        request.consumer_now_ms = 2;
        assert_eq!(
            context.prepare_realtime_request(&request, &mut output),
            Ok(())
        );
        request.publication_sequence = 0;
        request.command.sequence = 0;
        request.consumer_now_ms = 3;
        assert_eq!(
            context.prepare_realtime_request(&request, &mut output),
            Ok(())
        );

        let mut changed_under_same_publication = request;
        changed_under_same_publication.command.sequence = 1;
        assert_eq!(
            context.prepare_realtime_request(&changed_under_same_publication, &mut output),
            Err((
                FocMotionStatus::InvalidCommand,
                FOC_MOTION_DETAIL_COMMAND_SEQUENCE
            ))
        );
    }

    #[test]
    fn external_position_has_an_independent_wrapping_freshness_gate() {
        let mut storage = storage();
        let enabled = bundle(true);
        unsafe {
            assert_eq!(foc_rust_motion_init(&mut storage), FocMotionStatus::Ok);
            assert_eq!(
                foc_rust_motion_configure_realtime(&mut storage, &enabled, 12),
                FocMotionStatus::Ok
            );
            assert_eq!(foc_rust_motion_enable(&mut storage), FocMotionStatus::Ok);
        }
        let context = unsafe { context_mut(&mut storage) }.unwrap();
        let mut output = FocMotionOutputAbi::default();
        let mut request = realtime_request(1, 1);
        request.request_flags |= FOC_MOTION_REALTIME_REQUEST_POSITION_VALID;
        request.mechanical_position_rad = 0.25;
        request.position_sampled_at_ms = 90;
        assert_eq!(
            context.prepare_realtime_request(&request, &mut output),
            Ok(())
        );

        request.publication_sequence = 2;
        request.position_sampled_at_ms = 101;
        assert_eq!(
            context.prepare_realtime_request(&request, &mut output),
            Err((
                FocMotionStatus::InvalidFeedback,
                FOC_MOTION_DETAIL_COMMAND_TIME
            ))
        );
        request.position_sampled_at_ms = 0;
        request.request_flags &= !FOC_MOTION_REALTIME_REQUEST_POSITION_VALID;
        request.publication_sequence = 3;
        assert_eq!(
            context.prepare_realtime_request(&request, &mut output),
            Ok(())
        );
        request.position_sampled_at_ms = 99;
        request.publication_sequence = 4;
        assert_eq!(
            context.prepare_realtime_request(&request, &mut output),
            Err((
                FocMotionStatus::InvalidCommand,
                FOC_MOTION_DETAIL_INVALID_REQUEST
            ))
        );
    }

    #[test]
    fn combined_realtime_runs_divided_motion_then_holds_id_iq() {
        let mut controller = started_closed_loop_controller();
        let mut motion = storage();
        let enabled = bundle(true);
        unsafe {
            assert_eq!(foc_rust_motion_init(&mut motion), FocMotionStatus::Ok);
            assert_eq!(
                foc_rust_motion_configure_realtime(&mut motion, &enabled, 12),
                FocMotionStatus::Ok
            );
            assert_eq!(foc_rust_motion_enable(&mut motion), FocMotionStatus::Ok);
        }
        let feedback = crate::FocFeedback {
            phase_current_a: 0.2,
            phase_current_b: -0.1,
            phase_current_c: -0.1,
            dc_bus_voltage: 12.3,
            electrical_angle_rad: 0.0,
        };
        let mut input = FocRealtimeInput::command_model_from_legacy(feedback, 0, 1.0 / 12_000.0);
        let mut request = realtime_request(1, 1);
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        let mut motion_output = FocMotionOutputAbi::default();
        assert_eq!(
            unsafe {
                crate::foc_rust_realtime_step_with_motion(
                    &mut controller,
                    &mut motion,
                    &input,
                    &request,
                    &mut output,
                    &mut telemetry,
                    &mut motion_output,
                )
            },
            FocStatus::Ok
        );
        assert_ne!(
            motion_output.reference_flags & MOTION_REFERENCE_FLAG_TRANSITION,
            0
        );
        let held_id = motion_output.current_d_reference_a;
        let held_iq = motion_output.current_q_reference_a;

        input.control_sequence = 1;
        request.consumer_now_ms = 101;
        assert_eq!(
            unsafe {
                crate::foc_rust_realtime_step_with_motion(
                    &mut controller,
                    &mut motion,
                    &input,
                    &request,
                    &mut output,
                    &mut telemetry,
                    &mut motion_output,
                )
            },
            FocStatus::Ok
        );
        assert_eq!(motion_output.current_d_reference_a, held_id);
        assert_eq!(motion_output.current_q_reference_a, held_iq);
        assert_eq!(telemetry.id_reference_a, held_id);
        assert_eq!(telemetry.iq_reference_a, held_iq);
    }

    #[test]
    fn no_power_entry_executes_motion_while_normal_alignment_does_not() {
        let enabled = bundle(true);
        let feedback = crate::FocFeedback {
            dc_bus_voltage: 12.3,
            ..crate::FocFeedback::default()
        };
        let input = FocRealtimeInput::command_model_from_legacy(feedback, 0, 1.0 / 12_000.0);
        let request = realtime_request(1, 1);

        let mut normal_controller = started_alignment_controller();
        let mut normal_motion = storage();
        let mut normal_output = FocOutput::default();
        let mut normal_motion_output = FocMotionOutputAbi::default();
        unsafe {
            assert_eq!(
                foc_rust_motion_init(&mut normal_motion),
                FocMotionStatus::Ok
            );
            assert_eq!(
                foc_rust_motion_configure_realtime(&mut normal_motion, &enabled, 12),
                FocMotionStatus::Ok
            );
            assert_eq!(
                foc_rust_motion_enable(&mut normal_motion),
                FocMotionStatus::Ok
            );
            assert_eq!(
                crate::foc_rust_realtime_step_with_motion(
                    &mut normal_controller,
                    &mut normal_motion,
                    &input,
                    &request,
                    &mut normal_output,
                    core::ptr::null_mut(),
                    &mut normal_motion_output,
                ),
                FocStatus::Ok
            );
        }
        assert_eq!(
            normal_motion_output.control_mode,
            ControlMode::Inactive as u32
        );

        let mut probe_controller = started_alignment_controller();
        let mut probe_motion = storage();
        let mut probe_output = FocOutput::default();
        let mut probe_motion_output = FocMotionOutputAbi::default();
        unsafe {
            assert_eq!(foc_rust_motion_init(&mut probe_motion), FocMotionStatus::Ok);
            assert_eq!(
                foc_rust_motion_configure_realtime(&mut probe_motion, &enabled, 12),
                FocMotionStatus::Ok
            );
            assert_eq!(
                foc_rust_motion_enable(&mut probe_motion),
                FocMotionStatus::Ok
            );
            assert_eq!(
                crate::foc_rust_realtime_step_with_motion_no_power(
                    &mut probe_controller,
                    &mut probe_motion,
                    &input,
                    &request,
                    &mut probe_output,
                    core::ptr::null_mut(),
                    &mut probe_motion_output,
                ),
                FocStatus::Ok
            );
        }
        assert_eq!(
            probe_motion_output.control_mode,
            ControlMode::Velocity as u32
        );
        assert_ne!(
            probe_motion_output.reference_flags & MOTION_REFERENCE_FLAG_TRANSITION,
            0
        );
    }

    #[test]
    fn urgent_fault_and_stop_are_fast_tick_fail_closed_requests() {
        let mut controller = started_closed_loop_controller();
        let mut motion = storage();
        let enabled = bundle(true);
        unsafe {
            assert_eq!(foc_rust_motion_init(&mut motion), FocMotionStatus::Ok);
            assert_eq!(
                foc_rust_motion_configure_realtime(&mut motion, &enabled, 12),
                FocMotionStatus::Ok
            );
            assert_eq!(foc_rust_motion_enable(&mut motion), FocMotionStatus::Ok);
        }
        let feedback = crate::FocFeedback {
            dc_bus_voltage: 12.3,
            ..crate::FocFeedback::default()
        };
        let input = FocRealtimeInput::command_model_from_legacy(feedback, 0, 1.0 / 12_000.0);
        let mut request = realtime_request(1, 1);
        request.request_flags = FOC_MOTION_REALTIME_REQUEST_FAULT;
        request.fault_detail = 0x55AA;
        let mut output = FocOutput {
            duty_a: 0.8,
            duty_b: 0.8,
            duty_c: 0.8,
        };
        let mut motion_output = FocMotionOutputAbi::default();
        assert_eq!(
            unsafe {
                crate::foc_rust_realtime_step_with_motion(
                    &mut controller,
                    &mut motion,
                    &input,
                    &request,
                    &mut output,
                    core::ptr::null_mut(),
                    &mut motion_output,
                )
            },
            FocStatus::HardwareFault
        );
        assert_eq!(output, FocOutput::default());
        assert_eq!(motion_output.detail, 0x55AA);
        assert_ne!(
            unsafe { crate::foc_rust_fault_flags(&mut controller) } & FOC_FAULT_MOTION_CONTROL,
            0
        );

        let mut controller = started_closed_loop_controller();
        let mut motion = storage();
        unsafe {
            assert_eq!(foc_rust_motion_init(&mut motion), FocMotionStatus::Ok);
            assert_eq!(
                foc_rust_motion_configure_realtime(&mut motion, &enabled, 12),
                FocMotionStatus::Ok
            );
            assert_eq!(foc_rust_motion_enable(&mut motion), FocMotionStatus::Ok);
        }
        request.request_flags = FOC_MOTION_REALTIME_REQUEST_STOP;
        request.fault_detail = 0;
        output.duty_a = 0.8;
        assert_eq!(
            unsafe {
                crate::foc_rust_realtime_step_with_motion(
                    &mut controller,
                    &mut motion,
                    &input,
                    &request,
                    &mut output,
                    core::ptr::null_mut(),
                    &mut motion_output,
                )
            },
            FocStatus::Disabled
        );
        assert_eq!(output, FocOutput::default());
        assert_eq!(
            unsafe { crate::foc_rust_state(&mut controller) },
            FocState::Disabled
        );

        let mut controller = started_closed_loop_controller();
        let mut motion = storage();
        unsafe {
            assert_eq!(foc_rust_motion_init(&mut motion), FocMotionStatus::Ok);
            assert_eq!(
                foc_rust_motion_configure_realtime(&mut motion, &enabled, 12),
                FocMotionStatus::Ok
            );
            assert_eq!(foc_rust_motion_enable(&mut motion), FocMotionStatus::Ok);
        }
        let mut hardware_fault_input = input;
        hardware_fault_input.hardware_fault_flags = crate::FOC_REALTIME_HW_FAULT_DRIVER;
        assert_eq!(
            unsafe {
                crate::foc_rust_realtime_step_with_motion(
                    &mut controller,
                    &mut motion,
                    &hardware_fault_input,
                    &request,
                    &mut output,
                    core::ptr::null_mut(),
                    &mut motion_output,
                )
            },
            FocStatus::HardwareFault
        );
        assert_ne!(
            unsafe { crate::foc_rust_fault_flags(&mut controller) }
                & crate::FOC_FAULT_PLATFORM_INPUT,
            0
        );
    }

    #[test]
    fn rejected_request_does_not_advance_the_fast_tick_transaction() {
        let mut controller = started_closed_loop_controller();
        let mut motion = storage();
        let enabled = bundle(true);
        unsafe {
            assert_eq!(foc_rust_motion_init(&mut motion), FocMotionStatus::Ok);
            assert_eq!(
                foc_rust_motion_configure_realtime(&mut motion, &enabled, 12),
                FocMotionStatus::Ok
            );
            assert_eq!(foc_rust_motion_enable(&mut motion), FocMotionStatus::Ok);
        }
        let feedback = crate::FocFeedback {
            dc_bus_voltage: 12.3,
            ..crate::FocFeedback::default()
        };
        let input = FocRealtimeInput::command_model_from_legacy(feedback, 0, 1.0 / 12_000.0);
        let mut request = realtime_request(1, 1);
        request.consumer_now_ms = request.command.valid_until_ms;
        let mut output = FocOutput::default();
        let mut motion_output = FocMotionOutputAbi::default();
        assert_eq!(
            unsafe {
                crate::foc_rust_realtime_step_with_motion(
                    &mut controller,
                    &mut motion,
                    &input,
                    &request,
                    &mut output,
                    core::ptr::null_mut(),
                    &mut motion_output,
                )
            },
            FocStatus::HardwareFault
        );
        let controller_state = unsafe { crate::controller_mut(&mut controller) }.unwrap();
        assert_eq!(controller_state.last_control_sequence, u32::MAX);
        assert_eq!(controller_state.fault_flags, FOC_FAULT_MOTION_CONTROL);
        assert_eq!(motion_output.detail, FOC_MOTION_DETAIL_COMMAND_TIME);
    }
}
