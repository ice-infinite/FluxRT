//! Device-side configuration transaction state machine.
//!
//! The transaction is intentionally board independent and owns no protocol,
//! Flash, RTOS, or realtime-controller implementation.  It turns external
//! parameter edits into explicit apply and persistence plans.  Platform and
//! application adapters must execute those plans while the Axis remains
//! disabled, then confirm the result back to this state machine.

use crate::{
    decode_config_slot, prepare_config_save, AppConfig, AxisConfig, AxisState, BoardConfig,
    CalibrationData, ConfigApproval, ConfigBundle, ConfigRecordError, ConfigSlot,
    ConfigValidationError, ConfigWriteError, ExternalIoConfig, InverterConfig, MotorConfig,
    PreparedConfigWrite,
};

pub const CONFIG_GROUP_BOARD: u32 = 1 << 0;
pub const CONFIG_GROUP_MOTOR: u32 = 1 << 1;
pub const CONFIG_GROUP_INVERTER: u32 = 1 << 2;
pub const CONFIG_GROUP_AXIS: u32 = 1 << 3;
pub const CONFIG_GROUP_APP: u32 = 1 << 4;
pub const CONFIG_GROUP_CALIBRATION: u32 = 1 << 5;
pub const CONFIG_GROUP_EXTERNAL_IO: u32 = 1 << 6;
pub const CONFIG_GROUP_KNOWN_MASK: u32 = CONFIG_GROUP_BOARD
    | CONFIG_GROUP_MOTOR
    | CONFIG_GROUP_INVERTER
    | CONFIG_GROUP_AXIS
    | CONFIG_GROUP_APP
    | CONFIG_GROUP_CALIBRATION
    | CONFIG_GROUP_EXTERNAL_IO;

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigTransactionState {
    Idle = 0,
    Editing = 1,
    Validated = 2,
    ApplyPrepared = 3,
    VolatileApplied = 4,
    CommitPrepared = 5,
    StorageFault = 6,
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigApplyClass {
    /// Command-source and management policy only; no realtime object changes.
    ManagementOnly = 0,
    /// Rebuild board-independent motor/Axis control state while outputs are off.
    AxisRestart = 1,
    /// Reconfigure platform timing, sensing, protection, or calibration.
    PlatformRestart = 2,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConfigTransactionToken(u32);

impl ConfigTransactionToken {
    pub const fn raw(self) -> u32 {
        self.0
    }

    pub const fn from_raw(raw: u32) -> Option<Self> {
        if raw == 0 {
            None
        } else {
            Some(Self(raw))
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConfigApplyGuard {
    pub axis_state: AxisState,
    pub drive_active: bool,
    pub active_fault_flags: u32,
}

impl ConfigApplyGuard {
    pub const fn disabled() -> Self {
        Self {
            axis_state: AxisState::Disabled,
            drive_active: false,
            active_fault_flags: 0,
        }
    }

    pub const fn allows_apply(self) -> bool {
        matches!(self.axis_state, AxisState::Disabled)
            && !self.drive_active
            && self.active_fault_flags == 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConfigApplyPlan {
    pub token: ConfigTransactionToken,
    pub from_revision: u32,
    pub to_revision: u32,
    pub changed_groups: u32,
    pub apply_class: ConfigApplyClass,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConfigTransactionStatus {
    pub state: ConfigTransactionState,
    pub token: Option<ConfigTransactionToken>,
    pub active_revision: u32,
    pub pending_revision: Option<u32>,
    pub changed_groups: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
// The management-plane patch stays allocation-free and Copy on the MCU. The
// persistent ExternalIo group is intentionally carried inline.
#[allow(clippy::large_enum_variant)]
pub enum ConfigPatch {
    Board(BoardConfig),
    Motor(MotorConfig),
    Inverter(InverterConfig),
    Axis(AxisConfig),
    App(AppConfig),
    ExternalIo(ExternalIoConfig),
    Calibration(CalibrationData),
}

impl ConfigPatch {
    fn apply(self, config: &mut ConfigBundle) {
        match self {
            Self::Board(value) => config.board = value,
            Self::Motor(value) => config.motor = value,
            Self::Inverter(value) => config.inverter = value,
            Self::Axis(value) => config.axis = value,
            Self::App(value) => config.app = value,
            Self::ExternalIo(value) => config.external_io = value,
            Self::Calibration(value) => config.calibration = value,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigTransactionError {
    InvalidInitial(ConfigValidationError),
    HardwareIdentityMismatch,
    Busy,
    NoTransaction,
    StaleToken,
    InvalidState,
    EmptyTransaction,
    UnsafeToApply,
    Validation(ConfigValidationError),
    StoragePlan(ConfigWriteError),
    StorageRecord(ConfigRecordError),
    StorageVerification,
}

/// Board-independent owner of pending, active, and last-confirmed configuration.
///
/// This type is intended for one management-thread owner.  It is never called
/// from the realtime ISR and deliberately performs no synchronization.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConfigTransactionManager {
    active: ConfigBundle,
    baseline: ConfigBundle,
    pending: ConfigBundle,
    expected_board_id: u32,
    expected_motor_id: u32,
    state: ConfigTransactionState,
    token: Option<ConfigTransactionToken>,
    next_token: u32,
    changed_groups: u32,
    prepared_slot: usize,
    prepared_sequence: u32,
}

impl ConfigTransactionManager {
    pub fn new(
        active: ConfigBundle,
        expected_board_id: u32,
        expected_motor_id: u32,
    ) -> Result<Self, ConfigTransactionError> {
        active
            .validate()
            .map_err(ConfigTransactionError::InvalidInitial)?;
        if active.board.board_id != expected_board_id || active.motor.motor_id != expected_motor_id
        {
            return Err(ConfigTransactionError::HardwareIdentityMismatch);
        }
        Ok(Self {
            active,
            baseline: active,
            pending: active,
            expected_board_id,
            expected_motor_id,
            state: ConfigTransactionState::Idle,
            token: None,
            next_token: 0,
            changed_groups: 0,
            prepared_slot: 0,
            prepared_sequence: 0,
        })
    }

    pub const fn active(&self) -> &ConfigBundle {
        &self.active
    }

    pub fn pending(
        &self,
        token: ConfigTransactionToken,
    ) -> Result<&ConfigBundle, ConfigTransactionError> {
        self.check_token(token)?;
        Ok(&self.pending)
    }

    pub const fn status(&self) -> ConfigTransactionStatus {
        ConfigTransactionStatus {
            state: self.state,
            token: self.token,
            active_revision: self.active.bundle_revision,
            pending_revision: if self.token.is_some() {
                Some(self.pending.bundle_revision)
            } else {
                None
            },
            changed_groups: self.changed_groups,
        }
    }

    pub fn begin(&mut self) -> Result<ConfigTransactionToken, ConfigTransactionError> {
        if self.state != ConfigTransactionState::Idle {
            return Err(ConfigTransactionError::Busy);
        }
        self.next_token = self.next_token.wrapping_add(1);
        if self.next_token == 0 {
            self.next_token = 1;
        }
        let token = ConfigTransactionToken(self.next_token);
        self.baseline = self.active;
        self.pending = self.active;
        self.pending.bundle_revision = next_nonzero_revision(self.active.bundle_revision);
        self.changed_groups = 0;
        self.prepared_slot = 0;
        self.prepared_sequence = 0;
        self.token = Some(token);
        self.state = ConfigTransactionState::Editing;
        Ok(token)
    }

    pub fn set(
        &mut self,
        token: ConfigTransactionToken,
        patch: ConfigPatch,
    ) -> Result<(), ConfigTransactionError> {
        self.check_token(token)?;
        if !matches!(
            self.state,
            ConfigTransactionState::Editing | ConfigTransactionState::Validated
        ) {
            return Err(ConfigTransactionError::InvalidState);
        }
        patch.apply(&mut self.pending);
        self.changed_groups = changed_group_mask(&self.baseline, &self.pending);
        self.state = ConfigTransactionState::Editing;
        Ok(())
    }

    pub fn validate(
        &mut self,
        token: ConfigTransactionToken,
    ) -> Result<(), ConfigTransactionError> {
        self.check_token(token)?;
        if self.state != ConfigTransactionState::Editing {
            return Err(ConfigTransactionError::InvalidState);
        }
        if self.changed_groups == 0 {
            return Err(ConfigTransactionError::EmptyTransaction);
        }
        self.pending
            .validate()
            .map_err(ConfigTransactionError::Validation)?;
        self.state = ConfigTransactionState::Validated;
        Ok(())
    }

    /// Freeze a volatile-apply plan.  The caller must apply all changed groups
    /// through the correct management/platform adapters and then call
    /// [`Self::confirm_volatile_apply`].
    pub fn prepare_volatile_apply(
        &mut self,
        token: ConfigTransactionToken,
        guard: ConfigApplyGuard,
    ) -> Result<ConfigApplyPlan, ConfigTransactionError> {
        self.check_token(token)?;
        if self.state != ConfigTransactionState::Validated {
            return Err(ConfigTransactionError::InvalidState);
        }
        self.check_guard(guard)?;
        self.pending
            .validate_for_arm(self.expected_board_id, self.expected_motor_id)
            .map_err(ConfigTransactionError::Validation)?;
        let plan = ConfigApplyPlan {
            token,
            from_revision: self.active.bundle_revision,
            to_revision: self.pending.bundle_revision,
            changed_groups: self.changed_groups,
            apply_class: classify_apply(self.changed_groups),
        };
        self.state = ConfigTransactionState::ApplyPrepared;
        Ok(plan)
    }

    pub fn confirm_volatile_apply(
        &mut self,
        token: ConfigTransactionToken,
        guard: ConfigApplyGuard,
        plan: ConfigApplyPlan,
    ) -> Result<(), ConfigTransactionError> {
        self.check_token(token)?;
        if self.state != ConfigTransactionState::ApplyPrepared {
            return Err(ConfigTransactionError::InvalidState);
        }
        self.check_guard(guard)?;
        if plan.token != token
            || plan.from_revision != self.active.bundle_revision
            || plan.to_revision != self.pending.bundle_revision
            || plan.changed_groups != self.changed_groups
            || plan.apply_class != classify_apply(self.changed_groups)
        {
            return Err(ConfigTransactionError::StaleToken);
        }
        self.active = self.pending;
        self.state = ConfigTransactionState::VolatileApplied;
        Ok(())
    }

    pub fn cancel_volatile_apply(
        &mut self,
        token: ConfigTransactionToken,
    ) -> Result<(), ConfigTransactionError> {
        self.check_token(token)?;
        if self.state != ConfigTransactionState::ApplyPrepared {
            return Err(ConfigTransactionError::InvalidState);
        }
        self.state = ConfigTransactionState::Validated;
        Ok(())
    }

    /// Restore the pre-transaction active configuration.  A volatile apply may
    /// only be rolled back while outputs remain off.  A prepared Flash commit
    /// cannot be cancelled because the caller may already have programmed its
    /// final commit marker; it must be confirmed or recovered at startup.
    pub fn rollback(
        &mut self,
        token: ConfigTransactionToken,
        guard: ConfigApplyGuard,
    ) -> Result<(), ConfigTransactionError> {
        self.check_token(token)?;
        match self.state {
            ConfigTransactionState::Editing | ConfigTransactionState::Validated => {
                self.finish_transaction(self.baseline);
                Ok(())
            }
            ConfigTransactionState::ApplyPrepared => Err(ConfigTransactionError::InvalidState),
            ConfigTransactionState::VolatileApplied => {
                self.check_guard(guard)?;
                self.finish_transaction(self.baseline);
                Ok(())
            }
            _ => Err(ConfigTransactionError::InvalidState),
        }
    }

    pub fn prepare_commit(
        &mut self,
        token: ConfigTransactionToken,
        guard: ConfigApplyGuard,
        slots: &[ConfigSlot; 2],
    ) -> Result<PreparedConfigWrite, ConfigTransactionError> {
        self.check_token(token)?;
        if self.state != ConfigTransactionState::VolatileApplied {
            return Err(ConfigTransactionError::InvalidState);
        }
        self.check_guard(guard)?;
        let write = prepare_config_save(slots, self.active, ConfigApproval::Approved)
            .map_err(ConfigTransactionError::StoragePlan)?;
        self.prepared_slot = write.target_slot;
        self.prepared_sequence = write.sequence;
        self.state = ConfigTransactionState::CommitPrepared;
        Ok(write)
    }

    /// Confirm the read-back of the slot selected by [`Self::prepare_commit`].
    /// A bad read-back preserves the currently applied volatile configuration
    /// but locks the manager in `StorageFault`.  This avoids claiming that the
    /// external controller was rolled back when only this pure state machine
    /// observed the failure. Startup recovery must rescan both slots before
    /// another transaction is accepted or the Axis may arm again.
    pub fn confirm_commit(
        &mut self,
        token: ConfigTransactionToken,
        guard: ConfigApplyGuard,
        stored_slot_index: usize,
        stored_slot: &ConfigSlot,
    ) -> Result<(), ConfigTransactionError> {
        self.check_token(token)?;
        if self.state != ConfigTransactionState::CommitPrepared {
            return Err(ConfigTransactionError::InvalidState);
        }
        self.check_guard(guard)?;
        let decoded = match decode_config_slot(stored_slot) {
            Ok(decoded) => decoded,
            Err(error) => {
                self.enter_storage_fault();
                return Err(ConfigTransactionError::StorageRecord(error));
            }
        };
        if stored_slot_index != self.prepared_slot
            || decoded.sequence != self.prepared_sequence
            || decoded.approval != ConfigApproval::Approved
            || decoded.config != self.active
        {
            self.enter_storage_fault();
            return Err(ConfigTransactionError::StorageVerification);
        }
        let committed = self.active;
        self.finish_transaction(committed);
        Ok(())
    }

    /// Return whether the current active bundle may be offered to the existing
    /// arm gate.  A transaction in progress or an unresolved storage fault is
    /// never armable.
    pub fn validate_active_for_arm(&self) -> Result<(), ConfigTransactionError> {
        if self.state != ConfigTransactionState::Idle {
            return Err(ConfigTransactionError::Busy);
        }
        self.active
            .validate_for_arm(self.expected_board_id, self.expected_motor_id)
            .map_err(ConfigTransactionError::Validation)
    }

    fn check_token(&self, token: ConfigTransactionToken) -> Result<(), ConfigTransactionError> {
        match self.token {
            None => Err(ConfigTransactionError::NoTransaction),
            Some(current) if current != token => Err(ConfigTransactionError::StaleToken),
            Some(_) => Ok(()),
        }
    }

    fn check_guard(&self, guard: ConfigApplyGuard) -> Result<(), ConfigTransactionError> {
        if guard.allows_apply() {
            Ok(())
        } else {
            Err(ConfigTransactionError::UnsafeToApply)
        }
    }

    fn finish_transaction(&mut self, active: ConfigBundle) {
        self.active = active;
        self.baseline = active;
        self.pending = active;
        self.changed_groups = 0;
        self.prepared_slot = 0;
        self.prepared_sequence = 0;
        self.token = None;
        self.state = ConfigTransactionState::Idle;
    }

    fn enter_storage_fault(&mut self) {
        self.pending = self.active;
        self.changed_groups = 0;
        self.prepared_slot = 0;
        self.prepared_sequence = 0;
        self.token = None;
        self.state = ConfigTransactionState::StorageFault;
    }
}

const fn classify_apply(changed_groups: u32) -> ConfigApplyClass {
    if changed_groups & (CONFIG_GROUP_BOARD | CONFIG_GROUP_INVERTER | CONFIG_GROUP_CALIBRATION) != 0
    {
        ConfigApplyClass::PlatformRestart
    } else if changed_groups & (CONFIG_GROUP_MOTOR | CONFIG_GROUP_AXIS) != 0 {
        ConfigApplyClass::AxisRestart
    } else {
        ConfigApplyClass::ManagementOnly
    }
}

fn changed_group_mask(baseline: &ConfigBundle, pending: &ConfigBundle) -> u32 {
    let mut mask = 0;
    if baseline.board != pending.board {
        mask |= CONFIG_GROUP_BOARD;
    }
    if baseline.motor != pending.motor {
        mask |= CONFIG_GROUP_MOTOR;
    }
    if baseline.inverter != pending.inverter {
        mask |= CONFIG_GROUP_INVERTER;
    }
    if baseline.axis != pending.axis {
        mask |= CONFIG_GROUP_AXIS;
    }
    if baseline.app != pending.app {
        mask |= CONFIG_GROUP_APP;
    }
    if baseline.external_io != pending.external_io {
        mask |= CONFIG_GROUP_EXTERNAL_IO;
    }
    if baseline.calibration != pending.calibration {
        mask |= CONFIG_GROUP_CALIBRATION;
    }
    mask
}

const fn next_nonzero_revision(revision: u32) -> u32 {
    let next = revision.wrapping_add(1);
    if next == 0 {
        1
    } else {
        next
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CommandSourceConfig, FeedbackMode, BOARD_CAP_ENCODER, BOARD_CAP_HALL,
        BOARD_CAP_PHASE_VOLTAGE_SENSE, BOARD_CAP_THREE_SHUNT_CURRENT,
        COMMAND_SOURCE_PERMISSION_KNOWN_MASK, CONFIG_GROUP_VERSION, CONFIG_MAX_COMMAND_SOURCES,
    };

    const BOARD_ID: u32 = 0x4311_6001;
    const MOTOR_ID: u32 = 0x2804_1007;

    fn bundle(revision: u32) -> ConfigBundle {
        ConfigBundle {
            bundle_revision: revision,
            generated_by_version: 1,
            board: BoardConfig {
                version: CONFIG_GROUP_VERSION,
                board_id: BOARD_ID,
                pwm_frequency_hz: 12_000,
                control_frequency_hz: 12_000,
                capability_flags: BOARD_CAP_THREE_SHUNT_CURRENT
                    | BOARD_CAP_PHASE_VOLTAGE_SENSE
                    | BOARD_CAP_ENCODER
                    | BOARD_CAP_HALL,
                adc_reference_v: 3.3,
                current_gain_a_per_count: 0.001,
                bus_voltage_v_per_count: 0.012,
            },
            motor: MotorConfig {
                version: CONFIG_GROUP_VERSION,
                motor_id: MOTOR_ID,
                pole_pairs: 7,
                phase_resistance_ohm: 4.9667,
                d_inductance_h: 0.0012,
                q_inductance_h: 0.0013,
                flux_linkage_v_s: 0.025,
                continuous_current_a: 0.8,
                maximum_speed_rad_s: 200.0,
            },
            inverter: InverterConfig {
                version: CONFIG_GROUP_VERSION,
                inverter_id: 0x1600_0001,
                maximum_phase_current_a: 2.0,
                minimum_bus_voltage_v: 7.0,
                maximum_bus_voltage_v: 18.0,
                dead_time_s: 550.0e-9,
                transistor_drop_v: 0.35,
                brake_resistance_ohm: 0.0,
                maximum_duty: 0.95,
            },
            axis: AxisConfig {
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
                motion_control_enabled: 0,
                torque_ramp_rate_nm_s: 0.0,
                velocity_ramp_rate_rad_s2: 0.0,
                position_filter_bandwidth_rad_s: 0.0,
                trajectory_acceleration_rad_s2: 0.0,
                trajectory_deceleration_rad_s2: 0.0,
            },
            app: AppConfig {
                version: CONFIG_GROUP_VERSION,
                can_node_id: 1,
                uart_baud: 115_200,
                command_source_count: 1,
                command_sources: [
                    CommandSourceConfig {
                        source_id: 1,
                        priority: 10,
                        permissions: COMMAND_SOURCE_PERMISSION_KNOWN_MASK,
                        lease_ms: 100,
                        command_timeout_ms: 250,
                    },
                    CommandSourceConfig::default(),
                    CommandSourceConfig::default(),
                    CommandSourceConfig::default(),
                ],
            },
            external_io: ExternalIoConfig::default(),
            calibration: CalibrationData {
                version: CONFIG_GROUP_VERSION,
                board_id: BOARD_ID,
                motor_id: MOTOR_ID,
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

    fn approved_slot(config: ConfigBundle) -> [ConfigSlot; 2] {
        let mut slots = [ConfigSlot::erased(), ConfigSlot::erased()];
        let write = prepare_config_save(&slots, config, ConfigApproval::Approved).unwrap();
        write.apply_complete(&mut slots);
        slots
    }

    #[test]
    fn fake_client_can_apply_and_commit_motor_parameters() {
        let original = bundle(10);
        let mut slots = approved_slot(original);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let mut motor = original.motor;
        motor.phase_resistance_ohm = 5.1;
        manager.set(token, ConfigPatch::Motor(motor)).unwrap();
        manager.validate(token).unwrap();

        let plan = manager
            .prepare_volatile_apply(token, ConfigApplyGuard::disabled())
            .unwrap();
        assert_eq!(plan.apply_class, ConfigApplyClass::AxisRestart);
        assert_eq!(plan.changed_groups, CONFIG_GROUP_MOTOR);
        assert_eq!(manager.active(), &original);
        assert_eq!(manager.pending(token).unwrap().motor, motor);

        manager
            .confirm_volatile_apply(token, ConfigApplyGuard::disabled(), plan)
            .unwrap();
        assert_eq!(manager.active().motor, motor);
        let write = manager
            .prepare_commit(token, ConfigApplyGuard::disabled(), &slots)
            .unwrap();
        let target = write.target_slot;
        write.apply_complete(&mut slots);
        manager
            .confirm_commit(token, ConfigApplyGuard::disabled(), target, &slots[target])
            .unwrap();
        assert_eq!(manager.status().state, ConfigTransactionState::Idle);
        assert_eq!(manager.active().motor.phase_resistance_ohm, 5.1);
        assert_eq!(manager.active().bundle_revision, 11);
        assert_eq!(manager.validate_active_for_arm(), Ok(()));
    }

    #[test]
    fn motion_configuration_change_is_an_axis_restart_transaction() {
        let original = bundle(10);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let mut axis = original.axis;
        axis.motion_control_enabled = 1;
        axis.torque_ramp_rate_nm_s = 1.0;
        axis.velocity_ramp_rate_rad_s2 = 20.0;
        axis.position_filter_bandwidth_rad_s = 40.0;
        axis.trajectory_acceleration_rad_s2 = 30.0;
        axis.trajectory_deceleration_rad_s2 = 35.0;
        manager.set(token, ConfigPatch::Axis(axis)).unwrap();
        manager.validate(token).unwrap();
        let plan = manager
            .prepare_volatile_apply(token, ConfigApplyGuard::disabled())
            .unwrap();
        assert_eq!(plan.changed_groups, CONFIG_GROUP_AXIS);
        assert_eq!(plan.apply_class, ConfigApplyClass::AxisRestart);
    }

    #[test]
    fn invalid_parameter_and_stale_token_never_change_active() {
        let original = bundle(1);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let stale = ConfigTransactionToken::from_raw(token.raw().wrapping_add(1)).unwrap();
        assert_eq!(
            manager.set(stale, ConfigPatch::Motor(original.motor)),
            Err(ConfigTransactionError::StaleToken)
        );
        let mut motor = original.motor;
        motor.phase_resistance_ohm = 0.0;
        manager.set(token, ConfigPatch::Motor(motor)).unwrap();
        assert_eq!(
            manager.validate(token),
            Err(ConfigTransactionError::Validation(
                ConfigValidationError::MotorParameters
            ))
        );
        assert_eq!(manager.active(), &original);
    }

    #[test]
    fn apply_requires_disabled_fault_free_outputs_off_guard() {
        let original = bundle(1);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let mut axis = original.axis;
        axis.velocity_kp = 0.03;
        manager.set(token, ConfigPatch::Axis(axis)).unwrap();
        manager.validate(token).unwrap();
        for guard in [
            ConfigApplyGuard {
                axis_state: AxisState::ClosedLoop,
                drive_active: true,
                active_fault_flags: 0,
            },
            ConfigApplyGuard {
                axis_state: AxisState::Disabled,
                drive_active: true,
                active_fault_flags: 0,
            },
            ConfigApplyGuard {
                axis_state: AxisState::Disabled,
                drive_active: false,
                active_fault_flags: 1,
            },
        ] {
            assert_eq!(
                manager.prepare_volatile_apply(token, guard),
                Err(ConfigTransactionError::UnsafeToApply)
            );
        }
        assert_eq!(manager.active(), &original);
    }

    #[test]
    fn volatile_rollback_restores_pre_transaction_bundle() {
        let original = bundle(7);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let mut app = original.app;
        app.uart_baud = 921_600;
        manager.set(token, ConfigPatch::App(app)).unwrap();
        manager.validate(token).unwrap();
        let plan = manager
            .prepare_volatile_apply(token, ConfigApplyGuard::disabled())
            .unwrap();
        assert_eq!(plan.apply_class, ConfigApplyClass::ManagementOnly);
        manager
            .confirm_volatile_apply(token, ConfigApplyGuard::disabled(), plan)
            .unwrap();
        assert_eq!(manager.active().app.uart_baud, 921_600);
        manager
            .rollback(token, ConfigApplyGuard::disabled())
            .unwrap();
        assert_eq!(manager.active(), &original);
        assert_eq!(manager.status().state, ConfigTransactionState::Idle);
    }

    #[test]
    fn failed_commit_readback_preserves_volatile_active_and_locks_transactions() {
        let original = bundle(3);
        let slots = approved_slot(original);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let mut board = original.board;
        board.adc_reference_v = 3.29;
        manager.set(token, ConfigPatch::Board(board)).unwrap();
        manager.validate(token).unwrap();
        let plan = manager
            .prepare_volatile_apply(token, ConfigApplyGuard::disabled())
            .unwrap();
        assert_eq!(plan.apply_class, ConfigApplyClass::PlatformRestart);
        manager
            .confirm_volatile_apply(token, ConfigApplyGuard::disabled(), plan)
            .unwrap();
        let write = manager
            .prepare_commit(token, ConfigApplyGuard::disabled(), &slots)
            .unwrap();
        let erased = ConfigSlot::erased();
        assert_eq!(
            manager.confirm_commit(
                token,
                ConfigApplyGuard::disabled(),
                write.target_slot,
                &erased,
            ),
            Err(ConfigTransactionError::StorageRecord(
                ConfigRecordError::Erased
            ))
        );
        assert_eq!(manager.active().board, board);
        assert_eq!(manager.status().state, ConfigTransactionState::StorageFault);
        assert_eq!(manager.begin(), Err(ConfigTransactionError::Busy));
    }

    #[test]
    fn cancelled_apply_plan_can_be_revalidated_or_rolled_back() {
        let original = bundle(1);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let mut inverter = original.inverter;
        inverter.transistor_drop_v = 0.4;
        manager.set(token, ConfigPatch::Inverter(inverter)).unwrap();
        manager.validate(token).unwrap();
        manager
            .prepare_volatile_apply(token, ConfigApplyGuard::disabled())
            .unwrap();
        manager.cancel_volatile_apply(token).unwrap();
        assert_eq!(manager.status().state, ConfigTransactionState::Validated);
        manager
            .rollback(token, ConfigApplyGuard::disabled())
            .unwrap();
        assert_eq!(manager.active(), &original);
    }

    #[test]
    fn command_source_array_size_stays_fixed() {
        assert_eq!(CONFIG_MAX_COMMAND_SOURCES, 4);
    }

    #[test]
    fn setting_a_group_back_to_baseline_is_an_empty_transaction() {
        let original = bundle(1);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let mut motor = original.motor;
        motor.phase_resistance_ohm = 5.1;
        manager.set(token, ConfigPatch::Motor(motor)).unwrap();
        manager
            .set(token, ConfigPatch::Motor(original.motor))
            .unwrap();
        assert_eq!(manager.status().changed_groups, 0);
        assert_eq!(
            manager.validate(token),
            Err(ConfigTransactionError::EmptyTransaction)
        );
    }

    #[test]
    fn external_io_change_is_a_management_transaction() {
        let original = bundle(10);
        let mut manager = ConfigTransactionManager::new(original, BOARD_ID, MOTOR_ID).unwrap();
        let token = manager.begin().unwrap();
        let mut external = original.external_io;
        external.revision = 2;
        manager
            .set(token, ConfigPatch::ExternalIo(external))
            .unwrap();
        manager.validate(token).unwrap();
        let plan = manager
            .prepare_volatile_apply(token, ConfigApplyGuard::disabled())
            .unwrap();
        assert_eq!(plan.changed_groups, CONFIG_GROUP_EXTERNAL_IO);
        assert_eq!(plan.apply_class, ConfigApplyClass::ManagementOnly);
    }
}
