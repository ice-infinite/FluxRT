#ifndef FOC_PLATFORM_ADVANCED_CANDIDATE_H
#define FOC_PLATFORM_ADVANCED_CANDIDATE_H

#include <stdint.h>

#include "foc_advanced_bridge.h"
#include "foc_advanced_power_trial.h"
#include "foc_advanced_probe.h"
#include "foc_types.h"

/* Advanced-Lab only.  Configuration, start and finish execute while stopped;
 * the ADC ISR owns the Rust controller between start and finish. */
foc_status_t foc_platform_advanced_candidate_probe_start(
    const foc_advanced_runtime_config_t *config,
    const foc_advanced_probe_input_t *probe_input,
    uint32_t requested_ticks,
    float nominal_bus_voltage_v,
    float target_speed_rpm);
foc_status_t foc_platform_advanced_candidate_probe_get_status(
    foc_advanced_probe_status_t *status,
    foc_advanced_telemetry_t *telemetry);
foc_status_t foc_platform_advanced_candidate_probe_finish(void);

/* P5.4A fixed-envelope powered trial.  The caller selects only the audited
 * Basic/decoupling mode; target speed and the 100 ms active window are compile-
 * time constants owned by foc_advanced_power_trial.  Generic foc_start is
 * rejected by an Advanced-Lab image. */
foc_status_t foc_platform_advanced_candidate_power_trial_start(
    foc_advanced_power_trial_mode_t mode,
    const foc_runtime_config_t *active_realtime_config);
foc_status_t foc_platform_advanced_candidate_power_trial_get_status(
    foc_advanced_power_trial_status_t *status,
    foc_advanced_telemetry_t *telemetry,
    foc_advanced_power_trial_snapshot_t *snapshot);
foc_status_t foc_platform_advanced_candidate_power_trial_finish(
    const foc_runtime_config_t *restore_realtime_config);

#endif /* FOC_PLATFORM_ADVANCED_CANDIDATE_H */
