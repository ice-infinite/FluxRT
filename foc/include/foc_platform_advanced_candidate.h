#ifndef FOC_PLATFORM_ADVANCED_CANDIDATE_H
#define FOC_PLATFORM_ADVANCED_CANDIDATE_H

#include <stdint.h>

#include "foc_advanced_bridge.h"
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

#endif /* FOC_PLATFORM_ADVANCED_CANDIDATE_H */
