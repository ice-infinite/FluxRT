#ifndef FOC_PLATFORM_AS5600_ALIGNMENT_CANDIDATE_H
#define FOC_PLATFORM_AS5600_ALIGNMENT_CANDIDATE_H

#include "foc_as5600_alignment_trial.h"
#include "foc_rust_bridge.h"

/* Dedicated candidate authority. Generic foc_platform_control_start() remains
 * denied in this build. The ADC ISR owns the hard stop. */
foc_status_t foc_platform_as5600_alignment_trial_start(
    const foc_runtime_config_t *runtime_config,
    float startup_target_speed_rpm);
foc_status_t foc_platform_as5600_alignment_trial_get_status(
    foc_as5600_alignment_trial_status_t *status);

#endif /* FOC_PLATFORM_AS5600_ALIGNMENT_CANDIDATE_H */
