#ifndef FOC_PLATFORM_MOTION_CANDIDATE_H
#define FOC_PLATFORM_MOTION_CANDIDATE_H

/*
 * FluxRT P4.2E1 target-side motion candidate control plane.
 *
 * The STM32 platform owns the realtime motion context and dispatcher.  A task
 * may configure and publish through this API, but it never receives a pointer
 * to ISR-owned Rust state.  The implementation is compiled only for the
 * Diagnostic motion candidate; every other profile returns NOT_CONFIGURED.
 */

#include <stdint.h>

#include "foc_motion_dispatcher.h"
#include "foc_motion_probe.h"
#include "foc_motion_torque_trial.h"
#include "foc_types.h"

/*
 * Stopped-state handoff.  This validates the ABI, initializes/configures the
 * ISR-owned context, initializes the double-slot dispatcher, and finally
 * publishes the runtime route under an ADC-IRQ-only critical section.
 */
foc_status_t foc_platform_motion_candidate_configure(
    const foc_config_bundle_t *config,
    const foc_runtime_config_t *active_realtime_config,
    uint32_t active_bundle_revision,
    uint32_t outer_loop_divider);

/* Single management-task publisher.  Timestamps use the RT-Thread wrapping
 * millisecond domain also sampled by the ADC ISR consumer. */
foc_status_t foc_platform_motion_candidate_publish(
    const foc_product_command_t *command,
    uint32_t position_valid,
    float mechanical_position_rad,
    uint32_t position_sampled_at_ms);

/* Normal stop is distinct from a sticky motion fault.  Both requests are
 * consumed once by the ADC ISR; any downstream failure is fail-closed. */
foc_status_t foc_platform_motion_candidate_request_stop(void);
foc_status_t foc_platform_motion_candidate_request_fault(uint32_t fault_detail);

/* Stopped-state ownership return.  The route is removed before the Rust motion
 * context and dispatcher are reset.  It never arms or restarts the motor. */
foc_status_t foc_platform_motion_candidate_shutdown(void);
uint32_t foc_platform_motion_candidate_is_enabled(void);

/*
 * P4.2E1B Diagnostic-only no-power timing entry.  The start call requires the
 * candidate route and one command to have been published, but it never arms
 * TIM1: Gate, MOE and CC1/2/3 remain off while the ISR executes the complete
 * Rust motion + FOC path and writes CCR preload values only.
 */
foc_status_t foc_platform_motion_candidate_probe_start(
    uint32_t requested_ticks,
    float target_speed_rpm,
    uint32_t execute_motion);
foc_status_t foc_platform_motion_candidate_probe_get_status(
    foc_motion_probe_status_t *status);
foc_status_t foc_platform_motion_candidate_probe_finish(void);
foc_status_t foc_platform_motion_candidate_probe_inject_once(
    foc_motion_probe_injection_point_t point);

/* Fixed-envelope powered Torque trial. This is the only candidate API allowed
 * to cross the normal no-power arm interlock. Duration and limits are constants
 * in foc_motion_torque_trial.h, not caller-controlled values. */
foc_status_t foc_platform_motion_candidate_torque_trial_start(
    const foc_product_command_t *command,
    const foc_runtime_config_t *active_realtime_config,
    float startup_target_speed_rpm);
foc_status_t foc_platform_motion_candidate_torque_trial_get_status(
    foc_motion_torque_trial_status_t *status);

#endif /* FOC_PLATFORM_MOTION_CANDIDATE_H */
