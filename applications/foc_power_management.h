#ifndef FOC_POWER_MANAGEMENT_H
#define FOC_POWER_MANAGEMENT_H

/* Default-off application owner for the independent power supervisor ABI. */

#include <stdint.h>

#include "foc_config_bridge.h"
#include "foc_power_bridge.h"

#define FOC_POWER_MANAGEMENT_VERSION (1UL)

typedef uint32_t foc_power_management_result_t;
enum
{
    FOC_POWER_MANAGEMENT_OK = 0,
    FOC_POWER_MANAGEMENT_INVALID_ARGUMENT = 1,
    FOC_POWER_MANAGEMENT_ABI_ERROR = 2,
    FOC_POWER_MANAGEMENT_UNSAFE_STATE = 3,
    FOC_POWER_MANAGEMENT_POLICY_ERROR = 4,
    FOC_POWER_MANAGEMENT_SHUTDOWN = 5,
};

typedef void (*foc_power_management_force_safe_fn)(void *context);

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_power_management_force_safe_fn force_safe;
    void *context;
} foc_power_management_ops_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t configured;
    foc_power_status_t last_power_status;
    foc_power_context_storage_t storage;
    foc_power_runtime_config_t active_config;
    foc_power_output_t last_output;
    foc_power_management_ops_t ops;
} foc_power_management_t;

/* Initializes and commits the generated disabled policy. */
foc_power_management_result_t foc_power_management_init(
    foc_power_management_t *management,
    const foc_power_management_ops_t *ops);

/* Transactional configure; only accepted with a disabled, inactive, clean axis. */
foc_power_management_result_t foc_power_management_configure(
    foc_power_management_t *management,
    const foc_power_runtime_config_t *config,
    const foc_config_apply_guard_t *guard);

/* Management-rate step.  A shutdown result invokes force_safe synchronously. */
foc_power_management_result_t foc_power_management_step(
    foc_power_management_t *management,
    const foc_power_input_t *input,
    foc_power_output_t *output);

/* Sticky faults clear only from the same safe stopped-state guard. */
foc_power_management_result_t foc_power_management_reset_faults(
    foc_power_management_t *management,
    const foc_power_input_t *input,
    const foc_config_apply_guard_t *guard);

#endif /* FOC_POWER_MANAGEMENT_H */
