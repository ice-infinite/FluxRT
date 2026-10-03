#ifndef FOC_SIMPLE_INPUT_ADAPTER_H
#define FOC_SIMPLE_INPUT_ADAPTER_H

/* C application boundary for P2.5 simple inputs.
 *
 * Rust owns deterministic normalization. This adapter creates a read-only
 * sample plus a SETPOINT ProductCommand candidate. It cannot create an Axis
 * request, clear a fault, arm the power stage or write a PWM register.
 */

#include "foc_input_bridge.h"
#include "foc_input_service.h"

#define FOC_SIMPLE_INPUT_CANDIDATE_VERSION (1UL)

typedef uint32_t foc_simple_input_adapter_result_t;
enum
{
    FOC_SIMPLE_INPUT_ADAPTER_OK = 0,
    FOC_SIMPLE_INPUT_ADAPTER_INVALID_ARGUMENT = 1,
    FOC_SIMPLE_INPUT_ADAPTER_ABI_MISMATCH = 2,
    FOC_SIMPLE_INPUT_ADAPTER_NORMALIZATION_FAILED = 3,
    FOC_SIMPLE_INPUT_ADAPTER_INVALID_MAPPING = 4,
    FOC_SIMPLE_INPUT_ADAPTER_INVALID_CONFIG = 5,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_input_sample_t sample;
    foc_product_command_t command;
} foc_simple_input_candidate_t;

/* Calls each Rust entry with null arguments and requires deterministic
 * INVALID_ARGUMENT. This is a boot-time link/ABI presence check, not a hardware
 * test and not an input calibration test. */
foc_simple_input_adapter_result_t foc_simple_input_adapter_abi_self_check(void);

/* Convert the persistent V2 external-I/O configuration into the small Rust
 * normalization ABI. These helpers do not read hardware or normalize data. */
foc_simple_input_adapter_result_t foc_simple_input_adapter_map_centered_config(
    const foc_external_io_config_t *config,
    foc_external_input_mask_t input,
    foc_centered_input_config_t *output);

foc_simple_input_adapter_result_t foc_simple_input_adapter_map_step_dir_config(
    const foc_external_io_config_t *config,
    foc_step_dir_input_config_t *output);

foc_simple_input_adapter_result_t foc_simple_input_adapter_build_candidate(
    foc_external_input_mask_t input,
    uint32_t source_id,
    uint32_t sequence,
    uint32_t sampled_at_us,
    uint32_t now_ms,
    uint32_t timeout_ms,
    foc_control_mode_t control_mode,
    foc_feedback_mode_t feedback_mode,
    const foc_input_normalization_output_t *normalized,
    foc_simple_input_candidate_t *candidate);

#endif /* FOC_SIMPLE_INPUT_ADAPTER_H */
