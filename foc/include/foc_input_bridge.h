#ifndef FOC_INPUT_BRIDGE_H
#define FOC_INPUT_BRIDGE_H

/* P2.5A1 simple-input normalization ABI.
 *
 * Hardware capture stays in C. Rust validates and converts raw PWM pulse width,
 * ADC counts or accumulated Step/Dir counts. The result is only a setpoint
 * candidate; this ABI cannot request Axis state, arm outputs or touch PWM.
 */

#include <stddef.h>
#include <stdint.h>

#define FOC_INPUT_ABI_VERSION       (0x00010000UL)
#define FOC_INPUT_CONFIG_VERSION    (1UL)
#define FOC_INPUT_OUTPUT_VERSION    (1UL)

typedef uint32_t foc_input_normalization_status_t;
enum
{
    FOC_INPUT_STATUS_OK = 0,
    FOC_INPUT_STATUS_INVALID_ARGUMENT = 1,
    FOC_INPUT_STATUS_INVALID_LAYOUT = 2,
    FOC_INPUT_STATUS_INVALID_CONFIG = 3,
    FOC_INPUT_STATUS_OUT_OF_RANGE = 4,
    FOC_INPUT_STATUS_INVALID_VALUE = 5,
};

enum
{
    FOC_INPUT_QUALITY_CALIBRATED = (1UL << 0),
    FOC_INPUT_QUALITY_CENTERED = (1UL << 1),
    FOC_INPUT_QUALITY_KNOWN_MASK =
        FOC_INPUT_QUALITY_CALIBRATED | FOC_INPUT_QUALITY_CENTERED,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    int32_t raw_min;
    int32_t raw_neutral;
    int32_t raw_max;
    uint32_t deadband;
    uint32_t control_mode;
    uint32_t reserved;
    float negative_limit_si;
    float positive_limit_si;
} foc_centered_input_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t full_steps_per_revolution;
    uint32_t microsteps;
    uint32_t gear_numerator;
    uint32_t gear_denominator;
    int32_t direction;
    int32_t zero_count;
    float zero_position_rad;
    uint32_t reserved;
} foc_step_dir_input_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_input_normalization_status_t status;
    uint32_t quality_flags;
    int32_t raw_value;
    float normalized_value;
    float setpoint_si;
} foc_input_normalization_output_t;

_Static_assert(sizeof(foc_centered_input_config_t) == 40U,
               "centered input ABI layout changed");
_Static_assert(sizeof(foc_step_dir_input_config_t) == 40U,
               "Step/Dir input ABI layout changed");
_Static_assert(sizeof(foc_input_normalization_output_t) == 28U,
               "input output ABI layout changed");
_Static_assert(offsetof(foc_centered_input_config_t, negative_limit_si) == 32U,
               "centered input ABI offset changed");
_Static_assert(offsetof(foc_input_normalization_output_t, setpoint_si) == 24U,
               "input output ABI offset changed");

uint32_t foc_rust_input_normalize_pwm(
    const foc_centered_input_config_t *config,
    uint32_t pulse_width_us,
    foc_input_normalization_output_t *output);

uint32_t foc_rust_input_normalize_analog(
    const foc_centered_input_config_t *config,
    int32_t adc_counts,
    foc_input_normalization_output_t *output);

uint32_t foc_rust_input_convert_step_dir(
    const foc_step_dir_input_config_t *config,
    int32_t accumulated_count,
    foc_input_normalization_output_t *output);

#endif /* FOC_INPUT_BRIDGE_H */
