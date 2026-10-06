#ifndef FOC_DENGFOC_POWER_STAGE_H
#define FOC_DENGFOC_POWER_STAGE_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_board_dengfoc_v04.h"

#define FOC_DENGFOC_PHASE_COUNT (3U)

#define FOC_DENGFOC_CAP_PWM_DUTY_READBACK          (1UL << 0)
#define FOC_DENGFOC_CAP_CENTER_ALIGNED_PWM          (1UL << 1)
#define FOC_DENGFOC_CAP_COHERENT_THREE_PHASE_UPDATE (1UL << 2)
#define FOC_DENGFOC_CAP_PHASE_LOCKED_CURRENT_SAMPLE (1UL << 3)
#define FOC_DENGFOC_CAP_ADC_OVERRUN_DETECTION       (1UL << 4)
#define FOC_DENGFOC_CAP_HARDWARE_FAST_SHUTDOWN      (1UL << 5)
#define FOC_DENGFOC_CAP_PWM_CYCLE_ISR                (1UL << 6)
#define FOC_DENGFOC_CAP_KNOWN_MASK                   (0x7FUL)

/* Voltage-only commissioning does not close the current loop. It covers
 * fixed-axis alignment and bounded encoder-commutated direction checks, and
 * still requires coherent, read-back PWM plus a live cycle timebase. */
#define FOC_DENGFOC_VOLTAGE_COMMISSIONING_REQUIRED_CAPABILITIES \
    (FOC_DENGFOC_CAP_PWM_DUTY_READBACK |             \
     FOC_DENGFOC_CAP_CENTER_ALIGNED_PWM |            \
     FOC_DENGFOC_CAP_COHERENT_THREE_PHASE_UPDATE |   \
     FOC_DENGFOC_CAP_PWM_CYCLE_ISR)

/* Source compatibility for the already-recorded alignment target. */
#define FOC_DENGFOC_ALIGNMENT_REQUIRED_CAPABILITIES \
    FOC_DENGFOC_VOLTAGE_COMMISSIONING_REQUIRED_CAPABILITIES

/* A target may prepare a diagnostic timer with fewer capabilities, but an
 * actuating current loop must never weaken this set in its policy. */
#define FOC_DENGFOC_CURRENT_LOOP_REQUIRED_CAPABILITIES \
    (FOC_DENGFOC_CAP_PWM_DUTY_READBACK |              \
     FOC_DENGFOC_CAP_CENTER_ALIGNED_PWM |             \
     FOC_DENGFOC_CAP_COHERENT_THREE_PHASE_UPDATE |    \
     FOC_DENGFOC_CAP_PHASE_LOCKED_CURRENT_SAMPLE |    \
     FOC_DENGFOC_CAP_ADC_OVERRUN_DETECTION |          \
     FOC_DENGFOC_CAP_HARDWARE_FAST_SHUTDOWN |         \
     FOC_DENGFOC_CAP_PWM_CYCLE_ISR)

typedef enum
{
    FOC_DENGFOC_OPERATION_DISABLED = 0,
    FOC_DENGFOC_OPERATION_VOLTAGE_COMMISSIONING = 1,
    FOC_DENGFOC_OPERATION_VOLTAGE_ALIGNMENT =
        FOC_DENGFOC_OPERATION_VOLTAGE_COMMISSIONING,
    FOC_DENGFOC_OPERATION_CURRENT_LOOP = 2
} foc_dengfoc_operation_t;

typedef enum
{
    FOC_DENGFOC_POWER_RESET = 0,
    FOC_DENGFOC_POWER_SAFE_DISABLED = 1,
    FOC_DENGFOC_POWER_PWM_READY = 2,
    FOC_DENGFOC_POWER_ARMED = 3,
    FOC_DENGFOC_POWER_FAULT_LATCHED = 4
} foc_dengfoc_power_state_t;

typedef struct
{
    uint32_t allow_actuation;
    foc_dengfoc_operation_t operation;
    uint32_t required_capabilities;
    /* Zero means unrestricted and is valid only for CURRENT_LOOP. Voltage
     * commissioning commands are bounded around 50 percent duty by this count. */
    uint32_t maximum_active_duty_deviation_count;
} foc_dengfoc_power_policy_t;

typedef struct
{
    uint32_t sequence;
    uint32_t duty_count[FOC_DENGFOC_PHASE_COUNT];
} foc_dengfoc_pwm_command_t;

typedef bool (*foc_dengfoc_power_force_disable_fn)(void *context);
typedef bool (*foc_dengfoc_power_driver_disabled_fn)(
    void *context,
    bool *disabled);
typedef bool (*foc_dengfoc_power_configure_adc_fn)(
    void *context,
    uint32_t axis,
    uint32_t current_a_gpio,
    uint32_t current_b_gpio);
typedef bool (*foc_dengfoc_power_configure_pwm_fn)(
    void *context,
    uint32_t axis,
    const uint32_t pwm_gpio[FOC_DENGFOC_PHASE_COUNT],
    uint32_t frequency_hz,
    uint32_t resolution_bits);
typedef bool (*foc_dengfoc_power_write_pwm_fn)(
    void *context,
    uint32_t axis,
    const uint32_t duty_count[FOC_DENGFOC_PHASE_COUNT]);
typedef bool (*foc_dengfoc_power_read_pwm_fn)(
    void *context,
    uint32_t axis,
    uint32_t duty_count[FOC_DENGFOC_PHASE_COUNT]);
typedef bool (*foc_dengfoc_power_set_driver_enabled_fn)(
    void *context,
    bool enabled);
typedef bool (*foc_dengfoc_power_read_capabilities_fn)(
    void *context,
    uint32_t *capabilities);

typedef struct
{
    void *context;
    foc_dengfoc_power_force_disable_fn force_driver_disabled;
    foc_dengfoc_power_driver_disabled_fn driver_is_disabled;
    foc_dengfoc_power_configure_adc_fn configure_current_adc;
    foc_dengfoc_power_configure_pwm_fn configure_pwm;
    foc_dengfoc_power_write_pwm_fn write_pwm;
    foc_dengfoc_power_read_pwm_fn read_pwm;
    foc_dengfoc_power_set_driver_enabled_fn set_driver_enabled;
    foc_dengfoc_power_read_capabilities_fn read_capabilities;
} foc_dengfoc_power_ops_t;

typedef struct
{
    const foc_dengfoc_board_profile_t *profile;
    foc_dengfoc_power_ops_t ops;
    foc_dengfoc_power_state_t state;
    uint32_t axis;
    uint32_t maximum_duty_count;
    uint32_t fault_epoch;
    uint32_t last_sequence;
    uint32_t pwm_configured;
    uint32_t allow_actuation;
    foc_dengfoc_operation_t operation;
    uint32_t required_capabilities;
    uint32_t maximum_active_duty_deviation_count;
    uint32_t platform_capabilities;
    uint32_t missing_capabilities;
} foc_dengfoc_power_stage_t;

bool foc_dengfoc_power_stage_init(
    foc_dengfoc_power_stage_t *stage,
    const foc_dengfoc_board_profile_t *profile,
    uint32_t axis,
    const foc_dengfoc_power_policy_t *policy,
    const foc_dengfoc_power_ops_t *ops);

bool foc_dengfoc_power_stage_prepare(foc_dengfoc_power_stage_t *stage);

bool foc_dengfoc_power_stage_arm(
    foc_dengfoc_power_stage_t *stage,
    uint32_t expected_fault_epoch);

bool foc_dengfoc_power_stage_commit(
    foc_dengfoc_power_stage_t *stage,
    uint32_t expected_fault_epoch,
    const foc_dengfoc_pwm_command_t *command);

bool foc_dengfoc_power_stage_shutdown(foc_dengfoc_power_stage_t *stage);

void foc_dengfoc_power_stage_latch_fault(foc_dengfoc_power_stage_t *stage);

bool foc_dengfoc_power_stage_clear_fault(foc_dengfoc_power_stage_t *stage);

#endif /* FOC_DENGFOC_POWER_STAGE_H */
