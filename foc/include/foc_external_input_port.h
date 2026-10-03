#ifndef FOC_EXTERNAL_INPUT_PORT_H
#define FOC_EXTERNAL_INPUT_PORT_H

/*
 * Target-neutral raw-capture port for simple external inputs.
 *
 * Platform code owns Timer/ADC/GPIO registers and publishes only bounded raw
 * samples plus health.  This contract deliberately has no HAL/LL/RT-Thread
 * type and cannot arm an Axis, create a ProductCommand or write motor PWM.
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_external_io.h"

#define FOC_EXTERNAL_INPUT_RAW_CONTRACT_VERSION (0x00010000UL)
#define FOC_EXTERNAL_INPUT_RAW_SAMPLE_VERSION   (1UL)
#define FOC_EXTERNAL_INPUT_RAW_HEALTH_VERSION   (1UL)
#define FOC_EXTERNAL_INPUT_PORT_VERSION         (1UL)
#define FOC_EXTERNAL_INPUT_PORT_OPS_VERSION     (1UL)

typedef uint32_t foc_external_input_port_result_t;
enum
{
    FOC_EXTERNAL_INPUT_PORT_OK = 0,
    FOC_EXTERNAL_INPUT_PORT_DISABLED = 1,
    FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT = 2,
    FOC_EXTERNAL_INPUT_PORT_INVALID_LAYOUT = 3,
    FOC_EXTERNAL_INPUT_PORT_NOT_AVAILABLE = 4,
    FOC_EXTERNAL_INPUT_PORT_NO_SAMPLE = 5,
    FOC_EXTERNAL_INPUT_PORT_BUSY = 6,
    FOC_EXTERNAL_INPUT_PORT_HARDWARE_ERROR = 7,
};

enum
{
    FOC_EXTERNAL_INPUT_RAW_VALID_VALUE = (1UL << 0),
    FOC_EXTERNAL_INPUT_RAW_VALID_TIMESTAMP = (1UL << 1),
    FOC_EXTERNAL_INPUT_RAW_VALID_AUXILIARY = (1UL << 2),
    FOC_EXTERNAL_INPUT_RAW_VALID_KNOWN_MASK =
        FOC_EXTERNAL_INPUT_RAW_VALID_VALUE |
        FOC_EXTERNAL_INPUT_RAW_VALID_TIMESTAMP |
        FOC_EXTERNAL_INPUT_RAW_VALID_AUXILIARY,
};

enum
{
    FOC_EXTERNAL_INPUT_RAW_QUALITY_FIRST = (1UL << 0),
    FOC_EXTERNAL_INPUT_RAW_QUALITY_MISSED_PREVIOUS = (1UL << 1),
    FOC_EXTERNAL_INPUT_RAW_QUALITY_OVERRUN = (1UL << 2),
    FOC_EXTERNAL_INPUT_RAW_QUALITY_KNOWN_MASK =
        FOC_EXTERNAL_INPUT_RAW_QUALITY_FIRST |
        FOC_EXTERNAL_INPUT_RAW_QUALITY_MISSED_PREVIOUS |
        FOC_EXTERNAL_INPUT_RAW_QUALITY_OVERRUN,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_external_input_mask_t input;
    uint32_t instance_id;
    uint32_t sequence;
    uint32_t sampled_at_us;
    uint32_t valid_flags;
    uint32_t quality_flags;
    int32_t raw_value;
    uint32_t auxiliary_value;
} foc_external_input_raw_sample_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_external_input_mask_t supported_input_mask;
    foc_external_input_mask_t enabled_input_mask;
    foc_external_input_mask_t healthy_input_mask;
    foc_external_input_mask_t fault_input_mask;
    uint32_t sample_count;
    uint32_t missed_capture_count;
    foc_external_input_port_result_t last_error;
    uint32_t last_sampled_at_us;
} foc_external_input_raw_health_t;

typedef foc_external_input_port_result_t
(*foc_external_input_port_start_fn)(
    void *context,
    foc_external_input_mask_t input_mask);
typedef foc_external_input_port_result_t
(*foc_external_input_port_stop_fn)(
    void *context,
    foc_external_input_mask_t input_mask);
typedef foc_external_input_port_result_t
(*foc_external_input_port_read_fn)(
    void *context,
    foc_external_input_mask_t input,
    foc_external_input_raw_sample_t *sample);
typedef foc_external_input_port_result_t
(*foc_external_input_port_health_fn)(
    void *context,
    foc_external_input_raw_health_t *health);

/* The function table is management-only.  It must never be dispatched from
 * the motor-control ISR; the selected platform calls its own static capture
 * hook directly from the hardware IRQ. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_external_input_port_start_fn start;
    foc_external_input_port_stop_fn stop;
    foc_external_input_port_read_fn read_latest;
    foc_external_input_port_health_fn get_health;
} foc_external_input_port_ops_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    foc_external_input_mask_t supported_input_mask;
    foc_external_input_mask_t enabled_input_mask;
    void *context;
    const foc_external_input_port_ops_t *ops;
} foc_external_input_port_t;

foc_external_input_port_result_t foc_external_input_port_bind(
    foc_external_input_port_t *port,
    foc_external_input_mask_t supported_input_mask,
    void *context,
    const foc_external_input_port_ops_t *ops);

foc_external_input_port_result_t foc_external_input_port_start(
    foc_external_input_port_t *port,
    foc_external_input_mask_t input_mask);

foc_external_input_port_result_t foc_external_input_port_stop(
    foc_external_input_port_t *port,
    foc_external_input_mask_t input_mask);

foc_external_input_port_result_t foc_external_input_port_read_latest(
    const foc_external_input_port_t *port,
    foc_external_input_mask_t input,
    foc_external_input_raw_sample_t *sample);

foc_external_input_port_result_t foc_external_input_port_get_health(
    const foc_external_input_port_t *port,
    foc_external_input_raw_health_t *health);

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_external_input_raw_sample_t) == 40U,
               "external input raw sample layout drifted");
_Static_assert(sizeof(foc_external_input_raw_health_t) == 40U,
               "external input raw health layout drifted");
_Static_assert(offsetof(foc_external_input_raw_sample_t, raw_value) == 32U,
               "external input raw value offset drifted");
#endif

#endif /* FOC_EXTERNAL_INPUT_PORT_H */
