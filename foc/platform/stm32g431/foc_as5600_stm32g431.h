#ifndef FOC_AS5600_STM32G431_H
#define FOC_AS5600_STM32G431_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_feedback_bridge.h"

typedef struct
{
    uint32_t initialized;
    uint32_t sensor_present;
    uint32_t read_count;
    uint32_t error_count;
    uint32_t last_hal_status;
    uint32_t last_hal_error;
    uint32_t last_i2c_isr;
    uint32_t scl_high;
    uint32_t sda_high;
    uint16_t last_raw_count;
    uint16_t reserved;
} foc_as5600_stm32g431_diagnostics_t;

bool foc_as5600_stm32g431_init(void);
bool foc_as5600_stm32g431_read_raw(uint16_t *raw_count);
bool foc_as5600_stm32g431_poll(
    uint32_t sampled_at_ms,
    uint32_t sampled_at_us,
    foc_feedback_source_sample_t *output);
void foc_as5600_stm32g431_get_diagnostics(
    foc_as5600_stm32g431_diagnostics_t *output);

#endif /* FOC_AS5600_STM32G431_H */
