#include "foc_time_sync_dengfoc.h"

static uint32_t foc_time_sync_dengfoc_gpio_exists(uint32_t gpio)
{
    if ((gpio > 39U) || ((gpio >= 6U) && (gpio <= 11U)) ||
        (gpio == 20U) || (gpio == 24U) ||
        ((gpio >= 28U) && (gpio <= 31U)))
    {
        return 0U;
    }
    return 1U;
}

uint32_t foc_time_sync_dengfoc_input_resource_valid(
    const foc_dengfoc_board_profile_t *board,
    uint32_t enabled_axis_mask,
    uint32_t input_gpio)
{
    uint32_t axis_index;

    if ((board == NULL) ||
        (foc_dengfoc_v04_profile_valid(board) == false) ||
        ((enabled_axis_mask & ~0x03U) != 0U) ||
        ((enabled_axis_mask & 0x01U) == 0U) ||
        (input_gpio == FOC_TIME_SYNC_DENGFOC_GPIO_UNCONFIGURED) ||
        (foc_time_sync_dengfoc_gpio_exists(input_gpio) == 0U) ||
        (input_gpio == board->driver_enable_gpio))
    {
        return 0U;
    }

    for (axis_index = 0U; axis_index < FOC_DENGFOC_V04_AXIS_COUNT;
         ++axis_index)
    {
        const foc_dengfoc_axis_profile_t *axis = &board->axis[axis_index];
        if ((input_gpio == axis->pwm_a_gpio) ||
            (input_gpio == axis->pwm_b_gpio) ||
            (input_gpio == axis->pwm_c_gpio) ||
            (input_gpio == axis->current_a_adc_gpio) ||
            (input_gpio == axis->current_b_adc_gpio))
        {
            return 0U;
        }
        if (((enabled_axis_mask & (1UL << axis_index)) != 0U) &&
            ((input_gpio == axis->i2c_sda_gpio) ||
             (input_gpio == axis->i2c_scl_gpio)))
        {
            return 0U;
        }
    }
    return 1U;
}

uint32_t foc_time_sync_dengfoc_adapter_init(
    foc_time_sync_adapter_t *adapter,
    foc_time_sync_capture_t *capture,
    void *port_context,
    foc_time_sync_read_local_tick_isr_fn read_local_tick_isr)
{
    return foc_time_sync_adapter_init(
        adapter,
        capture,
        (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
        port_context,
        read_local_tick_isr,
        NULL);
}
