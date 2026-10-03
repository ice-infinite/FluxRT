#include "foc_external_io_board.h"
#include "foc_external_io_platform.h"

/*
 * STM32G431RB + X-NUCLEO-IHM16M1 provider selected by the target build.
 * A different board/MCU supplies the same public symbol from its own platform
 * directory; no common management source needs to change.
 */
void foc_external_io_platform_capabilities(
    foc_external_io_capabilities_t *capabilities)
{
    if (capabilities == 0)
    {
        return;
    }
    foc_external_io_build_capabilities(capabilities);

    capabilities->reserved_resource_mask =
        FOC_G431_RESOURCE_LPUART1_PA2_PA3 |
        FOC_G431_RESOURCE_TIM1_POWER_PWM |
        FOC_G431_RESOURCE_TIM2_CONTROL_DIVIDER |
        FOC_G431_RESOURCE_ADC12_CURRENT_BUS |
        FOC_G431_RESOURCE_ADC1_BEMF_TEMP |
        FOC_G431_RESOURCE_IHM16_CURRENT_REF_PB4;

    /* UART[0] is intentionally mapped to the occupied bring-up console. A
     * future Native UART driver must choose and document a dedicated instance
     * before it may set board_transport_mask. */
    capabilities->transport_resource_masks[0] =
        FOC_G431_RESOURCE_LPUART1_PA2_PA3;
    capabilities->transport_resource_masks[1] = 0U;
    capabilities->transport_resource_masks[2] =
        FOC_G431_RESOURCE_FDCAN1_CANDIDATE;
    capabilities->transport_resource_masks[3] =
        FOC_G431_RESOURCE_FDCAN1_CANDIDATE;
    capabilities->transport_resource_masks[4] = 0U;

    capabilities->input_resource_masks[0] =
        FOC_G431_RESOURCE_TIM3_CANDIDATE |
        FOC_G431_RESOURCE_IHM16_CURRENT_REF_PB4;
    capabilities->input_resource_masks[1] =
        FOC_G431_RESOURCE_TIM3_CANDIDATE;
    capabilities->input_resource_masks[2] =
        FOC_G431_RESOURCE_ADC1_REGULAR_PC2;
    capabilities->input_resource_masks[3] =
        FOC_G431_RESOURCE_TIM3_CANDIDATE;

    /* PC2/ADC1_IN8 is the verified IHM16M1 SPEED path and now has a
     * non-blocking regular-conversion driver.  Keep the product board mask
     * closed until P2.5A4 wires configuration/start/stop, timeout and no-power
     * target acceptance.  PB4 is physically strapped to CURRENT_REF on the
     * shield, so it is not a safe TIM3 input candidate. */
    capabilities->board_transport_mask = 0U;
    capabilities->board_input_mask = 0U;
}
