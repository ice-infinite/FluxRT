#ifndef FOC_SENSORLESS_PLATFORM_H
#define FOC_SENSORLESS_PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns capabilities proven for this concrete board/MCU integration.
 * The STM32G431/IHM16M1 provider intentionally returns zero until the no-power
 * timing gate and the motor saliency/polarity gates are closed. */
uint32_t foc_sensorless_platform_capabilities(void);

#ifdef __cplusplus
}
#endif

#endif
