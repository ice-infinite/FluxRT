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

/* Cold-start decomposition of the P5.5 composite probe (mode 3).
 *
 * The first measured composite tick overruns the 12,750-cycle deadline, which
 * stops the probe there, so the recurring (steady-state) cost is unmeasurable
 * from that run alone.  The platform records the first N ticks of a mode-3 probe
 * into a fixed RAM buffer and exposes them here; the Shell prints them after the
 * ADC ISR has exited, because printing on the ISR path changes what is measured.
 *
 * Both functions return 0 when no mode-3 probe has run.  `count` is the number
 * of recorded ticks; tick 0 is the first measured tick. */
#define FOC_SENSORLESS_PROBE_DECOMP_TICKS (256U)

uint32_t foc_platform_sensorless_probe_decomp_count(void);
/* Copies up to `capacity` entries into the caller's arrays and returns the
 * number written.  `total_cycles` is the whole-ISR duration and
 * `control_cycles` the fast-loop section, both measured by the platform;
 * `chain_cycles` is the sensorless chain's own cost as reported by the combined
 * entry, so `control - chain` attributes the rest of the fast loop (Clarke,
 * Park, current PI, final limit, SVPWM) without instrumenting every stage.
 * All three are already saturated to uint16_t. */
uint32_t foc_platform_sensorless_probe_decomp_copy(
    uint16_t *total_cycles,
    uint16_t *control_cycles,
    uint16_t *chain_cycles,
    uint32_t capacity);

#ifdef __cplusplus
}
#endif

#endif
