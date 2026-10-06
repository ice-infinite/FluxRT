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
#define FOC_SENSORLESS_PROBE_DECOMP_TICKS (128U)

uint32_t foc_platform_sensorless_probe_decomp_count(void);

/* One recorded tick of a mode-3 probe.  Every figure is in cycles and is
 * already saturated to uint16_t.
 *
 * `total` is the whole ISR, `control` the fast-loop section as measured by the
 * platform.  `chain` is the sensorless chain's own cost as reported by the
 * combined entry, so `control - chain` attributes everything else in the fast
 * loop (Clarke, Park, current PI, final limit, SVPWM).  The remaining columns
 * split `chain` completely, which is what decides whether that work can be
 * decimated: `separator` is the high-frequency current LPF, `hfi` the
 * rotating-injection update (or polarity step), `fusion` the angle/speed
 * supervisor, and `abi_*` the wrapper's own entry checks, sequence/ledger
 * validations, publish and fused-speed tail. */
typedef struct
{
    uint16_t total;
    uint16_t control;
    uint16_t chain;
    uint16_t separator;
    uint16_t hfi;
    uint16_t fusion;
    /* Remainder of `chain` after the three split points above. */
    uint16_t abi_prepare;
    uint16_t abi_checks;
    uint16_t abi_publish;
    uint16_t abi_tail;
    /* Attribution of the shared controller core. */
    uint16_t core_gate;
    uint16_t core_observer;
    uint16_t core_startup;
    uint16_t core_current_loop;
    /* That span split once more: control law vs the bookkeeping around it. */
    uint16_t tail_setup;
    uint16_t reference;
    uint16_t tail_finish;
} foc_probe_decomp_tick_t;

/* Copies up to `capacity` ticks into the caller's array and returns the number
 * written.  Returns 0 when no mode-3 probe has run. */
uint32_t foc_platform_sensorless_probe_decomp_copy(
    foc_probe_decomp_tick_t *ticks,
    uint32_t capacity);

#ifdef __cplusplus
}
#endif

#endif
