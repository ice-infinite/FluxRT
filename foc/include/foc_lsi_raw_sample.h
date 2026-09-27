#ifndef FOC_LSI_RAW_SAMPLE_H
#define FOC_LSI_RAW_SAMPLE_H

/*
 * FluxRT - hardware-neutral raw sample ABI for EXP-B3 Ls(I).
 *
 * Platform C fills this fixed record from one ADC/PWM epoch.  The application
 * owns storage and experiment sequencing; Rust/PC code receives converted SI
 * values later.  No HAL type crosses this boundary.
 */

#include <stdint.h>

#define FOC_LSI_RAW_SAMPLE_VERSION (1UL)

enum
{
    /* Bits 0..3 contain foc_lsi_state_t without a reverse application dependency. */
    FOC_LSI_RAW_FLAG_STATE_MASK = 0x000FU,
    FOC_LSI_RAW_FLAG_DRIVE_ACTIVE = (1U << 4),
    FOC_LSI_RAW_FLAG_PULSE_POSITIVE = (1U << 5),
    FOC_LSI_RAW_FLAG_PULSE_NEGATIVE = (1U << 6),
    FOC_LSI_RAW_FLAG_ADC_VALID = (1U << 7),
    FOC_LSI_RAW_FLAG_HARDWARE_FAULT = (1U << 8),
    FOC_LSI_RAW_FLAG_SOFTWARE_TRIP = (1U << 9),
};

typedef struct
{
    uint32_t sequence;
    uint32_t control_tick;
    uint16_t current_u_raw;
    uint16_t current_v_raw;
    /* Must come from the same injected sequence; stale slow-monitor Vbus is forbidden. */
    uint16_t bus_voltage_raw;
    /* Compare values active during this ADC sample, captured before any new write. */
    uint16_t compare_u;
    uint16_t compare_v;
    uint16_t compare_w;
    /* TIMx ARR/duty denominator (7083 at the current 170 MHz/12 kHz setup). */
    uint16_t pwm_period_ticks;
    uint16_t flags;
} foc_lsi_raw_sample_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_lsi_raw_sample_t) == 24U,
               "LSI raw sample ABI changed");
#endif

#endif /* FOC_LSI_RAW_SAMPLE_H */
