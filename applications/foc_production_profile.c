/*
 * Immutable motor/board profile policy for both Diagnostic and Production.
 *
 * This module belongs to the RT-Thread application-management layer: it decides
 * which board/motor identity is accepted and which approvals may affect start-up
 * configuration. Rust owns canonical runtime-config validation and CRC; the MCU
 * platform and realtime ISR never parse this record.
 */

#include "foc_production_profile.h"

#include <string.h>

_Static_assert(sizeof(foc_production_profile_t) == 40U,
               "production profile layout changed; bump schema and CRC");

/* CM4.4 revision 6: NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T.
 * runtime CRC 0xEDF4F6CA covers the ABI-V17/config-V11 bidirectional start and
 * handover candidate proved by the 2026-09-25 unloaded hardware matrix.
 * record CRC 0x888E4E91 covers the first nine words.
 * No approval bit is set: open-loop bring-up remains possible, observer handoff
 * remains impossible in Production until a later reviewed profile changes both
 * CRCs and approval evidence. */
const foc_production_profile_t g_foc_production_profile =
{
    FOC_PRODUCTION_PROFILE_MAGIC,
    sizeof(foc_production_profile_t),
    FOC_PRODUCTION_PROFILE_SCHEMA_VERSION,
    6U,
    FOC_PRODUCTION_PROFILE_BOARD_NUCLEO_G431_IHM16,
    FOC_PRODUCTION_PROFILE_MOTOR_GBM2804H_100T,
    FOC_RUST_CONFIG_VERSION,
    0xEDF4F6CAUL,
    0U,
    0x888E4E91UL,
};

static uint32_t crc32_word(uint32_t crc, uint32_t word)
{
    uint32_t byte_index;
    for (byte_index = 0U; byte_index < 4U; ++byte_index)
    {
        uint32_t bit_index;
        crc ^= (word >> (byte_index * 8U)) & 0xFFU;
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            const uint32_t reflected_polynomial =
                0xEDB88320UL & (0U - (crc & 1U));
            crc = (crc >> 1U) ^ reflected_polynomial;
        }
    }
    return crc;
}

uint32_t foc_production_profile_record_crc32(
    const foc_production_profile_t *profile)
{
    uint32_t crc = 0xFFFFFFFFUL;
    if (profile == NULL)
    {
        return 0U;
    }

    crc = crc32_word(crc, profile->magic);
    crc = crc32_word(crc, profile->struct_size);
    crc = crc32_word(crc, profile->schema_version);
    crc = crc32_word(crc, profile->profile_revision);
    crc = crc32_word(crc, profile->board_id);
    crc = crc32_word(crc, profile->motor_id);
    crc = crc32_word(crc, profile->runtime_config_version);
    crc = crc32_word(crc, profile->runtime_config_crc32);
    crc = crc32_word(crc, profile->approval_flags);
    return ~crc;
}

static void report_begin(foc_production_profile_report_t *report,
                         const foc_production_profile_t *profile,
                         uint32_t runtime_crc32)
{
    if (report == NULL)
    {
        return;
    }
    memset(report, 0, sizeof(*report));
    report->status = FOC_PROFILE_INVALID_ARGUMENT;
    report->computed_runtime_config_crc32 = runtime_crc32;
    if (profile != NULL)
    {
        report->profile_revision = profile->profile_revision;
        report->approval_flags = profile->approval_flags;
        report->computed_record_crc32 =
            foc_production_profile_record_crc32(profile);
    }
}

static foc_production_profile_status_t report_finish(
    foc_production_profile_report_t *report,
    foc_production_profile_status_t status)
{
    if (report != NULL)
    {
        report->status = status;
    }
    return status;
}

foc_production_profile_status_t foc_production_profile_validate(
    const foc_production_profile_t *profile,
    uint32_t computed_runtime_config_crc32,
    foc_production_profile_report_t *report)
{
    uint32_t computed_record_crc32;
    report_begin(report, profile, computed_runtime_config_crc32);
    if (profile == NULL)
    {
        return report_finish(report, FOC_PROFILE_INVALID_ARGUMENT);
    }
    if (profile->magic != FOC_PRODUCTION_PROFILE_MAGIC)
    {
        return report_finish(report, FOC_PROFILE_BAD_MAGIC);
    }
    if ((profile->struct_size != sizeof(*profile)) ||
        (profile->schema_version != FOC_PRODUCTION_PROFILE_SCHEMA_VERSION) ||
        (profile->profile_revision == 0U) ||
        (profile->runtime_config_version != FOC_RUST_CONFIG_VERSION))
    {
        return report_finish(report, FOC_PROFILE_BAD_LAYOUT);
    }
    if ((profile->board_id != FOC_PRODUCTION_PROFILE_BOARD_NUCLEO_G431_IHM16) ||
        (profile->motor_id != FOC_PRODUCTION_PROFILE_MOTOR_GBM2804H_100T))
    {
        return report_finish(report, FOC_PROFILE_WRONG_TARGET);
    }
    if (((profile->approval_flags & ~FOC_PROFILE_APPROVAL_KNOWN_MASK) != 0U) ||
        (((profile->approval_flags & FOC_PROFILE_APPROVAL_CLOSED_LOOP) != 0U) &&
         ((profile->approval_flags & FOC_PROFILE_APPROVAL_PARAMETERS) == 0U)))
    {
        return report_finish(report, FOC_PROFILE_BAD_APPROVAL);
    }
    computed_record_crc32 = foc_production_profile_record_crc32(profile);
    if (computed_record_crc32 != profile->record_crc32)
    {
        return report_finish(report, FOC_PROFILE_BAD_RECORD_CRC);
    }
    if (computed_runtime_config_crc32 != profile->runtime_config_crc32)
    {
        return report_finish(report, FOC_PROFILE_BAD_RUNTIME_CRC);
    }
    if ((profile->approval_flags & FOC_PROFILE_APPROVAL_CLOSED_LOOP) != 0U)
    {
        return report_finish(report, FOC_PROFILE_VALID_CLOSED_LOOP_APPROVED);
    }
    if ((profile->approval_flags & FOC_PROFILE_APPROVAL_PARAMETERS) != 0U)
    {
        return report_finish(report, FOC_PROFILE_VALID_PARAMETERS_APPROVED);
    }
    return report_finish(report, FOC_PROFILE_VALID_UNAPPROVED);
}

foc_production_profile_status_t foc_production_profile_apply(
    const foc_production_profile_t *profile,
    foc_runtime_config_t *config,
    foc_production_profile_report_t *report)
{
    foc_runtime_config_t candidate;
    foc_production_profile_status_t status;
    uint32_t runtime_crc32 = 0U;

    report_begin(report, profile, 0U);
    if ((profile == NULL) || (config == NULL))
    {
        return report_finish(report, FOC_PROFILE_INVALID_ARGUMENT);
    }

    /* Validate the immutable record before trusting approval_flags. The runtime
     * CRC is checked again below against the independently computed value. */
    status = foc_production_profile_validate(profile,
                                             profile->runtime_config_crc32,
                                             report);
    if (!FOC_PRODUCTION_PROFILE_STATUS_IS_VALID(status))
    {
        return status;
    }

    candidate = *config;
    candidate.observer_backend = FOC_OBSERVER_SMO_PLL;
    candidate.observer_enable = 1U;
    candidate.closed_loop_enable =
        ((profile->approval_flags & FOC_PROFILE_APPROVAL_CLOSED_LOOP) != 0U) ? 1U : 0U;
    candidate.observer_update_divider = 1U;
    candidate.voltage_utilization = 0.90f;
    /* Revision 6 carries only preloaded inverter-model candidates. It explicitly
     * approves neither observer correction nor PWM feed-forward, so every boot
     * starts with all three gates closed even if a caller supplied other values. */
    candidate.inverter_voltage_model.enabled = 0U;
    candidate.inverter_voltage_model.observer_voltage_correction_enable = 0U;
    candidate.inverter_voltage_model.pwm_feedforward_enable = 0U;

    if (foc_rust_runtime_config_crc32(&candidate, &runtime_crc32) != FOC_STATUS_OK)
    {
        report_begin(report, profile, runtime_crc32);
        return report_finish(report, FOC_PROFILE_RUNTIME_CONFIG_INVALID);
    }
    status = foc_production_profile_validate(profile, runtime_crc32, report);
    if (FOC_PRODUCTION_PROFILE_STATUS_IS_VALID(status))
    {
        *config = candidate;
    }
    return status;
}
