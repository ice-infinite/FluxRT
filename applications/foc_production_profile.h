#ifndef FLUXRT_FOC_PRODUCTION_PROFILE_H
#define FLUXRT_FOC_PRODUCTION_PROFILE_H

#include <stdint.h>

#include "foc_rust_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Immutable production-profile record format. Every field is a 32-bit word so
 * its CRC has one canonical representation on the host and the MCU. */
#define FOC_PRODUCTION_PROFILE_MAGIC                    (0x46505246UL) /* "FPRF" */
#define FOC_PRODUCTION_PROFILE_SCHEMA_VERSION           (1UL)
#define FOC_PRODUCTION_PROFILE_BOARD_NUCLEO_G431_IHM16  (0x43116001UL)
#define FOC_PRODUCTION_PROFILE_MOTOR_GBM2804H_100T      (0x28041007UL)

/* Approval is deliberately split: identified parameters may be approved while
 * observer handoff stays disabled. Closed-loop approval without parameter
 * approval is invalid and makes the whole profile fail closed. */
#define FOC_PROFILE_APPROVAL_PARAMETERS   (1UL << 0)
#define FOC_PROFILE_APPROVAL_CLOSED_LOOP  (1UL << 1)
#define FOC_PROFILE_APPROVAL_KNOWN_MASK   \
    (FOC_PROFILE_APPROVAL_PARAMETERS | FOC_PROFILE_APPROVAL_CLOSED_LOOP)

typedef uint32_t foc_production_profile_status_t;
enum
{
    FOC_PROFILE_VALID_UNAPPROVED = 0,
    FOC_PROFILE_VALID_PARAMETERS_APPROVED = 1,
    FOC_PROFILE_VALID_CLOSED_LOOP_APPROVED = 2,
    FOC_PROFILE_INVALID_ARGUMENT = 3,
    FOC_PROFILE_BAD_MAGIC = 4,
    FOC_PROFILE_BAD_LAYOUT = 5,
    FOC_PROFILE_WRONG_TARGET = 6,
    FOC_PROFILE_BAD_APPROVAL = 7,
    FOC_PROFILE_BAD_RECORD_CRC = 8,
    FOC_PROFILE_BAD_RUNTIME_CRC = 9,
    FOC_PROFILE_RUNTIME_CONFIG_INVALID = 10,
};

#define FOC_PRODUCTION_PROFILE_STATUS_IS_VALID(status) \
    ((status) <= FOC_PROFILE_VALID_CLOSED_LOOP_APPROVED)

/* record_crc32 covers every preceding word, including runtime_config_crc32 and
 * approval_flags. It does not cover itself. runtime_config_crc32 is computed by
 * Rust over all 74 ABI words of foc_runtime_config_t. */
typedef struct
{
    uint32_t magic;
    uint32_t struct_size;
    uint32_t schema_version;
    uint32_t profile_revision;
    uint32_t board_id;
    uint32_t motor_id;
    uint32_t runtime_config_version;
    uint32_t runtime_config_crc32;
    uint32_t approval_flags;
    uint32_t record_crc32;
} foc_production_profile_t;

typedef struct
{
    foc_production_profile_status_t status;
    uint32_t profile_revision;
    uint32_t approval_flags;
    uint32_t computed_runtime_config_crc32;
    uint32_t computed_record_crc32;
} foc_production_profile_report_t;

/* Current immutable baseline. It is structurally valid but intentionally has
 * approval_flags=0, so it can never enable observer handoff. */
extern const foc_production_profile_t g_foc_production_profile;

uint32_t foc_production_profile_record_crc32(
    const foc_production_profile_t *profile);

foc_production_profile_status_t foc_production_profile_validate(
    const foc_production_profile_t *profile,
    uint32_t computed_runtime_config_crc32,
    foc_production_profile_report_t *report);

/* Transactionally applies the application-owned baseline overrides, computes
 * the canonical Rust CRC, validates target identity/approval/record CRC, and
 * commits only on success. A failure leaves config byte-for-byte unchanged. */
foc_production_profile_status_t foc_production_profile_apply(
    const foc_production_profile_t *profile,
    foc_runtime_config_t *config,
    foc_production_profile_report_t *report);

#ifdef __cplusplus
}
#endif

#endif /* FLUXRT_FOC_PRODUCTION_PROFILE_H */
