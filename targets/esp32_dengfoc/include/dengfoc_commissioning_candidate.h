#ifndef DENGFOC_COMMISSIONING_CANDIDATE_H
#define DENGFOC_COMMISSIONING_CANDIDATE_H

#include <stdint.h>

/* Versioned, non-persistent commissioning candidate. This is deliberately
 * separate from the power transaction and from the approved product profile.
 * Revision 1 was derived from the two bounded M0 alignment runs recorded in
 * LOG-20261004-069/070. Revision 2 adds the measured commutation-direction
 * mapping from LOG-20261004-072. Revision 3 replaces the incompatible fixed
 * ten-sample direction gate with net motion plus three quantized increments,
 * based on LOG-20261004-076. It is not production calibration evidence. */
typedef struct
{
    uint32_t revision;
    uint32_t pole_pairs;
    int32_t commutation_direction;
    float electrical_offset_rad;
    uint32_t direction_ramp_ms;
    uint32_t direction_total_ms;
    float direction_start_voltage_v;
    float direction_maximum_voltage_v;
    float direction_minimum_movement_rad;
    float direction_maximum_movement_rad;
    float direction_minimum_increment_rad;
    uint32_t direction_minimum_evidence_samples;
    float direction_maximum_speed_rad_s;
} dengfoc_commissioning_candidate_t;

static constexpr dengfoc_commissioning_candidate_t
    kDengfocM0CommissioningCandidate = {
        .revision = 3U,
        .pole_pairs = 7U,
        /* LOG-20261004-072: +q produced negative AS5600 motion. Keep the
         * sensor coordinate unchanged and explicitly map commutation here. */
        .commutation_direction = -1,
        .electrical_offset_rad = 0.474181205F,
        .direction_ramp_ms = 250U,
        .direction_total_ms = 800U,
        .direction_start_voltage_v = 0.10F,
        .direction_maximum_voltage_v = 0.35F,
        .direction_minimum_movement_rad = 0.010F,
        .direction_maximum_movement_rad = 0.120F,
        /* Half one AS5600 count: reject sub-quantization float noise. */
        .direction_minimum_increment_rad = 0.0007669904F,
        .direction_minimum_evidence_samples = 3U,
        .direction_maximum_speed_rad_s = 20.0F,
};

static_assert(
    (kDengfocM0CommissioningCandidate.commutation_direction == 1) ||
        (kDengfocM0CommissioningCandidate.commutation_direction == -1),
    "commissioning commutation direction must be canonical");
static_assert(kDengfocM0CommissioningCandidate.pole_pairs > 0U,
              "commissioning pole pairs must be nonzero");
static_assert(
    kDengfocM0CommissioningCandidate.direction_ramp_ms <
        kDengfocM0CommissioningCandidate.direction_total_ms,
    "commissioning direction ramp must finish before timeout");
static_assert(
    (kDengfocM0CommissioningCandidate.direction_start_voltage_v > 0.0F) &&
        (kDengfocM0CommissioningCandidate.direction_maximum_voltage_v >
         kDengfocM0CommissioningCandidate.direction_start_voltage_v),
    "commissioning direction voltage envelope must be increasing");
static_assert(
    (kDengfocM0CommissioningCandidate.direction_minimum_movement_rad > 0.0F) &&
        (kDengfocM0CommissioningCandidate.direction_maximum_movement_rad >
         kDengfocM0CommissioningCandidate.direction_minimum_movement_rad),
    "commissioning direction movement envelope must be ordered");
static_assert(
    (kDengfocM0CommissioningCandidate.direction_minimum_increment_rad > 0.0F) &&
        (kDengfocM0CommissioningCandidate.direction_minimum_increment_rad <
         kDengfocM0CommissioningCandidate.direction_minimum_movement_rad) &&
        (kDengfocM0CommissioningCandidate
             .direction_minimum_evidence_samples >= 3U),
    "commissioning direction evidence gate must reject single-sample motion");

#endif /* DENGFOC_COMMISSIONING_CANDIDATE_H */
