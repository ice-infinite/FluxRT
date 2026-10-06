#include "foc_dengfoc_power_stage.h"

#include <stddef.h>
#include <string.h>

static bool ops_valid(const foc_dengfoc_power_ops_t *ops)
{
    return (ops != NULL) &&
           (ops->force_driver_disabled != NULL) &&
           (ops->driver_is_disabled != NULL) &&
           (ops->configure_current_adc != NULL) &&
           (ops->configure_pwm != NULL) &&
           (ops->write_pwm != NULL) &&
           (ops->read_pwm != NULL) &&
           (ops->set_driver_enabled != NULL) &&
           (ops->read_capabilities != NULL);
}

static bool all_zero(const uint32_t duty[FOC_DENGFOC_PHASE_COUNT])
{
    uint32_t phase;
    if (duty == NULL)
    {
        return false;
    }
    for (phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase)
    {
        if (duty[phase] != 0U)
        {
            return false;
        }
    }
    return true;
}

static uint32_t required_for_operation(foc_dengfoc_operation_t operation)
{
    switch (operation)
    {
    case FOC_DENGFOC_OPERATION_VOLTAGE_COMMISSIONING:
        return FOC_DENGFOC_VOLTAGE_COMMISSIONING_REQUIRED_CAPABILITIES;
    case FOC_DENGFOC_OPERATION_CURRENT_LOOP:
        return FOC_DENGFOC_CURRENT_LOOP_REQUIRED_CAPABILITIES;
    case FOC_DENGFOC_OPERATION_DISABLED:
    default:
        return 0U;
    }
}

static bool driver_disabled(const foc_dengfoc_power_stage_t *stage)
{
    bool disabled = false;
    return stage->ops.driver_is_disabled(stage->ops.context, &disabled) &&
           disabled;
}

static bool write_and_verify_zero(foc_dengfoc_power_stage_t *stage)
{
    const uint32_t zero[FOC_DENGFOC_PHASE_COUNT] = {0U, 0U, 0U};
    uint32_t actual[FOC_DENGFOC_PHASE_COUNT] = {1U, 1U, 1U};
    return stage->ops.write_pwm(stage->ops.context, stage->axis, zero) &&
           stage->ops.read_pwm(stage->ops.context, stage->axis, actual) &&
           all_zero(actual);
}

static void force_safe_best_effort(foc_dengfoc_power_stage_t *stage)
{
    const uint32_t zero[FOC_DENGFOC_PHASE_COUNT] = {0U, 0U, 0U};
    if (stage == NULL)
    {
        return;
    }
    if (stage->ops.force_driver_disabled != NULL)
    {
        (void)stage->ops.force_driver_disabled(stage->ops.context);
    }
    if ((stage->pwm_configured != 0U) && (stage->ops.write_pwm != NULL))
    {
        (void)stage->ops.write_pwm(stage->ops.context, stage->axis, zero);
    }
}

void foc_dengfoc_power_stage_latch_fault(foc_dengfoc_power_stage_t *stage)
{
    if (stage == NULL)
    {
        return;
    }
    force_safe_best_effort(stage);
    stage->fault_epoch += 1U;
    stage->last_sequence = 0U;
    stage->state = FOC_DENGFOC_POWER_FAULT_LATCHED;
}

bool foc_dengfoc_power_stage_init(
    foc_dengfoc_power_stage_t *stage,
    const foc_dengfoc_board_profile_t *profile,
    uint32_t axis,
    const foc_dengfoc_power_policy_t *policy,
    const foc_dengfoc_power_ops_t *ops)
{
    bool disabled = false;
    uint32_t operation_required;
    uint32_t maximum_duty_count;
    if (stage == NULL)
    {
        return false;
    }
    (void)memset(stage, 0, sizeof(*stage));
    if ((profile == NULL) || !foc_dengfoc_v04_profile_valid(profile) ||
        (axis >= FOC_DENGFOC_V04_AXIS_COUNT) || (policy == NULL) ||
        (policy->allow_actuation > 1U) ||
        (policy->operation > FOC_DENGFOC_OPERATION_CURRENT_LOOP) ||
        ((policy->required_capabilities &
          ~FOC_DENGFOC_CAP_KNOWN_MASK) != 0U) ||
        !ops_valid(ops) ||
        (profile->pwm_resolution_bits >= 31U))
    {
        return false;
    }
    maximum_duty_count = (1UL << profile->pwm_resolution_bits) - 1UL;
    operation_required = required_for_operation(policy->operation);
    if (((policy->allow_actuation == 0U) &&
         ((policy->operation != FOC_DENGFOC_OPERATION_DISABLED) ||
          (policy->maximum_active_duty_deviation_count != 0U))) ||
        ((policy->allow_actuation != 0U) &&
         ((policy->operation == FOC_DENGFOC_OPERATION_DISABLED) ||
          ((policy->required_capabilities & operation_required) !=
           operation_required))) ||
        ((policy->operation == FOC_DENGFOC_OPERATION_VOLTAGE_COMMISSIONING) &&
         ((policy->maximum_active_duty_deviation_count == 0U) ||
          (policy->maximum_active_duty_deviation_count >=
           (maximum_duty_count + 1U) / 2U))) ||
        ((policy->operation == FOC_DENGFOC_OPERATION_CURRENT_LOOP) &&
         (policy->maximum_active_duty_deviation_count != 0U)))
    {
        return false;
    }
    stage->profile = profile;
    stage->ops = *ops;
    stage->axis = axis;
    stage->maximum_duty_count = maximum_duty_count;
    stage->allow_actuation = policy->allow_actuation;
    stage->operation = policy->operation;
    stage->required_capabilities = policy->required_capabilities;
    stage->maximum_active_duty_deviation_count =
        policy->maximum_active_duty_deviation_count;
    if (!stage->ops.force_driver_disabled(stage->ops.context) ||
        !stage->ops.driver_is_disabled(
            stage->ops.context, &disabled) ||
        !disabled)
    {
        foc_dengfoc_power_stage_latch_fault(stage);
        return false;
    }
    stage->state = FOC_DENGFOC_POWER_SAFE_DISABLED;
    return true;
}

bool foc_dengfoc_power_stage_prepare(foc_dengfoc_power_stage_t *stage)
{
    uint32_t pwm_gpio[FOC_DENGFOC_PHASE_COUNT];
    uint32_t capabilities = 0U;
    const foc_dengfoc_axis_profile_t *axis;
    if ((stage == NULL) ||
        (stage->state != FOC_DENGFOC_POWER_SAFE_DISABLED))
    {
        return false;
    }
    axis = &stage->profile->axis[stage->axis];
    pwm_gpio[0] = axis->pwm_a_gpio;
    pwm_gpio[1] = axis->pwm_b_gpio;
    pwm_gpio[2] = axis->pwm_c_gpio;
    if (!stage->ops.force_driver_disabled(stage->ops.context) ||
        !driver_disabled(stage) ||
        !stage->ops.configure_current_adc(
            stage->ops.context,
            stage->axis,
            axis->current_a_adc_gpio,
            axis->current_b_adc_gpio) ||
        !stage->ops.configure_pwm(
            stage->ops.context,
            stage->axis,
            pwm_gpio,
            stage->profile->pwm_frequency_hz,
            stage->profile->pwm_resolution_bits))
    {
        foc_dengfoc_power_stage_latch_fault(stage);
        return false;
    }
    stage->pwm_configured = 1U;
    if (!write_and_verify_zero(stage) || !driver_disabled(stage) ||
        !stage->ops.read_capabilities(
            stage->ops.context, &capabilities) ||
        ((capabilities & ~FOC_DENGFOC_CAP_KNOWN_MASK) != 0U))
    {
        foc_dengfoc_power_stage_latch_fault(stage);
        return false;
    }
    stage->platform_capabilities = capabilities;
    stage->missing_capabilities =
        stage->required_capabilities & ~capabilities;
    stage->last_sequence = 0U;
    stage->state = FOC_DENGFOC_POWER_PWM_READY;
    return true;
}

bool foc_dengfoc_power_stage_arm(
    foc_dengfoc_power_stage_t *stage,
    uint32_t expected_fault_epoch)
{
    bool disabled = true;
    if ((stage == NULL) ||
        (stage->state != FOC_DENGFOC_POWER_PWM_READY) ||
        (stage->allow_actuation == 0U) ||
        (stage->missing_capabilities != 0U) ||
        (stage->fault_epoch != expected_fault_epoch) ||
        !write_and_verify_zero(stage) || !driver_disabled(stage) ||
        !stage->ops.set_driver_enabled(stage->ops.context, true) ||
        !stage->ops.driver_is_disabled(stage->ops.context, &disabled) ||
        disabled)
    {
        if ((stage != NULL) &&
            (stage->state == FOC_DENGFOC_POWER_PWM_READY) &&
            (stage->allow_actuation != 0U))
        {
            foc_dengfoc_power_stage_latch_fault(stage);
        }
        return false;
    }
    stage->state = FOC_DENGFOC_POWER_ARMED;
    return true;
}

bool foc_dengfoc_power_stage_commit(
    foc_dengfoc_power_stage_t *stage,
    uint32_t expected_fault_epoch,
    const foc_dengfoc_pwm_command_t *command)
{
    uint32_t actual[FOC_DENGFOC_PHASE_COUNT];
    uint32_t phase;
    if ((stage == NULL) || (command == NULL) ||
        (stage->state != FOC_DENGFOC_POWER_ARMED) ||
        (stage->fault_epoch != expected_fault_epoch) ||
        (command->sequence != (stage->last_sequence + 1U)))
    {
        if ((stage != NULL) &&
            (stage->state == FOC_DENGFOC_POWER_ARMED))
        {
            foc_dengfoc_power_stage_latch_fault(stage);
        }
        return false;
    }
    for (phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase)
    {
        if (command->duty_count[phase] > stage->maximum_duty_count)
        {
            foc_dengfoc_power_stage_latch_fault(stage);
            return false;
        }
        if (stage->maximum_active_duty_deviation_count != 0U)
        {
            const uint32_t neutral = (stage->maximum_duty_count + 1U) / 2U;
            const uint32_t deviation = command->duty_count[phase] > neutral
                                           ? command->duty_count[phase] - neutral
                                           : neutral - command->duty_count[phase];
            if (deviation > stage->maximum_active_duty_deviation_count)
            {
                foc_dengfoc_power_stage_latch_fault(stage);
                return false;
            }
        }
    }
    if (!stage->ops.write_pwm(
            stage->ops.context, stage->axis, command->duty_count) ||
        !stage->ops.read_pwm(stage->ops.context, stage->axis, actual))
    {
        foc_dengfoc_power_stage_latch_fault(stage);
        return false;
    }
    for (phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase)
    {
        if (actual[phase] != command->duty_count[phase])
        {
            foc_dengfoc_power_stage_latch_fault(stage);
            return false;
        }
    }
    {
        bool disabled = true;
        if (!stage->ops.driver_is_disabled(stage->ops.context, &disabled) ||
            disabled)
        {
            foc_dengfoc_power_stage_latch_fault(stage);
            return false;
        }
    }
    stage->last_sequence = command->sequence;
    return true;
}

bool foc_dengfoc_power_stage_shutdown(foc_dengfoc_power_stage_t *stage)
{
    bool ok;
    bool keep_fault_latched;
    if ((stage == NULL) || (stage->profile == NULL) ||
        !ops_valid(&stage->ops))
    {
        return false;
    }
    keep_fault_latched =
        stage->state == FOC_DENGFOC_POWER_FAULT_LATCHED;
    ok = stage->ops.force_driver_disabled(stage->ops.context);
    if (stage->pwm_configured != 0U)
    {
        ok = write_and_verify_zero(stage) && ok;
    }
    ok = driver_disabled(stage) && ok;
    stage->last_sequence = 0U;
    if (ok)
    {
        stage->state = keep_fault_latched
                           ? FOC_DENGFOC_POWER_FAULT_LATCHED
                           : FOC_DENGFOC_POWER_SAFE_DISABLED;
        return true;
    }
    foc_dengfoc_power_stage_latch_fault(stage);
    return false;
}

bool foc_dengfoc_power_stage_clear_fault(foc_dengfoc_power_stage_t *stage)
{
    if ((stage == NULL) ||
        (stage->state != FOC_DENGFOC_POWER_FAULT_LATCHED) ||
        !stage->ops.force_driver_disabled(stage->ops.context) ||
        !driver_disabled(stage))
    {
        return false;
    }
    if ((stage->pwm_configured != 0U) && !write_and_verify_zero(stage))
    {
        return false;
    }
    stage->last_sequence = 0U;
    stage->state = FOC_DENGFOC_POWER_SAFE_DISABLED;
    return true;
}
