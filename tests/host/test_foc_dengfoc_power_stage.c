#include "foc_board_dengfoc_v04.h"
#include "foc_dengfoc_power_stage.h"

#include <assert.h>
#include <string.h>

typedef struct
{
    bool driver_disabled;
    bool fail_configure_pwm;
    uint32_t configure_adc_calls;
    uint32_t configure_pwm_calls;
    uint32_t enable_calls;
    uint32_t disable_calls;
    uint32_t capabilities;
    uint32_t duty[FOC_DENGFOC_PHASE_COUNT];
} fake_power_t;

static bool fake_force_disabled(void *context)
{
    fake_power_t *fake = (fake_power_t *)context;
    fake->disable_calls += 1U;
    fake->driver_disabled = true;
    return true;
}

static bool fake_driver_disabled(void *context, bool *disabled)
{
    fake_power_t *fake = (fake_power_t *)context;
    if (disabled == NULL)
    {
        return false;
    }
    *disabled = fake->driver_disabled;
    return true;
}

static bool fake_configure_adc(
    void *context,
    uint32_t axis,
    uint32_t current_a_gpio,
    uint32_t current_b_gpio)
{
    fake_power_t *fake = (fake_power_t *)context;
    fake->configure_adc_calls += 1U;
    return fake->driver_disabled && (axis == 0U) &&
           (current_a_gpio == 39U) && (current_b_gpio == 36U);
}

static bool fake_configure_pwm(
    void *context,
    uint32_t axis,
    const uint32_t gpio[FOC_DENGFOC_PHASE_COUNT],
    uint32_t frequency_hz,
    uint32_t resolution_bits)
{
    fake_power_t *fake = (fake_power_t *)context;
    fake->configure_pwm_calls += 1U;
    return !fake->fail_configure_pwm && fake->driver_disabled &&
           (axis == 0U) && (gpio != NULL) &&
           (gpio[0] == 32U) && (gpio[1] == 33U) && (gpio[2] == 25U) &&
           (frequency_hz == 30000U) && (resolution_bits == 8U);
}

static bool fake_write_pwm(
    void *context,
    uint32_t axis,
    const uint32_t duty[FOC_DENGFOC_PHASE_COUNT])
{
    fake_power_t *fake = (fake_power_t *)context;
    if ((axis != 0U) || (duty == NULL))
    {
        return false;
    }
    (void)memcpy(fake->duty, duty, sizeof(fake->duty));
    return true;
}

static bool fake_read_pwm(
    void *context,
    uint32_t axis,
    uint32_t duty[FOC_DENGFOC_PHASE_COUNT])
{
    fake_power_t *fake = (fake_power_t *)context;
    if ((axis != 0U) || (duty == NULL))
    {
        return false;
    }
    (void)memcpy(duty, fake->duty, sizeof(fake->duty));
    return true;
}

static bool fake_set_enabled(void *context, bool enabled)
{
    fake_power_t *fake = (fake_power_t *)context;
    if (enabled)
    {
        fake->enable_calls += 1U;
        fake->driver_disabled = false;
    }
    else
    {
        fake->disable_calls += 1U;
        fake->driver_disabled = true;
    }
    return true;
}

static bool fake_read_capabilities(void *context, uint32_t *capabilities)
{
    fake_power_t *fake = (fake_power_t *)context;
    if (capabilities == NULL)
    {
        return false;
    }
    *capabilities = fake->capabilities;
    return true;
}

static foc_dengfoc_power_ops_t fake_ops(fake_power_t *fake)
{
    const foc_dengfoc_power_ops_t ops = {
        .context = fake,
        .force_driver_disabled = fake_force_disabled,
        .driver_is_disabled = fake_driver_disabled,
        .configure_current_adc = fake_configure_adc,
        .configure_pwm = fake_configure_pwm,
        .write_pwm = fake_write_pwm,
        .read_pwm = fake_read_pwm,
        .set_driver_enabled = fake_set_enabled,
        .read_capabilities = fake_read_capabilities,
    };
    return ops;
}

static void default_policy_never_enables_driver(void)
{
    fake_power_t fake;
    foc_dengfoc_power_stage_t stage;
    const foc_dengfoc_power_policy_t policy = {
        .allow_actuation = 0U,
        .operation = FOC_DENGFOC_OPERATION_DISABLED,
        .required_capabilities = 0U,
        .maximum_active_duty_deviation_count = 0U,
    };
    (void)memset(&fake, 0, sizeof(fake));
    fake.driver_disabled = true;
    const foc_dengfoc_power_ops_t ops = fake_ops(&fake);
    assert(foc_dengfoc_power_stage_init(
        &stage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &policy,
        &ops));
    assert(stage.state == FOC_DENGFOC_POWER_SAFE_DISABLED);
    assert(fake.enable_calls == 0U);
    assert(fake.configure_pwm_calls == 0U);
    assert(foc_dengfoc_power_stage_prepare(&stage));
    assert(stage.state == FOC_DENGFOC_POWER_PWM_READY);
    assert(fake.configure_adc_calls == 1U);
    assert(fake.configure_pwm_calls == 1U);
    assert(stage.platform_capabilities == 0U);
    assert(stage.missing_capabilities == 0U);
    assert(!foc_dengfoc_power_stage_arm(&stage, stage.fault_epoch));
    assert(stage.state == FOC_DENGFOC_POWER_PWM_READY);
    assert(fake.driver_disabled);
    assert(fake.enable_calls == 0U);
    assert(foc_dengfoc_power_stage_shutdown(&stage));
    assert(stage.state == FOC_DENGFOC_POWER_SAFE_DISABLED);
}

static void authorized_path_is_sequenced_and_faults_safe(void)
{
    fake_power_t fake;
    foc_dengfoc_power_stage_t stage;
    foc_dengfoc_pwm_command_t command = {
        .sequence = 1U,
        .duty_count = {10U, 20U, 30U},
    };
    const foc_dengfoc_power_policy_t policy = {
        .allow_actuation = 1U,
        .operation = FOC_DENGFOC_OPERATION_CURRENT_LOOP,
        .required_capabilities =
            FOC_DENGFOC_CURRENT_LOOP_REQUIRED_CAPABILITIES,
        .maximum_active_duty_deviation_count = 0U,
    };
    (void)memset(&fake, 0, sizeof(fake));
    fake.driver_disabled = true;
    fake.capabilities = FOC_DENGFOC_CAP_KNOWN_MASK;
    const foc_dengfoc_power_ops_t ops = fake_ops(&fake);
    assert(foc_dengfoc_power_stage_init(
        &stage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &policy,
        &ops));
    assert(foc_dengfoc_power_stage_prepare(&stage));
    assert(foc_dengfoc_power_stage_arm(&stage, 0U));
    assert(stage.state == FOC_DENGFOC_POWER_ARMED);
    assert(!fake.driver_disabled);
    assert(foc_dengfoc_power_stage_commit(&stage, 0U, &command));
    assert(fake.duty[0] == 10U);
    assert(fake.duty[1] == 20U);
    assert(fake.duty[2] == 30U);

    command.sequence = 2U;
    command.duty_count[1] = 256U;
    assert(!foc_dengfoc_power_stage_commit(&stage, 0U, &command));
    assert(stage.state == FOC_DENGFOC_POWER_FAULT_LATCHED);
    assert(stage.fault_epoch == 1U);
    assert(fake.driver_disabled);
    assert(fake.duty[0] == 0U && fake.duty[1] == 0U && fake.duty[2] == 0U);
    assert(foc_dengfoc_power_stage_shutdown(&stage));
    assert(stage.state == FOC_DENGFOC_POWER_FAULT_LATCHED);
    assert(foc_dengfoc_power_stage_clear_fault(&stage));
    assert(stage.state == FOC_DENGFOC_POWER_SAFE_DISABLED);
}

static void prepare_failure_latches_fault_and_disables(void)
{
    fake_power_t fake;
    foc_dengfoc_power_stage_t stage;
    const foc_dengfoc_power_policy_t policy = {
        .allow_actuation = 0U,
        .operation = FOC_DENGFOC_OPERATION_DISABLED,
        .required_capabilities = 0U,
        .maximum_active_duty_deviation_count = 0U,
    };
    (void)memset(&fake, 0, sizeof(fake));
    fake.driver_disabled = true;
    fake.fail_configure_pwm = true;
    const foc_dengfoc_power_ops_t ops = fake_ops(&fake);
    assert(foc_dengfoc_power_stage_init(
        &stage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &policy,
        &ops));
    assert(!foc_dengfoc_power_stage_prepare(&stage));
    assert(stage.state == FOC_DENGFOC_POWER_FAULT_LATCHED);
    assert(stage.fault_epoch == 1U);
    assert(fake.driver_disabled);
}

static void current_loop_missing_capability_is_fail_closed(void)
{
    fake_power_t fake;
    foc_dengfoc_power_stage_t stage;
    const foc_dengfoc_power_policy_t policy = {
        .allow_actuation = 1U,
        .operation = FOC_DENGFOC_OPERATION_CURRENT_LOOP,
        .required_capabilities =
            FOC_DENGFOC_CURRENT_LOOP_REQUIRED_CAPABILITIES,
        .maximum_active_duty_deviation_count = 0U,
    };
    (void)memset(&fake, 0, sizeof(fake));
    fake.driver_disabled = true;
    fake.capabilities = FOC_DENGFOC_CAP_CENTER_ALIGNED_PWM |
                        FOC_DENGFOC_CAP_PWM_CYCLE_ISR;
    const foc_dengfoc_power_ops_t ops = fake_ops(&fake);
    assert(foc_dengfoc_power_stage_init(
        &stage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &policy,
        &ops));
    assert(foc_dengfoc_power_stage_prepare(&stage));
    assert(stage.platform_capabilities == fake.capabilities);
    assert(stage.missing_capabilities ==
           (FOC_DENGFOC_CURRENT_LOOP_REQUIRED_CAPABILITIES &
            ~fake.capabilities));
    assert(!foc_dengfoc_power_stage_arm(&stage, stage.fault_epoch));
    assert(stage.state == FOC_DENGFOC_POWER_FAULT_LATCHED);
    assert(fake.driver_disabled);
    assert(fake.enable_calls == 0U);
}

static void current_loop_policy_cannot_weaken_required_capabilities(void)
{
    fake_power_t fake;
    foc_dengfoc_power_stage_t stage;
    const foc_dengfoc_power_policy_t weakened_policy = {
        .allow_actuation = 1U,
        .operation = FOC_DENGFOC_OPERATION_CURRENT_LOOP,
        .required_capabilities =
            FOC_DENGFOC_CURRENT_LOOP_REQUIRED_CAPABILITIES &
            ~FOC_DENGFOC_CAP_HARDWARE_FAST_SHUTDOWN,
        .maximum_active_duty_deviation_count = 0U,
    };
    (void)memset(&fake, 0, sizeof(fake));
    fake.driver_disabled = true;
    fake.capabilities = FOC_DENGFOC_CAP_KNOWN_MASK;
    const foc_dengfoc_power_ops_t ops = fake_ops(&fake);
    assert(!foc_dengfoc_power_stage_init(
        &stage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &weakened_policy,
        &ops));
    assert(fake.enable_calls == 0U);
}

static void voltage_commissioning_is_separate_and_duty_bounded(void)
{
    fake_power_t fake;
    foc_dengfoc_power_stage_t stage;
    foc_dengfoc_pwm_command_t command = {
        .sequence = 1U,
        .duty_count = {140U, 122U, 122U},
    };
    const foc_dengfoc_power_policy_t policy = {
        .allow_actuation = 1U,
        .operation = FOC_DENGFOC_OPERATION_VOLTAGE_COMMISSIONING,
        .required_capabilities =
            FOC_DENGFOC_VOLTAGE_COMMISSIONING_REQUIRED_CAPABILITIES,
        .maximum_active_duty_deviation_count = 16U,
    };
    (void)memset(&fake, 0, sizeof(fake));
    fake.driver_disabled = true;
    fake.capabilities =
        FOC_DENGFOC_VOLTAGE_COMMISSIONING_REQUIRED_CAPABILITIES;
    const foc_dengfoc_power_ops_t ops = fake_ops(&fake);
    assert(foc_dengfoc_power_stage_init(
        &stage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &policy,
        &ops));
    assert(foc_dengfoc_power_stage_prepare(&stage));
    assert(foc_dengfoc_power_stage_arm(&stage, 0U));
    assert(foc_dengfoc_power_stage_commit(&stage, 0U, &command));
    command.sequence = 2U;
    command.duty_count[0] = 145U;
    assert(!foc_dengfoc_power_stage_commit(&stage, 0U, &command));
    assert(stage.state == FOC_DENGFOC_POWER_FAULT_LATCHED);
    assert(fake.driver_disabled);
}

static void voltage_commissioning_policy_cannot_be_unbounded_or_weakened(void)
{
    fake_power_t fake;
    foc_dengfoc_power_stage_t stage;
    foc_dengfoc_power_policy_t policy = {
        .allow_actuation = 1U,
        .operation = FOC_DENGFOC_OPERATION_VOLTAGE_COMMISSIONING,
        .required_capabilities =
            FOC_DENGFOC_VOLTAGE_COMMISSIONING_REQUIRED_CAPABILITIES,
        .maximum_active_duty_deviation_count = 0U,
    };
    (void)memset(&fake, 0, sizeof(fake));
    fake.driver_disabled = true;
    fake.capabilities =
        FOC_DENGFOC_VOLTAGE_COMMISSIONING_REQUIRED_CAPABILITIES;
    const foc_dengfoc_power_ops_t ops = fake_ops(&fake);
    assert(!foc_dengfoc_power_stage_init(
        &stage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &policy,
        &ops));
    assert(fake.enable_calls == 0U);

    policy.maximum_active_duty_deviation_count = 16U;
    policy.required_capabilities &=
        ~FOC_DENGFOC_CAP_PWM_DUTY_READBACK;
    assert(!foc_dengfoc_power_stage_init(
        &stage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &policy,
        &ops));
    assert(fake.enable_calls == 0U);
}

int main(void)
{
    default_policy_never_enables_driver();
    authorized_path_is_sequenced_and_faults_safe();
    prepare_failure_latches_fault_and_disables();
    current_loop_missing_capability_is_fail_closed();
    current_loop_policy_cannot_weaken_required_capabilities();
    voltage_commissioning_is_separate_and_duty_bounded();
    voltage_commissioning_policy_cannot_be_unbounded_or_weakened();
    return 0;
}
