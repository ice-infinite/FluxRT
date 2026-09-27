#include "foc_lsi_actuation_executor.h"

#include <math.h>
#include <string.h>

static void foc_lsi_executor_safe_output(
    foc_lsi_executor_output_t *output,
    foc_lsi_executor_result_t result,
    foc_status_t planner_status)
{
    if (output == 0)
    {
        return;
    }
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->version = FOC_LSI_EXECUTOR_OUTPUT_VERSION;
    output->action = FOC_LSI_EXECUTOR_ACTION_SAFE;
    output->result = result;
    output->planner_status = planner_status;
    output->plan.struct_size = sizeof(output->plan);
    output->plan.version = FOC_LSI_ACTUATION_OUTPUT_VERSION;
    output->plan.safe_output_required = 1U;
}

#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
static uint32_t foc_lsi_bool_is_valid(uint32_t value)
{
    return (value <= 1U) ? 1U : 0U;
}

static uint32_t foc_lsi_executor_config_is_valid(
    const foc_lsi_executor_config_t *platform,
    const foc_lsi_actuation_config_t *actuation)
{
    return ((platform != 0) &&
            (actuation != 0) &&
            (platform->struct_size == sizeof(*platform)) &&
            (platform->version == FOC_LSI_EXECUTOR_CONFIG_VERSION) &&
            (actuation->struct_size == sizeof(*actuation)) &&
            (actuation->version == FOC_LSI_ACTUATION_CONFIG_VERSION) &&
            (platform->sample_rate_hz == actuation->sample_rate_hz) &&
            (platform->sample_rate_hz == 12000U) &&
            (platform->pwm_period_ticks > 0U) &&
            (platform->pwm_period_ticks <= 65535U) &&
            (platform->adc_max_code > 0U) &&
            (platform->adc_max_code <= 65535U) &&
            (platform->actuation_delay_control_ticks ==
             actuation->actuation_delay_control_ticks) &&
            (platform->actuation_delay_control_ticks == 1U) &&
            ((uint32_t)platform->current_u_offset_raw <= platform->adc_max_code) &&
            ((uint32_t)platform->current_v_offset_raw <= platform->adc_max_code) &&
            isfinite(platform->current_counts_per_amp) &&
            (platform->current_counts_per_amp > 0.0f) &&
            isfinite(platform->bus_volts_per_count) &&
            (platform->bus_volts_per_count > 0.0f) &&
            isfinite(platform->minimum_duty) &&
            isfinite(platform->maximum_duty) &&
            (platform->minimum_duty >= actuation->minimum_duty) &&
            (platform->maximum_duty <= actuation->maximum_duty) &&
            (platform->maximum_duty > platform->minimum_duty) &&
            isfinite(actuation->stator_resistance_ohm) &&
            (actuation->stator_resistance_ohm > 0.0f) &&
            (actuation->stator_resistance_ohm <= 20.0f) &&
            isfinite(actuation->maximum_bias_current_a) &&
            (actuation->maximum_bias_current_a > 0.0f) &&
            (actuation->maximum_bias_current_a <= 0.2f) &&
            isfinite(actuation->maximum_perturbation_voltage_v) &&
            (actuation->maximum_perturbation_voltage_v > 0.0f) &&
            (actuation->maximum_perturbation_voltage_v <= 0.4f) &&
            isfinite(actuation->current_trip_a) &&
            (actuation->current_trip_a > actuation->maximum_bias_current_a) &&
            (actuation->current_trip_a <= 1.15f) &&
            isfinite(actuation->minimum_bus_voltage_v) &&
            isfinite(actuation->maximum_bus_voltage_v) &&
            (actuation->minimum_bus_voltage_v >= 7.0f) &&
            (actuation->maximum_bus_voltage_v <= 18.0f) &&
            (actuation->maximum_bus_voltage_v >
             actuation->minimum_bus_voltage_v) &&
            isfinite(actuation->minimum_duty) &&
            isfinite(actuation->maximum_duty) &&
            (actuation->minimum_duty >= 0.03f) &&
            (actuation->maximum_duty <= 0.97f) &&
            (actuation->maximum_duty > actuation->minimum_duty)) ? 1U : 0U;
}

static uint32_t foc_lsi_executor_is_valid(const foc_lsi_executor_t *executor)
{
    return ((executor != 0) &&
            (executor->struct_size == sizeof(*executor)) &&
            (executor->version == FOC_LSI_EXECUTOR_VERSION) &&
            (executor->initialized != 0U) &&
            (foc_lsi_executor_config_is_valid(&executor->platform,
                                              &executor->actuation) != 0U)) ? 1U : 0U;
}

static uint32_t foc_lsi_raw_is_valid(
    const foc_lsi_executor_t *executor,
    const foc_lsi_raw_sample_t *raw)
{
    if ((raw == 0) ||
        ((uint32_t)raw->current_u_raw > executor->platform.adc_max_code) ||
        ((uint32_t)raw->current_v_raw > executor->platform.adc_max_code) ||
        ((uint32_t)raw->bus_voltage_raw > executor->platform.adc_max_code) ||
        ((uint32_t)raw->pwm_period_ticks != executor->platform.pwm_period_ticks) ||
        (raw->compare_u > raw->pwm_period_ticks) ||
        (raw->compare_v > raw->pwm_period_ticks) ||
        (raw->compare_w > raw->pwm_period_ticks) ||
        ((raw->flags & FOC_LSI_RAW_FLAG_ADC_VALID) == 0U))
    {
        return 0U;
    }
    return 1U;
}

static uint16_t foc_lsi_duty_to_compare(float duty, uint32_t period)
{
    return (uint16_t)((duty * (float)period) + 0.5f);
}
#endif

foc_status_t foc_lsi_executor_init(
    foc_lsi_executor_t *executor,
    const foc_lsi_executor_config_t *platform,
    const foc_lsi_actuation_config_t *actuation)
{
    if (executor == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    memset(executor, 0, sizeof(*executor));
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    if (foc_lsi_executor_config_is_valid(platform, actuation) == 0U)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    executor->struct_size = sizeof(*executor);
    executor->version = FOC_LSI_EXECUTOR_VERSION;
    executor->initialized = 1U;
    executor->platform = *platform;
    executor->actuation = *actuation;
    return FOC_STATUS_OK;
#else
    (void)platform;
    (void)actuation;
    return FOC_STATUS_DISABLED;
#endif
}

foc_status_t foc_lsi_executor_step(
    foc_lsi_executor_t *executor,
    const foc_lsi_executor_command_t *command,
    const foc_lsi_raw_sample_t *raw,
    foc_lsi_executor_output_t *output)
{
    foc_lsi_executor_safe_output(output,
                                 FOC_LSI_EXECUTOR_RESULT_INVALID_ARGUMENT,
                                 FOC_STATUS_INVALID_ARGUMENT);
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    if ((output == 0) ||
        (foc_lsi_executor_is_valid(executor) == 0U) ||
        (command == 0) ||
        (command->struct_size != sizeof(*command)) ||
        (command->version != FOC_LSI_EXECUTOR_COMMAND_VERSION) ||
        (command->drive_request > FOC_LSI_DRIVE_ABI_PULSE_NEGATIVE) ||
        (foc_lsi_bool_is_valid(command->force_safe_output) == 0U) ||
        (foc_lsi_bool_is_valid(command->capture_ready) == 0U) ||
        (foc_lsi_bool_is_valid(command->capture_full) == 0U) ||
        (foc_lsi_bool_is_valid(command->capture_error) == 0U) ||
        (foc_lsi_bool_is_valid(command->hardware_fault) == 0U) ||
        (foc_lsi_bool_is_valid(command->software_trip) == 0U) ||
        !isfinite(command->requested_bias_current_a) ||
        !isfinite(command->requested_perturbation_voltage_v))
    {
        if (executor != 0)
        {
            executor->pending_ledger_valid = 0U;
        }
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    /* Fault/error paths do not trust the raw sample enough to perform a ledger check. */
    if ((command->capture_error != 0U) ||
        (command->hardware_fault != 0U) ||
        (command->software_trip != 0U) ||
        ((raw != 0) &&
         ((raw->flags & (FOC_LSI_RAW_FLAG_HARDWARE_FAULT |
                         FOC_LSI_RAW_FLAG_SOFTWARE_TRIP)) != 0U)))
    {
        executor->pending_ledger_valid = 0U;
        foc_lsi_executor_safe_output(
            output,
            (command->capture_error != 0U) ?
                FOC_LSI_EXECUTOR_RESULT_CAPTURE_ERROR :
                FOC_LSI_EXECUTOR_RESULT_FAULT,
            FOC_STATUS_HARDWARE_FAULT);
        return FOC_STATUS_HARDWARE_FAULT;
    }

    if (foc_lsi_raw_is_valid(executor, raw) == 0U)
    {
        executor->pending_ledger_valid = 0U;
        foc_lsi_executor_safe_output(output,
                                     FOC_LSI_EXECUTOR_RESULT_RAW_SAMPLE_INVALID,
                                     FOC_STATUS_HARDWARE_FAULT);
        return FOC_STATUS_HARDWARE_FAULT;
    }

    output->source_control_tick = raw->control_tick;
    output->pwm_period_ticks = raw->pwm_period_ticks;
    if (executor->pending_ledger_valid != 0U)
    {
        uint16_t pulse_flags = raw->flags &
            (FOC_LSI_RAW_FLAG_PULSE_POSITIVE |
             FOC_LSI_RAW_FLAG_PULSE_NEGATIVE);
        uint16_t expected_pulse_flags = 0U;

        if (executor->pending_drive_request ==
            FOC_LSI_DRIVE_ABI_PULSE_POSITIVE)
        {
            expected_pulse_flags = FOC_LSI_RAW_FLAG_PULSE_POSITIVE;
        }
        else if (executor->pending_drive_request ==
                 FOC_LSI_DRIVE_ABI_PULSE_NEGATIVE)
        {
            expected_pulse_flags = FOC_LSI_RAW_FLAG_PULSE_NEGATIVE;
        }
        output->ledger_checked = 1U;
        if ((raw->control_tick !=
             executor->pending_expected_active_control_tick) ||
            (raw->compare_u != executor->pending_compare_u) ||
            (raw->compare_v != executor->pending_compare_v) ||
            (raw->compare_w != executor->pending_compare_w) ||
            ((raw->flags & FOC_LSI_RAW_FLAG_DRIVE_ACTIVE) == 0U) ||
            (pulse_flags != expected_pulse_flags))
        {
            executor->pending_ledger_valid = 0U;
            output->result = FOC_LSI_EXECUTOR_RESULT_LEDGER_MISMATCH;
            output->planner_status = FOC_STATUS_HARDWARE_FAULT;
            return FOC_STATUS_HARDWARE_FAULT;
        }
        output->ledger_matched = 1U;
        executor->pending_ledger_valid = 0U;
    }

    if (command->capture_full != 0U)
    {
        output->result = FOC_LSI_EXECUTOR_RESULT_CAPTURE_COMPLETE;
        output->planner_status = FOC_STATUS_DISABLED;
        return FOC_STATUS_DISABLED;
    }
    if ((command->capture_ready == 0U) &&
        (command->drive_request != FOC_LSI_DRIVE_ABI_OFF))
    {
        output->result = FOC_LSI_EXECUTOR_RESULT_CAPTURE_NOT_READY;
        output->planner_status = FOC_STATUS_NOT_CONFIGURED;
        return FOC_STATUS_NOT_CONFIGURED;
    }

    output->phase_u_current_a =
        ((float)((int32_t)executor->platform.current_u_offset_raw -
                 (int32_t)raw->current_u_raw)) /
        executor->platform.current_counts_per_amp;
    output->bus_voltage_v =
        (float)raw->bus_voltage_raw * executor->platform.bus_volts_per_count;

    {
        foc_lsi_actuation_input_t plan_input = {0};
        foc_lsi_actuation_output_t plan = {0};
        foc_status_t planner_status;
        uint16_t compare_u;
        uint16_t compare_v;
        uint16_t compare_w;

        plan_input.struct_size = sizeof(plan_input);
        plan_input.version = FOC_LSI_ACTUATION_INPUT_VERSION;
        plan_input.drive_request = command->drive_request;
        plan_input.force_safe_output = command->force_safe_output;
        plan_input.capture_ready = command->capture_ready;
        plan_input.capture_full = command->capture_full;
        plan_input.hardware_fault = command->hardware_fault;
        plan_input.software_trip = command->software_trip;
        plan_input.control_tick = raw->control_tick;
        plan_input.requested_bias_current_a = command->requested_bias_current_a;
        plan_input.requested_perturbation_voltage_v =
            command->requested_perturbation_voltage_v;
        plan_input.phase_u_current_a = output->phase_u_current_a;
        plan_input.bus_voltage_v = output->bus_voltage_v;
        planner_status = foc_rust_lsi_plan(&executor->actuation,
                                           &plan_input,
                                           &plan);
        output->planner_status = planner_status;
        output->plan = plan;
        if (planner_status != FOC_STATUS_OK)
        {
            output->result = FOC_LSI_EXECUTOR_RESULT_PLANNER_REJECTED;
            return planner_status;
        }

        if (command->drive_request == FOC_LSI_DRIVE_ABI_OFF)
        {
            if ((plan.safe_output_required == 0U) ||
                (plan.drive_active != 0U) ||
                (plan.duty_u != 0.0f) ||
                (plan.duty_v != 0.0f) ||
                (plan.duty_w != 0.0f))
            {
                output->result = FOC_LSI_EXECUTOR_RESULT_PLANNER_REJECTED;
                output->planner_status = FOC_STATUS_INVALID_ARGUMENT;
                return FOC_STATUS_INVALID_ARGUMENT;
            }
            output->result = FOC_LSI_EXECUTOR_RESULT_SAFE_REQUESTED;
            return FOC_STATUS_OK;
        }

        if ((plan.safe_output_required != 0U) ||
            (plan.drive_active == 0U) ||
            (plan.source_control_tick != raw->control_tick) ||
            (plan.expected_active_control_tick !=
             raw->control_tick + executor->platform.actuation_delay_control_ticks) ||
            !isfinite(plan.duty_u) ||
            !isfinite(plan.duty_v) ||
            !isfinite(plan.duty_w) ||
            !(plan.duty_u >= executor->platform.minimum_duty &&
              plan.duty_u <= executor->platform.maximum_duty) ||
            !(plan.duty_v >= executor->platform.minimum_duty &&
              plan.duty_v <= executor->platform.maximum_duty) ||
            !(plan.duty_w >= executor->platform.minimum_duty &&
              plan.duty_w <= executor->platform.maximum_duty))
        {
            output->result = FOC_LSI_EXECUTOR_RESULT_PLANNER_REJECTED;
            output->planner_status = FOC_STATUS_INVALID_ARGUMENT;
            return FOC_STATUS_INVALID_ARGUMENT;
        }

        compare_u = foc_lsi_duty_to_compare(
            plan.duty_u, executor->platform.pwm_period_ticks);
        compare_v = foc_lsi_duty_to_compare(
            plan.duty_v, executor->platform.pwm_period_ticks);
        compare_w = foc_lsi_duty_to_compare(
            plan.duty_w, executor->platform.pwm_period_ticks);
        if (((uint32_t)compare_u > executor->platform.pwm_period_ticks) ||
            ((uint32_t)compare_v > executor->platform.pwm_period_ticks) ||
            ((uint32_t)compare_w > executor->platform.pwm_period_ticks))
        {
            output->result = FOC_LSI_EXECUTOR_RESULT_PLANNER_REJECTED;
            output->planner_status = FOC_STATUS_INVALID_ARGUMENT;
            return FOC_STATUS_INVALID_ARGUMENT;
        }

        output->action = FOC_LSI_EXECUTOR_ACTION_WRITE_PRELOAD;
        output->result = FOC_LSI_EXECUTOR_RESULT_PRELOAD_READY;
        output->source_control_tick = plan.source_control_tick;
        output->expected_active_control_tick =
            plan.expected_active_control_tick;
        output->compare_u = compare_u;
        output->compare_v = compare_v;
        output->compare_w = compare_w;
        executor->pending_ledger_valid = 1U;
        executor->pending_source_control_tick = plan.source_control_tick;
        executor->pending_expected_active_control_tick =
            plan.expected_active_control_tick;
        executor->pending_drive_request = command->drive_request;
        executor->pending_compare_u = compare_u;
        executor->pending_compare_v = compare_v;
        executor->pending_compare_w = compare_w;
        return FOC_STATUS_OK;
    }
#else
    (void)executor;
    (void)command;
    (void)raw;
    if (output == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    foc_lsi_executor_safe_output(output,
                                 FOC_LSI_EXECUTOR_RESULT_CAPTURE_NOT_READY,
                                 FOC_STATUS_DISABLED);
    return FOC_STATUS_DISABLED;
#endif
}
