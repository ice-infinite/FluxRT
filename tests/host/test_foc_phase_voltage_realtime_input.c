#include "foc_phase_voltage_realtime_input.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_rust_bridge.h"

static foc_phase_voltage_realtime_input_request_t valid_request(
    uint32_t control_sequence,
    uint32_t age_ticks)
{
    foc_phase_voltage_realtime_input_request_t request = {0};
    request.struct_size = sizeof(request);
    request.version = FOC_PHASE_VOLTAGE_REALTIME_INPUT_ASSEMBLER_VERSION;
    request.control_sequence = control_sequence;
    request.phase_voltage_age_ticks = age_ticks;
    request.hardware_fault_flags = FOC_REALTIME_HW_FAULT_DRIVER |
                                   FOC_REALTIME_HW_FAULT_ADC_SAMPLE_ERROR;
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL;
    request.actual_dt_s = 1.0f / 12000.0f;
    request.legacy_feedback.phase_current_a = 0.25f;
    request.legacy_feedback.phase_current_b = -0.10f;
    request.legacy_feedback.phase_current_c = -0.15f;
    request.legacy_feedback.dc_bus_voltage = 12.3f;
    request.legacy_feedback.electrical_angle_rad = 0.75f;
    return request;
}

static foc_phase_voltage_platform_output_t nominal_output(
    uint32_t phase_sequence,
    foc_phase_voltage_request_t request_mode)
{
    foc_phase_voltage_platform_output_t output = {0};
    output.struct_size = sizeof(output);
    output.version = FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION;
    output.flags = FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID |
                   FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID |
                   FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY |
                   FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT;
    output.model_source = FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL;
    output.observer_eligible = 0U;
    output.phase_u_mv = 4469U;
    output.phase_v_mv = 8938U;
    output.phase_w_mv = 13407U;
    output.phase_sequence = phase_sequence;
    output.quality.struct_size = sizeof(output.quality);
    output.quality.version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    output.quality.state = FOC_PHASE_VOLTAGE_QUALITY_UNCALIBRATED;
    output.quality.reason_mask = FOC_PHASE_VOLTAGE_REASON_UNCALIBRATED;
    output.quality.requested_mode = (uint32_t)request_mode;
    output.quality.measured_eligible = 0U;
    output.quality.sample_sequence = phase_sequence;

    if (request_mode == FOC_PHASE_VOLTAGE_REQUEST_MEASURED)
    {
        output.quality.selected_source =
            FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE;
        output.quality.unavailable_active = 1U;
        output.quality.unavailable_event_count = 1U;
    }
    else if (request_mode == FOC_PHASE_VOLTAGE_REQUEST_HYBRID)
    {
        output.quality.selected_source =
            FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL;
        output.quality.fallback_active = 1U;
        output.quality.fallback_event_count = 3U;
    }
    else
    {
        output.quality.selected_source =
            FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL;
    }
    return output;
}

static void assert_output_is_zero(const foc_realtime_input_t *output)
{
    const unsigned char *bytes = (const unsigned char *)output;
    size_t index;
    for (index = 0U; index < sizeof(*output); ++index)
    {
        assert(bytes[index] == 0U);
    }
}

static void expect_rejected(
    const foc_phase_voltage_realtime_input_request_t *request,
    const foc_phase_voltage_platform_output_t *adapter_output)
{
    foc_realtime_input_t output;
    memset(&output, 0xA5, sizeof(output));
    assert(foc_phase_voltage_realtime_input_assemble(
               request, adapter_output, &output) == 0U);
    assert_output_is_zero(&output);
}

static void assert_legacy_fields(
    const foc_phase_voltage_realtime_input_request_t *request,
    const foc_realtime_input_t *output)
{
    assert(output->struct_size == sizeof(*output));
    assert(output->version == FOC_REALTIME_INPUT_VERSION);
    assert(output->control_sequence == request->control_sequence);
    assert(output->hardware_fault_flags == request->hardware_fault_flags);
    assert(output->actual_dt_s == request->actual_dt_s);
    assert(output->phase_current_a ==
           request->legacy_feedback.phase_current_a);
    assert(output->phase_current_b ==
           request->legacy_feedback.phase_current_b);
    assert(output->phase_current_c ==
           request->legacy_feedback.phase_current_c);
    assert(output->dc_bus_voltage ==
           request->legacy_feedback.dc_bus_voltage);
    assert(output->electrical_angle_rad == 0.0f);
}

static void assert_legacy_baseline_equal(
    const foc_realtime_input_t *expected,
    const foc_realtime_input_t *actual)
{
    assert(actual->struct_size == expected->struct_size);
    assert(actual->version == expected->version);
    assert(actual->control_sequence == expected->control_sequence);
    assert(actual->valid_flags == expected->valid_flags);
    assert(actual->hardware_fault_flags == expected->hardware_fault_flags);
    assert(actual->actual_dt_s == expected->actual_dt_s);
    assert(actual->phase_current_a == expected->phase_current_a);
    assert(actual->phase_current_b == expected->phase_current_b);
    assert(actual->phase_current_c == expected->phase_current_c);
    assert(actual->dc_bus_voltage == expected->dc_bus_voltage);
    assert(actual->electrical_angle_rad == expected->electrical_angle_rad);
    assert(actual->sensor_temperature_c == expected->sensor_temperature_c);
}

static void test_nominal_policies_are_fail_closed(void)
{
    foc_phase_voltage_realtime_input_request_t request = valid_request(12U, 2U);
    foc_phase_voltage_platform_output_t adapter_output;
    foc_realtime_input_t output;

    adapter_output = nominal_output(
        10U, FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL;
    assert(foc_phase_voltage_realtime_input_assemble(
               &request, &adapter_output, &output) == 1U);
    assert_legacy_fields(&request, &output);
    assert(output.valid_flags ==
           (FOC_REALTIME_VALID_PHASE_CURRENTS |
            FOC_REALTIME_VALID_DC_BUS_VOLTAGE));
    assert(output.phase_voltage_provenance ==
           FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_ST_NOMINAL);
    assert(output.phase_voltage_quality_state ==
           FOC_REALTIME_PHASE_VOLTAGE_QUALITY_UNCALIBRATED);
    assert(output.phase_voltage_reason_mask ==
           FOC_REALTIME_PHASE_VOLTAGE_REASON_UNCALIBRATED);
    assert(output.observer_voltage_selection ==
           FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL);
    assert(output.phase_voltage_a_v == 0.0f);

    adapter_output = nominal_output(
        10U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_MEASURED;
    assert(foc_phase_voltage_realtime_input_assemble(
               &request, &adapter_output, &output) == 1U);
    assert(output.observer_voltage_selection ==
           FOC_REALTIME_OBSERVER_VOLTAGE_UNAVAILABLE);
    assert((output.valid_flags & FOC_REALTIME_VALID_PHASE_VOLTAGES) == 0U);
    assert(output.phase_voltage_a_v == 0.0f);

    adapter_output = nominal_output(
        10U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    assert(foc_phase_voltage_realtime_input_assemble(
               &request, &adapter_output, &output) == 1U);
    assert(output.observer_voltage_selection ==
           FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL);
    assert(output.phase_voltage_fallback_event_count == 3U);
    assert((output.valid_flags & FOC_REALTIME_VALID_PHASE_VOLTAGES) == 0U);
}

static void test_wrapping_age_and_mismatch_fail_closed(void)
{
    foc_phase_voltage_realtime_input_request_t request = valid_request(42U, 1U);
    foc_phase_voltage_platform_output_t adapter_output = nominal_output(
        41U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    foc_realtime_input_t output;

    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    assert(foc_phase_voltage_realtime_input_assemble(
               &request, &adapter_output, &output) == 1U);
    assert((output.valid_flags & FOC_REALTIME_VALID_PHASE_VOLTAGES) == 0U);

    request = valid_request(42U, 0U);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL;
    adapter_output = nominal_output(
        42U, FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL);
    assert(foc_phase_voltage_realtime_input_assemble(
               &request, &adapter_output, &output) == 1U);
    assert(output.phase_voltage_sequence == 42U);
    assert(output.phase_voltage_age_ticks == 0U);

    request = valid_request(0U, 1U);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    adapter_output = nominal_output(
        UINT32_MAX, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    assert(foc_phase_voltage_realtime_input_assemble(
               &request, &adapter_output, &output) == 1U);
    assert(output.phase_voltage_sequence == UINT32_MAX);
    assert(output.phase_voltage_age_ticks == 1U);
    assert(output.observer_voltage_selection ==
           FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL);

    request = valid_request(1U, 2U);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL;
    adapter_output = nominal_output(
        UINT32_MAX, FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL);
    assert(foc_phase_voltage_realtime_input_assemble(
               &request, &adapter_output, &output) == 1U);
    assert(output.phase_voltage_sequence == UINT32_MAX);
    assert(output.phase_voltage_age_ticks == 2U);

    request.phase_voltage_age_ticks = 1U;
    expect_rejected(&request, &adapter_output);

    request = valid_request(8U, 3U);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    adapter_output = nominal_output(6U,
                                    FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    expect_rejected(&request, &adapter_output);
}

static void test_measured_cannot_be_forged(void)
{
    foc_phase_voltage_realtime_input_request_t request = valid_request(4U, 1U);
    foc_phase_voltage_platform_output_t adapter_output = nominal_output(
        3U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);

    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_MEASURED;
    adapter_output.quality.selected_source =
        FOC_PHASE_VOLTAGE_SELECTION_MEASURED;
    adapter_output.quality.measured_eligible = 1U;
    adapter_output.quality.unavailable_active = 0U;
    adapter_output.quality.state = FOC_PHASE_VOLTAGE_QUALITY_VALID;
    adapter_output.quality.reason_mask = 0U;
    expect_rejected(&request, &adapter_output);

    adapter_output = nominal_output(3U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    adapter_output.model_source = FOC_PHASE_VOLTAGE_MODEL_BOARD_CALIBRATED;
    adapter_output.flags &= ~(uint32_t)(
        FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY |
        FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT);
    adapter_output.observer_eligible = 1U;
    adapter_output.quality.selected_source =
        FOC_PHASE_VOLTAGE_SELECTION_MEASURED;
    adapter_output.quality.measured_eligible = 1U;
    adapter_output.quality.unavailable_active = 0U;
    adapter_output.quality.state = FOC_PHASE_VOLTAGE_QUALITY_VALID;
    adapter_output.quality.reason_mask = 0U;
    expect_rejected(&request, &adapter_output);

    adapter_output = nominal_output(3U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    adapter_output.quality.selected_source =
        FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE;
    adapter_output.quality.fallback_active = 0U;
    adapter_output.quality.unavailable_active = 1U;
    expect_rejected(&request, &adapter_output);

    adapter_output = nominal_output(3U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    expect_rejected(&request, &adapter_output);
}

static void test_v1_quality_model_joint_invariants(void)
{
    foc_phase_voltage_realtime_input_request_t request = valid_request(4U, 1U);
    foc_phase_voltage_platform_output_t adapter_output = nominal_output(
        3U, FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL);

    adapter_output.quality.state = FOC_PHASE_VOLTAGE_QUALITY_VALID;
    adapter_output.quality.reason_mask = 0U;
    expect_rejected(&request, &adapter_output);

    adapter_output = nominal_output(3U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    adapter_output.quality.state = FOC_PHASE_VOLTAGE_QUALITY_VALID;
    adapter_output.quality.reason_mask =
        FOC_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS;
    expect_rejected(&request, &adapter_output);

    adapter_output = nominal_output(
        3U, FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL);
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL;
    adapter_output.flags = FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT;
    adapter_output.model_source = FOC_PHASE_VOLTAGE_MODEL_NONE;
    adapter_output.phase_u_mv = 0U;
    adapter_output.phase_v_mv = 0U;
    adapter_output.phase_w_mv = 0U;
    adapter_output.quality.state = FOC_PHASE_VOLTAGE_QUALITY_VALID;
    adapter_output.quality.reason_mask = 0U;
    expect_rejected(&request, &adapter_output);
}

static void test_zero_fault_legacy_equivalence_128_snapshots(void)
{
    uint32_t index;

    for (index = 0U; index < 128U; ++index)
    {
        const foc_phase_voltage_request_t mode =
            ((index & 1U) == 0U) ?
                FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL :
                FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
        const uint32_t age = index % 3U;
        foc_phase_voltage_realtime_input_request_t request =
            valid_request(0xFFFFFF80U + index, age);
        foc_phase_voltage_platform_output_t adapter_output;
        foc_realtime_input_t expected;
        foc_realtime_input_t actual;

        request.hardware_fault_flags = 0U;
        request.requested_mode = (uint32_t)mode;
        request.actual_dt_s = (1.0f / 12000.0f) +
                              ((float)(index % 5U) * 0.000000001f);
        request.legacy_feedback.phase_current_a =
            ((float)index - 64.0f) * 0.01f;
        request.legacy_feedback.phase_current_b =
            -request.legacy_feedback.phase_current_a * 0.4f;
        request.legacy_feedback.phase_current_c =
            -(request.legacy_feedback.phase_current_a +
              request.legacy_feedback.phase_current_b);
        request.legacy_feedback.dc_bus_voltage =
            11.5f + ((float)(index % 9U) * 0.1f);
        adapter_output = nominal_output(
            request.control_sequence - age, mode);

        foc_realtime_input_from_legacy(&request.legacy_feedback,
                                       request.control_sequence,
                                       request.actual_dt_s,
                                       &expected);
        assert(foc_phase_voltage_realtime_input_assemble(
                   &request, &adapter_output, &actual) == 1U);
        assert_legacy_baseline_equal(&expected, &actual);
        assert(actual.hardware_fault_flags == 0U);
        assert((actual.valid_flags &
                FOC_REALTIME_VALID_PHASE_VOLTAGES) == 0U);
        assert(actual.phase_voltage_a_v == 0.0f);
        assert(actual.phase_voltage_b_v == 0.0f);
        assert(actual.phase_voltage_c_v == 0.0f);
    }
}

static void test_request_layout_flags_and_nonfinite_are_rejected(void)
{
    foc_phase_voltage_realtime_input_request_t request = valid_request(2U, 1U);
    foc_phase_voltage_platform_output_t adapter_output = nominal_output(
        1U, FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL);

    expect_rejected(0, &adapter_output);

    request.struct_size -= 1U;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.version += 1U;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.requested_mode = 99U;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.hardware_fault_flags |= (1UL << 31);
    expect_rejected(&request, &adapter_output);

    request = valid_request(2U, 1U);
    request.actual_dt_s = NAN;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.actual_dt_s = INFINITY;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.actual_dt_s = 0.0f;
    expect_rejected(&request, &adapter_output);

    request = valid_request(2U, 1U);
    request.legacy_feedback.phase_current_a = NAN;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.legacy_feedback.phase_current_b = INFINITY;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.legacy_feedback.phase_current_c = -INFINITY;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.legacy_feedback.dc_bus_voltage = NAN;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.legacy_feedback.dc_bus_voltage = 0.0f;
    expect_rejected(&request, &adapter_output);
    request = valid_request(2U, 1U);
    request.legacy_feedback.electrical_angle_rad = INFINITY;
    expect_rejected(&request, &adapter_output);
}

static void test_adapter_unknown_fields_are_rejected(void)
{
    foc_phase_voltage_realtime_input_request_t request = valid_request(2U, 1U);
    foc_phase_voltage_platform_output_t base = nominal_output(
        1U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    foc_phase_voltage_platform_output_t candidate;

    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    expect_rejected(&request, 0);

    candidate = base;
    candidate.struct_size -= 1U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.version += 1U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.flags |= (1UL << 31);
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.flags |= FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.model_source = 99U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.observer_eligible = 2U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.flags &=
        ~(uint32_t)FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.measured_eligible = 1U;
    expect_rejected(&request, &candidate);

    candidate = base;
    candidate.quality.struct_size -= 1U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.version += 1U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.state = 99U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.reason_mask |= (1UL << 31);
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.requested_mode = 99U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.selected_source = 99U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.fallback_active = 2U;
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.affected_phase_mask |= (1UL << 8);
    expect_rejected(&request, &candidate);
    candidate = base;
    candidate.quality.sample_sequence += 1U;
    expect_rejected(&request, &candidate);
}

static void test_known_invalid_adapter_snapshot_remains_fail_closed(void)
{
    foc_phase_voltage_realtime_input_request_t request = valid_request(1U, 1U);
    foc_phase_voltage_platform_output_t adapter_output = {0};
    foc_realtime_input_t output;

    adapter_output.struct_size = sizeof(adapter_output);
    adapter_output.version = FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION;
    adapter_output.flags = FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT |
                           FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED;
    adapter_output.model_source = FOC_PHASE_VOLTAGE_MODEL_NONE;
    adapter_output.phase_sequence = 0U;
    adapter_output.quality.struct_size = sizeof(adapter_output.quality);
    adapter_output.quality.version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    adapter_output.quality.state = FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE;
    adapter_output.quality.reason_mask =
        FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE;
    adapter_output.quality.requested_mode =
        FOC_PHASE_VOLTAGE_REQUEST_HYBRID;
    adapter_output.quality.selected_source =
        FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL;
    adapter_output.quality.fallback_active = 1U;
    adapter_output.quality.sample_sequence = 0U;
    adapter_output.quality.fallback_event_count = 1U;
    request.requested_mode = FOC_PHASE_VOLTAGE_REQUEST_HYBRID;

    assert(foc_phase_voltage_realtime_input_assemble(
               &request, &adapter_output, &output) == 1U);
    assert(output.observer_voltage_selection ==
           FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL);
    assert(output.phase_voltage_provenance ==
           FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_NONE);
    assert((output.valid_flags & FOC_REALTIME_VALID_PHASE_VOLTAGES) == 0U);
    assert(output.phase_voltage_quality_state ==
           FOC_REALTIME_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
    assert(output.phase_voltage_reason_mask ==
           FOC_REALTIME_PHASE_VOLTAGE_REASON_INVALID_SAMPLE);
}

int main(void)
{
    test_nominal_policies_are_fail_closed();
    test_wrapping_age_and_mismatch_fail_closed();
    test_measured_cannot_be_forged();
    test_v1_quality_model_joint_invariants();
    test_zero_fault_legacy_equivalence_128_snapshots();
    test_request_layout_flags_and_nonfinite_are_rejected();
    test_adapter_unknown_fields_are_rejected();
    test_known_invalid_adapter_snapshot_remains_fail_closed();
    assert(foc_phase_voltage_realtime_input_assemble(0, 0, 0) == 0U);
    puts("FOC PHASE VOLTAGE REALTIME INPUT: PASS");
    return 0;
}
