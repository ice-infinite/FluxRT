/* FluxRT - Identification-only EXP-B3 management surface.
 * Only the exact LSI1 command may request a platform-owned bounded session. */

#include "foc_build_profile.h"

#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)

#include <finsh.h>
#include <rtthread.h>

#include "foc_lsi_capture_service.h"
#include "foc_lsi_management.h"
#include "foc_platform.h"

/* 1.0 V bus at 3.3 V/12 bit and a 0.0625 divider is about 78 counts.
 * This command is deliberately a no-power evidence gate, not a motor command. */
#define FOC_LSI_CAPTURE_NO_POWER_BUS_MAX_RAW (80U)

static uint32_t foc_lsi_shell_ensure_initialized(void)
{
    return foc_lsi_management_shared_init();
}

static void foc_lsi_shell_print_status(
    const char *tag,
    unsigned int result,
    const foc_lsi_management_status_t *status)
{
    rt_kprintf("%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               tag,
               (unsigned int)status->version,
               result,
               (unsigned int)status->build_authorized,
               (unsigned int)status->output.state,
               (unsigned int)status->output.abort_reason,
               (unsigned int)status->output.force_safe_output,
               (unsigned int)status->output.drive_request,
               (unsigned int)status->identification_start_command_exposed,
               (unsigned int)status->pwm_adapter_present,
               (unsigned int)status->adc_adapter_present,
               (unsigned int)status->abort_command_count,
               (unsigned int)status->reset_command_count,
               (unsigned int)status->start_command_count,
               (unsigned int)status->start_consumed);
}

static int foc_lsi_shell_status(int argc, char **argv)
{
    foc_lsi_management_status_t status;

    (void)argv;
    rt_base_t level;

    if ((argc != 1) || (foc_lsi_shell_ensure_initialized() == 0U))
    {
        rt_kprintf("foc_lsi_status\n");
        return -1;
    }
    level = rt_hw_interrupt_disable();
    if (foc_lsi_management_shared_get_status(&status) == 0U)
    {
        rt_hw_interrupt_enable(level);
        rt_kprintf("foc_lsi_status\n");
        return -1;
    }
    rt_hw_interrupt_enable(level);
    foc_lsi_shell_print_status("FLSI_STATUS", 0U, &status);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_status, foc_lsi_status, -);

static int foc_lsi_shell_hw_status(int argc, char **argv)
{
    foc_platform_diagnostics_t diagnostics;

    (void)argv;
    if ((argc != 1) ||
        (foc_platform_get_diagnostics(&diagnostics) != FOC_STATUS_OK))
    {
        rt_kprintf("foc_lsi_hw_status\n");
        return -1;
    }
    /* 只读、机器可解析。configured/valid 分开，避免把“路径存在”误读为
     * “已经收到有效同序列样本”。period 是 TIM1 ARR/占空比分母。 */
    rt_kprintf("FLSI_HW,1,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)((diagnostics.flags &
                   FOC_PLATFORM_DIAG_LSI_SYNC_BUS_CONFIGURED) != 0U),
               (unsigned int)diagnostics.lsi_sync_bus_valid,
               (unsigned int)diagnostics.lsi_sync_bus_sample_count,
               (unsigned int)diagnostics.lsi_sync_bus_voltage_raw,
               (unsigned int)diagnostics.pwm_period_ticks,
               (unsigned int)diagnostics.sync_sample_count,
               (unsigned int)diagnostics.monitor_isr_last_cycles,
               (unsigned int)diagnostics.monitor_isr_min_cycles,
               (unsigned int)diagnostics.monitor_isr_max_cycles,
               (unsigned int)diagnostics.sync_interval_last_cycles,
               (unsigned int)diagnostics.sync_interval_max_cycles,
               (unsigned int)diagnostics.flags);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_hw_status, foc_lsi_hw_status, -);

static void foc_lsi_shell_capture_status_snapshot(
    foc_lsi_capture_service_status_t *status)
{
    rt_base_t level = rt_hw_interrupt_disable();

    foc_lsi_capture_service_get_status(status);
    rt_hw_interrupt_enable(level);
}

static void foc_lsi_shell_print_capture_status(
    const char *tag,
    unsigned int result,
    const foc_lsi_capture_service_status_t *status)
{
    rt_kprintf("%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               tag,
               result,
               (unsigned int)status->capture.state,
               (unsigned int)status->capture.sample_rate_hz,
               (unsigned int)status->capture.capacity,
               (unsigned int)status->capture.sample_count,
               (unsigned int)status->capture.unread_count,
               (unsigned int)status->capture.overflow_count,
               (unsigned int)status->last_contract_result,
               (unsigned int)status->contract_error_count,
               (unsigned int)status->storage_error_count);
}

static int foc_lsi_shell_capture_arm(int argc, char **argv)
{
    const uint32_t required = FOC_PLATFORM_DIAG_GATE_SAFE |
                              FOC_PLATFORM_DIAG_SYNC_RUNNING |
                              FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID |
                              FOC_PLATFORM_DIAG_LSI_SYNC_BUS_CONFIGURED;
    const uint32_t forbidden = FOC_PLATFORM_DIAG_DRIVER_FAULT |
                               FOC_PLATFORM_DIAG_OUTPUT_ACTIVE |
                               FOC_PLATFORM_DIAG_BREAK_LATCHED |
                               FOC_PLATFORM_DIAG_CURRENT_TRIP |
                               FOC_PLATFORM_DIAG_DEADLINE_MISSED |
                               FOC_PLATFORM_DIAG_CONTROL_ERROR |
                               FOC_PLATFORM_DIAG_OUTPUT_REJECTED |
                               FOC_PLATFORM_DIAG_LSI_CAPTURE_ERROR |
                               FOC_PLATFORM_DIAG_LSI_SESSION_RUNNING;
    foc_platform_diagnostics_t diagnostics;
    foc_lsi_capture_service_status_t status;
    rt_base_t level;
    uint32_t result = 0U;

    (void)argv;
    if ((argc != 1) ||
        (foc_platform_get_diagnostics(&diagnostics) != FOC_STATUS_OK) ||
        ((diagnostics.flags & required) != required) ||
        ((diagnostics.flags & forbidden) != 0U) ||
        (diagnostics.lsi_sync_bus_valid == 0U) ||
        (diagnostics.lsi_sync_bus_voltage_raw >
         FOC_LSI_CAPTURE_NO_POWER_BUS_MAX_RAW))
    {
        rt_kprintf("FLSI_CAP_ARM,0,REFUSED\n");
        return -1;
    }
    level = rt_hw_interrupt_disable();
    result = foc_lsi_capture_service_arm();
    rt_hw_interrupt_enable(level);
    foc_lsi_shell_capture_status_snapshot(&status);
    foc_lsi_shell_print_capture_status("FLSI_CAP_ARM", result, &status);
    return (result != 0U) ? 0 : -1;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_capture_arm, foc_lsi_capture_arm, -);

static int foc_lsi_shell_capture_status(int argc, char **argv)
{
    foc_lsi_capture_service_status_t status;

    (void)argv;
    if (argc != 1)
    {
        rt_kprintf("foc_lsi_capture_status\n");
        return -1;
    }
    foc_lsi_shell_capture_status_snapshot(&status);
    foc_lsi_shell_print_capture_status("FLSI_CAP_STATUS", 1U, &status);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_capture_status, foc_lsi_capture_status, -);

static int foc_lsi_shell_capture_dump(int argc, char **argv)
{
    foc_lsi_capture_service_status_t status;
    foc_lsi_raw_sample_t sample;
    uint32_t count = 0U;

    (void)argv;
    if (argc != 1)
    {
        rt_kprintf("foc_lsi_capture_dump\n");
        return -1;
    }
    foc_lsi_shell_capture_status_snapshot(&status);
    if ((status.capture.state != FOC_LSI_RAW_CAPTURE_COMPLETE) &&
        (status.capture.state != FOC_LSI_RAW_CAPTURE_OVERFLOW))
    {
        rt_kprintf("FLSI_CAP_DUMP,0,REFUSED,%u\n",
                   (unsigned int)status.capture.state);
        return -1;
    }
    while (foc_lsi_capture_service_pop(&sample) != 0U)
    {
        rt_kprintf("FLSI_RAW,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                   (unsigned int)sample.sequence,
                   (unsigned int)sample.control_tick,
                   (unsigned int)sample.current_u_raw,
                   (unsigned int)sample.current_v_raw,
                   (unsigned int)sample.bus_voltage_raw,
                   (unsigned int)sample.compare_u,
                   (unsigned int)sample.compare_v,
                   (unsigned int)sample.compare_w,
                   (unsigned int)sample.pwm_period_ticks,
                   (unsigned int)sample.flags);
        ++count;
    }
    rt_kprintf("FLSI_CAP_DUMP,1,%u\n", (unsigned int)count);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_capture_dump, foc_lsi_capture_dump, -);

static unsigned int foc_lsi_shell_milli(float value)
{
    return (unsigned int)(value * 1000.0f + 0.5f);
}

static int foc_lsi_shell_dump(int argc, char **argv)
{
    foc_lsi_config_t config;
    rt_base_t level;

    (void)argv;
    if ((argc != 1) || (foc_lsi_shell_ensure_initialized() == 0U))
    {
        rt_kprintf("foc_lsi_dump\n");
        return -1;
    }
    level = rt_hw_interrupt_disable();
    if (foc_lsi_management_shared_get_config(&config) == 0U)
    {
        rt_hw_interrupt_enable(level);
        return -1;
    }
    rt_hw_interrupt_enable(level);
    rt_kprintf("FLSI_CONFIG,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)config.version,
               (unsigned int)config.sample_rate_hz,
               (unsigned int)config.offset_sample_count,
               (unsigned int)config.bias_settle_ticks,
               (unsigned int)config.pulse_ticks_per_polarity,
               (unsigned int)config.pulse_pair_count,
               (unsigned int)config.cooldown_zero_ticks,
               (unsigned int)config.max_active_ticks,
               (unsigned int)config.total_timeout_ticks,
               foc_lsi_shell_milli(config.bias_current_a),
               foc_lsi_shell_milli(config.perturbation_voltage_v),
               foc_lsi_shell_milli(config.current_trip_a),
               foc_lsi_shell_milli(config.cooldown_current_threshold_a),
               foc_lsi_shell_milli(config.bus_voltage_min_v),
               foc_lsi_shell_milli(config.bus_voltage_max_v));
    return 0;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_dump, foc_lsi_dump, -);

static int foc_lsi_shell_abort(int argc, char **argv)
{
    foc_lsi_management_status_t status;
    foc_lsi_management_result_t result;
    rt_base_t level;

    (void)argv;
    if ((argc != 1) || (foc_lsi_shell_ensure_initialized() == 0U))
    {
        rt_kprintf("foc_lsi_abort\n");
        return -1;
    }
    foc_platform_emergency_stop();
    level = rt_hw_interrupt_disable();
    result = foc_lsi_management_shared_abort(&status);
    rt_hw_interrupt_enable(level);
    foc_lsi_shell_print_status("FLSI_ABORT", (unsigned int)result, &status);
    return (result == FOC_LSI_MANAGEMENT_INVALID) ? -1 : 0;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_abort, foc_lsi_abort, -);

static int foc_lsi_shell_reset(int argc, char **argv)
{
    foc_lsi_management_status_t status;
    foc_lsi_management_result_t result;
    rt_base_t level;

    (void)argv;
    if ((argc != 1) || (foc_lsi_shell_ensure_initialized() == 0U))
    {
        rt_kprintf("foc_lsi_reset\n");
        return -1;
    }
    foc_platform_emergency_stop();
    level = rt_hw_interrupt_disable();
    result = foc_lsi_management_shared_reset(&status);
    rt_hw_interrupt_enable(level);
    foc_lsi_shell_print_status("FLSI_RESET", (unsigned int)result, &status);
    return (result == FOC_LSI_MANAGEMENT_INVALID) ? -1 : 0;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_reset, foc_lsi_reset, -);

static int foc_lsi_shell_start(int argc, char **argv)
{
    foc_lsi_management_status_t status;
    foc_status_t platform_status;
    rt_base_t level;

    if ((argc != 2) || (rt_strcmp(argv[1], "LSI1") != 0) ||
        (foc_lsi_shell_ensure_initialized() == 0U))
    {
        rt_kprintf("foc_lsi_start LSI1\n");
        return -1;
    }
    platform_status = foc_platform_lsi_start(FOC_LSI_START_CONFIRMATION);
    level = rt_hw_interrupt_disable();
    (void)foc_lsi_management_shared_get_status(&status);
    rt_hw_interrupt_enable(level);
    foc_lsi_shell_print_status("FLSI_START",
                               (unsigned int)platform_status,
                               &status);
    return (platform_status == FOC_STATUS_OK) ? 0 : -1;
}
MSH_CMD_EXPORT_ALIAS(foc_lsi_shell_start, foc_lsi_start, -);

#endif
