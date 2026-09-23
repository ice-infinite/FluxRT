#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>

#include "foc_math_accel.h"
#include "foc_platform.h"
#include "foc_rust_bridge.h"

#ifndef FLUXRT_BUILD_PROFILE_NAME
#define FLUXRT_BUILD_PROFILE_NAME "diagnostic"
#endif
#ifndef FLUXRT_RUST_OPT_LEVEL_NAME
#define FLUXRT_RUST_OPT_LEVEL_NAME "s"
#endif

static foc_rust_context_t g_foc_controller;
static foc_runtime_config_t g_foc_runtime_config;
#if !defined(FLUXRT_PRODUCTION_BUILD)
static uint32_t g_foc_trace_rate_hz;
#endif
static foc_platform_config_t g_foc_platform_config =
{
    sizeof(foc_platform_config_t),
    FOC_PLATFORM_CONFIG_VERSION,
    7.0f,
    18.0f,
    1.15f,
    0.03f,
    0.97f,
    FOC_DEFAULT_ISR_DEADLINE_CYCLES,
};

static const char *foc_state_name(foc_state_t state)
{
    switch (state)
    {
    case FOC_STATE_DISABLED: return "disabled";
    case FOC_STATE_RUNNING: return "legacy-running";
    case FOC_STATE_ALIGNMENT: return "alignment";
    case FOC_STATE_OPEN_LOOP_RAMP: return "open-loop-ramp";
    case FOC_STATE_OPEN_LOOP_HOLD: return "open-loop-hold";
    case FOC_STATE_OBSERVER_TRANSITION: return "observer-transition";
    case FOC_STATE_CLOSED_LOOP: return "closed-loop";
    case FOC_STATE_FAULT: return "fault";
    default: return "uninitialized";
    }
}

static const char *foc_observer_name(foc_observer_backend_t backend)
{
    switch (backend)
    {
    case FOC_OBSERVER_SMO_PLL: return "rust-smo-pll";
    case FOC_OBSERVER_BEMF_PLL: return "rust-bemf-pll";
    case FOC_OBSERVER_ST_STO_PLL: return "st-sto-pll-reserved";
    default: return "invalid";
    }
}

static uint32_t foc_bus_voltage_mv(uint16_t raw)
{
    return ((uint32_t)raw * 52800UL + 2047UL) / 4095UL;
}

static int foc_start(int argc, char **argv)
{
    float target_rpm = g_foc_runtime_config.startup_final_speed_rpm;
    foc_status_t status;

    if (argc > 2)
    {
        rt_kprintf("Usage: foc_start [target_rpm]\n");
        return -1;
    }
    if (argc == 2)
    {
#if defined(FLUXRT_PRODUCTION_BUILD)
        target_rpm = (float)strtol(argv[1], RT_NULL, 10);
#else
        target_rpm = strtof(argv[1], RT_NULL);
#endif
    }
    status = foc_platform_control_start(target_rpm);
    if (status != FOC_STATUS_OK)
    {
        foc_platform_diagnostics_t diagnostics = {0};
        (void)foc_platform_get_diagnostics(&diagnostics);
        rt_kprintf("FOC start REFUSED: status=%u flags=0x%08x Vbus=%umV.\n",
                   (unsigned int)status,
                   (unsigned int)diagnostics.flags,
                   (unsigned int)foc_bus_voltage_mv(diagnostics.bus_voltage_raw));
        return -1;
    }
    rt_kprintf("FOC ARMED: target=%drpm startup=%drpm/%dmA observer=%s run/div=%u/%u closed_loop=%u.\n",
               (int)target_rpm,
               (int)g_foc_runtime_config.startup_final_speed_rpm,
               (int)(g_foc_runtime_config.startup_current_a * 1000.0f),
               foc_observer_name(g_foc_runtime_config.observer_backend),
               (unsigned int)g_foc_runtime_config.observer_enable,
               (unsigned int)g_foc_runtime_config.observer_update_divider,
               (unsigned int)g_foc_runtime_config.closed_loop_enable);
    return 0;
}
MSH_CMD_EXPORT(foc_start, start guarded ISR-owned current-controlled FOC);

static int foc_stop(int argc, char **argv)
{
    if (argc != 1)
    {
        rt_kprintf("Usage: foc_stop\n");
        return -1;
    }
    (void)argv;
    foc_platform_control_stop();
    rt_kprintf("FOC stopped; gate enable and TIM1 outputs are disabled.\n");
    return 0;
}
MSH_CMD_EXPORT(foc_stop, stop FOC and disable the power stage);

static void foc_print_timing(const foc_platform_diagnostics_t *diagnostics)
{
    foc_realtime_timing_stats_t timing = {0};

    (void)foc_platform_get_timing(&timing);
    rt_kprintf("FOC WCET step=%u total=%u pre/control/post=%u/%u/%u trace=%u/%u samples=%u invalid=%u.\n",
               (unsigned int)timing.wcet.step,
               (unsigned int)timing.wcet.total_cycles,
               (unsigned int)timing.wcet.precontrol_cycles,
               (unsigned int)timing.wcet.control_cycles,
               (unsigned int)timing.wcet.postcontrol_cycles,
               (unsigned int)timing.wcet.trace_enabled,
               (unsigned int)timing.wcet.trace_sampled,
               (unsigned int)timing.sample_count,
               (unsigned int)timing.invalid_sample_count);
    rt_kprintf("FTIMING,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)timing.version,
               (unsigned int)timing.sample_count,
               (unsigned int)timing.invalid_sample_count,
               (unsigned int)timing.wcet.step,
               (unsigned int)timing.wcet.total_cycles,
               (unsigned int)timing.wcet.precontrol_cycles,
               (unsigned int)timing.wcet.control_cycles,
               (unsigned int)timing.wcet.postcontrol_cycles,
               (unsigned int)timing.wcet.trace_enabled,
               (unsigned int)timing.wcet.trace_sampled,
               (unsigned int)timing.peak_precontrol_cycles,
               (unsigned int)timing.peak_control_cycles,
               (unsigned int)timing.peak_postcontrol_cycles,
               (unsigned int)g_foc_platform_config.isr_deadline_cycles,
               (unsigned int)diagnostics->deadline_miss_count,
               (unsigned int)diagnostics->realtime_error_count,
               (unsigned int)diagnostics->control_fault_flags,
               (unsigned int)diagnostics->flags);
}

static int foc_status(int argc, char **argv)
{
    foc_platform_diagnostics_t diagnostics = {0};
    foc_telemetry_t telemetry = {0};
    uint32_t peak_current_ma;

    if (argc != 1)
    {
        rt_kprintf("Usage: foc_status\n");
        return -1;
    }
    (void)argv;
    (void)foc_platform_get_diagnostics(&diagnostics);
    (void)foc_platform_get_telemetry(&telemetry);
    peak_current_ma = ((uint32_t)diagnostics.peak_current_delta_counts * 1595UL + 500UL) / 1000UL;
    rt_kprintf("FOC state=%s observer=%s reliable=%u closed=%u target=%drpm estimate=%drpm.\n",
               foc_state_name(telemetry.state),
               foc_observer_name(telemetry.observer_backend),
               (unsigned int)telemetry.observer_reliable,
               (unsigned int)telemetry.closed_loop_active,
               (int)telemetry.target_speed_rpm,
               (int)telemetry.measured_speed_rpm);
    rt_kprintf("FOC Iref d/q=%d/%dmA I d/q=%d/%dmA V d/q=%d/%dmV.\n",
               (int)(telemetry.id_reference_a * 1000.0f),
               (int)(telemetry.iq_reference_a * 1000.0f),
               (int)(telemetry.id_measured_a * 1000.0f),
               (int)(telemetry.iq_measured_a * 1000.0f),
               (int)(telemetry.vd_command_v * 1000.0f),
               (int)(telemetry.vq_command_v * 1000.0f));
    rt_kprintf("FOC flags=0x%08x steps=%u errors=%u ISRmax=%u/%u cycles misses=%u peak=%u (~%umA).\n",
               (unsigned int)diagnostics.flags,
               (unsigned int)diagnostics.realtime_step_count,
               (unsigned int)diagnostics.realtime_error_count,
               (unsigned int)diagnostics.maximum_isr_cycles,
               (unsigned int)g_foc_platform_config.isr_deadline_cycles,
               (unsigned int)diagnostics.deadline_miss_count,
               (unsigned int)diagnostics.peak_current_delta_counts,
               (unsigned int)peak_current_ma);
    rt_kprintf("FOC timing independent peaks pre/control/post=%u/%u/%u cycles; do not sum.\n",
               (unsigned int)diagnostics.maximum_precontrol_cycles,
               (unsigned int)diagnostics.maximum_control_cycles,
               (unsigned int)diagnostics.maximum_postcontrol_cycles);
    foc_print_timing(&diagnostics);
    rt_kprintf("FOC last status=%u rust_fault=0x%08x duty=%u/%u/%u per-mille.\n",
               (unsigned int)diagnostics.last_control_status,
               (unsigned int)diagnostics.control_fault_flags,
               (unsigned int)diagnostics.last_duty_a_per_mille,
               (unsigned int)diagnostics.last_duty_b_per_mille,
               (unsigned int)diagnostics.last_duty_c_per_mille);
    rt_kprintf("FOC ADC U/V/W=%u/%u/%u offsets=%u/%u/%u Vbus=%u (~%umV) Pot=%u.\n",
               (unsigned int)diagnostics.phase_u_raw,
               (unsigned int)diagnostics.phase_v_raw,
               (unsigned int)diagnostics.phase_w_raw,
               (unsigned int)diagnostics.phase_u_offset,
               (unsigned int)diagnostics.phase_v_offset,
               (unsigned int)diagnostics.phase_w_offset,
               (unsigned int)diagnostics.bus_voltage_raw,
               (unsigned int)foc_bus_voltage_mv(diagnostics.bus_voltage_raw),
               (unsigned int)diagnostics.potentiometer_raw);
    return 0;
}
MSH_CMD_EXPORT(foc_status, show FOC state observer and realtime diagnostics);

#if !defined(FLUXRT_PRODUCTION_BUILD)
static int foc_trace(int argc, char **argv)
{
    foc_platform_diagnostics_t diagnostics = {0};
    uint32_t sample_hz = 50U;
    uint32_t sample_divider;
    foc_status_t status;

    if ((argc == 1) || ((argc == 2) && (strcmp(argv[1], "status") == 0)))
    {
        (void)foc_platform_get_diagnostics(&diagnostics);
        rt_kprintf("FOC trace enabled=%u rate=%uHz dropped=%u.\n",
                   (unsigned int)foc_platform_trace_is_enabled(),
                   (unsigned int)g_foc_trace_rate_hz,
                   (unsigned int)foc_platform_trace_dropped());
        return 0;
    }
    if ((argc == 2) && (strcmp(argv[1], "stop") == 0))
    {
        foc_platform_trace_stop();
        g_foc_trace_rate_hz = 0U;
        rt_kprintf("FOC trace stopped; dropped=%u.\n",
                   (unsigned int)foc_platform_trace_dropped());
        return 0;
    }
    if ((argc < 2) || (argc > 3) || (strcmp(argv[1], "start") != 0))
    {
        rt_kprintf("Usage: foc_trace start [10..100 Hz] | stop | status\n");
        return -1;
    }
    if (argc == 3)
    {
        sample_hz = (uint32_t)strtoul(argv[2], RT_NULL, 10);
    }
    if ((sample_hz < 10U) || (sample_hz > 100U))
    {
        rt_kprintf("FOC trace rate must be 10..100 Hz.\n");
        return -1;
    }
    (void)foc_platform_get_diagnostics(&diagnostics);
    if (diagnostics.pwm_frequency_hz == 0U)
    {
        rt_kprintf("FOC trace REFUSED: PWM frequency is unavailable.\n");
        return -1;
    }
    sample_divider = diagnostics.pwm_frequency_hz / sample_hz;
    status = foc_platform_trace_start(sample_divider);
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FOC trace REFUSED: status=%u divider=%u.\n",
                   (unsigned int)status,
                   (unsigned int)sample_divider);
        return -1;
    }
    g_foc_trace_rate_hz = diagnostics.pwm_frequency_hz / sample_divider;
    rt_kprintf("FTR_HEADER,step,state,ia_ma,ib_ma,ic_ma,id_ref_ma,iq_ref_ma,id_ma,iq_ma,vd_mv,vq_mv,duty_a_pm,duty_b_pm,duty_c_pm,vbus_mv,control_angle_mrad,forced_angle_mrad,observer_angle_mrad,observer_speed_rpm,reliable,flags\n");
    rt_kprintf("FOC trace started: requested=%uHz actual=%uHz divider=%u.\n",
               (unsigned int)sample_hz,
               (unsigned int)(diagnostics.pwm_frequency_hz / sample_divider),
               (unsigned int)sample_divider);
    return 0;
}
MSH_CMD_EXPORT(foc_trace, stream decimated realtime FOC CSV telemetry);

static void foc_print_config(void)
{
    rt_kprintf("FOC config observer=%s run/div=%u/%u closed_loop=%u startup=%drpm/%dmA align=%dms ramp=%dms util=%d/1000.\n",
               foc_observer_name(g_foc_runtime_config.observer_backend),
               (unsigned int)g_foc_runtime_config.observer_enable,
               (unsigned int)g_foc_runtime_config.observer_update_divider,
               (unsigned int)g_foc_runtime_config.closed_loop_enable,
               (int)g_foc_runtime_config.startup_final_speed_rpm,
               (int)(g_foc_runtime_config.startup_current_a * 1000.0f),
               (int)(g_foc_runtime_config.alignment_duration_s * 1000.0f),
               (int)(g_foc_runtime_config.open_loop_ramp_duration_s * 1000.0f),
               (int)(g_foc_runtime_config.voltage_utilization * 1000.0f));
    rt_kprintf("FOC limits Vbus=%d..%dmV trip=%dmA duty=%d..%d/1000 deadline=%u cycles.\n",
               (int)(g_foc_platform_config.minimum_bus_voltage_v * 1000.0f),
               (int)(g_foc_platform_config.maximum_bus_voltage_v * 1000.0f),
               (int)(g_foc_platform_config.software_current_trip_a * 1000.0f),
               (int)(g_foc_platform_config.minimum_duty * 1000.0f),
               (int)(g_foc_platform_config.maximum_duty * 1000.0f),
               (unsigned int)g_foc_platform_config.isr_deadline_cycles);
    rt_kprintf("FOC SMO slide=%dmV boundary=%dmA filter=%d/1000 PLL kp/ki=%d/%d.\n",
               (int)(g_foc_runtime_config.observer_smo_k_slide_v * 1000.0f),
               (int)(g_foc_runtime_config.observer_smo_boundary_a * 1000.0f),
               (int)(g_foc_runtime_config.observer_emf_filter_alpha * 1000.0f),
               (int)g_foc_runtime_config.observer_pll_kp,
               (int)g_foc_runtime_config.observer_pll_ki);
    rt_kprintf("FOC handoff min=%drpm bemf=%dmV variance=%d/1000 confirm=%u acquire/loss=%d/%dms accel=%drpm/s PIpreload=%d/1000 Islew=%dmA/s.\n",
               (int)g_foc_runtime_config.observer_minimum_speed_rpm,
               (int)(g_foc_runtime_config.observer_minimum_bemf_v * 1000.0f),
               (int)(g_foc_runtime_config.observer_speed_variance_ratio * 1000.0f),
               (unsigned int)g_foc_runtime_config.observer_consecutive_samples,
               (int)(g_foc_runtime_config.observer_acquisition_timeout_s * 1000.0f),
               (int)(g_foc_runtime_config.observer_loss_timeout_s * 1000.0f),
               (int)g_foc_runtime_config.closed_loop_speed_ramp_rpm_per_s,
               (int)(g_foc_runtime_config.speed_pi_preload_ratio * 1000.0f),
               (int)(g_foc_runtime_config.closed_loop_current_slew_a_per_s * 1000.0f));
}

static int foc_cfg(int argc, char **argv)
{
    foc_runtime_config_t candidate = g_foc_runtime_config;
    foc_platform_diagnostics_t diagnostics = {0};
    foc_status_t status;

    if ((argc == 1) || ((argc == 2) && (strcmp(argv[1], "show") == 0)))
    {
        foc_print_config();
        return 0;
    }
    if (argc != 3)
    {
        rt_kprintf("Usage: foc_cfg show | observer smo|bemf|st | observe/closedloop 0|1 | obsdiv 1..32 | current/trip/boundary <mA> | slide/minbemf <mV> | filter/variance/preload <permille> | pll_kp/pll_ki <value> | startup/minspeed <rpm> | confirm <samples> | acquire/loss <ms> | accel <rpm/s> | islew <mA/s> | util <permille>\n");
        return -1;
    }
    (void)foc_platform_get_diagnostics(&diagnostics);
    if ((diagnostics.flags & FOC_PLATFORM_DIAG_REALTIME_ARMED) != 0U)
    {
        rt_kprintf("FOC config REFUSED: run foc_stop first.\n");
        return -1;
    }

    if (strcmp(argv[1], "observer") == 0)
    {
        if (strcmp(argv[2], "smo") == 0) candidate.observer_backend = FOC_OBSERVER_SMO_PLL;
        else if (strcmp(argv[2], "bemf") == 0) candidate.observer_backend = FOC_OBSERVER_BEMF_PLL;
        else if (strcmp(argv[2], "st") == 0) candidate.observer_backend = FOC_OBSERVER_ST_STO_PLL;
        else return -1;
    }
    else if (strcmp(argv[1], "closedloop") == 0)
    {
        int value = atoi(argv[2]);
        if ((value != 0) && (value != 1)) return -1;
        candidate.closed_loop_enable = (uint32_t)value;
    }
    else if (strcmp(argv[1], "observe") == 0)
    {
        int value = atoi(argv[2]);
        if ((value != 0) && (value != 1)) return -1;
        candidate.observer_enable = (uint32_t)value;
    }
    else if (strcmp(argv[1], "obsdiv") == 0)
    {
        candidate.observer_update_divider = (uint32_t)strtoul(argv[2], RT_NULL, 10);
    }
    else if (strcmp(argv[1], "startup") == 0)
    {
        candidate.startup_final_speed_rpm = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "current") == 0)
    {
        candidate.startup_current_a = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "util") == 0)
    {
        candidate.voltage_utilization = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "slide") == 0)
    {
        candidate.observer_smo_k_slide_v = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "boundary") == 0)
    {
        candidate.observer_smo_boundary_a = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "filter") == 0)
    {
        candidate.observer_emf_filter_alpha = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "pll_kp") == 0)
    {
        candidate.observer_pll_kp = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "pll_ki") == 0)
    {
        candidate.observer_pll_ki = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "minspeed") == 0)
    {
        candidate.observer_minimum_speed_rpm = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "minbemf") == 0)
    {
        candidate.observer_minimum_bemf_v = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "variance") == 0)
    {
        candidate.observer_speed_variance_ratio = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "confirm") == 0)
    {
        candidate.observer_consecutive_samples = (uint32_t)strtoul(argv[2], RT_NULL, 10);
    }
    else if (strcmp(argv[1], "acquire") == 0)
    {
        candidate.observer_acquisition_timeout_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "loss") == 0)
    {
        candidate.observer_loss_timeout_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "accel") == 0)
    {
        candidate.closed_loop_speed_ramp_rpm_per_s = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "preload") == 0)
    {
        candidate.speed_pi_preload_ratio = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "islew") == 0)
    {
        candidate.closed_loop_current_slew_a_per_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "trip") == 0)
    {
        foc_platform_config_t platform_candidate = g_foc_platform_config;
        platform_candidate.software_current_trip_a = strtof(argv[2], RT_NULL) / 1000.0f;
        status = foc_platform_configure(&platform_candidate);
        if (status != FOC_STATUS_OK)
        {
            rt_kprintf("FOC platform config rejected: %u.\n", (unsigned int)status);
            return -1;
        }
        g_foc_platform_config = platform_candidate;
        foc_print_config();
        return 0;
    }
    else
    {
        return -1;
    }

    status = foc_rust_configure(&g_foc_controller, &candidate);
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FOC Rust config rejected: %u.\n", (unsigned int)status);
        return -1;
    }
    g_foc_runtime_config = candidate;
    foc_print_config();
    return 0;
}
MSH_CMD_EXPORT(foc_cfg, show or change disabled FOC configuration);
#endif

int main(void)
{
    foc_status_t rust_status;
    foc_status_t platform_config_status;
    foc_status_t platform_status;
    foc_status_t bind_status;
    foc_status_t feedback_status;
    foc_feedback_t feedback = {0};
    foc_platform_diagnostics_t diagnostics = {0};
    uint32_t heartbeat_ms;

    foc_platform_emergency_stop();
    if ((foc_rust_abi_version() != FOC_RUST_ABI_VERSION) ||
        (foc_rust_context_required_size() > sizeof(g_foc_controller)) ||
        (foc_rust_context_required_align() > _Alignof(foc_rust_context_t)))
    {
        rt_kprintf("FATAL: C/Rust FOC ABI mismatch; PWM remains disabled.\n");
        while (1) rt_thread_mdelay(1000);
    }

    rust_status = foc_rust_init(&g_foc_controller);
    if (rust_status == FOC_STATUS_OK)
    {
        rust_status = foc_rust_default_st_config(&g_foc_runtime_config);
    }
    if (rust_status == FOC_STATUS_OK)
    {
        g_foc_runtime_config.observer_backend = FOC_OBSERVER_SMO_PLL;
        g_foc_runtime_config.observer_enable = 1U;
        g_foc_runtime_config.closed_loop_enable = 0U;
        g_foc_runtime_config.observer_update_divider = 1U;
        /* Preserve PWM/ADC sampling margin on X-NUCLEO-IHM16M1. */
        g_foc_runtime_config.voltage_utilization = 0.90f;
        rust_status = foc_rust_configure(&g_foc_controller, &g_foc_runtime_config);
    }
    platform_config_status = foc_platform_configure(&g_foc_platform_config);
    platform_status = foc_platform_init();
    bind_status = foc_platform_bind_controller(&g_foc_controller);
    rt_thread_mdelay(100);
    feedback_status = foc_platform_read_feedback(&feedback);
    (void)foc_platform_get_diagnostics(&diagnostics);
    heartbeat_ms = (uint32_t)rt_tick_get_millisecond();

    rt_kprintf("STM32G431 RT-Thread + Rust FOC realtime bring-up\n");
    rt_kprintf("Build profile=%s Rust opt-level=%s.\n",
               FLUXRT_BUILD_PROFILE_NAME,
               FLUXRT_RUST_OPT_LEVEL_NAME);
    rt_kprintf("Rust ABI=0x%08x context=%u/%u init/config=%u; platform cfg/init/bind=%u/%u/%u.\n",
               (unsigned int)foc_rust_abi_version(),
               (unsigned int)foc_rust_context_required_size(),
               (unsigned int)sizeof(g_foc_controller),
               (unsigned int)rust_status,
               (unsigned int)platform_config_status,
               (unsigned int)platform_status,
               (unsigned int)bind_status);
    rt_kprintf("Monitor feedback=%u flags=0x%08x PWM=%uHz sync=%u Vbus=%umV. Outputs DISABLED.\n",
               (unsigned int)feedback_status,
               (unsigned int)diagnostics.flags,
               (unsigned int)diagnostics.pwm_frequency_hz,
               (unsigned int)diagnostics.sync_sample_count,
               (unsigned int)foc_bus_voltage_mv(diagnostics.bus_voltage_raw));
    rt_kprintf("Math=%s; observer=%s; closed-loop handoff=%u.\n",
               (foc_math_accel_backend() == FOC_MATH_BACKEND_CORDIC) ? "STM32G4-CORDIC" : "CPU",
               foc_observer_name(g_foc_runtime_config.observer_backend),
               (unsigned int)g_foc_runtime_config.closed_loop_enable);
#if defined(FLUXRT_PRODUCTION_BUILD)
    rt_kprintf("Commands: foc_status, foc_start, foc_stop. Runtime tuning and trace are compiled out.\n");
#else
    rt_kprintf("Commands: foc_status, foc_cfg show, foc_trace, foc_start, foc_stop.\n");
#endif

    while (1)
    {
#if !defined(FLUXRT_PRODUCTION_BUILD)
        foc_trace_sample_t trace_sample;
#endif
        uint32_t now_ms = (uint32_t)rt_tick_get_millisecond();
#if !defined(FLUXRT_PRODUCTION_BUILD)
        while (foc_platform_trace_pop(&trace_sample) != 0U)
        {
            rt_kprintf("FTR,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%d,%d,%d,%d,%u,%u\n",
                       (unsigned int)trace_sample.step,
                       (unsigned int)trace_sample.state,
                       (int)trace_sample.phase_a_ma,
                       (int)trace_sample.phase_b_ma,
                       (int)trace_sample.phase_c_ma,
                       (int)trace_sample.id_reference_ma,
                       (int)trace_sample.iq_reference_ma,
                       (int)trace_sample.id_measured_ma,
                       (int)trace_sample.iq_measured_ma,
                       (int)trace_sample.vd_command_mv,
                       (int)trace_sample.vq_command_mv,
                       (unsigned int)trace_sample.duty_a_per_mille,
                       (unsigned int)trace_sample.duty_b_per_mille,
                       (unsigned int)trace_sample.duty_c_per_mille,
                       (unsigned int)trace_sample.bus_voltage_mv,
                       (int)trace_sample.control_angle_mrad,
                       (int)trace_sample.forced_angle_mrad,
                       (int)trace_sample.observer_angle_mrad,
                       (int)trace_sample.observer_speed_rpm,
                       (unsigned int)trace_sample.observer_reliable,
                       (unsigned int)trace_sample.flags);
        }
#endif
        if (((now_ms - heartbeat_ms) >= 5000U) &&
#if !defined(FLUXRT_PRODUCTION_BUILD)
            (foc_platform_trace_is_enabled() == 0U))
#else
            1)
#endif
        {
            heartbeat_ms = now_ms;
            (void)foc_platform_get_diagnostics(&diagnostics);
            if ((diagnostics.flags & FOC_PLATFORM_DIAG_REALTIME_ARMED) == 0U)
            {
                (void)foc_platform_read_feedback(&feedback);
            }
            rt_kprintf("FOC alive flags=0x%08x sync=%u steps=%u errors=%u Vbus=%umV.\n",
                       (unsigned int)diagnostics.flags,
                       (unsigned int)diagnostics.sync_sample_count,
                       (unsigned int)diagnostics.realtime_step_count,
                       (unsigned int)diagnostics.realtime_error_count,
                       (unsigned int)foc_bus_voltage_mv(diagnostics.bus_voltage_raw));
        }
        rt_thread_mdelay(10);
    }
}
