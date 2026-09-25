/*
 * FluxRT —— C 侧平台层与 ABI 的主机安全测试（无硬件）。
 * FluxRT - host-side safety test for the C platform layer and the ABI (no hardware).
 *
 * 职责 / Responsibility:
 *   - 在 PC 上跑同一份 `foc/include` 头文件与 `foc/runtime`、`foc/platform` 的
 *     纯软件部分，验证"空状态"下的行为与同拍时序不变量；
 *   - Runs the same `foc/include` headers and the software-only parts of
 *     `foc/runtime` and `foc/platform` on a PC to verify the empty-state
 *     behaviour and the same-tick timing invariants.
 *
 * 它守的不是功能，而是三条底线 / It guards three bottom lines, not features:
 *   1. ABI 触发线：ABI 版本、上下文容量、运行时配置与遥测结构体大小、加速后端
 *      枚举。任何一条变化都说明 C 与 Rust 不再匹配，必须在板上卡住而不是静默错位；
 *      ABI tripwires: the ABI version, the context capacity, the runtime-config and
 *      telemetry struct sizes and the acceleration backend enum. A change in any of
 *      them means C and Rust no longer match and must fail here, not silently
 *      misalign on the board;
 *   2. 安全空状态：未 configure 时平台必须拒绝 init / arm / 输出，并把反馈与
 *      diagnostics 保持为全 0，绝不返回未初始化内存；
 *      safe empty state: without configure the platform must refuse init, arm and
 *      output and keep feedback and diagnostics at zero; it must never return
 *      uninitialised memory;
 *   3. 同拍时序不变量：total == pre + control + post 只对同一拍成立，异常样本
 *      （分段和与 total 不符）不得污染 WCET，只能计入 invalid 计数。
 *      the same-tick timing invariant: total == pre + control + post holds only
 *      within one tick, and an inconsistent sample must not contaminate the WCET;
 *      it may only raise the invalid counter.
 *
 * 边界 / Boundary: 主机上没有 PWM、没有注入组 ADC、没有 CORDIC，因此这里只能
 * 覆盖"拒绝"路径；真正的 arm/关断/时序测量必须在板上验证。
 * There is no PWM, no injected-group ADC and no CORDIC on the host, so only the
 * refusal paths are covered here; arming, shutdown and real timing measurement
 * must be verified on the board.
 *
 * 参考 / Reference: docs/performance/2026-09-23-A0关联WCET基线.md
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "foc_production_profile.h"
#include "foc_platform.h"
#include "foc_math_accel.h"
#include "foc_pwm_timing.h"
#include "foc_rust_bridge.h"

/* The C host test does not link the Rust archive. This narrow stub lets it test
 * the application-layer profile transaction and all fail-closed branches; the
 * real CRC algorithm and exact value are independently pinned by Rust tests. */
static foc_status_t g_profile_crc_stub_status = FOC_STATUS_OK;
static uint32_t g_profile_crc_stub_value = 0xEDF4F6CAUL;

foc_status_t foc_rust_runtime_config_crc32(const foc_runtime_config_t *config,
                                           uint32_t *crc_out)
{
    if ((config == NULL) || (crc_out == NULL))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    if (g_profile_crc_stub_status == FOC_STATUS_OK)
    {
        *crc_out = g_profile_crc_stub_value;
    }
    return g_profile_crc_stub_status;
}

static void test_production_profile(void)
{
    foc_production_profile_t changed;
    foc_production_profile_report_t report;
    foc_runtime_config_t runtime = {0};
    foc_runtime_config_t before;

    assert(sizeof(foc_production_profile_t) == 40U);
    assert(foc_production_profile_record_crc32(&g_foc_production_profile) ==
           g_foc_production_profile.record_crc32);
    assert(foc_production_profile_validate(&g_foc_production_profile,
                                           0xEDF4F6CAUL,
                                           &report) ==
           FOC_PROFILE_VALID_UNAPPROVED);
    assert(report.approval_flags == 0U);
    assert(report.computed_record_crc32 == 0x888E4E91UL);

    changed = g_foc_production_profile;
    changed.motor_id ^= 1U;
    assert(foc_production_profile_validate(&changed,
                                           changed.runtime_config_crc32,
                                           &report) == FOC_PROFILE_WRONG_TARGET);

    changed = g_foc_production_profile;
    changed.approval_flags = FOC_PROFILE_APPROVAL_CLOSED_LOOP;
    assert(foc_production_profile_validate(&changed,
                                           changed.runtime_config_crc32,
                                           &report) == FOC_PROFILE_BAD_APPROVAL);

    changed = g_foc_production_profile;
    changed.record_crc32 ^= 1U;
    assert(foc_production_profile_validate(&changed,
                                           changed.runtime_config_crc32,
                                           &report) == FOC_PROFILE_BAD_RECORD_CRC);
    assert(foc_production_profile_validate(&g_foc_production_profile,
                                           0x8C7A8DF5UL,
                                           &report) == FOC_PROFILE_BAD_RUNTIME_CRC);

    runtime.struct_size = sizeof(runtime);
    runtime.config_version = FOC_RUST_CONFIG_VERSION;
    assert(foc_production_profile_apply(&g_foc_production_profile,
                                        &runtime,
                                        &report) ==
           FOC_PROFILE_VALID_UNAPPROVED);
    assert(runtime.observer_backend == FOC_OBSERVER_SMO_PLL);
    assert(runtime.observer_enable == 1U);
    assert(runtime.observer_update_divider == 1U);
    assert(runtime.closed_loop_enable == 0U);
    assert(runtime.voltage_utilization == 0.90f);
    assert(runtime.inverter_voltage_model.enabled == 0U);
    assert(runtime.inverter_voltage_model.observer_voltage_correction_enable == 0U);
    assert(runtime.inverter_voltage_model.pwm_feedforward_enable == 0U);

    before = runtime;
    g_profile_crc_stub_value ^= 1U;
    assert(foc_production_profile_apply(&g_foc_production_profile,
                                        &runtime,
                                        &report) == FOC_PROFILE_BAD_RUNTIME_CRC);
    assert(memcmp(&runtime, &before, sizeof(runtime)) == 0);
    g_profile_crc_stub_value ^= 1U;

    g_profile_crc_stub_status = FOC_STATUS_INVALID_ARGUMENT;
    assert(foc_production_profile_apply(&g_foc_production_profile,
                                        &runtime,
                                        &report) ==
           FOC_PROFILE_RUNTIME_CONFIG_INVALID);
    assert(memcmp(&runtime, &before, sizeof(runtime)) == 0);
    g_profile_crc_stub_status = FOC_STATUS_OK;
}

/* 锁定目标板两套中心对齐时序。Host 不模拟寄存器，只证明宏计划中的整数比、
 * RCR 半周期分频与 CCR preload 延迟互相一致。 */
static void test_pwm_timing_contract(void)
{
    foc_pwm_timing_plan_t baseline = {
        170000000U, 12000U, 12000U, 7083U, 1U, 1U,
        FOC_PWM_ADC_TRIGGER_OC4REF_RISING};
    foc_pwm_timing_plan_t candidate = {
        170000000U, 24000U, 12000U, 3541U, 3U, 2U,
        FOC_PWM_ADC_TRIGGER_DIVIDED_OC4REF};

    assert(foc_pwm_timing_ratio(&baseline) == 1U);
    assert(foc_pwm_timing_plan_is_valid(&baseline) == 1U);
    assert(foc_pwm_timing_ratio(&candidate) == 2U);
    assert(foc_pwm_timing_plan_is_valid(&candidate) == 1U);

    /* 24 kHz OC4REF 直连会把 ADC ISR 也拉到 24 kHz；TIM1 Update 虽能被
     * RCR 分到 12 kHz，却不保证落在三分流有效窗口。二者都必须拒绝。 */
    candidate.adc_trigger = FOC_PWM_ADC_TRIGGER_OC4REF_RISING;
    assert(foc_pwm_timing_plan_is_valid(&candidate) == 0U);
    candidate.adc_trigger = FOC_PWM_ADC_TRIGGER_UPDATE;
    assert(foc_pwm_timing_plan_is_valid(&candidate) == 0U);
    candidate.adc_trigger = FOC_PWM_ADC_TRIGGER_DIVIDED_OC4REF;
    candidate.repetition_counter = 1U;
    assert(foc_pwm_timing_plan_is_valid(&candidate) == 0U);
    candidate.repetition_counter = 3U;
    candidate.actuation_delay_pwm_ticks = 1U;
    assert(foc_pwm_timing_plan_is_valid(&candidate) == 0U);
}

/*
 * 同拍时序记录与 WCET 语义的回归测试。
 * Regression test for the same-tick timing record and the WCET semantics.
 *
 * 样本 / Sample: {step, total, pre, control, post, trace_enabled, trace_sampled}，
 * 单位全部是 DWT 周期 [cycles]，step 是递增的控制拍序号 [counts]。
 * {step, total, pre, control, post, trace_enabled, trace_sampled}, all cycles
 * except the monotonic tick index.
 *
 * 为什么这不是复述代码 / Why the assertions matter:
 *   code->wcet 保存"通过一致性校验的样本"的峰值口径，是 12500 cycles 截止判定
 *   的唯一依据；peak_* 是各分段的独立峰值，仅用于定位热点，**不能相加**。
 *   若把被拒绝的样本也算进 WCET，板上就会误报或漏报 deadline miss。
 *   code->wcet keeps the worst case over samples that passed the consistency check
 *   and is the only input to the 12500-cycle deadline verdict, while peak_* fields
 *   are the independent per-segment peaks used to locate hot spots and must never
 *   be summed. Counting a rejected sample would make the board report deadline
 *   misses incorrectly, in either direction.
 *
 * 返回 / Returns: void。失败即 assert 终止进程，由 test.ps1 判定。
 * void; a failure aborts through assert and is reported by test.ps1.
 */
static void test_correlated_realtime_timing(void)
{
    foc_realtime_timing_stats_t stats;
    /* 首个样本：total 100 = 20 + 60 + 20 [cycles]，满足同拍不变量，因此应被接受并
     * 直接成为 WCET 基线。
     * First sample: total 100 = 20 + 60 + 20 cycles, satisfying the same-tick
     * invariant, so it must be accepted and becomes the WCET baseline. */
    foc_realtime_timing_sample_t sample = {1U, 100U, 20U, 60U, 20U, 0U, 0U};

    foc_realtime_timing_reset(&stats);
    assert(stats.struct_size == sizeof(stats));
    assert(stats.version == FOC_REALTIME_TIMING_VERSION);
    assert(foc_realtime_timing_record(&stats, &sample) == 1U);
    assert(stats.sample_count == 1U);
    assert(stats.wcet.step == 1U);
    assert(stats.wcet.total_cycles == 100U);
    assert(stats.wcet.precontrol_cycles == 20U);
    assert(stats.wcet.control_cycles == 60U);
    assert(stats.wcet.postcontrol_cycles == 20U);

    sample.step = 2U;
    sample.total_cycles = 90U;
    sample.precontrol_cycles = 30U;
    sample.control_cycles = 50U;
    sample.postcontrol_cycles = 10U;
    sample.trace_enabled = 1U;
    sample.trace_sampled = 1U;
    /* 第二拍更快（90 < 100），因此返回值 0（本拍不刷新 WCET），但本拍仍被接受：
     * 分段峰值照常更新，control 保持第一拍的 60。peak_* 与 wcet.* 不是同一口径，
     * 这也是"禁止把三个分段峰值相加"的原因。
     * The second tick is faster (90 < 100), so the return value is 0 (this tick does
     * not refresh the WCET) even though the tick was accepted: the per-segment
     * peaks are still updated and control keeps tick 1's 60. peak_* and wcet.* are
     * different quantities, which is why those peaks must never be summed. */
    assert(foc_realtime_timing_record(&stats, &sample) == 0U);
    assert(stats.wcet.step == 1U);
    assert(stats.peak_precontrol_cycles == 30U);
    assert(stats.peak_control_cycles == 60U);
    assert(stats.peak_postcontrol_cycles == 20U);

    sample.step = 3U;
    sample.total_cycles = 120U;
    sample.precontrol_cycles = 25U;
    sample.control_cycles = 70U;
    sample.postcontrol_cycles = 25U;
    sample.trace_enabled = 1U;
    sample.trace_sampled = 0U;
    /* 第三拍更慢（120 > 100），刷新 WCET；trace_enabled 与 trace_sampled 必须被
     * 一起记录下来，否则无法判断某个 WCET 数字是否已包含 trace 采样开销。
     * The third tick is slower (120 > 100) and refreshes the WCET. trace_enabled and
     * trace_sampled have to be recorded together, otherwise a WCET figure cannot be
     * attributed to a tracing or non-tracing configuration. */
    assert(foc_realtime_timing_record(&stats, &sample) == 1U);
    assert(stats.wcet.step == 3U);
    assert(stats.wcet.trace_enabled == 1U);
    assert(stats.wcet.trace_sampled == 0U);

    /* 分段和 25 + 70 + 24 = 119 != total 120：这一拍是同拍不变量的反例，必须被
     * 拒绝且只抬高 invalid 计数，既不能进 WCET 也不能改 sample_count。
     * Segments sum to 25 + 70 + 24 = 119 but total says 120: this tick violates the
     * same-tick invariant, so it must be rejected and only raise the invalid
     * counter; it may neither enter the WCET nor change sample_count. */
    sample.postcontrol_cycles = 24U;
    assert(foc_realtime_timing_record(&stats, &sample) == 0U);
    assert(stats.invalid_sample_count == 1U);
    assert(stats.sample_count == 3U);
}

/* 256 拍固定窗必须精确停止且不覆盖最早样本；采集中禁止消费。 */
static void test_phase_voltage_capture(void)
{
    foc_phase_voltage_capture_t capture;
    foc_phase_voltage_capture_status_t status;
    foc_phase_voltage_sample_t input = {0};
    foc_phase_voltage_sample_t output = {0};
    uint32_t index;

    foc_phase_voltage_capture_init(&capture, 12000U);
    foc_phase_voltage_capture_get_status(&capture, &status);
    assert(status.struct_size == sizeof(status));
    assert(status.version == FOC_PHASE_VOLTAGE_CAPTURE_VERSION);
    assert(status.state == FOC_PHASE_VOLTAGE_CAPTURE_IDLE);
    assert(status.divider_mode == FOC_PHASE_VOLTAGE_DIVIDER_DISABLED);
    assert(status.capacity == 256U);
    assert(status.sample_count == 0U);
    assert(foc_phase_voltage_capture_arm(
               &capture, (foc_phase_voltage_divider_mode_t)2) == 0U);
    assert(foc_phase_voltage_capture_arm(
               &capture, FOC_PHASE_VOLTAGE_DIVIDER_ENABLED) == 1U);
    assert(foc_phase_voltage_capture_arm(
               &capture, FOC_PHASE_VOLTAGE_DIVIDER_DISABLED) == 0U);

    input.phase_u_raw = 101U;
    input.phase_v_raw = 202U;
    input.phase_w_raw = 303U;
    assert(foc_phase_voltage_capture_record_isr(&capture, &input) == 1U);
    assert(foc_phase_voltage_capture_pop(&capture, &output) == 0U);
    for (index = 1U; index < FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY; ++index)
    {
        input.phase_u_raw = (uint16_t)(101U + index);
        assert(foc_phase_voltage_capture_record_isr(&capture, &input) == 1U);
    }
    foc_phase_voltage_capture_get_status(&capture, &status);
    assert(status.state == FOC_PHASE_VOLTAGE_CAPTURE_COMPLETE);
    assert(status.divider_mode == FOC_PHASE_VOLTAGE_DIVIDER_ENABLED);
    assert(status.sample_rate_hz == 12000U);
    assert(status.sample_count == FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY);
    assert(status.unread_count == FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY);
    assert(foc_phase_voltage_capture_record_isr(&capture, &input) == 0U);
    assert(foc_phase_voltage_capture_pop(&capture, &output) == 1U);
    assert(output.sequence == 0U);
    assert(output.phase_u_raw == 101U);
    assert(output.phase_v_raw == 202U);
    assert(output.phase_w_raw == 303U);
    while (foc_phase_voltage_capture_pop(&capture, &output) != 0U)
    {
    }
    assert(output.sequence == 255U);
    assert(output.phase_u_raw == 356U);
    foc_phase_voltage_capture_get_status(&capture, &status);
    assert(status.unread_count == 0U);

    assert(foc_phase_voltage_capture_arm(
               &capture, FOC_PHASE_VOLTAGE_DIVIDER_DISABLED) == 1U);
    assert(foc_phase_voltage_capture_record_isr(&capture, &input) == 1U);
    foc_phase_voltage_capture_stop(&capture);
    foc_phase_voltage_capture_get_status(&capture, &status);
    assert(status.state == FOC_PHASE_VOLTAGE_CAPTURE_COMPLETE);
    assert(status.divider_mode == FOC_PHASE_VOLTAGE_DIVIDER_DISABLED);
    assert(status.sample_count == 1U);
}

/*
 * 官方元件值只形成 nominal/diagnostic 模型：可做 raw->mV 换算，但不能携带
 * BOARD_CALIBRATED 或 OBSERVER_ELIGIBLE。divider-off 原始码没有物理换算意义。
 */
static void test_phase_voltage_nominal_model(void)
{
    foc_phase_voltage_model_t model =
    {
        sizeof(foc_phase_voltage_model_t),
        FOC_PHASE_VOLTAGE_MODEL_VERSION,
        FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL,
        FOC_PHASE_VOLTAGE_MODEL_FLAG_NOMINAL_COMPONENTS |
            FOC_PHASE_VOLTAGE_MODEL_FLAG_DIAGNOSTIC_ONLY,
        3300U,
        4095U,
        10000U,
        2200U,
        18300U,
        4469U,
    };
    uint32_t phase_mv = 123U;

    assert(foc_phase_voltage_model_validate(&model) == 1U);
    assert(foc_phase_voltage_raw_to_mv(
               &model, FOC_PHASE_VOLTAGE_DIVIDER_ENABLED, 0U, &phase_mv) == 1U);
    assert(phase_mv == 0U);
    assert(foc_phase_voltage_raw_to_mv(
               &model, FOC_PHASE_VOLTAGE_DIVIDER_ENABLED, 2048U, &phase_mv) == 1U);
    assert(phase_mv == 9152U);
    assert(foc_phase_voltage_raw_to_mv(
               &model, FOC_PHASE_VOLTAGE_DIVIDER_ENABLED, 4095U, &phase_mv) == 1U);
    assert(phase_mv == 18300U);
    assert(foc_phase_voltage_raw_to_mv(
               &model, FOC_PHASE_VOLTAGE_DIVIDER_DISABLED, 2048U, &phase_mv) == 0U);
    assert(phase_mv == 0U);
    assert(foc_phase_voltage_raw_to_mv(
               &model, FOC_PHASE_VOLTAGE_DIVIDER_ENABLED, 4096U, &phase_mv) == 0U);
    assert(phase_mv == 0U);

    model.flags |= FOC_PHASE_VOLTAGE_MODEL_FLAG_OBSERVER_ELIGIBLE;
    assert(foc_phase_voltage_model_validate(&model) == 0U);
    model.source = FOC_PHASE_VOLTAGE_MODEL_BOARD_CALIBRATED;
    model.flags = FOC_PHASE_VOLTAGE_MODEL_FLAG_BOARD_CALIBRATED;
    assert(foc_phase_voltage_model_validate(&model) == 0U);
}

/*
 * 测试入口：ABI 触发线 -> 主机数学后端 -> 安全空状态顺序执行。
 * Test entry: ABI tripwires, then the host math backend, then the safe empty state.
 *
 * 失败语义 / Failure semantics:
 *   任何 assert 失败即中止进程并返回非零；test.ps1 会把它当作 C 侧回归失败。
 *   Any failed assert aborts the process with a non-zero status, which test.ps1
 *   treats as a C-side regression failure.
 *
 * 上下文 / Context: 主机（x86/ARM PC），单线程，无 RTOS、无中断、无硬件。
 * Host (x86/ARM PC), single-threaded, no RTOS, no interrupts and no hardware.
 */
int main(void)
{
    foc_feedback_t feedback = {0};
    /* 三相占空比按归一化 [0,1] 给出，1.0 表示上桥臂全通；这里故意给满值，用来
     * 证明未配置的平台连"输出满占空比"都会拒绝，而不是写进比较寄存器。
     * Three-phase duty is normalised to [0,1] where 1.0 is a fully on high side.
     * The value is deliberately at full scale so the test proves an unconfigured
     * platform refuses even a maximum-duty output instead of writing the compare
     * registers. */
    foc_output_t output = {1.0f, 1.0f, 1.0f};
    foc_platform_diagnostics_t diagnostics = {0};
    foc_realtime_timing_stats_t timing = {0};
    foc_math_benchmark_result_t math_benchmark = {0};
    foc_math_health_t math_health = {0};
    float sin_value = 1.0f;
    float cos_value = 1.0f;
    float magnitude = 1.0f;
    float angle = 1.0f;

    test_production_profile();
    test_pwm_timing_contract();
    test_correlated_realtime_timing();
    test_phase_voltage_capture();
    test_phase_voltage_nominal_model();

    /* ABI 触发线 / ABI tripwires:
     *   ABI 版本必须与 foc/include/foc_rust_bridge.h 中的 FOC_RUST_ABI_VERSION
     *   完全一致（当前 0x00110000）；上下文容量必须与静态断言一致；运行时配置 296 B、
     *   遥测 100 B 是 C 与 Rust 双方共同约定的结构体尺寸。改任何一处都必须同时
     *   提升 ABI 版本并更新本测试，否则板上应拒绝启动（见 applications/main.c）。
     *   The ABI version must match FOC_RUST_ABI_VERSION in
     *   foc/include/foc_rust_bridge.h exactly (currently 0x00110000); the context
     *   capacity must match its _Static_assert; and 296 B for the runtime
     *   configuration and 100 B
     *   for telemetry are the struct sizes both sides agreed on. Any change requires
     *   bumping the ABI version and updating this test, otherwise the board must
     *   refuse to start (see applications/main.c). */
    assert(FOC_RUST_ABI_VERSION == 0x00110000UL);
    assert(sizeof(foc_rust_context_t) == FOC_RUST_CONTEXT_CAPACITY);
    assert(sizeof(foc_observer_run_reliability_config_t) == 8U);
    assert(sizeof(foc_angle_compensation_config_t) == 8U);
    assert(sizeof(foc_inverter_voltage_model_config_t) == 36U);
    assert(sizeof(foc_runtime_config_t) == 296U);
    assert(sizeof(foc_telemetry_t) == 100U);
    assert(sizeof(foc_trace_sample_t) == 72U);
    assert(sizeof(foc_phase_voltage_sample_t) == 16U);
    assert(sizeof(foc_phase_voltage_model_t) == 40U);
    /* 主机没有 STM32G4 CORDIC，后端必须是 CPU；加速函数在 CPU 后端下返回 0 且
     * 把输出清零，因此两个断言要一起看：调用被拒绝，且没有泄漏未初始化值。
     * There is no STM32G4 CORDIC on the host, so the backend must be CPU; on that
     * backend the accelerated helpers return 0 and clear their output parameters,
     * so both assertions matter: the call is refused and no uninitialised value
     * escapes. */
    assert(foc_math_accel_backend() == FOC_MATH_BACKEND_CPU);
    assert(foc_math_accel_sin_cos(0.0f, &sin_value, &cos_value) == 0U);
    assert(sin_value == 0.0f && cos_value == 0.0f);
    assert(foc_math_accel_magnitude(3.0f, 4.0f, &magnitude) == 0U);
    assert(magnitude == 0.0f);
    assert(foc_math_accel_atan2(1.0f, 0.0f, &angle) == 0U);
    assert(angle == 0.0f);
    assert(foc_math_accel_benchmark(&math_benchmark, 64U) == 0U);
    assert(math_benchmark.struct_size == sizeof(math_benchmark));
    assert(math_benchmark.version == FOC_MATH_BENCHMARK_VERSION);
    assert(math_benchmark.fixed_delay_nops == 0U);
    assert(math_benchmark.realtime_uses_fixed_delay == 0U);
    assert(math_benchmark.fast_approx_available == 0U);
    assert(math_benchmark.realtime_uses_fast_approx == 0U);
    /* 主机桩也要保留诊断 API 的确定语义：三种调用各要求一次回退；reset 必须
     * 同时清掉计数与 pending，且 CPU 后端拒绝注入。
     * The host stub keeps deterministic diagnostic semantics too: each operation
     * requires one fallback; reset clears counters and pending state, and the CPU
     * backend refuses injection. */
    assert(sizeof(math_health) == 68U);
    assert(foc_math_accel_health_get(&math_health) == 1U);
    assert(math_health.struct_size == sizeof(math_health));
    assert(math_health.version == FOC_MATH_HEALTH_VERSION);
    assert(math_health.sin_cos_calls == 1U);
    assert(math_health.sin_cos_fallback_required == 1U);
    assert(math_health.magnitude_calls == 1U);
    assert(math_health.magnitude_fallback_required == 1U);
    assert(math_health.atan2_calls == 1U);
    assert(math_health.atan2_fallback_required == 1U);
    assert(foc_math_accel_inject_not_ready_once(
               FOC_MATH_INJECT_OPERATION_SIN_COS) == 0U);
    foc_math_accel_health_reset();
    assert(foc_math_accel_health_get(&math_health) == 1U);
    assert(math_health.sin_cos_calls == 0U);
    assert(math_health.magnitude_calls == 0U);
    assert(math_health.atan2_calls == 0U);
    assert(math_health.pending_injection_operation ==
           FOC_MATH_INJECT_OPERATION_NONE);

    /* 安全空状态 / Safe empty state：本测试刻意不调用 foc_platform_configure()，
     * 因此平台必须对所有"会动硬件"的入口说 NOT_CONFIGURED。
     * The platform is deliberately left unconfigured here, so every entry point that
     * could touch hardware must answer NOT_CONFIGURED. */
    foc_platform_emergency_stop();
    assert(foc_platform_init() == FOC_STATUS_NOT_CONFIGURED);
    /* 读反馈失败时结构体必须已清零：上位脚本会直接打印这些 [A] / [V] 字段，未定义
     * 值会被误读成真实测量。
     * A failed feedback read must leave the struct zeroed: host scripts print these
     * fields in A and V directly, and undefined values would read as measurements. */
    assert(foc_platform_read_feedback(&feedback) == FOC_STATUS_NOT_CONFIGURED);
    assert(feedback.phase_current_a == 0.0f);
    assert(feedback.phase_current_b == 0.0f);
    assert(feedback.phase_current_c == 0.0f);
    assert(feedback.dc_bus_voltage == 0.0f);
    /* trial_arm 是早期试转包络的兼容入口（实时路径用 foc_platform_control_start），
     * 这里保留它是为了证明"未配置时连兼容路径也不能 arm"。
     * trial_arm belongs to the legacy bounded trial envelope (the realtime path uses
     * foc_platform_control_start); it is exercised to prove that not even the legacy
     * path can arm an unconfigured platform. */
    assert(foc_platform_trial_arm() == FOC_STATUS_NOT_CONFIGURED);
    assert(foc_platform_apply_output(&output) == FOC_STATUS_NOT_CONFIGURED);
    foc_platform_trial_disarm();
    /* 空状态下诊断全 0，尤其是 flags 里不能出现 GATE_SAFE/OUTPUT_ACTIVE 这类
     * "看起来像证据"的位；pwm_frequency_hz == 0 就是 foc_trace 拒绝启动的依据。
     * In the empty state the diagnostics are all zero, in particular flags must not
     * claim evidence bits such as GATE_SAFE or OUTPUT_ACTIVE; pwm_frequency_hz == 0
     * is exactly what makes foc_trace refuse to start. */
    assert(foc_platform_get_diagnostics(&diagnostics) == FOC_STATUS_OK);
    assert(diagnostics.flags == 0U);
    assert(diagnostics.pwm_frequency_hz == 0U);
    assert(diagnostics.control_frequency_hz == 0U);
    assert(foc_platform_phase_voltage_capture_start(
               FOC_PHASE_VOLTAGE_DIVIDER_DISABLED) ==
           FOC_STATUS_NOT_CONFIGURED);
    assert(foc_platform_phase_voltage_capture_status(0) ==
           FOC_STATUS_INVALID_ARGUMENT);
    {
        foc_phase_voltage_capture_status_t phase_status = {0};
        foc_phase_voltage_sample_t phase_sample = {0};
        foc_phase_voltage_model_t phase_model = {0};
        uint32_t phase_mv = 0U;

        assert(foc_platform_phase_voltage_capture_status(&phase_status) ==
               FOC_STATUS_NOT_CONFIGURED);
        assert(phase_status.struct_size == sizeof(phase_status));
        assert(phase_status.state == FOC_PHASE_VOLTAGE_CAPTURE_IDLE);
        assert(foc_platform_phase_voltage_capture_pop(&phase_sample) == 0U);
        /* 编译期板卡描述不是 live hardware 状态，空平台也允许只读获取。 */
        assert(foc_platform_get_phase_voltage_model(&phase_model) == FOC_STATUS_OK);
        assert(phase_model.source ==
               FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL);
        assert((phase_model.flags &
                FOC_PHASE_VOLTAGE_MODEL_FLAG_OBSERVER_ELIGIBLE) == 0U);
        assert(foc_platform_phase_voltage_raw_to_mv(
                   FOC_PHASE_VOLTAGE_DIVIDER_ENABLED, 4095U, &phase_mv) ==
               FOC_STATUS_OK);
        assert(phase_mv == 18300U);
    }
    /* 时序统计的自检字段也必须成立：struct_size 是 ABI 自检，version 是数据版本，
     * 读取方用它来判断这块数据能否按当前头文件的布局解释。
     * The timing statistics self-check fields must hold as well: struct_size is the
     * ABI self-check and version is the data version, which a reader uses to decide
     * whether this data may be interpreted with the current header layout. */
    assert(foc_platform_get_timing(&timing) == FOC_STATUS_OK);
    assert(timing.struct_size == sizeof(timing));
    assert(timing.version == FOC_REALTIME_TIMING_VERSION);
    assert(timing.sample_count == 0U);

    puts("FOC C PLATFORM SAFE EMPTY STATE: PASS");
    return 0;
}
