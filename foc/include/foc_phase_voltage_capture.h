#ifndef FOC_PHASE_VOLTAGE_CAPTURE_H
#define FOC_PHASE_VOLTAGE_CAPTURE_H

/*
 * FluxRT —— 三相端电压短窗采集器。
 * FluxRT - short-window three-phase terminal-voltage capture.
 *
 * 这是纯 C、无硬件依赖的数据容器。STM32 平台 ISR 负责把同步 ADC 原始值送进来，
 * Shell 线程只在采集停止后读取，因此没有动态分配、锁或覆盖旧样本。
 * This is a hardware-independent C data container. The STM32 ISR feeds
 * synchronous raw ADC values and the shell reads only after capture stops, so
 * there is no allocation, locking, or overwriting of old samples.
 */

#include <stdint.h>

#define FOC_PHASE_VOLTAGE_CAPTURE_VERSION  (2UL)
#define FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY (256UL)

typedef enum
{
    FOC_PHASE_VOLTAGE_CAPTURE_IDLE = 0,
    FOC_PHASE_VOLTAGE_CAPTURE_ARMED = 1,
    FOC_PHASE_VOLTAGE_CAPTURE_COMPLETE = 2,
} foc_phase_voltage_capture_state_t;

/* PC9 分压网络模式。disabled 必须为 0，使初始化/异常路径天然保持断开。 */
typedef enum
{
    FOC_PHASE_VOLTAGE_DIVIDER_DISABLED = 0,
    FOC_PHASE_VOLTAGE_DIVIDER_ENABLED = 1,
} foc_phase_voltage_divider_mode_t;

/* 16 字节/拍；256 拍共 4096 字节，只在 Diagnostic/Calibration 目标上实例化。 */
typedef struct
{
    uint32_t sequence;
    uint16_t phase_u_raw;
    uint16_t phase_v_raw;
    uint16_t phase_w_raw;
    uint16_t current_u_raw;
    uint16_t current_v_raw;
    uint16_t bus_voltage_raw;
} foc_phase_voltage_sample_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t state;
    uint32_t divider_mode;
    uint32_t sample_rate_hz;
    uint32_t capacity;
    uint32_t sample_count;
    uint32_t unread_count;
} foc_phase_voltage_capture_status_t;

typedef struct
{
    volatile uint32_t state;
    volatile uint32_t divider_mode;
    volatile uint32_t write_index;
    uint32_t read_index;
    uint32_t sample_rate_hz;
    foc_phase_voltage_sample_t samples[FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY];
} foc_phase_voltage_capture_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_phase_voltage_sample_t) == 16U,
               "phase-voltage sample ABI changed");
_Static_assert(FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY == 256U,
               "capture capacity is part of the diagnostic contract");
#endif

void foc_phase_voltage_capture_init(foc_phase_voltage_capture_t *capture,
                                    uint32_t sample_rate_hz);
uint32_t foc_phase_voltage_capture_arm(
    foc_phase_voltage_capture_t *capture,
    foc_phase_voltage_divider_mode_t divider_mode);
uint32_t foc_phase_voltage_capture_record_isr(
    foc_phase_voltage_capture_t *capture,
    const foc_phase_voltage_sample_t *sample);
void foc_phase_voltage_capture_stop(foc_phase_voltage_capture_t *capture);
uint32_t foc_phase_voltage_capture_pop(foc_phase_voltage_capture_t *capture,
                                      foc_phase_voltage_sample_t *sample);
void foc_phase_voltage_capture_get_status(
    const foc_phase_voltage_capture_t *capture,
    foc_phase_voltage_capture_status_t *status);

#endif
