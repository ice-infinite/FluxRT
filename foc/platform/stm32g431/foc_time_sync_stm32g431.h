#ifndef FOC_TIME_SYNC_STM32G431_H
#define FOC_TIME_SYNC_STM32G431_H

#include "foc_time_sync.h"
#include "foc_time_sync_toggle_plan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The adapter remains target-neutral at its boundary.  The transmitter below
 * is an explicit diagnostic port: it is inert until init/start are called and
 * never touches TIM1, the power stage, or a sensorless capability bit. */
uint32_t foc_time_sync_stm32g431_adapter_init(
    foc_time_sync_adapter_t *adapter,
    foc_time_sync_capture_t *capture,
    void *port_context,
    foc_time_sync_read_local_tick_isr_fn read_local_tick_isr,
    foc_time_sync_read_control_tick_isr_fn read_control_tick_isr);

#define FOC_TIME_SYNC_STM32G431_TX_ABI_VERSION (0x00030000UL)
#define FOC_TIME_SYNC_STM32G431_TIMER_HZ       (1000000UL)
#define FOC_TIME_SYNC_STM32G431_CYCLE_HZ       (170000000UL)
#define FOC_TIME_SYNC_STM32G431_TX_GPIO_PORT   (2UL) /* GPIOB */
#define FOC_TIME_SYNC_STM32G431_TX_GPIO_PIN    (6UL)

#define FOC_TIME_SYNC_STM32G431_EDGE_CYCLE_VALID  (1UL << 0)
#define FOC_TIME_SYNC_STM32G431_CONTROL_TICK_VALID (1UL << 1)
#define FOC_TIME_SYNC_STM32G431_LOOPBACK_VALID     (1UL << 2)

typedef enum
{
    FOC_TIME_SYNC_STM32G431_TX_UNINITIALIZED = 0,
    FOC_TIME_SYNC_STM32G431_TX_READY = 1,
    FOC_TIME_SYNC_STM32G431_TX_BUSY = 2,
    FOC_TIME_SYNC_STM32G431_TX_COMPLETE = 3,
    FOC_TIME_SYNC_STM32G431_TX_FAILED = 4,
} foc_time_sync_stm32g431_tx_state_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t state;
    uint32_t started_count;
    uint32_t completed_count;
    uint32_t failed_count;
    uint32_t first_edge_count;
    uint32_t first_edge_timer_tick;
    uint32_t first_edge_cycle_tick;
    uint32_t first_edge_control_tick;
    uint32_t first_edge_flags;
    uint32_t loopback_edge_count;
    uint32_t loopback_edge_timer_tick;
    uint32_t loopback_edge_cycle_tick;
    int32_t loopback_delta_cycles;
    uint32_t last_session_id;
    uint32_t last_edge_sequence;
    uint32_t last_edge_tag;
    uint32_t last_hal_status;
} foc_time_sync_stm32g431_tx_status_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_time_sync_stm32g431_tx_status_t) == 76U,
               "STM32G431 time-sync TX status ABI size mismatch");
#endif

uint32_t foc_time_sync_stm32g431_tx_init(void);
uint32_t foc_time_sync_stm32g431_tx_start(
    const foc_time_sync_edge_identity_t *identity);
void foc_time_sync_stm32g431_tx_abort(void);
void foc_time_sync_stm32g431_tx_get_status(
    foc_time_sync_stm32g431_tx_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TIME_SYNC_STM32G431_H */
