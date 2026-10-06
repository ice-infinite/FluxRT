#ifndef FOC_TIME_SYNC_WIRE_H
#define FOC_TIME_SYNC_WIRE_H

#include <stddef.h>
#include <stdint.h>

#include "foc_time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Target-neutral, fixed-capacity H3 edge-identity wire codec. Hardware ports
 * translate each high/low pair to timer DMA or RMT symbols; this module never
 * touches GPIO, timers, DMA, an RTOS or a vendor SDK. */
#define FOC_TIME_SYNC_WIRE_ABI_VERSION    (0x00010000UL)
#define FOC_TIME_SYNC_WIRE_VERSION        (1U)
#define FOC_TIME_SYNC_WIRE_MAGIC_0        (0xA5U)
#define FOC_TIME_SYNC_WIRE_MAGIC_1        (0x5AU)
#define FOC_TIME_SYNC_WIRE_PACKET_BYTES   (20UL)
#define FOC_TIME_SYNC_WIRE_PAYLOAD_BYTES  (16UL)
#define FOC_TIME_SYNC_WIRE_DATA_BITS      (160UL)
#define FOC_TIME_SYNC_WIRE_SYMBOL_COUNT   (161UL)

typedef enum
{
    FOC_TIME_SYNC_WIRE_DECODER_IDLE = 0,
    FOC_TIME_SYNC_WIRE_DECODER_RECEIVING = 1,
    FOC_TIME_SYNC_WIRE_DECODER_COMPLETE = 2,
    FOC_TIME_SYNC_WIRE_DECODER_FAILED = 3,
} foc_time_sync_wire_decoder_state_t;

typedef enum
{
    FOC_TIME_SYNC_WIRE_FAILURE_NONE = 0,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_ARGUMENT = 1,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_TIMING = 2,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_SOF = 3,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_SYMBOL = 4,
    FOC_TIME_SYNC_WIRE_FAILURE_TRUNCATED = 5,
    FOC_TIME_SYNC_WIRE_FAILURE_TOO_MANY_SYMBOLS = 6,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_MAGIC = 7,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_VERSION = 8,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_LENGTH = 9,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_IDENTITY = 10,
    FOC_TIME_SYNC_WIRE_FAILURE_BAD_CRC = 11,
} foc_time_sync_wire_failure_t;

typedef struct
{
    uint16_t high_ticks;
    uint16_t low_ticks;
} foc_time_sync_wire_symbol_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint16_t short_high_ticks;
    uint16_t long_high_ticks;
    uint16_t data_low_ticks;
    uint16_t sof_high_ticks;
    uint16_t sof_low_ticks;
    uint16_t tolerance_ticks;
} foc_time_sync_wire_timing_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t state;
    uint32_t failure_reason;
    uint32_t accepted_symbols;
    uint32_t decoded_bits;
} foc_time_sync_wire_status_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t state;
    uint32_t failure_reason;
    uint32_t accepted_symbols;
    uint32_t decoded_bits;
    foc_time_sync_wire_timing_t timing;
    uint8_t packet[FOC_TIME_SYNC_WIRE_PACKET_BYTES];
} foc_time_sync_wire_decoder_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_time_sync_wire_symbol_t) == 4U,
               "time-sync wire symbol size mismatch");
_Static_assert(sizeof(foc_time_sync_wire_timing_t) == 20U,
               "time-sync wire timing size mismatch");
_Static_assert(sizeof(foc_time_sync_wire_status_t) == 24U,
               "time-sync wire status size mismatch");
_Static_assert(sizeof(foc_time_sync_wire_decoder_t) == 64U,
               "time-sync wire decoder size mismatch");
#endif

uint32_t foc_time_sync_wire_timing_valid(
    const foc_time_sync_wire_timing_t *timing);
uint32_t foc_time_sync_wire_encode(
    const foc_time_sync_edge_identity_t *identity,
    const foc_time_sync_wire_timing_t *timing,
    foc_time_sync_wire_symbol_t *symbols,
    uint32_t symbol_capacity,
    uint32_t *symbol_count);

uint32_t foc_time_sync_wire_decoder_init(
    foc_time_sync_wire_decoder_t *decoder,
    const foc_time_sync_wire_timing_t *timing);
void foc_time_sync_wire_decoder_reset(
    foc_time_sync_wire_decoder_t *decoder);
uint32_t foc_time_sync_wire_decoder_feed(
    foc_time_sync_wire_decoder_t *decoder,
    const foc_time_sync_wire_symbol_t *symbol);
uint32_t foc_time_sync_wire_decoder_finish(
    foc_time_sync_wire_decoder_t *decoder);
uint32_t foc_time_sync_wire_decoder_get_identity(
    const foc_time_sync_wire_decoder_t *decoder,
    foc_time_sync_edge_identity_t *identity);
void foc_time_sync_wire_decoder_get_status(
    const foc_time_sync_wire_decoder_t *decoder,
    foc_time_sync_wire_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TIME_SYNC_WIRE_H */
