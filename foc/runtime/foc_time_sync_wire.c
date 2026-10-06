#include "foc_time_sync_wire.h"

#include <string.h>

static uint32_t foc_time_sync_wire_crc32(const uint8_t *data,
                                         uint32_t length)
{
    uint32_t crc = UINT32_MAX;
    uint32_t byte_index;
    uint32_t bit_index;

    for (byte_index = 0U; byte_index < length; ++byte_index)
    {
        crc ^= (uint32_t)data[byte_index];
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static void foc_time_sync_wire_store_u32_le(uint8_t *output,
                                            uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8U) & 0xFFU);
    output[2] = (uint8_t)((value >> 16U) & 0xFFU);
    output[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint32_t foc_time_sync_wire_load_u32_le(const uint8_t *input)
{
    return (uint32_t)input[0] |
           ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) |
           ((uint32_t)input[3] << 24U);
}

static uint32_t foc_time_sync_wire_in_range(uint16_t actual,
                                            uint16_t expected,
                                            uint16_t tolerance)
{
    const uint32_t actual_u32 = (uint32_t)actual;
    const uint32_t expected_u32 = (uint32_t)expected;
    const uint32_t tolerance_u32 = (uint32_t)tolerance;
    const uint32_t minimum = (expected_u32 > tolerance_u32) ?
                                 (expected_u32 - tolerance_u32) :
                                 0U;
    const uint32_t maximum = expected_u32 + tolerance_u32;
    return ((actual_u32 >= minimum) && (actual_u32 <= maximum)) ? 1U : 0U;
}

static uint32_t foc_time_sync_wire_identity_valid(
    const foc_time_sync_edge_identity_t *identity)
{
    if ((identity == NULL) ||
        (identity->struct_size != sizeof(*identity)) ||
        (identity->version != FOC_TIME_SYNC_IDENTITY_VERSION) ||
        (identity->session_id == 0U) || (identity->edge_tag == 0U))
    {
        return 0U;
    }
    if ((identity->flags & FOC_TIME_SYNC_FLAG_RISING_EDGE) == 0U)
    {
        return 0U;
    }
    return ((identity->flags & ~FOC_TIME_SYNC_FLAG_ALLOWED_MASK) == 0U) ? 1U
                                                                        : 0U;
}

static void foc_time_sync_wire_fail(foc_time_sync_wire_decoder_t *decoder,
                                    uint32_t failure_reason)
{
    decoder->failure_reason = failure_reason;
    decoder->state = (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_FAILED;
}

uint32_t foc_time_sync_wire_timing_valid(
    const foc_time_sync_wire_timing_t *timing)
{
    uint32_t tolerance;
    uint32_t short_high;
    uint32_t long_high;
    uint32_t sof_high;

    if ((timing == NULL) || (timing->struct_size != sizeof(*timing)) ||
        (timing->version != FOC_TIME_SYNC_WIRE_ABI_VERSION) ||
        (timing->short_high_ticks == 0U) ||
        (timing->long_high_ticks == 0U) ||
        (timing->data_low_ticks == 0U) ||
        (timing->sof_high_ticks == 0U) ||
        (timing->sof_low_ticks == 0U))
    {
        return 0U;
    }

    tolerance = (uint32_t)timing->tolerance_ticks;
    short_high = (uint32_t)timing->short_high_ticks;
    long_high = (uint32_t)timing->long_high_ticks;
    sof_high = (uint32_t)timing->sof_high_ticks;
    if ((tolerance >= short_high) ||
        (tolerance >= (uint32_t)timing->long_high_ticks) ||
        (tolerance >= (uint32_t)timing->data_low_ticks) ||
        (tolerance >= (uint32_t)timing->sof_high_ticks) ||
        (tolerance >= (uint32_t)timing->sof_low_ticks) ||
        ((short_high + tolerance) >= (long_high - tolerance)) ||
        ((long_high + tolerance) >= (sof_high - tolerance)))
    {
        return 0U;
    }
    return 1U;
}

uint32_t foc_time_sync_wire_encode(
    const foc_time_sync_edge_identity_t *identity,
    const foc_time_sync_wire_timing_t *timing,
    foc_time_sync_wire_symbol_t *symbols,
    uint32_t symbol_capacity,
    uint32_t *symbol_count)
{
    uint8_t packet[FOC_TIME_SYNC_WIRE_PACKET_BYTES];
    uint32_t crc;
    uint32_t bit_index;

    if (symbol_count != NULL)
    {
        *symbol_count = 0U;
    }
    if ((foc_time_sync_wire_identity_valid(identity) == 0U) ||
        (foc_time_sync_wire_timing_valid(timing) == 0U) ||
        (symbols == NULL) || (symbol_count == NULL) ||
        (symbol_capacity < FOC_TIME_SYNC_WIRE_SYMBOL_COUNT))
    {
        return 0U;
    }

    (void)memset(packet, 0, sizeof(packet));
    packet[0] = FOC_TIME_SYNC_WIRE_MAGIC_0;
    packet[1] = FOC_TIME_SYNC_WIRE_MAGIC_1;
    packet[2] = FOC_TIME_SYNC_WIRE_VERSION;
    packet[3] = (uint8_t)FOC_TIME_SYNC_WIRE_PACKET_BYTES;
    foc_time_sync_wire_store_u32_le(&packet[4], identity->session_id);
    foc_time_sync_wire_store_u32_le(&packet[8], identity->edge_sequence);
    foc_time_sync_wire_store_u32_le(&packet[12], identity->edge_tag);
    crc = foc_time_sync_wire_crc32(packet, FOC_TIME_SYNC_WIRE_PAYLOAD_BYTES);
    foc_time_sync_wire_store_u32_le(&packet[16], crc);

    symbols[0].high_ticks = timing->sof_high_ticks;
    symbols[0].low_ticks = timing->sof_low_ticks;
    for (bit_index = 0U; bit_index < FOC_TIME_SYNC_WIRE_DATA_BITS;
         ++bit_index)
    {
        const uint32_t byte_index = bit_index / 8U;
        const uint32_t byte_bit = bit_index % 8U;
        const uint32_t value =
            ((uint32_t)packet[byte_index] >> byte_bit) & 1U;
        symbols[bit_index + 1U].high_ticks =
            (value == 0U) ? timing->short_high_ticks :
                            timing->long_high_ticks;
        symbols[bit_index + 1U].low_ticks = timing->data_low_ticks;
    }
    *symbol_count = FOC_TIME_SYNC_WIRE_SYMBOL_COUNT;
    return 1U;
}

uint32_t foc_time_sync_wire_decoder_init(
    foc_time_sync_wire_decoder_t *decoder,
    const foc_time_sync_wire_timing_t *timing)
{
    if ((decoder == NULL) ||
        (foc_time_sync_wire_timing_valid(timing) == 0U))
    {
        return 0U;
    }
    (void)memset(decoder, 0, sizeof(*decoder));
    decoder->struct_size = sizeof(*decoder);
    decoder->version = FOC_TIME_SYNC_WIRE_ABI_VERSION;
    decoder->timing = *timing;
    decoder->state = (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_IDLE;
    return 1U;
}

void foc_time_sync_wire_decoder_reset(
    foc_time_sync_wire_decoder_t *decoder)
{
    foc_time_sync_wire_timing_t timing;

    if ((decoder == NULL) || (decoder->struct_size != sizeof(*decoder)) ||
        (decoder->version != FOC_TIME_SYNC_WIRE_ABI_VERSION) ||
        (foc_time_sync_wire_timing_valid(&decoder->timing) == 0U))
    {
        return;
    }
    timing = decoder->timing;
    (void)memset(decoder, 0, sizeof(*decoder));
    decoder->struct_size = sizeof(*decoder);
    decoder->version = FOC_TIME_SYNC_WIRE_ABI_VERSION;
    decoder->timing = timing;
    decoder->state = (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_IDLE;
}

static uint32_t foc_time_sync_wire_decoder_validate_packet(
    foc_time_sync_wire_decoder_t *decoder)
{
    uint32_t expected_crc;
    uint32_t actual_crc;
    uint32_t session_id;
    uint32_t edge_tag;

    if ((decoder->packet[0] != FOC_TIME_SYNC_WIRE_MAGIC_0) ||
        (decoder->packet[1] != FOC_TIME_SYNC_WIRE_MAGIC_1))
    {
        foc_time_sync_wire_fail(
            decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_MAGIC);
        return 0U;
    }
    if (decoder->packet[2] != FOC_TIME_SYNC_WIRE_VERSION)
    {
        foc_time_sync_wire_fail(
            decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_VERSION);
        return 0U;
    }
    if (decoder->packet[3] != (uint8_t)FOC_TIME_SYNC_WIRE_PACKET_BYTES)
    {
        foc_time_sync_wire_fail(
            decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_LENGTH);
        return 0U;
    }
    session_id = foc_time_sync_wire_load_u32_le(&decoder->packet[4]);
    edge_tag = foc_time_sync_wire_load_u32_le(&decoder->packet[12]);
    if ((session_id == 0U) || (edge_tag == 0U))
    {
        foc_time_sync_wire_fail(
            decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_IDENTITY);
        return 0U;
    }
    expected_crc = foc_time_sync_wire_crc32(
        decoder->packet, FOC_TIME_SYNC_WIRE_PAYLOAD_BYTES);
    actual_crc = foc_time_sync_wire_load_u32_le(&decoder->packet[16]);
    if (expected_crc != actual_crc)
    {
        foc_time_sync_wire_fail(
            decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_CRC);
        return 0U;
    }
    decoder->state = (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_COMPLETE;
    return 1U;
}

uint32_t foc_time_sync_wire_decoder_feed(
    foc_time_sync_wire_decoder_t *decoder,
    const foc_time_sync_wire_symbol_t *symbol)
{
    uint32_t matches_short;
    uint32_t matches_long;
    uint32_t byte_index;
    uint32_t byte_bit;

    if ((decoder == NULL) || (symbol == NULL) ||
        (decoder->struct_size != sizeof(*decoder)) ||
        (decoder->version != FOC_TIME_SYNC_WIRE_ABI_VERSION) ||
        (foc_time_sync_wire_timing_valid(&decoder->timing) == 0U))
    {
        return 0U;
    }
    if (decoder->state == (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_COMPLETE)
    {
        foc_time_sync_wire_fail(
            decoder,
            (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_TOO_MANY_SYMBOLS);
        return 0U;
    }
    if (decoder->state == (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_FAILED)
    {
        return 0U;
    }
    if (decoder->state == (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_IDLE)
    {
        if ((foc_time_sync_wire_in_range(
                 symbol->high_ticks,
                 decoder->timing.sof_high_ticks,
                 decoder->timing.tolerance_ticks) == 0U) ||
            (foc_time_sync_wire_in_range(
                 symbol->low_ticks,
                 decoder->timing.sof_low_ticks,
                 decoder->timing.tolerance_ticks) == 0U))
        {
            foc_time_sync_wire_fail(
                decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_SOF);
            return 0U;
        }
        decoder->state =
            (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_RECEIVING;
        decoder->accepted_symbols = 1U;
        return 1U;
    }

    if (decoder->decoded_bits >= FOC_TIME_SYNC_WIRE_DATA_BITS)
    {
        foc_time_sync_wire_fail(
            decoder,
            (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_TOO_MANY_SYMBOLS);
        return 0U;
    }
    if (foc_time_sync_wire_in_range(
            symbol->low_ticks,
            decoder->timing.data_low_ticks,
            decoder->timing.tolerance_ticks) == 0U)
    {
        foc_time_sync_wire_fail(
            decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_SYMBOL);
        return 0U;
    }
    matches_short = foc_time_sync_wire_in_range(
        symbol->high_ticks,
        decoder->timing.short_high_ticks,
        decoder->timing.tolerance_ticks);
    matches_long = foc_time_sync_wire_in_range(
        symbol->high_ticks,
        decoder->timing.long_high_ticks,
        decoder->timing.tolerance_ticks);
    if ((matches_short + matches_long) != 1U)
    {
        foc_time_sync_wire_fail(
            decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_SYMBOL);
        return 0U;
    }

    byte_index = decoder->decoded_bits / 8U;
    byte_bit = decoder->decoded_bits % 8U;
    if (matches_long != 0U)
    {
        decoder->packet[byte_index] |= (uint8_t)(1U << byte_bit);
    }
    ++decoder->decoded_bits;
    ++decoder->accepted_symbols;
    if (decoder->decoded_bits == FOC_TIME_SYNC_WIRE_DATA_BITS)
    {
        return foc_time_sync_wire_decoder_validate_packet(decoder);
    }
    return 1U;
}

uint32_t foc_time_sync_wire_decoder_finish(
    foc_time_sync_wire_decoder_t *decoder)
{
    if ((decoder == NULL) || (decoder->struct_size != sizeof(*decoder)) ||
        (decoder->version != FOC_TIME_SYNC_WIRE_ABI_VERSION))
    {
        return 0U;
    }
    if (decoder->state == (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_COMPLETE)
    {
        return 1U;
    }
    if (decoder->state != (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_FAILED)
    {
        foc_time_sync_wire_fail(
            decoder, (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_TRUNCATED);
    }
    return 0U;
}

uint32_t foc_time_sync_wire_decoder_get_identity(
    const foc_time_sync_wire_decoder_t *decoder,
    foc_time_sync_edge_identity_t *identity)
{
    if ((decoder == NULL) || (identity == NULL) ||
        (decoder->struct_size != sizeof(*decoder)) ||
        (decoder->version != FOC_TIME_SYNC_WIRE_ABI_VERSION) ||
        (decoder->state !=
         (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_COMPLETE))
    {
        return 0U;
    }
    (void)memset(identity, 0, sizeof(*identity));
    identity->struct_size = sizeof(*identity);
    identity->version = FOC_TIME_SYNC_IDENTITY_VERSION;
    identity->session_id = foc_time_sync_wire_load_u32_le(
        &decoder->packet[4]);
    identity->edge_sequence = foc_time_sync_wire_load_u32_le(
        &decoder->packet[8]);
    identity->edge_tag = foc_time_sync_wire_load_u32_le(
        &decoder->packet[12]);
    identity->flags = FOC_TIME_SYNC_FLAG_RISING_EDGE |
                      FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED;
    return 1U;
}

void foc_time_sync_wire_decoder_get_status(
    const foc_time_sync_wire_decoder_t *decoder,
    foc_time_sync_wire_status_t *status)
{
    if (status == NULL)
    {
        return;
    }
    (void)memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_TIME_SYNC_WIRE_ABI_VERSION;
    if ((decoder == NULL) || (decoder->struct_size != sizeof(*decoder)) ||
        (decoder->version != FOC_TIME_SYNC_WIRE_ABI_VERSION))
    {
        return;
    }
    status->state = decoder->state;
    status->failure_reason = decoder->failure_reason;
    status->accepted_symbols = decoder->accepted_symbols;
    status->decoded_bits = decoder->decoded_bits;
}
