#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "foc_time_sync_wire.h"

static foc_time_sync_wire_timing_t make_timing(void)
{
    foc_time_sync_wire_timing_t timing;
    (void)memset(&timing, 0, sizeof(timing));
    timing.struct_size = sizeof(timing);
    timing.version = FOC_TIME_SYNC_WIRE_ABI_VERSION;
    timing.short_high_ticks = 10U;
    timing.long_high_ticks = 20U;
    timing.data_low_ticks = 10U;
    timing.sof_high_ticks = 40U;
    timing.sof_low_ticks = 30U;
    timing.tolerance_ticks = 2U;
    return timing;
}

static foc_time_sync_edge_identity_t make_identity(uint32_t sequence)
{
    foc_time_sync_edge_identity_t identity;
    (void)memset(&identity, 0, sizeof(identity));
    identity.struct_size = sizeof(identity);
    identity.version = FOC_TIME_SYNC_IDENTITY_VERSION;
    identity.session_id = 0x12345678UL;
    identity.edge_sequence = sequence;
    identity.edge_tag = 0x89ABCDEFUL;
    identity.flags = FOC_TIME_SYNC_FLAG_RISING_EDGE |
                     FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED;
    return identity;
}

static void encode_frame(foc_time_sync_wire_symbol_t *symbols,
                         uint32_t *symbol_count,
                         uint32_t sequence)
{
    const foc_time_sync_wire_timing_t timing = make_timing();
    const foc_time_sync_edge_identity_t identity = make_identity(sequence);
    assert(foc_time_sync_wire_encode(&identity,
                                     &timing,
                                     symbols,
                                     FOC_TIME_SYNC_WIRE_SYMBOL_COUNT,
                                     symbol_count) == 1U);
    assert(*symbol_count == FOC_TIME_SYNC_WIRE_SYMBOL_COUNT);
}

static void test_round_trip_and_sequence_wrap(void)
{
    foc_time_sync_wire_symbol_t symbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT];
    foc_time_sync_wire_decoder_t decoder;
    foc_time_sync_edge_identity_t decoded;
    foc_time_sync_wire_status_t status;
    foc_time_sync_wire_timing_t timing = make_timing();
    uint32_t symbol_count;
    uint32_t index;

    encode_frame(symbols, &symbol_count, UINT32_MAX);
    assert(foc_time_sync_wire_decoder_init(&decoder, &timing) == 1U);
    for (index = 0U; index < symbol_count; ++index)
    {
        assert(foc_time_sync_wire_decoder_feed(&decoder,
                                               &symbols[index]) == 1U);
    }
    assert(foc_time_sync_wire_decoder_finish(&decoder) == 1U);
    assert(foc_time_sync_wire_decoder_get_identity(&decoder, &decoded) == 1U);
    assert(decoded.session_id == 0x12345678UL);
    assert(decoded.edge_sequence == UINT32_MAX);
    assert(decoded.edge_tag == 0x89ABCDEFUL);
    assert(decoded.flags == (FOC_TIME_SYNC_FLAG_RISING_EDGE |
                             FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED));
    foc_time_sync_wire_decoder_get_status(&decoder, &status);
    assert(status.state == (uint32_t)FOC_TIME_SYNC_WIRE_DECODER_COMPLETE);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_NONE);
    assert(status.accepted_symbols == FOC_TIME_SYNC_WIRE_SYMBOL_COUNT);
    assert(status.decoded_bits == FOC_TIME_SYNC_WIRE_DATA_BITS);
}

static void test_wire_bytes_match_golden_vector(void)
{
    static const uint8_t expected[FOC_TIME_SYNC_WIRE_PACKET_BYTES] = {
        0xA5U, 0x5AU, 0x01U, 0x14U,
        0x78U, 0x56U, 0x34U, 0x12U,
        0x0DU, 0x0CU, 0x0BU, 0x0AU,
        0xEFU, 0xCDU, 0xABU, 0x89U,
        0x10U, 0x81U, 0x4AU, 0x3EU,
    };
    foc_time_sync_wire_symbol_t symbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT];
    foc_time_sync_wire_timing_t timing = make_timing();
    foc_time_sync_edge_identity_t identity = make_identity(0x0A0B0C0DUL);
    uint8_t actual[FOC_TIME_SYNC_WIRE_PACKET_BYTES];
    uint32_t symbol_count;
    uint32_t bit_index;

    identity.flags = FOC_TIME_SYNC_FLAG_RISING_EDGE;
    assert(foc_time_sync_wire_encode(&identity,
                                     &timing,
                                     symbols,
                                     FOC_TIME_SYNC_WIRE_SYMBOL_COUNT,
                                     &symbol_count) == 1U);
    (void)memset(actual, 0, sizeof(actual));
    for (bit_index = 0U; bit_index < FOC_TIME_SYNC_WIRE_DATA_BITS;
         ++bit_index)
    {
        if (symbols[bit_index + 1U].high_ticks == timing.long_high_ticks)
        {
            actual[bit_index / 8U] |=
                (uint8_t)(1U << (bit_index % 8U));
        }
        else
        {
            assert(symbols[bit_index + 1U].high_ticks ==
                   timing.short_high_ticks);
        }
    }
    assert(memcmp(actual, expected, sizeof(expected)) == 0);
}

static void test_bad_timing_and_identity_are_rejected(void)
{
    foc_time_sync_wire_symbol_t symbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT];
    foc_time_sync_wire_decoder_t decoder;
    foc_time_sync_wire_timing_t timing = make_timing();
    foc_time_sync_edge_identity_t identity = make_identity(1U);
    uint32_t symbol_count = 99U;

    timing.tolerance_ticks = 5U;
    assert(foc_time_sync_wire_timing_valid(&timing) == 0U);
    assert(foc_time_sync_wire_decoder_init(&decoder, &timing) == 0U);
    timing = make_timing();
    identity.edge_tag = 0U;
    assert(foc_time_sync_wire_encode(&identity,
                                     &timing,
                                     symbols,
                                     FOC_TIME_SYNC_WIRE_SYMBOL_COUNT,
                                     &symbol_count) == 0U);
    assert(symbol_count == 0U);

    timing = make_timing();
    timing.sof_high_ticks = 1U;
    assert(foc_time_sync_wire_timing_valid(&timing) == 0U);
}

static void test_bad_sof_and_symbol_width_fail_closed(void)
{
    foc_time_sync_wire_symbol_t symbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT];
    foc_time_sync_wire_decoder_t decoder;
    foc_time_sync_wire_status_t status;
    foc_time_sync_wire_timing_t timing = make_timing();
    uint32_t symbol_count;

    encode_frame(symbols, &symbol_count, 2U);
    assert(foc_time_sync_wire_decoder_init(&decoder, &timing) == 1U);
    symbols[0].high_ticks = 5U;
    assert(foc_time_sync_wire_decoder_feed(&decoder, &symbols[0]) == 0U);
    foc_time_sync_wire_decoder_get_status(&decoder, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_SOF);

    encode_frame(symbols, &symbol_count, 2U);
    foc_time_sync_wire_decoder_reset(&decoder);
    assert(foc_time_sync_wire_decoder_feed(&decoder, &symbols[0]) == 1U);
    symbols[1].high_ticks = 15U;
    assert(foc_time_sync_wire_decoder_feed(&decoder, &symbols[1]) == 0U);
    foc_time_sync_wire_decoder_get_status(&decoder, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_SYMBOL);
}

static void test_dropped_symbol_is_truncated(void)
{
    foc_time_sync_wire_symbol_t symbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT];
    foc_time_sync_wire_decoder_t decoder;
    foc_time_sync_wire_status_t status;
    foc_time_sync_wire_timing_t timing = make_timing();
    uint32_t symbol_count;
    uint32_t index;

    encode_frame(symbols, &symbol_count, 3U);
    assert(foc_time_sync_wire_decoder_init(&decoder, &timing) == 1U);
    for (index = 0U; index < symbol_count; ++index)
    {
        if (index != 17U)
        {
            assert(foc_time_sync_wire_decoder_feed(&decoder,
                                                   &symbols[index]) == 1U);
        }
    }
    assert(foc_time_sync_wire_decoder_finish(&decoder) == 0U);
    foc_time_sync_wire_decoder_get_status(&decoder, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_TRUNCATED);
}

static void test_inserted_symbol_is_rejected(void)
{
    foc_time_sync_wire_symbol_t symbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT];
    foc_time_sync_wire_decoder_t decoder;
    foc_time_sync_wire_status_t status;
    foc_time_sync_wire_timing_t timing = make_timing();
    uint32_t symbol_count;
    uint32_t index;

    encode_frame(symbols, &symbol_count, 4U);
    assert(foc_time_sync_wire_decoder_init(&decoder, &timing) == 1U);
    for (index = 0U; index < symbol_count; ++index)
    {
        assert(foc_time_sync_wire_decoder_feed(&decoder,
                                               &symbols[index]) == 1U);
    }
    assert(foc_time_sync_wire_decoder_feed(&decoder, &symbols[1]) == 0U);
    foc_time_sync_wire_decoder_get_status(&decoder, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_TOO_MANY_SYMBOLS);
    assert(foc_time_sync_wire_decoder_get_identity(&decoder, NULL) == 0U);
}

static void test_payload_corruption_is_caught_by_crc(void)
{
    foc_time_sync_wire_symbol_t symbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT];
    foc_time_sync_wire_decoder_t decoder;
    foc_time_sync_wire_status_t status;
    foc_time_sync_wire_timing_t timing = make_timing();
    uint32_t symbol_count;
    uint32_t index;
    const uint32_t payload_bit_symbol = 1U + (5U * 8U);

    encode_frame(symbols, &symbol_count, 5U);
    symbols[payload_bit_symbol].high_ticks =
        (symbols[payload_bit_symbol].high_ticks == timing.short_high_ticks) ?
            timing.long_high_ticks :
            timing.short_high_ticks;
    assert(foc_time_sync_wire_decoder_init(&decoder, &timing) == 1U);
    for (index = 0U; index < symbol_count; ++index)
    {
        const uint32_t result = foc_time_sync_wire_decoder_feed(
            &decoder, &symbols[index]);
        if ((index + 1U) < symbol_count)
        {
            assert(result == 1U);
        }
        else
        {
            assert(result == 0U);
        }
    }
    foc_time_sync_wire_decoder_get_status(&decoder, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_CRC);
}

static void test_header_corruption_is_classified(void)
{
    foc_time_sync_wire_symbol_t symbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT];
    foc_time_sync_wire_decoder_t decoder;
    foc_time_sync_wire_status_t status;
    foc_time_sync_wire_timing_t timing = make_timing();
    uint32_t symbol_count;
    uint32_t index;

    encode_frame(symbols, &symbol_count, 6U);
    symbols[1].high_ticks =
        (symbols[1].high_ticks == timing.short_high_ticks) ?
            timing.long_high_ticks :
            timing.short_high_ticks;
    assert(foc_time_sync_wire_decoder_init(&decoder, &timing) == 1U);
    for (index = 0U; index < symbol_count; ++index)
    {
        (void)foc_time_sync_wire_decoder_feed(&decoder, &symbols[index]);
    }
    foc_time_sync_wire_decoder_get_status(&decoder, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_WIRE_FAILURE_BAD_MAGIC);
}

int main(void)
{
    test_round_trip_and_sequence_wrap();
    test_wire_bytes_match_golden_vector();
    test_bad_timing_and_identity_are_rejected();
    test_bad_sof_and_symbol_width_fail_closed();
    test_dropped_symbol_is_truncated();
    test_inserted_symbol_is_rejected();
    test_payload_corruption_is_caught_by_crc();
    test_header_corruption_is_classified();
    return 0;
}
