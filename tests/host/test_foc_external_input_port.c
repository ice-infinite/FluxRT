#include "foc_external_input_port.h"

#include <assert.h>
#include <string.h>

typedef struct
{
    foc_external_input_mask_t enabled;
    foc_external_input_raw_sample_t sample;
    foc_external_input_raw_health_t health;
} fake_input_t;

static foc_external_input_port_result_t fake_start(
    void *context,
    foc_external_input_mask_t input_mask)
{
    fake_input_t *fake = (fake_input_t *)context;
    fake->enabled |= input_mask;
    fake->health.enabled_input_mask = fake->enabled;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static foc_external_input_port_result_t fake_stop(
    void *context,
    foc_external_input_mask_t input_mask)
{
    fake_input_t *fake = (fake_input_t *)context;
    fake->enabled &= ~input_mask;
    fake->health.enabled_input_mask = fake->enabled;
    fake->health.healthy_input_mask &= fake->enabled;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static foc_external_input_port_result_t fake_read(
    void *context,
    foc_external_input_mask_t input,
    foc_external_input_raw_sample_t *sample)
{
    fake_input_t *fake = (fake_input_t *)context;
    if ((fake->enabled & input) == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_DISABLED;
    }
    *sample = fake->sample;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static foc_external_input_port_result_t fake_health(
    void *context,
    foc_external_input_raw_health_t *health)
{
    const fake_input_t *fake = (const fake_input_t *)context;
    *health = fake->health;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

int main(void)
{
    fake_input_t fake;
    foc_external_input_port_t port;
    foc_external_input_raw_sample_t sample;
    foc_external_input_raw_health_t health;
    const foc_external_input_port_ops_t ops =
    {
        sizeof(foc_external_input_port_ops_t),
        FOC_EXTERNAL_INPUT_PORT_OPS_VERSION,
        fake_start,
        fake_stop,
        fake_read,
        fake_health,
    };

    (void)memset(&fake, 0, sizeof(fake));
    fake.sample.struct_size = sizeof(fake.sample);
    fake.sample.version = FOC_EXTERNAL_INPUT_RAW_SAMPLE_VERSION;
    fake.sample.input = FOC_EXTERNAL_INPUT_ANALOG;
    fake.sample.sequence = 7U;
    fake.sample.sampled_at_us = 1234U;
    fake.sample.valid_flags = FOC_EXTERNAL_INPUT_RAW_VALID_VALUE |
                              FOC_EXTERNAL_INPUT_RAW_VALID_TIMESTAMP;
    fake.sample.raw_value = 2048;
    fake.health.struct_size = sizeof(fake.health);
    fake.health.version = FOC_EXTERNAL_INPUT_RAW_HEALTH_VERSION;
    fake.health.supported_input_mask = FOC_EXTERNAL_INPUT_ANALOG;

    assert(sizeof(foc_external_input_raw_sample_t) == 40U);
    assert(sizeof(foc_external_input_raw_health_t) == 40U);
    assert(foc_external_input_port_bind(
               &port, FOC_EXTERNAL_INPUT_ANALOG, &fake, &ops) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(port.enabled_input_mask == 0U);
    assert(foc_external_input_port_start(
               &port, FOC_EXTERNAL_INPUT_PWM_PULSE) ==
           FOC_EXTERNAL_INPUT_PORT_NOT_AVAILABLE);
    assert(foc_external_input_port_start(
               &port, FOC_EXTERNAL_INPUT_ANALOG) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(foc_external_input_port_read_latest(
               &port, FOC_EXTERNAL_INPUT_ANALOG, &sample) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(sample.sequence == 7U);
    assert(sample.sampled_at_us == 1234U);
    assert(sample.raw_value == 2048);
    assert(foc_external_input_port_get_health(&port, &health) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(health.enabled_input_mask == FOC_EXTERNAL_INPUT_ANALOG);
    assert(foc_external_input_port_stop(
               &port, FOC_EXTERNAL_INPUT_ANALOG) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(port.enabled_input_mask == 0U);
    assert(foc_external_input_port_read_latest(
               &port, FOC_EXTERNAL_INPUT_ANALOG, &sample) ==
           FOC_EXTERNAL_INPUT_PORT_DISABLED);
    assert(sample.struct_size == 0U);
    return 0;
}
