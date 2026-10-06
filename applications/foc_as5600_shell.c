#include <rtthread.h>
#include <stdlib.h>

#include "foc_as5600_stm32g431.h"
#include "foc_platform.h"

#define FOC_ENCODER_WATCH_DEFAULT_COUNT (10U)
#define FOC_ENCODER_WATCH_MAX_COUNT     (100U)
#define FOC_ENCODER_WATCH_DEFAULT_MS    (20U)
#define FOC_ENCODER_WATCH_MIN_MS        (5U)
#define FOC_ENCODER_WATCH_MAX_MS        (1000U)

static void foc_encoder_print(uint32_t index)
{
    uint32_t now_ms = (uint32_t)rt_tick_get_millisecond();
    uint16_t raw_count = 0U;
    uint32_t angle_mdeg;
    foc_as5600_stm32g431_diagnostics_t diagnostics = {0};
    bool ok = foc_as5600_stm32g431_read_raw(&raw_count);
    foc_as5600_stm32g431_get_diagnostics(&diagnostics);
    angle_mdeg = ((uint32_t)raw_count * 360000UL) / 4096UL;
    rt_kprintf("FENC,%u,%u,%u,%u,%u,%u,%u,%u,%u,%08x,%u,%u,%u\n",
               (unsigned int)index,
               ok ? 1U : 0U,
               (unsigned int)now_ms,
               (unsigned int)raw_count,
               (unsigned int)angle_mdeg,
               (unsigned int)diagnostics.sensor_present,
               (unsigned int)diagnostics.read_count,
               (unsigned int)diagnostics.error_count,
               (unsigned int)diagnostics.last_hal_status,
               (unsigned int)diagnostics.last_hal_error,
               (unsigned int)diagnostics.last_i2c_isr,
               (unsigned int)diagnostics.scl_high,
               (unsigned int)diagnostics.sda_high);
}

static int foc_encoder_status(int argc, char **argv)
{
    (void)argv;
    if (argc != 1)
    {
        rt_kprintf("foc_encoder_status\n");
        return -1;
    }
    foc_platform_control_stop();
    (void)foc_as5600_stm32g431_init();
    foc_encoder_print(0U);
    return 0;
}
MSH_CMD_EXPORT(foc_encoder_status, read AS5600 once with motor arm compiled out);

static int foc_encoder_watch(int argc, char **argv)
{
    uint32_t count = FOC_ENCODER_WATCH_DEFAULT_COUNT;
    uint32_t period_ms = FOC_ENCODER_WATCH_DEFAULT_MS;
    uint32_t index;
    char *end = RT_NULL;
    unsigned long parsed;

    if ((argc < 1) || (argc > 3))
    {
        rt_kprintf("foc_encoder_watch [count<=100] [period_ms=5..1000]\n");
        return -1;
    }
    if (argc >= 2)
    {
        parsed = strtoul(argv[1], &end, 10);
        if ((end == argv[1]) || (*end != '\0') || (parsed == 0UL) ||
            (parsed > FOC_ENCODER_WATCH_MAX_COUNT))
        {
            rt_kprintf("foc_encoder_watch [count<=100] [period_ms=5..1000]\n");
            return -1;
        }
        count = (uint32_t)parsed;
    }
    if (argc >= 3)
    {
        parsed = strtoul(argv[2], &end, 10);
        if ((end == argv[2]) || (*end != '\0') ||
            (parsed < FOC_ENCODER_WATCH_MIN_MS) ||
            (parsed > FOC_ENCODER_WATCH_MAX_MS))
        {
            rt_kprintf("foc_encoder_watch [count<=100] [period_ms=5..1000]\n");
            return -1;
        }
        period_ms = (uint32_t)parsed;
    }

    foc_platform_control_stop();
    (void)foc_as5600_stm32g431_init();
    for (index = 0U; index < count; ++index)
    {
        foc_encoder_print(index);
        if ((index + 1U) < count)
        {
            rt_thread_mdelay((rt_int32_t)period_ms);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(foc_encoder_watch, sample AS5600 in bounded Shell context);
