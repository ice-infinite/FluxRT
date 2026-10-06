#include "foc_as5600_stm32g431.h"

#include <stddef.h>
#include <string.h>

#include <drv_common.h>

#include "foc_as5600.h"
#include "foc_build_profile.h"
#include "foc_feedback_adapter.h"
#include "stm32g4xx_hal.h"

#if defined(FLUXRT_AS5600_TRUTH_BUILD)

/* IHM16M1 leaves PC11 free. PC8 also reaches the shield's H3 network through
 * R87; PB10 reaches the same network through R84. This diagnostic therefore
 * keeps PB10 as a high-impedance input and owns PC8/PC11 only while active. */
#define FOC_AS5600_I2C_TIMEOUT_MS       (20U)
#define FOC_AS5600_I2C_TRIALS           (2U)
#define FOC_AS5600_I2C_7BIT_ADDRESS     (FOC_AS5600_I2C_ADDRESS << 1U)

/* PCLK1=170 MHz. PRESC=15, SCLDEL=4, SDADEL=2, SCLH=79, SCLL=99 gives a
 * deliberately conservative standard-mode bus (about 55 kHz after filter and
 * edge delays), which is preferable for first wiring proof. */
#define FOC_AS5600_I2C3_TIMING          (0xF0424F63UL)

static I2C_HandleTypeDef g_as5600_i2c3;
static foc_as5600_tracker_t g_as5600_tracker;
static foc_as5600_stm32g431_diagnostics_t g_as5600_diagnostics;

static uint32_t foc_as5600_saturating_increment(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : (value + 1U);
}

static void foc_as5600_update_pin_state(void)
{
    g_as5600_diagnostics.scl_high =
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET) ? 1U : 0U;
    g_as5600_diagnostics.sda_high =
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_SET) ? 1U : 0U;
}

static void foc_as5600_recover_bus(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t pulse;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_11, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    rt_hw_us_delay(10U);

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_RESET)
    {
        for (pulse = 0U; pulse < 9U; ++pulse)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
            rt_hw_us_delay(10U);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
            rt_hw_us_delay(10U);
        }
    }
    /* STOP: SDA low while SCL high, then release SDA. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    rt_hw_us_delay(10U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    rt_hw_us_delay(10U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    rt_hw_us_delay(10U);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c)
{
    GPIO_InitTypeDef gpio = {0};
    if ((i2c == NULL) || (i2c->Instance != I2C3))
    {
        return;
    }
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF8_I2C3;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2c)
{
    if ((i2c == NULL) || (i2c->Instance != I2C3))
    {
        return;
    }
    __HAL_RCC_I2C3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_11);
}

bool foc_as5600_stm32g431_init(void)
{
    HAL_StatusTypeDef status;
    foc_as5600_config_t tracker_config = {0};

    if (g_as5600_diagnostics.initialized != 0U)
    {
        return true;
    }
    (void)memset(&g_as5600_diagnostics, 0, sizeof(g_as5600_diagnostics));
    (void)memset(&g_as5600_i2c3, 0, sizeof(g_as5600_i2c3));
    foc_as5600_recover_bus();
    __HAL_RCC_I2C3_CONFIG(RCC_I2C3CLKSOURCE_PCLK1);

    g_as5600_i2c3.Instance = I2C3;
    g_as5600_i2c3.Init.Timing = FOC_AS5600_I2C3_TIMING;
    g_as5600_i2c3.Init.OwnAddress1 = 0U;
    g_as5600_i2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_as5600_i2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_as5600_i2c3.Init.OwnAddress2 = 0U;
    g_as5600_i2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    g_as5600_i2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_as5600_i2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    status = HAL_I2C_Init(&g_as5600_i2c3);
    g_as5600_diagnostics.last_hal_status = (uint32_t)status;
    if (status != HAL_OK)
    {
        g_as5600_diagnostics.error_count = 1U;
        g_as5600_diagnostics.last_hal_error = HAL_I2C_GetError(&g_as5600_i2c3);
        g_as5600_diagnostics.last_i2c_isr = I2C3->ISR;
        foc_as5600_update_pin_state();
        return false;
    }

    tracker_config.axis_id = 0U;
    tracker_config.direction = 1;
    tracker_config.pole_pairs = 7U;
    tracker_config.pole_pair_revision = 1U;
    tracker_config.electrical_offset_rad = 0.0F;
    tracker_config.calibrated = 0U;
    tracker_config.maximum_sample_period_us = 100000U;
    if (!foc_as5600_tracker_init(&g_as5600_tracker, &tracker_config))
    {
        g_as5600_diagnostics.error_count = 1U;
        return false;
    }

    g_as5600_diagnostics.initialized = 1U;
    status = HAL_I2C_IsDeviceReady(&g_as5600_i2c3,
                                   FOC_AS5600_I2C_7BIT_ADDRESS,
                                   FOC_AS5600_I2C_TRIALS,
                                   FOC_AS5600_I2C_TIMEOUT_MS);
    g_as5600_diagnostics.sensor_present = (status == HAL_OK) ? 1U : 0U;
    g_as5600_diagnostics.last_hal_status = (uint32_t)status;
    g_as5600_diagnostics.last_hal_error = HAL_I2C_GetError(&g_as5600_i2c3);
    g_as5600_diagnostics.last_i2c_isr = I2C3->ISR;
    foc_as5600_update_pin_state();
    return true;
}

bool foc_as5600_stm32g431_read_raw(uint16_t *raw_count)
{
    uint8_t bytes[2] = {0U, 0U};
    HAL_StatusTypeDef status;
    uint16_t decoded = 0U;
    if ((raw_count == NULL) || !foc_as5600_stm32g431_init())
    {
        return false;
    }
    status = HAL_I2C_Mem_Read(&g_as5600_i2c3,
                              FOC_AS5600_I2C_7BIT_ADDRESS,
                              FOC_AS5600_RAW_ANGLE_REGISTER,
                              I2C_MEMADD_SIZE_8BIT,
                              bytes,
                              sizeof(bytes),
                              FOC_AS5600_I2C_TIMEOUT_MS);
    g_as5600_diagnostics.last_hal_status = (uint32_t)status;
    g_as5600_diagnostics.last_hal_error = HAL_I2C_GetError(&g_as5600_i2c3);
    g_as5600_diagnostics.last_i2c_isr = I2C3->ISR;
    foc_as5600_update_pin_state();
    if ((status != HAL_OK) ||
        !foc_as5600_decode_raw_angle(bytes[0], bytes[1], &decoded))
    {
        g_as5600_diagnostics.sensor_present = 0U;
        g_as5600_diagnostics.error_count = foc_as5600_saturating_increment(
            g_as5600_diagnostics.error_count);
        return false;
    }
    g_as5600_diagnostics.sensor_present = 1U;
    g_as5600_diagnostics.read_count = foc_as5600_saturating_increment(
        g_as5600_diagnostics.read_count);
    g_as5600_diagnostics.last_raw_count = decoded;
    *raw_count = decoded;
    return true;
}

bool foc_as5600_stm32g431_poll(
    uint32_t sampled_at_ms,
    uint32_t sampled_at_us,
    foc_feedback_source_sample_t *output)
{
    uint16_t raw_count = 0U;
    foc_absolute_encoder_feedback_port_t encoder = {0};
    if (output == NULL)
    {
        return false;
    }
    (void)memset(output, 0, sizeof(*output));
    if (!foc_as5600_stm32g431_read_raw(&raw_count) ||
        !foc_as5600_tracker_update(&g_as5600_tracker,
                                   raw_count,
                                   sampled_at_ms,
                                   sampled_at_us,
                                   &encoder) ||
        !foc_feedback_adapt_absolute_encoder(&encoder, output))
    {
        (void)memset(output, 0, sizeof(*output));
        return false;
    }
    return true;
}

void foc_as5600_stm32g431_get_diagnostics(
    foc_as5600_stm32g431_diagnostics_t *output)
{
    if (output != NULL)
    {
        foc_as5600_update_pin_state();
        *output = g_as5600_diagnostics;
    }
}

#endif /* FLUXRT_AS5600_TRUTH_BUILD */
