#include "foc_math_accel.h"

#if defined(FOC_TARGET_STM32G431)
#include "rtconfig.h"
#endif

#if defined(FOC_MATH_BACKEND_STM32G4_CORDIC) && defined(STM32G431xx)

#include <float.h>
#include <limits.h>

#include "stm32g4xx.h"
#include "stm32g4xx_ll_cordic.h"

#define FOC_PI_F                    (3.14159265358979323846f)
#define FOC_TWO_PI_F                (2.0f * FOC_PI_F)
#define FOC_CORDIC_TIMEOUT_LOOPS    (64U)
#define FOC_Q31_SCALE_F             (2147483648.0f)
#define FOC_Q15_SCALE_F             (32768.0f)

#define FOC_CORDIC_CONFIG_COSSIN                                               \
    (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_6CYCLES |                \
     LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_2 |           \
     LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

#define FOC_CORDIC_CONFIG_MODULUS                                              \
    (LL_CORDIC_FUNCTION_MODULUS | LL_CORDIC_PRECISION_6CYCLES |               \
     LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |           \
     LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)

#define FOC_CORDIC_CONFIG_PHASE                                                \
    (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_6CYCLES |                 \
     LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_1 |           \
     LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

static volatile foc_math_backend_t g_foc_math_backend = FOC_MATH_BACKEND_CPU;

static uint32_t foc_cordic_wait_ready(void)
{
    uint32_t remaining = FOC_CORDIC_TIMEOUT_LOOPS;
    while (((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) && (remaining > 0U))
    {
        --remaining;
    }
    return (remaining > 0U) ? 1U : 0U;
}

static void foc_restore_interrupts(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static int32_t foc_float_to_q31(float value)
{
    if (value >= 1.0f)
    {
        return INT32_MAX;
    }
    if (value <= -1.0f)
    {
        return INT32_MIN;
    }
    return (int32_t)(value * FOC_Q31_SCALE_F);
}

static int16_t foc_float_to_q15(float value)
{
    if (value >= 0.999969482421875f)
    {
        return INT16_MAX;
    }
    if (value <= -1.0f)
    {
        return INT16_MIN;
    }
    return (int16_t)(value * FOC_Q15_SCALE_F);
}

foc_math_backend_t foc_math_accel_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    (void)RCC->AHB1ENR;
    __DSB();
    if ((RCC->AHB1ENR & RCC_AHB1ENR_CORDICEN) != 0U)
    {
        g_foc_math_backend = FOC_MATH_BACKEND_CORDIC;
    }
    return g_foc_math_backend;
}

foc_math_backend_t foc_math_accel_backend(void)
{
    return g_foc_math_backend;
}

uint32_t foc_math_accel_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    int32_t cos_q31;
    int32_t sin_q31;
    uint32_t primask;

    if ((sin_out == 0) || (cos_out == 0))
    {
        return 0U;
    }
    *sin_out = 0.0f;
    *cos_out = 0.0f;
    if ((g_foc_math_backend != FOC_MATH_BACKEND_CORDIC) ||
        !(angle_rad <= FLT_MAX && angle_rad >= -FLT_MAX) ||
        (angle_rad > (8.0f * FOC_PI_F)) || (angle_rad < (-8.0f * FOC_PI_F)))
    {
        return 0U;
    }

    while (angle_rad >= FOC_PI_F)
    {
        angle_rad -= FOC_TWO_PI_F;
    }
    while (angle_rad < -FOC_PI_F)
    {
        angle_rad += FOC_TWO_PI_F;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    CORDIC->CSR = FOC_CORDIC_CONFIG_COSSIN;
    CORDIC->WDATA = (uint32_t)foc_float_to_q31(angle_rad / FOC_PI_F);
    CORDIC->WDATA = (uint32_t)INT32_MAX;
    if (foc_cordic_wait_ready() == 0U)
    {
        foc_restore_interrupts(primask);
        return 0U;
    }
    cos_q31 = (int32_t)CORDIC->RDATA;
    sin_q31 = (int32_t)CORDIC->RDATA;
    foc_restore_interrupts(primask);

    *sin_out = (float)sin_q31 / FOC_Q31_SCALE_F;
    *cos_out = (float)cos_q31 / FOC_Q31_SCALE_F;
    return 1U;
}

uint32_t foc_math_accel_magnitude(float x, float y, float *magnitude_out)
{
    float abs_x;
    float abs_y;
    float scale;
    int16_t x_q15;
    int16_t y_q15;
    uint32_t packed_input;
    uint32_t packed_output;
    uint32_t primask;

    if (magnitude_out == 0)
    {
        return 0U;
    }
    *magnitude_out = 0.0f;
    if ((g_foc_math_backend != FOC_MATH_BACKEND_CORDIC) ||
        !(x <= FLT_MAX && x >= -FLT_MAX) || !(y <= FLT_MAX && y >= -FLT_MAX))
    {
        return 0U;
    }

    abs_x = (x < 0.0f) ? -x : x;
    abs_y = (y < 0.0f) ? -y : y;
    scale = ((abs_x > abs_y) ? abs_x : abs_y) * 2.0f;
    if (scale == 0.0f)
    {
        return 1U;
    }
    if (!(scale <= FLT_MAX))
    {
        return 0U;
    }

    x_q15 = foc_float_to_q15(x / scale);
    y_q15 = foc_float_to_q15(y / scale);
    packed_input = ((uint32_t)(uint16_t)y_q15 << 16U) | (uint32_t)(uint16_t)x_q15;

    primask = __get_PRIMASK();
    __disable_irq();
    CORDIC->CSR = FOC_CORDIC_CONFIG_MODULUS;
    CORDIC->WDATA = packed_input;
    if (foc_cordic_wait_ready() == 0U)
    {
        foc_restore_interrupts(primask);
        return 0U;
    }
    packed_output = CORDIC->RDATA;
    foc_restore_interrupts(primask);

    *magnitude_out = ((float)(packed_output & 0xFFFFU) / FOC_Q15_SCALE_F) * scale;
    return 1U;
}

uint32_t foc_math_accel_atan2(float y, float x, float *angle_out)
{
    float abs_x;
    float abs_y;
    float scale;
    int32_t angle_q31;
    uint32_t primask;

    if (angle_out == 0)
    {
        return 0U;
    }
    *angle_out = 0.0f;
    if ((g_foc_math_backend != FOC_MATH_BACKEND_CORDIC) ||
        !(x <= FLT_MAX && x >= -FLT_MAX) || !(y <= FLT_MAX && y >= -FLT_MAX))
    {
        return 0U;
    }
    abs_x = (x < 0.0f) ? -x : x;
    abs_y = (y < 0.0f) ? -y : y;
    scale = (abs_x > abs_y) ? abs_x : abs_y;
    if (scale == 0.0f)
    {
        return 1U;
    }
    scale *= 2.0f;

    primask = __get_PRIMASK();
    __disable_irq();
    CORDIC->CSR = FOC_CORDIC_CONFIG_PHASE;
    CORDIC->WDATA = (uint32_t)foc_float_to_q31(x / scale);
    CORDIC->WDATA = (uint32_t)foc_float_to_q31(y / scale);
    if (foc_cordic_wait_ready() == 0U)
    {
        foc_restore_interrupts(primask);
        return 0U;
    }
    angle_q31 = (int32_t)CORDIC->RDATA;
    foc_restore_interrupts(primask);
    *angle_out = ((float)angle_q31 / FOC_Q31_SCALE_F) * FOC_PI_F;
    if (*angle_out < 0.0f)
    {
        *angle_out += FOC_TWO_PI_F;
    }
    return 1U;
}

#else

foc_math_backend_t foc_math_accel_init(void)
{
    return FOC_MATH_BACKEND_CPU;
}

foc_math_backend_t foc_math_accel_backend(void)
{
    return FOC_MATH_BACKEND_CPU;
}

uint32_t foc_math_accel_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    (void)angle_rad;
    if (sin_out != 0)
    {
        *sin_out = 0.0f;
    }
    if (cos_out != 0)
    {
        *cos_out = 0.0f;
    }
    return 0U;
}

uint32_t foc_math_accel_magnitude(float x, float y, float *magnitude_out)
{
    (void)x;
    (void)y;
    if (magnitude_out != 0)
    {
        *magnitude_out = 0.0f;
    }
    return 0U;
}

uint32_t foc_math_accel_atan2(float y, float x, float *angle_out)
{
    (void)y;
    (void)x;
    if (angle_out != 0)
    {
        *angle_out = 0.0f;
    }
    return 0U;
}

#endif
