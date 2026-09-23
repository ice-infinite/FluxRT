#ifndef FOC_MATH_ACCEL_H
#define FOC_MATH_ACCEL_H

#include <stdint.h>

typedef enum
{
    FOC_MATH_BACKEND_CPU = 0,
    FOC_MATH_BACKEND_CORDIC = 1
} foc_math_backend_t;

/*
 * Optional platform math accelerator. A zero calculation return value is not
 * a control fault: the Rust bridge immediately performs the same operation on
 * CpuMath. This keeps the control algorithm portable to MCUs without CORDIC.
 */
foc_math_backend_t foc_math_accel_init(void);
foc_math_backend_t foc_math_accel_backend(void);
uint32_t foc_math_accel_sin_cos(float angle_rad, float *sin_out, float *cos_out);
uint32_t foc_math_accel_magnitude(float x, float y, float *magnitude_out);
uint32_t foc_math_accel_atan2(float y, float x, float *angle_out);

#endif
