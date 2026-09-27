/* Host proof that non-runtime profiles fail closed before platform setup. */

#include "foc_platform.h"

int main(void)
{
#if !defined(FLUXRT_MOTOR_ARM_DISABLED_BUILD)
#error "This test must be compiled with a motor-arm-disabled profile"
#endif
    foc_platform_emergency_stop();
    if (foc_platform_control_start(582.0f) != FOC_STATUS_DISABLED)
    {
        return 1;
    }
    if (foc_platform_trial_arm() != FOC_STATUS_DISABLED)
    {
        return 2;
    }
    if (foc_platform_lsi_start(0x4C534931UL) !=
#if defined(FLUXRT_IDENTIFICATION_BUILD)
        FOC_STATUS_NOT_CONFIGURED)
#else
        FOC_STATUS_DISABLED)
#endif
    {
        return 3;
    }
    foc_platform_control_stop();
    return 0;
}
