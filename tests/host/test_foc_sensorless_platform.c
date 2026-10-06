#include <assert.h>

#include "foc_sensorless_platform.h"

int main(void)
{
    assert(foc_sensorless_platform_capabilities() == 0U);
    return 0;
}
