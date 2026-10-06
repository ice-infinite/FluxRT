#include "foc_sensorless_platform.h"

uint32_t foc_sensorless_platform_capabilities(void)
{
    /* P5.5 fail-closed baseline: having an ABI and a simulated control chain is
     * not board evidence. Each bit must be raised only after its target gate is
     * measured and recorded; until then an enabled configuration transaction is
     * rejected by foc_rust_sensorless_configure(). */
    return 0U;
}
