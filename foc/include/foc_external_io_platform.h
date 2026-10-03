#ifndef FOC_EXTERNAL_IO_PLATFORM_H
#define FOC_EXTERNAL_IO_PLATFORM_H

/*
 * Build-selected platform provider for external-I/O capabilities.
 *
 * Exactly one target platform implements this symbol.  Application and
 * management code consume only this target-neutral contract; MCU peripheral,
 * pin and resource definitions remain private to foc/platform/<target>/.
 */

#include "foc_external_io.h"

void foc_external_io_platform_capabilities(
    foc_external_io_capabilities_t *capabilities);

#endif /* FOC_EXTERNAL_IO_PLATFORM_H */
