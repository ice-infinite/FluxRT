#include "foc_external_io_management.h"

/*
 * This target intentionally receives only applications/ and foc/include/ as
 * include roots.  It fails to compile if the common management header starts
 * depending on a platform-private header again.
 */
int main(void)
{
    return (sizeof(foc_external_io_management_t) > 0U) ? 0 : 1;
}
