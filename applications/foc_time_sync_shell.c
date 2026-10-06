/* Explicit Identification-only H3 transmitter surface.  No command is run at
 * boot: PB6 remains a low GPIO until the operator calls foc_h3_sync_init and a
 * frame is emitted only by foc_h3_sync_send. */

#include "foc_build_profile.h"

#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)

#include <finsh.h>
#include <rtthread.h>

#include "foc_shell_parse.h"
#include "foc_time_sync_stm32g431.h"

static void foc_time_sync_shell_print_status(
    const char *tag,
    uint32_t result)
{
    foc_time_sync_stm32g431_tx_status_t status;

    foc_time_sync_stm32g431_tx_get_status(&status);
    rt_kprintf("%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d\n",
               tag,
               (unsigned int)result,
               (unsigned int)status.version,
               (unsigned int)status.state,
               (unsigned int)status.started_count,
               (unsigned int)status.completed_count,
               (unsigned int)status.failed_count,
               (unsigned int)status.first_edge_count,
               (unsigned int)status.first_edge_timer_tick,
               (unsigned int)status.first_edge_cycle_tick,
               (unsigned int)status.first_edge_control_tick,
               (unsigned int)status.first_edge_flags,
               (unsigned int)status.last_session_id,
               (unsigned int)status.last_edge_sequence,
               (unsigned int)status.last_edge_tag,
               (unsigned int)status.last_hal_status,
               (unsigned int)status.loopback_edge_count,
               (unsigned int)status.loopback_edge_timer_tick,
               (unsigned int)status.loopback_edge_cycle_tick,
               (int)status.loopback_delta_cycles);
}

static int foc_time_sync_shell_init(int argc, char **argv)
{
    uint32_t result;

    (void)argv;
    if (argc != 1)
    {
        rt_kprintf("foc_h3_sync_init\n");
        return -1;
    }
    result = foc_time_sync_stm32g431_tx_init();
    foc_time_sync_shell_print_status("FH3_SYNC_INIT", result);
    return (result != 0U) ? 0 : -1;
}
MSH_CMD_EXPORT_ALIAS(foc_time_sync_shell_init, foc_h3_sync_init, -);

static int foc_time_sync_shell_status(int argc, char **argv)
{
    (void)argv;
    if (argc != 1)
    {
        rt_kprintf("foc_h3_sync_status\n");
        return -1;
    }
    foc_time_sync_shell_print_status("FH3_SYNC_STATUS", 1U);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(foc_time_sync_shell_status, foc_h3_sync_status, -);

static int foc_time_sync_shell_send(int argc, char **argv)
{
    foc_time_sync_edge_identity_t identity = {
        sizeof(foc_time_sync_edge_identity_t),
        FOC_TIME_SYNC_IDENTITY_VERSION,
        0U,
        0U,
        0U,
        FOC_TIME_SYNC_FLAG_RISING_EDGE |
            FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED,
    };
    uint32_t result;

    if ((argc != 4) ||
        (foc_shell_parse_u32(argv[1], &identity.session_id) == 0U) ||
        (foc_shell_parse_u32(argv[2], &identity.edge_sequence) == 0U) ||
        (foc_shell_parse_u32(argv[3], &identity.edge_tag) == 0U) ||
        (identity.session_id == 0U) || (identity.edge_tag == 0U))
    {
        rt_kprintf("foc_h3_sync_send <session> <sequence> <tag>\n");
        return -1;
    }
    result = foc_time_sync_stm32g431_tx_start(&identity);
    foc_time_sync_shell_print_status("FH3_SYNC_SEND", result);
    return (result != 0U) ? 0 : -1;
}
MSH_CMD_EXPORT_ALIAS(foc_time_sync_shell_send, foc_h3_sync_send, -);

static int foc_time_sync_shell_abort(int argc, char **argv)
{
    (void)argv;
    if (argc != 1)
    {
        rt_kprintf("foc_h3_sync_abort\n");
        return -1;
    }
    foc_time_sync_stm32g431_tx_abort();
    foc_time_sync_shell_print_status("FH3_SYNC_ABORT", 1U);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(foc_time_sync_shell_abort, foc_h3_sync_abort, -);

#endif /* FLUXRT_LSI_IDENTIFICATION_BUILD */
