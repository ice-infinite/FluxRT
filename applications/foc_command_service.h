#ifndef FOC_COMMAND_SERVICE_H
#define FOC_COMMAND_SERVICE_H

/*
 * Sole device-management entry for adapter-produced ProductCommand values.
 *
 * Protocol/input adapters may call this service, never MotorService, FOC or
 * PWM directly.  The bound sink is the future Rust CommandArbiter bridge.  A
 * sink result of zero means accepted; every non-zero result is fail-closed and
 * preserved as diagnostic detail.  All APIs are one management-context only.
 */

#include <stdint.h>

#include "foc_config_bridge.h"
#include "foc_external_io.h"

#define FOC_COMMAND_SERVICE_VERSION (1UL)

typedef uint32_t foc_command_service_result_t;
enum
{
    FOC_COMMAND_SERVICE_OK = 0,
    FOC_COMMAND_SERVICE_DISABLED = 1,
    FOC_COMMAND_SERVICE_INVALID_ARGUMENT = 2,
    FOC_COMMAND_SERVICE_UNSAFE_STATE = 3,
    FOC_COMMAND_SERVICE_INVALID_CONFIG = 4,
    FOC_COMMAND_SERVICE_UNKNOWN_SOURCE = 5,
    FOC_COMMAND_SERVICE_UNAUTHORIZED = 6,
    FOC_COMMAND_SERVICE_EXPIRED = 7,
    FOC_COMMAND_SERVICE_SINK_REJECTED = 8,
};

typedef struct
{
    uint32_t source_id;
    uint32_t priority;
    uint32_t permissions;
    uint32_t lease_ms;
    uint32_t command_timeout_ms;
} foc_command_service_source_t;

typedef uint32_t (*foc_command_service_configure_fn)(
    void *context,
    const foc_command_service_source_t *sources,
    uint32_t source_count);
typedef uint32_t (*foc_command_service_submit_fn)(
    void *context,
    const foc_product_command_t *command,
    uint32_t now_ms);

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_command_service_configure_fn configure;
    foc_command_service_submit_fn submit;
    void *context;
} foc_command_service_ops_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t enabled;
    uint32_t source_count;
    uint32_t submitted;
    uint32_t accepted;
    uint32_t rejected;
    uint32_t unknown_source;
    uint32_t unauthorized;
    uint32_t expired;
    uint32_t sink_rejected;
    uint32_t last_source_id;
    foc_command_service_result_t last_result;
    uint32_t last_detail;
} foc_command_service_status_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t enabled;
    foc_external_io_capabilities_t capabilities;
    foc_command_service_ops_t ops;
    uint32_t source_count;
    foc_command_service_source_t sources[FOC_EXTERNAL_IO_MAX_COMMAND_SOURCES];
    uint32_t submitted;
    uint32_t accepted;
    uint32_t rejected;
    uint32_t unknown_source;
    uint32_t unauthorized;
    uint32_t expired;
    uint32_t sink_rejected;
    uint32_t last_source_id;
    foc_command_service_result_t last_result;
    uint32_t last_detail;
} foc_command_service_t;

foc_command_service_result_t foc_command_service_init(
    foc_command_service_t *service,
    const foc_external_io_capabilities_t *capabilities,
    const foc_command_service_ops_t *ops);

foc_command_service_result_t foc_command_service_apply_config(
    foc_command_service_t *service,
    const foc_external_io_config_t *config,
    const foc_config_apply_guard_t *guard,
    foc_external_io_validation_t *validation);

/* Disable is unconditional and removes all configured sources. */
foc_command_service_result_t foc_command_service_disable(
    foc_command_service_t *service);

foc_command_service_result_t foc_command_service_submit(
    foc_command_service_t *service,
    const foc_product_command_t *command,
    uint32_t now_ms);

foc_command_service_result_t foc_command_service_get_status(
    const foc_command_service_t *service,
    foc_command_service_status_t *status);

#endif /* FOC_COMMAND_SERVICE_H */
