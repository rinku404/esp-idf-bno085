#ifndef BNO085_SH2_MULTI_H
#define BNO085_SH2_MULTI_H

#include "sh2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Per-slot dispatch table into one compiled-and-renamed copy of the
 * vendored sh2.c/shtp.c pair. sh2_decodeSensorEvent() is intentionally
 * NOT here: sh2_SensorValue.c has no mutable state and is compiled once,
 * shared by all slots -- call it directly, unrenamed.
 */
typedef struct sh2_vtable_s {
    int  (*open)(sh2_Hal_t *pHal, sh2_EventCallback_t *eventCallback, void *eventCookie);
    void (*close)(void);
    void (*service)(void);
    int  (*setSensorCallback)(sh2_SensorCallback_t *callback, void *cookie);
    int  (*setSensorConfig)(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);
} sh2_vtable_t;

typedef struct sh2_instance_slot_s {
    int                 index;   /* 0 .. CONFIG_BNO085_MAX_INSTANCES-1 */
    const sh2_vtable_t *vtable;
} sh2_instance_slot_t;

/**
 * Acquire a free BNO085 instance slot. Returns NULL if all are in use.
 * Called once per bno085_init(). Not thread-safe; intended for single-task
 * startup initialization.
 */
const sh2_instance_slot_t *sh2_multi_acquire_slot(void);

/**
 * Release a previously-acquired slot, allowing it to be reused.
 * Called once per bno085_deinit().
 */
void sh2_multi_release_slot(const sh2_instance_slot_t *slot);

#ifdef __cplusplus
}
#endif

#endif /* BNO085_SH2_MULTI_H */
