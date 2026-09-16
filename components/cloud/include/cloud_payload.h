#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "air_monitor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Largest body cloud_payload_build() can produce.
 *
 * Every field this board can measure, at full precision, fits comfortably; the
 * buffer is sized so the serializer never has to think about it.
 */
#define CLOUD_PAYLOAD_MAX   512

/**
 * @brief Serialize a device snapshot into the telemetry JSON body.
 *
 * Only fields that carry a real reading are emitted. A sensor that failed, is
 * still warming up, or is not fitted leaves its property out of the object
 * entirely rather than sending a zero the backend cannot distinguish from a
 * genuine measurement -- so a snapshot taken before the first acquisition cycle
 * still produces a valid body carrying just the two metadata properties.
 *
 * @param out           Destination buffer, NUL-terminated on success.
 * @param out_size      Capacity of @p out; CLOUD_PAYLOAD_MAX is always enough.
 * @param dev           Snapshot to serialize.
 * @param rssi          Signal strength in dBm, already translated for offline.
 * @param boot_minutes  Whole minutes since power-on.
 *
 * @return Bytes written excluding the terminator, or 0 on failure.
 */
size_t cloud_payload_build(char *out, size_t out_size, const air_monitor_device *dev,
                           int rssi, uint32_t boot_minutes);

#ifdef __cplusplus
}
#endif
