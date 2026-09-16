#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start posting measurements on the configured interval.
 *
 * Spawns one low-priority task that waits for the station to come online, then
 * posts the current sensor snapshot every CONFIG_CLOUD_POST_INTERVAL_S seconds.
 * Requires wifi_sta_init() and air_monitor_start() to have run.
 *
 * Idempotent: a second call is a no-op.
 */
esp_err_t cloud_start(void);

/**
 * @brief Gate transmission without tearing the task down.
 *
 * A post already in flight still completes. Disabling is the right way to
 * silence telemetry from a menu, because re-enabling costs nothing -- unlike a
 * stop/start round trip, which churns the heap the TLS handshake needs.
 */
void cloud_set_enabled(bool enabled);

/** @brief True when transmission is enabled (the default). */
bool cloud_is_enabled(void);

/**
 * @brief Post at the next opportunity instead of waiting out the interval.
 *
 * Does not reset the cadence: the interval after the forced post is measured
 * from when that post started, exactly as a periodic one would be.
 */
void cloud_post_now(void);

#ifdef __cplusplus
}
#endif
