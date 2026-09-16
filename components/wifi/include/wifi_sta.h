#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RSSI value reported while the station is not associated.
 *
 * The driver returns 0 dBm for "no reading", which a dashboard would happily
 * plot as an unusually strong signal. Consumers translate this sentinel rather
 * than forwarding it -- see cloud.c.
 */
#define WIFI_RSSI_INVALID   0

/**
 * @brief Bring up NVS, the network interface, and the station driver.
 *
 * Connects to the network configured under "WiFi station" in menuconfig and
 * keeps trying: a dropped link is retried with exponential backoff, so a router
 * reboot heals itself without any help from the application.
 *
 * Safe to call once, before any other function here.
 */
esp_err_t wifi_sta_init(void);

/** @brief True once the station has associated and DHCP has produced an IP. */
bool wifi_sta_is_online(void);

/**
 * @brief Block until the station is online.
 *
 * @param timeout_ms  How long to wait; 0 polls without blocking.
 * @return true if online, false if the wait expired first.
 */
bool wifi_sta_wait_online(uint32_t timeout_ms);

/**
 * @brief Current signal strength in dBm, or WIFI_RSSI_INVALID when offline.
 */
int wifi_sta_rssi(void);

/**
 * @brief Device identity: the station MAC as 12 lowercase hex characters.
 *
 * Stable across reboots and unique per board, which is what makes it usable as
 * the serial number in the telemetry URL. Valid after wifi_sta_init(); returns
 * a NUL-terminated string owned by this component.
 */
const char *wifi_sta_serial(void);

/** @brief SSID the station is currently targeting: stored if any, else the Kconfig default. */
const char *wifi_sta_ssid(void);

/**
 * @brief Store new credentials in NVS and reconnect with them.
 *
 * Survives a reboot and takes precedence over the Kconfig values from then on.
 * Returns without associating: the ordinary retry path applies the new config,
 * so a wrong passphrase surfaces as the usual disconnect reason rather than as
 * an error here.
 *
 * @param ssid      1..32 characters. Required.
 * @param password  WPA2 passphrase, or NULL/"" for an open network.
 */
esp_err_t wifi_sta_set_credentials(const char *ssid, const char *password);

/** @brief Re-apply the current credentials and restart the association attempt. */
esp_err_t wifi_sta_reconnect(void);

/**
 * @brief Block until the link is up, or until @p timeout_ms says it is not.
 *
 * Turns a fire-and-forget wifi_sta_init() into something with a verdict: on
 * failure it names the last disconnect reason, which separates "the SSID was
 * never on the air" from "the passphrase was rejected". Retrying continues in
 * the background either way -- this reports, it does not give up.
 *
 * Call it after the UI tasks exist, not between init and them: it blocks its
 * caller for the whole timeout.
 *
 * @return ESP_OK if online, ESP_ERR_TIMEOUT if the wait expired.
 */
esp_err_t wifi_sta_verify_link(uint32_t timeout_ms);

/**
 * @brief Log one line describing the current link state.
 *
 * Offline it names the target SSID and how long the backoff has grown to;
 * online it reports the AP, channel, RSSI, authmode and the leased IP. Meant
 * to be called from whatever is about to depend on the link, so a failure has
 * the link's state sitting next to it in the log rather than several seconds
 * earlier.
 */
void wifi_sta_log_status(void);

#ifdef __cplusplus
}
#endif
