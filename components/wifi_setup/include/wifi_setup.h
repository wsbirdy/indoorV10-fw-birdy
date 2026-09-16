#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Setup portal: an open SoftAP serving a one-page credential form.
 *
 * Offered when the station cannot reach its configured network, so it has to
 * work for someone holding a phone and nothing else -- no app, no serial
 * console, no second device on the same LAN. That rules out the IDF's
 * provisioning manager, whose protocomm transport needs a companion app, and
 * leaves a plain HTTP form on a fixed address.
 *
 * The AP is deliberately open. A WPA2 SoftAP would need a passphrase printed
 * on the device or the panel to be usable by a stranger, and the only secret
 * that crosses the link is the passphrase the user is about to type anyway.
 * The window is small and bounded: see WIFI_SETUP_TIMEOUT_MS.
 */

/** @brief The address the form is served on. Fixed, so it can be printed on the panel. */
#define WIFI_SETUP_PORTAL_URL   "http://192.168.4.1"

/** @brief Portal lifetime with no successful submission, after which it closes itself. */
#define WIFI_SETUP_TIMEOUT_MS   (5 * 60 * 1000)

/**
 * @brief Bring up the SoftAP and the HTTP server.
 *
 * Switches the radio to AP+STA: the station keeps retrying underneath, so a
 * network that comes back on its own still heals without anyone touching the
 * form. Safe to call when already active -- it is a no-op.
 *
 * Requires wifi_sta_init() to have run: the netif stack, the default event
 * loop and NVS all belong to it.
 */
esp_err_t wifi_setup_start(void);

/**
 * @brief Tear the portal down and return the radio to station-only mode.
 *
 * Safe to call when not active. Never call it from an HTTP handler -- stopping
 * the server from inside one of its own worker tasks deadlocks; the portal
 * closes itself on a timer after a successful submission instead.
 */
void wifi_setup_stop(void);

/** @brief True between a successful start and the matching stop. */
bool wifi_setup_is_active(void);

/**
 * @brief SSID the portal is advertising, for printing on the panel.
 *
 * Valid only while active; returns "" otherwise. Owned by this component.
 */
const char *wifi_setup_ap_ssid(void);

/**
 * @brief True once a form submission has been accepted.
 *
 * The portal keeps serving for a few seconds after this so the browser can
 * render the confirmation page, then closes itself.
 */
bool wifi_setup_credentials_received(void);

#ifdef __cplusplus
}
#endif
