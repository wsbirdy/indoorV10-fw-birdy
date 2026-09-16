#include <stdio.h>
#include <string.h>

#include "wifi_sta.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_sta";

/*
 * Reconnect backoff. The first retry is immediate-ish so a momentary glitch
 * costs a second, and the ceiling keeps a genuinely absent AP from pinning the
 * radio at full duty for hours.
 */
#define RETRY_BASE_MS       1000
#define RETRY_MAX_MS        60000

/*
 * How long an association may sit without a DHCP lease before it is torn down
 * and retried. Associating is not the same as being usable: an AP that accepts
 * us while its DHCP server is wedged leaves the station happily connected and
 * permanently offline, and no driver event is ever emitted to say so.
 */
#define DHCP_TIMEOUT_MS     15000

#define ONLINE_BIT          BIT0

/*
 * Credentials live in NVS once the setup portal has written any, and fall back
 * to the Kconfig values otherwise. That split is what lets a device be flashed
 * with a factory default and still be re-pointed at a different network in the
 * field without a rebuild.
 */
#define NVS_NAMESPACE       "wifi_sta"
#define NVS_KEY_SSID        "ssid"
#define NVS_KEY_PASS        "pass"

static char s_ssid[33];             // 32 + NUL, the 802.11 maximum
static char s_pass[65];             // 64 + NUL
static uint8_t s_last_reason;       // last wifi_err_reason_t seen, 0 if none yet

static EventGroupHandle_t s_events;
static esp_timer_handle_t s_retry_timer;
static esp_timer_handle_t s_dhcp_timer;
static uint32_t s_retry_delay_ms = RETRY_BASE_MS;
static char s_serial[13];           // 12 hex + NUL
static bool s_started;


/* --------------------------------------------------------- credentials */

static void load_credentials(void)
{
    // Kconfig is the factory default; a stored SSID overrides it.
    strlcpy(s_ssid, CONFIG_WIFI_STA_SSID, sizeof(s_ssid));
    strlcpy(s_pass, CONFIG_WIFI_STA_PASSWORD, sizeof(s_pass));

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored credentials, using the built-in default \"%s\"", s_ssid);
        return;
    }

    size_t len = sizeof(s_ssid);
    if (nvs_get_str(h, NVS_KEY_SSID, s_ssid, &len) == ESP_OK) {
        len = sizeof(s_pass);
        if (nvs_get_str(h, NVS_KEY_PASS, s_pass, &len) != ESP_OK) {
            s_pass[0] = '\0';       // stored as an open network
        }
        ESP_LOGI(TAG, "using stored credentials for \"%s\"", s_ssid);
    } else {
        ESP_LOGI(TAG, "no stored credentials, using the built-in default \"%s\"", s_ssid);
    }
    nvs_close(h);
}

static esp_err_t apply_sta_config(void)
{
    wifi_config_t cfg = { 0 };
    strlcpy((char *)cfg.sta.ssid, s_ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, s_pass, sizeof(cfg.sta.password));
    return esp_wifi_set_config(WIFI_IF_STA, &cfg);
}


/* --------------------------------------------------------------- retry */

static void schedule_reconnect(void)
{
    esp_timer_stop(s_retry_timer);          // harmless when not armed
    ESP_LOGI(TAG, "retry in %lu ms", (unsigned long)s_retry_delay_ms);
    esp_timer_start_once(s_retry_timer, (uint64_t)s_retry_delay_ms * 1000);

    s_retry_delay_ms *= 2;
    if (s_retry_delay_ms > RETRY_MAX_MS) {
        s_retry_delay_ms = RETRY_MAX_MS;
    }
}

/*
 * Every association attempt goes through here, because the failure mode is not
 * obvious: when esp_wifi_connect() returns an error it emits no
 * WIFI_EVENT_STA_DISCONNECTED, and STA_DISCONNECTED is the only thing that
 * arms the retry timer. Dropping the return value therefore does not cost one
 * attempt, it ends the retry chain outright and leaves the station offline
 * until the next reboot.
 */
static void connect_now(const char *why)
{
    const esp_err_t err = esp_wifi_connect();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s: associating with \"%s\"", why, s_ssid);
        return;
    }

    ESP_LOGW(TAG, "%s: connect call failed (%s), re-arming", why, esp_err_to_name(err));
    schedule_reconnect();
}

static void retry_timer_cb(void *arg)
{
    connect_now("retry");
}

static void dhcp_timer_cb(void *arg)
{
    /*
     * Drop the association rather than calling connect_now() straight away: the
     * disconnect raises STA_DISCONNECTED, which runs the normal teardown and
     * backoff instead of stacking a second attempt on top of a live link.
     */
    ESP_LOGW(TAG, "no DHCP lease within %d ms, dropping the association", DHCP_TIMEOUT_MS);
    const esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "disconnect failed (%s), retrying anyway", esp_err_to_name(err));
        schedule_reconnect();
    }
}


/* ---------------------------------------------------------------- names */

/*
 * A bare reason code sends you to the header every time, and the three that
 * matter most all mean something a person can act on: 201 is "that SSID is not
 * on the air", 202 and 204 are both "the passphrase did not take". Codes with
 * no entry here print as their number, which wifi_err_reason_t resolves.
 */
static const char *disconnect_reason_name(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_AUTH_EXPIRE:       return "auth expired";
    case WIFI_REASON_ASSOC_EXPIRE:      return "association expired";
    case WIFI_REASON_ASSOC_TOOMANY:     return "AP has too many stations";
    case WIFI_REASON_ASSOC_LEAVE:       return "we left";
    case WIFI_REASON_BEACON_TIMEOUT:    return "beacon timeout, AP out of range?";
    case WIFI_REASON_NO_AP_FOUND:       return "no AP with that SSID in range";
    case WIFI_REASON_AUTH_FAIL:         return "auth failed, wrong password?";
    case WIFI_REASON_ASSOC_FAIL:        return "association failed";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "4-way handshake timeout, wrong password?";
    case WIFI_REASON_CONNECTION_FAIL:   return "connection failed";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
                                        return "AP found but its security does not match";
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
                                        return "AP found but below the authmode threshold";
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
                                        return "AP found but too weak";
    default:                            return "see wifi_err_reason_t";
    }
}

static const char *authmode_name(wifi_auth_mode_t mode)
{
    switch (mode) {
    case WIFI_AUTH_OPEN:            return "open";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA-PSK";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2-PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2-PSK";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3-PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3-PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
    default:                        return "other";
    }
}


/* --------------------------------------------------------------- events */

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        connect_now("driver started");
        break;

    case WIFI_EVENT_STA_CONNECTED: {
        /*
         * Association is not yet "online" -- DHCP still has to answer. Logging
         * it separately is what distinguishes "the AP rejected us" from "the AP
         * took us but nothing handed out a lease", which look identical from
         * the ONLINE_BIT alone.
         */
        const wifi_event_sta_connected_t *ev = data;
        ESP_LOGI(TAG, "associated with \"%.*s\" bssid %02x:%02x:%02x:%02x:%02x:%02x "
                      "ch %u %s, waiting for DHCP",
                 ev->ssid_len, (const char *)ev->ssid,
                 ev->bssid[0], ev->bssid[1], ev->bssid[2],
                 ev->bssid[3], ev->bssid[4], ev->bssid[5],
                 ev->channel, authmode_name(ev->authmode));

        // Associated is not online. Give DHCP a deadline to prove it.
        esp_timer_stop(s_dhcp_timer);
        esp_timer_start_once(s_dhcp_timer, (uint64_t)DHCP_TIMEOUT_MS * 1000);
        break;
    }

    case WIFI_EVENT_STA_DISCONNECTED: {
        const wifi_event_sta_disconnected_t *ev = data;
        // Losing the association also loses the IP; anything mid-flight on a
        // socket is already doomed, so drop the flag before the retry is armed.
        s_last_reason = ev->reason;
        const bool was_online = (xEventGroupGetBits(s_events) & ONLINE_BIT) != 0;
        xEventGroupClearBits(s_events, ONLINE_BIT);
        esp_timer_stop(s_dhcp_timer);   // the association it was watching is gone
        ESP_LOGW(TAG, "disconnected from \"%s\": reason %u (%s)%s",
                 s_ssid, ev->reason, disconnect_reason_name(ev->reason),
                 was_online ? "" : " -- never got online");
        schedule_reconnect();
        break;
    }

    default:
        ESP_LOGD(TAG, "wifi event %ld", (long)id);
        break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id != IP_EVENT_STA_GOT_IP) {
        return;
    }

    const ip_event_got_ip_t *ev = data;
    ESP_LOGI(TAG, "online, ip " IPSTR " mask " IPSTR " gw " IPSTR,
             IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.netmask), IP2STR(&ev->ip_info.gw));

    esp_timer_stop(s_dhcp_timer);   // the lease arrived in time

    // A successful association is what resets the backoff -- not a successful
    // retry attempt, which may still be followed by an immediate drop.
    s_retry_delay_ms = RETRY_BASE_MS;
    xEventGroupSetBits(s_events, ONLINE_BIT);
}


/* ------------------------------------------------------------ accessors */

bool wifi_sta_is_online(void)
{
    if (!s_events) {
        return false;
    }
    return (xEventGroupGetBits(s_events) & ONLINE_BIT) != 0;
}

bool wifi_sta_wait_online(uint32_t timeout_ms)
{
    if (!s_events) {
        return false;
    }

    const EventBits_t bits = xEventGroupWaitBits(s_events, ONLINE_BIT, pdFALSE, pdTRUE,
                                                 pdMS_TO_TICKS(timeout_ms));
    return (bits & ONLINE_BIT) != 0;
}

int wifi_sta_rssi(void)
{
    if (!wifi_sta_is_online()) {
        return WIFI_RSSI_INVALID;
    }

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return WIFI_RSSI_INVALID;
    }
    return ap.rssi;
}

const char *wifi_sta_serial(void)
{
    return s_serial;
}

const char *wifi_sta_ssid(void)
{
    return s_ssid;
}

esp_err_t wifi_sta_verify_link(uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_started, ESP_ERR_INVALID_STATE, TAG, "not started");

    ESP_LOGI(TAG, "verifying the link to \"%s\" (up to %lu ms)",
             s_ssid, (unsigned long)timeout_ms);

    if (wifi_sta_wait_online(timeout_ms)) {
        wifi_sta_log_status();
        return ESP_OK;
    }

    /*
     * The reason code from the last failed attempt is the whole answer here --
     * 201 means the SSID was never seen, 202/204 mean it was seen and the
     * passphrase was rejected, and those two point at completely different
     * things to go and fix.
     */
    if (s_last_reason != 0) {
        ESP_LOGE(TAG, "could not join \"%s\" within %lu ms; last failure was "
                      "reason %u (%s)",
                 s_ssid, (unsigned long)timeout_ms, s_last_reason,
                 disconnect_reason_name(s_last_reason));
    } else {
        ESP_LOGE(TAG, "could not join \"%s\" within %lu ms; no association attempt "
                      "has completed yet",
                 s_ssid, (unsigned long)timeout_ms);
    }
    ESP_LOGW(TAG, "continuing offline -- the retry loop keeps running and the panel "
                  "offers a setup hotspot");
    return ESP_ERR_TIMEOUT;
}

esp_err_t wifi_sta_reconnect(void)
{
    ESP_RETURN_ON_FALSE(s_started, ESP_ERR_INVALID_STATE, TAG, "not started");
    ESP_RETURN_ON_ERROR(apply_sta_config(), TAG, "set config failed");

    /*
     * Drop whatever is in flight and let the ordinary retry path pick the new
     * config up, rather than associating from here: one code path owns
     * association, and reusing it means the new credentials get the same
     * DHCP deadline and backoff handling as any other attempt. The delay is
     * reset first so fresh credentials are tried at once instead of inheriting
     * a backoff that may already have grown to a minute.
     */
    esp_timer_stop(s_dhcp_timer);
    esp_wifi_disconnect();          // emits no event if we were not associated
    s_retry_delay_ms = RETRY_BASE_MS;
    schedule_reconnect();
    return ESP_OK;
}

esp_err_t wifi_sta_set_credentials(const char *ssid, const char *password)
{
    ESP_RETURN_ON_FALSE(ssid && ssid[0], ESP_ERR_INVALID_ARG, TAG, "empty ssid");
    if (!password) {
        password = "";
    }

    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h), TAG, "nvs open failed");

    esp_err_t err = nvs_set_str(h, NVS_KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, NVS_KEY_PASS, password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    ESP_RETURN_ON_ERROR(err, TAG, "credential store failed");

    strlcpy(s_ssid, ssid, sizeof(s_ssid));
    strlcpy(s_pass, password, sizeof(s_pass));
    ESP_LOGI(TAG, "stored credentials for \"%s\", reconnecting", s_ssid);

    return wifi_sta_reconnect();
}

void wifi_sta_log_status(void)
{
    if (!s_started) {
        ESP_LOGW(TAG, "status: driver not started");
        return;
    }

    if (!wifi_sta_is_online()) {
        ESP_LOGW(TAG, "status: offline, target \"%s\", next retry in <=%lu ms",
                 s_ssid, (unsigned long)s_retry_delay_ms);
        return;
    }

    wifi_ap_record_t ap;
    const esp_err_t err = esp_wifi_sta_get_ap_info(&ap);
    if (err != ESP_OK) {
        // Online per the event group but the driver has no AP record: worth
        // seeing, because it means the two disagree.
        ESP_LOGW(TAG, "status: online but no AP record (%s)", esp_err_to_name(err));
        return;
    }

    esp_netif_ip_info_t ip = { 0 };
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_get_ip_info(netif, &ip);
    }

    ESP_LOGI(TAG, "status: online ssid \"%s\" ch %u rssi %d dBm %s ip " IPSTR " serial %s",
             (const char *)ap.ssid, ap.primary, ap.rssi, authmode_name(ap.authmode),
             IP2STR(&ip.ip), s_serial);
}


/* ------------------------------------------------------------- lifecycle */

static esp_err_t nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // A partition left over from a differently-sized build is not worth
        // failing the boot over: the only thing in it is the Wi-Fi driver's
        // own calibration data, which it will rewrite.
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t wifi_sta_init(void)
{
    ESP_RETURN_ON_FALSE(!s_started, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    // This component and the cloud poster are the two the field cares about, so
    // they run at DEBUG while everything else stays at the project default.
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_RETURN_ON_ERROR(nvs_init(), TAG, "nvs init failed");

    s_events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_events, ESP_ERR_NO_MEM, TAG, "event group create failed");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop create failed");
    esp_netif_create_default_wifi_sta();

    const wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                            on_wifi_event, NULL, NULL),
                        TAG, "wifi handler register failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                            on_ip_event, NULL, NULL),
                        TAG, "ip handler register failed");

    const esp_timer_create_args_t retry_args = {
        .callback = retry_timer_cb,
        .name = "wifi_retry",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&retry_args, &s_retry_timer), TAG,
                        "retry timer create failed");

    const esp_timer_create_args_t dhcp_args = {
        .callback = dhcp_timer_cb,
        .name = "wifi_dhcp",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&dhcp_args, &s_dhcp_timer), TAG,
                        "dhcp timer create failed");

    load_credentials();

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode failed");
    ESP_RETURN_ON_ERROR(apply_sta_config(), TAG, "set config failed");

    // The reconnect logic above is the only thing that should be deciding when
    // to associate, so keep the driver's credentials out of its own NVS.
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "set storage failed");

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

    /*
     * Band selection is a runtime setting with no Kconfig behind it, and it can
     * only be set once the driver is running -- before esp_wifi_start() the API
     * returns ESP_ERR_WIFI_NOT_STARTED. Left alone, a station can end up
     * scanning one band while the AP is on the other, which looks exactly like
     * a healthy network that simply will not answer: reason 201, no AP found.
     * AUTO covers both, which is the only sane default for a dual-band part.
     *
     * STA_START may already have fired a first association by now. If it did
     * and it picked the wrong band, that attempt fails and the ordinary retry
     * path repeats it with both bands available.
     */
    esp_err_t band_err = esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO);
    if (band_err != ESP_OK) {
        ESP_LOGW(TAG, "could not select dual-band operation: %s", esp_err_to_name(band_err));
    } else {
        wifi_band_mode_t mode = 0;
        if (esp_wifi_get_band_mode(&mode) == ESP_OK) {
            ESP_LOGI(TAG, "band mode %s",
                     mode == WIFI_BAND_MODE_AUTO    ? "2.4 GHz + 5 GHz" :
                     mode == WIFI_BAND_MODE_5G_ONLY ? "5 GHz only" : "2.4 GHz only");
        }
    }

    uint8_t mac[6] = { 0 };
    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, mac), TAG, "get mac failed");
    snprintf(s_serial, sizeof(s_serial), "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    s_started = true;
    ESP_LOGI(TAG, "station started, ssid \"%s\", serial %s", s_ssid, s_serial);
    return ESP_OK;
}
