#include <stdio.h>
#include <string.h>

#include "cloud_client.h"

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "wifi_sta.h"

#if CONFIG_CLOUD_TLS_PINNED_CA
extern const char server_ca_pem_start[] asm("_binary_server_ca_pem_start");
#else
#include "esp_crt_bundle.h"
#endif

static const char *TAG = "cloud_client";

#define URL_MAX     160

/*
 * Enough of the response to carry an error message. The backend answers a good
 * post with an empty body, so this only ever fills up when something has gone
 * wrong and the text is the most useful thing in the log.
 */
#define RESP_MAX    192


/* ------------------------------------------------------------------- url */

/*
 * The serial number is the whole of the device's identity: there is no token
 * and no signed body, so the backend knows a post only by the path it arrived
 * on. Anything sitting on the far end of this URL has to be prepared for that.
 */
static bool build_measures_url(char *buf, size_t size)
{
    const char *serial = CONFIG_CLOUD_SERIAL[0] != '\0' ? CONFIG_CLOUD_SERIAL
                                                        : wifi_sta_serial();
    if (serial[0] == '\0') {
        return false;   // wifi_sta_init() has not run yet
    }

    const int n = snprintf(buf, size, "%s://%s/sensors/airgradient:%s/measures",
                           CONFIG_CLOUD_SCHEME, CONFIG_CLOUD_HOST, serial);
    return n > 0 && (size_t)n < size;
}


/* ---------------------------------------------------------------- mapping */

static cloud_result_t map_post_status(int status)
{
    switch (status) {
    case 200:
    case 201:
        return CLOUD_OK;

    case 429:
        // Rate limited, but the sample was stored. Retrying would spend radio
        // time re-sending something the server already has.
        return CLOUD_OK;

    case 400:
        return CLOUD_ERR_NOT_REGISTERED;

    default:
        return CLOUD_ERR_SERVER;
    }
}

const char *cloud_result_name(cloud_result_t result)
{
    switch (result) {
    case CLOUD_OK:                  return "ok";
    case CLOUD_ERR_TRANSPORT:       return "transport";
    case CLOUD_ERR_NOT_REGISTERED:  return "not registered";
    case CLOUD_ERR_SERVER:          return "server";
    default:                        return "?";
    }
}


/* -------------------------------------------------------------- response */

/** @brief Accumulates the response body so a failure can be logged with its text. */
typedef struct {
    char   buf[RESP_MAX];
    size_t len;
} response_t;

static esp_err_t on_http_event(esp_http_client_event_t *evt)
{
    response_t *resp = evt->user_data;

    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "http: transport error");
        break;

    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "http: connected");
        break;

    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "http: request sent");
        break;

    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "http: < %s: %s", evt->header_key, evt->header_value);
        break;

    case HTTP_EVENT_ON_DATA:
        if (resp && evt->data_len > 0) {
            // Truncate rather than grow: this is a log line, not a parser input.
            const size_t room = sizeof(resp->buf) - 1 - resp->len;
            const size_t take = ((size_t)evt->data_len < room) ? (size_t)evt->data_len : room;
            memcpy(resp->buf + resp->len, evt->data, take);
            resp->len += take;
            resp->buf[resp->len] = '\0';
        }
        break;

    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "http: response complete");
        break;

    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "http: disconnected");
        break;

    default:
        break;
    }
    return ESP_OK;
}


/* ------------------------------------------------------------------- post */

cloud_result_t cloud_client_post_measures(const char *body, size_t body_len)
{
    if (!body) {
        return CLOUD_ERR_TRANSPORT;
    }

    char url[URL_MAX];
    if (!build_measures_url(url, sizeof(url))) {
        ESP_LOGE(TAG, "could not build url (wifi_sta_init() not run, or serial empty)");
        return CLOUD_ERR_TRANSPORT;
    }

    response_t resp = { .len = 0 };

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = CONFIG_CLOUD_HTTP_TIMEOUT_MS,
        .event_handler = on_http_event,
        .user_data = &resp,
#if CONFIG_CLOUD_TLS_PINNED_CA
        .cert_pem = server_ca_pem_start,
#else
        .crt_bundle_attach = esp_crt_bundle_attach,
#endif
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "client init failed");
        return CLOUD_ERR_TRANSPORT;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, (int)body_len);

    ESP_LOGD(TAG, "POST %s (%u byte body, %s, timeout %d ms)",
             url, (unsigned)body_len,
#if CONFIG_CLOUD_TLS_PINNED_CA
             "pinned CA",
#else
             "cert bundle",
#endif
             CONFIG_CLOUD_HTTP_TIMEOUT_MS);

    const int64_t started_us = esp_timer_get_time();
    const esp_err_t err = esp_http_client_perform(client);
    const int64_t elapsed_ms = (esp_timer_get_time() - started_us) / 1000;

    if (err != ESP_OK) {
        // No status code exists here -- DNS, TLS or the timeout ended it before
        // the server ever answered, so the elapsed time is the useful half.
        ESP_LOGW(TAG, "perform failed after %lld ms: %s", elapsed_ms, esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return CLOUD_ERR_TRANSPORT;
    }

    const int status = esp_http_client_get_status_code(client);
    const int64_t content_len = esp_http_client_get_content_length(client);
    esp_http_client_cleanup(client);

    const cloud_result_t result = map_post_status(status);

    if (result == CLOUD_OK) {
        ESP_LOGD(TAG, "post -> %d (%s) in %lld ms, %lld bytes back",
                 status, cloud_result_name(result), elapsed_ms, content_len);
    } else {
        ESP_LOGW(TAG, "post -> %d (%s) in %lld ms%s%s",
                 status, cloud_result_name(result), elapsed_ms,
                 resp.len ? ", body: " : "", resp.len ? resp.buf : "");
        ESP_LOGW(TAG, "  url was %s", url);
    }
    return result;
}
