#include <stdatomic.h>

#include "cloud.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "air_monitor.h"
#include "cloud_client.h"
#include "cloud_payload.h"
#include "wifi_sta.h"

static const char *TAG = "cloud";

/*
 * Below the acquisition tasks (4-5) so a slow TLS handshake can never delay a
 * sensor read, and above the idle task so it still runs on a busy board.
 */
#define CLOUD_TASK_PRIORITY     3

/*
 * The body is built on this stack, and mbedTLS is not frugal during a
 * handshake. 6 KB leaves headroom over the observed high-water mark.
 */
#define CLOUD_TASK_STACK        6144

/* How long to wait for the link before giving up on this cycle and re-checking. */
#define ONLINE_WAIT_MS          10000

/*
 * Reported instead of the driver's 0 dBm sentinel. A dashboard plotting the
 * raw 0 would show the best signal the device ever had at exactly the moment
 * it had none.
 */
#define RSSI_OFFLINE            (-127)

static atomic_bool s_enabled = ATOMIC_VAR_INIT(true);
static TaskHandle_t s_task;

static uint32_t uptime_minutes(void)
{
    return (uint32_t)(esp_timer_get_time() / (60 * 1000000LL));
}

static void post_once(void)
{
    air_monitor_device dev;
    air_monitor_get(&dev);

    const int raw_rssi = wifi_sta_rssi();
    const int rssi = (raw_rssi == WIFI_RSSI_INVALID) ? RSSI_OFFLINE : raw_rssi;

    char body[CLOUD_PAYLOAD_MAX];
    const size_t len = cloud_payload_build(body, sizeof(body), &dev, rssi, uptime_minutes());
    if (len == 0) {
        ESP_LOGE(TAG, "payload build failed");
        return;
    }

    // The link state belongs next to the attempt it explains: a post that fails
    // at -89 dBm is a different problem from one that fails at -45.
    wifi_sta_log_status();
    ESP_LOGD(TAG, "post %u bytes, rssi %d dBm, heap %lu free (%lu largest block)",
             (unsigned)len, rssi,
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI(TAG, "post %s", body);

    const int64_t started_us = esp_timer_get_time();
    const cloud_result_t result = cloud_client_post_measures(body, len);
    const int64_t elapsed_ms = (esp_timer_get_time() - started_us) / 1000;

    if (result == CLOUD_OK) {
        ESP_LOGI(TAG, "accepted (%u bytes in %lld ms)", (unsigned)len, elapsed_ms);
    } else {
        // Nothing is queued for a later attempt: this interval's sample is
        // simply lost, and the next tick carries fresher readings anyway.
        ESP_LOGW(TAG, "post failed after %lld ms: %s -- sample dropped, next try in %d s",
                 elapsed_ms, cloud_result_name(result), CONFIG_CLOUD_POST_INTERVAL_S);
    }
}

static void cloud_task(void *arg)
{
    while (1) {
        /*
         * Anchor the interval to the start of the cycle, not the end. A post
         * that takes 15 s then leaves 45 s of quiet rather than stretching the
         * cadence out to 75 s and drifting a little further every time the
         * network is slow.
         */
        const int64_t cycle_started_us = esp_timer_get_time();

        if (!atomic_load(&s_enabled)) {
            ESP_LOGD(TAG, "disabled, skipping cycle");
        } else if (!wifi_sta_wait_online(ONLINE_WAIT_MS)) {
            ESP_LOGW(TAG, "offline after waiting %d ms, skipping cycle", ONLINE_WAIT_MS);
            wifi_sta_log_status();
        } else {
            post_once();
        }

        const int64_t elapsed_ms = (esp_timer_get_time() - cycle_started_us) / 1000;
        int64_t remaining_ms = (int64_t)CONFIG_CLOUD_POST_INTERVAL_S * 1000 - elapsed_ms;
        if (remaining_ms < 0) {
            // Overran the interval; go straight into the next cycle rather
            // than trying to catch up on the ones that were missed.
            remaining_ms = 0;
        }

        // A notification from cloud_post_now() cuts the wait short.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS((uint32_t)remaining_ms));
    }
}


/* ------------------------------------------------------------------- api */

void cloud_set_enabled(bool enabled)
{
    atomic_store(&s_enabled, enabled);
    ESP_LOGI(TAG, "%s", enabled ? "enabled" : "disabled");

    if (enabled) {
        cloud_post_now();
    }
}

bool cloud_is_enabled(void)
{
    return atomic_load(&s_enabled);
}

void cloud_post_now(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

esp_err_t cloud_start(void)
{
    if (s_task) {
        return ESP_OK;
    }

    // Paired with wifi_sta: these two tags carry the field diagnostics, so they
    // run at DEBUG while the project default stays at INFO.
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("cloud_client", ESP_LOG_DEBUG);

    const BaseType_t ok = xTaskCreate(cloud_task, "cloud", CLOUD_TASK_STACK, NULL,
                                      CLOUD_TASK_PRIORITY, &s_task);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "could not create cloud task");

    ESP_LOGI(TAG, "posting to %s every %d s", CONFIG_CLOUD_HOST, CONFIG_CLOUD_POST_INTERVAL_S);
    return ESP_OK;
}
