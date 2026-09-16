#include <string.h>
#include "air_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"

#include "sht40.h"
#include "sgp41.h"
#include "spl07.h"
#include "pm.h"
#include "co2.h"

static const char *TAG = "air_monitor";

/* Per-sensor acquisition periods. */
#define SHT40_PERIOD_MS     1000
#define SGP41_PERIOD_MS     1000    // the gas index algorithm assumes exactly 1 Hz
#define SPL07_PERIOD_MS     1000
#define CO2_PERIOD_MS       5000
#define PM_TIMEOUT_MS       3000    // the sensor free-runs at roughly 1 Hz

/* Fallback compensation before the SHT40 has produced a reading. */
#define DEFAULT_TEMP_C      25.0f
#define DEFAULT_RH_PCT      50.0f

static air_monitor_device s_device;
static SemaphoreHandle_t s_lock;

static uint32_t now_ms(void)
{
    return pdTICKS_TO_MS(xTaskGetTickCount());
}


/* ------------------------------------------------------------ accessors */

void air_monitor_get(air_monitor_device *out)
{
    if (!out || !s_lock) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_device;
    xSemaphoreGive(s_lock);
}

bool air_monitor_compensation(float *temp_c, float *rh_pct)
{
    if (!temp_c || !rh_pct || !s_lock) {
        return false;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool valid = s_device.sht40_valid;
    *temp_c = valid ? s_device.temperature_c : DEFAULT_TEMP_C;
    *rh_pct = valid ? s_device.humidity_pct : DEFAULT_RH_PCT;
    xSemaphoreGive(s_lock);

    return valid;
}


/* --------------------------------------------------------------- tasks */

static void sht40_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        float temperature = 0.0f, humidity = 0.0f;
        const bool ok = (sht40_read(&temperature, &humidity) == ESP_OK);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (ok) {
            s_device.temperature_c = temperature;
            s_device.humidity_pct = humidity;
        }
        s_device.sht40_valid = ok;
        s_device.sht40_updated_ms = now_ms();
        xSemaphoreGive(s_lock);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SHT40_PERIOD_MS));
    }
}

static void sgp41_task(void *arg)
{
    // The NOx pixel is invalid until conditioned, which takes one command per
    // second for the first 10 s after power-up.
    int conditioning_left = SGP41_CONDITIONING_S;

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        uint16_t voc_raw = 0, nox_raw = 0;
        bool ok = false;

        if (conditioning_left > 0) {
            if (sgp41_condition() == ESP_OK) {
                conditioning_left--;
                if (conditioning_left == 0) {
                    ESP_LOGI(TAG, "SGP41 conditioning complete");
                }
            }
        } else {
            // Compensate with whatever the SHT40 task last published.
            float temperature = DEFAULT_TEMP_C, humidity = DEFAULT_RH_PCT;
            air_monitor_compensation(&temperature, &humidity);
            ok = (sgp41_measure(temperature, humidity, &voc_raw, &nox_raw) == ESP_OK);
        }

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (ok) {
            s_device.voc_raw = voc_raw;
            s_device.nox_raw = nox_raw;
        }
        s_device.sgp41_conditioning = (conditioning_left > 0);
        s_device.sgp41_valid = ok;
        s_device.sgp41_updated_ms = now_ms();
        xSemaphoreGive(s_lock);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SGP41_PERIOD_MS));
    }
}

static void spl07_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        float pressure = 0.0f, die_temp = 0.0f;
        const bool ok = (spl07_read(&pressure, &die_temp) == ESP_OK);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (ok) {
            s_device.pressure_pa = pressure;
            s_device.die_temp_c = die_temp;
            s_device.altitude_m = spl07_altitude_m(pressure, AIR_MONITOR_SEA_LEVEL_PA);
        }
        s_device.spl07_valid = ok;
        s_device.spl07_updated_ms = now_ms();
        xSemaphoreGive(s_lock);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SPL07_PERIOD_MS));
    }
}

static void pm_task(void *arg)
{
    while (1) {
        pm_data_t pm;
        // Blocks until the next reading arrives; the sensor sets the cadence.
        const bool ok = (pm_read(&pm, PM_TIMEOUT_MS) == ESP_OK);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (ok) {
            s_device.pm = pm;
        }
        s_device.pm_valid = ok;
        s_device.pm_updated_ms = now_ms();
        xSemaphoreGive(s_lock);

        if (!ok) {
            ESP_LOGW(TAG, "no particulate reading");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void co2_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        uint16_t ppm = 0;
        const bool ok = (co2_read_co2(&ppm) == ESP_OK);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (ok) {
            s_device.co2_ppm = ppm;
        }
        s_device.co2_valid = ok;
        s_device.co2_updated_ms = now_ms();
        xSemaphoreGive(s_lock);

        if (!ok) {
            ESP_LOGW(TAG, "no CO2 response");
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CO2_PERIOD_MS));
    }
}


/* ------------------------------------------------------------- lifecycle */

esp_err_t air_monitor_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "mutex create failed");
    }

    memset(&s_device, 0, sizeof(s_device));

    esp_err_t err;
    if ((err = sht40_init()) != ESP_OK) {
        ESP_LOGE(TAG, "SHT40 init failed: %s", esp_err_to_name(err));
    }
    if ((err = sgp41_init()) != ESP_OK) {
        ESP_LOGE(TAG, "SGP41 init failed: %s", esp_err_to_name(err));
    } else {
        bool passed = false;
        if (sgp41_self_test(&passed) == ESP_OK) {
            ESP_LOGI(TAG, "SGP41 self test %s", passed ? "passed" : "FAILED");
        }
    }
    if ((err = spl07_init()) != ESP_OK) {
        ESP_LOGE(TAG, "SPL07-003 init failed: %s", esp_err_to_name(err));
    }
    if ((err = pm_init()) != ESP_OK) {
        ESP_LOGE(TAG, "particulate module init failed: %s", esp_err_to_name(err));
    }
    if ((err = co2_init()) != ESP_OK) {
        ESP_LOGE(TAG, "CO2 module init failed: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

esp_err_t air_monitor_start(void)
{
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_INVALID_STATE, TAG, "air_monitor_init() first");

    static const struct {
        TaskFunction_t fn;
        const char *name;
        uint32_t stack;
        UBaseType_t priority;
    } tasks[] = {
        { sht40_task,    "sht40",    3072, 5 },
        { sgp41_task,    "sgp41",    3072, 5 },
        { spl07_task,    "spl07",    3072, 5 },
        { pm_task,       "pm",       3072, 4 },
        { co2_task,      "co2",      3072, 4 },
    };

    for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
        BaseType_t ok = xTaskCreate(tasks[i].fn, tasks[i].name, tasks[i].stack, NULL,
                                    tasks[i].priority, NULL);
        ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG,
                            "could not create %s task", tasks[i].name);
    }

    ESP_LOGI(TAG, "%u acquisition tasks running", (unsigned)(sizeof(tasks) / sizeof(tasks[0])));
    return ESP_OK;
}
