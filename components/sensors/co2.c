#include <stddef.h>
#include "co2.h"
#include "co2_drivers.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "co2";

/*
 * Probe order matters. The S12 identifies itself exactly -- own address, own
 * firmware type constant -- so it goes first and can only match a real S12.
 * The S88 has no known firmware type yet and so accepts any module that
 * answers; a catch-all like that has to come last or it would bind an S12
 * before the S12 driver ever got a turn. The S12 also answers at the S88's
 * address 0xFE, which the datasheet reserves for production and test, so
 * probing that address first would mis-identify the part.
 */
static const co2_driver_t *const s_drivers[] = {
    &senseair_s12_driver,
    &senseair_s88_driver,
};

/*
 * Bound once by co2_init() and only read afterwards, so the acquisition task
 * needs no lock to dispatch through it.
 */
static const co2_driver_t *s_active;

esp_err_t co2_init(void)
{
    s_active = NULL;

    for (size_t i = 0; i < sizeof(s_drivers) / sizeof(s_drivers[0]); i++) {
        const co2_driver_t *drv = s_drivers[i];
        if (drv->probe() == ESP_OK) {
            s_active = drv;
            ESP_LOGI(TAG, "bound %s", drv->name);
            return ESP_OK;
        }
        ESP_LOGD(TAG, "%s did not answer", drv->name);
    }

    ESP_LOGE(TAG, "no supported CO2 module on lpuart");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t co2_read_co2(uint16_t *ppm)
{
    ESP_RETURN_ON_FALSE(s_active, ESP_ERR_INVALID_STATE, TAG, "no module bound");
    return s_active->read_co2(ppm);
}

esp_err_t co2_read_status(uint16_t *status)
{
    ESP_RETURN_ON_FALSE(s_active, ESP_ERR_INVALID_STATE, TAG, "no module bound");
    return s_active->read_status(status);
}

co2_part_t co2_active_part(void)
{
    return s_active ? s_active->part : CO2_PART_NONE;
}

const char *co2_active_name(void)
{
    return s_active ? s_active->name : "none";
}
