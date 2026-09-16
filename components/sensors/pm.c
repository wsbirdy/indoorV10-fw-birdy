#include <stddef.h>
#include "pm.h"
#include "pm_drivers.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "pm";

/*
 * Probe order. The SPS30 goes first because it identifies itself exactly -- a
 * fixed product-type string at a fixed I2C address -- and because probing it
 * is cheap: one short transaction that either ACKs or does not.
 *
 * The PMS5003 has no identify command in the active mode it powers up in, so
 * the only way to recognise it is to wait for a well-formed frame, which costs
 * up to a couple of seconds. Cheap and exact before slow and inferred.
 */
static const pm_driver_t *const s_drivers[] = {
    &sps30_pm_driver,
    &pms5003_pm_driver,
};

/*
 * Bound once by pm_init() and only read afterwards, so the acquisition task
 * needs no lock to dispatch through it.
 */
static const pm_driver_t *s_active;

esp_err_t pm_init(void)
{
    s_active = NULL;

    for (size_t i = 0; i < sizeof(s_drivers) / sizeof(s_drivers[0]); i++) {
        const pm_driver_t *drv = s_drivers[i];
        if (drv->probe() == ESP_OK) {
            s_active = drv;
            ESP_LOGI(TAG, "bound %s", drv->name);
            return ESP_OK;
        }
        ESP_LOGD(TAG, "%s did not answer", drv->name);
    }

    ESP_LOGE(TAG, "no supported particulate module found");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t pm_read(pm_data_t *out, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_active, ESP_ERR_INVALID_STATE, TAG, "no module bound");
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");
    return s_active->read(out, timeout_ms);
}

pm_part_t pm_active_part(void)
{
    return s_active ? s_active->part : PM_PART_NONE;
}

const char *pm_active_name(void)
{
    return s_active ? s_active->name : "none";
}
