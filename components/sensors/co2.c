#include <stddef.h>
#include <stdio.h>
#include "co2.h"
#include "co2_drivers.h"
#include "senseair_modbus.h"
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

/** One line of what the module is set up to do, so a boot log answers "is ABC on?". */
static void log_info(void)
{
    co2_info_t info;
    if (co2_read_info(&info) != ESP_OK) {
        ESP_LOGW(TAG, "%s: status and calibration settings unreadable", s_active->name);
        return;
    }

    char status[64];
    ESP_LOGI(TAG, "%s: fw %u.%u, %.2f C, status %s (0x%04X), calibrations 0x%02X",
             s_active->name, info.fw_rev >> 8, info.fw_rev & 0xFF,
             info.temperature_c100 / 100.0, co2_status_text(info.error_status, status,
                                                            sizeof(status)),
             info.error_status, info.cal_status);
    ESP_LOGI(TAG, "%s: ABC %s, period %u h, target %u ppm, %u h since last; "
             "%s mode, %u s period",
             s_active->name, info.abc_enabled ? "on" : "off", info.abc_period_h,
             info.abc_target_ppm, info.abc_time_h,
             info.single_mode ? "single" : "continuous", info.meas_period_s);
}

esp_err_t co2_init(void)
{
    s_active = NULL;

    for (size_t i = 0; i < sizeof(s_drivers) / sizeof(s_drivers[0]); i++) {
        const co2_driver_t *drv = s_drivers[i];
        if (drv->probe() == ESP_OK) {
            s_active = drv;
            ESP_LOGI(TAG, "bound %s", drv->name);
            log_info();
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

esp_err_t co2_read_info(co2_info_t *info)
{
    ESP_RETURN_ON_FALSE(s_active, ESP_ERR_INVALID_STATE, TAG, "no module bound");
    ESP_RETURN_ON_FALSE(s_active->read_info, ESP_ERR_NOT_SUPPORTED, TAG,
                        "%s has no status read", s_active->name);
    return s_active->read_info(info);
}

esp_err_t co2_calibrate(co2_cal_t kind, uint16_t target_ppm)
{
    ESP_RETURN_ON_FALSE(s_active, ESP_ERR_INVALID_STATE, TAG, "no module bound");
    ESP_RETURN_ON_FALSE(s_active->calibrate, ESP_ERR_NOT_SUPPORTED, TAG,
                        "%s cannot calibrate", s_active->name);
    return s_active->calibrate(kind, target_ppm);
}

const char *co2_status_text(uint16_t status, char *buf, size_t len)
{
    /*
     * Both bound modules are Senseair parts sharing one ErrorStatus layout
     * (TDE15927 section 2.1), so the bits are named once here rather than per
     * driver.
     */
    static const struct {
        uint16_t    bit;
        const char *name;
    } BITS[] = {
        { SENSEAIR_ERR_COMMUNICATION,  "comms" },
        { SENSEAIR_ERR_ALGORITHM,      "algorithm" },
        { SENSEAIR_ERR_CALIBRATION,    "calibration" },
        { SENSEAIR_ERR_SELF_DIAG,      "self-diag" },
        { SENSEAIR_ERR_OUT_OF_RANGE,   "out of range" },
        { SENSEAIR_ERR_ABNORMAL_LEVEL, "abnormal signal" },
        { SENSEAIR_ERR_NO_MEASUREMENT, "warming up" },
    };

    if (len == 0) {
        return buf;
    }
    buf[0] = '\0';
    if (status == 0) {
        snprintf(buf, len, "OK");
        return buf;
    }

    size_t used = 0;
    for (size_t i = 0; i < sizeof(BITS) / sizeof(BITS[0]) && used < len; i++) {
        if (status & BITS[i].bit) {
            used += snprintf(buf + used, len - used, "%s%s", used ? ", " : "", BITS[i].name);
        }
    }
    if (buf[0] == '\0') {
        snprintf(buf, len, "unknown");
    }
    return buf;
}

co2_part_t co2_active_part(void)
{
    return s_active ? s_active->part : CO2_PART_NONE;
}

const char *co2_active_name(void)
{
    return s_active ? s_active->name : "none";
}
