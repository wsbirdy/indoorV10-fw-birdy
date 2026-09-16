#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Particulate modules this build knows how to drive. */
typedef enum {
    PM_PART_NONE = 0,   // nothing bound; no module answered at init
    PM_PART_SPS30,      // Sensirion SPS30, I2C
    PM_PART_PMS5003,    // Plantower PMS5003, UART
} pm_part_t;

/**
 * @brief Which readings a sample actually carries.
 *
 * The two parts do not measure the same set: only the SPS30 reports a PM4.0
 * bin and a typical particle size. Rather than let a consumer read a zero and
 * believe it, a reading advertises the fields it filled in.
 */
typedef enum {
    PM_FIELD_PM1_0        = 1u << 0,
    PM_FIELD_PM2_5        = 1u << 1,
    PM_FIELD_PM4_0        = 1u << 2,   // SPS30 only
    PM_FIELD_PM10         = 1u << 3,
    PM_FIELD_TYPICAL_SIZE = 1u << 4,   // SPS30 only
} pm_field_t;

/** @brief One particulate reading, in the units both parts can agree on. */
typedef struct {
    float pm1_0_ugm3;
    float pm2_5_ugm3;
    float pm4_0_ugm3;
    float pm10_ugm3;
    float typical_size_um;
    uint32_t fields;        // bitmask of pm_field_t
} pm_data_t;

/** @brief True when @p field carries a real reading in @p data. */
static inline bool pm_has(const pm_data_t *data, pm_field_t field)
{
    return (data->fields & (uint32_t)field) != 0;
}

/**
 * @brief One concrete particulate module driver.
 *
 * The bus a part sits on is its own business: the SPS30 driver talks I2C and
 * the PMS5003 driver talks UART, and neither fact escapes this struct.
 */
typedef struct {
    const char *name;
    pm_part_t part;

    /** @brief Confirm this module is present and leave it measuring. */
    esp_err_t (*probe)(void);

    /** @brief Wait up to @p timeout_ms for a reading. */
    esp_err_t (*read)(pm_data_t *out, uint32_t timeout_ms);
} pm_driver_t;

/**
 * @brief Probe both buses and bind whichever supported module answers.
 *
 * Requires i2c_init() and the PMS5003's UART to be up. Candidates are tried in
 * table order and the first successful probe wins. Returns ESP_ERR_NOT_FOUND
 * if nothing answered, after which pm_read() fails with ESP_ERR_INVALID_STATE
 * rather than returning zeroes.
 */
esp_err_t pm_init(void);

/** @brief Wait up to @p timeout_ms for a reading from the bound module. */
esp_err_t pm_read(pm_data_t *out, uint32_t timeout_ms);

/** @brief Which module got bound, or PM_PART_NONE. */
pm_part_t pm_active_part(void);

/** @brief Name of the bound module for logging; "none" when unbound. */
const char *pm_active_name(void);

#ifdef __cplusplus
}
#endif
