#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CO2 modules this build knows how to drive. */
typedef enum {
    CO2_PART_NONE = 0,  // nothing bound; no module answered at init
    CO2_PART_S88,
    CO2_PART_S12,
} co2_part_t;

/**
 * @brief One concrete CO2 module driver.
 *
 * Adding a variant means defining one of these and listing it in the table in
 * co2.c; nothing above this header changes.
 */
typedef struct {
    const char *name;
    co2_part_t part;

    /** @brief Confirm this exact module is on the bus. */
    esp_err_t (*probe)(void);

    /** @brief Read the CO2 concentration in ppm. */
    esp_err_t (*read_co2)(uint16_t *ppm);

    /** @brief Read the module status word; non-zero means a latched fault. */
    esp_err_t (*read_status)(uint16_t *status);
} co2_driver_t;

/**
 * @brief Probe the bus and bind whichever supported module answers.
 *
 * Requires the LP UART to be initialized. Candidates are tried in table order
 * and the first successful probe wins, so the whole product family runs one
 * binary. Returns ESP_ERR_NOT_FOUND if nothing answered -- the reads below
 * then fail with ESP_ERR_INVALID_STATE rather than returning stale data.
 */
esp_err_t co2_init(void);

/** @brief Read the CO2 concentration in ppm from the bound module. */
esp_err_t co2_read_co2(uint16_t *ppm);

/**
 * @brief Read the module status word.
 *
 * Non-zero means a fault is latched; see the module's Modbus documentation for
 * the individual bits.
 */
esp_err_t co2_read_status(uint16_t *status);

/** @brief Which module got bound, or CO2_PART_NONE. */
co2_part_t co2_active_part(void);

/** @brief Name of the bound module for logging; "none" when unbound. */
const char *co2_active_name(void);

#ifdef __cplusplus
}
#endif
