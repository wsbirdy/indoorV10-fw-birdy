#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPL07_I2C_ADDR_PRIMARY      0x77    // SDO tied high
#define SPL07_I2C_ADDR_SECONDARY    0x76    // SDO tied low

/**
 * @brief Probe both addresses, read the calibration coefficients and start
 *        continuous pressure + temperature measurement. Requires i2c_init().
 */
esp_err_t spl07_init(void);

/**
 * @brief Read the latest compensated sample.
 *
 * @param pressure_pa  pressure in pascals
 * @param temp_c       die temperature in degrees Celsius
 */
esp_err_t spl07_read(float *pressure_pa, float *temp_c);

/** @brief Pressure altitude from the ISA model, relative to @p sea_level_pa. */
float spl07_altitude_m(float pressure_pa, float sea_level_pa);

#ifdef __cplusplus
}
#endif
