#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SGP41_I2C_ADDR          0x59
#define SGP41_CONDITIONING_S    10      // NOx pixel needs 10 s of conditioning at power-up

/** @brief Attach the SGP41 and read its serial number. Requires i2c_init(). */
esp_err_t sgp41_init(void);

/**
 * @brief Run one 1-second conditioning step.
 *
 * Must be called once per second for SGP41_CONDITIONING_S seconds after power-up,
 * before any sgp41_measure() call, or the NOx signal stays invalid.
 */
esp_err_t sgp41_condition(void);

/**
 * @brief Read raw VOC/NOx ticks with humidity/temperature compensation.
 *
 * Pass the SHT40 readings for compensation. These are raw ticks, not indices --
 * see the note in sgp41.c about the gas index algorithm.
 */
esp_err_t sgp41_measure(float temp_c, float rh_pct, uint16_t *voc_raw, uint16_t *nox_raw);

/** @brief Turn the hotplate off (idle mode, ~34 uA). */
esp_err_t sgp41_heater_off(void);

/** @brief Built-in self test; sets *passed on success. */
esp_err_t sgp41_self_test(bool *passed);

#ifdef __cplusplus
}
#endif
