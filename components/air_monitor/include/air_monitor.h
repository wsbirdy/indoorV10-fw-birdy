#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "pm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Complete state of the air monitor board, one field group per device.
 *
 * Every group carries its own validity flag and update timestamp because the
 * acquisition tasks run independently and at different rates -- a consumer can
 * tell a stale group from a fresh one.
 */
typedef struct {
    /* SHT40 */
    float temperature_c;
    float humidity_pct;
    bool sht40_valid;
    uint32_t sht40_updated_ms;

    /* SGP41 -- raw ticks, not the 0..500 gas index */
    uint16_t voc_raw;
    uint16_t nox_raw;
    bool sgp41_conditioning;
    bool sgp41_valid;
    uint32_t sgp41_updated_ms;

    /* SPL07-003 */
    float pressure_pa;
    float die_temp_c;
    float altitude_m;
    bool spl07_valid;
    uint32_t spl07_updated_ms;

    /* Particulate module -- whichever part pm_init() bound */
    pm_data_t pm;
    bool pm_valid;
    uint32_t pm_updated_ms;

    /* Senseair CO2 */
    uint16_t co2_ppm;
    bool co2_valid;
    uint32_t co2_updated_ms;
} air_monitor_device;

/**
 * @brief Initialize every sensor on the board.
 *
 * Buses must already be up. A device that fails to probe is logged and skipped
 * rather than aborting, so one dead part does not take the board down.
 */
esp_err_t air_monitor_init(void);

/**
 * @brief Start one acquisition task per sensor.
 *
 * Call after air_monitor_init().
 */
esp_err_t air_monitor_start(void);

/** @brief Copy the current device state. Safe from any task. */
void air_monitor_get(air_monitor_device *out);

/**
 * @brief Latest temperature/humidity for gas-sensor compensation.
 *
 * Falls back to 25 C / 50 %RH when the SHT40 has no valid reading yet.
 *
 * @return true if the values came from a real reading
 */
bool air_monitor_compensation(float *temp_c, float *rh_pct);

/** @brief Sea-level reference used for the altitude estimate, in pascals. */
#define AIR_MONITOR_SEA_LEVEL_PA    101325.0f

#ifdef __cplusplus
}
#endif
