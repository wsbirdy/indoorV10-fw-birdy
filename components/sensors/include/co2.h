#pragma once

#include <stdbool.h>
#include <stddef.h>
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
 * @brief Calibrations the module can run.
 *
 * Each one is acted on at the module's next measurement, and every reading
 * after it uses the new parameters -- including after a power cycle. Run one
 * only with the sensor sitting in the gas it is told it is in.
 */
typedef enum {
    /** Fresh air, against the module's own ABC target register (usually 400 ppm). */
    CO2_CAL_BACKGROUND,
    /** A known concentration, given as target_ppm to co2_calibrate(). */
    CO2_CAL_TARGET,
    /** 0 ppm, e.g. under nitrogen. */
    CO2_CAL_ZERO,
    /** Apply the ABC correction now instead of at the end of the ABC period. */
    CO2_CAL_FORCED_ABC,
    /** Undo every calibration and go back to the factory parameters. */
    CO2_CAL_FACTORY_RESTORE,
} co2_cal_t;

/*
 * co2_info_t.cal_status bits: which calibrations have succeeded since the word
 * was last cleared. co2_calibrate() clears it before it starts, so after a
 * calibration exactly one bit says which one ran.
 */
#define CO2_CAL_DONE_FACTORY        (1u << 2)
#define CO2_CAL_DONE_ABC            (1u << 3)
#define CO2_CAL_DONE_TARGET         (1u << 4)
#define CO2_CAL_DONE_BACKGROUND     (1u << 5)
#define CO2_CAL_DONE_ZERO           (1u << 6)

/** @brief Module status and calibration configuration, as read by co2_read_info(). */
typedef struct {
    uint16_t error_status;      ///< ErrorStatus word; co2_status_text() spells it out
    uint16_t cal_status;        ///< CO2_CAL_DONE_* bits
    uint16_t meter_control;     ///< raw feature-enable word
    bool     abc_enabled;       ///< automatic baseline correction is running
    uint16_t abc_period_h;      ///< ABC cycle length, hours
    uint16_t abc_target_ppm;    ///< what ABC and background calibration aim for
    uint16_t abc_time_h;        ///< hours since the last ABC correction
    bool     single_mode;       ///< measures only on request, rather than continuously
    uint16_t meas_period_s;     ///< continuous-mode measurement period
    uint16_t fw_rev;            ///< main revision in the high byte, sub in the low
    int16_t  temperature_c100;  ///< module chip temperature, degC x100
} co2_info_t;

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

    /** @brief Read status and calibration configuration. */
    esp_err_t (*read_info)(co2_info_t *info);

    /** @brief Run a calibration and wait for the module to confirm it. */
    esp_err_t (*calibrate)(co2_cal_t kind, uint16_t target_ppm);
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

/**
 * @brief Describe a status word as text, e.g. "OK" or "calibration, out of range".
 *
 * The power-up "no measurement yet" bit is reported as "warming up" rather than
 * as a fault.
 *
 * @return @p buf
 */
const char *co2_status_text(uint16_t status, char *buf, size_t len);

/**
 * @brief Read the module's status and calibration configuration.
 *
 * About ten Modbus transactions, so a few hundred milliseconds; safe to call
 * while the acquisition task is polling, which it is serialized against.
 */
esp_err_t co2_read_info(co2_info_t *info);

/**
 * @brief Calibrate the bound module and wait until it confirms.
 *
 * Blocks for up to one measurement period plus ten seconds while the module
 * takes the measurement the calibration rides on, so call it from a task that
 * can afford that. Readings keep flowing meanwhile.
 *
 * @param kind        which calibration
 * @param target_ppm  the concentration the sensor is in; used by CO2_CAL_TARGET
 *                    only, ignored otherwise
 * @return ESP_OK once the module reports success,
 *         ESP_ERR_INVALID_ARG for a target of 0 or past 32767 ppm,
 *         ESP_ERR_INVALID_STATE for background with no ABC target set, or
 *         forced ABC with ABC off -- the module would accept either and do nothing,
 *         ESP_FAIL if the module flagged a calibration error,
 *         ESP_ERR_TIMEOUT if it never confirmed
 */
esp_err_t co2_calibrate(co2_cal_t kind, uint16_t target_ppm);

/** @brief Which module got bound, or CO2_PART_NONE. */
co2_part_t co2_active_part(void);

/** @brief Name of the bound module for logging; "none" when unbound. */
const char *co2_active_name(void);

#ifdef __cplusplus
}
#endif
