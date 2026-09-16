#pragma once

/*
 * Modbus RTU transport shared by the Senseair CO2 variants.
 *
 * Private to the sensors component: the variant drivers include it, nothing
 * outside them should. Public API for CO2 is co2.h.
 *
 * Register numbers and addresses below are from Senseair TDE15927 rev 3,
 * "Modbus on S12 CO2". Note the document numbers registers from 1 while the
 * wire carries address = (register number - 1); everything here is the wire
 * address, so IR4 appears as 0x0003.
 */

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Addressing (TDE15927 table 1.1, table 1.2 and HR20).
 *
 * The S12 ships with address 0x68 and accepts 1..247 (0x01..0xF7). Address
 * 0xFE also reaches it, but the datasheet reserves that for production and
 * test, so it is not a safe way to identify a part -- an S12 answers there
 * too. 0x00, 0xF8..0xFD and 0xFF are ignored by the sensor.
 */
#define SENSEAIR_SLAVE_S12_DEFAULT  0x68
#define SENSEAIR_SLAVE_TEST         0xFE

/* Input registers (function code 0x04). */
#define SENSEAIR_REG_STATUS         0x0000  // IR1  ErrorStatus
#define SENSEAIR_REG_CO2            0x0003  // IR4  filtered, pressure compensated
#define SENSEAIR_REG_TEMPERATURE    0x0004  // IR5  signed, degC x100
#define SENSEAIR_REG_MEAS_COUNT     0x0006  // IR7  wraps at 255
#define SENSEAIR_REG_CO2_UNFILTERED 0x000A  // IR11
#define SENSEAIR_REG_FW_TYPE        0x0017  // IR24 constant per model
#define SENSEAIR_REG_FW_REV         0x001C  // IR29
#define SENSEAIR_REG_SENSOR_ID_HI   0x001D  // IR30 serial number, unique per unit
#define SENSEAIR_REG_SENSOR_ID_LO   0x001E  // IR31

/*
 * ErrorStatus bits (TDE15927 section 2.1). The register powers up at 0x0080
 * -- "no measurement completed" -- and clears itself once the first
 * measurement lands, so that bit is not a fault at probe time.
 */
#define SENSEAIR_ERR_COMMUNICATION  (1u << 1)
#define SENSEAIR_ERR_ALGORITHM      (1u << 2)
#define SENSEAIR_ERR_CALIBRATION    (1u << 3)
#define SENSEAIR_ERR_SELF_DIAG      (1u << 4)
#define SENSEAIR_ERR_OUT_OF_RANGE   (1u << 5)
#define SENSEAIR_ERR_NO_MEASUREMENT (1u << 7)
#define SENSEAIR_ERR_ABNORMAL_LEVEL (1u << 10)

/** @brief Faults worth logging; excludes the benign power-up bit. */
#define SENSEAIR_ERR_REAL_FAULTS    (SENSEAIR_ERR_COMMUNICATION | \
                                     SENSEAIR_ERR_ALGORITHM     | \
                                     SENSEAIR_ERR_CALIBRATION   | \
                                     SENSEAIR_ERR_SELF_DIAG     | \
                                     SENSEAIR_ERR_OUT_OF_RANGE  | \
                                     SENSEAIR_ERR_ABNORMAL_LEVEL)

/** @brief Read one input register (function code 0x04). */
esp_err_t senseair_modbus_read_input_reg(uint8_t slave, uint16_t reg, uint16_t *value);

/**
 * @brief Read a gas concentration register.
 *
 * Concentrations are signed 16-bit per the datasheet; a negative reading means
 * the module could not produce a valid sample, so it is rejected here rather
 * than handed up as a very large ppm value.
 *
 * @return ESP_ERR_INVALID_RESPONSE on a negative reading.
 */
esp_err_t senseair_modbus_read_concentration(uint8_t slave, uint16_t reg, uint16_t *ppm);

/** @brief Read the 32-bit sensor ID (serial number, unique per unit). */
esp_err_t senseair_modbus_read_sensor_id(uint8_t slave, uint32_t *id);

/**
 * @brief Shared probe body for the S-series variants.
 *
 * Waits out the power-up delay, retries ErrorStatus a few times, then checks
 * the firmware type register -- the one value that is constant per model and
 * so the only reliable way to tell two variants apart. (Sensor ID is a serial
 * number and differs per unit, so it cannot serve here.)
 *
 * @param expected_fw_type the model's constant, or 0 to accept any module that
 *        answers. A driver passing 0 cannot be distinguished from another and
 *        must be probed last.
 * @param name variant name, for the log line
 * @return ESP_OK if this variant is on the bus, ESP_ERR_NOT_FOUND if a
 *         different model answered, or the transport error if none did
 */
esp_err_t senseair_modbus_probe_variant(uint8_t slave, uint16_t expected_fw_type,
                                        const char *name);

#ifdef __cplusplus
}
#endif
