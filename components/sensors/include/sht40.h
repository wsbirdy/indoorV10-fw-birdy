#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHT40_I2C_ADDR      0x44    // SHT40-AD1B; -BD1B variants use 0x45

/** @brief Attach the SHT40 to the I2C bus and soft-reset it. Requires i2c_init(). */
esp_err_t sht40_init(void);

/**
 * @brief High-precision one-shot measurement (~10 ms).
 *
 * @param temp_c  degrees Celsius
 * @param rh_pct  relative humidity, clamped to 0..100
 */
esp_err_t sht40_read(float *temp_c, float *rh_pct);

#ifdef __cplusplus
}
#endif
