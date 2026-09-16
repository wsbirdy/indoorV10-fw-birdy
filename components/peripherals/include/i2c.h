#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_PORT_NUM        I2C_NUM_0
#define I2C_SDA_GPIO        25   
#define I2C_SCL_GPIO        26
#define I2C_CLK_SPEED_HZ    400000

/**
 * @brief Initialize the I2C master bus.
 *
 * Safe to call once at startup. Subsequent calls are no-ops.
 */
esp_err_t i2c_init(void);

/** @brief Bus handle created by i2c_init(), NULL before init. */
i2c_master_bus_handle_t i2c_get_bus(void);

/**
 * @brief Attach a device to the bus at the bus's default clock.
 *
 * @param addr_7bit 7-bit device address
 * @param out_dev   receives the device handle
 */
esp_err_t i2c_add_device(uint8_t addr_7bit, i2c_master_dev_handle_t *out_dev);

/**
 * @brief Attach a device that cannot run at the bus default.
 *
 * The clock is per-device in the ESP-IDF master driver: it is applied for that
 * device's transactions only, so one slow part does not hold the rest of the
 * bus down. Use this for parts rated below I2C_CLK_SPEED_HZ.
 *
 * @param scl_speed_hz clock for this device's transactions
 */
esp_err_t i2c_add_device_at_speed(uint8_t addr_7bit, uint32_t scl_speed_hz,
                                  i2c_master_dev_handle_t *out_dev);

#ifdef __cplusplus
}
#endif
