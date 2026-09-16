#include "sht40.h"
#include "i2c.h"
#include "sensirion_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "sht40";

#define CMD_MEASURE_HIGH_PRECISION  0xFD
#define CMD_SOFT_RESET              0x94

#define MEASURE_DELAY_MS            10  // datasheet: 8.3 ms max for high precision
#define I2C_TIMEOUT_MS              100

static i2c_master_dev_handle_t s_dev;

esp_err_t sht40_init(void)
{
    if (s_dev) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2c_add_device(SHT40_I2C_ADDR, &s_dev), TAG, "attach failed");

    const uint8_t cmd = CMD_SOFT_RESET;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_dev, &cmd, 1, I2C_TIMEOUT_MS), TAG,
                        "soft reset failed");
    vTaskDelay(pdMS_TO_TICKS(2)); // datasheet: 1 ms max

    ESP_LOGI(TAG, "ready at 0x%02X", SHT40_I2C_ADDR);
    return ESP_OK;
}

esp_err_t sht40_read(float *temp_c, float *rh_pct)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(temp_c && rh_pct, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    const uint8_t cmd = CMD_MEASURE_HIGH_PRECISION;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_dev, &cmd, 1, I2C_TIMEOUT_MS), TAG,
                        "measure command failed");

    // The SHT40 NACKs its address until the conversion is done, so wait it out
    // rather than relying on clock stretching.
    vTaskDelay(pdMS_TO_TICKS(MEASURE_DELAY_MS));

    uint8_t rx[6];
    ESP_RETURN_ON_ERROR(i2c_master_receive(s_dev, rx, sizeof(rx), I2C_TIMEOUT_MS), TAG,
                        "read failed");

    ESP_RETURN_ON_FALSE(sensirion_crc8_ok(&rx[0]), ESP_ERR_INVALID_CRC, TAG, "temperature CRC");
    ESP_RETURN_ON_FALSE(sensirion_crc8_ok(&rx[3]), ESP_ERR_INVALID_CRC, TAG, "humidity CRC");

    const uint16_t t_ticks = ((uint16_t)rx[0] << 8) | rx[1];
    const uint16_t rh_ticks = ((uint16_t)rx[3] << 8) | rx[4];

    *temp_c = -45.0f + 175.0f * ((float)t_ticks / 65535.0f);

    float rh = -6.0f + 125.0f * ((float)rh_ticks / 65535.0f);
    if (rh < 0.0f) {
        rh = 0.0f;
    } else if (rh > 100.0f) {
        rh = 100.0f;
    }
    *rh_pct = rh;

    return ESP_OK;
}
