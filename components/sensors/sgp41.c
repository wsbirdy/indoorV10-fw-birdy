#include "sgp41.h"
#include "i2c.h"
#include "sensirion_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "sgp41";

/*
 * NOTE: this driver returns RAW ticks. Converting them into the 0..500 VOC/NOx
 * index that Sensirion documents requires their gas index algorithm, which is a
 * separate source drop (sensirion_gas_index_algorithm.c) not vendored here.
 * Raw ticks are still useful for verifying the part responds and drifts sanely.
 */

#define CMD_EXECUTE_CONDITIONING    0x2612
#define CMD_MEASURE_RAW             0x2619
#define CMD_EXECUTE_SELF_TEST       0x280E
#define CMD_TURN_HEATER_OFF         0x3615
#define CMD_GET_SERIAL_NUMBER       0x3682

#define CONDITIONING_DELAY_MS       50
#define MEASURE_DELAY_MS            55      // datasheet: 50 ms max
#define SELF_TEST_DELAY_MS          320
#define HEATER_OFF_DELAY_MS         1
#define SERIAL_DELAY_MS             1

#define I2C_TIMEOUT_MS              100

/* Default compensation: 50 %RH and 25 degC, with their pre-computed CRCs. */
#define DEFAULT_RH_TICKS            0x8000
#define DEFAULT_T_TICKS             0x6666

static i2c_master_dev_handle_t s_dev;

static void put_word(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)(value & 0xFF);
    dst[2] = sensirion_crc8(dst, 2);
}

/** Send a command plus optional parameter words, wait, then read result words. */
static esp_err_t sgp41_command(uint16_t cmd, const uint16_t *params, size_t n_params,
                               uint32_t delay_ms, uint16_t *result, size_t n_result)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(n_params <= 2 && n_result <= 3, ESP_ERR_INVALID_ARG, TAG, "bad length");

    uint8_t tx[2 + 2 * 3];
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFF);
    for (size_t i = 0; i < n_params; i++) {
        put_word(&tx[2 + i * 3], params[i]);
    }

    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_dev, tx, 2 + n_params * 3, I2C_TIMEOUT_MS), TAG,
                        "cmd 0x%04X write failed", cmd);

    if (delay_ms) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    if (n_result == 0) {
        return ESP_OK;
    }

    uint8_t rx[3 * 3];
    ESP_RETURN_ON_ERROR(i2c_master_receive(s_dev, rx, n_result * 3, I2C_TIMEOUT_MS), TAG,
                        "cmd 0x%04X read failed", cmd);

    for (size_t i = 0; i < n_result; i++) {
        const uint8_t *w = &rx[i * 3];
        ESP_RETURN_ON_FALSE(sensirion_crc8_ok(w), ESP_ERR_INVALID_CRC, TAG,
                            "cmd 0x%04X word %u CRC", cmd, (unsigned)i);
        result[i] = ((uint16_t)w[0] << 8) | w[1];
    }
    return ESP_OK;
}

esp_err_t sgp41_init(void)
{
    if (s_dev) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2c_add_device(SGP41_I2C_ADDR, &s_dev), TAG, "attach failed");

    uint16_t serial[3];
    ESP_RETURN_ON_ERROR(sgp41_command(CMD_GET_SERIAL_NUMBER, NULL, 0, SERIAL_DELAY_MS, serial, 3),
                        TAG, "serial read failed");

    ESP_LOGI(TAG, "ready at 0x%02X, serial %04X%04X%04X",
             SGP41_I2C_ADDR, serial[0], serial[1], serial[2]);
    return ESP_OK;
}

esp_err_t sgp41_condition(void)
{
    const uint16_t params[2] = { DEFAULT_RH_TICKS, DEFAULT_T_TICKS };
    uint16_t discard;

    // Returns the VOC raw value; the NOx pixel is what is actually being conditioned.
    return sgp41_command(CMD_EXECUTE_CONDITIONING, params, 2, CONDITIONING_DELAY_MS, &discard, 1);
}

esp_err_t sgp41_measure(float temp_c, float rh_pct, uint16_t *voc_raw, uint16_t *nox_raw)
{
    ESP_RETURN_ON_FALSE(voc_raw && nox_raw, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    if (rh_pct < 0.0f) {
        rh_pct = 0.0f;
    } else if (rh_pct > 100.0f) {
        rh_pct = 100.0f;
    }
    if (temp_c < -45.0f) {
        temp_c = -45.0f;
    } else if (temp_c > 130.0f) {
        temp_c = 130.0f;
    }

    const uint16_t params[2] = {
        (uint16_t)((rh_pct * 65535.0f) / 100.0f + 0.5f),
        (uint16_t)(((temp_c + 45.0f) * 65535.0f) / 175.0f + 0.5f),
    };

    uint16_t result[2];
    ESP_RETURN_ON_ERROR(sgp41_command(CMD_MEASURE_RAW, params, 2, MEASURE_DELAY_MS, result, 2),
                        TAG, "measure failed");

    *voc_raw = result[0];
    *nox_raw = result[1];
    return ESP_OK;
}

esp_err_t sgp41_heater_off(void)
{
    return sgp41_command(CMD_TURN_HEATER_OFF, NULL, 0, HEATER_OFF_DELAY_MS, NULL, 0);
}

esp_err_t sgp41_self_test(bool *passed)
{
    ESP_RETURN_ON_FALSE(passed, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    uint16_t result;
    ESP_RETURN_ON_ERROR(sgp41_command(CMD_EXECUTE_SELF_TEST, NULL, 0, SELF_TEST_DELAY_MS,
                                      &result, 1),
                        TAG, "self test failed");

    // Low byte: bit0 = VOC pixel failed, bit1 = NOx pixel failed. 0x00 means all good.
    *passed = ((result & 0x0003) == 0);
    if (!*passed) {
        ESP_LOGW(TAG, "self test flagged 0x%04X", result);
    }
    return ESP_OK;
}
