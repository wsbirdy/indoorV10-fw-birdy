#include <string.h>
#include "senseair_modbus.h"
#include "uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "senseair_mb";

#define SENSEAIR_PORT       UART_ID_LP
#define FUNC_READ_INPUT_REG 0x04

#define REQUEST_LEN         8
#define RESPONSE_LEN        7   // addr + func + bytecount + 2 data + 2 CRC
#define RESPONSE_TIMEOUT_MS 500

static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x0001) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

esp_err_t senseair_modbus_read_input_reg(uint8_t slave, uint16_t reg, uint16_t *value)
{
    ESP_RETURN_ON_FALSE(value, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    uint8_t req[REQUEST_LEN] = {
        slave,
        FUNC_READ_INPUT_REG,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        0x00, 0x01,             // read one register
    };
    const uint16_t crc = modbus_crc16(req, REQUEST_LEN - 2);
    req[6] = (uint8_t)(crc & 0xFF);     // Modbus sends the CRC low byte first
    req[7] = (uint8_t)(crc >> 8);

    // Drop any stale bytes so a late reply from a previous poll is not mistaken
    // for this one.
    uint8_t discard[16];
    while (uart_read(SENSEAIR_PORT, discard, sizeof(discard), 0) > 0) {
        ;
    }

    ESP_RETURN_ON_FALSE(uart_write(SENSEAIR_PORT, req, REQUEST_LEN) == REQUEST_LEN,
                        ESP_FAIL, TAG, "request write failed");

    uint8_t rsp[RESPONSE_LEN];
    const int got = uart_read(SENSEAIR_PORT, rsp, RESPONSE_LEN, RESPONSE_TIMEOUT_MS);
    ESP_RETURN_ON_FALSE(got == RESPONSE_LEN, ESP_ERR_TIMEOUT, TAG,
                        "short response (%d of %d bytes)", got, RESPONSE_LEN);

    ESP_RETURN_ON_FALSE(rsp[0] == slave, ESP_ERR_INVALID_RESPONSE, TAG,
                        "wrong slave 0x%02X", rsp[0]);
    // A set MSB in the function code marks a Modbus exception reply.
    ESP_RETURN_ON_FALSE(rsp[1] == FUNC_READ_INPUT_REG, ESP_ERR_INVALID_RESPONSE, TAG,
                        "exception response, func 0x%02X", rsp[1]);
    ESP_RETURN_ON_FALSE(rsp[2] == 2, ESP_ERR_INVALID_RESPONSE, TAG,
                        "unexpected byte count %u", rsp[2]);

    const uint16_t rx_crc = (uint16_t)rsp[5] | ((uint16_t)rsp[6] << 8);
    ESP_RETURN_ON_FALSE(rx_crc == modbus_crc16(rsp, RESPONSE_LEN - 2), ESP_ERR_INVALID_CRC,
                        TAG, "crc mismatch");

    *value = ((uint16_t)rsp[3] << 8) | rsp[4];
    return ESP_OK;
}

esp_err_t senseair_modbus_read_concentration(uint8_t slave, uint16_t reg, uint16_t *ppm)
{
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(senseair_modbus_read_input_reg(slave, reg, &raw), TAG,
                        "concentration read failed");

    ESP_RETURN_ON_FALSE((int16_t)raw >= 0, ESP_ERR_INVALID_RESPONSE, TAG,
                        "negative concentration %d", (int)(int16_t)raw);

    *ppm = raw;
    return ESP_OK;
}

esp_err_t senseair_modbus_read_sensor_id(uint8_t slave, uint32_t *id)
{
    ESP_RETURN_ON_FALSE(id, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    uint16_t hi = 0, lo = 0;
    ESP_RETURN_ON_ERROR(senseair_modbus_read_input_reg(slave, SENSEAIR_REG_SENSOR_ID_HI, &hi),
                        TAG, "sensor id high read failed");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_input_reg(slave, SENSEAIR_REG_SENSOR_ID_LO, &lo),
                        TAG, "sensor id low read failed");

    *id = ((uint32_t)hi << 16) | lo;
    return ESP_OK;
}

esp_err_t senseair_modbus_probe_variant(uint8_t slave, uint16_t expected_fw_type,
                                        const char *name)
{
    // Modules need a moment after power-up before they answer on the bus.
    vTaskDelay(pdMS_TO_TICKS(100));

    uint16_t status = 0;
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 3 && err != ESP_OK; attempt++) {
        err = senseair_modbus_read_input_reg(slave, SENSEAIR_REG_STATUS, &status);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    if (err != ESP_OK) {
        return err;      // nothing at this address
    }

    uint16_t fw_type = 0;
    const esp_err_t type_err = senseair_modbus_read_input_reg(slave, SENSEAIR_REG_FW_TYPE,
                                                              &fw_type);

    /*
     * expected_fw_type 0 means the constant for this model has not been taken
     * from its datasheet yet, so accept whatever answered. Such a driver has to
     * sit last in the probe table or it will swallow the models above it.
     */
    if (expected_fw_type != 0) {
        ESP_RETURN_ON_ERROR(type_err, TAG, "%s: firmware type unreadable", name);
        ESP_RETURN_ON_FALSE(fw_type == expected_fw_type, ESP_ERR_NOT_FOUND, TAG,
                            "%s: firmware type 0x%04X is not 0x%04X",
                            name, fw_type, expected_fw_type);
    }

    uint32_t id = 0;
    if (senseair_modbus_read_sensor_id(slave, &id) == ESP_OK) {
        ESP_LOGI(TAG, "%s at 0x%02X: fw type 0x%04X, sensor id %lu, status 0x%04X",
                 name, slave, fw_type, (unsigned long)id, status);
    } else {
        ESP_LOGI(TAG, "%s at 0x%02X: fw type 0x%04X, status 0x%04X",
                 name, slave, fw_type, status);
    }

    // 0x0080 alone is the power-up "no measurement completed" state, not a fault.
    if (status & SENSEAIR_ERR_REAL_FAULTS) {
        ESP_LOGW(TAG, "%s: module reports fault, status 0x%04X", name, status);
    }
    return ESP_OK;
}
