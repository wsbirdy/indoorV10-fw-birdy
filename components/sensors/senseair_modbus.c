#include <string.h>
#include "senseair_modbus.h"
#include "uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "senseair_mb";

#define SENSEAIR_PORT           UART_ID_LP
#define FUNC_READ_HOLDING_REG   0x03
#define FUNC_READ_INPUT_REG     0x04
#define FUNC_WRITE_MULTIPLE     0x10
#define FUNC_EXCEPTION_FLAG     0x80

#define READ_REQUEST_LEN    8   // addr + func + reg(2) + count(2) + CRC(2)
#define READ_RESPONSE_LEN   7   // addr + func + bytecount + 2 data + 2 CRC
#define WRITE_REQUEST_LEN   11  // addr + func + reg(2) + count(2) + bytecount + 2 data + CRC(2)
#define WRITE_RESPONSE_LEN  8   // addr + func + reg(2) + count(2) + CRC(2)
#define EXCEPTION_LEN       5   // addr + (func | 0x80) + code + CRC(2)
#define RESPONSE_TIMEOUT_MS 500

/*
 * Calibration status polling. The sensor calibrates on the first measurement
 * after the command, so the wait is one measurement period plus slack for the
 * sampling itself and the turnaround (TDE15927 table 1.3, at most 180 ms).
 */
#define CAL_POLL_MS         500
#define CAL_SLACK_MS        10000

/*
 * The acquisition task polls the module while a calibration or a status read
 * runs from another task, and a Modbus request/response pair must not be
 * interleaved with another on the one UART. Created on the first transaction,
 * which co2_init()'s probe makes before any task that could race it exists.
 */
static SemaphoreHandle_t s_bus_lock;

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

static void put_crc(uint8_t *frame, size_t len_without_crc)
{
    const uint16_t crc = modbus_crc16(frame, len_without_crc);
    frame[len_without_crc]     = (uint8_t)(crc & 0xFF);    // Modbus sends the CRC low byte first
    frame[len_without_crc + 1] = (uint8_t)(crc >> 8);
}

static bool crc_ok(const uint8_t *frame, size_t len)
{
    const uint16_t rx_crc = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    return rx_crc == modbus_crc16(frame, len - 2);
}

/**
 * One request/response exchange, serialized against every other.
 *
 * Checks the slave address, the function code and the CRC. A Modbus exception
 * reply is shorter than any normal one, so it arrives as a short read after the
 * timeout and is recognised and reported by its code here rather than as a
 * bare timeout.
 */
static esp_err_t transact(const uint8_t *req, size_t req_len, uint8_t *rsp, size_t rsp_len)
{
    if (!s_bus_lock) {
        s_bus_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_bus_lock, ESP_ERR_NO_MEM, TAG, "bus lock create failed");
    }

    xSemaphoreTake(s_bus_lock, portMAX_DELAY);

    // Drop any stale bytes so a late reply from a previous poll is not mistaken
    // for this one.
    uint8_t discard[16];
    while (uart_read(SENSEAIR_PORT, discard, sizeof(discard), 0) > 0) {
        ;
    }

    esp_err_t err = ESP_OK;
    int got = 0;
    if (uart_write(SENSEAIR_PORT, req, req_len) != (int)req_len) {
        err = ESP_FAIL;
    } else {
        got = uart_read(SENSEAIR_PORT, rsp, rsp_len, RESPONSE_TIMEOUT_MS);
    }

    xSemaphoreGive(s_bus_lock);

    ESP_RETURN_ON_ERROR(err, TAG, "request write failed");

    const uint8_t slave = req[0];
    const uint8_t func  = req[1];

    if (got == EXCEPTION_LEN && rsp[0] == slave && rsp[1] == (func | FUNC_EXCEPTION_FLAG) &&
        crc_ok(rsp, EXCEPTION_LEN)) {
        // 0x02 illegal address, 0x03 illegal value, per the Modbus spec.
        ESP_LOGW(TAG, "slave 0x%02X: exception 0x%02X on func 0x%02X", slave, rsp[2], func);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_RETURN_ON_FALSE(got == (int)rsp_len, ESP_ERR_TIMEOUT, TAG,
                        "short response (%d of %u bytes)", got, (unsigned)rsp_len);
    ESP_RETURN_ON_FALSE(rsp[0] == slave, ESP_ERR_INVALID_RESPONSE, TAG,
                        "wrong slave 0x%02X", rsp[0]);
    ESP_RETURN_ON_FALSE(rsp[1] == func, ESP_ERR_INVALID_RESPONSE, TAG,
                        "unexpected func 0x%02X", rsp[1]);
    ESP_RETURN_ON_FALSE(crc_ok(rsp, rsp_len), ESP_ERR_INVALID_CRC, TAG, "crc mismatch");
    return ESP_OK;
}

static esp_err_t read_reg(uint8_t slave, uint8_t func, uint16_t reg, uint16_t *value)
{
    ESP_RETURN_ON_FALSE(value, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    uint8_t req[READ_REQUEST_LEN] = {
        slave,
        func,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        0x00, 0x01,             // read one register
    };
    put_crc(req, READ_REQUEST_LEN - 2);

    uint8_t rsp[READ_RESPONSE_LEN];
    ESP_RETURN_ON_ERROR(transact(req, sizeof(req), rsp, sizeof(rsp)), TAG,
                        "read 0x%04X failed", reg);
    ESP_RETURN_ON_FALSE(rsp[2] == 2, ESP_ERR_INVALID_RESPONSE, TAG,
                        "unexpected byte count %u", rsp[2]);

    *value = ((uint16_t)rsp[3] << 8) | rsp[4];
    return ESP_OK;
}

esp_err_t senseair_modbus_read_input_reg(uint8_t slave, uint16_t reg, uint16_t *value)
{
    return read_reg(slave, FUNC_READ_INPUT_REG, reg, value);
}

esp_err_t senseair_modbus_read_holding_reg(uint8_t slave, uint16_t reg, uint16_t *value)
{
    return read_reg(slave, FUNC_READ_HOLDING_REG, reg, value);
}

esp_err_t senseair_modbus_write_holding_reg(uint8_t slave, uint16_t reg, uint16_t value)
{
    // The S12 has no single-register write (0x06); 0x10 with a count of one.
    uint8_t req[WRITE_REQUEST_LEN] = {
        slave,
        FUNC_WRITE_MULTIPLE,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        0x00, 0x01,             // one register
        0x02,                   // two data bytes
        (uint8_t)(value >> 8), (uint8_t)(value & 0xFF),
    };
    put_crc(req, WRITE_REQUEST_LEN - 2);

    uint8_t rsp[WRITE_RESPONSE_LEN];
    ESP_RETURN_ON_ERROR(transact(req, sizeof(req), rsp, sizeof(rsp)), TAG,
                        "write 0x%04X failed", reg);

    // The reply echoes the start address and count.
    ESP_RETURN_ON_FALSE(memcmp(&rsp[2], &req[2], 4) == 0, ESP_ERR_INVALID_RESPONSE, TAG,
                        "write 0x%04X: echo mismatch", reg);
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

esp_err_t senseair_modbus_read_info(uint8_t slave, co2_info_t *info)
{
    ESP_RETURN_ON_FALSE(info, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");
    memset(info, 0, sizeof(*info));

    uint16_t temp = 0, mode = 0;
    ESP_RETURN_ON_ERROR(senseair_modbus_read_input_reg(slave, SENSEAIR_REG_STATUS,
                                                       &info->error_status), TAG, "status");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_input_reg(slave, SENSEAIR_REG_TEMPERATURE, &temp),
                        TAG, "temperature");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_input_reg(slave, SENSEAIR_REG_FW_REV,
                                                       &info->fw_rev), TAG, "fw rev");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_holding_reg(slave, SENSEAIR_HR_CAL_STATUS,
                                                         &info->cal_status), TAG, "cal status");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_holding_reg(slave, SENSEAIR_HR_METER_CONTROL,
                                                         &info->meter_control), TAG, "meter ctl");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_holding_reg(slave, SENSEAIR_HR_ABC_PERIOD,
                                                         &info->abc_period_h), TAG, "abc period");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_holding_reg(slave, SENSEAIR_HR_ABC_TARGET,
                                                         &info->abc_target_ppm), TAG, "abc target");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_holding_reg(slave, SENSEAIR_HR_ABC_TIME,
                                                         &info->abc_time_h), TAG, "abc time");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_holding_reg(slave, SENSEAIR_HR_MEAS_MODE, &mode),
                        TAG, "meas mode");
    ESP_RETURN_ON_ERROR(senseair_modbus_read_holding_reg(slave, SENSEAIR_HR_MEAS_PERIOD,
                                                         &info->meas_period_s), TAG, "meas period");

    info->temperature_c100 = (int16_t)temp;
    info->single_mode      = (mode & 0xFF) == 1;
    info->meas_period_s   &= 0x07FF;
    if (info->meas_period_s == 0) {
        info->meas_period_s = 1;    // table 2.3: 0 runs at the 1 s minimum
    }

    // ABC runs only with a period of 1..65534 h *and* the meter control bit clear.
    info->abc_enabled = !(info->meter_control & SENSEAIR_METER_ABC_DISABLE) &&
                        info->abc_period_h != 0 && info->abc_period_h != 0xFFFF;
    return ESP_OK;
}

esp_err_t senseair_modbus_calibrate(uint8_t slave, co2_cal_t kind, uint16_t target_ppm)
{
    static const struct {
        uint16_t    command;
        uint16_t    done_bit;
        const char *name;
    } CAL[] = {
        [CO2_CAL_BACKGROUND]      = { 0x7C06, CO2_CAL_DONE_BACKGROUND, "background" },
        [CO2_CAL_TARGET]          = { 0x7C05, CO2_CAL_DONE_TARGET,     "target" },
        [CO2_CAL_ZERO]            = { 0x7C07, CO2_CAL_DONE_ZERO,       "zero" },
        [CO2_CAL_FORCED_ABC]      = { 0x7C03, CO2_CAL_DONE_ABC,        "forced ABC" },
        [CO2_CAL_FACTORY_RESTORE] = { 0x7C02, CO2_CAL_DONE_FACTORY,    "factory restore" },
    };
    ESP_RETURN_ON_FALSE((unsigned)kind < sizeof(CAL) / sizeof(CAL[0]), ESP_ERR_INVALID_ARG,
                        TAG, "unknown calibration %d", (int)kind);

    co2_info_t info;
    ESP_RETURN_ON_ERROR(senseair_modbus_read_info(slave, &info), TAG, "pre-calibration read");

    /*
     * Refuse the two commands that would otherwise be accepted and then do
     * nothing, leaving the caller to time out with no idea why.
     */
    if (kind == CO2_CAL_TARGET) {
        ESP_RETURN_ON_FALSE(target_ppm > 0 && target_ppm <= 0x7FFF, ESP_ERR_INVALID_ARG, TAG,
                            "target %u ppm out of range", target_ppm);
    } else if (kind == CO2_CAL_BACKGROUND) {
        ESP_RETURN_ON_FALSE(info.abc_target_ppm != 0, ESP_ERR_INVALID_STATE, TAG,
                            "ABC target is 0; use a target calibration instead");
        target_ppm = info.abc_target_ppm;
    } else if (kind == CO2_CAL_FORCED_ABC) {
        ESP_RETURN_ON_FALSE(info.abc_enabled, ESP_ERR_INVALID_STATE, TAG,
                            "forced ABC needs ABC enabled");
    }

    ESP_LOGI(TAG, "slave 0x%02X: %s calibration%s, target %u ppm, period %u s",
             slave, CAL[kind].name, info.single_mode ? " (single mode)" : "",
             target_ppm, info.meas_period_s);

    /*
     * Clear both status words first, as the datasheet recommends, so the done
     * bit and the ErrorStatus calibration bit polled below can only have been
     * set by this calibration.
     */
    ESP_RETURN_ON_ERROR(senseair_modbus_write_holding_reg(slave, SENSEAIR_HR_CAL_STATUS, 0),
                        TAG, "clear calibration status");
    ESP_RETURN_ON_ERROR(senseair_modbus_write_holding_reg(slave, SENSEAIR_HR_CLEAR_ERROR, 1),
                        TAG, "clear ErrorStatus");
    if (kind == CO2_CAL_TARGET) {
        ESP_RETURN_ON_ERROR(senseair_modbus_write_holding_reg(slave, SENSEAIR_HR_CAL_TARGET,
                                                              target_ppm), TAG, "target");
    }
    ESP_RETURN_ON_ERROR(senseair_modbus_write_holding_reg(slave, SENSEAIR_HR_CAL_COMMAND,
                                                          CAL[kind].command), TAG, "command");

    // The calibration rides on the next measurement; in single mode we take it.
    if (info.single_mode) {
        ESP_RETURN_ON_ERROR(senseair_modbus_write_holding_reg(slave, SENSEAIR_HR_START_SINGLE, 1),
                            TAG, "start measurement");
    }

    const TickType_t deadline = xTaskGetTickCount() +
                                pdMS_TO_TICKS((uint32_t)info.meas_period_s * 1000 + CAL_SLACK_MS);
    do {
        vTaskDelay(pdMS_TO_TICKS(CAL_POLL_MS));

        uint16_t cal = 0, status = 0;
        if (senseair_modbus_read_holding_reg(slave, SENSEAIR_HR_CAL_STATUS, &cal) == ESP_OK &&
            (cal & CAL[kind].done_bit)) {
            ESP_LOGI(TAG, "slave 0x%02X: %s calibration done, status 0x%04X",
                     slave, CAL[kind].name, cal);
            return ESP_OK;
        }
        if (senseair_modbus_read_input_reg(slave, SENSEAIR_REG_STATUS, &status) == ESP_OK &&
            (status & SENSEAIR_ERR_CALIBRATION)) {
            ESP_LOGE(TAG, "slave 0x%02X: %s calibration failed, ErrorStatus 0x%04X",
                     slave, CAL[kind].name, status);
            return ESP_FAIL;
        }
    } while ((int32_t)(deadline - xTaskGetTickCount()) > 0);

    ESP_LOGE(TAG, "slave 0x%02X: %s calibration not confirmed in time", slave, CAL[kind].name);
    return ESP_ERR_TIMEOUT;
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
