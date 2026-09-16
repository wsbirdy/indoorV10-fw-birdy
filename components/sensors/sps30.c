#include <string.h>
#include "pm_drivers.h"
#include "i2c.h"
#include "sensirion_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "sps30";

/*
 * Sensirion SPS30 over I2C, per the datasheet (version 2.0 - D1, June 2023),
 * section 6.
 *
 * The part is rated for standard mode only, 100 kbit/s, while this board's bus
 * runs at 400 kHz for everything else -- hence the per-device clock. It also
 * does not use clock stretching.
 *
 * Note for the hardware side: the SEL pin must be tied to GND before or as the
 * sensor powers up, or it comes up in UART mode and never answers here.
 */
#define SPS30_I2C_ADDR          0x69
#define SPS30_I2C_SPEED_HZ      100000

/* Command pointers (datasheet table 8). */
#define CMD_START_MEASUREMENT   0x0010
#define CMD_STOP_MEASUREMENT    0x0104
#define CMD_READ_DATA_READY     0x0202
#define CMD_READ_MEASURED       0x0300
#define CMD_READ_PRODUCT_TYPE   0xD002

/* Argument to Start Measurement: big-endian IEEE754 floats. */
#define OUTPUT_FORMAT_FLOAT     0x03

/*
 * Product type is documented as always "00080000" and explicitly recommended
 * as the product identifier, which makes it the probe.
 */
#define PRODUCT_TYPE_SPS30      "00080000"
#define PRODUCT_TYPE_LEN        8

/*
 * Ten big-endian floats, each sent as two CRC-protected words: 4 data bytes
 * and 2 CRC bytes per value.
 */
#define MEASURED_VALUE_COUNT    10
#define MEASURED_RAW_LEN        (MEASURED_VALUE_COUNT * 6)

#define I2C_TIMEOUT_MS          100
#define START_MEASUREMENT_MS    20      // datasheet max execution time
#define POLL_INTERVAL_MS        100

static i2c_master_dev_handle_t s_dev;
static bool s_measuring;

/*
 * The datasheet sequences a read as write-pointer, STOP, then START and read,
 * rather than a repeated start, so the pointer write and the data read are
 * issued as two separate transactions here.
 */
static esp_err_t send_command(uint16_t cmd)
{
    const uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_master_transmit(s_dev, tx, sizeof(tx), I2C_TIMEOUT_MS);
}

static esp_err_t read_response(uint16_t cmd, uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_ERROR(send_command(cmd), TAG, "pointer write failed");
    return i2c_master_receive(s_dev, buf, len, I2C_TIMEOUT_MS);
}

/**
 * @brief Strip the per-word CRCs from a response, checking each one.
 *
 * @param raw   groups of [MSB, LSB, CRC]
 * @param words how many such groups
 * @param out   receives @p words * 2 bytes of payload
 */
static esp_err_t unpack_words(const uint8_t *raw, size_t words, uint8_t *out)
{
    for (size_t i = 0; i < words; i++) {
        const uint8_t *group = &raw[i * 3];
        ESP_RETURN_ON_FALSE(sensirion_crc8_ok(group), ESP_ERR_INVALID_CRC, TAG,
                            "crc mismatch in word %u", (unsigned)i);
        out[i * 2]     = group[0];
        out[i * 2 + 1] = group[1];
    }
    return ESP_OK;
}

static float be_float(const uint8_t *p)
{
    const uint32_t bits = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                          ((uint32_t)p[2] << 8)  | p[3];
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static esp_err_t read_product_type(char *out, size_t out_len)
{
    // 8 characters arrive as 4 CRC-protected words.
    uint8_t raw[(PRODUCT_TYPE_LEN / 2) * 3];
    uint8_t chars[PRODUCT_TYPE_LEN];

    ESP_RETURN_ON_ERROR(read_response(CMD_READ_PRODUCT_TYPE, raw, sizeof(raw)), TAG,
                        "product type read failed");
    ESP_RETURN_ON_ERROR(unpack_words(raw, PRODUCT_TYPE_LEN / 2, chars), TAG,
                        "product type corrupt");

    ESP_RETURN_ON_FALSE(out_len > PRODUCT_TYPE_LEN, ESP_ERR_INVALID_SIZE, TAG, "buffer small");
    memcpy(out, chars, PRODUCT_TYPE_LEN);
    out[PRODUCT_TYPE_LEN] = '\0';
    return ESP_OK;
}

static esp_err_t start_measurement(void)
{
    const uint8_t args[2] = { OUTPUT_FORMAT_FLOAT, 0x00 };
    const uint8_t tx[5] = {
        (uint8_t)(CMD_START_MEASUREMENT >> 8), (uint8_t)(CMD_START_MEASUREMENT & 0xFF),
        args[0], args[1], sensirion_crc8(args, sizeof(args)),
    };

    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_dev, tx, sizeof(tx), I2C_TIMEOUT_MS), TAG,
                        "start measurement failed");
    vTaskDelay(pdMS_TO_TICKS(START_MEASUREMENT_MS));

    s_measuring = true;
    return ESP_OK;
}

static esp_err_t data_ready(bool *ready)
{
    uint8_t raw[3];
    uint8_t flag[2];

    ESP_RETURN_ON_ERROR(read_response(CMD_READ_DATA_READY, raw, sizeof(raw)), TAG,
                        "data-ready read failed");
    ESP_RETURN_ON_ERROR(unpack_words(raw, 1, flag), TAG, "data-ready corrupt");

    *ready = (flag[1] == 0x01);
    return ESP_OK;
}

static esp_err_t sps30_probe(void)
{
    if (s_dev == NULL) {
        ESP_RETURN_ON_ERROR(i2c_add_device_at_speed(SPS30_I2C_ADDR, SPS30_I2C_SPEED_HZ, &s_dev),
                            TAG, "attach failed");
    }

    char product[PRODUCT_TYPE_LEN + 1];
    ESP_RETURN_ON_ERROR(read_product_type(product, sizeof(product)), TAG, "no response");

    ESP_RETURN_ON_FALSE(strcmp(product, PRODUCT_TYPE_SPS30) == 0, ESP_ERR_NOT_FOUND, TAG,
                        "product type \"%s\" is not an SPS30", product);

    // The part powers up idle; nothing is measured until it is told to start.
    ESP_RETURN_ON_ERROR(start_measurement(), TAG, "could not start measuring");

    ESP_LOGI(TAG, "ready at 0x%02X (product type %s)", SPS30_I2C_ADDR, product);
    return ESP_OK;
}

static esp_err_t sps30_read(pm_data_t *out, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_dev && s_measuring, ESP_ERR_INVALID_STATE, TAG, "not measuring");

    /*
     * New values land once a second. Poll the ready flag rather than reading
     * blind so a caller gets a fresh sample instead of the previous one again.
     */
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    bool ready = false;

    while (!ready) {
        ESP_RETURN_ON_ERROR(data_ready(&ready), TAG, "data-ready poll failed");
        if (ready) {
            break;
        }
        if (esp_timer_get_time() >= deadline_us) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }

    uint8_t raw[MEASURED_RAW_LEN];
    uint8_t values[MEASURED_VALUE_COUNT * 4];

    ESP_RETURN_ON_ERROR(read_response(CMD_READ_MEASURED, raw, sizeof(raw)), TAG,
                        "measured values read failed");
    ESP_RETURN_ON_ERROR(unpack_words(raw, MEASURED_RAW_LEN / 3, values), TAG,
                        "measured values corrupt");

    /*
     * Mass concentrations first, then number concentrations, then typical
     * particle size (datasheet section 4.3). Only the mass bins and the size
     * are carried up; the per-bin counts have no consumer on this board.
     */
    out->pm1_0_ugm3      = be_float(&values[0]);
    out->pm2_5_ugm3      = be_float(&values[4]);
    out->pm4_0_ugm3      = be_float(&values[8]);
    out->pm10_ugm3       = be_float(&values[12]);
    out->typical_size_um = be_float(&values[36]);

    out->fields = PM_FIELD_PM1_0 | PM_FIELD_PM2_5 | PM_FIELD_PM4_0 |
                  PM_FIELD_PM10  | PM_FIELD_TYPICAL_SIZE;
    return ESP_OK;
}

const pm_driver_t sps30_pm_driver = {
    .name  = "Sensirion SPS30",
    .part  = PM_PART_SPS30,
    .probe = sps30_probe,
    .read  = sps30_read,
};
