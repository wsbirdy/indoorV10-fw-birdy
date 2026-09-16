#include <math.h>
#include "spl07.h"
#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "spl07";

#define REG_PSR_B2      0x00
#define REG_TMP_B2      0x03
#define REG_PRS_CFG     0x06
#define REG_TMP_CFG     0x07
#define REG_MEAS_CFG    0x08
#define REG_CFG_REG     0x09
#define REG_RESET       0x0C
#define REG_PROD_ID     0x0D
#define REG_COEF        0x10

/* ID (0x0D) is REV_ID[7:4] | PROD_ID[3:0]; only the product nibble identifies the part. */
#define ID_PROD_MASK    0x0F
#define PROD_ID_SPL07   0x01
#define PROD_ID_SPL06   0x00

#define MEAS_CFG_COEF_RDY   0x80
#define MEAS_CFG_SENSOR_RDY 0x40
#define MEAS_CFG_TMP_EXT    0x08    // 0 = internal ASIC sensor, which the datasheet recommends
#define MEAS_CTRL_CONT_PT   0x07    // continuous pressure + temperature

/* 8x oversampling on both channels: no result shift needed, kP == kT. */
#define OSR_8X_CFG      0x03
#define RATE_4HZ_CFG    (0x02 << 4)
#define K_SCALE_8X      7864320.0f

#define I2C_TIMEOUT_MS  100

static i2c_master_dev_handle_t s_dev;

/* Calibration coefficients. c31/c40 exist on the SPL07-003 but not the SPL06-001. */
static struct {
    int16_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
    int16_t c31, c40;
} s_cal;

static esp_err_t reg_write(uint8_t reg, uint8_t value)
{
    const uint8_t tx[2] = { reg, value };
    return i2c_master_transmit(s_dev, tx, sizeof(tx), I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

/** Sign-extend @p value, which carries @p bits significant bits. */
static int32_t sign_extend(uint32_t value, uint8_t bits)
{
    const uint32_t sign_bit = 1u << (bits - 1);
    return (value & sign_bit) ? (int32_t)(value | ~((1u << bits) - 1u)) : (int32_t)value;
}

static esp_err_t read_calibration(void)
{
    uint8_t c[21]; // 0x10 .. 0x24
    ESP_RETURN_ON_ERROR(reg_read(REG_COEF, c, sizeof(c)), TAG, "coefficient read failed");

    s_cal.c0  = (int16_t)sign_extend(((uint32_t)c[0] << 4) | (c[1] >> 4), 12);
    s_cal.c1  = (int16_t)sign_extend((((uint32_t)c[1] & 0x0F) << 8) | c[2], 12);
    s_cal.c00 = sign_extend(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | (c[5] >> 4), 20);
    s_cal.c10 = sign_extend((((uint32_t)c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | c[7], 20);
    s_cal.c01 = (int16_t)(((uint16_t)c[8] << 8) | c[9]);
    s_cal.c11 = (int16_t)(((uint16_t)c[10] << 8) | c[11]);
    s_cal.c20 = (int16_t)(((uint16_t)c[12] << 8) | c[13]);
    s_cal.c21 = (int16_t)(((uint16_t)c[14] << 8) | c[15]);
    s_cal.c30 = (int16_t)(((uint16_t)c[16] << 8) | c[17]);
    s_cal.c31 = (int16_t)sign_extend(((uint32_t)c[18] << 4) | (c[19] >> 4), 12);
    s_cal.c40 = (int16_t)sign_extend((((uint32_t)c[19] & 0x0F) << 8) | c[20], 12);

    return ESP_OK;
}

esp_err_t spl07_init(void)
{
    if (s_dev) {
        return ESP_OK;
    }

    i2c_master_bus_handle_t bus = i2c_get_bus();
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_STATE, TAG, "i2c bus not initialized");

    // The address depends on how SDO is strapped, so try both rather than guess.
    uint8_t addr = SPL07_I2C_ADDR_PRIMARY;
    if (i2c_master_probe(bus, addr, I2C_TIMEOUT_MS) != ESP_OK) {
        addr = SPL07_I2C_ADDR_SECONDARY;
        ESP_RETURN_ON_ERROR(i2c_master_probe(bus, addr, I2C_TIMEOUT_MS), TAG,
                            "no response at 0x77 or 0x76");
    }
    ESP_RETURN_ON_ERROR(i2c_add_device(addr, &s_dev), TAG, "attach failed");

    ESP_RETURN_ON_ERROR(reg_write(REG_RESET, 0x89), TAG, "soft reset failed"); // flush FIFO + SOFT_RST=1001
    vTaskDelay(pdMS_TO_TICKS(50)); // datasheet: TCoef_rdy is 40 ms max after reset

    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_PROD_ID, &id, 1), TAG, "product id read failed");
    const uint8_t prod_id = id & ID_PROD_MASK;
    if (prod_id != PROD_ID_SPL07 && prod_id != PROD_ID_SPL06) {
        ESP_LOGW(TAG, "unexpected product id 0x%X (expected 0x%X)", prod_id, PROD_ID_SPL07);
    }

    /* Coefficients are only valid once COEF_RDY is set, and the datasheet asks that no
     * measurement be started before self-initialization completes (SENSOR_RDY). */
    const uint8_t ready = MEAS_CFG_COEF_RDY | MEAS_CFG_SENSOR_RDY;
    uint8_t status = 0;
    for (int i = 0; i < 20; i++) {
        ESP_RETURN_ON_ERROR(reg_read(REG_MEAS_CFG, &status, 1), TAG, "status read failed");
        if ((status & ready) == ready) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_RETURN_ON_FALSE((status & ready) == ready, ESP_ERR_TIMEOUT, TAG,
                        "sensor/coefficients not ready (MEAS_CFG=0x%02X)", status);

    ESP_RETURN_ON_ERROR(read_calibration(), TAG, "calibration failed");

    /* On the SPL07-003 both config registers are RATE[7:4] | PRC[3:0]; there is no
     * TMP_EXT bit here (it lives in MEAS_CFG bit 3, and internal is the recommended source). */
    ESP_RETURN_ON_ERROR(reg_write(REG_PRS_CFG, RATE_4HZ_CFG | OSR_8X_CFG), TAG, "prs cfg failed");
    ESP_RETURN_ON_ERROR(reg_write(REG_TMP_CFG, RATE_4HZ_CFG | OSR_8X_CFG), TAG, "tmp cfg failed");
    ESP_RETURN_ON_ERROR(reg_write(REG_CFG_REG, 0x00), TAG, "cfg failed"); // shift only needed above 8x
    ESP_RETURN_ON_ERROR(reg_write(REG_MEAS_CFG, MEAS_CTRL_CONT_PT), TAG, "meas cfg failed");

    ESP_LOGI(TAG, "ready at 0x%02X, id 0x%02X (c00=%ld c10=%ld)",
             addr, id, (long)s_cal.c00, (long)s_cal.c10);
    return ESP_OK;
}

esp_err_t spl07_read(float *pressure_pa, float *temp_c)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(pressure_pa && temp_c, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    uint8_t raw[6]; // PSR_B2..B0 then TMP_B2..B0
    ESP_RETURN_ON_ERROR(reg_read(REG_PSR_B2, raw, sizeof(raw)), TAG, "sample read failed");

    const int32_t p_raw =
        sign_extend(((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2], 24);
    const int32_t t_raw =
        sign_extend(((uint32_t)raw[3] << 16) | ((uint32_t)raw[4] << 8) | raw[5], 24);

    const float p_sc = (float)p_raw / K_SCALE_8X;
    const float t_sc = (float)t_raw / K_SCALE_8X;

    *temp_c = 0.5f * (float)s_cal.c0 + (float)s_cal.c1 * t_sc;

    // SPL07-003 compensation: 4th-order in pressure, with the c31/c21 cross terms.
    *pressure_pa =
        (float)s_cal.c00 +
        p_sc * ((float)s_cal.c10 +
                p_sc * ((float)s_cal.c20 +
                        p_sc * ((float)s_cal.c30 + p_sc * (float)s_cal.c40))) +
        t_sc * (float)s_cal.c01 +
        t_sc * p_sc * ((float)s_cal.c11 +
                       p_sc * ((float)s_cal.c21 + p_sc * (float)s_cal.c31));

    return ESP_OK;
}

float spl07_altitude_m(float pressure_pa, float sea_level_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1902949f));
}
