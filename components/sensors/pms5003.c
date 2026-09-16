#include "pm_drivers.h"
#include "uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "pms5003";

#define PMS_PORT            UART_ID_0
#define PMS_START_BYTE_1    0x42
#define PMS_START_BYTE_2    0x4D
#define PMS_FRAME_LEN       32      // 2 start + 2 length + 26 payload + 2 checksum
#define PMS_BODY_LEN        (PMS_FRAME_LEN - 2)
#define PMS_EXPECTED_LENGTH 28      // value carried in the length field

#define WARMUP_MS           30000   // fan/laser stabilisation before readings are meaningful

/*
 * The part free-runs in active mode with no identify command, so recognising
 * it means waiting for a well-formed frame. It emits one about every second;
 * this allows for a couple of missed frames right after power-up.
 */
#define PROBE_TIMEOUT_MS    2500

/** @brief One decoded 32-byte PMS5003 frame. */
typedef struct {
    uint16_t pm1_0_cf1;     // ug/m3, factory calibration environment
    uint16_t pm2_5_cf1;
    uint16_t pm10_cf1;
    uint16_t pm1_0_atm;     // ug/m3, atmospheric environment -- use these outdoors
    uint16_t pm2_5_atm;
    uint16_t pm10_atm;
    uint16_t particles_0_3; // count per 0.1 L of air, per size bin
    uint16_t particles_0_5;
    uint16_t particles_1_0;
    uint16_t particles_2_5;
    uint16_t particles_5_0;
    uint16_t particles_10;
} pms5003_frame_t;

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static esp_err_t read_frame(pms5003_frame_t *out, uint32_t timeout_ms)
{
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline_us) {
        const uint32_t remaining_ms =
            (uint32_t)((deadline_us - esp_timer_get_time()) / 1000) + 1;

        // Hunt for the two start bytes one at a time so a partial frame resyncs.
        uint8_t byte = 0;
        if (uart_read(PMS_PORT, &byte, 1, remaining_ms) != 1 || byte != PMS_START_BYTE_1) {
            continue;
        }
        if (uart_read(PMS_PORT, &byte, 1, 100) != 1 || byte != PMS_START_BYTE_2) {
            continue;
        }

        uint8_t body[PMS_BODY_LEN];
        if (uart_read(PMS_PORT, body, sizeof(body), 200) != (int)sizeof(body)) {
            ESP_LOGW(TAG, "short frame");
            continue;
        }

        const uint16_t frame_length = be16(&body[0]);
        if (frame_length != PMS_EXPECTED_LENGTH) {
            ESP_LOGW(TAG, "unexpected frame length %u", frame_length);
            continue;
        }

        // Checksum covers every byte before it, start bytes included.
        uint16_t sum = PMS_START_BYTE_1 + PMS_START_BYTE_2;
        for (size_t i = 0; i < PMS_BODY_LEN - 2; i++) {
            sum += body[i];
        }
        const uint16_t checksum = be16(&body[PMS_BODY_LEN - 2]);
        if (sum != checksum) {
            ESP_LOGW(TAG, "checksum mismatch (got 0x%04X, computed 0x%04X)", checksum, sum);
            continue;
        }

        const uint8_t *d = &body[2]; // first data word follows the length field
        out->pm1_0_cf1     = be16(&d[0]);
        out->pm2_5_cf1     = be16(&d[2]);
        out->pm10_cf1      = be16(&d[4]);
        out->pm1_0_atm     = be16(&d[6]);
        out->pm2_5_atm     = be16(&d[8]);
        out->pm10_atm      = be16(&d[10]);
        out->particles_0_3 = be16(&d[12]);
        out->particles_0_5 = be16(&d[14]);
        out->particles_1_0 = be16(&d[16]);
        out->particles_2_5 = be16(&d[18]);
        out->particles_5_0 = be16(&d[20]);
        out->particles_10  = be16(&d[22]);

        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t pms5003_probe(void)
{
    // The PMS5003 powers up in active mode and needs no configuration, so there is
    // nothing to send. Drain anything already buffered so the first read starts clean.
    uint8_t discard[64];
    while (uart_read(PMS_PORT, discard, sizeof(discard), 0) > 0) {
        ;
    }

    pms5003_frame_t frame;
    ESP_RETURN_ON_ERROR(read_frame(&frame, PROBE_TIMEOUT_MS), TAG, "no frame on uart%d",
                        (int)PMS_PORT);

    ESP_LOGI(TAG, "listening on uart%d (allow ~%d s warm-up for stable readings)",
             (int)PMS_PORT, WARMUP_MS / 1000);
    return ESP_OK;
}

static esp_err_t pms5003_read(pm_data_t *out, uint32_t timeout_ms)
{
    pms5003_frame_t frame;
    ESP_RETURN_ON_ERROR(read_frame(&frame, timeout_ms), TAG, "no frame");

    /*
     * The atmospheric-environment values are the ones to publish; the CF=1 set
     * is calibrated for a factory test chamber. Per-bin particle counts are
     * decoded but dropped here -- nothing on this board consumes them.
     */
    out->pm1_0_ugm3 = (float)frame.pm1_0_atm;
    out->pm2_5_ugm3 = (float)frame.pm2_5_atm;
    out->pm10_ugm3  = (float)frame.pm10_atm;

    // No PM4.0 bin and no particle-size output on this part.
    out->pm4_0_ugm3      = 0.0f;
    out->typical_size_um = 0.0f;

    out->fields = PM_FIELD_PM1_0 | PM_FIELD_PM2_5 | PM_FIELD_PM10;
    return ESP_OK;
}

const pm_driver_t pms5003_pm_driver = {
    .name  = "Plantower PMS5003",
    .part  = PM_PART_PMS5003,
    .probe = pms5003_probe,
    .read  = pms5003_read,
};
