#include "cap1203.h"
#include "i2c.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "cap1203";

#define REG_MAIN_CONTROL        0x00
#define REG_GENERAL_STATUS      0x02
#define REG_SENSOR_INPUT_STATUS 0x03
#define REG_NOISE_FLAG_STATUS   0x0A
#define REG_DELTA_COUNT_1       0x10    // ..0x12, one per pad, two's complement
#define REG_SENSITIVITY         0x1F
#define REG_SENSOR_ENABLE       0x21
#define REG_CALIBRATION_STATUS  0x26
#define REG_THRESHOLD_1         0x30    // ..0x32, one per pad
#define REG_NOISE_THRESHOLD     0x38
#define REG_BASE_COUNT_1        0x50    // ..0x52, one per pad
#define REG_PRODUCT_ID          0xFD
#define REG_MANUFACTURER_ID     0xFE

#define PRODUCT_ID_CAP1203      0x6D
#define MANUFACTURER_ID_MCHP    0x5D

#define MAIN_CONTROL_INT        0x01

#define I2C_TIMEOUT_MS          100

/* DELTA_SENSE 000b: 128x, the most sensitive setting (datasheet table 5-7). */
#define SENSITIVITY_HIGHEST     0

static i2c_master_dev_handle_t s_dev;

static esp_err_t reg_write(uint8_t reg, uint8_t value)
{
    const uint8_t tx[2] = { reg, value };
    return i2c_master_transmit(s_dev, tx, sizeof(tx), I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, value, 1, I2C_TIMEOUT_MS);
}


/* ======================================================================
 * Register dump -- development aid, off by default
 *
 * Flip CAP1203_DEBUG_DUMP to 1 to watch the pads react to whatever is near
 * them. It goes to its own log tag so the stream can be isolated or silenced
 * without touching anything else:
 *
 *     idf.py monitor --print-filter "cap1203.dbg:I *:W"     # only this
 *     esp_log_level_set("cap1203.dbg", ESP_LOG_NONE);       # mute at runtime
 *
 * At 0 the periodic dump compiles out entirely and costs nothing.
 * ====================================================================== */

#define CAP1203_DEBUG_DUMP      1

/*
 * One line per interval, not one per poll: the button task runs at 50 ms and
 * dumping at that rate would bury every other log on the console. A second is
 * slow enough to read as it scrolls and still fast enough to see a hand
 * approach the panel.
 */
#define CAP1203_DUMP_PERIOD_MS  1000

/*
 * Mute everything else while the dump runs. report_task alone puts eight lines
 * on the console every five seconds, which is what makes a 1 Hz stream hard to
 * read. Setting the wildcard tag drops the default level *and* discards every
 * per-tag override, so it also undoes the DEBUG levels wifi_sta and cloud set
 * for themselves -- which is why it has to happen after their init, not from
 * cap1203_init(). See QUIET_AFTER_MS.
 *
 * Off by default, and it should stay off unless someone is actively watching
 * the pads: QUIET_AFTER_MS lands in the middle of startup, so turning it on
 * blanks the wifi, cloud and display init lines and leaves a console that
 * looks like the boot hung. Prefer the monitor-side filter in the comment
 * above, which mutes nothing on the device.
 */
#define CAP1203_DEBUG_QUIET     0

/*
 * Applied on the first dump past this mark. wifi_sta_init() and cloud_start()
 * raise their own tags during startup; muting before they run would simply be
 * overwritten by them.
 */
#define QUIET_AFTER_MS          5000

/* Repeat the column header this often, so a scrolled-past screen stays readable. */
#define DUMP_HEADER_EVERY       20

static const char *DBG_TAG = "cap1203.dbg";

static uint32_t now_ms(void);   // defined with the button task, below

/*
 * Names and addresses follow the CAP1203 datasheet's register map. Addresses
 * it leaves undefined are not listed: they read back as 0 and would say
 * nothing.
 */
static const struct {
    uint8_t     addr;
    const char *name;
} REG_MAP[] = {
    { 0x00, "Main Control" },
    { 0x02, "General Status" },
    { 0x03, "Sensor Input Status" },
    { 0x0A, "Noise Flag Status" },
    { 0x10, "CS1 Delta Count" },
    { 0x11, "CS2 Delta Count" },
    { 0x12, "CS3 Delta Count" },
    { 0x1F, "Sensitivity Control" },
    { 0x20, "Configuration" },
    { 0x21, "Sensor Input Enable" },
    { 0x22, "Sensor Input Config" },
    { 0x23, "Sensor Input Config 2" },
    { 0x24, "Averaging and Sample Config" },
    { 0x26, "Calibration Activate/Status" },
    { 0x27, "Interrupt Enable" },
    { 0x28, "Repeat Rate Enable" },
    { 0x2A, "Multiple Touch Config" },
    { 0x2B, "Multiple Touch Pattern Config" },
    { 0x2D, "Multiple Touch Pattern" },
    { 0x2E, "Base Count Out" },
    { 0x2F, "Recalibration Config" },
    { 0x30, "CS1 Threshold" },
    { 0x31, "CS2 Threshold" },
    { 0x32, "CS3 Threshold" },
    { 0x38, "Sensor Input Noise Threshold" },
    { 0x40, "Standby Channel" },
    { 0x41, "Standby Config" },
    { 0x42, "Standby Sensitivity" },
    { 0x43, "Standby Threshold" },
    { 0x44, "Configuration 2" },
    { 0x50, "CS1 Base Count" },
    { 0x51, "CS2 Base Count" },
    { 0x52, "CS3 Base Count" },
    { 0x60, "Power Button" },
    { 0x61, "Power Button Config" },
    { 0xB1, "CS1 Calibration" },
    { 0xB2, "CS2 Calibration" },
    { 0xB3, "CS3 Calibration" },
    { 0xB9, "Calibration LSB 1" },
    { 0xFD, "Product ID" },
    { 0xFE, "Manufacturer ID" },
    { 0xFF, "Revision" },
};

esp_err_t cap1203_dump_registers(void)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    ESP_LOGI(DBG_TAG, "---- CAP1203 registers ----");
    for (size_t i = 0; i < sizeof(REG_MAP) / sizeof(REG_MAP[0]); i++) {
        uint8_t value = 0;
        const esp_err_t err = reg_read(REG_MAP[i].addr, &value);
        if (err != ESP_OK) {
            /*
             * Give up on the first failure rather than carrying on. This runs
             * inside cap1203_init(), which runs inside app_main: a part that
             * answers its ID and then stops, or a bus that wedges mid-dump,
             * would otherwise cost forty-odd I2C_TIMEOUT_MS waits in a row and
             * hold up the rest of startup for seconds. One register is enough
             * to know the dump is not going to work.
             */
            ESP_LOGW(DBG_TAG, "0x%02X %-30s   read failed: %s -- dump abandoned",
                     REG_MAP[i].addr, REG_MAP[i].name, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(DBG_TAG, "0x%02X %-30s = 0x%02X  %3u", REG_MAP[i].addr, REG_MAP[i].name,
                 value, value);
    }
    return ESP_OK;
}

#if CAP1203_DEBUG_DUMP
/*
 * Thresholds, gain and the noise threshold are configuration: they do not move
 * once init has run. Printing them on every row was most of what made the line
 * too wide to read, so they go in a banner above the columns instead.
 */
static void dump_banner(void)
{
    uint8_t thresh[3] = { 0 }, noise_thresh = 0, sensitivity = 0;

    for (int i = 0; i < 3; i++) {
        reg_read((uint8_t)(REG_THRESHOLD_1 + i), &thresh[i]);
    }
    reg_read(REG_NOISE_THRESHOLD, &noise_thresh);
    reg_read(REG_SENSITIVITY, &sensitivity);

    ESP_LOGI(DBG_TAG, "thresholds %u/%u/%u   noise thr %u   DELTA_SENSE %u   "
                      "(a pad reads as touched once its delta reaches its threshold)",
             thresh[0], thresh[1], thresh[2], noise_thresh,
             (unsigned)((sensitivity >> 4) & 0x07));
    ESP_LOGI(DBG_TAG, "  d1   d2   d3 |  b1  b2  b3 | touch noise  gen  cal");
}

/*
 * One row per interval. delta is the live movement away from base, and base is
 * the slowly-recalibrated reference it is measured against -- watching the two
 * together is what separates "a hand approached" from "the reference drifted".
 * All three pads on one row makes crosstalk between neighbours visible.
 */
static void dump_live(void)
{
    uint8_t delta[3] = { 0 }, base[3] = { 0 };
    uint8_t status = 0, noise = 0, general = 0, calib = 0;

    for (int i = 0; i < 3; i++) {
        reg_read((uint8_t)(REG_DELTA_COUNT_1 + i), &delta[i]);
        reg_read((uint8_t)(REG_BASE_COUNT_1 + i), &base[i]);
    }
    reg_read(REG_SENSOR_INPUT_STATUS, &status);
    reg_read(REG_NOISE_FLAG_STATUS, &noise);
    reg_read(REG_GENERAL_STATUS, &general);
    reg_read(REG_CALIBRATION_STATUS, &calib);

    // Delta counts are two's complement, so they have to be read as signed or
    // a pad drifting below its base prints as ~250 instead of -6.
    ESP_LOGI(DBG_TAG, "%4d %4d %4d | %3u %3u %3u |  0x%02X  0x%02X 0x%02X 0x%02X",
             (int)(int8_t)delta[0], (int)(int8_t)delta[1], (int)(int8_t)delta[2],
             base[0], base[1], base[2],
             status, noise, general, calib);
}

/** Drives the banner cadence and the one-time muting of everything else. */
static void dump_tick(void)
{
    static unsigned row;

#if CAP1203_DEBUG_QUIET
    static bool quieted;
    if (!quieted && now_ms() > QUIET_AFTER_MS) {
        quieted = true;
        esp_log_level_set("*", ESP_LOG_WARN);
        esp_log_level_set(DBG_TAG, ESP_LOG_INFO);
        ESP_LOGW(DBG_TAG, "other logs muted to WARN for this dump "
                          "(CAP1203_DEBUG_QUIET in cap1203.c)");
        row = 0;    // banner first, now that the console is clear
    }
#endif

    if (row % DUMP_HEADER_EVERY == 0) {
        dump_banner();
    }
    row++;

    dump_live();
}
#endif

/*
 * Pass-through on success; on failure it unwinds the attach first.
 *
 * i2c_add_device() has to set s_dev before the probe, because the probe reads
 * through it -- but that means a failed probe would otherwise leave a live
 * handle for a part that was never identified. cap1203_init()'s early-out
 * reads that handle as "already initialized", so the next caller would be told
 * the driver is ready, and reg_read()/reg_write() would keep addressing a
 * device that is not there. Detaching keeps a failure a failure, and keeps a
 * later retry possible.
 */
static esp_err_t detach_on_error(esp_err_t err)
{
    if (err != ESP_OK && s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    return err;
}

/*
 * Bounded by construction: four short transfers, each capped at
 * I2C_TIMEOUT_MS, plus a register dump that abandons itself on the first bad
 * read. An absent or wedged part costs app_main a fraction of a second here,
 * never a hang -- and the caller is free to treat the error as non-fatal,
 * which is what main.c does.
 */
esp_err_t cap1203_init(void)
{
    if (s_dev) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2c_add_device(CAP1203_I2C_ADDR, &s_dev), TAG, "attach failed");

    uint8_t product_id = 0, manufacturer_id = 0;
    ESP_RETURN_ON_ERROR(detach_on_error(reg_read(REG_PRODUCT_ID, &product_id)), TAG,
                        "product id read failed");
    ESP_RETURN_ON_ERROR(detach_on_error(reg_read(REG_MANUFACTURER_ID, &manufacturer_id)), TAG,
                        "manufacturer id read failed");

    if (product_id != PRODUCT_ID_CAP1203 || manufacturer_id != MANUFACTURER_ID_MCHP) {
        ESP_LOGE(TAG, "unexpected ids 0x%02X/0x%02X", product_id, manufacturer_id);
        return detach_on_error(ESP_ERR_NOT_FOUND);
    }

    // CS1..CS3
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_SENSOR_ENABLE, 0x07)), TAG, "enable failed");

    // Maximum gain. The pads are read through the enclosure wall, so the
    // default setting needs a firmer press than the board is meant to want.
    ESP_RETURN_ON_ERROR(detach_on_error(cap1203_set_sensitivity(SENSITIVITY_HIGHEST)), TAG,
                        "sensitivity failed");

    ESP_LOGI(TAG, "ready at 0x%02X, DELTA_SENSE %d (128x)", CAP1203_I2C_ADDR, SENSITIVITY_HIGHEST);

#if CAP1203_DEBUG_DUMP
    // One full pass at boot gives the periodic lines below a reference to be
    // read against: thresholds, gain and averaging do not change afterwards.
    // Advisory only -- the part has already identified itself, so a dump that
    // cannot complete is a reason to skip the dump, not to fail init.
    cap1203_dump_registers();
#endif

    return ESP_OK;
}

esp_err_t cap1203_read(uint8_t *pads)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(pads, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_SENSOR_INPUT_STATUS, &status), TAG, "status read failed");
    *pads = status & 0x07;

    // Status stays latched until INT is cleared, so a released pad would otherwise stick.
    uint8_t main_control = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_MAIN_CONTROL, &main_control), TAG, "control read failed");
    if (main_control & MAIN_CONTROL_INT) {
        ESP_RETURN_ON_ERROR(reg_write(REG_MAIN_CONTROL, main_control & ~MAIN_CONTROL_INT), TAG,
                            "clear int failed");
    }

    return ESP_OK;
}

esp_err_t cap1203_set_sensitivity(uint8_t delta_sense)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(delta_sense <= 7, ESP_ERR_INVALID_ARG, TAG, "delta_sense must be 0..7");

    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_SENSITIVITY, &value), TAG, "sensitivity read failed");

    // DELTA_SENSE is bits [6:4], written as the datasheet numbers it: 000b is
    // 128x and 111b is 1x. BASE_SHIFT[3:0] is not ours to touch.
    value = (uint8_t)((value & ~0x70) | ((delta_sense & 0x07) << 4));

    return reg_write(REG_SENSITIVITY, value);
}

/* ======================================================================
 * Buttons
 * ====================================================================== */

#define POLL_PERIOD_MS      50      // fast enough that a tap never feels dropped
#define EVENT_QUEUE_DEPTH   8

/*
 * Backoff for a part that answers at init and stops answering later -- a
 * cracked joint, a brownout, a neighbour wedging the bus.
 *
 * At the normal rate a failing read costs up to three I2C_TIMEOUT_MS waits
 * every POLL_PERIOD_MS, from a task that sits above every sensor and the UI.
 * That is most of the shared bus spent on a part that is gone, and it lands
 * squarely on air_monitor's acquisition. After this many consecutive failures
 * the poll drops to POLL_RETRY_MS, which keeps the recovery path alive at a
 * cost the rest of the board can absorb. One good read restores the rate.
 */
#define POLL_FAIL_LIMIT     10
#define POLL_RETRY_MS       2000

static QueueHandle_t s_events;
static TaskHandle_t  s_task;

static volatile uint8_t  s_pads;
static volatile bool     s_pads_valid;
static volatile uint32_t s_pads_updated_ms;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void post(cap1203_button_t button)
{
    // Depth 8 against a UI that drains every frame: a full queue means the
    // consumer has stalled, and the stale press is the one worth losing.
    if (xQueueSend(s_events, &button, 0) != pdTRUE) {
        ESP_LOGD(TAG, "event queue full, dropped button %d", (int)button);
    }
}

static void buttons_task(void *arg)
{
    (void)arg;

    uint8_t previous = 0;
    unsigned consecutive_failures = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
#if CAP1203_DEBUG_DUMP
    uint32_t last_dump_ms = 0;
#endif

    while (1) {
        uint8_t pads = 0;
        const bool ok = (cap1203_read(&pads) == ESP_OK);

        if (!ok) {
            pads = 0;
            if (consecutive_failures < POLL_FAIL_LIMIT) {
                consecutive_failures++;
                if (consecutive_failures == POLL_FAIL_LIMIT) {
                    ESP_LOGW(TAG, "%d reads failed in a row, backing off to %d ms "
                                  "(pads stay dead until one succeeds)",
                             POLL_FAIL_LIMIT, POLL_RETRY_MS);
                }
            }
        } else if (consecutive_failures) {
            if (consecutive_failures >= POLL_FAIL_LIMIT) {
                ESP_LOGI(TAG, "pads responding again");
            }
            consecutive_failures = 0;
        }

        const bool backing_off = (consecutive_failures >= POLL_FAIL_LIMIT);

        s_pads = pads;
        s_pads_valid = ok;
        s_pads_updated_ms = now_ms();

        /*
         * Leading edge only. No extra debounce: the status register latches
         * until cap1203_read() clears the interrupt, so a tap shorter than the
         * poll period is still caught -- requiring two consecutive samples
         * would throw exactly those away.
         */
        const uint8_t pressed = (uint8_t)(pads & ~previous);
        previous = pads;

        if (pressed & CAP1203_PAD1) {
            post(CAP1203_BUTTON_LEFT);
        }
        if (pressed & CAP1203_PAD2) {
            post(CAP1203_BUTTON_ENTER);
        }
        if (pressed & CAP1203_PAD3) {
            post(CAP1203_BUTTON_RIGHT);
        }

#if CAP1203_DEBUG_DUMP
        /*
         * Rate-limited inside the poll loop rather than given a task of its
         * own: the registers are read over the same I2C device handle, and one
         * reader means no lock is needed to keep the two apart.
         *
         * Skipped entirely while backing off. A dump is fifteen more reads on
         * a bus that has already shown it cannot serve one, which is the last
         * thing to spend the backoff on.
         */
        if (!backing_off && now_ms() - last_dump_ms >= CAP1203_DUMP_PERIOD_MS) {
            last_dump_ms = now_ms();
            dump_tick();
        }
#endif

        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(backing_off ? POLL_RETRY_MS : POLL_PERIOD_MS));
    }
}

esp_err_t cap1203_buttons_start(void)
{
    if (s_task) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(cap1203_init(), TAG, "init failed");

    if (s_events == NULL) {
        s_events = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(cap1203_button_t));
        ESP_RETURN_ON_FALSE(s_events, ESP_ERR_NO_MEM, TAG, "event queue alloc failed");
    }

    // Priority above the acquisition tasks: the loop is short and a late poll
    // is a press the user has to repeat.
    ESP_RETURN_ON_FALSE(xTaskCreate(buttons_task, "cap1203", 3072, NULL, 6, &s_task) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task creation failed");

    return ESP_OK;
}

bool cap1203_button_wait(cap1203_button_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return false;
    }

    const TickType_t ticks =
        (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    /*
     * A NULL queue means cap1203_buttons_start() never got past probing the
     * part, so no press will ever arrive. Block anyway: this call is the only
     * pacing a wait-and-repaint loop has, and returning immediately turns that
     * loop into a busy spin that starves the idle task until the watchdog
     * reboots the board. Missing pads should cost the buttons, not the board.
     */
    if (s_events == NULL) {
        vTaskDelay(ticks);
        return false;
    }

    return xQueueReceive(s_events, out, ticks) == pdTRUE;
}

uint8_t cap1203_pads(void)
{
    return s_pads;
}

bool cap1203_pads_valid(void)
{
    return s_pads_valid;
}

uint32_t cap1203_pads_updated_ms(void)
{
    return s_pads_updated_ms;
}
