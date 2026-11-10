#include "cap1133.h"
#include "i2c.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/*
 * Structured as cap1166.c minus the slider and the RESET line. Where a choice
 * here matches the CAP1166 driver, the reasoning written out there applies
 * unchanged; the comments below cover only what differs.
 */

static const char *TAG = "cap1133";

/* Register addresses, CAP1133 datasheet table 5.1. */
#define REG_MAIN_CONTROL        0x00
#define REG_SENSOR_INPUT_STATUS 0x03
#define REG_DELTA_COUNT_1       0x10    // ..0x12, one per pad, two's complement
#define REG_SENSITIVITY         0x1F
#define REG_SENSOR_ENABLE       0x21
#define REG_AVG_AND_SAMPLING    0x24
#define REG_INTERRUPT_ENABLE    0x27
#define REG_REPEAT_RATE_ENABLE  0x28
#define REG_MULTIPLE_TOUCH_CFG  0x2A
#define REG_THRESHOLD_1         0x30    // ..0x32, one per pad
#define REG_LED_LINKING         0x72
#define REG_LED_POLARITY        0x73
#define REG_LED_OUTPUT_CONTROL  0x74
#define REG_LED_DIRECT_DUTY     0x93
#define REG_PRODUCT_ID          0xFD
#define REG_MANUFACTURER_ID     0xFE
#define REG_REVISION            0xFF

#define PRODUCT_ID_CAP1133      0x54
#define PRODUCT_ID_CAP1203      0x6D    // same address, see cap1133_init()
#define MANUFACTURER_ID_MCHP    0x5D

#define MAIN_CONTROL_INT        0x01

#define I2C_TIMEOUT_MS          100

/* DELTA_SENSE 000b: 128x, the most sensitive setting (datasheet table 5.8). */
#define SENSITIVITY_HIGHEST     0

/*
 * Multiple touch: blocking on, one touch at a time (MULT_BLK_EN, B_MULT_T 00b).
 * The reset value, written because nothing resets this part on a soft reboot.
 * With no slider there is no reason for two pads to be down at once, and a
 * palm across all three should read as nothing rather than as a press.
 */
#define MULTIPLE_TOUCH_SINGLE   0x80

/* Lit while idle, dark under a finger. See LED_POLARITY_NON_INVERTED in cap1166.c. */
#define LED_POLARITY_NON_INVERTED   CAP1133_PADS_ALL

/* LED Direct Duty Cycle: max 100%, min 0% -- fully on and fully off. */
#define LED_DIRECT_DUTY_FULL        0xF0

/*
 * Averaging and sampling: 8 samples per channel on a 35 ms cycle
 * (AVG 011b, SAMP_TIME 10b = 1.28 ms, CYCLE_TIME 00b).
 *
 * The CAP1166 had to halve its averaging to fit a 35 ms cycle. With three
 * channels this part does not: 3 x 8 x 1.28 ms = 30.7 ms of sampling fits
 * inside 35 ms at the default averaging, so the cycle (and with it touch
 * latency) halves from the reset value's 70 ms at no cost in noise.
 */
#define AVG_AND_SAMPLING_FAST       0x38

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

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}


/* ======================================================================
 * Register dump -- development aid
 * ====================================================================== */

static const char *DBG_TAG = "cap1133.dbg";

/* Datasheet table 5.1, in order. */
static const struct {
    uint8_t     addr;
    const char *name;
} REG_MAP[] = {
    { 0x00, "Main Control" },
    { 0x02, "General Status" },
    { 0x03, "Sensor Input Status" },
    { 0x04, "LED Status" },
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
    { 0x26, "Calibration Activate" },
    { 0x27, "Interrupt Enable" },
    { 0x28, "Repeat Rate Enable" },
    { 0x2A, "Multiple Touch Config" },
    { 0x2B, "Multiple Touch Pattern Config" },
    { 0x2D, "Multiple Touch Pattern" },
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
    { 0x71, "LED Output Type" },
    { 0x72, "Sensor Input LED Linking" },
    { 0x73, "LED Polarity" },
    { 0x74, "LED Output Control" },
    { 0x77, "LED Linked Transition Control" },
    { 0x79, "LED Mirror Control" },
    { 0x81, "LED Behavior 1" },
    { 0x84, "LED Pulse 1 Period" },
    { 0x85, "LED Pulse 2 Period" },
    { 0x86, "LED Breathe Period" },
    { 0x88, "LED Config" },
    { 0x90, "LED Pulse 1 Duty Cycle" },
    { 0x91, "LED Pulse 2 Duty Cycle" },
    { 0x92, "LED Breathe Duty Cycle" },
    { 0x93, "LED Direct Duty Cycle" },
    { 0x94, "LED Direct Ramp Rates" },
    { 0x95, "LED Off Delay" },
    { 0xB1, "CS1 Calibration" },
    { 0xB2, "CS2 Calibration" },
    { 0xB3, "CS3 Calibration" },
    { 0xB9, "Calibration LSB 1" },
    { 0xFD, "Product ID" },
    { 0xFE, "Manufacturer ID" },
    { 0xFF, "Revision" },
};

esp_err_t cap1133_dump_registers(void)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    ESP_LOGI(DBG_TAG, "---- CAP1133 registers ----");
    for (size_t i = 0; i < sizeof(REG_MAP) / sizeof(REG_MAP[0]); i++) {
        uint8_t value = 0;
        const esp_err_t err = reg_read(REG_MAP[i].addr, &value);
        if (err != ESP_OK) {
            ESP_LOGW(DBG_TAG, "0x%02X %-30s   read failed: %s -- dump abandoned",
                     REG_MAP[i].addr, REG_MAP[i].name, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(DBG_TAG, "0x%02X %-30s = 0x%02X  %3u", REG_MAP[i].addr, REG_MAP[i].name,
                 value, value);
    }
    return ESP_OK;
}


/* ======================================================================
 * Setup
 * ====================================================================== */

/* Pass-through on success; on failure it unwinds the attach first. */
static esp_err_t detach_on_error(esp_err_t err)
{
    if (err != ESP_OK && s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    return err;
}

esp_err_t cap1133_init(void)
{
    if (s_dev) {
        return ESP_OK;
    }

    // No RESET pin on this part: power-on reset is the only one, and it is
    // long finished by the time app_main gets here.
    ESP_RETURN_ON_ERROR(i2c_add_device(CAP1133_I2C_ADDR, &s_dev), TAG, "attach failed");

    uint8_t product_id = 0, manufacturer_id = 0, revision = 0;
    ESP_RETURN_ON_ERROR(detach_on_error(reg_read(REG_PRODUCT_ID, &product_id)), TAG,
                        "product id read failed");
    ESP_RETURN_ON_ERROR(detach_on_error(reg_read(REG_MANUFACTURER_ID, &manufacturer_id)), TAG,
                        "manufacturer id read failed");

    if (product_id != PRODUCT_ID_CAP1133 || manufacturer_id != MANUFACTURER_ID_MCHP) {
        // The CAP1203 sits at the same fixed address, so a wrong-board build
        // lands here rather than on a NACK. Name it.
        if (product_id == PRODUCT_ID_CAP1203) {
            ESP_LOGE(TAG, "found a CAP1203 at 0x%02X, not a CAP1133 -- "
                          "set TOUCH_IC to TOUCH_IC_CAP1203 in touch.h", CAP1133_I2C_ADDR);
        } else {
            ESP_LOGE(TAG, "unexpected ids 0x%02X/0x%02X at 0x%02X", product_id,
                     manufacturer_id, CAP1133_I2C_ADDR);
        }
        return detach_on_error(ESP_ERR_NOT_FOUND);
    }

    (void)reg_read(REG_REVISION, &revision);    // logged below; not worth failing over

    /*
     * Everything below is written even where it is the reset value: nothing
     * resets this part except a power cycle, so a soft reboot inherits the
     * previous build's settings.
     */
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_SENSOR_ENABLE, CAP1133_PADS_ALL)), TAG,
                        "enable failed");

    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_MULTIPLE_TOUCH_CFG,
                                                  MULTIPLE_TOUCH_SINGLE)),
                        TAG, "multiple touch config failed");

    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_AVG_AND_SAMPLING, AVG_AND_SAMPLING_FAST)),
                        TAG, "sampling config failed");

    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_INTERRUPT_ENABLE, CAP1133_PADS_ALL)),
                        TAG, "interrupt enable failed");

    // No hardware auto-repeat: the task finds its own edges.
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_REPEAT_RATE_ENABLE, 0x00)), TAG,
                        "repeat rate config failed");

    ESP_RETURN_ON_ERROR(detach_on_error(cap1133_led_link(CAP1133_PADS_ALL)), TAG,
                        "led linking failed");

    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_LED_POLARITY,
                                                  LED_POLARITY_NON_INVERTED)),
                        TAG, "led polarity failed");

    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_LED_DIRECT_DUTY,
                                                  LED_DIRECT_DUTY_FULL)),
                        TAG, "led duty failed");

    ESP_RETURN_ON_ERROR(detach_on_error(cap1133_set_sensitivity(SENSITIVITY_HIGHEST)), TAG,
                        "sensitivity failed");

    ESP_LOGI(TAG, "ready at 0x%02X (rev 0x%02X), CS1-3 buttons, "
                  "3 LEDs linked and lit until touched, DELTA_SENSE %d (128x)",
             CAP1133_I2C_ADDR, revision, SENSITIVITY_HIGHEST);

    return ESP_OK;
}

esp_err_t cap1133_read(uint8_t *pads)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(pads, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_SENSOR_INPUT_STATUS, &status), TAG, "status read failed");
    *pads = status & CAP1133_PADS_ALL;

    // Status stays latched until INT is cleared. Read-modify-write, because
    // this register also holds GAIN[1:0] and the power-state bits.
    uint8_t main_control = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_MAIN_CONTROL, &main_control), TAG, "control read failed");
    if (main_control & MAIN_CONTROL_INT) {
        ESP_RETURN_ON_ERROR(reg_write(REG_MAIN_CONTROL, main_control & ~MAIN_CONTROL_INT), TAG,
                            "clear int failed");
    }

    return ESP_OK;
}

esp_err_t cap1133_read_deltas(int8_t delta[3])
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(delta, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    for (int i = 0; i < 3; i++) {
        uint8_t raw = 0;
        ESP_RETURN_ON_ERROR(reg_read((uint8_t)(REG_DELTA_COUNT_1 + i), &raw), TAG,
                            "delta %d read failed", i + 1);
        delta[i] = (int8_t)raw;     // two's complement, saturating at +127/-128
    }
    return ESP_OK;
}

esp_err_t cap1133_read_thresholds(uint8_t threshold[3])
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(threshold, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_ERROR(reg_read((uint8_t)(REG_THRESHOLD_1 + i), &threshold[i]), TAG,
                            "threshold %d read failed", i + 1);
    }
    return ESP_OK;
}

esp_err_t cap1133_set_sensitivity(uint8_t delta_sense)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(delta_sense <= 7, ESP_ERR_INVALID_ARG, TAG, "delta_sense must be 0..7");

    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_SENSITIVITY, &value), TAG, "sensitivity read failed");

    // DELTA_SENSE is bits [6:4]; BASE_SHIFT[3:0] is left alone.
    value = (uint8_t)((value & ~0x70) | ((delta_sense & 0x07) << 4));

    return reg_write(REG_SENSITIVITY, value);
}

esp_err_t cap1133_led_link(uint8_t pad_mask)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    return reg_write(REG_LED_LINKING, (uint8_t)(pad_mask & CAP1133_PADS_ALL));
}

esp_err_t cap1133_led_set(uint8_t led_mask)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    // Non-inverted polarity: an actuated output is a dark LED (table 5.48),
    // so "lit" in the argument is a clear bit in the register.
    return reg_write(REG_LED_OUTPUT_CONTROL, (uint8_t)(~led_mask & CAP1133_PADS_ALL));
}


/* ======================================================================
 * Polling task
 * ====================================================================== */

/*
 * While a pad is held, keep reading at the part's own cycle so the release is
 * seen promptly. Idle, ALERT# does the waking and this is only a backstop;
 * without ALERT# it is the response time.
 */
#define POLL_ACTIVE_MS      35
#if CAP1133_ALERT_GPIO >= 0
#define POLL_IDLE_MS        500
#else
#define POLL_IDLE_MS        50
#endif

#define EVENT_QUEUE_DEPTH   8

/* Backoff for a part that stops answering after init. */
#define POLL_FAIL_LIMIT     10
#define POLL_RETRY_MS       2000

static QueueHandle_t s_events;
static TaskHandle_t  s_task;

#if CAP1133_ALERT_GPIO >= 0
static void IRAM_ATTR alert_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static esp_err_t alert_attach(TaskHandle_t task)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CAP1133_ALERT_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,      // active low, open drain
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "alert gpio %d config failed",
                        CAP1133_ALERT_GPIO);

    const esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "gpio isr service install failed");
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CAP1133_ALERT_GPIO, alert_isr, task), TAG,
                        "alert isr attach failed");

    ESP_LOGI(TAG, "interrupt driven on gpio %d", CAP1133_ALERT_GPIO);
    return ESP_OK;
}
#endif

/* A line still low after the clear means an event landed mid read/clear. */
static bool alert_asserted(void)
{
#if CAP1133_ALERT_GPIO >= 0
    return gpio_get_level(CAP1133_ALERT_GPIO) == 0;
#else
    return false;
#endif
}

static void wait_for_activity(TickType_t ticks)
{
#if CAP1133_ALERT_GPIO >= 0
    ulTaskNotifyTake(pdTRUE, ticks);
#else
    vTaskDelay(ticks);
#endif
}

static volatile uint8_t  s_pads;
static volatile bool     s_pads_valid;
static volatile uint32_t s_pads_updated_ms;

/* Written by any task; acted on only by touch_task. */
static volatile uint32_t s_led_timeout_ms;      ///< 0 = never go dark
static volatile bool     s_led_timeout_changed;

/* Dark first, then unlink, so no unlinked LED shows a stale output bit. */
static esp_err_t leds_sleep(void)
{
    ESP_RETURN_ON_ERROR(cap1133_led_set(0), TAG, "led off failed");
    return cap1133_led_link(0);
}

static esp_err_t leds_wake(void)
{
    return cap1133_led_link(CAP1133_PADS_ALL);
}

static void post_event(cap1133_event_t event)
{
    if (xQueueSend(s_events, &event, 0) != pdTRUE) {
        ESP_LOGD(TAG, "event queue full, dropped event %d", (int)event);
    }
}

static void touch_task(void *arg)
{
    (void)arg;

    uint8_t  previous = 0;
    unsigned consecutive_failures = 0;

    bool     leds_asleep = false;
    bool     swallowing = false;    ///< the touch that woke the LEDs is still down
    uint32_t last_touch_ms = now_ms();

    while (1) {
        uint8_t pads = 0;
        const bool ok = (cap1133_read(&pads) == ESP_OK);

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

        // A new timeout setting restarts the clock; "never" wakes them now.
        if (s_led_timeout_changed) {
            s_led_timeout_changed = false;
            last_touch_ms = now_ms();
            if (s_led_timeout_ms == 0 && leds_asleep && leds_wake() == ESP_OK) {
                leds_asleep = false;
            }
        }

        if (pads != 0) {
            last_touch_ms = now_ms();

            // The touch that wakes a dark panel is spent on waking it.
            if (leds_asleep && leds_wake() == ESP_OK) {
                leds_asleep = false;
                swallowing = true;
                ESP_LOGD(TAG, "leds awake");
            }
        }

        if (swallowing) {
            previous = pads;
            if (pads != 0) {
                goto wait;
            }
            swallowing = false;
        }

        if (!leds_asleep && ok && s_led_timeout_ms != 0 &&
            now_ms() - last_touch_ms >= s_led_timeout_ms) {
            if (leds_sleep() == ESP_OK) {
                leds_asleep = true;
                ESP_LOGD(TAG, "leds asleep after %lu ms idle",
                         (unsigned long)s_led_timeout_ms);
            }
        }

        // Leading edge only. The status register latches until cleared, so a
        // tap shorter than the poll period is still caught.
        const uint8_t pressed = (uint8_t)(pads & ~previous);
        previous = pads;

        if (pressed & CAP1133_PAD1) {
            post_event(CAP1133_EVENT_LEFT);
        }
        if (pressed & CAP1133_PAD2) {
            post_event(CAP1133_EVENT_ENTER);
        }
        if (pressed & CAP1133_PAD3) {
            post_event(CAP1133_EVENT_RIGHT);
        }

wait:;
        uint32_t wait_ms;
        if (backing_off) {
            wait_ms = POLL_RETRY_MS;
        } else if (pads != 0 || alert_asserted()) {
            wait_ms = POLL_ACTIVE_MS;
        } else {
            wait_ms = POLL_IDLE_MS;
        }

        wait_for_activity(pdMS_TO_TICKS(wait_ms));
    }
}

esp_err_t cap1133_start(void)
{
    if (s_task) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(cap1133_init(), TAG, "init failed");

    if (s_events == NULL) {
        s_events = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(cap1133_event_t));
        ESP_RETURN_ON_FALSE(s_events, ESP_ERR_NO_MEM, TAG, "event queue alloc failed");
    }

    ESP_RETURN_ON_FALSE(xTaskCreate(touch_task, "cap1133", 3072, NULL, 6, &s_task) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task creation failed");

#if CAP1133_ALERT_GPIO >= 0
    // Non-fatal: the task's own timeout keeps the pads working, just polled.
    if (alert_attach(s_task) != ESP_OK) {
        ESP_LOGW(TAG, "could not attach the interrupt, falling back to polling");
    }
#endif

    return ESP_OK;
}

void cap1133_led_idle_timeout(uint32_t timeout_ms)
{
    s_led_timeout_ms = timeout_ms;
    s_led_timeout_changed = true;

    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

bool cap1133_event_wait(cap1133_event_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return false;
    }

    const TickType_t ticks =
        (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    // No queue means start failed. Block anyway, so a caller that paces its
    // loop on this does not spin and trip the watchdog.
    if (s_events == NULL) {
        vTaskDelay(ticks);
        return false;
    }

    return xQueueReceive(s_events, out, ticks) == pdTRUE;
}

uint8_t cap1133_pads(void)
{
    return s_pads;
}

bool cap1133_pads_valid(void)
{
    return s_pads_valid;
}

uint32_t cap1133_pads_updated_ms(void)
{
    return s_pads_updated_ms;
}
