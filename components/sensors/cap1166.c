#include <stdlib.h>

#include "cap1166.h"
#include "i2c.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "cap1166";

/* Only what this driver touches. REG_MAP below names the rest, for the dump. */
#define REG_MAIN_CONTROL        0x00
#define REG_SENSOR_INPUT_STATUS 0x03
#define REG_DELTA_COUNT_1       0x10    // ..0x15, one per pad, two's complement
#define REG_SENSITIVITY         0x1F
#define REG_SENSOR_ENABLE       0x21
#define REG_REPEAT_RATE_ENABLE  0x28
#define REG_MULTIPLE_TOUCH_CFG  0x2A
#define REG_LED_LINKING         0x72
#define REG_LED_OUTPUT_CONTROL  0x74
#define REG_PRODUCT_ID          0xFD
#define REG_MANUFACTURER_ID     0xFE
#define REG_REVISION            0xFF

#define PRODUCT_ID_CAP1166      0x51
#define MANUFACTURER_ID_MCHP    0x5D

#define MAIN_CONTROL_INT        0x01

#define I2C_TIMEOUT_MS          100

/* DELTA_SENSE 000b: 128x, the most sensitive setting (datasheet table 5.7). */
#define SENSITIVITY_HIGHEST     0

/*
 * Multiple touch: blocking left on, but raised from one simultaneous touch to
 * three (MULT_BLK_EN | B_MULT_T = 10b).
 *
 * The reset value of 80h allows exactly one pad at a time, which would make
 * the slider impossible: reading a position between two pads depends on both
 * of them being flagged while a finger bridges them, and at the default the
 * second one is silently blocked. Three is the smallest setting that leaves
 * room for a two-pad bridge plus a button, while still rejecting the four-plus
 * pattern of a hand laid flat on the panel.
 */
#define MULTIPLE_TOUCH_MAX_3    0x88

/*
 * RESET timings, from the datasheet's electrical characteristics:
 *
 *   tRST_DLY     10 ms min   how long RESET must be held high to register
 *   tCOMM_DLY    15 ms max   release -> the part answers on the bus
 *   tRESET      200 ms max   release -> the first conversion is ready
 *
 * The first two are waited out here; the third is not. Nothing needs a
 * conversion during init -- identifying and configuring the part only needs
 * the bus -- and holding app_main for another 200 ms to reach a reading that
 * the poll task will pick up on its own is a boot delay bought for nothing.
 * Until it lands the pads simply read untouched.
 *
 * The + 1 is not decoration. At CONFIG_FREERTOS_HZ 100 a tick is 10 ms, so
 * pdMS_TO_TICKS(15) is one tick, and a one-tick delay expires at the next tick
 * boundary -- which may be microseconds away. Two ticks is what makes the
 * 10 ms minimum a minimum.
 */
#define RESET_ASSERT_TICKS  (pdMS_TO_TICKS(15) + 1)     // >= tRST_DLY
#define RESET_COMMS_TICKS   (pdMS_TO_TICKS(20) + 1)     // >= tCOMM_DLY

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

/*
 * A periodic line showing the slider pads reacting, for setting gain and
 * thresholds against a real enclosure. Off in committed code: it is ~10 I2C
 * reads a second from a task that outranks the sensors, and a console stream
 * that never stops.
 */
#define CAP1166_DEBUG_DUMP      0
#define CAP1166_DUMP_PERIOD_MS  1000

static const char *DBG_TAG = "cap1166.dbg";

/*
 * Names and addresses follow the CAP1166 datasheet's register map (table 5.1).
 * Addresses it leaves undefined are not listed: they read back as 0 and would
 * say nothing.
 */
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
    { 0x13, "CS4 Delta Count" },
    { 0x14, "CS5 Delta Count" },
    { 0x15, "CS6 Delta Count" },
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
    { 0x2F, "Recalibration Config" },
    { 0x30, "CS1 Threshold" },
    { 0x31, "CS2 Threshold" },
    { 0x32, "CS3 Threshold" },
    { 0x33, "CS4 Threshold" },
    { 0x34, "CS5 Threshold" },
    { 0x35, "CS6 Threshold" },
    { 0x38, "Sensor Input Noise Threshold" },
    { 0x40, "Standby Channel" },
    { 0x41, "Standby Config" },
    { 0x42, "Standby Sensitivity" },
    { 0x43, "Standby Threshold" },
    { 0x44, "Configuration 2" },
    { 0x50, "CS1 Base Count" },
    { 0x51, "CS2 Base Count" },
    { 0x52, "CS3 Base Count" },
    { 0x53, "CS4 Base Count" },
    { 0x54, "CS5 Base Count" },
    { 0x55, "CS6 Base Count" },
    { 0x71, "LED Output Type" },
    { 0x72, "Sensor Input LED Linking" },
    { 0x73, "LED Polarity" },
    { 0x74, "LED Output Control" },
    { 0x77, "LED Linked Transition Control" },
    { 0x79, "LED Mirror Control" },
    { 0x81, "LED Behavior 1" },
    { 0x82, "LED Behavior 2" },
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
    { 0xFD, "Product ID" },
    { 0xFE, "Manufacturer ID" },
    { 0xFF, "Revision" },
};

esp_err_t cap1166_dump_registers(void)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    ESP_LOGI(DBG_TAG, "---- CAP1166 registers ----");
    for (size_t i = 0; i < sizeof(REG_MAP) / sizeof(REG_MAP[0]); i++) {
        uint8_t value = 0;
        const esp_err_t err = reg_read(REG_MAP[i].addr, &value);
        if (err != ESP_OK) {
            // Abandon rather than wait out a timeout per register: this can be
            // called from init, which runs inside app_main.
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

/*
 * Pass-through on success; on failure it unwinds the attach first.
 *
 * i2c_add_device() has to set s_dev before the probe, because the probe reads
 * through it -- but that means a failed probe would otherwise leave a live
 * handle for a part that was never identified, which cap1166_init()'s
 * early-out would then read as "already initialized".
 */
#if CAP1166_RESET_GPIO >= 0
/*
 * Drive the part through a clean reset before speaking to it.
 *
 * Worth doing even though power-on already resets it, because a firmware
 * restart is not a power cycle: the MCU reboots and the CAP1166 keeps whatever
 * it was last configured with, including a gain and a link mask from a build
 * that may no longer match this one. Pulsing RESET means init always starts
 * from the datasheet defaults it is written against.
 *
 * It also settles the line. Configuring the GPIO as an output and driving it
 * low is what guarantees the part is not being held asleep by a pin nobody
 * ever claimed.
 */
static esp_err_t reset_part(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CAP1166_RESET_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "reset gpio %d config failed",
                        CAP1166_RESET_GPIO);

    gpio_set_level(CAP1166_RESET_GPIO, 1);      // active high: held in deep sleep
    vTaskDelay(RESET_ASSERT_TICKS);
    gpio_set_level(CAP1166_RESET_GPIO, 0);      // released: the part starts
    vTaskDelay(RESET_COMMS_TICKS);

    return ESP_OK;
}
#else
/* RESET is tied to ground on the board; there is nothing to drive. */
static esp_err_t reset_part(void)
{
    return ESP_OK;
}
#endif

static esp_err_t detach_on_error(esp_err_t err)
{
    if (err != ESP_OK && s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    return err;
}

esp_err_t cap1166_init(void)
{
    if (s_dev) {
        return ESP_OK;
    }

    // Before the first transfer: while RESET is asserted the communications
    // block is in reset too, and the part will not even acknowledge its address.
    ESP_RETURN_ON_ERROR(reset_part(), TAG, "reset failed");

    ESP_RETURN_ON_ERROR(i2c_add_device(CAP1166_I2C_ADDR, &s_dev), TAG, "attach failed");

    uint8_t product_id = 0, manufacturer_id = 0, revision = 0;
    ESP_RETURN_ON_ERROR(detach_on_error(reg_read(REG_PRODUCT_ID, &product_id)), TAG,
                        "product id read failed");
    ESP_RETURN_ON_ERROR(detach_on_error(reg_read(REG_MANUFACTURER_ID, &manufacturer_id)), TAG,
                        "manufacturer id read failed");

    if (product_id != PRODUCT_ID_CAP1166 || manufacturer_id != MANUFACTURER_ID_MCHP) {
        /*
         * 6Dh here is the CAP1203 this board used to carry. It answers at the
         * same address and shares most of the register map, so the failure is
         * worth naming: it means the firmware was built for the wrong board,
         * not that the panel is broken. See TOUCH_IC in touch.h.
         */
        if (product_id == 0x6D) {
            ESP_LOGE(TAG, "found a CAP1203 at 0x%02X, not a CAP1166 -- "
                          "set TOUCH_IC to TOUCH_IC_CAP1203 in touch.h", CAP1166_I2C_ADDR);
        } else {
            ESP_LOGE(TAG, "unexpected ids 0x%02X/0x%02X at 0x%02X -- check the ADDR_COMM "
                          "strapping resistor against the table in cap1166.h",
                     product_id, manufacturer_id, CAP1166_I2C_ADDR);
        }
        return detach_on_error(ESP_ERR_NOT_FOUND);
    }

    (void)reg_read(REG_REVISION, &revision);    // logged below; not worth failing over

    // CS1..CS6. Already the reset value, written anyway so a warm restart with
    // half the pads disabled comes back in a known state.
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_SENSOR_ENABLE, CAP1166_PADS_ALL)), TAG,
                        "enable failed");

    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_MULTIPLE_TOUCH_CFG, MULTIPLE_TOUCH_MAX_3)),
                        TAG, "multiple touch config failed");

    /*
     * No hardware auto-repeat. This driver polls the status register and finds
     * its own edges, so a repeat only adds interrupt assertions nobody reads,
     * and a slider held still is exactly the case it would fire on.
     */
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_REPEAT_RATE_ENABLE, 0x00)), TAG,
                        "repeat rate config failed");

    /*
     * Every LED follows its own pad, decided inside the part.
     *
     * The rest of the LED block is left at its reset values, which already say
     * what this board wants: open-drain outputs (71h) sinking through LEDs tied
     * to VDD, inverted polarity (73h) so an actuated output pulls the pin low,
     * Direct behaviour (81h/82h) so the LED is simply on for the duration of
     * the touch rather than pulsing or breathing, and a 0-100% direct duty
     * cycle (93h). Linking is the only bit that has to change.
     */
    ESP_RETURN_ON_ERROR(detach_on_error(cap1166_led_link(CAP1166_PADS_ALL)), TAG,
                        "led linking failed");

    // Maximum gain. The pads are read through the enclosure wall, so the
    // default setting needs a firmer press than the board is meant to want.
    ESP_RETURN_ON_ERROR(detach_on_error(cap1166_set_sensitivity(SENSITIVITY_HIGHEST)), TAG,
                        "sensitivity failed");

    ESP_LOGI(TAG, "ready at 0x%02X (rev 0x%02X), CS1-3 buttons, CS4-6 slider, "
                  "6 LEDs linked, DELTA_SENSE %d (128x)",
             CAP1166_I2C_ADDR, revision, SENSITIVITY_HIGHEST);

    return ESP_OK;
}

esp_err_t cap1166_read(uint8_t *pads)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(pads, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_SENSOR_INPUT_STATUS, &status), TAG, "status read failed");
    *pads = status & CAP1166_PADS_ALL;

    // Status stays latched until INT is cleared, so a released pad would otherwise stick.
    uint8_t main_control = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_MAIN_CONTROL, &main_control), TAG, "control read failed");
    if (main_control & MAIN_CONTROL_INT) {
        ESP_RETURN_ON_ERROR(reg_write(REG_MAIN_CONTROL, main_control & ~MAIN_CONTROL_INT), TAG,
                            "clear int failed");
    }

    return ESP_OK;
}

esp_err_t cap1166_read_deltas(int8_t delta[6])
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(delta, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    for (int i = 0; i < 6; i++) {
        uint8_t raw = 0;
        ESP_RETURN_ON_ERROR(reg_read((uint8_t)(REG_DELTA_COUNT_1 + i), &raw), TAG,
                            "delta %d read failed", i + 1);
        delta[i] = (int8_t)raw;     // two's complement, saturating at +127/-128
    }
    return ESP_OK;
}

esp_err_t cap1166_set_sensitivity(uint8_t delta_sense)
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

esp_err_t cap1166_led_link(uint8_t pad_mask)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    return reg_write(REG_LED_LINKING, (uint8_t)(pad_mask & CAP1166_PADS_ALL));
}

esp_err_t cap1166_led_set(uint8_t led_mask)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    return reg_write(REG_LED_OUTPUT_CONTROL, (uint8_t)(led_mask & CAP1166_PADS_ALL));
}


/* ======================================================================
 * Slider
 *
 * CS4, CS5 and CS6 as one strip. The touched pads say roughly where the finger
 * is; their delta counts -- how far each pad has moved from its own calibrated
 * reference -- say how the touch divides between them, which is what turns
 * three pads into a position rather than three steps.
 * ====================================================================== */

/** Pads in strip order: index 0 is the bottom (CS4), index 2 the top (CS6). */
#define SLIDER_PAD_COUNT    3
#define SLIDER_FIRST_PAD    3           ///< CS4 is delta[] index 3

/*
 * How far the position has to move before the callback fires again. Deltas
 * wander by a count or two under a resting finger, and without a deadband that
 * wander reaches whatever the position drives. Three points out of a hundred
 * is below what the eye resolves on an LED strip and well above the noise.
 */
#define SLIDER_DEADBAND     3

static volatile uint8_t s_slider_live = CAP1166_SLIDER_RELEASED;
static volatile uint8_t s_slider_last;      ///< last position seen while touched
static int              s_slider_reported = -1;
static cap1166_slider_cb_t volatile s_slider_cb;

/**
 * Weighted centroid over the touched pads.
 *
 * Weighting by delta is what gives a position between the pads. Clamping each
 * weight to at least 1 is what keeps it sane when the deltas say something
 * unhelpful -- saturated at +127 on a hard press, or driven to 0 by the analog
 * noise filter -- because then the touched pads alone still place the finger,
 * at the midpoint of whichever ones are lit.
 */
static uint8_t slider_centroid(uint8_t engaged, const int8_t delta[6])
{
    int32_t weighted = 0, total = 0;

    for (int i = 0; i < SLIDER_PAD_COUNT; i++) {
        if (!(engaged & (CAP1166_PAD4 << i))) {
            continue;
        }
        int32_t weight = delta[SLIDER_FIRST_PAD + i];
        if (weight < 1) {
            weight = 1;
        }
        weighted += weight * i;
        total    += weight;
    }

    if (total <= 0) {
        return s_slider_last;   // unreachable while engaged != 0; be safe anyway
    }

    // Index 0..2 onto 0..100, rounded. Bottom pad is 0 because CS4 is mounted
    // at the bottom of the strip.
    return (uint8_t)((weighted * 100 + total) / (total * 2));
}

static void post_event(cap1166_event_t event);

static void slider_update(uint8_t pads)
{
    const uint8_t engaged = pads & CAP1166_PADS_SLIDER;

    if (engaged == 0) {
        if (s_slider_live != CAP1166_SLIDER_RELEASED) {
            s_slider_live = CAP1166_SLIDER_RELEASED;
            s_slider_reported = -1;
            // One event per gesture, at the end of it -- the live value went
            // out through the callback while the finger was moving.
            post_event(CAP1166_EVENT_SLIDER);
        }
        return;
    }

    int8_t delta[6] = { 0 };
    if (cap1166_read_deltas(delta) != ESP_OK) {
        // Position unknown for this pass. The strip is still held, so hold the
        // last one rather than inventing a jump.
        return;
    }

    const uint8_t percent = slider_centroid(engaged, delta);
    const bool first_touch = (s_slider_live == CAP1166_SLIDER_RELEASED);

    s_slider_live = percent;
    s_slider_last = percent;

    const int moved = (s_slider_reported < 0) ? SLIDER_DEADBAND
                                              : abs((int)percent - s_slider_reported);
    if (first_touch || moved >= SLIDER_DEADBAND) {
        s_slider_reported = percent;

        // One read, so the null check and the call cannot disagree about a
        // callback being registered or cleared between them.
        const cap1166_slider_cb_t callback = s_slider_cb;
        if (callback) {
            callback(percent);
        }
    }
}

uint8_t cap1166_slider_percent(void)
{
    return s_slider_live;
}

uint8_t cap1166_slider_percent_last(void)
{
    return s_slider_last;
}

void cap1166_slider_on_change(cap1166_slider_cb_t callback)
{
    s_slider_cb = callback;
}


/* ======================================================================
 * Polling task
 * ====================================================================== */

#define POLL_PERIOD_MS      50      // fast enough that a tap never feels dropped
#define EVENT_QUEUE_DEPTH   8

/*
 * Backoff for a part that answers at init and stops answering later. At the
 * normal rate a failing read costs several I2C_TIMEOUT_MS waits every poll,
 * from a task that outranks every sensor -- most of a shared bus spent on a
 * part that is gone. One good read restores the rate.
 */
#define POLL_FAIL_LIMIT     10
#define POLL_RETRY_MS       2000

static QueueHandle_t s_events;
static TaskHandle_t  s_task;

static volatile uint8_t  s_pads;
static volatile bool     s_pads_valid;
static volatile uint32_t s_pads_updated_ms;

static void post_event(cap1166_event_t event)
{
    // Depth 8 against a UI that drains every frame: a full queue means the
    // consumer has stalled, and the stale press is the one worth losing.
    if (xQueueSend(s_events, &event, 0) != pdTRUE) {
        ESP_LOGD(TAG, "event queue full, dropped event %d", (int)event);
    }
}

#if CAP1166_DEBUG_DUMP
static void dump_tick(uint8_t pads)
{
    int8_t delta[6] = { 0 };
    if (cap1166_read_deltas(delta) != ESP_OK) {
        return;
    }
    ESP_LOGI(DBG_TAG, "pads 0x%02X | btn %4d %4d %4d | slider %4d %4d %4d -> %u",
             pads, delta[0], delta[1], delta[2], delta[3], delta[4], delta[5],
             cap1166_slider_percent());
}
#endif

static void touch_task(void *arg)
{
    (void)arg;

    uint8_t  previous_buttons = 0;
    unsigned consecutive_failures = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
#if CAP1166_DEBUG_DUMP
    uint32_t last_dump_ms = 0;
#endif

    while (1) {
        uint8_t pads = 0;
        const bool ok = (cap1166_read(&pads) == ESP_OK);

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
         * Leading edge only, and only over the three button pads -- the slider
         * pads are a position, not three more buttons, and pressing one of
         * them must not also walk the menu.
         *
         * No extra debounce: the status register latches until cap1166_read()
         * clears the interrupt, so a tap shorter than the poll period is still
         * caught -- requiring two consecutive samples would throw exactly those
         * away.
         */
        const uint8_t buttons = (uint8_t)(pads & CAP1166_PADS_BUTTON);
        const uint8_t pressed = (uint8_t)(buttons & ~previous_buttons);
        previous_buttons = buttons;

        if (pressed & CAP1166_PAD1) {
            post_event(CAP1166_EVENT_LEFT);
        }
        if (pressed & CAP1166_PAD2) {
            post_event(CAP1166_EVENT_ENTER);
        }
        if (pressed & CAP1166_PAD3) {
            post_event(CAP1166_EVENT_RIGHT);
        }

        // Reads three more registers, but only while the strip is being
        // touched, so an idle panel costs nothing extra on the bus.
        if (!backing_off) {
            slider_update(pads);
        }

#if CAP1166_DEBUG_DUMP
        if (!backing_off && now_ms() - last_dump_ms >= CAP1166_DUMP_PERIOD_MS) {
            last_dump_ms = now_ms();
            dump_tick(pads);
        }
#endif

        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(backing_off ? POLL_RETRY_MS : POLL_PERIOD_MS));
    }
}

esp_err_t cap1166_start(void)
{
    if (s_task) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(cap1166_init(), TAG, "init failed");

    if (s_events == NULL) {
        s_events = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(cap1166_event_t));
        ESP_RETURN_ON_FALSE(s_events, ESP_ERR_NO_MEM, TAG, "event queue alloc failed");
    }

    // Priority above the acquisition tasks: the loop is short and a late poll
    // is a press the user has to repeat.
    ESP_RETURN_ON_FALSE(xTaskCreate(touch_task, "cap1166", 3072, NULL, 6, &s_task) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task creation failed");

    return ESP_OK;
}

bool cap1166_event_wait(cap1166_event_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return false;
    }

    const TickType_t ticks =
        (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    /*
     * A NULL queue means cap1166_start() never got past probing the part, so
     * no event will ever arrive. Block anyway: this call is the only pacing a
     * wait-and-repaint loop has, and returning immediately turns that loop into
     * a busy spin that starves the idle task until the watchdog reboots the
     * board. Missing pads should cost the buttons, not the board.
     */
    if (s_events == NULL) {
        vTaskDelay(ticks);
        return false;
    }

    return xQueueReceive(s_events, out, ticks) == pdTRUE;
}

uint8_t cap1166_pads(void)
{
    return s_pads;
}

bool cap1166_pads_valid(void)
{
    return s_pads_valid;
}

uint32_t cap1166_pads_updated_ms(void)
{
    return s_pads_updated_ms;
}
