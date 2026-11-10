#include <stdlib.h>

#include "cap1166.h"
#include "i2c.h"
#include "driver/gpio.h"
#include "esp_attr.h"
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
#define REG_AVG_AND_SAMPLING    0x24
#define REG_INTERRUPT_ENABLE    0x27
#define REG_REPEAT_RATE_ENABLE  0x28
#define REG_MULTIPLE_TOUCH_CFG  0x2A
#define REG_THRESHOLD_1         0x30    // ..0x35, one per pad
#define REG_LED_LINKING         0x72
#define REG_LED_POLARITY        0x73
#define REG_LED_OUTPUT_CONTROL  0x74
#define REG_LED_DIRECT_DUTY     0x93
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
#if CAP1166_RESET_GPIO >= 0
#define RESET_ASSERT_TICKS  (pdMS_TO_TICKS(15) + 1)     // >= tRST_DLY
#define RESET_COMMS_TICKS   (pdMS_TO_TICKS(20) + 1)     // >= tCOMM_DLY
#endif

/*
 * LED polarity: non-inverted on every channel, which is what makes the panel
 * light up and go dark under a finger rather than the other way round.
 *
 * The part has no "invert the link" control, but it does not need one. Table
 * 5.48 gives the appearance of a Direct-behaviour LED as a function of
 * polarity and the actuating signal:
 *
 *   POL   actuated?   brightness
 *   ----  ----------  -----------------------
 *    0    no          min duty            = 0%     <- reset default
 *    0    yes         max duty            = 100%
 *    1    no          100 - min duty      = 100%   <- this board
 *    1    yes         100 - max duty      = 0%
 *
 * with the duty cycles being the reset value of the LED Direct Duty Cycle
 * register (93h = F0h: max 100%, min 0%). So flipping this one register turns
 * "dark until touched" into "lit until touched", and the link still does all
 * the work inside the part.
 *
 * Setting these bits also sets the matching LED Mirror Control bits, because
 * Configuration 2's BLK_POL_MIR is left at its default. That pairing is the
 * intended one -- mirroring is how the brightness interpretation follows the
 * polarity -- so it is deliberately not blocked.
 */
#define LED_POLARITY_NON_INVERTED   CAP1166_PADS_ALL

/*
 * LED Direct Duty Cycle: max 100% (MAX_DUTY[7:4] = Fh), min 0% (MIN_DUTY[3:0]
 * = 0h). Through the polarity table above, the min duty sets how bright an
 * idle pad is (100 - 0 = full brightness) and the max duty how dark a touched
 * or switched-off one is (100 - 100 = off). This is the reset value, but it is
 * written anyway: the MCU does not drive RESET, so after a soft reboot the
 * part keeps whatever was last written here rather than its defaults.
 */
#define LED_DIRECT_DUTY_FULL        0xF0

/*
 * Averaging and sampling: 4 samples per channel on a 35 ms cycle
 * (AVG 010b, SAMP_TIME 10b = 1.28 ms unchanged, CYCLE_TIME 00b).
 *
 * This is the part's own decision rate, and it is the floor under every other
 * latency here -- no amount of interrupt or polling work gets a touch reported
 * sooner than the part decides there was one. The reset value of 39h is 8
 * samples on a 70 ms cycle: 6 channels x 8 samples x 1.28 ms is 61 ms of
 * sampling, so every touch and release waited up to 70 ms before anything
 * downstream could hear about it, the linked LEDs included.
 *
 * At 4 samples the sweep is 6 x 4 x 1.28 = 30.7 ms, which still fits inside a
 * 35 ms cycle -- the datasheet extends the cycle when it does not, so the two
 * halves have to be chosen together. Halving the averaging is the cost: the
 * delta counts get noisier, which matters most at the 128x gain this board
 * runs. If touches start registering on their own, this is the first register
 * to put back to 39h.
 */
#define AVG_AND_SAMPLING_FAST       0x28

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
/*
 * RESET is tied to ground on the board; there is nothing to drive. Power-on
 * reset is long finished by now: app_main gets here seconds after boot, far
 * past the part's 15 ms tCOMM_DLY.
 */
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

    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_AVG_AND_SAMPLING, AVG_AND_SAMPLING_FAST)),
                        TAG, "sampling config failed");

    // Every pad raises ALERT#, which is what CAP1166_ALERT_GPIO waits on. Also
    // the reset value, written so the two are not separated by a silent default.
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_INTERRUPT_ENABLE, CAP1166_PADS_ALL)),
                        TAG, "interrupt enable failed");

    /*
     * No hardware auto-repeat. This driver polls the status register and finds
     * its own edges, so a repeat only adds interrupt assertions nobody reads,
     * and a slider held still is exactly the case it would fire on.
     */
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_REPEAT_RATE_ENABLE, 0x00)), TAG,
                        "repeat rate config failed");

    /*
     * Every LED follows its own pad, decided inside the part: no I2C write per
     * touch, no latency, and nothing to go wrong if a task stalls.
     */
    ESP_RETURN_ON_ERROR(detach_on_error(cap1166_led_link(CAP1166_PADS_ALL)), TAG,
                        "led linking failed");

    // Lit while idle, dark under a finger. See LED_POLARITY_NON_INVERTED.
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_LED_POLARITY,
                                                  LED_POLARITY_NON_INVERTED)),
                        TAG, "led polarity failed");

    // Full brightness when lit, fully dark when not. See LED_DIRECT_DUTY_FULL.
    ESP_RETURN_ON_ERROR(detach_on_error(reg_write(REG_LED_DIRECT_DUTY,
                                                  LED_DIRECT_DUTY_FULL)),
                        TAG, "led duty failed");

    /*
     * The rest of the LED block stays at its reset values, which already say
     * what this board wants: open-drain outputs (71h) sinking through the LEDs
     * tied to +3.3V, and Direct behaviour (81h/82h) so a pad simply switches
     * its LED rather than pulsing or breathing it.
     */

    // Maximum gain. The pads are read through the enclosure wall, so the
    // default setting needs a firmer press than the board is meant to want.
    ESP_RETURN_ON_ERROR(detach_on_error(cap1166_set_sensitivity(SENSITIVITY_HIGHEST)), TAG,
                        "sensitivity failed");

    ESP_LOGI(TAG, "ready at 0x%02X (rev 0x%02X), CS1-3 buttons, CS4-6 slider, "
                  "6 LEDs linked and lit until touched, DELTA_SENSE %d (128x)",
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

esp_err_t cap1166_read_thresholds(uint8_t threshold[6])
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(threshold, ESP_ERR_INVALID_ARG, TAG, "NULL out pointer");

    for (int i = 0; i < 6; i++) {
        ESP_RETURN_ON_ERROR(reg_read((uint8_t)(REG_THRESHOLD_1 + i), &threshold[i]), TAG,
                            "threshold %d read failed", i + 1);
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

    /*
     * Inverted on the way to the register, so callers can go on thinking in
     * terms of "lit". Every channel runs non-inverted here, and for those an
     * actuated output is a dark LED (table 5.48) -- so a bit that means lit in
     * this argument is a clear bit in the register.
     */
    return reg_write(REG_LED_OUTPUT_CONTROL, (uint8_t)(~led_mask & CAP1166_PADS_ALL));
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

/*
 * Two cadences, because the panel asks for opposite things depending on
 * whether a finger is on it.
 *
 * While something is touched the driver has to keep reading: the slider's
 * position only exists as a fresh pair of delta counts, and ALERT# will not
 * fire again until the finger comes off (repeat rate is disabled). 35 ms
 * matches the part's own cycle, so it reads about as often as there is
 * something new to read.
 *
 * While nothing is touched there is nothing to poll for. With ALERT# wired the
 * wait is just a long backstop -- the interrupt does the waking, and this only
 * covers an edge somehow missed or a part that has stopped talking. Without
 * it, this interval *is* the response time, so it stays short.
 */
#define POLL_ACTIVE_MS      35
#if CAP1166_ALERT_GPIO >= 0
#define POLL_IDLE_MS        500
#else
#define POLL_IDLE_MS        50
#endif

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

#if CAP1166_ALERT_GPIO >= 0
/*
 * ALERT# went low: a pad was touched or released. Nothing is read here -- that
 * needs I2C, which an ISR cannot do -- so this only wakes the task, which then
 * reads the status register and clears the interrupt, releasing the line.
 *
 * The notification latches, so an edge that arrives while the task is mid-read
 * is not lost: the next wait returns immediately instead of sleeping through
 * the event that just happened.
 */
static void IRAM_ATTR alert_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static esp_err_t alert_attach(TaskHandle_t task)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CAP1166_ALERT_GPIO,
        .mode         = GPIO_MODE_INPUT,
        // The board pulls this up through R19; the internal one keeps the line
        // defined even on a board where that is missing or unpopulated.
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        // ALERT# is active low and open drain, so activity is a falling edge.
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "alert gpio %d config failed",
                        CAP1166_ALERT_GPIO);

    // One service for the whole application. Another component getting there
    // first is the expected case, not a failure.
    const esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "gpio isr service install failed");
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CAP1166_ALERT_GPIO, alert_isr, task), TAG,
                        "alert isr attach failed");

    ESP_LOGI(TAG, "interrupt driven on gpio %d", CAP1166_ALERT_GPIO);
    return ESP_OK;
}
#endif

/**
 * Is ALERT# still pulled down?
 *
 * Reading the status register and clearing the interrupt are two transfers,
 * and an event landing between them is cleared along with the one being
 * acknowledged. The line stays low in that case, but no new falling edge will
 * ever arrive for it -- so the level, not the edge, is what says there is
 * still something outstanding.
 */
static bool alert_asserted(void)
{
#if CAP1166_ALERT_GPIO >= 0
    return gpio_get_level(CAP1166_ALERT_GPIO) == 0;
#else
    return false;
#endif
}

/**
 * Sleep until the part has something to say, or @p ticks elapse.
 *
 * Without ALERT# there is nothing to be woken by, so the timeout is the whole
 * mechanism and this is an ordinary delay.
 */
static void wait_for_activity(TickType_t ticks)
{
#if CAP1166_ALERT_GPIO >= 0
    ulTaskNotifyTake(pdTRUE, ticks);
#else
    vTaskDelay(ticks);
#endif
}

static volatile uint8_t  s_pads;
static volatile bool     s_pads_valid;
static volatile uint32_t s_pads_updated_ms;

/*
 * LED idle timeout. The setting is written by any task; everything else here
 * is owned by touch_task, which is also the only thing that talks to the LED
 * registers after init, so sleeping and waking never race each other.
 */
static volatile uint32_t s_led_timeout_ms;      ///< 0 = never go dark
static volatile bool     s_led_timeout_changed;

/**
 * Put every LED out: unlinked, so a touch no longer lights anything, and all
 * outputs driven dark. Dark first, then unlink, so there is no moment where an
 * unlinked LED shows whatever the output register last held.
 */
static esp_err_t leds_sleep(void)
{
    ESP_RETURN_ON_ERROR(cap1166_led_set(0), TAG, "led off failed");
    return cap1166_led_link(0);
}

/** Hand the LEDs back to their pads, as init left them. */
static esp_err_t leds_wake(void)
{
    return cap1166_led_link(CAP1166_PADS_ALL);
}

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

    bool     leds_asleep = false;
    bool     swallowing = false;    ///< the touch that woke the LEDs is still down
    uint32_t last_touch_ms = now_ms();
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
         * LED idle timeout. A new setting restarts the clock, so choosing
         * "5 s" in the menu does not put the LEDs out on the spot just because
         * the last touch before that press was long ago -- and choosing
         * "never" brings them straight back.
         */
        if (s_led_timeout_changed) {
            s_led_timeout_changed = false;
            last_touch_ms = now_ms();
            if (s_led_timeout_ms == 0 && leds_asleep && leds_wake() == ESP_OK) {
                leds_asleep = false;
            }
        }

        if (pads != 0) {
            last_touch_ms = now_ms();

            /*
             * Any pad wakes them, and that touch is spent on waking: it posts
             * no button and moves no slider, so a tap aimed at a dark panel
             * cannot also walk the menu. Held until every pad is let go, so
             * a finger left resting does not turn into a press either.
             */
            if (leds_asleep && leds_wake() == ESP_OK) {
                leds_asleep = false;
                swallowing = true;
                ESP_LOGD(TAG, "leds awake");
            }
        }

        if (swallowing) {
            previous_buttons = (uint8_t)(pads & CAP1166_PADS_BUTTON);
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

wait:;
        /*
         * A finger on the panel is the one thing the part will not interrupt
         * about again until it leaves, so that is when this keeps reading of
         * its own accord. Idle, it waits to be told.
         *
         * The idle backstop also paces the LED timeout, which therefore lands
         * up to POLL_IDLE_MS late -- half a second against a 5 s minimum.
         */
        uint32_t wait_ms;
        if (backing_off) {
            wait_ms = POLL_RETRY_MS;
        } else if (pads != 0 || alert_asserted()) {
            /*
             * A line still held down means an edge was swallowed by the
             * read/clear window above. Coming back at the active cadence
             * rather than skipping the wait is deliberate: a part that wedges
             * with ALERT# low would otherwise turn this loop into a spin on
             * the shared bus.
             */
            wait_ms = POLL_ACTIVE_MS;
        } else {
            wait_ms = POLL_IDLE_MS;
        }

        wait_for_activity(pdMS_TO_TICKS(wait_ms));
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

#if CAP1166_ALERT_GPIO >= 0
    /*
     * After the task exists, because the handler needs something to notify.
     * Non-fatal: the task is already running and its own timeout keeps the
     * pads working, just at the polled cadence rather than on demand.
     */
    if (alert_attach(s_task) != ESP_OK) {
        ESP_LOGW(TAG, "could not attach the interrupt, falling back to polling");
    }
#endif

    return ESP_OK;
}

void cap1166_led_idle_timeout(uint32_t timeout_ms)
{
    s_led_timeout_ms = timeout_ms;
    s_led_timeout_changed = true;

    // Act on it now rather than at the end of a 500 ms idle wait.
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
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
