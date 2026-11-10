#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif_sntp.h"

#include "i2c.h"
#include "uart.h"
#include "spi.h"
#include "pwm.h"

#include "air_monitor.h"
#include "aqi.h"
#include "touch.h"
#include "cloud.h"
#include "co2.h"
#include "pm.h"
#include "epaper_display.h"
#include "tpl5010.h"
#include "ws2812.h"
#include "wifi_sta.h"
#include "wifi_setup.h"

static const char *TAG = "main";

#define REPORT_PERIOD_MS    5000

/*
 * How often the dashboard repaints with nobody touching the panel -- chosen
 * from the menu, see REFRESH_RATES. A press always repaints at once.
 *
 * Fast refresh drives the whole panel without the several flickers a full GC
 * waveform needs and finishes in about a second, which is what makes a 5 s
 * cadence possible at all. Every sixth update comes back as a full refresh:
 * the driver enforces the datasheet's EPAPER_MAX_FAST_REFRESHES limit itself,
 * so a slower rate also means fewer of those full-panel flashes.
 *
 * DISPLAY_TICK_MS is the fastest the loop ever runs on its own, and what it
 * falls back to while something on screen has a deadline of its own (the
 * Wi-Fi offer timing out), so a slow rate never strands it.
 */
#define DISPLAY_TICK_MS     5000


/* ----------------------------------------------------------------- UI state */

/*
 * A full-screen question that pre-empts whatever page was showing. Only the
 * Wi-Fi setup offer uses it so far, which is why it lives here rather than in
 * the display component: the panel renders it as an ordinary INFO page and
 * knows nothing about it.
 */
typedef enum {
    UI_PROMPT_NONE = 0,
    UI_PROMPT_ASK,      ///< "open a setup hotspot?", waiting on a pad
    UI_PROMPT_PORTAL,   ///< portal is up, showing how to reach it
} ui_prompt_t;

/*
 * Everything the buttons can change. Owned by ui_task; report_task only reads
 * led_on, and a byte read racing a byte write costs at worst one frame of the
 * strip being a cycle late.
 *
 * The LED bar's brightness is deliberately not in here -- the slider writes it
 * from the touch task, so it lives on its own below.
 */
static struct {
    display_screen_t screen;
    uint8_t          menu_row;
    bool             led_on;
    bool             use_fahrenheit;
    bool             dark_theme;
#if CONFIG_DISPLAY_FRONTLIGHT
    uint8_t          frontlight;    ///< index into FRONTLIGHT_LEVELS
#endif
    uint8_t          pad_led;       ///< index into PAD_LED_TIMEOUTS
    uint8_t          refresh;       ///< index into REFRESH_RATES
    ui_prompt_t      prompt;
    int64_t          prompt_since_us;
} s_ui = {
    .screen = DISPLAY_SCREEN_HOME,
    .led_on = true,
#if CONFIG_DISPLAY_FRONTLIGHT
    .frontlight = 0,                // off until asked for; see apply_frontlight()
#endif
    .pad_led = 2,                   // 15 s
    .refresh = 0,                   // 5 s
};


/* ------------------------------------------------------- frontlight (IO3) */

#if CONFIG_DISPLAY_FRONTLIGHT
/*
 * The panel's frontlight, driven by the LEDC channel pwm.c puts on GPIO 3.
 * Only on the frontlit panel; the bare one (CONFIG_DISPLAY_FRONTLIGHT off) is
 * the same glass and driver with nothing here to light.
 * E-paper holds its image with no power, so unlike the LED ring this is purely
 * a "can I read it right now" control -- it stays where it was left.
 *
 * Four discrete steps rather than a continuous ramp: three pads mean the menu
 * can only offer a row that cycles a short list. 15% is the top of the range,
 * not a derated maximum -- the whole useful span of this circuit sits at the
 * bottom of the duty scale, so the steps divide that span evenly instead of
 * spreading across a 0..100% range whose upper end is never wanted.
 *
 * At the 10-bit resolution pwm.h sets, these land on 0, 51, 102 and 153 counts,
 * so there is still plenty of room between the steps.
 */
static const struct {
    const char *name;
    float       percent;
} FRONTLIGHT_LEVELS[] = {
    { "Off",  0.0f },
    { "Low",  5.0f },
    { "Med",  10.0f },
    { "High", 15.0f },
};

#define FRONTLIGHT_LEVEL_COUNT  (sizeof(FRONTLIGHT_LEVELS) / sizeof(FRONTLIGHT_LEVELS[0]))

static void apply_frontlight(void)
{
    const float percent = FRONTLIGHT_LEVELS[s_ui.frontlight].percent;
    const esp_err_t err = pwm_set_duty_percent(percent);

    if (err != ESP_OK) {
        // Not fatal: the panel is readable in room light, it is the frontlight
        // that is missing, so say so and carry on.
        ESP_LOGE(TAG, "frontlight set failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "frontlight %s (%.0f%% duty on gpio %d)",
             FRONTLIGHT_LEVELS[s_ui.frontlight].name, percent, PWM_GPIO);
}
#endif // CONFIG_DISPLAY_FRONTLIGHT


/* ------------------------------------------------------------ pad LEDs */

/*
 * How long the touch pads' own LEDs stay lit with nobody touching the panel.
 * After that they go dark, and the next tap on any pad only wakes them -- see
 * cap1166_led_idle_timeout(). A short list cycled by one menu row, like the
 * frontlight.
 */
static const struct {
    const char *name;
    uint32_t    timeout_ms;     ///< 0 = never go dark
} PAD_LED_TIMEOUTS[] = {
    { "Always on", 0 },
    { "5 s",       5000 },
    { "15 s",      15000 },
    { "30 s",      30000 },
};

#define PAD_LED_TIMEOUT_COUNT   (sizeof(PAD_LED_TIMEOUTS) / sizeof(PAD_LED_TIMEOUTS[0]))

static void apply_pad_led_timeout(void)
{
    touch_led_idle_timeout(PAD_LED_TIMEOUTS[s_ui.pad_led].timeout_ms);
}


/* ------------------------------------------------------------ refresh rate */

/*
 * The dashboard's repaint interval. The readings themselves are sampled on
 * their own schedule regardless; this only decides how often the panel shows
 * the latest of them. Slower is calmer to look at and wears the panel less.
 */
static const struct {
    const char *name;
    uint32_t    period_ms;
} REFRESH_RATES[] = {
    { "5 s",   5000 },
    { "10 s",  10000 },
    { "30 s",  30000 },
    { "1 min", 60000 },
};

#define REFRESH_RATE_COUNT  (sizeof(REFRESH_RATES) / sizeof(REFRESH_RATES[0]))

_Static_assert(DISPLAY_TICK_MS <= 5000,
               "DISPLAY_TICK_MS must not be slower than the fastest refresh rate");


/* ------------------------------------------------- LED bar brightness (slider) */

/*
 * The slider's range, expressed on the WS2812's 0..255 scale.
 *
 * The bottom is a floor, not off. Switching the ring off is the LED ring menu
 * row's job, and a slider that could also do it would leave two controls
 * disagreeing about the same state -- so the dimmest setting is dim, and off
 * stays one press away.
 *
 * The top is half scale on purpose. These LEDs draw ~60 mA each at full, which
 * is 660 mA for the eleven on this strip, and ws2812.h starts at 64 for that
 * reason. 128 is twice that default and still well inside what the supply is
 * asked for elsewhere. Raise it here if the board's supply says otherwise.
 */
#define LED_BRIGHTNESS_MIN      8
#define LED_BRIGHTNESS_MAX      128

/* Boots at the top of the slider's range, LED_BRIGHTNESS_MAX on the strip's
 * scale. The slider only ever comes down from here. */
#define LED_BRIGHTNESS_DEFAULT_PCT  100

/*
 * Written by the touch task from the slider callback and read by ui_task and
 * the menu. A byte, so a read racing a write costs at worst a menu row showing
 * the previous percentage for one frame.
 */
static volatile uint8_t s_led_brightness_pct = LED_BRIGHTNESS_DEFAULT_PCT;

static uint8_t brightness_scale(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    return (uint8_t)(LED_BRIGHTNESS_MIN +
                     ((LED_BRIGHTNESS_MAX - LED_BRIGHTNESS_MIN) * percent + 50) / 100);
}

/*
 * Runs on the touch task, once per slider movement past the driver's deadband.
 *
 * It repaints the strip itself rather than waking ui_task, because ui_task
 * repaints the e-paper panel too: at ~1 s a refresh and a limited fast-refresh
 * budget, a panel cannot be asked to follow a finger. The strip can -- staging
 * is a memcpy and the shift-out is under a millisecond -- so brightness tracks
 * the slider live and the panel hears about it once, on release, as
 * TOUCH_EVENT_SLIDER.
 *
 * ws2812 takes its own lock, which is what makes this safe against report_task
 * painting the AQI colour at the same moment.
 */
/*
 * Range the slider has covered during the touch test's slider step. Written
 * here, on the touch task, because the callback sees every position while the
 * ui loop only sees the end of the gesture. See touch_test_poll().
 */
static volatile bool    s_tt_slider_tracking;
static volatile uint8_t s_tt_slider_min = 100;
static volatile uint8_t s_tt_slider_max = 0;

static void on_slider_moved(uint8_t percent)
{
    if (s_tt_slider_tracking) {
        if (percent < s_tt_slider_min) {
            s_tt_slider_min = percent;
        }
        if (percent > s_tt_slider_max) {
            s_tt_slider_max = percent;
        }
    }

    s_led_brightness_pct = percent;
    ws2812_set_brightness(brightness_scale(percent));
    ws2812_refresh();
}


/* ------------------------------------------------------------- AQI -> LEDs */

/*
 * The strip is a bar graph: two LEDs per AQI band, all lit in that band's EPA
 * colour, so the height reads as "how bad" and the colour names the category.
 * Hazardous is the whole strip.
 */
static int leds_for_aqi(int aqi, aqi_category_t category)
{
    static const int BAND_LO[] = { 0, 51, 101, 151, 201 };
    static const int BAND_HI[] = { 50, 100, 150, 200, 300 };

    if (category >= AQI_HAZARDOUS) {
        return WS2812_LED_COUNT;
    }

    const int midpoint = (BAND_LO[category] + BAND_HI[category]) / 2;
    return (int)category * 2 + ((aqi > midpoint) ? 2 : 1);
}

/*
 * Purple is the one colour the EPA scale never uses -- it runs green, yellow,
 * orange, red, purple-red, maroon -- so a plain purple ring cannot be misread
 * as an air quality band. It means the network is down, nothing more.
 */
#define OFFLINE_R   96
#define OFFLINE_G   0
#define OFFLINE_B   160

static void update_leds(const air_monitor_device *dev)
{
    ws2812_set_all(0, 0, 0);

    if (!s_ui.led_on) {
        // Switched off from the menu: nothing lit, but still refreshed so the
        // strip actually goes dark rather than holding its last frame.
        ws2812_refresh();
        return;
    }

    if (!wifi_sta_is_online()) {
        /*
         * Outranks the AQI colour deliberately: the ring is a single-colour
         * indicator, so it can only say one thing at a time, and a monitor
         * that has quietly stopped reporting is the more urgent of the two.
         * The panel keeps showing the readings either way.
         */
        ws2812_set_all(OFFLINE_R, OFFLINE_G, OFFLINE_B);
        ws2812_refresh();
        return;
    }

    if (!dev->pm_valid) {
        // No particle data: one dim white pixel rather than a misleading colour.
        ws2812_set_pixel(0, 24, 24, 24);
    } else {
        const int aqi = aqi_from_pm25(dev->pm.pm2_5_ugm3);
        const aqi_category_t category = aqi_category(aqi);

        uint8_t r, g, b;
        aqi_category_rgb(category, &r, &g, &b);

        ws2812_set_all(r,g,b);
        // const int lit = leds_for_aqi(aqi, category);
        // for (int i = 0; i < lit; i++) {
        //     ws2812_set_pixel((uint16_t)i, r, g, b);
        // }
    }

    ws2812_refresh();
}

/* -------------------------------------------------------------- reporting */

static void report_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        air_monitor_device dev;
        air_monitor_get(&dev);

        ESP_LOGI(TAG, "---- report ----");

        if (dev.sht40_valid) {
            ESP_LOGI(TAG, "SHT40    %.2f C  %.2f %%RH  (t=%lu ms)",
                     dev.temperature_c, dev.humidity_pct, (unsigned long)dev.sht40_updated_ms);
        } else {
            ESP_LOGW(TAG, "SHT40    read failed");
        }

        if (dev.spl07_valid) {
            ESP_LOGI(TAG, "SPL07    %.1f Pa (%.2f hPa)  %.2f C  alt %.1f m  (t=%lu ms)",
                     dev.pressure_pa, dev.pressure_pa / 100.0f, dev.die_temp_c, dev.altitude_m,
                     (unsigned long)dev.spl07_updated_ms);
        } else {
            ESP_LOGW(TAG, "SPL07    read failed");
        }

        if (dev.sgp41_conditioning) {
            ESP_LOGI(TAG, "SGP41    conditioning");
        } else if (dev.sgp41_valid) {
            ESP_LOGI(TAG, "SGP41    VOC raw %u  NOx raw %u  (t=%lu ms)",
                     dev.voc_raw, dev.nox_raw, (unsigned long)dev.sgp41_updated_ms);
        } else {
            ESP_LOGW(TAG, "SGP41    read failed");
        }

        if (touch_pads_valid()) {
            const uint8_t pads = touch_pads();

            // Only the part that has a strip gets a column for it, so the line
            // does not claim hardware this board is not carrying.
            char slider[24] = "";
            if (touch_has_slider()) {
                const uint8_t position = touch_slider_percent();
                if (position == TOUCH_SLIDER_RELEASED) {
                    snprintf(slider, sizeof(slider), "  slider --");
                } else {
                    snprintf(slider, sizeof(slider), "  slider %u%%", position);
                }
            }

            ESP_LOGI(TAG, "%s  pads %c%c%c%s  led bar %u%%  (t=%lu ms)", TOUCH_IC_NAME,
                     (pads & TOUCH_PAD_LEFT)  ? 'L' : '-',
                     (pads & TOUCH_PAD_ENTER) ? 'E' : '-',
                     (pads & TOUCH_PAD_RIGHT) ? 'R' : '-',
                     slider, s_led_brightness_pct,
                     (unsigned long)touch_pads_updated_ms());
        } else {
            ESP_LOGW(TAG, "%s  read failed", TOUCH_IC_NAME);
        }

        if (dev.pm_valid && pm_has(&dev.pm, PM_FIELD_PM4_0)) {
            ESP_LOGI(TAG, "%s  PM1.0 %.1f  PM2.5 %.1f  PM4.0 %.1f  PM10 %.1f ug/m3  (t=%lu ms)",
                     pm_active_name(), dev.pm.pm1_0_ugm3, dev.pm.pm2_5_ugm3,
                     dev.pm.pm4_0_ugm3, dev.pm.pm10_ugm3,
                     (unsigned long)dev.pm_updated_ms);
        } else if (dev.pm_valid) {
            ESP_LOGI(TAG, "%s  PM1.0 %.1f  PM2.5 %.1f  PM10 %.1f ug/m3  (t=%lu ms)",
                     pm_active_name(), dev.pm.pm1_0_ugm3, dev.pm.pm2_5_ugm3,
                     dev.pm.pm10_ugm3, (unsigned long)dev.pm_updated_ms);
        } else {
            ESP_LOGW(TAG, "%s  no reading", pm_active_name());
        }

        if (dev.co2_valid) {
            ESP_LOGI(TAG, "%s  %u ppm CO2  (t=%lu ms)", co2_active_name(),
                     dev.co2_ppm, (unsigned long)dev.co2_updated_ms);
        } else {
            ESP_LOGW(TAG, "%s  no response", co2_active_name());
        }

        update_leds(&dev);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(REPORT_PERIOD_MS));
    }
}



/* ---------------------------------------------------------------- e-paper */

/*
 * The screen layout lives in components/display now. This end only translates
 * the board's device state into the snapshot the display service consumes: a
 * reading the sensor did not produce is left at its sentinel, which is what
 * makes it render as "--" instead of as a plausible zero.
 */
static void snapshot_values(const air_monitor_device *dev, display_values_t *v)
{
    display_values_reset(v);

    if (dev->pm_valid) {
        // The bound part decides whether it measured PM2.5 at all; one that
        // did not leaves the headline at its "--" sentinel. PM1.0 and PM10
        // are logged and uploaded but not shown.
        if (pm_has(&dev->pm, PM_FIELD_PM2_5)) {
            v->pm25_ugm3 = dev->pm.pm2_5_ugm3;
        }
    }
    if (dev->co2_valid) {
        v->co2_ppm = dev->co2_ppm;
    }
    if (dev->sht40_valid) {
        v->temperature_c = dev->temperature_c;
        v->humidity_pct  = dev->humidity_pct;
    }
    if (dev->sgp41_valid) {
        v->tvoc_raw = dev->voc_raw;
        v->nox_raw  = dev->nox_raw;
    }
    v->tvoc_conditioning = dev->sgp41_conditioning;

    if (dev->spl07_valid) {
        v->pressure_hpa = dev->pressure_pa / 100.0f;
    }
}

/* --------------------------------------------------------- Wi-Fi setup ask */

/*
 * How long the station gets to find its network before the offer appears. Long
 * enough to cover a slow AP and a DHCP round trip, short enough that someone
 * standing over a freshly plugged-in monitor is still there to answer.
 */
#define WIFI_BOOT_GRACE_MS  30000

/*
 * How long app_main waits for a verdict on the link. Deliberately shorter than
 * the grace period above, so the log says why the link failed before the panel
 * offers to do something about it.
 */
#define WIFI_VERIFY_MS      20000

/*
 * How long the question stays up unanswered. Timing out matters more than it
 * looks: most of these run unattended, and a monitor parked on a question is a
 * monitor showing no readings.
 */
#define WIFI_ASK_MS         60000

static char    s_prompt_text[256];
static bool    s_wifi_ready;            ///< wifi_sta_init() succeeded
static bool    s_boot_ask_done;         ///< the offer is made once per boot
static int64_t s_boot_deadline_us;

static void prompt_dismiss(void)
{
    s_ui.prompt = UI_PROMPT_NONE;
    s_ui.screen = DISPLAY_SCREEN_HOME;
}

static void prompt_open_portal(void)
{
    const esp_err_t err = wifi_setup_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not open the setup portal: %s", esp_err_to_name(err));
        snprintf(s_prompt_text, sizeof(s_prompt_text),
                 "Setup unavailable\n\n"
                 "The hotspot could not start (%s).\n\n"
                 "Middle pad = try again    Left or right = back",
                 esp_err_to_name(err));
        /*
         * Stays an ASK, not a PORTAL: PORTAL clears itself as soon as the
         * portal is not running, which for a failed start is immediately, and
         * the message would be gone before anyone could read it. As an ASK it
         * holds for the same 60 s as the original question and retries on the
         * middle pad.
         */
        s_ui.prompt = UI_PROMPT_ASK;
        s_ui.prompt_since_us = esp_timer_get_time();
        return;
    }

    snprintf(s_prompt_text, sizeof(s_prompt_text),
             "Wi-Fi setup\n\n"
             "1. Join the network\n"
             "   \"%s\"\n"
             "2. Open %s\n\n"
             "Any pad cancels.",
             wifi_setup_ap_ssid(), WIFI_SETUP_PORTAL_URL);
    s_ui.prompt = UI_PROMPT_PORTAL;
}

/** Drives the offer's own lifecycle; called once per ui_task pass. */
static void update_wifi_prompt(void)
{
    const int64_t now = esp_timer_get_time();

    switch (s_ui.prompt) {
    case UI_PROMPT_NONE:
        /*
         * Asked once, and only about the boot-time failure. A link that drops
         * later is the reconnect logic's problem: interrupting the dashboard
         * every time a router reboots would be worse than the outage.
         */
        if (s_wifi_ready && !s_boot_ask_done && !wifi_sta_is_online() &&
            now >= s_boot_deadline_us) {
            s_boot_ask_done = true;
            snprintf(s_prompt_text, sizeof(s_prompt_text),
                     "No Wi-Fi\n\n"
                     "Could not join \"%s\".\n\n"
                     "Open a setup hotspot?\n\n"
                     "Middle pad = yes    Left or right = no",
                     wifi_sta_ssid());
            s_ui.prompt = UI_PROMPT_ASK;
            s_ui.prompt_since_us = now;
            ESP_LOGI(TAG, "offering the Wi-Fi setup portal");
        }
        break;

    case UI_PROMPT_ASK:
        if (wifi_sta_is_online()) {
            ESP_LOGI(TAG, "setup offer withdrawn: came online on its own");
            prompt_dismiss();
        } else if (now - s_ui.prompt_since_us > (int64_t)WIFI_ASK_MS * 1000) {
            ESP_LOGI(TAG, "setup offer timed out, retrying quietly");
            prompt_dismiss();
        }
        break;

    case UI_PROMPT_PORTAL:
        // The portal closes itself once credentials are applied, and again on
        // its own timeout; either way the panel follows it back to the dashboard.
        if (!wifi_setup_is_active()) {
            ESP_LOGI(TAG, "setup portal closed, back to the dashboard");
            prompt_dismiss();
        } else if (wifi_sta_is_online()) {
            /*
             * The link came up -- either the submitted credentials worked or the
             * old network returned while the form sat untouched. Nothing left to
             * set up, so stop advertising an open AP.
             */
            ESP_LOGI(TAG, "online, closing the setup portal");
            wifi_setup_stop();
            prompt_dismiss();
        }
        break;
    }
}

/** @return true when the press was consumed by the prompt. */
static bool handle_prompt_button(touch_event_t event)
{
    switch (s_ui.prompt) {
    case UI_PROMPT_ASK:
        if (event == TOUCH_EVENT_ENTER) {
            prompt_open_portal();
        } else {
            ESP_LOGI(TAG, "setup offer declined");
            prompt_dismiss();
        }
        return true;

    case UI_PROMPT_PORTAL:
        ESP_LOGI(TAG, "setup cancelled from the panel");
        wifi_setup_stop();
        prompt_dismiss();
        return true;

    case UI_PROMPT_NONE:
    default:
        return false;
    }
}


/* ------------------------------------------------------------------- menu */

/*
 * Three buttons, so the usual three-key idiom: left and right move, enter acts.
 * That applies to the menu's rows. The dashboard has nothing to move between,
 * so there only enter does anything: it opens the menu.
 *
 * The slider is not part of this. It is a direct control for one setting, not
 * a fourth key, so it never moves a cursor or opens anything.
 */
enum {
    MENU_LED,
#if CONFIG_DISPLAY_FRONTLIGHT
    MENU_FRONTLIGHT,
#endif
    MENU_PAD_LED,
    MENU_THEME,
    MENU_UNITS,
    MENU_UPLOAD,
    MENU_REFRESH_RATE,
    MENU_REFRESH,
    MENU_TOUCH_TEST,
    MENU_ABOUT,
    MENU_BACK,
    MENU_ROW_COUNT,
};

/*
 * The renderer draws a fixed-size row array and silently ignores anything past
 * it, which would look like a menu item that simply does not exist.
 */
_Static_assert(MENU_ROW_COUNT <= DISPLAY_MAX_ROWS,
               "menu has more rows than the display renderer will draw");

static char s_about_text[224];

/*
 * Built once, after wifi_sta_init(): the serial is the station MAC and only
 * exists once the driver has started. Calling this any earlier prints a blank
 * where the identity should be.
 */
static void build_about_text(void)
{
    const esp_app_desc_t *app = esp_app_get_description();

    /*
     * The same string the telemetry URL is keyed on, so a board can be matched
     * against its readings on the dashboard by reading it off the panel. Empty
     * only when the Wi-Fi driver never came up, and a blank line there would
     * read as a missing feature rather than a missing prerequisite.
     */
    const char *serial = wifi_sta_serial();
    if (serial == NULL || serial[0] == '\0') {
        serial = "unavailable";
    }

    snprintf(s_about_text, sizeof(s_about_text),
             "%s\n"
             "serial %s\n"
             "firmware %s\n"
             "built %s %s\n"
             "IDF %s\n"
             "\n"
             "Press any pad to go back.",
             app->project_name, serial, app->version, app->date, app->time, app->idf_ver);
}

/*
 * "On  45%" rather than a row of its own: the percentage is not editable from
 * the menu -- the slider owns it -- so it belongs beside the switch it
 * qualifies rather than pretending to be another thing to press enter on. A
 * board with no slider has no adjustable percentage, and shows none.
 */
static char s_led_row_value[16];

static void fill_led_row_value(void)
{
    if (!s_ui.led_on) {
        snprintf(s_led_row_value, sizeof(s_led_row_value), "Off");
    } else if (touch_has_slider()) {
        snprintf(s_led_row_value, sizeof(s_led_row_value), "On  %u%%", s_led_brightness_pct);
    } else {
        snprintf(s_led_row_value, sizeof(s_led_row_value), "On");
    }
}

static void fill_menu(display_values_t *v)
{
    fill_led_row_value();

    v->rows[MENU_LED]        = (display_row_t){ "LED ring",     s_led_row_value };
#if CONFIG_DISPLAY_FRONTLIGHT
    v->rows[MENU_FRONTLIGHT] = (display_row_t){ "Frontlight",
                                                FRONTLIGHT_LEVELS[s_ui.frontlight].name };
#endif
    v->rows[MENU_PAD_LED]    = (display_row_t){ "Pad LEDs",
                                                touch_has_leds()
                                                    ? PAD_LED_TIMEOUTS[s_ui.pad_led].name
                                                    : "None" };
    v->rows[MENU_THEME]      = (display_row_t){ "Theme",        s_ui.dark_theme ? "Dark" : "Light" };
    v->rows[MENU_UNITS]      = (display_row_t){ "Temperature",  s_ui.use_fahrenheit ? "F" : "C" };
    v->rows[MENU_UPLOAD]     = (display_row_t){ "Upload",
                                                cloud_is_enabled()
                                                    ? (wifi_sta_is_online() ? "On" : "No link")
                                                    : "Off" };
    v->rows[MENU_REFRESH_RATE] = (display_row_t){ "Refresh rate",
                                                  REFRESH_RATES[s_ui.refresh].name };
    v->rows[MENU_REFRESH]    = (display_row_t){ "Full refresh", NULL };
    v->rows[MENU_TOUCH_TEST] = (display_row_t){ "Touch test",   NULL };
    v->rows[MENU_ABOUT]      = (display_row_t){ "About",        NULL };
    v->rows[MENU_BACK]       = (display_row_t){ "Back",         NULL };

    v->row_count    = MENU_ROW_COUNT;
    v->selected_row = s_ui.menu_row;
    v->menu_title   = "MENU";
}

static void touch_test_begin(void);

static void menu_activate(void)
{
    switch (s_ui.menu_row) {
    case MENU_LED:
        s_ui.led_on = !s_ui.led_on;
        break;

#if CONFIG_DISPLAY_FRONTLIGHT
    case MENU_FRONTLIGHT:
        // Cycles Off -> Low -> Med -> High -> Off, applied immediately so the
        // pad press is confirmed by the panel getting brighter under your hand
        // rather than by the row text alone.
        s_ui.frontlight = (uint8_t)((s_ui.frontlight + 1) % FRONTLIGHT_LEVEL_COUNT);
        apply_frontlight();
        break;
#endif

    case MENU_PAD_LED:
        // Cycles Always on -> 5 s -> 15 s -> 30 s. A part without LEDs keeps
        // showing "None" and the press does nothing.
        if (touch_has_leds()) {
            s_ui.pad_led = (uint8_t)((s_ui.pad_led + 1) % PAD_LED_TIMEOUT_COUNT);
            apply_pad_led_timeout();
        }
        break;

    case MENU_THEME:
        // Every pixel flips, which a partial refresh would leave ghosted; take
        // the one full refresh now rather than living with the afterimage.
        s_ui.dark_theme = !s_ui.dark_theme;
        display_request_full_refresh();
        break;

    case MENU_UNITS:
        s_ui.use_fahrenheit = !s_ui.use_fahrenheit;
        break;

    case MENU_UPLOAD:
        // Enabling posts immediately rather than at the end of the interval,
        // so the row's "On" is confirmed by a log line within a second or two.
        cloud_set_enabled(!cloud_is_enabled());
        break;

    case MENU_REFRESH_RATE:
        // Cycles 5 s -> 10 s -> 30 s -> 1 min. Takes effect from the next wait,
        // which starts as soon as this press has been painted.
        s_ui.refresh = (uint8_t)((s_ui.refresh + 1) % REFRESH_RATE_COUNT);
        ESP_LOGI(TAG, "dashboard refresh every %s", REFRESH_RATES[s_ui.refresh].name);
        break;

    case MENU_REFRESH:
        display_request_full_refresh();
        s_ui.screen = DISPLAY_SCREEN_HOME;
        break;

    case MENU_TOUCH_TEST:
        touch_test_begin();
        break;

    case MENU_ABOUT:
        s_ui.screen = DISPLAY_SCREEN_INFO;
        break;

    case MENU_BACK:
    default:
        s_ui.screen = DISPLAY_SCREEN_HOME;
        break;
    }
}

/* -------------------------------------------------------------------- clock */

/*
 * Wall time comes from SNTP once the station is online; there is no RTC
 * battery, so until the first sync the clock reads as dashes rather than as
 * 1 JAN 1970. Anything before 2024 is taken to mean "never synced".
 */
#define CLOCK_VALID_AFTER   1704067200      // 2024-01-01T00:00:00Z

static void clock_start(void)
{
    setenv("TZ", CONFIG_CLOCK_TZ, 1);
    tzset();

    // Needs the netif layer up, so only after wifi_sta_init(). lwIP keeps
    // retrying on its own until the link is there, then resyncs periodically.
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_CLOCK_NTP_SERVER);
    const esp_err_t err = esp_netif_sntp_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sntp start failed: %s", esp_err_to_name(err));
    }
}

/* "02 NOV 18:00". Only ui_task calls this, so one static buffer will do. */
static const char *clock_text(void)
{
    static char text[16];

    const time_t now = time(NULL);
    if (now < CLOCK_VALID_AFTER) {
        return "-- --- --:--";
    }

    struct tm local;
    localtime_r(&now, &local);
    strftime(text, sizeof(text), "%d %b %H:%M", &local);

    for (char *c = text; *c != '\0'; c++) {
        *c = (char)toupper((unsigned char)*c);
    }
    return text;
}


/* ------------------------------------------------------------ notifications */

/*
 * How long a notification covers the readings grid: long enough to read after
 * the hand has moved away, short enough that it is gone by the time anyone
 * looks again. ui_wait_ms() wakes the loop when it runs out, so at a slow
 * refresh rate it still leaves on time rather than at the next scheduled frame.
 */
#define NOTIFY_MS  15000

/*
 * One slot, newest wins: a second notification arriving inside the first's
 * window replaces it and restarts the clock. Only ui_task touches these.
 */
static char    s_notify_text[32];
static int64_t s_notify_since_us;

static void notify(const char *text)
{
    snprintf(s_notify_text, sizeof(s_notify_text), "%s", text);
    s_notify_since_us = esp_timer_get_time();
}

static const char *notify_current(void)
{
    if (s_notify_since_us == 0 ||
        esp_timer_get_time() - s_notify_since_us >= (int64_t)NOTIFY_MS * 1000) {
        return NULL;
    }
    return s_notify_text;
}

/*
 * The link going down is announced once, as it happens, rather than held on
 * screen for as long as it lasts -- a notification covers the readings, and an
 * AP that stays away for an hour must not hide them for an hour. The lasting
 * state is the "NO WIFI" marker on the top line, and the purple ring.
 *
 * Starts "down", so a board that boots with no AP in range says nothing here.
 */
static bool s_link_was_up;

static void update_link_notice(void)
{
    const bool up = wifi_sta_is_online();

    if (s_link_was_up && !up) {
        notify(cloud_is_enabled() ? "Wi-Fi offline - retrying" : "Wi-Fi offline");
    }
    s_link_was_up = up;
}


/* --------------------------------------------------------------- touch test */

/*
 * A guided check of the touch panel, started from the menu, for bringing up a
 * board or chasing a pad that "sometimes doesn't work".
 *
 *   1. Idle     the chip answers and every pad reads released. The press
 *               that started the test is given TT_RELEASE_MS to lift; a pad
 *               still down after that is reported stuck.
 *   2..4.       left, middle, right, one at a time, each within TT_STEP_MS.
 *               A different pad answering is noted -- crosstalk or a mixed-up
 *               layout -- but does not fail the step.
 *   5. Slider   only on a part that has one: one swipe has to cover both
 *               ends of the strip, TT_SLIDER_LOW and TT_SLIDER_HIGH.
 *
 * Pass/fail alone does not tell a pad that barely made it from one with room
 * to spare, so the test also reads each pad's delta count -- its signal --
 * against the threshold the part judges a touch by, live on the page and as
 * the best seen per pad in the result. These go through touch.h like
 * everything else here, so the test is the same on every supported part.
 *
 * The result stays on the panel, and goes to the log along with a full
 * register dump, until any pad is pressed. While the test runs it owns every pad, so nothing can open the menu
 * or walk it by accident.
 *
 * The test changes nothing it does not put back: the pad LEDs are held lit for
 * its duration (a dark panel spends the first touch on waking, which would
 * read as a missed press), and the LED bar brightness the swipe moves is
 * restored at the end.
 */
#define TT_RELEASE_MS   3000
#define TT_STEP_MS      10000
#define TT_SLIDER_MS    15000
#define TT_TICK_MS      1000    ///< repaint cadence while running, for the countdown
#define TT_SLIDER_LOW   15
#define TT_SLIDER_HIGH  85

typedef enum {
    TT_OFF = 0,
    TT_IDLE,
    TT_LEFT,
    TT_MIDDLE,
    TT_RIGHT,
    TT_SLIDER,
    TT_DONE,
} tt_step_t;

typedef enum {
    TT_PENDING = 0,
    TT_PASS,
    TT_FAIL,
    TT_SKIPPED,
} tt_result_t;

/* Owned by ui_task, apart from the slider range above. */
static struct {
    tt_step_t   step;
    int64_t     step_since_us;
    tt_result_t result[TT_DONE];    ///< indexed by step
    uint8_t     stray[TT_DONE];     ///< TOUCH_PAD_* bits that answered instead
    uint8_t     stuck;              ///< pads still down when the idle check gave up
    bool        no_response;        ///< the chip never answered
    uint8_t     saved_brightness;

    bool        signal_ok;          ///< the last delta read succeeded
    int8_t      delta[TOUCH_BUTTON_COUNT];      ///< live, left first
    int8_t      peak[TOUCH_BUTTON_COUNT];       ///< best seen during that pad's step
    bool        have_threshold;
    uint8_t     threshold[TOUCH_BUTTON_COUNT];
} s_tt;

/* Which button a pad step is asking for, as an index into delta[]. */
static int tt_step_pad(tt_step_t step)
{
    switch (step) {
    case TT_LEFT:   return 0;
    case TT_MIDDLE: return 1;
    case TT_RIGHT:  return 2;
    default:        return -1;
    }
}

/*
 * Read the live deltas, and fold the one being asked for into its peak.
 * Sampled every pass and again on the press itself, while the finger is still
 * down, so a quick tap between two passes still leaves its mark.
 */
static void tt_sample_signal(void)
{
    s_tt.signal_ok = (touch_read_button_deltas(s_tt.delta) == ESP_OK);
    if (!s_tt.signal_ok) {
        return;
    }

    const int pad = tt_step_pad(s_tt.step);
    if (pad >= 0 && s_tt.delta[pad] > s_tt.peak[pad]) {
        s_tt.peak[pad] = s_tt.delta[pad];
    }
}

static char s_tt_text[480];

static bool touch_test_active(void)
{
    return s_tt.step != TT_OFF;
}

static uint32_t tt_elapsed_ms(void)
{
    return (uint32_t)((esp_timer_get_time() - s_tt.step_since_us) / 1000);
}

/* "L M -" -- the three buttons, in panel order, as the instructions name them. */
static void tt_pad_letters(uint8_t pads, char out[6])
{
    out[0] = (pads & TOUCH_PAD_LEFT)  ? 'L' : '-';
    out[1] = ' ';
    out[2] = (pads & TOUCH_PAD_ENTER) ? 'M' : '-';
    out[3] = ' ';
    out[4] = (pads & TOUCH_PAD_RIGHT) ? 'R' : '-';
    out[5] = '\0';
}

static void tt_enter(tt_step_t step)
{
    s_tt.step = step;
    s_tt.step_since_us = esp_timer_get_time();

    if (step == TT_SLIDER) {
        s_tt_slider_min = 100;
        s_tt_slider_max = 0;
        s_tt_slider_tracking = true;
    }
}

static const char *tt_result_name(tt_result_t r)
{
    switch (r) {
    case TT_PASS:    return "OK";
    case TT_FAIL:    return "FAIL";
    case TT_SKIPPED: return "skipped";
    default:         return "-";
    }
}

/** Every step that ran passed. */
static bool tt_passed(void)
{
    for (int i = TT_IDLE; i < TT_DONE; i++) {
        if (s_tt.result[i] == TT_FAIL) {
            return false;
        }
    }
    return true;
}

/** Put back what the test borrowed. Safe to call more than once. */
static void tt_restore(void)
{
    s_tt_slider_tracking = false;

    if (touch_has_slider() && s_led_brightness_pct != s_tt.saved_brightness) {
        s_led_brightness_pct = s_tt.saved_brightness;
        ws2812_set_brightness(brightness_scale(s_tt.saved_brightness));
        ws2812_refresh();
    }

    apply_pad_led_timeout();
}

static void tt_finish(void)
{
    tt_restore();

    // The full picture for whoever reads the log: thresholds, gain, noise
    // flags, base counts. Advisory, so a failed dump changes no result.
    if (!s_tt.no_response) {
        (void)touch_dump_registers();
    }

    ESP_LOGI(TAG, "touch test peaks (delta/threshold): left %d/%u, middle %d/%u, right %d/%u",
             s_tt.peak[0], s_tt.threshold[0], s_tt.peak[1], s_tt.threshold[1],
             s_tt.peak[2], s_tt.threshold[2]);

    ESP_LOGI(TAG, "touch test %s: idle %s%s, left %s, middle %s, right %s, slider %s (%u-%u%%)",
             tt_passed() ? "PASS" : "FAIL",
             tt_result_name(s_tt.result[TT_IDLE]),
             s_tt.no_response ? " (no response)" : (s_tt.stuck ? " (pad stuck)" : ""),
             tt_result_name(s_tt.result[TT_LEFT]),
             tt_result_name(s_tt.result[TT_MIDDLE]),
             tt_result_name(s_tt.result[TT_RIGHT]),
             tt_result_name(s_tt.result[TT_SLIDER]),
             s_tt_slider_min, s_tt_slider_max);

    s_tt.step = TT_DONE;
}

static void tt_advance(void)
{
    switch (s_tt.step) {
    case TT_IDLE:   tt_enter(TT_LEFT);   break;
    case TT_LEFT:   tt_enter(TT_MIDDLE); break;
    case TT_MIDDLE: tt_enter(TT_RIGHT);  break;
    case TT_RIGHT:
        if (touch_has_slider()) {
            tt_enter(TT_SLIDER);
        } else {
            s_tt.result[TT_SLIDER] = TT_SKIPPED;
            tt_finish();
        }
        break;
    case TT_SLIDER:
        s_tt_slider_tracking = false;
        tt_finish();
        break;
    default:
        break;
    }
}

static void tt_settle(tt_result_t result)
{
    s_tt.result[s_tt.step] = result;
    tt_advance();
}

static bool tt_slider_covered(void)
{
    return s_tt_slider_min <= TT_SLIDER_LOW && s_tt_slider_max >= TT_SLIDER_HIGH;
}

static void touch_test_begin(void)
{
    memset(&s_tt, 0, sizeof(s_tt));
    s_tt.saved_brightness = s_led_brightness_pct;

    // Held lit, so no touch is spent waking a dark panel. See the note above.
    touch_led_idle_timeout(0);

    // Configuration, not live state: read once. Missing thresholds just leave
    // the signal shown without its yardstick.
    s_tt.have_threshold = (touch_read_button_thresholds(s_tt.threshold) == ESP_OK);

    s_ui.screen = DISPLAY_SCREEN_INFO;
    tt_enter(TT_IDLE);
    ESP_LOGI(TAG, "touch test started (%s)", TOUCH_IC_NAME);
}

/** Time-driven half of the test; called once per ui_task pass. */
static void touch_test_poll(void)
{
    if (!touch_test_active()) {
        return;
    }

    /*
     * Something else took the screen -- the Wi-Fi offer is the only thing that
     * can. Abandon quietly rather than leave a half-run test owning the pads
     * behind a page nobody can see.
     */
    if (s_ui.screen != DISPLAY_SCREEN_INFO) {
        tt_restore();
        s_tt.step = TT_OFF;
        ESP_LOGI(TAG, "touch test abandoned");
        return;
    }

    if (s_tt.step != TT_DONE) {
        tt_sample_signal();
    }

    const uint32_t elapsed = tt_elapsed_ms();

    switch (s_tt.step) {
    case TT_IDLE:
        if (touch_pads_valid() && touch_pads() == 0) {
            tt_settle(TT_PASS);
        } else if (elapsed >= TT_RELEASE_MS) {
            if (!touch_pads_valid()) {
                // Nothing after this can pass; say so now rather than make
                // someone sit through three timeouts to learn it.
                s_tt.no_response = true;
                s_tt.result[TT_IDLE] = TT_FAIL;
                for (int i = TT_LEFT; i < TT_DONE; i++) {
                    s_tt.result[i] = TT_SKIPPED;
                }
                tt_finish();
            } else {
                s_tt.stuck = touch_pads();
                tt_settle(TT_FAIL);
            }
        }
        break;

    case TT_LEFT:
    case TT_MIDDLE:
    case TT_RIGHT:
        if (elapsed >= TT_STEP_MS) {
            tt_settle(TT_FAIL);
        }
        break;

    case TT_SLIDER:
        if (tt_slider_covered()) {
            tt_settle(TT_PASS);
        } else if (elapsed >= TT_SLIDER_MS) {
            tt_settle(TT_FAIL);
        }
        break;

    default:
        break;
    }
}

/** Event-driven half: every pad comes here while the test is running. */
static void touch_test_event(touch_event_t event)
{
    static const struct {
        tt_step_t     step;
        touch_event_t event;
        uint8_t       pad;
    } PADS[] = {
        { TT_LEFT,   TOUCH_EVENT_LEFT,  TOUCH_PAD_LEFT },
        { TT_MIDDLE, TOUCH_EVENT_ENTER, TOUCH_PAD_ENTER },
        { TT_RIGHT,  TOUCH_EVENT_RIGHT, TOUCH_PAD_RIGHT },
    };

    switch (s_tt.step) {
    case TT_DONE:
        // Any pad, slider included, closes the result.
        s_tt.step = TT_OFF;
        s_ui.screen = DISPLAY_SCREEN_MENU;
        return;

    case TT_LEFT:
    case TT_MIDDLE:
    case TT_RIGHT:
        for (size_t i = 0; i < sizeof(PADS) / sizeof(PADS[0]); i++) {
            if (PADS[i].event != event) {
                continue;
            }
            if (PADS[i].step == s_tt.step) {
                tt_sample_signal();     // finger still down: catch its peak
                tt_settle(TT_PASS);
            } else {
                s_tt.stray[s_tt.step] |= PADS[i].pad;
            }
            return;
        }
        return;     // a slider gesture is not an answer here

    case TT_SLIDER:
        // The callback has the range already; the end of a gesture is just
        // the moment to judge it, rather than waiting for the next tick.
        if (event == TOUCH_EVENT_SLIDER && tt_slider_covered()) {
            tt_settle(TT_PASS);
        }
        return;

    case TT_IDLE:
    default:
        return;     // nothing asked for yet
    }
}

static int tt_append(int n, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static int tt_append(int n, const char *fmt, ...)
{
    if (n < 0 || (size_t)n >= sizeof(s_tt_text)) {
        return n;
    }
    va_list args;
    va_start(args, fmt);
    const int wrote = vsnprintf(s_tt_text + n, sizeof(s_tt_text) - (size_t)n, fmt, args);
    va_end(args);
    return (wrote < 0) ? n : n + wrote;
}

/* One line of the result page, e.g. "Left    OK  peak 87/64, also saw R". */
static int tt_append_pad_result(int n, const char *label, tt_step_t step)
{
    n = tt_append(n, "%-8s%-5s", label, tt_result_name(s_tt.result[step]));
    if (s_tt.result[step] != TT_SKIPPED) {
        // The peak is what separates a dead pad (near 0) from a weak one
        // (close under the threshold) when a step fails.
        const int pad = tt_step_pad(step);
        if (s_tt.have_threshold) {
            n = tt_append(n, " peak %d/%u", s_tt.peak[pad], s_tt.threshold[pad]);
        } else {
            n = tt_append(n, " peak %d", s_tt.peak[pad]);
        }
    }
    if (s_tt.stray[step]) {
        char pads[6];
        tt_pad_letters(s_tt.stray[step], pads);
        n = tt_append(n, ", also saw %s", pads);
    }
    return tt_append(n, "\n");
}

/*
 * "Signal L 3 M 71 R -1  of 64" -- live deltas, and the threshold they have to
 * reach when the thresholds share one value, which is the usual case. Mixed
 * thresholds get a line of their own rather than a misleading single number.
 */
static int tt_append_signal(int n)
{
    if (!s_tt.signal_ok) {
        return tt_append(n, "Signal: (no answer)\n");
    }

    n = tt_append(n, "Signal L %d M %d R %d", s_tt.delta[0], s_tt.delta[1], s_tt.delta[2]);

    if (!s_tt.have_threshold) {
        return tt_append(n, "\n");
    }
    if (s_tt.threshold[0] == s_tt.threshold[1] && s_tt.threshold[1] == s_tt.threshold[2]) {
        return tt_append(n, "  of %u\n", s_tt.threshold[0]);
    }
    return tt_append(n, "\nThreshold L %u M %u R %u\n",
                     s_tt.threshold[0], s_tt.threshold[1], s_tt.threshold[2]);
}

/* Page text for the current step. Built every pass, so the countdown moves. */
static const char *touch_test_text(void)
{
    static const char *const INSTRUCTION[TT_DONE] = {
        [TT_LEFT]   = "Touch the LEFT pad.",
        [TT_MIDDLE] = "Touch the MIDDLE pad.",
        [TT_RIGHT]  = "Touch the RIGHT pad.",
        [TT_SLIDER] = "Slide one finger along the whole strip, end to end.",
    };

    const int total = touch_has_slider() ? 4 : 3;
    int n = 0;

    if (s_tt.step == TT_DONE) {
        n = tt_append(n, "TOUCH TEST: %s\n\n", tt_passed() ? "PASS" : "FAIL");

        n = tt_append(n, "%-8s", "Idle");
        if (s_tt.no_response) {
            n = tt_append(n, "FAIL - %s not answering\n", TOUCH_IC_NAME);
        } else if (s_tt.stuck) {
            char pads[6];
            tt_pad_letters(s_tt.stuck, pads);
            n = tt_append(n, "FAIL - stuck: %s%s\n", pads,
                          (s_tt.stuck & TOUCH_PADS_SLIDER) ? " +slider" : "");
        } else {
            n = tt_append(n, "OK, %s answering\n", TOUCH_IC_NAME);
        }

        n = tt_append_pad_result(n, "Left",   TT_LEFT);
        n = tt_append_pad_result(n, "Middle", TT_MIDDLE);
        n = tt_append_pad_result(n, "Right",  TT_RIGHT);

        if (touch_has_slider()) {
            n = tt_append(n, "%-8s%s", "Slider", tt_result_name(s_tt.result[TT_SLIDER]));
            if (s_tt.result[TT_SLIDER] != TT_SKIPPED) {
                if (s_tt_slider_max >= s_tt_slider_min) {
                    n = tt_append(n, " - covered %u-%u%%", s_tt_slider_min, s_tt_slider_max);
                } else {
                    n = tt_append(n, " - never touched");
                }
            }
            n = tt_append(n, "\n");
        }

        tt_append(n, "\nAny pad: back to the menu.");
        return s_tt_text;
    }

    const uint32_t elapsed = tt_elapsed_ms();
    char pads[6];
    tt_pad_letters(touch_pads(), pads);

    if (s_tt.step == TT_IDLE) {
        n = tt_append(n, "TOUCH TEST\n\nChecking the panel.\nLift your finger off every pad.\n\n");
        n = tt_append(n, "Touched now: %s\n", touch_pads_valid() ? pads : "(no answer)");
        tt_append_signal(n);
        return s_tt_text;
    }

    const uint32_t limit = (s_tt.step == TT_SLIDER) ? TT_SLIDER_MS : TT_STEP_MS;
    const uint32_t left_s = (elapsed >= limit) ? 0 : (limit - elapsed + 999) / 1000;

    n = tt_append(n, "TOUCH TEST  step %d of %d\n\n%s\n\n",
                  (int)(s_tt.step - TT_IDLE), total, INSTRUCTION[s_tt.step]);
    n = tt_append(n, "Touched now: %s\n", touch_pads_valid() ? pads : "(no answer)");
    n = tt_append_signal(n);

    if (s_tt.step == TT_SLIDER) {
        if (s_tt_slider_max >= s_tt_slider_min) {
            n = tt_append(n, "Covered so far: %u-%u%%\n", s_tt_slider_min, s_tt_slider_max);
        } else {
            n = tt_append(n, "Covered so far: nothing\n");
        }
    } else if (s_tt.stray[s_tt.step]) {
        char stray[6];
        tt_pad_letters(s_tt.stray[s_tt.step], stray);
        n = tt_append(n, "Wrong pad answered: %s\n", stray);
    }

    tt_append(n, "Time left: %lu s\n", (unsigned long)left_s);
    return s_tt_text;
}

static void handle_event(touch_event_t event)
{
    // The test owns every pad while it runs -- unless the Wi-Fi offer has
    // taken the screen, in which case touch_test_poll() is about to abandon it.
    if (touch_test_active() && s_ui.prompt == UI_PROMPT_NONE) {
        touch_test_event(event);
        return;
    }

    if (event == TOUCH_EVENT_SLIDER) {
        /*
         * A finished slider gesture, not a key. It must not open the menu or answer the Wi-Fi offer -- someone reaching for the
         * brightness has not agreed to anything. The strip already followed the
         * finger from the touch task; all that is left is to say so on the
         * panel, once.
         */
        char text[24];
        snprintf(text, sizeof(text), "LED bar %u%%", touch_slider_percent_last());
        notify(text);
        return;
    }

    // The offer owns the whole screen while it is up, so it owns the buttons too.
    if (handle_prompt_button(event)) {
        return;
    }

    switch (s_ui.screen) {
    case DISPLAY_SCREEN_HOME:
        // Left and right have nothing to move between here, so they are ignored.
        if (event != TOUCH_EVENT_LEFT && event != TOUCH_EVENT_RIGHT) {
            s_ui.screen = DISPLAY_SCREEN_MENU;
            s_ui.menu_row = 0;
        }
        break;

    case DISPLAY_SCREEN_MENU:
        if (event == TOUCH_EVENT_LEFT) {
            s_ui.menu_row = (uint8_t)((s_ui.menu_row + MENU_ROW_COUNT - 1) % MENU_ROW_COUNT);
        } else if (event == TOUCH_EVENT_RIGHT) {
            s_ui.menu_row = (uint8_t)((s_ui.menu_row + 1) % MENU_ROW_COUNT);
        } else {
            menu_activate();
        }
        break;

    case DISPLAY_SCREEN_INFO:
    default:
        s_ui.screen = DISPLAY_SCREEN_MENU;   // About is read-only: any pad leaves
        break;
    }
}

/** Overlay the UI state onto a freshly snapshotted set of readings. */
static void apply_ui(display_values_t *v)
{
    v->use_fahrenheit = s_ui.use_fahrenheit;
    v->dark_theme     = s_ui.dark_theme;

    if (s_ui.prompt != UI_PROMPT_NONE) {
        // Rendered as an ordinary wrapped-text page; the display component has
        // no notion of a prompt and does not need one.
        v->screen = DISPLAY_SCREEN_INFO;
        v->info_text = s_prompt_text;
        return;
    }

    v->screen = s_ui.screen;

    switch (s_ui.screen) {
    case DISPLAY_SCREEN_HOME:
        v->clock_text    = clock_text();
        v->status_text   = wifi_sta_is_online() ? NULL : "NO WIFI";
        v->snackbar_text = notify_current();
        break;
    case DISPLAY_SCREEN_MENU:
        fill_menu(v);
        break;
    case DISPLAY_SCREEN_INFO:
    default:
        v->info_text = touch_test_active() ? touch_test_text() : s_about_text;
        break;
    }
}

/*
 * How long ui_task may sleep before it has to repaint: the chosen refresh
 * interval, cut short by anything on screen that is due to change sooner.
 */
static uint32_t ui_wait_ms(void)
{
    uint32_t wait_ms = REFRESH_RATES[s_ui.refresh].period_ms;

    /*
     * The offer has its own timeout and follows the link state, so while it is
     * up -- or could still be made, see update_wifi_prompt() -- the loop keeps
     * the base tick it always had rather than lingering for a minute. A board
     * that came online never asks, and drops straight to the chosen rate.
     */
    const bool offer_pending = s_wifi_ready && !s_boot_ask_done && !wifi_sta_is_online();
    if (s_ui.prompt != UI_PROMPT_NONE || offer_pending) {
        wait_ms = DISPLAY_TICK_MS < wait_ms ? DISPLAY_TICK_MS : wait_ms;
    }

    // A running touch test counts down on screen.
    if (touch_test_active() && s_tt.step != TT_DONE && TT_TICK_MS < wait_ms) {
        wait_ms = TT_TICK_MS;
    }

    // A notification leaves the moment it expires, not a frame later.
    if (s_notify_since_us != 0) {
        const int64_t left_ms =
            NOTIFY_MS - (esp_timer_get_time() - s_notify_since_us) / 1000;
        if (left_ms > 0 && (uint64_t)left_ms < wait_ms) {
            wait_ms = (uint32_t)left_ms + 1;    // land just past the deadline
        }
    }

    return wait_ms;
}

static void ui_task(void *arg)
{
    while (1) {
        /*
         * One wait serves both jobs: a press wakes it early, otherwise it falls
         * through on the periodic tick and repaints with whatever the sensors
         * have. The boot splash therefore stays up until the first of the two
         * happens rather than being overwritten by an empty dashboard.
         */
        touch_event_t event;
        const bool pressed = touch_event_wait(&event, ui_wait_ms());

        if (pressed) {
            handle_event(event);
        }

        // Runs on every pass, press or tick, so the offer can appear, time out
        // and clear itself without anybody touching the panel.
        update_wifi_prompt();
        update_link_notice();
        touch_test_poll();

        air_monitor_device dev;
        air_monitor_get(&dev);

        display_values_t values;
        snapshot_values(&dev, &values);
        apply_ui(&values);

        /*
         * Periodic frames may be dropped when the worker is still painting --
         * the next tick carries the same readings. A press may not: waiting
         * costs a few hundred milliseconds and is the difference between a
         * button that works and one that looks broken.
         */
        display_update(&values, pressed);
    }
}


/* ------------------------------------------------------------------ startup */

static void buses_init(void)
{
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(uart_init_all());
    ESP_ERROR_CHECK(spi_init());
    ESP_ERROR_CHECK(pwm_init());

    ESP_LOGI(TAG, "buses initialized");
}

void app_main(void)
{
    // The watchdog goes up first so its task is pulsing DONE before the slow
    // device probes start.
    ESP_ERROR_CHECK(tpl5010_init());

    buses_init();

    // pwm_init() leaves the channel at 0% duty, which is already "off"; going
    // through apply_frontlight() puts the saved level on the pin and logs it,
    // so the boot log says what GPIO 3 is doing rather than leaving it implied.
#if CONFIG_DISPLAY_FRONTLIGHT
    apply_frontlight();
#else
    ESP_LOGI(TAG, "no frontlight on this panel; gpio %d held low", PWM_GPIO);
#endif

    ESP_ERROR_CHECK(air_monitor_init());

    // The pads are their own reader: reading the status register clears the
    // latched interrupt, so air_monitor must not poll them as well.
    if (touch_start() != ESP_OK) {
        ESP_LOGE(TAG, "%s touch panel unavailable", TOUCH_IC_NAME);
        /*
         * Treated as a sign the front board is not answering, display
         * included. epaper's busy wait has no timeout, so a BUSY line that
         * never drops would otherwise hold app_main in display_init() forever,
         * and nothing after it -- sensors, wifi, the UI -- would ever start.
         * Must be set before display_init().
         */
        epaper_set_busy_bypass(true);
    } else {
        apply_pad_led_timeout();
    }

    if (ws2812_init() != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed");
    } else {
        /*
         * Put the starting brightness on the strip before anything lights it,
         * so the percentage the menu reports is the one the LEDs are actually
         * running at rather than whatever ws2812.c defaulted to.
         */
        ws2812_set_brightness(brightness_scale(s_led_brightness_pct));

        /*
         * Only now: the callback repaints the strip, so it must not be
         * reachable until there is a strip to repaint. A touch landing in the
         * gap between touch_start() and here is lost, which costs one slider
         * movement during boot and nothing else.
         */
        touch_slider_on_change(touch_has_slider() ? on_slider_moved : NULL);
    }

    /*
     * Paint a boot page and let the panel catch up in the background: the
     * initial full refresh takes ~2 s, which is time better spent starting the
     * sensor tasks. The panel is deliberately left awake from here on -- a
     * partial refresh diffs against the basemap in its RAM, and epaper_sleep()
     * would drop it.
     */
    display_values_t splash;
    display_values_reset(&splash);
    splash.screen = DISPLAY_SCREEN_INFO;
    splash.info_text =
        touch_has_slider()
            ? "Starting up\n\n"
              "Waiting for the first sensor readings.\n\n"
              "The middle pad opens the menu.\n"
              "The slider sets the LED bar brightness."
            : "Starting up\n\n"
              "Waiting for the first sensor readings.\n\n"
              "The middle pad opens the menu.";

    if (display_init(NULL, &splash, true) != ESP_OK) {
        ESP_LOGE(TAG, "display init failed");
    }

    ESP_ERROR_CHECK(air_monitor_start());

    /*
     * Networking comes up last and non-fatally: the monitor's job is the panel
     * and the LED ring, both of which work perfectly well with no AP in range.
     * The station reconnects on its own, and the cloud task simply skips a
     * cycle whenever the link is down.
     */
    esp_err_t err = wifi_sta_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi init failed: %s", esp_err_to_name(err));
    } else {
        // The setup portal reconfigures this station, so it is only offered
        // when there is a station to reconfigure.
        s_wifi_ready = true;
        s_boot_deadline_us = esp_timer_get_time() + (int64_t)WIFI_BOOT_GRACE_MS * 1000;

        clock_start();

        if ((err = cloud_start()) != ESP_OK) {
            ESP_LOGE(TAG, "cloud start failed: %s", esp_err_to_name(err));
        }
    }

    // After wifi_sta_init(), which is what produces the serial, and before
    // ui_task exists to render the page.
    build_about_text();

    xTaskCreate(report_task, "report", 4096, NULL, 3, NULL);
    xTaskCreate(ui_task, "ui", 4096, NULL, 2, NULL);

    /*
     * Confirm the link last, and only once the panel and the LED ring have
     * tasks of their own: this blocks for up to WIFI_VERIFY_MS and the display
     * must not be frozen behind it. Nothing downstream branches on the result
     * -- the point is a definite verdict in the log, with the reason code,
     * instead of a silent start that leaves "is it even trying?" unanswered.
     */
    if (s_wifi_ready) {
        wifi_sta_verify_link(WIFI_VERIFY_MS);
    }

    // app_main returns: its task is deleted. That's fine and normal.
}
