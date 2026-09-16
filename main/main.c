#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "i2c.h"
#include "uart.h"
#include "spi.h"
#include "pwm.h"

#include "air_monitor.h"
#include "aqi.h"
#include "cap1203.h"
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
 * Fast refresh drives the whole panel without the several flickers a full GC
 * waveform needs and finishes in about a second, which is what makes a 5 s
 * cadence possible at all. Every sixth update comes back as a full refresh:
 * the driver enforces the datasheet's EPAPER_MAX_FAST_REFRESHES limit itself.
 */
#define DISPLAY_PERIOD_MS   5000


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
 * Everything the three pads can change. Owned by ui_task; report_task only
 * reads led_on, and a byte read racing a byte write costs at worst one frame
 * of the strip being a cycle late.
 */
static struct {
    display_screen_t screen;
    display_metric_t metric;
    uint8_t          menu_row;
    bool             led_on;
    bool             use_fahrenheit;
    uint8_t          frontlight;    ///< index into FRONTLIGHT_LEVELS
    ui_prompt_t      prompt;
    int64_t          prompt_since_us;
} s_ui = {
    .screen = DISPLAY_SCREEN_HOME,
    .metric = DISPLAY_METRIC_PM25,
    .led_on = true,
    .frontlight = 0,                // off until asked for; see apply_frontlight()
};


/* ------------------------------------------------------- frontlight (IO3) */

/*
 * The panel's frontlight, driven by the LEDC channel pwm.c puts on GPIO 3.
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

        if (cap1203_pads_valid()) {
            const uint8_t pads = cap1203_pads();
            ESP_LOGI(TAG, "CAP1203  pads %c%c%c  (t=%lu ms)",
                     (pads & CAP1203_PAD1) ? 'L' : '-',
                     (pads & CAP1203_PAD2) ? 'E' : '-',
                     (pads & CAP1203_PAD3) ? 'R' : '-',
                     (unsigned long)cap1203_pads_updated_ms());
        } else {
            ESP_LOGW(TAG, "CAP1203  read failed");
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
        // Field by field: the bound part decides which of these it actually
        // measured, and one it did not stays at its "--" sentinel.
        if (pm_has(&dev->pm, PM_FIELD_PM1_0)) {
            v->pm1_ugm3 = dev->pm.pm1_0_ugm3;
        }
        if (pm_has(&dev->pm, PM_FIELD_PM2_5)) {
            v->pm25_ugm3 = dev->pm.pm2_5_ugm3;
        }
        if (pm_has(&dev->pm, PM_FIELD_PM10)) {
            v->pm10_ugm3 = dev->pm.pm10_ugm3;
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
        v->altitude_m   = dev->altitude_m;
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
static bool handle_prompt_button(cap1203_button_t button)
{
    switch (s_ui.prompt) {
    case UI_PROMPT_ASK:
        if (button == CAP1203_BUTTON_ENTER) {
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
 * Three pads, so the usual three-key idiom: left and right move, enter acts.
 * On the dashboard that means walking the highlight along the readings; in the
 * menu it means walking the rows.
 */
enum {
    MENU_LED,
    MENU_FRONTLIGHT,
    MENU_UNITS,
    MENU_HIGHLIGHT,
    MENU_UPLOAD,
    MENU_REFRESH,
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

static void fill_menu(display_values_t *v)
{
    v->rows[MENU_LED]        = (display_row_t){ "LED ring",     s_ui.led_on ? "On" : "Off" };
    v->rows[MENU_FRONTLIGHT] = (display_row_t){ "Frontlight",
                                                FRONTLIGHT_LEVELS[s_ui.frontlight].name };
    v->rows[MENU_UNITS]      = (display_row_t){ "Temperature",  s_ui.use_fahrenheit ? "F" : "C" };
    v->rows[MENU_HIGHLIGHT]  = (display_row_t){ "Highlight",
                                                s_ui.metric == DISPLAY_METRIC_NONE ? "Off" : "On" };
    v->rows[MENU_UPLOAD]     = (display_row_t){ "Upload",
                                                cloud_is_enabled()
                                                    ? (wifi_sta_is_online() ? "On" : "No link")
                                                    : "Off" };
    v->rows[MENU_REFRESH]    = (display_row_t){ "Full refresh", NULL };
    v->rows[MENU_ABOUT]      = (display_row_t){ "About",        NULL };
    v->rows[MENU_BACK]       = (display_row_t){ "Back",         NULL };

    v->row_count    = MENU_ROW_COUNT;
    v->selected_row = s_ui.menu_row;
    v->menu_title   = "MENU";
}

static void menu_activate(void)
{
    switch (s_ui.menu_row) {
    case MENU_LED:
        s_ui.led_on = !s_ui.led_on;
        break;

    case MENU_FRONTLIGHT:
        // Cycles Off -> Low -> Med -> High -> Off, applied immediately so the
        // pad press is confirmed by the panel getting brighter under your hand
        // rather than by the row text alone.
        s_ui.frontlight = (uint8_t)((s_ui.frontlight + 1) % FRONTLIGHT_LEVEL_COUNT);
        apply_frontlight();
        break;

    case MENU_UNITS:
        s_ui.use_fahrenheit = !s_ui.use_fahrenheit;
        break;

    case MENU_HIGHLIGHT:
        // Toggling off parks the metric; toggling on restores the headline,
        // which is where the left/right walk naturally starts.
        s_ui.metric = (s_ui.metric == DISPLAY_METRIC_NONE) ? DISPLAY_METRIC_PM25
                                                           : DISPLAY_METRIC_NONE;
        break;

    case MENU_UPLOAD:
        // Enabling posts immediately rather than at the end of the interval,
        // so the row's "On" is confirmed by a log line within a second or two.
        cloud_set_enabled(!cloud_is_enabled());
        break;

    case MENU_REFRESH:
        display_request_full_refresh();
        s_ui.screen = DISPLAY_SCREEN_HOME;
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

static void handle_button(cap1203_button_t button)
{
    // The offer owns the whole screen while it is up, so it owns the pads too.
    if (handle_prompt_button(button)) {
        return;
    }

    switch (s_ui.screen) {
    case DISPLAY_SCREEN_HOME:
        if (button == CAP1203_BUTTON_LEFT) {
            s_ui.metric = display_metric_next(s_ui.metric, -1);
        } else if (button == CAP1203_BUTTON_RIGHT) {
            s_ui.metric = display_metric_next(s_ui.metric, +1);
        } else {
            s_ui.screen = DISPLAY_SCREEN_MENU;
            s_ui.menu_row = 0;
        }
        break;

    case DISPLAY_SCREEN_MENU:
        if (button == CAP1203_BUTTON_LEFT) {
            s_ui.menu_row = (uint8_t)((s_ui.menu_row + MENU_ROW_COUNT - 1) % MENU_ROW_COUNT);
        } else if (button == CAP1203_BUTTON_RIGHT) {
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
        v->highlight = s_ui.metric;
        if (!wifi_sta_is_online()) {
            /*
             * The footer otherwise names the sensors that are not reporting;
             * the snackbar takes it over while offline, which is the right
             * trade because a dead link explains missing cloud data in a way
             * no per-sensor status can. Matches the purple ring.
             */
            v->snackbar_text = cloud_is_enabled() ? "Wi-Fi offline - retrying"
                                                  : "Wi-Fi offline";
        }
        break;
    case DISPLAY_SCREEN_MENU:
        fill_menu(v);
        break;
    case DISPLAY_SCREEN_INFO:
    default:
        v->info_text = s_about_text;
        break;
    }
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
        cap1203_button_t button;
        const bool pressed = cap1203_button_wait(&button, DISPLAY_PERIOD_MS);

        if (pressed) {
            handle_button(button);
        }

        // Runs on every pass, press or tick, so the offer can appear, time out
        // and clear itself without anybody touching the panel.
        update_wifi_prompt();

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
    apply_frontlight();

    ESP_ERROR_CHECK(air_monitor_init());

    // The pads are their own reader now: cap1203_read() clears the latched
    // interrupt, so air_monitor must not poll them as well.
    if (cap1203_buttons_start() != ESP_OK) {
        ESP_LOGE(TAG, "touch buttons unavailable");
    }

    if (ws2812_init() != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed");
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
    splash.info_text = "Starting up\n\n"
                       "Waiting for the first sensor readings.\n\n"
                       "Left / right pads move the highlight, the middle pad opens the menu.";

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
