/*
 * Display service -- see epaper_display.h for what this is and where it came from.
 *
 * The division of labour is the reference's: render_frame() paints the whole
 * screen into a software framebuffer in about a millisecond, the worker task
 * pushes that framebuffer at the panel over the next 0.4 s to 2 s, and the
 * refresh tier is picked from what actually changed between two snapshots.
 *
 * Orientation is not this file's problem: it draws in canvas coordinates and
 * gfx.c maps them onto the panel (see the orientation block in gfx.h).
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "epaper_display.h"
#include "gfx.h"
#include "aqi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "epaper_display";

/* --- Defaults (epaper_display.h: display_config_t) --- */
#define DEFAULT_TASK_STACK      3072
#define DEFAULT_TASK_PRIORITY   4

/* --- Layout (landscape canvas, GFX_W x GFX_H = 250 x 122) -----------------
 *
 *   +--------------------------------------------------------------+
 *   | AIR QUALITY                                                ! |  title
 *   +----------------------+---------------------------------------+
 *   | PM2.5 ug/m3          |  CO2                          850 ppm |
 *   |         123          |  TEMP                          24.5 C |
 *   |      UNHEALTHY       |  HUMID                         55.0 % |
 *   |       AQI 185        |  VOC                            31000 |
 *   |----------------------|  NOX                            16000 |
 *   | PM1.0          98 ug |  PRESS                     1013.2 hPa |
 *   | PM10          141 ug |  ALT                             45 m |
 *   +--------------------------------------------------------------+
 *   | offline: PMS CO2                                             |  footer
 *   +--------------------------------------------------------------+
 *
 * The reference splits the panel into a frozen header and a body band so a
 * partial refresh can redraw the body alone. That split does not survive the
 * rotation: the driver refreshes whole panel rows, and panel rows run down the
 * canvas's x axis once the image is turned a quarter turn, so a band would be
 * a column of this layout rather than a strip of it. Partial refreshes here
 * cover the whole canvas instead -- still differential, still no flash, they
 * just carry every changed pixel rather than a chosen band.
 */
#define STATUS_H        13
#define MARGIN_X        2

#define COL_L_X         MARGIN_X
#define COL_L_RIGHT     120
#define DIVIDER_X       124
#define COL_R_X         130
#define COL_R_RIGHT     (GFX_W - MARGIN_X)

#define ROW_PITCH       13
#define LINE_PITCH      10
#define FOOTER_Y        (GFX_H - GFX_FONT_H - 3)

/** Characters that fit on one scale-1 line inside the margins. */
#define WRAP_COLS       ((GFX_W - 2 * MARGIN_X) / GFX_ADVANCE)

/** Refresh tier. */
typedef enum {
    REFRESH_FULL,       ///< Full GC waveform. Flashes, repaints both RAM planes.
    REFRESH_PARTIAL,    ///< Differential against the basemap. No flash.
} refresh_mode_t;

/* --- Service state --- */

static display_config_t s_config;

static uint8_t s_render_buf[EPD_FB_SIZE];   // painted by the caller's task
static uint8_t s_spi_buf[EPD_FB_SIZE];      // owned by the worker once signalled

static uint8_t          s_diff_count;       // partials since the last full refresh
static refresh_mode_t   s_pending_mode = REFRESH_FULL;

/*
 * The basemap a partial refresh diffs against lives in the panel's RAM, and
 * epaper_sleep() drops it. Until a full refresh has put it back, a partial
 * would be differencing against nothing.
 */
static bool s_basemap_valid;

static TaskHandle_t     s_task;
static volatile bool    s_running;
static volatile bool    s_worker_busy;

/* ======================================================================
 * Value helpers
 * ====================================================================== */

void display_values_reset(display_values_t *values)
{
    if (values) {
        *values = DISPLAY_VALUES_DEFAULT;
    }
}

/* A reading is present when it is not sitting at its sentinel. Comparing
 * floats for equality is exact here: the sentinel is assigned, never computed. */
static bool has_pm(float v)     { return v != DISPLAY_INVALID_PM; }
static bool has_co2(int v)      { return v != DISPLAY_INVALID_CO2; }
static bool has_temp(float v)   { return v != DISPLAY_INVALID_TEMPERATURE; }
static bool has_hum(float v)    { return v != DISPLAY_INVALID_HUMIDITY; }
static bool has_gas(int v)      { return v != DISPLAY_INVALID_TVOC; }
static bool has_press(float v)  { return v != DISPLAY_INVALID_PRESSURE; }
static bool has_alt(float v)    { return v != DISPLAY_INVALID_ALTITUDE; }

/** True when at least one sensor group has nothing to show. */
static bool any_sensor_offline(const display_values_t *v)
{
    return !has_pm(v->pm25_ugm3) || !has_co2(v->co2_ppm) || !has_temp(v->temperature_c) ||
           !has_gas(v->tvoc_raw) || !has_press(v->pressure_hpa);
}

/* ======================================================================
 * Drawing
 * ====================================================================== */

/** Centre a string between two x bounds rather than across the whole canvas. */
static void draw_text_between(int left, int right, int y, const char *s, int scale, bool black)
{
    const int x = left + (right - left - gfx_text_width(s, scale)) / 2;
    gfx_text(s_render_buf, x, y, s, scale, black);
}

/*
 * The highlight is an inverted band: fill black, then draw the text white.
 * gfx's `black` argument is the ink, so every call inside a band passes !hl.
 */
static void highlight_band(int left, int right, int y, int h)
{
    gfx_fill_rect(s_render_buf, left - 2, y, (right - left) + 4, h, true);
}

/** One label/value line inside a column: label left, value flushed right. */
static int draw_row(int left, int right, int y, const char *label, const char *value, bool hl)
{
    if (hl) {
        highlight_band(left, right, y - 3, ROW_PITCH);
    }
    gfx_text(s_render_buf, left, y, label, 1, !hl);
    gfx_text_right(s_render_buf, right, y, value, 1, !hl);
    return y + ROW_PITCH;
}

/** Same, but prints "--" when the reading is absent. */
static int draw_row_fmt(int left, int right, int y, const char *label, bool present, bool hl,
                        const char *fmt, ...)
{
    char value[16] = "--";

    if (present) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(value, sizeof(value), fmt, args);
        va_end(args);
    }

    return draw_row(left, right, y, label, value, hl);
}

display_metric_t display_metric_next(display_metric_t from, int delta)
{
    const int span = DISPLAY_METRIC_COUNT - 1;  // the cycle excludes NONE
    int index = (int)from - 1;                  // NONE lands on -1

    if (delta >= 0) {
        index = (index + 1) % span;
    } else {
        index = (index <= 0) ? (span - 1) : (index - 1);
    }

    return (display_metric_t)(index + 1);
}

/*
 * Title bar: white on black, with a marker on the right when something is not
 * reporting -- the one glance that says the numbers below are incomplete.
 */
static void draw_status_bar(const display_values_t *v)
{
    gfx_fill_rect(s_render_buf, 0, 0, GFX_W, STATUS_H, true);
    gfx_text(s_render_buf, MARGIN_X + 2, 3, "AIR QUALITY", 1, false);

    if (any_sensor_offline(v)) {
        gfx_text_right(s_render_buf, GFX_W - MARGIN_X - 2, 3, "!", 1, false);
    }
}

/** Left column: the PM2.5 headline, the AQI band it maps to, and the other
 *  two particle sizes underneath. */
static void draw_headline(const display_values_t *v)
{
    char buf[24];

    /* The headline is a block, not a row, so its highlight covers the whole
     * upper half of the column rather than one line of text. */
    const bool hl = (v->highlight == DISPLAY_METRIC_PM25);
    if (hl) {
        highlight_band(COL_L_X, COL_L_RIGHT, STATUS_H + 1, 60);
    }

    gfx_text(s_render_buf, COL_L_X, 17, "PM2.5 ug/m3", 1, !hl);

    if (has_pm(v->pm25_ugm3)) {
        const int aqi = aqi_from_pm25(v->pm25_ugm3);

        snprintf(buf, sizeof(buf), "%.0f", v->pm25_ugm3);
        draw_text_between(COL_L_X, COL_L_RIGHT, 28, buf, 3, !hl);

        draw_text_between(COL_L_X, COL_L_RIGHT, 53, aqi_category_name(aqi_category(aqi)), 1, !hl);

        snprintf(buf, sizeof(buf), "AQI %d", aqi);
        draw_text_between(COL_L_X, COL_L_RIGHT, 64, buf, 1, !hl);
    } else {
        draw_text_between(COL_L_X, COL_L_RIGHT, 28, "--", 3, !hl);
        draw_text_between(COL_L_X, COL_L_RIGHT, 64, "NO DATA", 1, !hl);
    }

    gfx_hline(s_render_buf, COL_L_X, 75, COL_L_RIGHT - COL_L_X, true);

    int y = 79;
    y = draw_row_fmt(COL_L_X, COL_L_RIGHT, y, "PM1.0", has_pm(v->pm1_ugm3),
                     v->highlight == DISPLAY_METRIC_PM1, "%.0f ug", v->pm1_ugm3);
    y = draw_row_fmt(COL_L_X, COL_L_RIGHT, y, "PM10", has_pm(v->pm10_ugm3),
                     v->highlight == DISPLAY_METRIC_PM10, "%.0f ug", v->pm10_ugm3);
}

/** Right column: everything that is not a particle count. */
static void draw_readings(const display_values_t *v)
{
    int y = 17;

    y = draw_row_fmt(COL_R_X, COL_R_RIGHT, y, "CO2", has_co2(v->co2_ppm),
                     v->highlight == DISPLAY_METRIC_CO2, "%d ppm", v->co2_ppm);

    /* The stored reading is always Celsius; only the presentation changes. */
    const float temp = v->use_fahrenheit ? (v->temperature_c * 9.0f / 5.0f + 32.0f)
                                         : v->temperature_c;
    y = draw_row_fmt(COL_R_X, COL_R_RIGHT, y, "TEMP", has_temp(v->temperature_c),
                     v->highlight == DISPLAY_METRIC_TEMP,
                     v->use_fahrenheit ? "%.1f F" : "%.1f C", temp);

    y = draw_row_fmt(COL_R_X, COL_R_RIGHT, y, "HUMID", has_hum(v->humidity_pct),
                     v->highlight == DISPLAY_METRIC_HUMIDITY, "%.1f %%", v->humidity_pct);

    if (v->tvoc_conditioning) {
        y = draw_row(COL_R_X, COL_R_RIGHT, y, "VOC", "warmup",
                     v->highlight == DISPLAY_METRIC_TVOC);
        y = draw_row(COL_R_X, COL_R_RIGHT, y, "NOX", "warmup",
                     v->highlight == DISPLAY_METRIC_NOX);
    } else {
        y = draw_row_fmt(COL_R_X, COL_R_RIGHT, y, "VOC", has_gas(v->tvoc_raw),
                         v->highlight == DISPLAY_METRIC_TVOC, "%d", v->tvoc_raw);
        y = draw_row_fmt(COL_R_X, COL_R_RIGHT, y, "NOX", has_gas(v->nox_raw),
                         v->highlight == DISPLAY_METRIC_NOX, "%d", v->nox_raw);
    }

    y = draw_row_fmt(COL_R_X, COL_R_RIGHT, y, "PRESS", has_press(v->pressure_hpa),
                     v->highlight == DISPLAY_METRIC_PRESSURE, "%.1f hPa", v->pressure_hpa);
    y = draw_row_fmt(COL_R_X, COL_R_RIGHT, y, "ALT", has_alt(v->altitude_m),
                     v->highlight == DISPLAY_METRIC_ALTITUDE, "%.0f m", v->altitude_m);
}

/** First menu row's text baseline-top, just under the title strip. */
#define MENU_TOP_Y          17
/** Rows that fit between the title strip and the bottom edge. */
#define MENU_VISIBLE_ROWS   ((GFX_H - MENU_TOP_Y + 3) / ROW_PITCH)

/**
 * @brief A 3-pixel-tall triangle in the menu's left gutter, pointing up or
 *        down, beside the text row whose top is @p row_y.
 *
 * Drawn in the row's own foreground, so it stays visible on the cursor band.
 */
static void draw_scroll_marker(int row_y, bool up, bool on_selected)
{
    const bool black = !on_selected;
    for (int i = 0; i < 3; i++) {
        const int w = up ? (1 + 2 * i) : (5 - 2 * i);
        gfx_hline(s_render_buf, MARGIN_X + 2 - w / 2, row_y + 2 + i, w, black);
    }
}

/**
 * @brief Full-canvas list. The selected row is inverted edge to edge, so it
 *        reads as the cursor without needing a marker glyph the 5x7 font
 *        does not have.
 */
static void draw_menu(const display_values_t *v)
{
    gfx_fill_rect(s_render_buf, 0, 0, GFX_W, STATUS_H, true);
    gfx_text(s_render_buf, MARGIN_X + 2, 3, v->menu_title ? v->menu_title : "MENU", 1, false);
    gfx_text_right(s_render_buf, GFX_W - MARGIN_X - 2, 3, "L/R move   ENTER select", 1, false);

    const uint8_t count = (v->row_count < DISPLAY_MAX_ROWS) ? v->row_count : DISPLAY_MAX_ROWS;

    /*
     * Scroll only as far as the cursor needs: the window starts at the top and
     * moves down just enough to keep the selected row on its last line. The
     * walk wraps from the last row to the first, which snaps the window back.
     */
    uint8_t first = 0;
    if (v->selected_row >= MENU_VISIBLE_ROWS) {
        first = (uint8_t)(v->selected_row - MENU_VISIBLE_ROWS + 1);
    }

    int y = MENU_TOP_Y;
    for (uint8_t i = first; i < count && i < first + MENU_VISIBLE_ROWS; i++) {
        const bool selected = (i == v->selected_row);

        if (selected) {
            gfx_fill_rect(s_render_buf, 0, y - 3, GFX_W, ROW_PITCH, true);
        }

        gfx_text(s_render_buf, MARGIN_X + 6, y, v->rows[i].label, 1, !selected);
        if (v->rows[i].value) {
            gfx_text_right(s_render_buf, GFX_W - MARGIN_X - 6, y, v->rows[i].value, 1, !selected);
        }

        y += ROW_PITCH;
    }

    // Small markers in the left gutter say there is more above or below, so
    // rows scrolled out of view do not read as rows that do not exist.
    if (first > 0) {
        draw_scroll_marker(MENU_TOP_Y, true, v->selected_row == first);
    }
    if (first + MENU_VISIBLE_ROWS < count) {
        const int last_y = MENU_TOP_Y + (MENU_VISIBLE_ROWS - 1) * ROW_PITCH;
        draw_scroll_marker(last_y, false,
                           v->selected_row == first + MENU_VISIBLE_ROWS - 1);
    }
}

/*
 * Bottom strip. The snackbar owns it when there is one; otherwise it names
 * whatever is not reporting, so a row of dashes is explainable. All five names
 * is 19 characters, which the landscape width swallows on one line.
 */
static void draw_footer(const display_values_t *v)
{
    if (v->snackbar_text != NULL && v->snackbar_text[0] != '\0') {
        gfx_fill_rect(s_render_buf, 0, FOOTER_Y - 3, GFX_W, GFX_FONT_H + 6, true);
        gfx_text_center(s_render_buf, FOOTER_Y, v->snackbar_text, 1, false);
        return;
    }

    const struct { bool ok; const char *name; } sensors[] = {
        { has_pm(v->pm25_ugm3),       "PMS" },
        { has_co2(v->co2_ppm),        "CO2" },
        { has_temp(v->temperature_c), "SHT" },
        { has_gas(v->tvoc_raw),       "SGP" },
        { has_press(v->pressure_hpa), "SPL" },
    };

    char buf[40];
    size_t used = 0;

    for (unsigned i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
        if (sensors[i].ok || used + 4 >= sizeof(buf)) {
            continue;
        }
        memcpy(&buf[used], sensors[i].name, 3);
        used += 3;
        buf[used++] = ' ';
    }
    buf[used] = '\0';

    gfx_hline(s_render_buf, 0, FOOTER_Y - 4, GFX_W, true);

    if (used == 0) {
        gfx_text(s_render_buf, COL_L_X, FOOTER_Y, "all sensors ok", 1, true);
    } else {
        const int x = gfx_text(s_render_buf, COL_L_X, FOOTER_Y, "offline: ", 1, true);
        gfx_text(s_render_buf, x, FOOTER_Y, buf, 1, true);
    }
}

/*
 * Word wrap, standing in for go_text_wrap.cpp in the reference. Breaks on
 * spaces, honours '\n' as a hard break, and splits any word longer than the
 * line rather than letting gfx clip it off the right edge.
 */
static void draw_info(const display_values_t *v)
{
    const char *p = v->info_text;
    if (p == NULL) {
        return;
    }

    int y = MARGIN_X;

    while (*p != '\0' && y + GFX_FONT_H <= GFX_H) {
        if (*p == '\n') {
            p++;
            y += LINE_PITCH;
            continue;
        }

        /* How much of the rest of the line fits. */
        size_t run = 0;
        while (p[run] != '\0' && p[run] != '\n' && run < WRAP_COLS) {
            run++;
        }

        /* Back up to the last space unless the whole run fits or the word is
         * longer than a line on its own. */
        size_t take = run;
        if (p[run] != '\0' && p[run] != '\n' && p[run] != ' ') {
            size_t brk = run;
            while (brk > 0 && p[brk - 1] != ' ') {
                brk--;
            }
            if (brk > 0) {
                take = brk - 1;   // drop the space itself
            }
        }

        char line[WRAP_COLS + 1];
        memcpy(line, p, take);
        line[take] = '\0';
        gfx_text(s_render_buf, MARGIN_X, y, line, 1, true);

        p += take;
        while (*p == ' ') {
            p++;
        }
        y += LINE_PITCH;
    }
}

static void render_frame(const display_values_t *v)
{
    gfx_clear(s_render_buf);

    if (v->screen == DISPLAY_SCREEN_INFO) {
        draw_info(v);
        return;
    }
    if (v->screen == DISPLAY_SCREEN_MENU) {
        draw_menu(v);
        return;
    }

    draw_status_bar(v);
    draw_headline(v);
    draw_readings(v);
    gfx_fill_rect(s_render_buf, DIVIDER_X, STATUS_H + 3, 1, FOOTER_Y - STATUS_H - 10, true);
    draw_footer(v);
}

/* ======================================================================
 * Worker
 * ====================================================================== */

static void worker_paint(void)
{
    esp_err_t err;

    if (s_pending_mode == REFRESH_FULL) {
        err = epaper_display_mode(s_spi_buf, EPAPER_REFRESH_FULL);
        s_diff_count = 0;
        s_basemap_valid = (err == ESP_OK);
    } else {
        /* Whole canvas, for the reason given with the layout block above. */
        err = epaper_display_region(s_spi_buf, 0, EPD_HEIGHT);
        if (s_diff_count < UINT8_MAX) {
            s_diff_count++;
        }
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "refresh failed: %s", esp_err_to_name(err));
    }
}

static void worker_task(void *arg)
{
    (void)arg;

    while (s_running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!s_running) {
            break;
        }

        worker_paint();
        s_worker_busy = false;
    }

    vTaskDelete(NULL);
}

/* ======================================================================
 * Public API
 * ====================================================================== */

esp_err_t display_init(const display_config_t *config, const display_values_t *initial,
                       bool defer_refresh)
{
    if (s_task != NULL) {
        return ESP_OK;      // idempotent, like the rest of the board's init
    }

    s_config.task_stack_size = (config && config->task_stack_size)
                                   ? config->task_stack_size : DEFAULT_TASK_STACK;
    s_config.task_priority   = (config && config->task_priority)
                                   ? config->task_priority : DEFAULT_TASK_PRIORITY;
    s_config.max_partial_ops = (config && config->max_partial_ops)
                                   ? config->max_partial_ops : EPAPER_MAX_FAST_REFRESHES;

    ESP_RETURN_ON_ERROR(epaper_init(), TAG, "panel init failed");

    const display_values_t first = initial ? *initial : DISPLAY_VALUES_DEFAULT;
    render_frame(&first);

    memcpy(s_spi_buf, s_render_buf, sizeof(s_spi_buf));
    s_pending_mode = REFRESH_FULL;

    if (!defer_refresh) {
        /* Paint inline: the worker does not exist yet, so nothing else can be
         * holding the bus. */
        worker_paint();
    }

    s_running = true;
    if (xTaskCreate(worker_task, "disp_worker", s_config.task_stack_size, NULL,
                    s_config.task_priority, &s_task) != pdPASS) {
        s_running = false;
        s_task = NULL;
        ESP_LOGE(TAG, "worker task creation failed");
        return ESP_ERR_NO_MEM;
    }

    if (defer_refresh) {
        s_worker_busy = true;
        xTaskNotifyGive(s_task);
    }

    return ESP_OK;
}

bool display_update(const display_values_t *values, bool wait)
{
    if (s_task == NULL || values == NULL) {
        return false;
    }

    if (s_worker_busy) {
        if (!wait) {
            return false;   // drop the frame; the next one carries the same state
        }
        while (s_worker_busy) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    render_frame(values);

    /*
     * Correctness first -- a partial with no basemap to difference against
     * paints nothing -- then the ghosting budget, which the panel datasheet
     * sets. Everything else goes out differentially, which is the tier that
     * does not flash.
     */
    if (!s_basemap_valid || s_diff_count >= s_config.max_partial_ops) {
        s_pending_mode = REFRESH_FULL;
    } else {
        s_pending_mode = REFRESH_PARTIAL;
    }

    memcpy(s_spi_buf, s_render_buf, sizeof(s_spi_buf));
    s_worker_busy = true;

    xTaskNotifyGive(s_task);
    return true;
}

void display_update_sync(const display_values_t *values)
{
    if (values == NULL) {
        return;
    }

    display_flush();

    render_frame(values);
    memcpy(s_spi_buf, s_render_buf, sizeof(s_spi_buf));
    s_pending_mode = REFRESH_FULL;

    worker_paint();
}

void display_flush(void)
{
    while (s_worker_busy) {
        vTaskDelay(1);
    }
}

void display_request_full_refresh(void)
{
    /* Reuses the no-basemap arm of the tier policy rather than adding a
     * second way to force a tier. */
    s_basemap_valid = false;
}

void display_clear(void)
{
    display_flush();

    if (epaper_clear() != ESP_OK) {
        ESP_LOGW(TAG, "clear failed");
        s_basemap_valid = false;
        return;
    }

    gfx_clear(s_render_buf);
    memcpy(s_spi_buf, s_render_buf, sizeof(s_spi_buf));
    s_diff_count = 0;
    s_basemap_valid = true;
}

void display_deep_sleep(void)
{
    display_flush();

    if (epaper_sleep() != ESP_OK) {
        ESP_LOGW(TAG, "deep sleep failed");
    }

    /* Sleep mode 1 drops the RAM planes, so the next frame cannot be a
     * differential one. */
    s_basemap_valid = false;
}

void display_stop(void)
{
    if (s_task == NULL) {
        return;
    }

    s_running = false;
    xTaskNotifyGive(s_task);        // wake the worker so it can exit its loop

    display_flush();
    vTaskDelay(pdMS_TO_TICKS(50));  // let the task reach vTaskDelete
    s_task = NULL;
}
