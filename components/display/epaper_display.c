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
 *   +##############################################################+
 *   |#SENSORS OK#############02 NOV 18:00##################NO WIFI#|    2  top bar
 *   +##############################+###############################+   11
 *   | PM2.5                  ug/m3 | CO2                       ppm |   15
 *   |             35               |            1240               |   25  scale 5
 *   |       MODERATE  AQI 99       |            POOR               |   63
 *   +#####CLIMATE#########+#######GASES#######+#####PRESSURE#######+   74
 *   | TEMP         24.5C  | VOC         31000 |       1013         |   93
 *   | HUMID          55%  | NOX         16000 |        hPa         |  107
 *   +---------------------+-------------------+--------------------+
 *
 * PM2.5 and CO2 are the two readings that say whether the room needs
 * attention, so they get the large figures. Everything else is grouped
 * underneath, each group under its own inverted header: climate, gases, and
 * pressure on its own, which gives it room to be the one secondary figure
 * drawn at double size. Sensor health sits at the left of the top line.
 *
 * A snackbar is a notification, not a footer: while one is up it is drawn as a
 * solid block over the whole grid, and the grid comes back when the caller
 * stops sending it. The headline figures stay visible underneath it.
 *
 * The reference splits the panel into a frozen header and a body band so a
 * partial refresh can redraw the body alone. That split does not survive the
 * rotation: the driver refreshes whole panel rows, and panel rows run down the
 * canvas's x axis once the image is turned a quarter turn, so a band would be
 * a column of this layout rather than a strip of it. Partial refreshes here
 * cover the whole canvas instead -- still differential, still no flash, they
 * just carry every changed pixel rather than a chosen band.
 */
#define STATUS_H        13      // menu title strip
#define MARGIN_X        2

/* The two headline halves, either side of a one-pixel divider. */
#define HALF_DIVIDER_X  124
#define HALF_L_LEFT     MARGIN_X
#define HALF_L_RIGHT    (HALF_DIVIDER_X - 3)
#define HALF_R_LEFT     (HALF_DIVIDER_X + 3)
#define HALF_R_RIGHT    (GFX_W - MARGIN_X)

#define TOP_Y           2       // clock and link status
#define TOP_RULE_Y      11      // bottom row of the top bar

#define HERO_LABEL_Y    15
#define HERO_VALUE_Y    25
#define HERO_SCALE      5       // 35 px tall; "1240" is 116 px, inside a half
#define HERO_BAND_Y     63
#define HERO_RULE_Y     74

#define GROUP_HEAD_H    13      // inverted header strip, from HERO_RULE_Y
#define GROUP_ROW1_Y    (HERO_RULE_Y + 19)
#define GROUP_ROW2_Y    (HERO_RULE_Y + 33)
#define GROUP_PAD       4       // text inset from a group's edges

#define ROW_PITCH       13
#define LINE_PITCH      10

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
/* ======================================================================
 * Drawing
 * ====================================================================== */

/** Centre a string between two x bounds rather than across the whole canvas. */
static void draw_text_between(int left, int right, int y, const char *s, int scale, bool black)
{
    const int x = left + (right - left - gfx_text_width(s, scale)) / 2;
    gfx_text(s_render_buf, x, y, s, scale, black);
}

/** One label/value line inside a column: label left, value flushed right. */
static void draw_row(int left, int right, int y, const char *label, const char *value)
{
    gfx_text(s_render_buf, left, y, label, 1, true);
    gfx_text_right(s_render_buf, right, y, value, 1, true);
}

/*
 * The three groups under the headline, separated by one-pixel rules at their
 * shared edges. Not even thirds: Climate needs the most room ("HUMID 100%",
 * "TEMP 104.0F") and Pressure the least.
 */
static const struct { int left, right; } GROUPS[] = {
    { 0,   83 },
    { 83,  166 },
    { 166, GFX_W },
};

#define GROUP_COUNT     (sizeof(GROUPS) / sizeof(GROUPS[0]))

/** A group's header: its name white on black, across the group's full width. */
static void draw_group_head(unsigned group, const char *title)
{
    const int left = GROUPS[group].left;
    const int right = GROUPS[group].right;

    gfx_fill_rect(s_render_buf, left, HERO_RULE_Y, right - left, GROUP_HEAD_H, true);
    draw_text_between(left, right, HERO_RULE_Y + 3, title, 1, false);
}

/** One label/value line inside a group; "--" when the reading is absent. */
static void draw_group_row(unsigned group, int y, const char *label, bool present,
                           const char *fmt, ...)
{
    char value[16] = "--";

    if (present) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(value, sizeof(value), fmt, args);
        va_end(args);
    }

    draw_row(GROUPS[group].left + GROUP_PAD, GROUPS[group].right - GROUP_PAD, y,
             label, value);
}

/*
 * CO2 has no official index the way PM2.5 has the EPA AQI. These are
 * ventilation bands: 1000 ppm is the usual "open a window" line. Adjust here.
 */
static const char *co2_band_name(int ppm)
{
    if (ppm < 800) {
        return "GOOD";
    } else if (ppm < 1000) {
        return "MODERATE";
    } else if (ppm < 1500) {
        return "POOR";
    }
    return "BAD";
}

/**
 * @brief One headline half: label and unit on top, the figure large in the
 *        middle, its band word underneath.
 */
static void draw_hero(int left, int right, const char *label, const char *unit,
                      const char *value, const char *band)
{
    gfx_text(s_render_buf, left, HERO_LABEL_Y, label, 1, true);
    gfx_text_right(s_render_buf, right, HERO_LABEL_Y, unit, 1, true);

    /* A five-digit CO2 reading is 146 px at full size: step down rather than
     * clip, and keep the smaller figure centred on the same line. */
    int scale = HERO_SCALE;
    while (scale > 1 && gfx_text_width(value, scale) > right - left) {
        scale--;
    }
    const int y = HERO_VALUE_Y + GFX_FONT_H * (HERO_SCALE - scale) / 2;
    draw_text_between(left, right, y, value, scale, true);

    draw_text_between(left, right, HERO_BAND_Y, band, 1, true);
}

static void draw_pm25(const display_values_t *v)
{
    if (!has_pm(v->pm25_ugm3)) {
        draw_hero(HALF_L_LEFT, HALF_L_RIGHT, "PM2.5", "ug/m3", "--", "NO DATA");
        return;
    }

    const int aqi = aqi_from_pm25(v->pm25_ugm3);
    const char *name = aqi_category_name(aqi_category(aqi));
    char value[12];
    char band[32];

    snprintf(value, sizeof(value), "%.0f", v->pm25_ugm3);

    /* "SENSITIVE GRP  AQI 150" overruns the half; the bare number does not. */
    snprintf(band, sizeof(band), "%s  AQI %d", name, aqi);
    if (gfx_text_width(band, 1) > HALF_L_RIGHT - HALF_L_LEFT) {
        snprintf(band, sizeof(band), "%s %d", name, aqi);
    }

    draw_hero(HALF_L_LEFT, HALF_L_RIGHT, "PM2.5", "ug/m3", value, band);
}

static void draw_co2(const display_values_t *v)
{
    if (!has_co2(v->co2_ppm)) {
        draw_hero(HALF_R_LEFT, HALF_R_RIGHT, "CO2", "ppm", "--", "NO DATA");
        return;
    }

    char value[12];
    snprintf(value, sizeof(value), "%d", v->co2_ppm);
    draw_hero(HALF_R_LEFT, HALF_R_RIGHT, "CO2", "ppm", value, co2_band_name(v->co2_ppm));
}

/*
 * Left end of the top line: "SENSORS OK", the one sensor that is not
 * reporting ("SGP OFF"), or how many are not -- the one glance that says a
 * "--" below is a fault rather than a reading.
 */
static void format_sensor_status(const display_values_t *v, char *buf, size_t len)
{
    const struct { bool ok; const char *name; } sensors[] = {
        { has_pm(v->pm25_ugm3),       "PMS" },
        { has_co2(v->co2_ppm),        "CO2" },
        { has_temp(v->temperature_c), "SHT" },
        { has_gas(v->tvoc_raw),       "SGP" },
        { has_press(v->pressure_hpa), "SPL" },
    };

    unsigned offline = 0;
    const char *name = NULL;

    for (unsigned i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
        if (!sensors[i].ok) {
            offline++;
            name = sensors[i].name;
        }
    }

    if (offline == 0) {
        snprintf(buf, len, "SENSORS OK");
    } else if (offline == 1) {
        snprintf(buf, len, "%s OFF", name);
    } else {
        snprintf(buf, len, "%u SENSORS OFF", offline);
    }
}

static void draw_climate(const display_values_t *v)
{
    /* The stored reading is always Celsius; only the presentation changes. */
    const float temp = v->use_fahrenheit ? (v->temperature_c * 9.0f / 5.0f + 32.0f)
                                         : v->temperature_c;

    draw_group_head(0, "CLIMATE");
    draw_group_row(0, GROUP_ROW1_Y, "TEMP", has_temp(v->temperature_c),
                   v->use_fahrenheit ? "%.1fF" : "%.1fC", temp);
    draw_group_row(0, GROUP_ROW2_Y, "HUMID", has_hum(v->humidity_pct), "%.0f%%", v->humidity_pct);
}

static void draw_gases(const display_values_t *v)
{
    draw_group_head(1, "GASES");

    if (v->tvoc_conditioning) {
        draw_group_row(1, GROUP_ROW1_Y, "VOC", true, "warmup");
        draw_group_row(1, GROUP_ROW2_Y, "NOX", true, "warmup");
    } else {
        draw_group_row(1, GROUP_ROW1_Y, "VOC", has_gas(v->tvoc_raw), "%d", v->tvoc_raw);
        draw_group_row(1, GROUP_ROW2_Y, "NOX", has_gas(v->nox_raw), "%d", v->nox_raw);
    }
}

/* A single reading, so it is drawn as a figure rather than a label/value row. */
static void draw_pressure(const display_values_t *v)
{
    const int left = GROUPS[2].left + GROUP_PAD;
    const int right = GROUPS[2].right - GROUP_PAD;

    draw_group_head(2, "PRESSURE");

    char value[12] = "--";
    if (has_press(v->pressure_hpa)) {
        snprintf(value, sizeof(value), "%.0f", v->pressure_hpa);
    }
    draw_text_between(left, right, GROUP_ROW1_Y - 1, value, 2, true);
    draw_text_between(left, right, GROUP_ROW2_Y + 1, "hPa", 1, true);
}

static void draw_groups(const display_values_t *v)
{
    draw_climate(v);
    draw_gases(v);
    draw_pressure(v);

    for (unsigned g = 1; g < GROUP_COUNT; g++) {
        gfx_fill_rect(s_render_buf, GROUPS[g].left, HERO_RULE_Y, 1, GFX_H - HERO_RULE_Y, true);
    }
}

/*
 * Top bar, white on black: sensor health left, the clock centred, the link
 * status right. Solid, like the group headers, so the frame reads as one
 * style; its bottom edge doubles as the rule above the headline.
 */
static void draw_top_line(const display_values_t *v)
{
    gfx_fill_rect(s_render_buf, 0, 0, GFX_W, TOP_RULE_Y + 1, true);

    char status[16];
    format_sensor_status(v, status, sizeof(status));
    gfx_text(s_render_buf, MARGIN_X + 2, TOP_Y, status, 1, false);

    if (v->clock_text != NULL) {
        gfx_text_center(s_render_buf, TOP_Y, v->clock_text, 1, false);
    }
    if (v->status_text != NULL) {
        gfx_text_right(s_render_buf, GFX_W - MARGIN_X - 2, TOP_Y, v->status_text, 1, false);
    }
}

static void draw_dashboard(const display_values_t *v)
{
    draw_top_line(v);
    draw_pm25(v);
    draw_co2(v);

    gfx_fill_rect(s_render_buf, HALF_DIVIDER_X, TOP_RULE_Y, 1, HERO_RULE_Y - TOP_RULE_Y, true);
    gfx_hline(s_render_buf, 0, HERO_RULE_Y, GFX_W, true);

    draw_groups(v);
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
 * The notification: a solid block over everything below the headline, with the
 * text centred in it. Double size when it fits on one line (20 characters),
 * which a short "LED bar 45%" does; longer text drops to the normal size.
 */
static void draw_snackbar(const display_values_t *v)
{
    if (v->snackbar_text == NULL || v->snackbar_text[0] == '\0') {
        return;
    }

    // One white line under the rule keeps the block visibly a layer on top
    // rather than part of the layout.
    const int top = HERO_RULE_Y + 2;
    gfx_fill_rect(s_render_buf, 0, top, GFX_W, GFX_H - top, true);

    const int scale = (gfx_text_width(v->snackbar_text, 2) <= GFX_W - 2 * MARGIN_X) ? 2 : 1;
    const int y = top + (GFX_H - top - GFX_FONT_H * scale) / 2;
    gfx_text_center(s_render_buf, y, v->snackbar_text, scale, false);
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
    } else if (v->screen == DISPLAY_SCREEN_MENU) {
        draw_menu(v);
    } else {
        draw_dashboard(v);
        draw_snackbar(v);
    }

    /*
     * Dark theme is the light frame with every pixel flipped, done last so no
     * drawing code has to know about it: the inverted bars and cursor come out
     * as light bands on black, which is exactly the role they play in light.
     * The RAM's unused columns flip too, but they are off the glass.
     */
    if (v->dark_theme) {
        for (size_t i = 0; i < sizeof(s_render_buf); i++) {
            s_render_buf[i] = (uint8_t)~s_render_buf[i];
        }
    }
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

bool display_is_busy(void)
{
    return s_worker_busy;
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
