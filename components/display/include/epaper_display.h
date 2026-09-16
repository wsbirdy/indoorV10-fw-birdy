#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "epaper.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Display service: the layer between "what the board measured" and "what is on
 * the panel". Ported from DisplayService in the AirGradient Go display library
 * (example/go_display_driver/go_display.cpp), which this board cannot build
 * verbatim -- it is C++ and pulls in u8g2, rtos.h, measures_types.h and
 * provisioning_qr.h, none of which are vendored here.
 *
 * What carried over is the shape, which is the part that matters:
 *
 *   - callers submit a whole-screen value snapshot, never draw calls;
 *   - rendering into the framebuffer is synchronous and cheap (~1 ms), while
 *     the panel refresh (0.4 s .. 2 s) runs on a worker task;
 *   - the refresh tier is chosen from what changed between snapshots, not by
 *     the caller.
 *
 * What was dropped: u8g2 (the gfx.h 5x7 primitives stand in), and every screen
 * whose state this board has no source for -- menus, battery, BLE pairing,
 * Wi-Fi provisioning, GPS, the fuel-gauge learning dashboard.
 */

/* --- Missing-reading sentinels -----------------------------------------
 *
 * Mirrors MeasuresInvalid in airgradient-common/measures_types.h so a port of
 * either side keeps comparing equal. A field left at its sentinel renders as
 * "--" rather than as a plausible-looking zero.
 */
#define DISPLAY_INVALID_PM          (-1.0f)
#define DISPLAY_INVALID_CO2         (-1)
#define DISPLAY_INVALID_TEMPERATURE (-1000.0f)
#define DISPLAY_INVALID_HUMIDITY    (-1.0f)
#define DISPLAY_INVALID_TVOC        (-1)
#define DISPLAY_INVALID_NOX         (-1)
#define DISPLAY_INVALID_PRESSURE    (-1.0f)
#define DISPLAY_INVALID_ALTITUDE    (-10000.0f)

/**
 * @brief Which page the panel is showing.
 *
 * The reference enumerates 26 screens because it has provisioning, BLE pairing
 * and a factory fuel-gauge path. This board has three.
 */
typedef enum {
    DISPLAY_SCREEN_HOME,    ///< Sensor dashboard: headline PM2.5 + AQI + rows
    DISPLAY_SCREEN_MENU,    ///< Full-canvas list. Owns the whole canvas.
    DISPLAY_SCREEN_INFO,    ///< Single wrapped-text page. Owns the whole canvas.
} display_screen_t;

/**
 * @brief A reading the dashboard can single out.
 *
 * Declaration order is the cycle order the left and right pads walk, which is
 * also reading order on the panel: down the left column, then down the right.
 * Mirrors Metric in the reference, widened to this board's sensor set.
 */
typedef enum {
    DISPLAY_METRIC_NONE = 0,    ///< Nothing highlighted (menu and info pages)
    DISPLAY_METRIC_PM25,
    DISPLAY_METRIC_PM1,
    DISPLAY_METRIC_PM10,
    DISPLAY_METRIC_CO2,
    DISPLAY_METRIC_TEMP,
    DISPLAY_METRIC_HUMIDITY,
    DISPLAY_METRIC_TVOC,
    DISPLAY_METRIC_NOX,
    DISPLAY_METRIC_PRESSURE,
    DISPLAY_METRIC_ALTITUDE,
    DISPLAY_METRIC_COUNT,
} display_metric_t;

/**
 * @brief Step the highlight, wrapping at both ends.
 *
 * DISPLAY_METRIC_NONE is not part of the cycle -- stepping from it lands on
 * the first real metric in whichever direction was asked for.
 *
 * @param delta  +1 for the right pad, -1 for the left.
 */
display_metric_t display_metric_next(display_metric_t from, int delta);

/** @brief Longest menu the renderer will draw. */
#define DISPLAY_MAX_ROWS 8

/**
 * @brief One menu line: a name on the left, its current setting on the right.
 *
 * Both pointers are borrowed and must outlive the submission. @p value may be
 * NULL for a row that does something rather than holding a setting.
 */
typedef struct {
    const char *label;
    const char *value;
} display_row_t;

/**
 * @brief One complete description of the screen. Nothing else is consulted.
 *
 * The reference's DisplayValues is a C++ struct with default member
 * initializers; C has none, so start from DISPLAY_VALUES_DEFAULT or
 * display_values_reset() and overwrite what you know. Zero-initializing by
 * hand is wrong -- it reads as "0 ug/m3", not "no reading".
 *
 * String pointers are borrowed and must stay valid until the next submission.
 */
typedef struct {
    /* --- Sensor readings --- */
    float pm1_ugm3;
    float pm25_ugm3;
    float pm10_ugm3;
    int   co2_ppm;
    float temperature_c;
    float humidity_pct;
    int   tvoc_raw;         ///< SGP41 raw ticks, not the 0..500 gas index
    int   nox_raw;
    float pressure_hpa;
    float altitude_m;

    /** SGP41 is in its warm-up window: shown as "warmup", not as "--". */
    bool tvoc_conditioning;

    /* --- Presentation --- */

    /** Render temperature in Fahrenheit. The value above stays Celsius. */
    bool use_fahrenheit;

    /* --- Screen selection --- */
    display_screen_t screen;

    /** Reading drawn inverted on the dashboard. NONE highlights nothing. */
    display_metric_t highlight;

    /* --- DISPLAY_SCREEN_MENU --- */
    display_row_t rows[DISPLAY_MAX_ROWS];
    uint8_t       row_count;
    uint8_t       selected_row;     ///< Index into rows[], drawn inverted

    /** Title strip text for the menu. NULL falls back to "MENU". */
    const char *menu_title;

    /** Body text for DISPLAY_SCREEN_INFO. Wraps on spaces; '\n' breaks hard. */
    const char *info_text;

    /** One line pinned to the bottom of the dashboard. NULL for none. */
    const char *snackbar_text;
} display_values_t;

/** @brief Initializer that marks every reading absent. */
#define DISPLAY_VALUES_DEFAULT                      \
    (display_values_t) {                            \
        .pm1_ugm3       = DISPLAY_INVALID_PM,       \
        .pm25_ugm3      = DISPLAY_INVALID_PM,       \
        .pm10_ugm3      = DISPLAY_INVALID_PM,       \
        .co2_ppm        = DISPLAY_INVALID_CO2,      \
        .temperature_c  = DISPLAY_INVALID_TEMPERATURE, \
        .humidity_pct   = DISPLAY_INVALID_HUMIDITY, \
        .tvoc_raw       = DISPLAY_INVALID_TVOC,     \
        .nox_raw        = DISPLAY_INVALID_NOX,      \
        .pressure_hpa   = DISPLAY_INVALID_PRESSURE, \
        .altitude_m     = DISPLAY_INVALID_ALTITUDE, \
        .screen         = DISPLAY_SCREEN_HOME,      \
        .highlight      = DISPLAY_METRIC_NONE,      \
    }

/** @brief Same, for assigning into an existing struct. */
void display_values_reset(display_values_t *values);

/**
 * @brief Worker task and refresh-budget settings.
 *
 * The reference carries SPI and pin fields here; this board's are fixed in
 * epaper.h, so only the tunables remain. Pass NULL to display_init() for the
 * defaults below.
 */
typedef struct {
    uint16_t task_stack_size;   ///< default 3072
    uint8_t  task_priority;     ///< default 4
    /**
     * Partial/fast refreshes allowed before one is promoted to full.
     * Defaults to EPAPER_MAX_FAST_REFRESHES, the panel datasheet's limit;
     * epaper_display_panel.c enforces the same ceiling independently.
     */
    uint8_t  max_partial_ops;
} display_config_t;

/**
 * @brief Bring up the panel, paint @p initial, and start the worker task.
 *
 * @param config   NULL for defaults.
 * @param initial  first frame. NULL paints a blank canvas.
 * @param defer_refresh
 *        false: the initial full refresh happens inline, so the call blocks
 *        for ~2 s and the panel is correct on return.
 *        true: the frame is rendered, the worker is started, and the refresh is
 *        posted as its first job. Returns in ~10 ms; the panel catches up while
 *        the caller gets on with sensor bring-up. The worker holds the SPI bus
 *        for the duration, which serializes any other device on it.
 */
esp_err_t display_init(const display_config_t *config, const display_values_t *initial,
                       bool defer_refresh);

/**
 * @brief Submit a frame.
 *
 * Renders inline (fast) and hands the buffer to the worker. The refresh tier is
 * derived from the difference against the previous submission.
 *
 * @param wait  false: return false immediately if the worker is still painting
 *              -- the frame is dropped, which is the right answer for periodic
 *              updates. true: block until the worker is free, then submit.
 * @return true if the frame was queued.
 */
bool display_update(const display_values_t *values, bool wait);

/**
 * @brief Render and drive the panel inline, bypassing the worker.
 *
 * For the paths where there is nothing else to overlap with -- the last frame
 * before sleep, or a panic screen.
 */
void display_update_sync(const display_values_t *values);

/** @brief Block until the queued frame has finished painting. */
void display_flush(void);

/**
 * @brief Make the next submission a full refresh.
 *
 * The tier policy already promotes one every few frames to keep afterimages
 * down. This is for asking on purpose -- a menu action, or any point where the
 * panel looks like it has accumulated residue.
 */
void display_request_full_refresh(void);

/** @brief Blank the panel with a full refresh. Blocking. */
void display_clear(void);

/**
 * @brief Put the panel controller into deep sleep (~1 uA).
 *
 * The panel must not be left biased for long periods. This discards the RAM
 * basemap, so the next frame is forced to a full refresh -- do not call it
 * between periodic updates, only when the board itself is going idle.
 */
void display_deep_sleep(void);

/** @brief Stop the worker task. Call before entering ESP deep sleep. */
void display_stop(void);

#ifdef __cplusplus
}
#endif
