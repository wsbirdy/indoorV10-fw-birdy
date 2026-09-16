#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "spi.h"    // SPI_CS_GPIO and friends

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Panel pins. SCLK and MOSI come from the shared bus in spi.h; the four below
 * belong to the panel. CS is driven by this driver rather than by the SPI
 * peripheral, so that a whole command-plus-frame burst stays selected.
 * Adjust to match the board.
 */
#define EPD_CS_GPIO         SPI_CS_GPIO
#define EPD_DC_GPIO         8
#define EPD_RST_GPIO        10
#define EPD_BUSY_GPIO       9

/* GDEY0213B74: 2.13", 122 x 250, SSD1680 controller. */
#define EPD_WIDTH           122                     // visible columns
#define EPD_HEIGHT          250
#define EPD_ROW_BYTES       ((EPD_WIDTH + 7) / 8)   // 16, last 6 bits off-glass
#define EPD_RAM_WIDTH       (EPD_ROW_BYTES * 8)     // 128, the controller's row pitch
#define EPD_FB_SIZE         (EPD_ROW_BYTES * EPD_HEIGHT)

/**
 * @brief Refresh waveform tier, in decreasing cost and increasing ghosting.
 */
typedef enum {
    /**
     * Full GC waveform (~2 s). Flickers several times, drives every pixel from
     * scratch and primes both RAM planes, which clears accumulated afterimages.
     * Required before the first EPAPER_REFRESH_PARTIAL.
     */
    EPAPER_REFRESH_FULL,
    /**
     * Full-screen, non-differential (~800 ms). Flashes once. Selects the
     * panel's shortest OTP waveform by overriding the controller's temperature
     * register, so contrast is slightly weaker than a full refresh.
     */
    EPAPER_REFRESH_FAST,
    /**
     * Differential against the basemap left in RAM by the last full or fast
     * refresh (~400 ms). Does not flicker at all, and is the cheapest, but
     * leaves the most residue behind.
     */
    EPAPER_REFRESH_PARTIAL,
} epaper_refresh_t;

/**
 * @brief Consecutive fast or partial refreshes allowed before a full one.
 *
 * The panel datasheet asks for a full refresh after every 5 fast or partial
 * updates, to keep afterimages from accumulating. The driver counts these
 * itself and silently promotes the sixth to a full refresh, so callers can ask
 * for fast refreshes unconditionally and still stay inside the limit.
 */
#define EPAPER_MAX_FAST_REFRESHES   5

/**
 * @brief Bind and configure every pin the panel needs.
 *
 * Brings up the shared SPI bus (SCLK, MOSI and the CS the peripheral drives),
 * configures DC and RST as outputs at their idle levels, configures BUSY as an
 * input, and allocates the DMA transmit buffer.
 *
 * Idempotent: the second and later calls return ESP_OK without touching
 * anything. A call that fails part-way unwinds what it bound, so it is safe to
 * retry. Callers normally do not need this -- epaper_init() calls it -- but it
 * is exposed for boards that want the pins in a known state before the panel
 * is driven.
 */
esp_err_t epaper_pins_init(void);

/**
 * @brief Bind the pins (see epaper_pins_init()) and reset the panel.
 *
 * Self-contained: no other bus or peripheral has to be initialized first.
 * Safe to call repeatedly; the pin and bus setup happens once while the reset
 * happens every time. Must be called again after epaper_sleep(), which only a
 * hardware reset can undo.
 */
esp_err_t epaper_init(void);

/**
 * @brief Push a full framebuffer and run a full refresh. Equivalent to
 *        epaper_display_mode(fb, EPAPER_REFRESH_FULL).
 *
 * @param fb EPD_FB_SIZE bytes, MSB is the leftmost pixel, 1 = white, 0 = black
 */
esp_err_t epaper_display(const uint8_t *fb);

/**
 * @brief Push a full framebuffer and refresh with the given waveform.
 *
 * The controller is reconfigured from scratch on every call, so modes can be
 * mixed freely -- except that EPAPER_REFRESH_PARTIAL needs a preceding full or
 * fast refresh to have established the basemap.
 *
 * @param fb    EPD_FB_SIZE bytes, same layout as epaper_display()
 * @param mode  waveform tier. May be promoted to EPAPER_REFRESH_FULL to honour
 *              EPAPER_MAX_FAST_REFRESHES.
 */
esp_err_t epaper_display_mode(const uint8_t *fb, epaper_refresh_t mode);

/**
 * @brief Partially refresh a horizontal band, leaving the rest of the panel
 *        untouched. Always full width.
 *
 * @param rows    @p height * EPD_ROW_BYTES bytes, topmost row first
 * @param y       top row of the band, 0 = top of the panel
 * @param height  number of rows; @p y + @p height must not exceed EPD_HEIGHT
 */
esp_err_t epaper_display_region(const uint8_t *rows, uint16_t y, uint16_t height);

/** @brief Blank the panel to white with a full refresh. */
esp_err_t epaper_clear(void);

/**
 * @brief Enter deep sleep (~1 uA).
 *
 * The panel must not be left biased between updates. epaper_init() has to be
 * called again before the next display.
 */
esp_err_t epaper_sleep(void);

#ifdef __cplusplus
}
#endif
