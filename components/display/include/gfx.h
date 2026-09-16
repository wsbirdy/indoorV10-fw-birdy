#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "epaper.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimal 1-bit drawing onto an e-paper framebuffer (see epaper.h for the
 * layout: MSB is the leftmost pixel, 1 = white, 0 = black).
 *
 * The built-in font is 5x7 with a one-pixel gap, so a character costs 6 px of
 * width at scale 1. Everything clips silently at the canvas edges.
 */
#define GFX_FONT_W      5
#define GFX_FONT_H      7
#define GFX_ADVANCE     (GFX_FONT_W + 1)

/* --- Orientation --------------------------------------------------------
 *
 * Callers draw in canvas coordinates; gfx_pixel() is the single place that
 * maps those onto the panel's RAM. Two independent corrections live there:
 *
 * GFX_LANDSCAPE  The panel is portrait (122 x 250). With this set, the canvas
 *                is 250 x 122 and the image is rotated a quarter turn.
 *
 * GFX_MIRROR_X   The glass scans its source outputs opposite to the RAM
 *                column order the driver writes, so a framebuffer laid out
 *                left-to-right reaches the glass reversed. The AirGradient
 *                reference compensates for the same thing by handing u8g2
 *                U8G2_MIRROR (go_display.cpp, u8g2_SetupBuffer); this is the
 *                equivalent for gfx.
 *
 * GFX_ROTATE_CW  Which way up the quarter turn goes. Flipping it turns the
 *                image through 180 degrees -- the knob to reach for if the
 *                board reads upside down in its enclosure.
 */
#define GFX_LANDSCAPE   1
#define GFX_MIRROR_X    1
#define GFX_ROTATE_CW   1

/** Canvas size. Use these for layout, never EPD_WIDTH / EPD_HEIGHT. */
#if GFX_LANDSCAPE
#define GFX_W           EPD_HEIGHT      // 250
#define GFX_H           EPD_WIDTH       // 122
#else
#define GFX_W           EPD_WIDTH
#define GFX_H           EPD_HEIGHT
#endif

/** @brief Fill the whole framebuffer with white. */
void gfx_clear(uint8_t *fb);

/** @brief Set one pixel. */
void gfx_pixel(uint8_t *fb, int x, int y, bool black);

/** @brief Filled rectangle. */
void gfx_fill_rect(uint8_t *fb, int x, int y, int w, int h, bool black);

/** @brief Horizontal rule, one pixel tall. */
void gfx_hline(uint8_t *fb, int x, int y, int w, bool black);

/** @brief Rendered width of @p s in pixels, excluding the trailing gap. */
int gfx_text_width(const char *s, int scale);

/**
 * @brief Draw a string with its top-left corner at (@p x, @p y).
 *
 * Characters outside 0x20..0x7E render as a blank. Newlines are not handled.
 *
 * @return the x coordinate one advance past the last character
 */
int gfx_text(uint8_t *fb, int x, int y, const char *s, int scale, bool black);

/** @brief Draw a string horizontally centred on the panel. */
int gfx_text_center(uint8_t *fb, int y, const char *s, int scale, bool black);

/** @brief Draw a string ending at @p right_x. */
int gfx_text_right(uint8_t *fb, int right_x, int y, const char *s, int scale, bool black);

#ifdef __cplusplus
}
#endif
