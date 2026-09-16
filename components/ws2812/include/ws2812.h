#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Single-wire data line. Any GPIO works; the RMT peripheral drives it. */
#define WS2812_GPIO         27
#define WS2812_LED_COUNT    11

/** @brief Reserve an RMT channel and blank the strip. */
esp_err_t ws2812_init(void);

/**
 * @brief Stage one pixel. Nothing lights up until ws2812_refresh().
 *
 * @param index  0 .. WS2812_LED_COUNT-1, counted from the LED nearest the MCU
 * @param r,g,b  0 = off, 255 = full
 */
esp_err_t ws2812_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/** @brief Stage the same colour on every pixel. */
void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Scale every channel on the way out, so staged colours keep their
 *        full resolution. Defaults to WS2812_DEFAULT_BRIGHTNESS.
 *
 * Applied at refresh time; call ws2812_refresh() to make it visible.
 */
void ws2812_set_brightness(uint8_t brightness);

/** @brief Shift the staged colours out and latch them. */
esp_err_t ws2812_refresh(void);

/** @brief Stage black on every pixel and refresh. */
esp_err_t ws2812_clear(void);

/*
 * These LEDs are uncomfortably bright at full scale and draw ~60 mA each,
 * which is 660 mA for the strip. Start low.
 */
#define WS2812_DEFAULT_BRIGHTNESS   64

#ifdef __cplusplus
}
#endif
