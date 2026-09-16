#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAP1203_I2C_ADDR    0x28

#define CAP1203_PAD1        0x01
#define CAP1203_PAD2        0x02
#define CAP1203_PAD3        0x04

/** @brief Verify the part ID and enable all three pads. Requires i2c_init(). */
esp_err_t cap1203_init(void);

/**
 * @brief Read which pads are currently touched and clear the latched interrupt.
 *
 * @param pads bitmask of CAP1203_PAD1..PAD3
 */
esp_err_t cap1203_read(uint8_t *pads);

/**
 * @brief Set touch sensitivity.
 *
 * Numbered exactly as the datasheet's DELTA_SENSE[2:0] field (table 5-7), so
 * the value here is the value that reaches the register:
 *
 *   0 = 128x  most sensitive       4 = 8x
 *   1 = 64x                        5 = 4x
 *   2 = 32x   (power-on default)   6 = 2x
 *   3 = 16x                        7 = 1x   least sensitive
 *
 * The more sensitive settings detect a lighter touch and are correspondingly
 * more willing to report a false one in a noisy environment.
 */
esp_err_t cap1203_set_sensitivity(uint8_t delta_sense);

/**
 * @brief Log every readable register once, by name, on the "cap1203.dbg" tag.
 *
 * A development aid for judging how the pads behave in a given enclosure. For a
 * continuous view of the delta and base counts instead of a single snapshot,
 * set CAP1203_DEBUG_DUMP to 1 at the top of cap1203.c; that also calls this
 * once at init. Roughly 40 I2C reads, so do not put it on a hot path.
 */
esp_err_t cap1203_dump_registers(void);

/* ======================================================================
 * Buttons
 *
 * The three pads as an event stream, for callers that want presses rather
 * than pad states.
 *
 * Whoever polls the part consumes its events: cap1203_read() clears the
 * latched interrupt, so two pollers would steal presses from each other.
 * The task started below is therefore the board's only reader, and
 * cap1203_pads() exists so a second caller can see the state without
 * taking it away.
 * ====================================================================== */

/** Pad identity, left to right along the board edge. */
typedef enum {
    CAP1203_BUTTON_NONE = 0,
    CAP1203_BUTTON_LEFT,    ///< CAP1203_PAD1
    CAP1203_BUTTON_ENTER,   ///< CAP1203_PAD2
    CAP1203_BUTTON_RIGHT,   ///< CAP1203_PAD3
} cap1203_button_t;

/**
 * @brief Start the polling task that turns pad states into press events.
 *
 * Calls cap1203_init() if it has not run yet. Idempotent.
 */
esp_err_t cap1203_buttons_start(void);

/**
 * @brief Wait for the next press.
 *
 * Leading edge only: holding a pad yields one event, not a stream. Release
 * and touch again for another.
 *
 * Always blocks for the full timeout when no press arrives, including when the
 * part is absent and cap1203_buttons_start() failed -- callers that use this as
 * their loop's pacing stay paced whether or not the hardware is there.
 *
 * @param out         receives the pad. Untouched when the wait times out.
 * @param timeout_ms  how long to block; 0 polls, UINT32_MAX waits forever.
 * @return true when @p out was filled.
 */
bool cap1203_button_wait(cap1203_button_t *out, uint32_t timeout_ms);

/** @brief Pads touched at the last poll, as a CAP1203_PAD* bitmask. */
uint8_t cap1203_pads(void);

/** @brief Whether the last poll reached the part. */
bool cap1203_pads_valid(void);

/** @brief Millisecond timestamp of the last poll. */
uint32_t cap1203_pads_updated_ms(void);

#ifdef __cplusplus
}
#endif
