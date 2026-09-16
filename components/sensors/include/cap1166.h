#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CAP1166 -- Microchip 6-channel capacitive touch sensor with 6 LED drivers.
 *
 * Same register family as the CAP1203 this board used to carry, widened from
 * three sensor inputs to six and with an LED driver behind each one. It sits
 * at the same default address, so the two parts are alternatives, never
 * neighbours: see touch.h, which picks one at compile time.
 *
 * Board wiring
 * ------------
 *   CS1 CS2 CS3   three discrete buttons: left, enter, right
 *   CS4 CS5 CS6   one vertical slider strip, CS6 at the top, CS4 at the bottom
 *   LED1..LED6    one indicator per pad, lit by the part itself (see below)
 *
 * The LEDs are hardware-linked to their own sensor input, so LED n follows a
 * touch on CS n with no firmware in the loop -- no I2C write, no latency, and
 * nothing to go wrong if a task stalls. cap1166_led_link() changes which of
 * them behave that way; cap1166_led_set() drives the ones that do not.
 */

/** Address with ADDR_COMM tied to VDD, which is this board's fit (table 3.1). */
#define CAP1166_I2C_ADDR    0x28

#define CAP1166_PAD1        0x01
#define CAP1166_PAD2        0x02
#define CAP1166_PAD3        0x04
#define CAP1166_PAD4        0x08
#define CAP1166_PAD5        0x10
#define CAP1166_PAD6        0x20

/** The three discrete buttons. */
#define CAP1166_PADS_BUTTON (CAP1166_PAD1 | CAP1166_PAD2 | CAP1166_PAD3)

/** The three pads that make up the slider strip. */
#define CAP1166_PADS_SLIDER (CAP1166_PAD4 | CAP1166_PAD5 | CAP1166_PAD6)

/** Every pad, which is also the reset value of the sensor enable register. */
#define CAP1166_PADS_ALL    0x3F

/**
 * @brief Verify the part ID, enable all six pads, link the LEDs. Requires i2c_init().
 *
 * Bounded: a handful of short transfers, each capped at the driver's I2C
 * timeout. An absent or unresponsive part costs a fraction of a second and
 * returns an error -- it never blocks the caller.
 */
esp_err_t cap1166_init(void);

/**
 * @brief Read which pads are currently touched and clear the latched interrupt.
 *
 * @param pads bitmask of CAP1166_PAD1..PAD6
 */
esp_err_t cap1166_read(uint8_t *pads);

/**
 * @brief Read the per-pad delta counts, the signed distance from each pad's
 *        calibrated "not touched" reference.
 *
 * This is what makes a slider out of three pads: a finger between two of them
 * raises both, and the ratio says where between them it sits. Saturates at
 * +127, which the datasheet calls out as a sign the gain is set too high --
 * see cap1166_set_sensitivity().
 *
 * @param delta receives six counts, CS1 first.
 */
esp_err_t cap1166_read_deltas(int8_t delta[6]);

/**
 * @brief Set touch sensitivity.
 *
 * Numbered exactly as the datasheet's DELTA_SENSE[2:0] field (table 5.7), so
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
esp_err_t cap1166_set_sensitivity(uint8_t delta_sense);

/**
 * @brief Choose which LEDs the part drives from their own sensor input.
 *
 * A linked LED lights while its pad is touched and goes out when released,
 * entirely inside the chip. An unlinked one answers to cap1166_led_set()
 * instead. Defaults to all six linked.
 *
 * @param pad_mask CAP1166_PAD* bits; LED n is linked when CS n's bit is set.
 */
esp_err_t cap1166_led_link(uint8_t pad_mask);

/**
 * @brief Drive the LEDs that cap1166_led_link() left unlinked.
 *
 * Bits for linked LEDs are accepted but have no visible effect -- the link
 * wins. Lit means the pin is sinking current, which is on for this board's
 * wiring (LEDs to VDD through their resistors).
 *
 * @param led_mask CAP1166_PAD* bits, reused as LED positions: PAD1 is LED1.
 */
esp_err_t cap1166_led_set(uint8_t led_mask);

/**
 * @brief Log every readable register once, by name, on the "cap1166.dbg" tag.
 *
 * A development aid for judging how the pads behave in a given enclosure.
 * Roughly 50 I2C reads, so do not put it on a hot path. Gives up at the first
 * failed read rather than waiting out a timeout per register.
 */
esp_err_t cap1166_dump_registers(void);


/* ======================================================================
 * Events
 *
 * The pads as a stream, for callers that want presses rather than states.
 *
 * Whoever polls the part consumes its events: cap1166_read() clears the
 * latched interrupt, so two pollers would steal presses from each other.
 * The task started below is therefore the board's only reader, and
 * cap1166_pads() exists so a second caller can see the state without
 * taking it away.
 * ====================================================================== */

/** What came off the panel. */
typedef enum {
    CAP1166_EVENT_NONE = 0,
    CAP1166_EVENT_LEFT,     ///< CS1
    CAP1166_EVENT_ENTER,    ///< CS2
    CAP1166_EVENT_RIGHT,    ///< CS3
    /**
     * The slider was let go. Posted once per gesture, on release, with the
     * final position readable from cap1166_slider_percent_last().
     *
     * Movement itself does not come through here: a drag produces a stream of
     * positions, and a consumer that repaints an e-paper panel must not see
     * one event per millimetre. Register a callback for the live value and
     * take this as "the gesture is over, settle up".
     */
    CAP1166_EVENT_SLIDER,
} cap1166_event_t;

/**
 * @brief Start the polling task that turns pad states into events.
 *
 * Calls cap1166_init() if it has not run yet. Idempotent.
 */
esp_err_t cap1166_start(void);

/**
 * @brief Wait for the next event.
 *
 * Buttons are leading edge only: holding a pad yields one event, not a stream.
 * Release and touch again for another.
 *
 * Always blocks for the full timeout when nothing arrives, including when the
 * part is absent and cap1166_start() failed -- callers that use this as their
 * loop's pacing stay paced whether or not the hardware is there.
 *
 * @param out         receives the event. Untouched when the wait times out.
 * @param timeout_ms  how long to block; 0 polls, UINT32_MAX waits forever.
 * @return true when @p out was filled.
 */
bool cap1166_event_wait(cap1166_event_t *out, uint32_t timeout_ms);

/** @brief Pads touched at the last poll, as a CAP1166_PAD* bitmask. */
uint8_t cap1166_pads(void);

/** @brief Whether the last poll reached the part. */
bool cap1166_pads_valid(void);

/** @brief Millisecond timestamp of the last poll. */
uint32_t cap1166_pads_updated_ms(void);


/* ======================================================================
 * Slider
 *
 * CS4, CS5 and CS6 read as one strip rather than three buttons. Position runs
 * 0 at the bottom pad (CS4) to 100 at the top (CS6), so it matches the way the
 * strip is mounted and the way a finger moves along it.
 *
 * Resolution is better than the three pads suggest: a finger bridging two of
 * them lights both, and the delta counts say how the touch divides between
 * them. It degrades gracefully -- if the deltas are saturated or equal, the
 * reading falls back to the midpoint between the touched pads, which is still
 * five distinct positions across the strip.
 * ====================================================================== */

/** Returned by cap1166_slider_percent() when no slider pad is touched. */
#define CAP1166_SLIDER_RELEASED 0xFF

/**
 * @brief Where the finger is on the strip right now.
 *
 * @return 0..100, bottom to top, or CAP1166_SLIDER_RELEASED when untouched.
 */
uint8_t cap1166_slider_percent(void);

/** @brief The last position seen while the strip was being touched. */
uint8_t cap1166_slider_percent_last(void);

/**
 * @brief Called from the polling task whenever the slider position changes.
 *
 * This is the live value, fired at the poll rate while a finger moves, so the
 * thing it drives has to be cheap -- a PWM duty, a strip refresh. Anything
 * expensive belongs on CAP1166_EVENT_SLIDER instead, which arrives once at the
 * end of the gesture.
 *
 * Runs on the touch task, so whatever it touches must tolerate that. Pass NULL
 * to remove a previously registered callback.
 *
 * @param percent 0..100, bottom to top.
 */
typedef void (*cap1166_slider_cb_t)(uint8_t percent);

/** @brief Register (or with NULL, clear) the movement callback. */
void cap1166_slider_on_change(cap1166_slider_cb_t callback);

#ifdef __cplusplus
}
#endif
