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
 * three sensor inputs to six and with an LED driver behind each one. Which of
 * the two a board carries is a compile-time choice -- see touch.h.
 *
 * Board wiring (U3 on the Capacitive Touch sheet, 20-QFN)
 * -------------------------------------------------------
 *   CS1 CS2 CS3   touch1..3_pad, three discrete buttons: left, enter, right
 *   CS4 CS5 CS6   touch4..6_pad, one vertical slider strip. CS6 is the pad at
 *                 the top of the strip and CS4 the one at the bottom, which is
 *                 the order the position in cap1166_slider_percent() runs in.
 *   LED1..LED6    D4..D9, one green indicator per pad, each through its own
 *                 resistor to +3.3V. The part sinks the current, so an LED is
 *                 lit when its output is driven low.
 *   ALERT#        pulled up to +3.3V and brought out as Touch_INT on GPIO 25.
 *                 What wakes the driver when a pad changes, so it can sleep
 *                 between touches instead of polling. See CAP1166_ALERT_GPIO.
 *   RESET         not driven by the MCU. See CAP1166_RESET_GPIO.
 *   ADDR_COMM     strapped to ground, see CAP1166_I2C_ADDR below.
 *
 * The LEDs are hardware-linked to their own sensor input, so LED n answers a
 * touch on CS n with no firmware in the loop -- no I2C write, no latency, and
 * nothing to go wrong if a task stalls. cap1166_led_link() changes which of
 * them behave that way; cap1166_led_set() drives the ones that do not.
 *
 * The panel idles lit and goes dark under a finger, which is the opposite of
 * the part's reset behaviour and costs one register to arrange -- see the LED
 * polarity note in cap1166.c. It is still the link doing the work: the LED
 * that goes out is chosen inside the part, not by this driver.
 */

/*
 * I2C address, set in hardware by the resistor from ADDR_COMM (pin 12 on the
 * 20-QFN) to ground. Datasheet table 3.1:
 *
 *      GND     SPI, 4-wire          -- not I2C at all
 *      56k     SPI, 3-wire          -- not I2C at all
 *      68k     BC-Link              -- not I2C at all
 *      82k     0101_100  =  0x2C
 *   -> 100k    0101_011  =  0x2B    <- this board
 *      120k    0101_010  =  0x2A
 *      150k    0101_001  =  0x29
 *      VDD     0101_000  =  0x28
 *
 * Note the bottom three rows are not I2C: a board that grounds ADDR_COMM
 * outright, or gets the resistor value badly wrong, puts the part on SPI and
 * it will never answer here no matter what address is tried.
 */
#define CAP1166_I2C_ADDR    0x2B

/*
 * RESET -- pin 20 on the 20-QFN. No longer wired to the MCU, hence -1: the
 * driver then skips the reset pulse and talks to the part as power-on left it.
 *
 * Active high, and unusually strong for a reset: while it is high the part is
 * held in deep sleep with the communications block itself in reset, so it will
 * not even acknowledge its address, and every register is forced back to its
 * default. Driving it low is what starts the part.
 *
 * That is why -1 is not a safe value here when the line is wired to the MCU.
 * An output that is never configured stays an input, the line floats, and a
 * floating RESET that settles high holds the part asleep forever -- which
 * looks exactly like a dead or mis-addressed part.
 *
 * So -1 relies on the board tying RESET to ground itself, which is the
 * datasheet's "if not used, connect to ground". Set a GPIO number here again
 * if a board revision routes Touch_RST back to the MCU.
 *
 * Without the pulse, a firmware restart (as opposed to a power cycle) leaves
 * the part configured by whatever ran before. cap1166_init() rewrites every
 * register it depends on, so that only matters for registers it leaves alone.
 */
#define CAP1166_RESET_GPIO  -1

/*
 * ALERT# -- pin 11 on the 20-QFN, brought out as Touch_INT on GPIO 25 and
 * pulled to +3.3V through R19.
 *
 * Active low and open drain, which is the part's reset default and what the
 * board's pull-up expects. It asserts when a pad is touched or released and
 * stays asserted until the interrupt is cleared, which cap1166_read() does on
 * every pass.
 *
 * With this set the driver sleeps between touches instead of polling at a
 * fixed rate, so a press is picked up as soon as the part reports it. At -1 it
 * falls back to polling, which still works -- it just costs the poll interval
 * in latency and a steady trickle of I2C traffic.
 */
#define CAP1166_ALERT_GPIO  25

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
 * @brief Read each pad's touch threshold, in the same units as the deltas: a
 *        pad reads as touched once its delta count reaches this.
 *
 * @param threshold receives six values, CS1 first.
 */
esp_err_t cap1166_read_thresholds(uint8_t threshold[6]);

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
 * A linked LED goes out while its pad is touched and comes back when released,
 * entirely inside the chip. An unlinked one answers to cap1166_led_set()
 * instead, and holds whatever that last set. Defaults to all six linked.
 *
 * @param pad_mask CAP1166_PAD* bits; LED n is linked when CS n's bit is set.
 */
esp_err_t cap1166_led_link(uint8_t pad_mask);

/**
 * @brief Drive the LEDs that cap1166_led_link() left unlinked.
 *
 * Bits for linked LEDs are accepted but have no visible effect -- the link
 * wins.
 *
 * @param led_mask CAP1166_PAD* bits, reused as LED positions: PAD1 is LED1.
 *                 A set bit is a lit LED, which is the opposite of the bit
 *                 that reaches the register; see the definition.
 */
esp_err_t cap1166_led_set(uint8_t led_mask);

/**
 * @brief Put the LEDs out after a stretch with no touch.
 *
 * Once @p timeout_ms passes with every pad untouched, all six LEDs go dark and
 * stop following their pads. The next touch on any pad brings them back, and
 * is used up doing so: it posts no event and moves no slider, so a tap on a
 * dark panel only wakes it. The clock restarts on every touch and whenever
 * this is called.
 *
 * Handled by the task cap1166_start() runs; safe to call from any task, before
 * or after it starts. Lands up to half a second after the deadline.
 *
 * @param timeout_ms idle time before the LEDs go dark; 0 keeps them lit.
 */
void cap1166_led_idle_timeout(uint32_t timeout_ms);

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
