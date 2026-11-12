#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CAP1133 -- Microchip (ex-SMSC) 3-channel capacitive touch sensor with
 * 3 LED drivers, 10-pin 3 x 3 mm DFN.
 *
 * The CAP1166's register map cut down to three sensor inputs and three LED
 * drivers: every register this driver touches sits at the same address and
 * means the same thing on both parts. Which part a board carries is a
 * compile-time choice -- see touch.h.
 *
 * Expected wiring
 * ---------------
 *   CS1 CS2 CS3   three discrete buttons: left, enter, right. No slider -- the
 *                 part has no pads to spare for one.
 *   LED1..LED3    one indicator per pad, each through its own resistor to
 *                 +3.3V. The part sinks the current (open drain, its reset
 *                 default), so an LED is lit when its output is driven low.
 *   ALERT#        pin 1, open drain, active low. See CAP1133_ALERT_GPIO.
 *
 * There is no RESET pin and no address strap on this part: it always answers
 * at 0x28, and the only reset is a power cycle. A firmware restart therefore
 * finds the part configured by whatever ran before, which is why
 * cap1133_init() writes every register it depends on, defaults included.
 *
 * As on the CAP1166, the LEDs are hardware-linked to their own sensor input
 * and the panel idles lit, going dark under a finger -- see cap1133.c.
 */

/*
 * Fixed: "The CAP1133 responds to SMBus address 0101_000" (datasheet 3.2).
 * The same address as the CAP1203, so a board fitted with the other part still
 * answers here; cap1133_init() tells the two apart by product ID.
 */
#define CAP1133_I2C_ADDR    0x28

/*
 * ALERT# -- pin 1 on the DFN. Assumed to be routed to the same Touch_INT net
 * (GPIO 6, pulled up) as the CAP1166 fit. Set -1 to poll instead, which still
 * works at the cost of up to 50 ms of latency and steady bus traffic.
 */
#define CAP1133_ALERT_GPIO  6

#define CAP1133_PAD1        0x01
#define CAP1133_PAD2        0x02
#define CAP1133_PAD3        0x04

/** Every pad, which is also the reset value of the sensor enable register. */
#define CAP1133_PADS_ALL    0x07

/**
 * @brief Verify the part ID, enable all three pads, link the LEDs. Requires i2c_init().
 *
 * Bounded: a handful of short transfers, each capped at the driver's I2C
 * timeout. An absent part costs a fraction of a second and returns an error.
 */
esp_err_t cap1133_init(void);

/**
 * @brief Read which pads are currently touched and clear the latched interrupt.
 *
 * @param pads bitmask of CAP1133_PAD1..PAD3
 */
esp_err_t cap1133_read(uint8_t *pads);

/**
 * @brief Read each pad's delta count: its signed distance from the calibrated
 *        "not touched" reference. A pad reads as touched once this reaches
 *        its threshold. Saturates at +127.
 *
 * @param delta receives three counts, CS1 first.
 */
esp_err_t cap1133_read_deltas(int8_t delta[3]);

/**
 * @brief Read each pad's touch threshold, in the same units as the deltas.
 *
 * @param threshold receives three values, CS1 first.
 */
esp_err_t cap1133_read_thresholds(uint8_t threshold[3]);

/**
 * @brief Set touch sensitivity, numbered as the datasheet's DELTA_SENSE[2:0]
 *        field (table 5.8): 0 = 128x (most sensitive) .. 7 = 1x. Power-on
 *        default is 2 (32x).
 */
esp_err_t cap1133_set_sensitivity(uint8_t delta_sense);

/**
 * @brief Choose which LEDs the part drives from their own sensor input.
 *
 * A linked LED goes out while its pad is touched, entirely inside the chip.
 * An unlinked one answers to cap1133_led_set() instead.
 *
 * @param pad_mask CAP1133_PAD* bits; LED n is linked when CS n's bit is set.
 */
esp_err_t cap1133_led_link(uint8_t pad_mask);

/**
 * @brief Drive the LEDs that cap1133_led_link() left unlinked.
 *
 * @param led_mask CAP1133_PAD* bits reused as LED positions. A set bit is a
 *                 lit LED.
 */
esp_err_t cap1133_led_set(uint8_t led_mask);

/**
 * @brief Put the LEDs out after a stretch with no touch.
 *
 * Same contract as cap1166_led_idle_timeout(): the touch that wakes a dark
 * panel is spent on waking it and posts no event.
 *
 * @param timeout_ms idle time before the LEDs go dark; 0 keeps them lit.
 */
void cap1133_led_idle_timeout(uint32_t timeout_ms);

/** @brief Log every readable register once, by name, on the "cap1133.dbg" tag. */
esp_err_t cap1133_dump_registers(void);


/* ======================================================================
 * Events
 *
 * The task started below is the board's only reader -- cap1133_read() clears
 * the latched interrupt, so a second poller would steal presses.
 * cap1133_pads() lets anyone else see the state without taking it away.
 * ====================================================================== */

typedef enum {
    CAP1133_EVENT_NONE = 0,
    CAP1133_EVENT_LEFT,     ///< CS1
    CAP1133_EVENT_ENTER,    ///< CS2
    CAP1133_EVENT_RIGHT,    ///< CS3
} cap1133_event_t;

/** @brief Start the polling task. Calls cap1133_init() if needed. Idempotent. */
esp_err_t cap1133_start(void);

/**
 * @brief Wait for the next press. Leading edge only.
 *
 * Always blocks for the full timeout when nothing arrives, including when the
 * part is absent, so a caller using it for pacing stays paced.
 *
 * @return true when @p out was filled.
 */
bool cap1133_event_wait(cap1133_event_t *out, uint32_t timeout_ms);

/** @brief Pads touched at the last poll, as a CAP1133_PAD* bitmask. */
uint8_t cap1133_pads(void);

/** @brief Whether the last poll reached the part. */
bool cap1133_pads_valid(void);

/** @brief Millisecond timestamp of the last poll. */
uint32_t cap1133_pads_updated_ms(void);

#ifdef __cplusplus
}
#endif
