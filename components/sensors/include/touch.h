#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The board's touch panel, whichever part is fitted.
 *
 * Two generations of this board exist and they carry different Microchip
 * capacitive controllers. One part is fitted, not both, and which one is a
 * property of the board rather than something worth probing for at runtime.
 *
 *   CAP1203   three pads, no LED drivers. Address 0x28.
 *             CS1/CS2/CS3 -> left, enter, right. No slider.
 *
 *   CAP1166   six pads, six LED drivers. Address set by a strapping resistor;
 *             0x2B on this board. See cap1166.h for the decode table.
 *             CS1/CS2/CS3 -> left, enter, right, each with its own LED.
 *             CS4/CS5/CS6 -> one vertical slider, CS6 top and CS4 bottom,
 *             each with its own LED. Drives the WS2812 bar's brightness.
 *
 * Everything above the driver talks to this header instead of to a part, so
 * changing the fit is the one line below and nothing else. Callers that use
 * the slider still compile against a CAP1203: touch_has_slider() is a compile
 * time constant, the position reads as released, and the dead branch is
 * dropped by the optimiser.
 */

#define TOUCH_IC_CAP1203    1203
#define TOUCH_IC_CAP1166    1166

/* ---------------------------------------------------------------------
 * BOARD FIT -- set this to the part this board carries.
 * --------------------------------------------------------------------- */
#ifndef TOUCH_IC
#define TOUCH_IC            TOUCH_IC_CAP1166
#endif

#if TOUCH_IC == TOUCH_IC_CAP1166
#include "cap1166.h"
#elif TOUCH_IC == TOUCH_IC_CAP1203
#include "cap1203.h"
#else
#error "TOUCH_IC must be TOUCH_IC_CAP1203 or TOUCH_IC_CAP1166"
#endif

/** Button pads, in the bit positions both parts use for CS1..CS3. */
#define TOUCH_PAD_LEFT      0x01
#define TOUCH_PAD_ENTER     0x02
#define TOUCH_PAD_RIGHT     0x04

/** What came off the panel. */
typedef enum {
    TOUCH_EVENT_NONE = 0,
    TOUCH_EVENT_LEFT,
    TOUCH_EVENT_ENTER,
    TOUCH_EVENT_RIGHT,
    /**
     * The slider was let go, with its final position in
     * touch_slider_percent_last(). Never produced by a part without one.
     *
     * Movement itself does not come through here -- see
     * touch_slider_on_change(). This is the "gesture is over" tick, for work
     * too expensive to do per millimetre, such as repainting the panel.
     */
    TOUCH_EVENT_SLIDER,
} touch_event_t;

/** Returned by touch_slider_percent() when nothing is on the strip. */
#define TOUCH_SLIDER_RELEASED   0xFF

/** Live slider position, 0..100 bottom to top. Runs on the touch task. */
typedef void (*touch_slider_cb_t)(uint8_t percent);


/* ======================================================================
 * The part behind the panel. One thin mapping per fit; every one of these
 * folds away at compile time.
 * ====================================================================== */

#if TOUCH_IC == TOUCH_IC_CAP1166

#define TOUCH_IC_NAME       "CAP1166"

/** Sensor inputs on the part: buttons first, then the slider. */
#define TOUCH_PAD_COUNT     6

/** Slider pads, for reading them out of touch_pads(). */
#define TOUCH_PADS_SLIDER   CAP1166_PADS_SLIDER

static inline bool touch_has_slider(void)   { return true; }

static inline esp_err_t touch_start(void)   { return cap1166_start(); }
static inline uint8_t  touch_pads(void)     { return cap1166_pads(); }
static inline bool     touch_pads_valid(void)      { return cap1166_pads_valid(); }
static inline uint32_t touch_pads_updated_ms(void) { return cap1166_pads_updated_ms(); }

/** Per-pad base counts, CS1 first. Reads the part, so not for a hot path. */
static inline esp_err_t touch_read_base_counts(uint8_t base[TOUCH_PAD_COUNT])
{
    return cap1166_read_base_counts(base);
}

static inline bool touch_event_wait(touch_event_t *out, uint32_t timeout_ms)
{
    cap1166_event_t event = CAP1166_EVENT_NONE;
    if (!cap1166_event_wait(&event, timeout_ms)) {
        return false;
    }

    // Spelled out rather than cast: the two enumerations agreeing today is not
    // a reason for a silent breakage the day one of them gains a member.
    switch (event) {
    case CAP1166_EVENT_LEFT:   *out = TOUCH_EVENT_LEFT;   return true;
    case CAP1166_EVENT_ENTER:  *out = TOUCH_EVENT_ENTER;  return true;
    case CAP1166_EVENT_RIGHT:  *out = TOUCH_EVENT_RIGHT;  return true;
    case CAP1166_EVENT_SLIDER: *out = TOUCH_EVENT_SLIDER; return true;
    case CAP1166_EVENT_NONE:
    default:                   return false;
    }
}

static inline uint8_t touch_slider_percent(void)      { return cap1166_slider_percent(); }
static inline uint8_t touch_slider_percent_last(void) { return cap1166_slider_percent_last(); }

static inline void touch_slider_on_change(touch_slider_cb_t callback)
{
    cap1166_slider_on_change(callback);
}

#else   /* TOUCH_IC_CAP1203 */

#define TOUCH_IC_NAME       "CAP1203"

/** Sensor inputs on the part. */
#define TOUCH_PAD_COUNT     3

/** No slider pads on this part, so nothing in touch_pads() belongs to one. */
#define TOUCH_PADS_SLIDER   0x00

static inline bool touch_has_slider(void)   { return false; }

static inline esp_err_t touch_start(void)   { return cap1203_buttons_start(); }
static inline uint8_t  touch_pads(void)     { return cap1203_pads(); }
static inline bool     touch_pads_valid(void)      { return cap1203_pads_valid(); }
static inline uint32_t touch_pads_updated_ms(void) { return cap1203_pads_updated_ms(); }

/** Per-pad base counts, CS1 first. Reads the part, so not for a hot path. */
static inline esp_err_t touch_read_base_counts(uint8_t base[TOUCH_PAD_COUNT])
{
    return cap1203_read_base_counts(base);
}

static inline bool touch_event_wait(touch_event_t *out, uint32_t timeout_ms)
{
    cap1203_button_t button = CAP1203_BUTTON_NONE;
    if (!cap1203_button_wait(&button, timeout_ms)) {
        return false;
    }

    switch (button) {
    case CAP1203_BUTTON_LEFT:  *out = TOUCH_EVENT_LEFT;  return true;
    case CAP1203_BUTTON_ENTER: *out = TOUCH_EVENT_ENTER; return true;
    case CAP1203_BUTTON_RIGHT: *out = TOUCH_EVENT_RIGHT; return true;
    case CAP1203_BUTTON_NONE:
    default:                   return false;
    }
}

/* A part with no strip reads as permanently released, so a caller written for
 * the slider still builds and simply never sees one move. */
static inline uint8_t touch_slider_percent(void)      { return TOUCH_SLIDER_RELEASED; }
static inline uint8_t touch_slider_percent_last(void) { return 0; }
static inline void touch_slider_on_change(touch_slider_cb_t callback) { (void)callback; }

#endif

#ifdef __cplusplus
}
#endif
