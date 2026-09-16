#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TPL5010 nano-power watchdog.
 *
 *   pin 1 DELAY -- 51 k to GND, sets the watchdog interval
 *   pin 2 DONE  -- driven by the MCU to acknowledge the interval
 *   pin 5 RSTn  -- asserted if an interval goes unacknowledged
 *
 * DONE is the only line wired to the MCU. WAKE is left unconnected, so there is
 * no edge to answer: the firmware pulses DONE on its own schedule, faster than
 * the interval the DELAY resistor programs.
 *
 * This is a whole-system reset, so it is serviced at system level: a dedicated
 * high-priority task emits the pulses. If the scheduler stops running, DONE
 * stops and the board resets. For catching a single stalled task, use the
 * IDF task watchdog (esp_task_wdt) alongside this.
 */
#define TPL5010_DONE_GPIO       15

#define TPL5010_DONE_PULSE_US   20      // datasheet minimum is well under 1 us

/*
 * Must stay comfortably under the DELAY-programmed interval. Pulsing early is
 * free -- a DONE just restarts the interval -- so this is deliberately short.
 */
#define TPL5010_DONE_PERIOD_MS  1000

/** @brief Configure DONE and start the task that pulses it. */
esp_err_t tpl5010_init(void);

#ifdef __cplusplus
}
#endif
