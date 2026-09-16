#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_GPIO            3
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_CHANNEL         LEDC_CHANNEL_0
#define PWM_SPEED_MODE      LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION      LEDC_TIMER_10_BIT
#define PWM_FREQ_HZ         5000
#define PWM_DUTY_MAX        ((1u << PWM_RESOLUTION) - 1u)

/**
 * @brief Configure the LEDC timer and channel. Output starts at 0% duty.
 *
 * Safe to call once at startup. Subsequent calls are no-ops.
 */
esp_err_t pwm_init(void);

/** @brief Set duty as a raw value, 0..PWM_DUTY_MAX. */
esp_err_t pwm_set_duty(uint32_t duty);

/** @brief Set duty as a percentage, 0..100. */
esp_err_t pwm_set_duty_percent(float percent);

#ifdef __cplusplus
}
#endif
