#include "pwm.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "pwm";

static bool s_ready;

esp_err_t pwm_init(void)
{
    if (s_ready) {
        return ESP_OK; // already up
    }

    const ledc_timer_config_t timer_cfg = {
        .speed_mode = PWM_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "timer config failed");

    const ledc_channel_config_t ch_cfg = {
        .speed_mode = PWM_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "channel config failed");

    s_ready = true;
    ESP_LOGI(TAG, "channel %d ready (gpio=%d, %d Hz, %d-bit)",
             PWM_CHANNEL, PWM_GPIO, PWM_FREQ_HZ, PWM_RESOLUTION);
    return ESP_OK;
}

esp_err_t pwm_set_duty(uint32_t duty)
{
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    if (duty > PWM_DUTY_MAX) {
        duty = PWM_DUTY_MAX;
    }

    ESP_RETURN_ON_ERROR(ledc_set_duty(PWM_SPEED_MODE, PWM_CHANNEL, duty), TAG, "set duty failed");
    return ledc_update_duty(PWM_SPEED_MODE, PWM_CHANNEL);
}

esp_err_t pwm_set_duty_percent(float percent)
{
    if (percent < 0.0f) {
        percent = 0.0f;
    } else if (percent > 100.0f) {
        percent = 100.0f;
    }

    return pwm_set_duty((uint32_t)((percent / 100.0f) * PWM_DUTY_MAX + 0.5f));
}
