#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "main";
#define LED_GPIO GPIO_NUM_8 // change to your board's LED GPIO number

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    int64_t t_led = 0, t_log = 0;
    bool led_state = false;

    while(1)
    {
        int64_t t_now = esp_timer_get_time()/1000; // convert to milliseconds

        if (t_now - t_led >= 500) // 500 ms
        {
            led_state = !led_state;
            gpio_set_level(LED_GPIO, led_state);
            t_led = t_now;
        }

        if (t_now - t_log >= 1000) // 1 second
        {
            ESP_LOGI(TAG, "LED state: %s", led_state ? "ON" : "OFF");
            t_log = t_now;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // small delay to yield CPU
    }    

}
