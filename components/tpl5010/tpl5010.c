#include "tpl5010.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "tpl5010";

#define WDT_TASK_STACK      2560
#define WDT_TASK_PRIO       10      // above the sensor tasks: servicing must not be starved

static TaskHandle_t s_task;

static void done_pulse(void)
{
    gpio_set_level(TPL5010_DONE_GPIO, 1);
    esp_rom_delay_us(TPL5010_DONE_PULSE_US);
    gpio_set_level(TPL5010_DONE_GPIO, 0);
}

/*
 * Pulsing from a task is what makes this a watchdog: DONE only goes out while
 * the scheduler is still running, so a locked-up system stops being
 * acknowledged and the TPL5010 resets it.
 */
static void wdt_task(void *arg)
{
    (void)arg;

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        done_pulse();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TPL5010_DONE_PERIOD_MS));
    }
}

esp_err_t tpl5010_init(void)
{
    if (s_task) {
        return ESP_OK;
    }

    const gpio_config_t done_cfg = {
        .pin_bit_mask = (1ULL << TPL5010_DONE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&done_cfg), TAG, "done config failed");
    gpio_set_level(TPL5010_DONE_GPIO, 0);

    BaseType_t ok = xTaskCreate(wdt_task, "tpl5010", WDT_TASK_STACK, NULL, WDT_TASK_PRIO, &s_task);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "task create failed");

    ESP_LOGI(TAG, "ready (done=%d, every %d ms)", TPL5010_DONE_GPIO, TPL5010_DONE_PERIOD_MS);
    return ESP_OK;
}
