#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "main";

typedef struct {
    uint32_t timestamp_ms;
    float temperature;
    float humidity;
}sensor_data_t;

static QueueHandle_t sensor_data_queue;

static void sensor_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        sensor_data_t data = {
            .timestamp_ms = pdTickToMS(xTaskGetTickCount()),
            .temperature = 25.0f,
            .humidity = 50.0f
        };
        xQueueSend(sensor_data_queue, &data, 0); // Never block, just send
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000)); // fixed cadence, no drift
    }
}

static void report_task(void *arg)
{
    sensor_data_t data;
    while (1)
    {
        if(xQueueReceive(sensor_data_queue, &data, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "Timestamp: %u ms, Temperature: %.2f C, Humidity: %.2f %%", data.timestamp_ms, data.temperature, data.humidity);
        }
    }
}


void app_main(void)
{
    sensor_data_queue = xQueueCreate(10, sizeof(sensor_data_t));
    configASSERT(sensor_data_queue);

    xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);
    xTaskCreate(report_task, "report", 4096, NULL, 4, NULL);
    // app_main returns: its task is deleted. That's fine and normal.

}
