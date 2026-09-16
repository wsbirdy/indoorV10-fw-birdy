#include <string.h>
#include "spi.h"
#include "freertos/FreeRTOS.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "spi";

static bool s_ready;

esp_err_t spi_init(void)
{
    if (s_ready) {
        return ESP_OK; // already up
    }

    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = SPI_SCLK_GPIO,
        .mosi_io_num = SPI_MOSI_GPIO,
        .miso_io_num = SPI_MISO_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_TRANSFER_SZ,
    };

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI_HOST_ID, &bus_cfg, SPI_DMA_CH_AUTO),
                        TAG, "bus init failed");

    s_ready = true;
    ESP_LOGI(TAG, "host %d ready (sclk=%d mosi=%d miso=%d)",
             SPI_HOST_ID, SPI_SCLK_GPIO, SPI_MOSI_GPIO, SPI_MISO_GPIO);
    return ESP_OK;
}

esp_err_t spi_add_device(uint8_t mode, int cs_gpio, spi_device_handle_t *out_dev)
{
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "bus not initialized");
    ESP_RETURN_ON_FALSE(out_dev, ESP_ERR_INVALID_ARG, TAG, "out_dev is NULL");

    const spi_device_interface_config_t dev_cfg = {
        .mode = mode,
        .clock_speed_hz = SPI_CLK_SPEED_HZ,
        .spics_io_num = cs_gpio,
        .queue_size = 4,
        .flags = SPI_DEVICE_NO_DUMMY,
    };

    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI_HOST_ID, &dev_cfg, out_dev), TAG,
                        "add device failed");

    if (cs_gpio < 0) {
        ESP_LOGI(TAG, "device attached (cs driven by caller, mode=%u, %d Hz)",
                 mode, SPI_CLK_SPEED_HZ);
    } else {
        ESP_LOGI(TAG, "device attached (cs=%d mode=%u, %d Hz)", cs_gpio, mode, SPI_CLK_SPEED_HZ);
    }
    return ESP_OK;
}

esp_err_t spi_remove_device(spi_device_handle_t dev)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");

    return spi_bus_remove_device(dev);
}

esp_err_t spi_acquire_bus(spi_device_handle_t dev)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");

    // Finite timeouts are rejected by the driver; portMAX_DELAY is the only option.
    return spi_device_acquire_bus(dev, portMAX_DELAY);
}

void spi_release_bus(spi_device_handle_t dev)
{
    if (dev) {
        spi_device_release_bus(dev);
    }
}

esp_err_t spi_transfer_polling(spi_device_handle_t dev, const void *tx, size_t len)
{
    ESP_RETURN_ON_FALSE(dev && len, ESP_ERR_INVALID_ARG, TAG, "bad transfer args");

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8; // bits, not bytes
    t.tx_buffer = tx;

    return spi_device_polling_transmit(dev, &t);
}

esp_err_t spi_transfer(spi_device_handle_t dev, const void *tx, void *rx, size_t len)
{
    ESP_RETURN_ON_FALSE(dev && len, ESP_ERR_INVALID_ARG, TAG, "bad transfer args");

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8; // bits, not bytes
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    return spi_device_transmit(dev, &t);
}
