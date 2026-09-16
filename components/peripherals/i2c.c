#include "i2c.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "i2c";

static i2c_master_bus_handle_t s_bus;

esp_err_t i2c_init(void)
{
    if (s_bus) {
        return ESP_OK; // already up
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "bus create failed");

    ESP_LOGI(TAG, "master ready on port %d (sda=%d scl=%d, %d Hz)",
             I2C_PORT_NUM, I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_CLK_SPEED_HZ);
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_get_bus(void)
{
    return s_bus;
}

esp_err_t i2c_add_device_at_speed(uint8_t addr_7bit, uint32_t scl_speed_hz,
                                  i2c_master_dev_handle_t *out_dev)
{
    ESP_RETURN_ON_FALSE(s_bus, ESP_ERR_INVALID_STATE, TAG, "bus not initialized");
    ESP_RETURN_ON_FALSE(out_dev, ESP_ERR_INVALID_ARG, TAG, "out_dev is NULL");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_7bit,
        .scl_speed_hz = scl_speed_hz,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, out_dev), TAG,
                        "add device 0x%02X failed", addr_7bit);

    if (scl_speed_hz == I2C_CLK_SPEED_HZ) {
        ESP_LOGI(TAG, "device 0x%02X attached", addr_7bit);
    } else {
        ESP_LOGI(TAG, "device 0x%02X attached at %lu Hz", addr_7bit,
                 (unsigned long)scl_speed_hz);
    }
    return ESP_OK;
}

esp_err_t i2c_add_device(uint8_t addr_7bit, i2c_master_dev_handle_t *out_dev)
{
    return i2c_add_device_at_speed(addr_7bit, I2C_CLK_SPEED_HZ, out_dev);
}
