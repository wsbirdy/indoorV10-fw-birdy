#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_HOST_ID         SPI2_HOST
#define SPI_SCLK_GPIO       0
#define SPI_MOSI_GPIO       1
#define SPI_MISO_GPIO       -1
#define SPI_CS_GPIO         7
#define SPI_CLK_SPEED_HZ    (10 * 1000 * 1000)
/* 4096 so a display driver can chunk on a whole 4 KB bounce buffer. */
#define SPI_MAX_TRANSFER_SZ 4096

/**
 * @brief Initialize the SPI bus (DMA enabled).
 *
 * Safe to call once at startup. Subsequent calls are no-ops.
 */
esp_err_t spi_init(void);

/**
 * @brief Attach a device on the bus.
 *
 * @param mode     SPI mode 0..3
 * @param cs_gpio  chip select for the peripheral to drive, or -1 to keep CS
 *                 out of the driver's hands and toggle it yourself
 * @param out_dev  receives the device handle
 */
esp_err_t spi_add_device(uint8_t mode, int cs_gpio, spi_device_handle_t *out_dev);

/** @brief Detach a device previously added with spi_add_device(). */
esp_err_t spi_remove_device(spi_device_handle_t dev);

/**
 * @brief Reserve the bus for @p dev until spi_release_bus().
 *
 * Required when a device drives CS itself: without it another device's
 * transaction can land in the middle of a burst that is still selected.
 * Blocks until the bus is free. Must not be nested.
 */
esp_err_t spi_acquire_bus(spi_device_handle_t dev);

/** @brief Release a bus taken with spi_acquire_bus(). */
void spi_release_bus(spi_device_handle_t dev);

/** @brief Full-duplex transfer; @p rx may be NULL for write-only. */
esp_err_t spi_transfer(spi_device_handle_t dev, const void *tx, void *rx, size_t len);

/**
 * @brief Write-only transfer that spins instead of waiting on the interrupt.
 *
 * Lower overhead than spi_transfer() for the short command bytes a display
 * sends between bulk writes, at the cost of busy-waiting the CPU.
 */
esp_err_t spi_transfer_polling(spi_device_handle_t dev, const void *tx, size_t len);

#ifdef __cplusplus
}
#endif
