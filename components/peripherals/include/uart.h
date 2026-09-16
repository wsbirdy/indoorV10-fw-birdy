#pragma once

#include <stddef.h>
#include <stdint.h>
#include "driver/uart.h"
#include "soc/uart_pins.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * UART0 - the PMS5003, on the port's IOMUX default pins (GPIO 11 TX / 12 RX on
 * the C5). These are also the console's default pins, which is why the console
 * is configured onto USB Serial/JTAG instead; uart.c has a build guard for it.
 *
 * Named explicitly rather than left at UART_PIN_NO_CHANGE: that value means
 * "mux nothing", so the port only reached the pads back when the console had
 * already routed them. With the console moved off UART0 nobody does, and the
 * peripheral would run happily while connected to no pin at all.
 */
#define UART0_PORT_NUM      UART_NUM_0
#define UART0_TX_GPIO       U0TXD_GPIO_NUM
#define UART0_RX_GPIO       U0RXD_GPIO_NUM
#define UART0_BAUD_RATE     9600        // PMS5003 runs at 9600 8N1

/* UART1 */
#define UART1_PORT_NUM      UART_NUM_1
#define UART1_TX_GPIO       23
#define UART1_RX_GPIO       24
#define UART1_BAUD_RATE     115200

/* LP UART - TX/RX must stay within the chip's LP IO range */
#define LPUART_PORT_NUM     LP_UART_NUM_0
#define LPUART_TX_GPIO      5
#define LPUART_RX_GPIO      4
#define LPUART_BAUD_RATE    9600        // Senseair Modbus RTU runs at 9600 8N1

#define UART_RX_BUF_SIZE    1024
#define LPUART_RX_BUF_SIZE  256     // LP UART FIFO is only 16 bytes deep
#define UART_TX_BUF_SIZE    0       // 0 = blocking writes, no TX ring buffer

/** @brief Logical handle for each configured port. */
typedef enum {
    UART_ID_0 = 0,  // UART0 on default TXD/RXD
    UART_ID_1,      // UART1 on GPIO 23/24
    UART_ID_LP,     // LP UART on GPIO 5/4
    UART_ID_MAX
} uart_id_t;

/**
 * @brief Initialize one port.
 *
 * Safe to call once per port at startup. Subsequent calls are no-ops.
 */
esp_err_t uart_init(uart_id_t id);

/** @brief Initialize all three ports; stops at the first failure. */
esp_err_t uart_init_all(void);

/** @brief Write @p len bytes, blocking until they are queued to the FIFO. */
int uart_write(uart_id_t id, const void *data, size_t len);

/**
 * @brief Read up to @p len bytes.
 *
 * @param timeout_ms how long to wait for the first byte
 * @return number of bytes read, or -1 on error
 */
int uart_read(uart_id_t id, void *buf, size_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
