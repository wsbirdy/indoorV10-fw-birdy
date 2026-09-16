#include "sdkconfig.h"
#include "uart.h"
#include "freertos/FreeRTOS.h"
#include "esp_check.h"
#include "esp_log.h"

/*
 * uart_init() reconfigures whatever port it is handed, baud rate included. If
 * one of them is also the console the reconfiguration is invisible -- logging
 * keeps working, just at the sensor's baud rate -- and the first burst of log
 * lines then blocks its caller for seconds and trips the task watchdog. The
 * console lives on the USB Serial/JTAG peripheral for exactly this reason
 * (see sdkconfig.defaults); this catches anyone moving it back.
 */
#if defined(CONFIG_ESP_CONSOLE_UART) && !defined(CONFIG_ESP_CONSOLE_NONE)
#if CONFIG_ESP_CONSOLE_UART_NUM == 0 || CONFIG_ESP_CONSOLE_UART_NUM == 1
#error "Console is on a UART this module reconfigures. Move the console to USB Serial/JTAG, or drop that port from s_cfg."
#endif
#endif

static const char *TAG = "uart";

typedef struct {
    const char *name;
    uart_port_t port;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
    int rx_buf_size;
    bool is_lp;         // LP UART takes its clock from the LP domain
} uart_cfg_t;

static const uart_cfg_t s_cfg[UART_ID_MAX] = {
    [UART_ID_0] = {
        .name = "uart0",
        .port = UART0_PORT_NUM,
        .tx_gpio = UART0_TX_GPIO,
        .rx_gpio = UART0_RX_GPIO,
        .baud_rate = UART0_BAUD_RATE,
        .rx_buf_size = UART_RX_BUF_SIZE,
        .is_lp = false,
    },
    [UART_ID_1] = {
        .name = "uart1",
        .port = UART1_PORT_NUM,
        .tx_gpio = UART1_TX_GPIO,
        .rx_gpio = UART1_RX_GPIO,
        .baud_rate = UART1_BAUD_RATE,
        .rx_buf_size = UART_RX_BUF_SIZE,
        .is_lp = false,
    },
    [UART_ID_LP] = {
        .name = "lpuart",
        .port = LPUART_PORT_NUM,
        .tx_gpio = LPUART_TX_GPIO,
        .rx_gpio = LPUART_RX_GPIO,
        .baud_rate = LPUART_BAUD_RATE,
        .rx_buf_size = LPUART_RX_BUF_SIZE,
        .is_lp = true,
    },
};

static bool s_ready[UART_ID_MAX];

esp_err_t uart_init(uart_id_t id)
{
    ESP_RETURN_ON_FALSE(id < UART_ID_MAX, ESP_ERR_INVALID_ARG, TAG, "bad uart id %d", id);

    if (s_ready[id]) {
        return ESP_OK; // already up
    }

    const uart_cfg_t *c = &s_cfg[id];

    uart_config_t cfg = {
        .baud_rate = c->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // source_clk and lp_source_clk share a union; pick the one for this domain
    if (c->is_lp) {
        cfg.lp_source_clk = LP_UART_SCLK_DEFAULT;
    } else {
        cfg.source_clk = UART_SCLK_DEFAULT;
    }

    ESP_RETURN_ON_ERROR(uart_driver_install(c->port, c->rx_buf_size, UART_TX_BUF_SIZE, 0, NULL, 0),
                        TAG, "%s: driver install failed", c->name);
    ESP_RETURN_ON_ERROR(uart_param_config(c->port, &cfg),
                        TAG, "%s: param config failed", c->name);
    ESP_RETURN_ON_ERROR(uart_set_pin(c->port, c->tx_gpio, c->rx_gpio,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "%s: set pin failed", c->name);

    s_ready[id] = true;

    if (c->tx_gpio == UART_PIN_NO_CHANGE) {
        ESP_LOGI(TAG, "%s ready (default pins, %d baud)", c->name, c->baud_rate);
    } else {
        ESP_LOGI(TAG, "%s ready (tx=%d rx=%d, %d baud)",
                 c->name, c->tx_gpio, c->rx_gpio, c->baud_rate);
    }
    return ESP_OK;
}

esp_err_t uart_init_all(void)
{
    for (int id = 0; id < UART_ID_MAX; id++) {
        ESP_RETURN_ON_ERROR(uart_init(id), TAG, "init of %s failed", s_cfg[id].name);
    }
    return ESP_OK;
}

int uart_write(uart_id_t id, const void *data, size_t len)
{
    if (id >= UART_ID_MAX || !s_ready[id]) {
        return -1;
    }
    return uart_write_bytes(s_cfg[id].port, data, len);
}

int uart_read(uart_id_t id, void *buf, size_t len, uint32_t timeout_ms)
{
    if (id >= UART_ID_MAX || !s_ready[id]) {
        return -1;
    }
    return uart_read_bytes(s_cfg[id].port, buf, len, pdMS_TO_TICKS(timeout_ms));
}
