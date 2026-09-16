/*
 * SSD1680 panel driver ported from the AirGradient Go display library
 * (example/go_display_driver/go_display.cpp, file-local `namespace { }` block:
 * driver_init, driver_hw_init_full/_fast, driver_set_basemap, driver_part_*,
 * driver_fast_*, driver_deep_sleep).
 *
 * The command sequences, register values, reset timings and RAM addressing are
 * transcribed as-is. What changed:
 *   - C++ -> C, and the DisplayService::Config pin fields are replaced by the
 *     board's fixed pins from epaper.h / spi.h.
 *   - RTOS::delay_ms -> vTaskDelay.
 *   - The bounce buffer and SPI device come from the shared peripherals
 *     component instead of being allocated against a private bus config.
 *
 * The rest of go_display.cpp -- DisplayService, the u8g2 screen rendering and
 * the AirGradient product UI -- is not built here: it needs u8g2, rtos.h,
 * measures_types.h, provisioning_qr.h and go_text_wrap.h, none of which are
 * vendored, and its screens are driven by BLE/Wi-Fi/GPS/fuel-gauge state this
 * board does not have. the originals are kept in example/go_display_driver/ as reference.
 */

#include <string.h>
#include "epaper.h"
#include "spi.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "epaper";

/* go_display.cpp: WIDTH_PX / HEIGHT_PX / FRAME_BYTES / TX_BOUNCE_BYTES.
 * The panel is 122 columns of glass inside a 128-bit RAM row, so FRAME_BYTES
 * matches EPD_FB_SIZE (16 * 250 = 4000). */
#define WIDTH_PX            EPD_RAM_WIDTH       // 128
#define HEIGHT_PX           EPD_HEIGHT          // 250
#define FRAME_BYTES         EPD_FB_SIZE         // 4000
#define TX_BOUNCE_BYTES     4096

/* Poll cadence must be >= 1 tick; a sub-tick delay degrades to vTaskDelay(0)
 * (bare yield), starving lower-priority tasks during refresh. */
#define BUSY_POLL_MS        10

static spi_device_handle_t s_spi;
static uint8_t *s_tx_bounce;
static bool s_inited;
static uint8_t s_since_full;    // fast/partial refreshes since the last full one

/* --- Low-level GPIO helpers (go_display.cpp: set_cs / set_dc / set_rst) --- */

static inline void set_cs(int level)  { gpio_set_level(EPD_CS_GPIO, level); }
static inline void set_dc(int level)  { gpio_set_level(EPD_DC_GPIO, level); }
static inline void set_rst(int level) { gpio_set_level(EPD_RST_GPIO, level); }

static void delay_ms(uint32_t ms)
{
    if (ms == 0) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* BUSY=1 means controller processing; BUSY=0 means ready. */
static void wait_busy_low(void)
{
    while (gpio_get_level(EPD_BUSY_GPIO) != 0) {
        delay_ms(BUSY_POLL_MS);
    }
}

/* --- SPI transfer primitives --- */

static esp_err_t spi_write_byte(uint8_t value)
{
    s_tx_bounce[0] = value;
    return spi_transfer_polling(s_spi, s_tx_bounce, 1);
}

static esp_err_t spi_write_bulk(const uint8_t *data, size_t len)
{
    size_t offset = 0;
    while (offset < len) {
        const size_t chunk = (len - offset > TX_BOUNCE_BYTES) ? TX_BOUNCE_BYTES : (len - offset);
        memcpy(s_tx_bounce, data + offset, chunk);
        ESP_RETURN_ON_ERROR(spi_transfer_polling(s_spi, s_tx_bounce, chunk), TAG, "bulk write failed");
        offset += chunk;
    }
    return ESP_OK;
}

/* Not in go_display.cpp, which always had a caller framebuffer to point at.
 * epaper_clear() has none, so this streams a constant instead of keeping a
 * whole blank frame in .bss. */
static esp_err_t spi_write_fill(uint8_t value, size_t len)
{
    memset(s_tx_bounce, value, (len < TX_BOUNCE_BYTES) ? len : TX_BOUNCE_BYTES);

    size_t offset = 0;
    while (offset < len) {
        const size_t chunk = (len - offset > TX_BOUNCE_BYTES) ? TX_BOUNCE_BYTES : (len - offset);
        ESP_RETURN_ON_ERROR(spi_transfer_polling(s_spi, s_tx_bounce, chunk), TAG, "fill write failed");
        offset += chunk;
    }
    return ESP_OK;
}

/* --- SSD1680 command/data protocol --- */

static esp_err_t write_cmd(uint8_t command)
{
    set_cs(0);
    set_dc(0);
    const esp_err_t err = spi_write_byte(command);
    set_cs(1);
    return err;
}

static esp_err_t write_data(uint8_t data)
{
    set_cs(0);
    set_dc(1);
    const esp_err_t err = spi_write_byte(data);
    set_cs(1);
    return err;
}

static esp_err_t write_data_bytes(const uint8_t *data, size_t len)
{
    set_cs(0);
    set_dc(1);
    const esp_err_t err = spi_write_bulk(data, len);
    set_cs(1);
    return err;
}

static esp_err_t write_data_fill(uint8_t value, size_t len)
{
    set_cs(0);
    set_dc(1);
    const esp_err_t err = spi_write_fill(value, len);
    set_cs(1);
    return err;
}

/* --- Driver bring-up (go_display.cpp: driver_init) --- */

esp_err_t epaper_pins_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(spi_init(), TAG, "spi bus init failed");

    // Configure output GPIOs (CS, DC, RST)
    const gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << EPD_CS_GPIO) | (1ULL << EPD_DC_GPIO) | (1ULL << EPD_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out_cfg), TAG, "cs/dc/rst config failed");

    // Configure BUSY input
    const gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << EPD_BUSY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&in_cfg), fail, TAG, "busy config failed");

    set_cs(1);
    set_dc(1);
    set_rst(1);

    // DMA-capable bounce buffer for chunked SPI writes
    s_tx_bounce = heap_caps_malloc(TX_BOUNCE_BYTES, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(s_tx_bounce, ESP_ERR_NO_MEM, fail, TAG, "no DMA memory for tx buffer");

    // Attach SPI device (CS managed manually, not by the ESP-IDF driver)
    ESP_GOTO_ON_ERROR(spi_add_device(0, -1, &s_spi), fail, TAG, "spi attach failed");

    delay_ms(100);
    s_inited = true;
    ESP_LOGI(TAG, "EPD driver init done (sclk=%d mosi=%d cs=%d dc=%d rst=%d busy=%d)",
             SPI_SCLK_GPIO, SPI_MOSI_GPIO, EPD_CS_GPIO,
             EPD_DC_GPIO, EPD_RST_GPIO, EPD_BUSY_GPIO);
    return ESP_OK;

fail:
    if (s_spi) {
        spi_remove_device(s_spi);
        s_spi = NULL;
    }
    if (s_tx_bounce) {
        heap_caps_free(s_tx_bounce);
        s_tx_bounce = NULL;
    }
    gpio_reset_pin(EPD_CS_GPIO);
    gpio_reset_pin(EPD_DC_GPIO);
    gpio_reset_pin(EPD_RST_GPIO);
    gpio_reset_pin(EPD_BUSY_GPIO);
    return ret;
}

/* --- Full SSD1680 initialization sequence (driver_hw_init_full) --- */

static esp_err_t driver_hw_init_full(void)
{
    // Hardware reset
    set_rst(0);
    delay_ms(10);
    set_rst(1);
    delay_ms(10);
    wait_busy_low();

    // Software reset
    ESP_RETURN_ON_ERROR(write_cmd(0x12), TAG, "sw reset failed");
    wait_busy_low();

    // Driver output control: gate lines = HEIGHT_PX - 1
    ESP_RETURN_ON_ERROR(write_cmd(0x01), TAG, "driver output failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) % 256)), TAG, "gate lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) / 256)), TAG, "gate hi failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "gate scan failed");

    // Data entry mode: X increment, Y decrement
    ESP_RETURN_ON_ERROR(write_cmd(0x11), TAG, "entry mode failed");
    ESP_RETURN_ON_ERROR(write_data(0x01), TAG, "entry mode data failed");

    // RAM-X window: 0 to 15 (0..127 pixels)
    ESP_RETURN_ON_ERROR(write_cmd(0x44), TAG, "x window failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "x start failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)(WIDTH_PX / 8 - 1)), TAG, "x end failed");

    // RAM-Y window: 249 to 0
    ESP_RETURN_ON_ERROR(write_cmd(0x45), TAG, "y window failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) % 256)), TAG, "y start lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) / 256)), TAG, "y start hi failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "y end lo failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "y end hi failed");

    // Border waveform (full mode)
    ESP_RETURN_ON_ERROR(write_cmd(0x3C), TAG, "border failed");
    ESP_RETURN_ON_ERROR(write_data(0x05), TAG, "border data failed");

    // Display update control
    ESP_RETURN_ON_ERROR(write_cmd(0x21), TAG, "update ctrl failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "update ctrl a failed");
    ESP_RETURN_ON_ERROR(write_data(0x80), TAG, "update ctrl b failed");

    // Built-in temperature sensor
    ESP_RETURN_ON_ERROR(write_cmd(0x18), TAG, "temp sensor failed");
    ESP_RETURN_ON_ERROR(write_data(0x80), TAG, "temp sensor data failed");

    // RAM-X counter to 0
    ESP_RETURN_ON_ERROR(write_cmd(0x4E), TAG, "x counter failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "x counter data failed");

    // RAM-Y counter to HEIGHT_PX - 1
    ESP_RETURN_ON_ERROR(write_cmd(0x4F), TAG, "y counter failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) % 256)), TAG, "y counter lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) / 256)), TAG, "y counter hi failed");
    wait_busy_low();

    return ESP_OK;
}

/* Write frame to both SSD1680 RAM planes + trigger full update. */
static esp_err_t driver_set_basemap(const uint8_t *data)
{
    ESP_RETURN_ON_ERROR(write_cmd(0x24), TAG, "new plane failed");
    ESP_RETURN_ON_ERROR(write_data_bytes(data, FRAME_BYTES), TAG, "new plane data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x26), TAG, "old plane failed");
    ESP_RETURN_ON_ERROR(write_data_bytes(data, FRAME_BYTES), TAG, "old plane data failed");

    // Trigger full update
    ESP_RETURN_ON_ERROR(write_cmd(0x22), TAG, "update seq failed");
    ESP_RETURN_ON_ERROR(write_data(0xF7), TAG, "update seq data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x20), TAG, "activation failed");
    wait_busy_low();
    return ESP_OK;
}

/* Same, filling both planes with a constant (used by epaper_clear). */
static esp_err_t driver_set_basemap_fill(uint8_t value)
{
    ESP_RETURN_ON_ERROR(write_cmd(0x24), TAG, "new plane failed");
    ESP_RETURN_ON_ERROR(write_data_fill(value, FRAME_BYTES), TAG, "new plane fill failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x26), TAG, "old plane failed");
    ESP_RETURN_ON_ERROR(write_data_fill(value, FRAME_BYTES), TAG, "old plane fill failed");

    ESP_RETURN_ON_ERROR(write_cmd(0x22), TAG, "update seq failed");
    ESP_RETURN_ON_ERROR(write_data(0xF7), TAG, "update seq data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x20), TAG, "activation failed");
    wait_busy_low();
    return ESP_OK;
}

/* --- Partial update (driver_part_begin / _write_region / _commit) --- */

static esp_err_t driver_part_begin(void)
{
    // Hardware reset
    set_rst(0);
    delay_ms(10);
    set_rst(1);
    delay_ms(10);
    wait_busy_low();

    // Re-establish Y mapping after reset
    ESP_RETURN_ON_ERROR(write_cmd(0x01), TAG, "driver output failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) % 256)), TAG, "gate lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) / 256)), TAG, "gate hi failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "gate scan failed");

    // Border waveform (partial mode -- differs from full init)
    ESP_RETURN_ON_ERROR(write_cmd(0x3C), TAG, "border failed");
    ESP_RETURN_ON_ERROR(write_data(0x80), TAG, "border data failed");

    // Data entry mode (same as full init)
    ESP_RETURN_ON_ERROR(write_cmd(0x11), TAG, "entry mode failed");
    ESP_RETURN_ON_ERROR(write_data(0x01), TAG, "entry mode data failed");

    return ESP_OK;
}

/*
 * Write pixel data to a rectangular region for partial update.
 * Coordinates are logical screen coordinates (Y=0 at top).
 */
static esp_err_t driver_part_write_region(unsigned int x_start_px, unsigned int y_start_px,
                                          const uint8_t *data, unsigned int height_px,
                                          unsigned int width_px)
{
    const unsigned int x_start = x_start_px / 8;
    const unsigned int x_end = x_start + width_px / 8 - 1;

    // Map logical top-origin Y to controller Y (inverted).
    // Controller Y=249 corresponds to screen top (Y=0).
    const unsigned int y_start = (HEIGHT_PX - 1) - y_start_px;
    const unsigned int y_end = (HEIGHT_PX - 1) - (y_start_px + height_px - 1);

    // Set RAM window
    ESP_RETURN_ON_ERROR(write_cmd(0x44), TAG, "x window failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)x_start), TAG, "x start failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)x_end), TAG, "x end failed");

    ESP_RETURN_ON_ERROR(write_cmd(0x45), TAG, "y window failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)(y_start % 256)), TAG, "y start lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)(y_start / 256)), TAG, "y start hi failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)(y_end % 256)), TAG, "y end lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)(y_end / 256)), TAG, "y end hi failed");

    // Set RAM cursor
    ESP_RETURN_ON_ERROR(write_cmd(0x4E), TAG, "x counter failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)x_start), TAG, "x counter data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x4F), TAG, "y counter failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)(y_start % 256)), TAG, "y counter lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)(y_start / 256)), TAG, "y counter hi failed");

    // Write pixel data (new RAM plane only; partial uses diff against basemap)
    ESP_RETURN_ON_ERROR(write_cmd(0x24), TAG, "new plane failed");
    return write_data_bytes(data, (size_t)height_px * width_px / 8);
}

static esp_err_t driver_part_commit(void)
{
    ESP_RETURN_ON_ERROR(write_cmd(0x22), TAG, "update seq failed");
    ESP_RETURN_ON_ERROR(write_data(0xFF), TAG, "update seq data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x20), TAG, "activation failed");
    wait_busy_low();
    return ESP_OK;
}

/*
 * Fast refresh (0xC7 waveform) -- full-screen, no flash.
 * Uses a temperature-override trick: force the SSD1680 to load the OTP LUT for
 * 100 C, which selects a shorter, more aggressive waveform. The 0xC7 trigger
 * then drives all pixels unconditionally (non-differential).
 */
static esp_err_t driver_hw_init_fast(void)
{
    // Hardware reset
    set_rst(0);
    delay_ms(10);
    set_rst(1);
    delay_ms(10);

    // Software reset (clears all registers to defaults)
    ESP_RETURN_ON_ERROR(write_cmd(0x12), TAG, "sw reset failed");
    wait_busy_low();

    // Select built-in temperature sensor
    ESP_RETURN_ON_ERROR(write_cmd(0x18), TAG, "temp sensor failed");
    ESP_RETURN_ON_ERROR(write_data(0x80), TAG, "temp sensor data failed");

    // Load actual temperature value into register
    ESP_RETURN_ON_ERROR(write_cmd(0x22), TAG, "temp load failed");
    ESP_RETURN_ON_ERROR(write_data(0xB1), TAG, "temp load data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x20), TAG, "temp load activation failed");
    wait_busy_low();

    // Override temperature register to 100 C -> selects faster OTP LUT
    ESP_RETURN_ON_ERROR(write_cmd(0x1A), TAG, "temp override failed");
    ESP_RETURN_ON_ERROR(write_data(0x64), TAG, "temp override lo failed"); // 100 C
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "temp override hi failed");

    // Reload LUT using the overridden temperature
    ESP_RETURN_ON_ERROR(write_cmd(0x22), TAG, "lut reload failed");
    ESP_RETURN_ON_ERROR(write_data(0x91), TAG, "lut reload data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x20), TAG, "lut reload activation failed");
    wait_busy_low();

    /* Re-establish display geometry (SW reset cleared these).
     * Must match driver_hw_init_full() for our frame buffer layout. */

    // Driver output control: gate lines = HEIGHT_PX - 1
    ESP_RETURN_ON_ERROR(write_cmd(0x01), TAG, "driver output failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) % 256)), TAG, "gate lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) / 256)), TAG, "gate hi failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "gate scan failed");

    // Data entry mode: X increment, Y decrement
    ESP_RETURN_ON_ERROR(write_cmd(0x11), TAG, "entry mode failed");
    ESP_RETURN_ON_ERROR(write_data(0x01), TAG, "entry mode data failed");

    // RAM-X window: 0 to 15 (0..127 pixels)
    ESP_RETURN_ON_ERROR(write_cmd(0x44), TAG, "x window failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "x start failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)(WIDTH_PX / 8 - 1)), TAG, "x end failed");

    // RAM-Y window: 249 to 0
    ESP_RETURN_ON_ERROR(write_cmd(0x45), TAG, "y window failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) % 256)), TAG, "y start lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) / 256)), TAG, "y start hi failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "y end lo failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "y end hi failed");

    // RAM-X counter to 0
    ESP_RETURN_ON_ERROR(write_cmd(0x4E), TAG, "x counter failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "x counter data failed");

    // RAM-Y counter to HEIGHT_PX - 1
    ESP_RETURN_ON_ERROR(write_cmd(0x4F), TAG, "y counter failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) % 256)), TAG, "y counter lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) / 256)), TAG, "y counter hi failed");

    return ESP_OK;
}

/*
 * Write full frame to both RAM planes for fast refresh. Writes 0x24 (new) and
 * 0x26 (old/basemap) with identical data so that subsequent partial refreshes
 * (which diff 0x24 vs 0x26) remain coherent.
 */
static esp_err_t driver_fast_write(const uint8_t *data)
{
    // Write image to new RAM plane (0x24)
    ESP_RETURN_ON_ERROR(write_cmd(0x24), TAG, "new plane failed");
    ESP_RETURN_ON_ERROR(write_data_bytes(data, FRAME_BYTES), TAG, "new plane data failed");

    // Reset RAM cursor for second plane write
    ESP_RETURN_ON_ERROR(write_cmd(0x4E), TAG, "x counter failed");
    ESP_RETURN_ON_ERROR(write_data(0x00), TAG, "x counter data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x4F), TAG, "y counter failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) % 256)), TAG, "y counter lo failed");
    ESP_RETURN_ON_ERROR(write_data((uint8_t)((HEIGHT_PX - 1) / 256)), TAG, "y counter hi failed");

    // Write same image to old RAM plane (0x26) -- basemap coherence
    ESP_RETURN_ON_ERROR(write_cmd(0x26), TAG, "old plane failed");
    return write_data_bytes(data, FRAME_BYTES);
}

/* Trigger fast display update (0xC7 waveform -- non-differential, no flash). */
static esp_err_t driver_fast_commit(void)
{
    ESP_RETURN_ON_ERROR(write_cmd(0x22), TAG, "update seq failed");
    ESP_RETURN_ON_ERROR(write_data(0xC7), TAG, "update seq data failed");
    ESP_RETURN_ON_ERROR(write_cmd(0x20), TAG, "activation failed");
    wait_busy_low();
    return ESP_OK;
}

/* Put SSD1680 into deep sleep mode 1. */
static esp_err_t driver_deep_sleep(void)
{
    ESP_RETURN_ON_ERROR(write_cmd(0x10), TAG, "deep sleep failed");
    ESP_RETURN_ON_ERROR(write_data(0x01), TAG, "deep sleep data failed");
    delay_ms(100);
    return ESP_OK;
}


/* ======================================================================
 * epaper.h API on top of the ported driver
 * ====================================================================== */

esp_err_t epaper_init(void)
{
    ESP_RETURN_ON_ERROR(epaper_pins_init(), TAG, "pin init failed");

    // Bring the panel out of deep sleep; each refresh configures it from there.
    set_rst(0);
    delay_ms(10);
    set_rst(1);
    delay_ms(10);
    wait_busy_low();

    return ESP_OK;
}

/*
 * The panel vendor asks for a full refresh after every 5 consecutive fast or
 * partial updates, to stop afterimages accumulating.
 */
static epaper_refresh_t apply_refresh_policy(epaper_refresh_t requested)
{
    if (requested != EPAPER_REFRESH_FULL && s_since_full >= EPAPER_MAX_FAST_REFRESHES) {
        ESP_LOGD(TAG, "promoting to a full refresh after %u fast/partial updates", s_since_full);
        return EPAPER_REFRESH_FULL;
    }
    return requested;
}

static void note_refresh(epaper_refresh_t mode)
{
    if (mode == EPAPER_REFRESH_FULL) {
        s_since_full = 0;
    } else if (s_since_full < UINT8_MAX) {
        s_since_full++;
    }
}

static esp_err_t do_display(const uint8_t *fb, epaper_refresh_t mode)
{
    if (mode == EPAPER_REFRESH_FAST) {
        ESP_RETURN_ON_ERROR(driver_hw_init_fast(), TAG, "fast init failed");
        ESP_RETURN_ON_ERROR(driver_fast_write(fb), TAG, "fast write failed");
        return driver_fast_commit();
    }

    ESP_RETURN_ON_ERROR(driver_hw_init_full(), TAG, "full init failed");
    return driver_set_basemap(fb);
}

esp_err_t epaper_display_mode(const uint8_t *fb, epaper_refresh_t mode)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(fb, ESP_ERR_INVALID_ARG, TAG, "NULL framebuffer");

    mode = apply_refresh_policy(mode);

    // Delegate before taking the bus -- acquisition must not nest.
    if (mode == EPAPER_REFRESH_PARTIAL) {
        return epaper_display_region(fb, 0, EPD_HEIGHT);
    }

    ESP_RETURN_ON_ERROR(spi_acquire_bus(s_spi), TAG, "bus acquire failed");
    const esp_err_t err = do_display(fb, mode);
    spi_release_bus(s_spi);

    ESP_RETURN_ON_ERROR(err, TAG, "refresh failed");

    note_refresh(mode);
    return ESP_OK;
}

esp_err_t epaper_display(const uint8_t *fb)
{
    return epaper_display_mode(fb, EPAPER_REFRESH_FULL);
}

static esp_err_t do_display_region(const uint8_t *rows, uint16_t y, uint16_t height)
{
    ESP_RETURN_ON_ERROR(driver_part_begin(), TAG, "partial init failed");
    ESP_RETURN_ON_ERROR(driver_part_write_region(0, y, rows, height, WIDTH_PX),
                        TAG, "region write failed");
    return driver_part_commit();
}

esp_err_t epaper_display_region(const uint8_t *rows, uint16_t y, uint16_t height)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(rows, ESP_ERR_INVALID_ARG, TAG, "NULL region");
    ESP_RETURN_ON_FALSE(height && (uint32_t)y + height <= EPD_HEIGHT, ESP_ERR_INVALID_ARG,
                        TAG, "band %u..%u outside the panel", y, y + height);

    ESP_RETURN_ON_ERROR(spi_acquire_bus(s_spi), TAG, "bus acquire failed");
    const esp_err_t err = do_display_region(rows, y, height);
    spi_release_bus(s_spi);

    ESP_RETURN_ON_ERROR(err, TAG, "partial refresh failed");

    note_refresh(EPAPER_REFRESH_PARTIAL);
    return ESP_OK;
}

static esp_err_t do_clear(void)
{
    ESP_RETURN_ON_ERROR(driver_hw_init_full(), TAG, "full init failed");
    return driver_set_basemap_fill(0xFF); // 1 = white
}

esp_err_t epaper_clear(void)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    ESP_RETURN_ON_ERROR(spi_acquire_bus(s_spi), TAG, "bus acquire failed");
    const esp_err_t err = do_clear();
    spi_release_bus(s_spi);

    ESP_RETURN_ON_ERROR(err, TAG, "clear failed");

    note_refresh(EPAPER_REFRESH_FULL);
    return ESP_OK;
}

esp_err_t epaper_sleep(void)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    ESP_RETURN_ON_ERROR(spi_acquire_bus(s_spi), TAG, "bus acquire failed");
    const esp_err_t err = driver_deep_sleep();
    spi_release_bus(s_spi);

    return err;
}
