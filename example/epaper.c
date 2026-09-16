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

/* SSD1680 commands */
#define CMD_DRIVER_OUTPUT_CONTROL   0x01
#define CMD_DEEP_SLEEP              0x10
#define CMD_DATA_ENTRY_MODE         0x11
#define CMD_SW_RESET                0x12
#define CMD_TEMP_SENSOR_CONTROL     0x18
#define CMD_TEMP_WRITE              0x1A
#define CMD_MASTER_ACTIVATION       0x20
#define CMD_DISPLAY_UPDATE_CTRL_1   0x21
#define CMD_DISPLAY_UPDATE_CTRL_2   0x22
#define CMD_WRITE_RAM_NEW           0x24    // the frame being drawn
#define CMD_WRITE_RAM_OLD           0x26    // basemap the differential waveforms diff against
#define CMD_BORDER_WAVEFORM         0x3C
#define CMD_SET_RAM_X_RANGE         0x44
#define CMD_SET_RAM_Y_RANGE         0x45
#define CMD_SET_RAM_X_COUNTER       0x4E
#define CMD_SET_RAM_Y_COUNTER       0x4F

/*
 * Display update sequences, written to CMD_DISPLAY_UPDATE_CTRL_2. Each value
 * names a fixed chain of stages in the controller (SSD1680 table for 0x22);
 * the ones that matter here are whether the temperature is re-sampled and
 * whether the panel is driven with waveform Mode 1 or Mode 2.
 */
#define SEQ_FULL        0xF7    // clock, analog, load temp, display Mode 1, power down
#define SEQ_FAST        0xC7    // clock, analog, display Mode 1, power down -- no temp reload
#define SEQ_PARTIAL     0xFF    // clock, analog, load temp, display Mode 2, power down
#define SEQ_LOAD_TEMP   0xB1    // clock, load temp, load LUT Mode 1, stop clock
#define SEQ_LOAD_LUT    0x91    // clock, load LUT Mode 1, stop clock -- keeps the temp register

/*
 * Border waveform (0x3C). A[7:6] picks the source: 00 follows the LUT, 10 ties
 * the border to VCOM. Driving a differential update through the GS transition
 * flashes the border, so partial mode parks it on VCOM instead.
 */
#define BORDER_FULL         0x05    // GS transition, follow LUT, LUT1
#define BORDER_PARTIAL      0x80    // VCOM

/* Data entry (0x11) A[1:0]: 01 = Y decrement, X increment. The panel scans
 * gates from the bottom, so RAM Y 249 is the top row of the display. */
#define ENTRY_MODE          0x01

#define TEMP_SENSOR_INTERNAL    0x80
/*
 * Fast refresh loads the OTP waveform for 100 C, which is far shorter than the
 * one for room temperature. This is a LUT selector written to the controller's
 * temperature register, not a thermal setting -- the panel itself is only rated
 * 0..50 C and is unaffected. Good Display's own fast-refresh sample does this.
 */
#define TEMP_OVERRIDE_C         0x64    // 100 C

#define GATE_LINES          (EPD_HEIGHT - 1)
#define RAM_Y_TOP           (EPD_HEIGHT - 1)    // controller Y of the topmost visible row

#define BUSY_TIMEOUT_MS     5000
#define TX_CHUNK_BYTES      SPI_MAX_TRANSFER_SZ

static bool s_pins_ready;       // GPIO + SPI bound; guards against a second bind
static spi_device_handle_t s_dev;
static uint8_t *s_tx;   // DMA-capable bounce buffer; callers may hand us any memory
static uint8_t s_since_full;    // fast/partial refreshes since the last full one

static esp_err_t epaper_wait_busy(void)
{
    // SSD1680 drives BUSY high while it is working.
    for (int elapsed = 0; elapsed < BUSY_TIMEOUT_MS; elapsed += 10) {
        if (gpio_get_level(EPD_BUSY_GPIO) == 0) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGE(TAG, "busy timeout");
    return ESP_ERR_TIMEOUT;
}

/*
 * CS is ours, not the SPI peripheral's. Hardware CS deasserts between
 * transactions, which would break a frame into 4 KB chunks the panel reads as
 * separate bursts; holding it low across the whole write keeps the controller's
 * RAM pointer advancing. D/C has to be stable for the entire byte, so it is set
 * before CS goes low.
 */
static void epaper_select(int dc_level)
{
    gpio_set_level(EPD_DC_GPIO, dc_level);
    gpio_set_level(EPD_CS_GPIO, 0);
}

static void epaper_deselect(void)
{
    gpio_set_level(EPD_CS_GPIO, 1);
}

static esp_err_t epaper_cmd(uint8_t cmd)
{
    s_tx[0] = cmd;

    epaper_select(0);
    const esp_err_t err = spi_transfer(s_dev, s_tx, NULL, 1);
    epaper_deselect();

    return err;
}

/* Copies through the DMA-capable bounce buffer so a caller framebuffer in
 * non-DMA memory (PSRAM, stack) still works, and chunks to stay under the bus
 * max_transfer_sz. CS stays low for every chunk. */
static esp_err_t epaper_data(const uint8_t *data, size_t len)
{
    esp_err_t err = ESP_OK;

    epaper_select(1);
    for (size_t offset = 0; offset < len && err == ESP_OK; offset += TX_CHUNK_BYTES) {
        const size_t n = (len - offset < TX_CHUNK_BYTES) ? (len - offset) : TX_CHUNK_BYTES;
        memcpy(s_tx, &data[offset], n);
        err = spi_transfer(s_dev, s_tx, NULL, n);
    }
    epaper_deselect();

    ESP_RETURN_ON_ERROR(err, TAG, "data write failed");
    return ESP_OK;
}

/* Same, for a constant byte -- avoids keeping a whole blank frame in .bss. */
static esp_err_t epaper_data_fill(uint8_t value, size_t len)
{
    esp_err_t err = ESP_OK;

    memset(s_tx, value, (len < TX_CHUNK_BYTES) ? len : TX_CHUNK_BYTES);

    epaper_select(1);
    for (size_t offset = 0; offset < len && err == ESP_OK; offset += TX_CHUNK_BYTES) {
        const size_t n = (len - offset < TX_CHUNK_BYTES) ? (len - offset) : TX_CHUNK_BYTES;
        err = spi_transfer(s_dev, s_tx, NULL, n);
    }
    epaper_deselect();

    ESP_RETURN_ON_ERROR(err, TAG, "fill write failed");
    return ESP_OK;
}

static esp_err_t epaper_cmd_with_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_ERROR(epaper_cmd(cmd), TAG, "cmd 0x%02X failed", cmd);
    return epaper_data(data, len);
}

static esp_err_t epaper_cmd_with_byte(uint8_t cmd, uint8_t value)
{
    return epaper_cmd_with_data(cmd, &value, 1);
}

/*
 * RES# is active low. A hardware reset is also the only way out of deep sleep,
 * where BUSY is held high, so every entry point starts with one and then waits
 * for BUSY -- the panel ignores commands while BUSY is asserted.
 */
static esp_err_t epaper_hw_reset(void)
{
    gpio_set_level(EPD_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(EPD_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    return epaper_wait_busy();
}

/* Hardware reset, then the software reset the power-on flow calls for. */
static esp_err_t epaper_reset_sequence(void)
{
    ESP_RETURN_ON_ERROR(epaper_hw_reset(), TAG, "not ready after reset");

    ESP_RETURN_ON_ERROR(epaper_cmd(CMD_SW_RESET), TAG, "sw reset failed");
    vTaskDelay(pdMS_TO_TICKS(10));  // datasheet power-on flow: wait 10 ms after 0x12
    return epaper_wait_busy();
}

/**
 * Set the RAM window and cursor for a full-width horizontal band.
 *
 * @p y is a logical row (0 = top of the panel); the controller scans Y
 * downward from RAM_Y_TOP, so the mapping is inverted.
 */
static esp_err_t epaper_set_window(uint16_t y, uint16_t height)
{
    const uint16_t y_start = RAM_Y_TOP - y;
    const uint16_t y_end = RAM_Y_TOP - (y + height - 1);

    const uint8_t x_range[] = { 0x00, EPD_ROW_BYTES - 1 };
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_SET_RAM_X_RANGE, x_range, sizeof(x_range)),
                        TAG, "x range failed");

    const uint8_t y_range[] = {
        y_start & 0xFF, (y_start >> 8) & 0xFF,
        y_end & 0xFF, (y_end >> 8) & 0xFF,
    };
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_SET_RAM_Y_RANGE, y_range, sizeof(y_range)),
                        TAG, "y range failed");

    return ESP_OK;
}

static esp_err_t epaper_set_cursor(uint16_t y)
{
    const uint16_t ram_y = RAM_Y_TOP - y;

    ESP_RETURN_ON_ERROR(epaper_cmd_with_byte(CMD_SET_RAM_X_COUNTER, 0x00),
                        TAG, "x counter failed");

    const uint8_t y_counter[] = { ram_y & 0xFF, (ram_y >> 8) & 0xFF };
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_SET_RAM_Y_COUNTER, y_counter,
                                             sizeof(y_counter)),
                        TAG, "y counter failed");

    return ESP_OK;
}

/* Gate count, scan direction, data entry mode and the full-frame window. A
 * software reset clears all of these, so each hw init below has to redo them. */
static esp_err_t epaper_set_geometry(void)
{
    const uint8_t driver_output[] = { GATE_LINES & 0xFF, (GATE_LINES >> 8) & 0xFF, 0x00 };
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_DRIVER_OUTPUT_CONTROL, driver_output,
                                             sizeof(driver_output)),
                        TAG, "driver output failed");

    ESP_RETURN_ON_ERROR(epaper_cmd_with_byte(CMD_DATA_ENTRY_MODE, ENTRY_MODE),
                        TAG, "entry mode failed");

    ESP_RETURN_ON_ERROR(epaper_set_window(0, EPD_HEIGHT), TAG, "window failed");
    return epaper_set_cursor(0);
}

/*
 * Spelled out rather than calling epaper_set_geometry(), because the datasheet's
 * power-on flow interleaves the border and LUT load between the RAM window and
 * the RAM counters. Keep the order.
 */
static esp_err_t epaper_hw_init_full(void)
{
    ESP_RETURN_ON_ERROR(epaper_reset_sequence(), TAG, "reset failed");

    const uint8_t driver_output[] = { GATE_LINES & 0xFF, (GATE_LINES >> 8) & 0xFF, 0x00 };
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_DRIVER_OUTPUT_CONTROL, driver_output,
                                             sizeof(driver_output)),
                        TAG, "driver output failed");

    ESP_RETURN_ON_ERROR(epaper_cmd_with_byte(CMD_DATA_ENTRY_MODE, ENTRY_MODE),
                        TAG, "entry mode failed");
    ESP_RETURN_ON_ERROR(epaper_set_window(0, EPD_HEIGHT), TAG, "window failed");

    ESP_RETURN_ON_ERROR(epaper_cmd_with_byte(CMD_BORDER_WAVEFORM, BORDER_FULL),
                        TAG, "border failed");

    /* RAM content normal; B[7]=1 narrows the source window to S8..S167, which
     * is where this 122-column glass actually sits on the 176-source driver. */
    const uint8_t update_ctrl[] = { 0x00, 0x80 };
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_DISPLAY_UPDATE_CTRL_1, update_ctrl,
                                             sizeof(update_ctrl)),
                        TAG, "update ctrl failed");

    // Let the panel pick its waveform from its own die temperature.
    ESP_RETURN_ON_ERROR(epaper_cmd_with_byte(CMD_TEMP_SENSOR_CONTROL, TEMP_SENSOR_INTERNAL),
                        TAG, "temp sensor failed");

    ESP_RETURN_ON_ERROR(epaper_set_cursor(0), TAG, "cursor failed");
    return epaper_wait_busy();
}

/* Runs the update sequence and waits for the panel to finish. */
static esp_err_t epaper_commit(uint8_t sequence)
{
    ESP_RETURN_ON_ERROR(epaper_cmd_with_byte(CMD_DISPLAY_UPDATE_CTRL_2, sequence),
                        TAG, "sequence 0x%02X failed", sequence);
    ESP_RETURN_ON_ERROR(epaper_cmd(CMD_MASTER_ACTIVATION), TAG, "activation failed");
    return epaper_wait_busy();
}

/*
 * Fast refresh setup. Lies to the controller about the temperature so it loads
 * the OTP LUT for 100 C, which is a much shorter waveform than the one for room
 * temperature. SEQ_FAST then drives every pixel with it, unconditionally.
 */
static esp_err_t epaper_hw_init_fast(void)
{
    ESP_RETURN_ON_ERROR(epaper_reset_sequence(), TAG, "reset failed");

    ESP_RETURN_ON_ERROR(epaper_cmd_with_byte(CMD_TEMP_SENSOR_CONTROL, TEMP_SENSOR_INTERNAL),
                        TAG, "temp sensor failed");
    ESP_RETURN_ON_ERROR(epaper_commit(SEQ_LOAD_TEMP), TAG, "temp load failed");

    const uint8_t temp_override[] = { TEMP_OVERRIDE_C, 0x00 };
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_TEMP_WRITE, temp_override,
                                             sizeof(temp_override)),
                        TAG, "temp override failed");
    ESP_RETURN_ON_ERROR(epaper_commit(SEQ_LOAD_LUT), TAG, "lut reload failed");

    // The software reset above cleared the geometry; it has to match the full
    // init or the framebuffer lands in the wrong place.
    return epaper_set_geometry();
}

/* Partial refresh setup. The hardware reset restores the register defaults but
 * leaves both RAM planes intact, which is exactly what the differential
 * waveform needs; only the three registers below have to be re-sent. */
static esp_err_t epaper_hw_init_partial(void)
{
    // No software reset here: 0x12 would be harmless for RAM, but the register
    // defaults it restores are exactly the three re-sent below anyway.
    ESP_RETURN_ON_ERROR(epaper_hw_reset(), TAG, "not ready after reset");

    const uint8_t driver_output[] = { GATE_LINES & 0xFF, (GATE_LINES >> 8) & 0xFF, 0x00 };
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_DRIVER_OUTPUT_CONTROL, driver_output,
                                             sizeof(driver_output)),
                        TAG, "driver output failed");

    // Partial mode drives the border differently; BORDER_FULL flashes it.
    ESP_RETURN_ON_ERROR(epaper_cmd_with_byte(CMD_BORDER_WAVEFORM, BORDER_PARTIAL),
                        TAG, "border failed");

    return epaper_cmd_with_byte(CMD_DATA_ENTRY_MODE, ENTRY_MODE);
}

/* Undo a partial epaper_pins_init() so a failed attempt can be retried. */
static void epaper_unbind_pins(void)
{
    if (s_dev) {
        spi_remove_device(s_dev);
        s_dev = NULL;
    }
    if (s_tx) {
        heap_caps_free(s_tx);
        s_tx = NULL;
    }
    gpio_reset_pin(EPD_CS_GPIO);
    gpio_reset_pin(EPD_DC_GPIO);
    gpio_reset_pin(EPD_RST_GPIO);
    gpio_reset_pin(EPD_BUSY_GPIO);
}

esp_err_t epaper_pins_init(void)
{
    if (s_pins_ready) {
        return ESP_OK; // already bound
    }

    esp_err_t ret = ESP_OK;     // the ESP_GOTO_ON_* macros assign to this

    /* SCLK and MOSI belong to the shared bus, and bring DMA with them.
     * spi_init() is a no-op if the board already brought the bus up, so the
     * panel does not care whether it is initialized before or after the rest
     * of the board. */
    ESP_RETURN_ON_ERROR(spi_init(), TAG, "spi bus init failed");

    /* CS is active low and driven by this driver; DC selects command (low) or
     * data (high); RST is the active-low reset. */
    const gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << EPD_CS_GPIO) | (1ULL << EPD_DC_GPIO) |
                        (1ULL << EPD_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&out_cfg), fail, TAG, "cs/dc/rst config failed");

    /* BUSY is driven by the panel; it idles low and goes high while working.
     * No pull -- the panel drives it the whole time it is powered. */
    const gpio_config_t busy_cfg = {
        .pin_bit_mask = (1ULL << EPD_BUSY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&busy_cfg), fail, TAG, "busy config failed");

    // Idle levels: deselected, reset released, DC parked on data.
    gpio_set_level(EPD_CS_GPIO, 1);
    gpio_set_level(EPD_RST_GPIO, 1);
    gpio_set_level(EPD_DC_GPIO, 1);

    /* Every SPI write is copied through this buffer, so it has to come from a
     * DMA-capable heap -- the bus runs with SPI_DMA_CH_AUTO. */
    s_tx = heap_caps_malloc(TX_CHUNK_BYTES, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(s_tx, ESP_ERR_NO_MEM, fail, TAG, "no DMA memory for tx buffer");

    // Mode 0. CS is passed as -1: we drive it ourselves, see epaper_select().
    ESP_GOTO_ON_ERROR(spi_add_device(0, -1, &s_dev), fail, TAG, "spi attach failed");

    s_pins_ready = true;
    ESP_LOGI(TAG, "pins bound (sclk=%d mosi=%d cs=%d dc=%d rst=%d busy=%d)",
             SPI_SCLK_GPIO, SPI_MOSI_GPIO, EPD_CS_GPIO,
             EPD_DC_GPIO, EPD_RST_GPIO, EPD_BUSY_GPIO);
    return ESP_OK;

fail:
    epaper_unbind_pins();
    return ret;
}

esp_err_t epaper_init(void)
{
    ESP_RETURN_ON_ERROR(epaper_pins_init(), TAG, "pin init failed");

    // Leave the panel in its power-on state; each refresh configures it for the
    // waveform it needs. Resetting here also proves out the BUSY handshake and
    // is what brings the panel back out of deep sleep.
    ESP_RETURN_ON_ERROR(epaper_hw_reset(), TAG, "not ready after reset");

    ESP_LOGI(TAG, "ready (%dx%d in %d-bit rows)", EPD_WIDTH, EPD_HEIGHT, EPD_RAM_WIDTH);
    return ESP_OK;
}

/*
 * The panel vendor asks for a full refresh after every 5 consecutive fast or
 * partial updates, to stop afterimages accumulating. Enforcing it here rather
 * than in each caller means no caller can forget.
 */
static epaper_refresh_t epaper_apply_refresh_policy(epaper_refresh_t requested)
{
    if (requested == EPAPER_REFRESH_FULL) {
        return EPAPER_REFRESH_FULL;
    }
    if (s_since_full >= EPAPER_MAX_FAST_REFRESHES) {
        ESP_LOGD(TAG, "promoting to a full refresh after %u fast/partial updates",
                 s_since_full);
        return EPAPER_REFRESH_FULL;
    }
    return requested;
}

static void epaper_note_refresh(epaper_refresh_t mode)
{
    if (mode == EPAPER_REFRESH_FULL) {
        s_since_full = 0;
    } else if (s_since_full < UINT8_MAX) {
        s_since_full++;
    }
}

static esp_err_t epaper_do_display(const uint8_t *fb, epaper_refresh_t mode)
{
    ESP_RETURN_ON_ERROR(mode == EPAPER_REFRESH_FAST ? epaper_hw_init_fast()
                                                    : epaper_hw_init_full(),
                        TAG, "hw init failed");

    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_WRITE_RAM_NEW, fb, EPD_FB_SIZE),
                        TAG, "new plane write failed");

    /* The differential waveforms diff the new plane against the old one, so the
     * old plane has to be primed with the same frame or the next partial
     * refresh ghosts whatever the panel powered up with. */
    ESP_RETURN_ON_ERROR(epaper_set_cursor(0), TAG, "cursor rewind failed");
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_WRITE_RAM_OLD, fb, EPD_FB_SIZE),
                        TAG, "old plane write failed");

    return epaper_commit(mode == EPAPER_REFRESH_FAST ? SEQ_FAST : SEQ_FULL);
}

esp_err_t epaper_display_mode(const uint8_t *fb, epaper_refresh_t mode)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(fb, ESP_ERR_INVALID_ARG, TAG, "NULL framebuffer");

    mode = epaper_apply_refresh_policy(mode);

    // Delegate before taking the bus -- acquisition must not nest.
    if (mode == EPAPER_REFRESH_PARTIAL) {
        return epaper_display_region(fb, 0, EPD_HEIGHT);
    }

    ESP_RETURN_ON_ERROR(spi_acquire_bus(s_dev), TAG, "bus acquire failed");
    const esp_err_t err = epaper_do_display(fb, mode);
    spi_release_bus(s_dev);

    ESP_RETURN_ON_ERROR(err, TAG, "refresh failed");

    epaper_note_refresh(mode);
    return ESP_OK;
}

esp_err_t epaper_display(const uint8_t *fb)
{
    return epaper_display_mode(fb, EPAPER_REFRESH_FULL);
}

static esp_err_t epaper_do_display_region(const uint8_t *rows, uint16_t y, uint16_t height)
{
    ESP_RETURN_ON_ERROR(epaper_hw_init_partial(), TAG, "hw init failed");

    ESP_RETURN_ON_ERROR(epaper_set_window(y, height), TAG, "window failed");
    ESP_RETURN_ON_ERROR(epaper_set_cursor(y), TAG, "cursor failed");

    // New plane only -- the waveform diffs it against the basemap already there.
    ESP_RETURN_ON_ERROR(epaper_cmd_with_data(CMD_WRITE_RAM_NEW, rows,
                                             (size_t)height * EPD_ROW_BYTES),
                        TAG, "region write failed");

    return epaper_commit(SEQ_PARTIAL);
}

esp_err_t epaper_display_region(const uint8_t *rows, uint16_t y, uint16_t height)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(rows, ESP_ERR_INVALID_ARG, TAG, "NULL region");
    ESP_RETURN_ON_FALSE(height && (uint32_t)y + height <= EPD_HEIGHT, ESP_ERR_INVALID_ARG,
                        TAG, "band %u..%u outside the panel", y, y + height);

    ESP_RETURN_ON_ERROR(spi_acquire_bus(s_dev), TAG, "bus acquire failed");
    const esp_err_t err = epaper_do_display_region(rows, y, height);
    spi_release_bus(s_dev);

    ESP_RETURN_ON_ERROR(err, TAG, "partial refresh failed");

    epaper_note_refresh(EPAPER_REFRESH_PARTIAL);
    return ESP_OK;
}

static esp_err_t epaper_do_clear(void)
{
    ESP_RETURN_ON_ERROR(epaper_hw_init_full(), TAG, "hw init failed");

    ESP_RETURN_ON_ERROR(epaper_cmd(CMD_WRITE_RAM_NEW), TAG, "new plane failed");
    ESP_RETURN_ON_ERROR(epaper_data_fill(0xFF, EPD_FB_SIZE), TAG, "new plane fill failed");

    ESP_RETURN_ON_ERROR(epaper_set_cursor(0), TAG, "cursor rewind failed");
    ESP_RETURN_ON_ERROR(epaper_cmd(CMD_WRITE_RAM_OLD), TAG, "old plane failed");
    ESP_RETURN_ON_ERROR(epaper_data_fill(0xFF, EPD_FB_SIZE), TAG, "old plane fill failed");

    return epaper_commit(SEQ_FULL);
}

esp_err_t epaper_clear(void)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    ESP_RETURN_ON_ERROR(spi_acquire_bus(s_dev), TAG, "bus acquire failed");
    const esp_err_t err = epaper_do_clear();
    spi_release_bus(s_dev);

    ESP_RETURN_ON_ERROR(err, TAG, "clear failed");

    epaper_note_refresh(EPAPER_REFRESH_FULL);
    return ESP_OK;
}

esp_err_t epaper_sleep(void)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    /* Mode 1 keeps the RAM, so the basemap survives for the next differential
     * refresh. BUSY stays high from here on and only a hardware reset gets the
     * controller back -- epaper_init() does exactly that. */
    const uint8_t mode = 0x01;

    ESP_RETURN_ON_ERROR(spi_acquire_bus(s_dev), TAG, "bus acquire failed");
    const esp_err_t err = epaper_cmd_with_data(CMD_DEEP_SLEEP, &mode, 1);
    spi_release_bus(s_dev);

    ESP_RETURN_ON_ERROR(err, TAG, "deep sleep failed");
    vTaskDelay(pdMS_TO_TICKS(100)); // let the panel settle before the bus goes quiet

    return ESP_OK;
}
