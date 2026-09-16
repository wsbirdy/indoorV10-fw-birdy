#include <string.h>
#include "ws2812.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_rom_sys.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "ws2812";

/*
 * 100 ns per tick, which divides the WS2812B bit timings evenly:
 *   bit 0: 0.4 us high, 0.8 us low
 *   bit 1: 0.8 us high, 0.4 us low
 * Both are inside the +/-150 ns the part allows.
 */
#define RMT_RESOLUTION_HZ   (10 * 1000 * 1000)
#define T0H_TICKS           4
#define T0L_TICKS           8
#define T1H_TICKS           8
#define T1L_TICKS           4

/* The C5 gives each RMT channel 48 symbol words; one block is plenty since the
 * driver refills from the interrupt as it drains. */
#define RMT_MEM_SYMBOLS     48

/* Original WS2812B latches after 50 us, the -V5 revision wants 280 us. */
#define RESET_US            300

#define BYTES_PER_LED       3
#define STRIP_BYTES         (WS2812_LED_COUNT * BYTES_PER_LED)

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_encoder;

/* Wire order is green, red, blue -- staged at full scale, brightness is
 * applied when the frame is shifted out. */
static uint8_t s_pixels[STRIP_BYTES];
static uint8_t s_frame[STRIP_BYTES];
static uint8_t s_brightness = WS2812_DEFAULT_BRIGHTNESS;

/*
 * The strip has more than one author. The reporting task paints the AQI colour
 * every few seconds, and the touch task changes the brightness under a finger
 * on the slider -- two tasks, one staging buffer and one RMT channel. Without
 * this, a refresh could rewrite s_frame while the encoder was still shifting
 * the previous one out of it.
 *
 * Held across the transmit and its wait, so a frame always goes out whole.
 * NULL before ws2812_init(), which is the one moment nothing else can be
 * calling in.
 */
static SemaphoreHandle_t s_lock;

static void strip_lock(void)
{
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

static void strip_unlock(void)
{
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

esp_err_t ws2812_init(void)
{
    if (s_chan) {
        return ESP_OK; // already up
    }

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "lock alloc failed");
    }

    const rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = RMT_MEM_SYMBOLS,
        .trans_queue_depth = 2,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&chan_cfg, &s_chan), TAG, "channel alloc failed");

    const rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = { .level0 = 1, .duration0 = T0H_TICKS, .level1 = 0, .duration1 = T0L_TICKS },
        .bit1 = { .level0 = 1, .duration0 = T1H_TICKS, .level1 = 0, .duration1 = T1L_TICKS },
        .flags.msb_first = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&enc_cfg, &s_encoder), TAG, "encoder alloc failed");

    ESP_RETURN_ON_ERROR(rmt_enable(s_chan), TAG, "enable failed");

    ESP_RETURN_ON_ERROR(ws2812_clear(), TAG, "initial blank failed");

    ESP_LOGI(TAG, "ready (%d LEDs on gpio %d, brightness %u/255)",
             WS2812_LED_COUNT, WS2812_GPIO, s_brightness);
    return ESP_OK;
}

esp_err_t ws2812_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    ESP_RETURN_ON_FALSE(index < WS2812_LED_COUNT, ESP_ERR_INVALID_ARG, TAG,
                        "pixel %u past the end of the strip", index);

    strip_lock();
    uint8_t *px = &s_pixels[index * BYTES_PER_LED];
    px[0] = g;
    px[1] = r;
    px[2] = b;
    strip_unlock();
    return ESP_OK;
}

void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    strip_lock();
    for (uint16_t i = 0; i < WS2812_LED_COUNT; i++) {
        uint8_t *px = &s_pixels[i * BYTES_PER_LED];
        px[0] = g;
        px[1] = r;
        px[2] = b;
    }
    strip_unlock();
}

void ws2812_set_brightness(uint8_t brightness)
{
    strip_lock();
    s_brightness = brightness;
    strip_unlock();
}

/** Caller holds the lock. */
static esp_err_t refresh_locked(void)
{
    for (size_t i = 0; i < STRIP_BYTES; i++) {
        s_frame[i] = (uint8_t)((s_pixels[i] * s_brightness) / 255);
    }

    // eot_level 0 parks the line low, which is what starts the latch interval.
    const rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };
    ESP_RETURN_ON_ERROR(rmt_transmit(s_chan, s_encoder, s_frame, sizeof(s_frame), &tx_cfg),
                        TAG, "transmit failed");
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(100)),
                        TAG, "transmit did not finish");

    /* Hold the line low long enough for the strip to latch. Cheaper than a
     * composite encoder carrying a reset symbol, and 300 us is too short to
     * hand back to the scheduler. */
    esp_rom_delay_us(RESET_US);
    return ESP_OK;
}

esp_err_t ws2812_refresh(void)
{
    ESP_RETURN_ON_FALSE(s_chan, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    strip_lock();
    const esp_err_t err = refresh_locked();
    strip_unlock();
    return err;
}

esp_err_t ws2812_clear(void)
{
    ESP_RETURN_ON_FALSE(s_chan, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    strip_lock();
    memset(s_pixels, 0, sizeof(s_pixels));
    const esp_err_t err = refresh_locked();
    strip_unlock();
    return err;
}
