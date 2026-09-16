# SSD1680 e-paper panel, the 1-bit drawing primitives that target it, and the
# display service that sits on top of both.
#
# All three are ports of the AirGradient Go display library, which cannot be
# built verbatim here: it is C++ and needs u8g2, rtos.h, measures_types.h,
# provisioning_qr.h and go_text_wrap.h, none of which are vendored. The
# untouched original is kept in example/go_display_driver/ for reference.
#
#   epaper_display_panel.c  <- the file-local driver_* block (SSD1680 command
#                              sequences, reset timings, RAM addressing)
#   epaper_display.c        <- DisplayService (snapshot API, worker task,
#                              refresh tiering), gfx.c standing in for u8g2
#
# epaper.h includes spi.h, so peripherals is public. epaper_display.c maps PM2.5
# to an AQI band for the headline, so aqi comes in too.
idf_component_register(
    SRCS        "epaper_display_panel.c"
                "gfx.c"
                "epaper_display.c"
    INCLUDE_DIRS "include"
    REQUIRES    peripherals
                aqi
    PRIV_REQUIRES esp_driver_gpio
)
