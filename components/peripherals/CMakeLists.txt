# Thin, board-specific wrappers over the ESP-IDF peripheral drivers. Pin
# assignments and bus parameters live in the public headers.
#
# The IDF driver headers appear in our own public headers, so they are REQUIRES
# rather than PRIV_REQUIRES: anything including i2c.h also needs i2c_master.h.
idf_component_register(
    SRCS        "i2c.c"
                "uart.c"
                "spi.c"
                "pwm.c"
    INCLUDE_DIRS "include"
    REQUIRES    esp_driver_i2c
                esp_driver_uart
                esp_driver_spi
                esp_driver_ledc
)
