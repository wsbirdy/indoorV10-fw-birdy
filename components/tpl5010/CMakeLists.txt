# External nano-power watchdog. Pulses DONE from its own task.
idf_component_register(
    SRCS        "tpl5010.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_gpio
)
