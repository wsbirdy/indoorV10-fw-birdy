# One translation unit per part. Each exposes only its own header; the bus it
# happens to sit on is an implementation detail, hence PRIV_REQUIRES.
idf_component_register(
    SRCS        "sensirion_crc.c"
                "sht40.c"
                "sgp41.c"
                "spl07.c"
                "cap1203.c"
                "cap1166.c"
                "cap1133.c"
                "pm.c"
                "pms5003.c"
                "sps30.c"
                "co2.c"
                "senseair_modbus.c"
                "senseair_s88.c"
                "senseair_s12.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES peripherals
                  esp_driver_gpio   # touch drivers: ALERT# interrupt, CAP1166 RESET line
                  esp_timer
)
