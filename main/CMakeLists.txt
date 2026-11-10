# Application entry point: startup order, the report task and the screen
# layout. Everything reusable lives in components/.
#
# main implicitly depends on every component, so REQUIRES is not strictly
# needed here -- it is listed to document what the application actually uses.
idf_component_register(
    SRCS        "main.c"
    INCLUDE_DIRS "."
    REQUIRES    peripherals
                air_monitor
                display
                ws2812
                aqi
                tpl5010
                sensors
                wifi
                wifi_setup
                cloud
                esp_app_format   # esp_app_desc.h, for the About page
                esp_timer        # prompt timeouts and the boot grace period
                esp_netif        # SNTP, for the dashboard clock
)
