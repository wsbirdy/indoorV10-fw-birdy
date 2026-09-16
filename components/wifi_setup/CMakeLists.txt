# SoftAP credential portal. Depends on wifi for the station it is reconfiguring;
# the HTTP server and the AP netif are an implementation detail of wifi_setup.c.
idf_component_register(
    SRCS        "wifi_setup.c"
    INCLUDE_DIRS "include"
    REQUIRES    wifi
    PRIV_REQUIRES esp_wifi
                  esp_netif
                  esp_http_server
                  esp_timer
)
