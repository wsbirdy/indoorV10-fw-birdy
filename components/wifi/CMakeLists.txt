# Station-mode Wi-Fi with self-healing reconnect. The ESP-IDF Wi-Fi, netif and
# NVS stacks are an implementation detail of wifi_sta.c, hence PRIV_REQUIRES.
idf_component_register(
    SRCS        "wifi_sta.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi
                  esp_netif
                  esp_event
                  esp_timer
                  nvs_flash
)
