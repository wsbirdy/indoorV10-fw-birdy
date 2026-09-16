# Telemetry: one task owning the cadence, a JSON serializer, and the HTTP call.
# The split is what lets the payload contract be read without reading any
# networking code -- cloud_payload.c is the only file that knows the property
# names, and cloud_client.c the only one that knows esp_http_client exists.
#
# server_ca.pem is embedded unconditionally so the build does not depend on a
# Kconfig-selected source file; cloud_client.c only references the symbol when
# CONFIG_CLOUD_TLS_PINNED_CA is set.

# cJSON moved out of the ESP-IDF tree in v6.0: it was the core "json" component
# through v5.x, and is the managed "espressif__cjson" from v6.0 on, pulled in by
# idf_component.yml next to this file. The header and the API are the same
# either way, so cloud_payload.c does not care -- only the dependency name does.
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    set(cloud_json_component espressif__cjson)
else()
    set(cloud_json_component json)
endif()

idf_component_register(
    SRCS        "cloud.c"
                "cloud_client.c"
                "cloud_payload.c"
    INCLUDE_DIRS "include"
    REQUIRES    air_monitor
    PRIV_REQUIRES wifi
                  esp_http_client
                  esp-tls
                  esp_timer
                  ${cloud_json_component}
    EMBED_TXTFILES "server_ca.pem"
)
