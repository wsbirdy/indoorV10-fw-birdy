# Telemetry: one task owning the cadence, a JSON serializer, and the HTTP call.
# The split is what lets the payload contract be read without reading any
# networking code -- cloud_payload.c is the only file that knows the property
# names, and cloud_client.c the only one that knows esp_http_client exists.
#
# server_ca.pem is embedded unconditionally so the build does not depend on a
# Kconfig-selected source file; cloud_client.c only references the symbol when
# CONFIG_CLOUD_TLS_PINNED_CA is set.
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
                  json
    EMBED_TXTFILES "server_ca.pem"
)
