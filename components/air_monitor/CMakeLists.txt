# Owns one acquisition task per sensor and the snapshot they publish into.
# air_monitor.h embeds pm_data_t, so sensors is a public dependency.
idf_component_register(
    SRCS        "air_monitor.c"
    INCLUDE_DIRS "include"
    REQUIRES    sensors
)
