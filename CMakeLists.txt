idf_component_register(
    SRCS "src/ads1220.c"
    INCLUDE_DIRS "src"
    REQUIRES driver esp_timer
)
