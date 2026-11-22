
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES camera_manager wifi_manager stream_server esp_driver_gpio esp_system
)
