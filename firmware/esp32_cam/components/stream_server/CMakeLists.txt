
idf_component_register(
    SRCS "stream_server.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server camera_manager
)
