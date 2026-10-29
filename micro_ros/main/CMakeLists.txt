idf_component_register(SRCS "main.c" "sensores.c" "seguridad.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES drivers esp_wifi micro_ros_espidf_component)
