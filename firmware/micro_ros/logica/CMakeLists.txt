# Logica del robot: sensores (filtrado), freno de emergencia y centrado en el pasillo.
# Usa los drivers de drivers/ y no sabe nada de micro-ROS: main/ la une con ROS.
idf_component_register(SRCS "sensores.c"
                            "seguridad.c"
                            "centrado.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES drivers esp_timer)
