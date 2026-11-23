# Logica del robot: sensores (filtrado), freno de emergencia, centrado en el pasillo,
# camara/brazo y velocidad de las ruedas.
# Usa los drivers de drivers/ y no sabe nada de micro-ROS: main/ la une con ROS.
idf_component_register(SRCS "sensores.c"
                            "seguridad.c"
                            "centrado.c"
                            "brazo.c"
                            "ruedas.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES drivers esp_timer)
