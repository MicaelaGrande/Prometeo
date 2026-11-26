# Libreria de encoders: cuenta pulsos por interrupcion y calcula rpm y velocidad de cada rueda.
# No depende del resto del firmware: sus opciones estan en Kconfig.projbuild (aca al lado).
idf_component_register(SRCS "encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer)
