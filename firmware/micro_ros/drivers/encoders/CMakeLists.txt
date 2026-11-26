# Libreria de encoders: cuenta pulsos con el periferico PCNT (sin interrupciones) y calcula las rpm de cada rueda.
# No depende del resto del firmware: sus opciones estan en Kconfig.projbuild (aca al lado).
idf_component_register(SRCS "encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_gpio esp_driver_pcnt esp_timer)
