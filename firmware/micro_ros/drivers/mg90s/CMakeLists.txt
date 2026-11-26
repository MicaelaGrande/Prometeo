# Libreria mg90s. No depende del resto del firmware: sus opciones estan en Kconfig.projbuild
# (aca al lado) y se prueba sola con el proyecto de ejemplo/ (tiene su propio sdkconfig).
idf_component_register(SRCS "mg90s.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer esp_driver_ledc)
