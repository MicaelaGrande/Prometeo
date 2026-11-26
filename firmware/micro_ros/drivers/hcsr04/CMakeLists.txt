# Libreria hcsr04. No depende del resto del firmware: sus opciones estan en Kconfig.projbuild
# (aca al lado) y se prueba sola con el proyecto de ejemplo/ (tiene su propio sdkconfig).
idf_component_register(SRCS "hcsr04.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_rmt esp_timer)
