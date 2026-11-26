# Libreria de motores: 4 motores con puente H L9110S + encoders + lazo de velocidad.
# No depende del resto del firmware: sus opciones estan en Kconfig.projbuild (aca al lado)
# y se prueba sola con el proyecto de ejemplo/ (tiene su propio sdkconfig).
idf_component_register(SRCS "motores.c"
                            "encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer esp_driver_ledc)
