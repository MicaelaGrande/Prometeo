# Libreria de motores: 4 motores con puente H L9110S + lazo de velocidad con los encoders.
# Sus opciones estan en Kconfig.projbuild (aca al lado). Usa la libreria drivers/encoders.
idf_component_register(SRCS "motores.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer esp_driver_ledc
                       PRIV_REQUIRES encoders)
