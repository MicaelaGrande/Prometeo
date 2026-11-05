# Para agregar un sensor: crear drivers/<sensor>/<sensor>.c y .h,
# y sumar su .c a SRCS y su carpeta a INCLUDE_DIRS.
idf_component_register(SRCS "hcsr04/hcsr04.c"
                            "motores/motores.c"
                            "mg90s/mg90s.c"
                            "encoders/encoder.c"
                            "l9110s/l9110s.c"
                       INCLUDE_DIRS "hcsr04"
                       "l9110s"
                       "motores"
                       "mg90s"
                       "encoders"
                       REQUIRES driver esp_timer esp_driver_ledc)
