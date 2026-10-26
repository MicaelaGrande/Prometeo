# Para agregar un sensor: crear drivers/<sensor>/<sensor>.c y .h,
# y sumar su .c a SRCS y su carpeta a INCLUDE_DIRS.
idf_component_register(SRCS "hcsr04/hcsr04.c"
                            "motores/motores.c"
                       INCLUDE_DIRS "hcsr04"
                       "motores"
                       REQUIRES driver esp_timer esp_driver_ledc)
