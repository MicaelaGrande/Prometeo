# PROMETEO_BUILD_MOTOR_TEST (menuconfig -> Prometeo - Test de motores):
# compila main_test_motores.c en vez del firmware normal.
if(CONFIG_PROMETEO_BUILD_MOTOR_TEST)
    set(main_srcs "main_test_motores.c")
else()
    set(main_srcs "main.c" "red.c")
endif()

# OJO: ESP-IDF resuelve los REQUIRES antes de leer sdkconfig, asi que aca CONFIG_* todavia no existe.
# Por eso el test se elige con la variable de entorno PROMETEO_MINIMAL=1 (la misma que hace que el
# CMakeLists de arriba compile solo main): el test solo usa los drivers (motores, encoders) y no
# depende de logica/ ni de mg90s, que todavia usan la version vieja del driver de motores.
set(main_reqs encoders motores esp_wifi micro_ros_espidf_component)

idf_component_register(SRCS ${main_srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ${main_reqs})
