# PROMETEO_BUILD_MOTOR_TEST (menuconfig -> Prometeo - Test de motores):
# compila main_test_motores.c en vez del firmware normal
if(CONFIG_PROMETEO_BUILD_MOTOR_TEST)
    set(main_srcs "main_test_motores.c")
else()
    set(main_srcs "main.c" "sensores.c" "seguridad.c")
endif()

idf_component_register(SRCS ${main_srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES drivers esp_wifi micro_ros_espidf_component)
