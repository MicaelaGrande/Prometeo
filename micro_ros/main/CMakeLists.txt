# PROMETEO_BUILD_MOTOR_TEST (menuconfig -> Prometeo - Test de motores):
# compila main_test_motores.c en vez del firmware normal
# PROMETEO_BUILD_L9110S_TEST: compila main_test_l9110s.c (test del driver L9110S)
if(CONFIG_PROMETEO_BUILD_MOTOR_TEST)
    set(main_srcs "main_test_motores.c")
elseif(CONFIG_PROMETEO_BUILD_L9110S_TEST)
    set(main_srcs "main_test_l9110s.c")
else()
    set(main_srcs "main.c" "sensores.c" "seguridad.c")
endif()

idf_component_register(SRCS ${main_srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES drivers esp_wifi micro_ros_espidf_component)
