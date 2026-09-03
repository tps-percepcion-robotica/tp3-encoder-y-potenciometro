idf_component_register(SRCS "main.c" "pote.c" "as5600.c" "consola.c" "uros.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_adc driver esp_driver_uart vfs micro_ros_espidf_component)
