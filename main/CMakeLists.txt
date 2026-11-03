idf_component_register(SRCS "main.c" "drivers/motor.c"
                       INCLUDE_DIRS "." "drivers"
                       REQUIRES esp_driver_gpio esp_driver_ledc freertos)
