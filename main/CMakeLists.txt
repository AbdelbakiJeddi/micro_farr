idf_component_register(SRCS "main.c" "drivers/i2c_bus.c" "drivers/motor.c" "drivers/encoder.c" "drivers/imu.c"
                       INCLUDE_DIRS "." "drivers"
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_i2c freertos)
