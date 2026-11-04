idf_component_register(SRCS "main.c" "bench.c" "app_ipc.c" "maze.c" "tasks/sensor_task.c" "tasks/motion_control_task.c" "tasks/navigation_task.c" "tasks/telemetry_task.c" "drivers/i2c_bus.c" "drivers/motor.c" "drivers/encoder.c" "drivers/imu.c" "drivers/tof.c" "drivers/oled.c" "drivers/button.c"
                       INCLUDE_DIRS "." "drivers" "tasks"
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_i2c esp_lcd esp_timer freertos)
