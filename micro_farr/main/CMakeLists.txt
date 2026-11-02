idf_component_register(
    SRCS "micro_farr.c"
         "app_ipc.c"
         "sensor_task.c"
         "motion_control_task.c"
         "navigation_task.c"
         "supervisor_task.c"
         "drivers/drv_motor.c"
         "drivers/drv_encoder.c"
         "drivers/drv_imu.c"
         "drivers/drv_tof.c"
         "drivers/drv_oled.c"
    INCLUDE_DIRS "."
                 "drivers")
