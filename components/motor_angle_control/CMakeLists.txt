idf_component_register(
    SRCS "motor_angle_control.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_uart esp_driver_i2c driver esp_timer)
