idf_component_register(
    SRCS "motor_open_control.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_uart driver esp_timer)
