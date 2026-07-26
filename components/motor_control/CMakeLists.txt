# motor_control 组件：BLDC 电机 FOC 控制
#
# 内部模块:
#   moter_driver/     — 电机驱动硬件抽象（LEDC PWM + GPIO）
#   sensors/          — 传感器驱动（AS5600）
#   algorithm/        — FOC 算法层（PID + LPF + 工具 + 控制逻辑）
#   icm/              — 集成控制模块（motor_control + sensor 抽象）

idf_component_register(
    SRCS "moter_driver/src/bldc_motor.c"
         "moter_driver/src/bldc_foc_modulation.c"
         "sensors/src/AS5600.c"
         "algorithm/src/foc.c"
         "algorithm/src/foc_utils.c"
         "algorithm/src/pid.c"
         "algorithm/src/lowpass_filter.c"
         "icm/src/motor_control.c"
         "icm/src/sensor.c"
    INCLUDE_DIRS "moter_driver/inc"
                  "sensors/inc"
                  "algorithm/inc"
                  "icm/inc"
    REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_i2c esp_timer driver)
