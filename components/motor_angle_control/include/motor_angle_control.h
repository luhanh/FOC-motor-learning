#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化角度闭环控制系统 (DRV8313 + AS5600)
 *
 * 完成 GPIO、LEDC PWM(20kHz)、I2C(AS5600)、UART 初始化，
 * 并创建角度控制任务。
 *
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t motor_angle_control_init(void);

#ifdef __cplusplus
}
#endif
