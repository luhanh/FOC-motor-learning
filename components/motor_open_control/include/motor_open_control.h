#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机开环控制系统
 *
 * 完成 GPIO、LEDC PWM、UART 初始化，执行软启动，
 * 并创建电机控制任务和串口命令接收任务。
 *
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t motor_open_control_init(void);

#ifdef __cplusplus
}
#endif
