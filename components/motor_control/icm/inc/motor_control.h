#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"

/* ==================== 控制模式枚举（仅暴露给 main.c 的必要类型） ==================== */

/** 运动控制模式 */
typedef enum {
    MC_CTRL_TORQUE = 0,          /* 力矩控制 */
    MC_CTRL_VELOCITY,            /* 速度闭环控制 */
    MC_CTRL_ANGLE,               /* 角度闭环控制 */
} mc_control_type_t;

/** 力矩控制类型 */
typedef enum {
    MC_TORQUE_VOLTAGE = 0,       /* 电压控制 */
} mc_torque_type_t;

/* ==================== 电机控制上下文（不透明指针，main.c 不感知内部结构） ==================== */

typedef struct motor_control_t motor_control_t;

/* ==================== API ==================== */

/**
 * @brief   初始化电机控制系统（硬件 + 算法 + 传感器一站式初始化）
 * @param   out_mc  输出：控制上下文指针
 * @return  ESP_OK: 成功
 * @note    main.c 唯一需要调用的初始化接口
 *          内部完成：PWM 引脚配置 → LEDC 初始化 → AS5600 传感器绑定 → FOC 上下文创建
 */
esp_err_t motor_control_init(motor_control_t **out_mc);

/**
 * @brief   反初始化电机控制系统
 * @param   mc  控制上下文指针
 * @return  ESP_OK: 成功
 */
esp_err_t motor_control_deinit(motor_control_t *mc);

/**
 * @brief   启动 FOC 校准流程（传感器对齐 + 电零点校准）
 * @param   mc  控制上下文指针
 * @return  ESP_OK: 成功
 */
esp_err_t motor_control_calibrate(motor_control_t *mc);

/**
 * @brief   执行一次 FOC 控制循环
 * @param   mc  控制上下文指针
 * @note    应在定时器中以固定频率调用（推荐 1kHz）
 */
void motor_control_step(motor_control_t *mc);

/**
 * @brief   使能电机输出
 * @param   mc  控制上下文指针
 * @return  ESP_OK: 成功
 */
esp_err_t motor_control_enable(motor_control_t *mc);

/**
 * @brief   禁用电机输出
 * @param   mc  控制上下文指针
 * @return  ESP_OK: 成功
 */
esp_err_t motor_control_disable(motor_control_t *mc);

/**
 * @brief   设置目标值
 * @param   mc      控制上下文指针
 * @param   target  目标值（单位取决于控制模式：V / rad/s / rad）
 */
void motor_control_set_target(motor_control_t *mc, float target);

/**
 * @brief   设置运动控制模式
 * @param   mc    控制上下文指针
 * @param   type  控制模式
 */
void motor_control_set_control_type(motor_control_t *mc, mc_control_type_t type);

/**
 * @brief   设置力矩控制类型
 * @param   mc    控制上下文指针
 * @param   type  力矩控制类型
 */
void motor_control_set_torque_type(motor_control_t *mc, mc_torque_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H_ */
