#ifndef BLDC_FOC_MODULATION_H_
#define BLDC_FOC_MODULATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"

/* ==================== 梯形波查表哨兵值 ==================== */

#define BLDC_HIGH_IMPEDANCE 0

/* ==================== FOC 调制类型枚举 ==================== */

typedef enum {
    BLDC_MODULATION_SINE_PWM = 0,   /* 正弦 PWM 调制 */
    BLDC_MODULATION_SVPWM,          /* 空间矢量 PWM 调制（Midpoint Clamp） */
    BLDC_MODULATION_TRAPEZOID_120,  /* 梯形波 120° 换相（6 步） */
    BLDC_MODULATION_TRAPEZOID_150,  /* 梯形波 150° 换相（12 步） */
} bldc_modulation_type_t;

/* ==================== FOC 调制控制接口 ==================== */

/**
 * @brief   设置调制方式（正弦 PWM / SVPWM / 梯形波 120° / 梯形波 150°）
 * @param   modulation  调制类型
 * @return  ESP_OK: 成功
 */
esp_err_t bldc_foc_set_modulation(bldc_modulation_type_t modulation);

/**
 * @brief   设置是否使用居中调制
 * @note    居中调制将三相电压偏置到母线电压的一半，适用于全桥驱动器
 *          非居中调制将最低相电压拉至 0，适用于低侧电流采样场景
 * @param   centered  true: 居中, false: 非居中
 * @return  ESP_OK: 成功
 */
esp_err_t bldc_foc_set_modulation_centered(bool centered);

/**
 * @brief   FOC 核心驱动：dq 电压 + 电角度 → 调制 → 三相 PWM 输出
 * @note    对标 SimpleFOC BLDCMotor::setPhaseVoltage()
 *          根据当前调制类型自动选择 SVPWM / 正弦 PWM / 梯形波换相策略
 * @param   Uq        交轴电压（转矩分量, V），范围 [±voltage_limit]
 * @param   Ud        直轴电压（磁链分量, V），通常为 0（Id=0 控制）
 * @param   angle_el  电角度（弧度），范围 [0, 2π)
 * @return  ESP_OK: 成功, ESP_ERR_INVALID_STATE: 驱动未初始化
 */
esp_err_t bldc_foc_set_voltage(float Uq, float Ud, float angle_el);

/**
 * @brief   角度归一化到 [0, 2π) 范围
 * @param   angle  输入角度（弧度）
 * @return  归一化后的角度
 */
float bldc_foc_normalize_angle(float angle);

/**
 * @brief   机械角度 → 电角度换算
 * @param   mechanical_angle  机械角度（弧度）
 * @param   pole_pairs        电机极对数
 * @return  归一化后的电角度 [0, 2π)
 */
float bldc_foc_electrical_angle(float mechanical_angle, int pole_pairs);

#ifdef __cplusplus
}
#endif

#endif /* BLDC_FOC_MODULATION_H_ */
