#ifndef BLDC_MOTOR_H_
#define BLDC_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "hal/gpio_types.h"

/* ==================== 常量定义 ==================== */

/* 默认母线电压 (V) */
#define BLDC_MOTOR_DEF_POWER_SUPPLY 12.0f

/* 引脚未使用哨兵值（使用 ESP-IDF 标准 NC） */
#define BLDC_MOTOR_PIN_NC GPIO_NUM_NC

/* ==================== 电机驱动配置结构体 ==================== */

typedef struct {
    /* ---- PWM 引脚 ---- */
    int pwm_pin_u; /* U 相 PWM 引脚 */
    int pwm_pin_v; /* V 相 PWM 引脚 */
    int pwm_pin_w; /* W 相 PWM 引脚 */

    /* ---- 全局使能引脚（控制整个驱动芯片的 EN 信号，NC 表示不使用） ---- */
    int enable_pin; /* 驱动芯片全局使能引脚 */

    /* ---- 三相独立使能引脚（梯形波 6 步换相用，NC 表示不使用） ---- */
    int enable_pin_u; /* U 相独立使能引脚（预留） */
    int enable_pin_v; /* V 相独立使能引脚（预留） */
    int enable_pin_w; /* W 相独立使能引脚（预留） */

    /* ---- 使能引脚有效电平 ---- */
    bool enable_active_high; /* true: 高有效, false: 低有效 */

    /* ---- 母线电压参数 ---- */
    float voltage_power_supply; /* 母线电压 (V) */
    float voltage_limit;        /* 电压上限 (V), 0 或不设置则等于母线电压 */

    /* ---- LEDC 配置 ---- */
    ledc_mode_t speed_mode;            /* LEDC 速度模式（高速/低速） */
    ledc_timer_t timer_num;            /* LEDC 定时器编号 */
    ledc_channel_t channel_u;          /* U 相 LEDC 通道 */
    ledc_channel_t channel_v;          /* V 相 LEDC 通道 */
    ledc_channel_t channel_w;          /* W 相 LEDC 通道 */
    uint32_t pwm_freq_hz;              /* PWM 频率（Hz），推荐 20000 */
    ledc_timer_bit_t duty_resolution;  /* 占空比分辨率（推荐 LEDC_TIMER_8_BIT） */
} bldc_motor_config_t;

/* ==================== 对外接口 ==================== */

/**
 * @brief   初始化 BLDC 电机驱动
 * @param   config  电机驱动配置参数指针
 * @return  ESP_OK: 成功, 其他: 失败
 */
esp_err_t bldc_motor_init(const bldc_motor_config_t *config);

/**
 * @brief   反初始化 BLDC 电机驱动，释放硬件资源
 * @return  ESP_OK: 成功, 其他: 失败
 */
esp_err_t bldc_motor_deinit(void);

/**
 * @brief   使能电机驱动器（拉高全局 EN 引脚 → PWM=0）
 * @note    调用前需先完成 bldc_motor_init()
 * @return  ESP_OK: 成功, 其他: 失败
 */
esp_err_t bldc_motor_enable(void);

/**
 * @brief   禁用电机驱动器（PWM=0 → 拉低全局 EN 引脚）
 * @note    电机将完全失能，不响应任何力矩指令
 * @return  ESP_OK: 成功, 其他: 失败
 */
esp_err_t bldc_motor_disable(void);

/**
 * @brief   设置三相 PWM 原始占空比（底层接口）
 * @param   duty_u  U 相占空比（0 ~ (2^duty_resolution - 1)）
 * @param   duty_v  V 相占空比（0 ~ (2^duty_resolution - 1)）
 * @param   duty_w  W 相占空比（0 ~ (2^duty_resolution - 1)）
 * @return  ESP_OK: 成功, ESP_ERR_INVALID_STATE: 驱动未初始化
 */
esp_err_t bldc_motor_set_duty(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w);

/**
 * @brief   设置三相相电压，驱动内部完成电压限幅→占空比转换（对标 setPwm）
 * @param   voltage_u  U 相电压 (V), 范围 [0, voltage_limit]
 * @param   voltage_v  V 相电压 (V), 范围 [0, voltage_limit]
 * @param   voltage_w  W 相电压 (V), 范围 [0, voltage_limit]
 * @return  ESP_OK: 成功, ESP_ERR_INVALID_STATE: 驱动未初始化
 */
esp_err_t bldc_motor_set_phase_voltage(float voltage_u, float voltage_v, float voltage_w);

/**
 * @brief   获取驱动器配置的电压上限
 * @return  电压上限 (V)
 */
float bldc_motor_get_voltage_limit(void);

/**
 * @brief   查询驱动是否已完成初始化
 * @return  true: 已初始化, false: 未初始化
 */
bool bldc_motor_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* BLDC_MOTOR_H_ */
