#ifndef PID_H_
#define PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ==================== PID 控制器结构体 ==================== */

typedef struct {
    float P;                /* 比例增益 */
    float I;                /* 积分增益 */
    float D;                /* 微分增益 */
    float output_ramp;      /* 输出变化率限制 (单位/s)，FOC_NOT_SET=不限制 */
    float limit;            /* 输出限幅 (±limit)，FOC_NOT_SET=不限幅 */
    float Ts;               /* 固定采样时间 (s)，FOC_NOT_SET=自适应 */
    float error_prev;       /* 上次误差（内部状态） */
    float output_prev;      /* 上次输出（内部状态） */
    float integral_prev;    /* 上次积分值（内部状态） */
    int64_t timestamp_prev; /* 上次调用时间戳 (us)（内部状态） */
} pid_controller_t;

/* ==================== API ==================== */

/**
 * @brief   初始化 PID 控制器
 * @param   pid   PID 控制器指针
 * @param   P     比例增益
 * @param   I     积分增益
 * @param   D     微分增益
 * @param   ramp  输出变化率限制 (FOC_NOT_SET=不限制)
 * @param   limit 输出限幅 (FOC_NOT_SET=不限幅)
 * @param   Ts    固定采样时间 (FOC_NOT_SET=自适应)
 */
void pid_init(pid_controller_t *pid, float P, float I, float D,
              float ramp, float limit, float Ts);

/**
 * @brief   执行一次 PID 计算
 * @param   pid   PID 控制器指针
 * @param   error 当前误差
 * @return  控制输出
 * @note    采用 Tustin 积分 + 抗饱和 + 输出 ramp 限幅
 */
float pid_update(pid_controller_t *pid, float error);

/**
 * @brief   重置 PID 控制器内部状态
 * @param   pid  PID 控制器指针
 */
void pid_reset(pid_controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
