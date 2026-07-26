#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==================== 低通滤波器结构体 ==================== */

typedef struct {
    float Tf;               /* 滤波时间常数 (s) */
    float Ts;               /* 固定采样时间 (s)，FOC_NOT_SET=自适应 */
    int64_t timestamp_prev; /* 上次调用时间戳 (us)（内部状态） */
    float y_prev;           /* 上次输出值（内部状态） */
} lpf_t;

/* ==================== API ==================== */

/**
 * @brief   初始化一阶低通滤波器
 * @param   lpf 滤波器指针
 * @param   Tf  滤波时间常数 (s)
 * @param   Ts  固定采样时间 (s)，FOC_NOT_SET=自适应
 */
void lpf_init(lpf_t *lpf, float Tf, float Ts);

/**
 * @brief   执行一次低通滤波
 * @param   lpf 滤波器指针
 * @param   x   输入值
 * @return  滤波后输出值
 */
float lpf_update(lpf_t *lpf, float x);

#ifdef __cplusplus
}
#endif

#endif /* LOWPASS_FILTER_H_ */
