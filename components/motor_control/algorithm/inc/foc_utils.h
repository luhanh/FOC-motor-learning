#ifndef FOC_UTILS_H_
#define FOC_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ==================== 数学常量 ==================== */

#define FOC_PI        3.14159265359f
#define FOC_2PI       6.28318530718f
#define FOC_PI_2      1.57079632679f
#define FOC_PI_3      1.04719755120f
#define FOC_3PI_2     4.71238898038f
#define FOC_PI_6      0.52359877559f
#define FOC_SQRT3     1.73205080757f
#define FOC_SQRT3_2   0.86602540378f
#define FOC_2_SQRT3   1.15470053838f
#define FOC_1_SQRT3   0.57735026919f
#define FOC_SQRT2     1.41421356237f
#define FOC_120_D2R   2.09439510239f
#define FOC_RPM2RADS  0.10471975512f

/* ==================== 宏函数 ==================== */

#define FOC_SIGN(a)       ( ((a) < 0) ? -1 : ((a) > 0) )
#define FOC_ROUND(x)      ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))
#define FOC_CONSTRAIN(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define FOC_UNUSED(v)     (void)(v)

/* ==================== 哨兵值 ==================== */

#define FOC_NOT_SET         (-12345.0f)
#define FOC_HIGH_IMPEDANCE  0
#define FOC_ACTIVE          1

/* 最小可检测运动角度 (rad) */
#define FOC_MIN_ANGLE_DETECT_MOVEMENT (FOC_2PI / 101.0f)

/* ==================== 数据结构 ==================== */

/** dq 变量 */
typedef struct {
    float d;
    float q;
} foc_dq_t;

typedef foc_dq_t foc_dq_voltage_t;
typedef foc_dq_t foc_dq_current_t;

/** αβ 变量 */
typedef struct {
    float alpha;
    float beta;
} foc_ab_t;

/** 三相变量 */
typedef struct {
    float a;
    float b;
    float c;
} foc_phase_current_t;

typedef foc_phase_current_t foc_phase_voltage_t;

/* ==================== API ==================== */

/**
 * @brief   快速正弦近似（查表 + 线性插值）
 * @param   a  角度 (rad), 范围 [0, 2π)
 * @return  近似正弦值
 */
float foc_sin(float a);

/**
 * @brief   快速余弦近似
 * @param   a  角度 (rad), 范围 [0, 2π)
 * @return  近似余弦值
 */
float foc_cos(float a);

/**
 * @brief   同时计算正弦和余弦
 * @param   a  角度 (rad)
 * @param   s  输出正弦值指针
 * @param   c  输出余弦值指针
 */
void foc_sincos(float a, float *s, float *c);

/**
 * @brief   快速 atan2 近似
 * @param   y  y 值
 * @param   x  x 值
 * @return  近似 atan2 值 (rad)
 */
float foc_atan2(float y, float x);

/**
 * @brief   角度归一化到 [0, 2π) 范围
 * @param   angle  输入角度 (rad)
 * @return  归一化后的角度
 */
float foc_normalize_angle(float angle);

/**
 * @brief   机械角度 → 电角度换算（不做归一化）
 * @param   shaft_angle  机械角度 (rad)
 * @param   pole_pairs   极对数
 * @return  电角度 (rad)
 */
float foc_electrical_angle_calc(float shaft_angle, int pole_pairs);

/**
 * @brief   符号函数
 * @param   x  输入值
 * @return  1.0f (x>0), -1.0f (x<0), 0.0f (x=0)
 */
float foc_sign(float x);

/**
 * @brief   数值限幅
 * @param   amt   输入值
 * @param   low   下限
 * @param   high  上限
 * @return  限幅后的值
 */
float foc_constrain(float amt, float low, float high);

/**
 * @brief   判断浮点数是否已设置（非 FOC_NOT_SET 哨兵值）
 * @param   val  输入值
 * @return  true: 已设置, false: 未设置
 */
bool foc_isset(float val);

/**
 * @brief   快速平方根近似（Fast Inverse Square Root）
 * @param   value  输入值
 * @return  近似 sqrt(value)
 */
float foc_sqrt_approx(float value);

#ifdef __cplusplus
}
#endif

#endif /* FOC_UTILS_H_ */
