#include "foc_utils.h"
#include <math.h>

/* ==================== 快速三角函数（查表 + 线性插值） ==================== */

/**
 * @brief   65 元素正弦查表（16bit 精度）
 * @note    对标 Arduino-FOC _sin()，精度 RMS ~0.000065
 */
__attribute__((weak)) float foc_sin(float a) {
    static const uint16_t sine_array[65] = {
        0,    804,  1608, 2411, 3212, 4011, 4808, 5602,
        6393, 7180, 7962, 8740, 9512, 10279,11039,11793,
        12540,13279,14010,14733,15447,16151,16846,17531,
        18205,18868,19520,20160,20788,21403,22006,22595,
        23170,23732,24279,24812,25330,25833,26320,26791,
        27246,27684,28106,28511,28899,29269,29622,29957,
        30274,30572,30853,31114,31357,31581,31786,31972,
        32138,32286,32413,32522,32610,32679,32729,32758,
        32768
    };

    int32_t t1, t2;
    unsigned int i = (unsigned int)(a * (64.0f * 4.0f * 256.0f / FOC_2PI));
    int frac = i & 0xff;
    i = (i >> 8) & 0xff;

    if (i < 64) {
        t1 = (int32_t)sine_array[i];
        t2 = (int32_t)sine_array[i + 1];
    } else if (i < 128) {
        t1 = (int32_t)sine_array[128 - i];
        t2 = (int32_t)sine_array[127 - i];
    } else if (i < 192) {
        t1 = -(int32_t)sine_array[i - 128];
        t2 = -(int32_t)sine_array[i - 127];
    } else {
        t1 = -(int32_t)sine_array[256 - i];
        t2 = -(int32_t)sine_array[255 - i];
    }

    return (1.0f / 32768.0f) * (float)(t1 + (((t2 - t1) * frac) >> 8));
}

__attribute__((weak)) float foc_cos(float a) {
    float a_sin = a + FOC_PI_2;
    if (a_sin > FOC_2PI) a_sin -= FOC_2PI;
    return foc_sin(a_sin);
}

__attribute__((weak)) void foc_sincos(float a, float *s, float *c) {
    *s = foc_sin(a);
    *c = foc_cos(a);
}

/* ==================== 快速 atan2 ==================== */

/**
 * @brief   多项式近似 atan2
 * @note    基于 Odrive 项目，MIT 许可
 *          https://math.stackexchange.com/a/1105038/81278
 */
__attribute__((weak)) float foc_atan2(float y, float x) {
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    /* 防止除零 */
    float a = (abs_x < abs_y) ? (abs_x / (abs_y + 1.175494351e-38f))
                              : (abs_y / (abs_x + 1.175494351e-38f));
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a +
              a;
    if (abs_y > abs_x) r = 1.57079637f - r;
    if (x < 0.0f) r = 3.14159274f - r;
    if (y < 0.0f) r = -r;
    return r;
}

/* ==================== 快速平方根 ==================== */

/**
 * @brief   快速平方根近似（Fast Inverse Square Root）
 * @note    https://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
__attribute__((weak)) float foc_sqrt_approx(float number) {
    union {
        float f;
        uint32_t i;
    } y = {.f = number};
    y.i = 0x5f375a86 - (y.i >> 1);
    return number * y.f;
}

/* ==================== 基础工具函数 ==================== */

float foc_sign(float x) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

float foc_constrain(float amt, float low, float high) {
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

bool foc_isset(float val) {
    return val != FOC_NOT_SET;
}

float foc_normalize_angle(float angle) {
    float a = fmodf(angle, FOC_2PI);
    if (a < 0.0f) a += FOC_2PI;
    return a;
}

float foc_electrical_angle_calc(float shaft_angle, int pole_pairs) {
    return shaft_angle * (float)pole_pairs;
}
