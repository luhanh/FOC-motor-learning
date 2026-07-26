#include "bldc_foc_modulation.h"
#include "bldc_motor.h"
#include "foc_utils.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "bldc_foc";

/* ==================== 静态上下文 ==================== */

static bldc_modulation_type_t s_modulation = BLDC_MODULATION_SVPWM;
static bool s_modulation_centered = true;

/* ==================== FOC 调制内部辅助 ==================== */

/* ---- 梯形波 120° 换相表（6 扇区） ---- */
/* 每相取值: 1=正驱, -1=反驱, 0=高阻态关断 */
/* 参考: https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5 */
static const int s_trap_120_map[6][3] = {
    {0,  1, -1},
    {-1, 1,  0},
    {-1, 0,  1},
    {0, -1,  1},
    {1, -1,  0},
    {1,  0, -1},
};

/* ---- 梯形波 150° 换相表（12 扇区） ---- */
/* 参考: https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8 */
static const int s_trap_150_map[12][3] = {
    {0,  1, -1},
    {-1, 1, -1},
    {-1, 1,  0},
    {-1, 1,  1},
    {-1, 0,  1},
    {-1, -1, 1},
    {0, -1,  1},
    {1, -1,  1},
    {1, -1,  0},
    {1, -1, -1},
    {1,  0, -1},
    {1,  1, -1},
};

/* 三数最小值 */
static inline float s_min3(float a, float b, float c) {
    float m = (a < b) ? a : b;
    return (m < c) ? m : c;
}

/* 三数最大值 */
static inline float s_max3(float a, float b, float c) {
    float m = (a > b) ? a : b;
    return (m > c) ? m : c;
}

/* ---- 根据扇区号计算梯形波相位使能状态 ---- */
static void s_trap_get_phase_mask(const int map[][3], int sector,
                                  int *mask_u, int *mask_v, int *mask_w) {
    /* 全局使能模式下，高阻态通过 PWM 占空比 = 0 实现（而非关断 EN 引脚）。
     * 返回各相的使能掩码：0 = 高阻态（PWM=0），1 = 正常驱动 */
    if (map[sector][0] == BLDC_HIGH_IMPEDANCE) {
        *mask_u = 0; *mask_v = 1; *mask_w = 1;
    } else if (map[sector][1] == BLDC_HIGH_IMPEDANCE) {
        *mask_u = 1; *mask_v = 0; *mask_w = 1;
    } else if (map[sector][2] == BLDC_HIGH_IMPEDANCE) {
        *mask_u = 1; *mask_v = 1; *mask_w = 0;
    } else {
        *mask_u = 1; *mask_v = 1; *mask_w = 1;
    }
}

/* ==================== FOC 调制核心实现 ==================== */

/**
 * @brief   核心调制函数：dq 电压 + 电角度 → 三相电压 → PWM
 * @note    内部根据 s_modulation 选择调制策略，计算结果直接写入硬件
 */
static void s_foc_modulate(float Uq, float Ud, float angle_el) {
    float Ua, Ub, Uc;
    float center;
    int sector;
    float v_limit = bldc_motor_get_voltage_limit();

    switch (s_modulation) {
    case BLDC_MODULATION_TRAPEZOID_120: {
        /* 6 扇区梯形波换相（全局使能模式：高阻态通过 PWM=0 实现） */
        sector = (int)(6.0f * (foc_normalize_angle(angle_el + FOC_PI_6) / FOC_2PI));
        if (sector >= 6) sector = 0;

        int mu, mv, mw;
        s_trap_get_phase_mask(s_trap_120_map, sector, &mu, &mv, &mw);

        center = s_modulation_centered ? (v_limit / 2.0f) : Uq;

        Ua = (float)s_trap_120_map[sector][0] * Uq * (float)mu + center;
        Ub = (float)s_trap_120_map[sector][1] * Uq * (float)mv + center;
        Uc = (float)s_trap_120_map[sector][2] * Uq * (float)mw + center;
        break;
    }
    case BLDC_MODULATION_TRAPEZOID_150: {
        /* 12 扇区梯形波换相（全局使能模式：高阻态通过 PWM=0 实现） */
        sector = (int)(12.0f * (foc_normalize_angle(angle_el + FOC_PI_6) / FOC_2PI));
        if (sector >= 12) sector = 0;

        int mu, mv, mw;
        s_trap_get_phase_mask(s_trap_150_map, sector, &mu, &mv, &mw);

        center = s_modulation_centered ? (v_limit / 2.0f) : Uq;

        Ua = (float)s_trap_150_map[sector][0] * Uq * (float)mu + center;
        Ub = (float)s_trap_150_map[sector][1] * Uq * (float)mv + center;
        Uc = (float)s_trap_150_map[sector][2] * Uq * (float)mw + center;
        break;
    }
    case BLDC_MODULATION_SINE_PWM:
    case BLDC_MODULATION_SVPWM:
    default: {
        /* Park 逆变换: dq → αβ */
        float sa, ca;
        foc_sincos(angle_el, &sa, &ca);
        float Ualpha = ca * Ud - sa * Uq;
        float Ubeta = sa * Ud + ca * Uq;

        /* Clarke 逆变换: αβ → abc */
        Ua = Ualpha;
        Ub = -0.5f * Ualpha + FOC_SQRT3_2 * Ubeta;
        Uc = -0.5f * Ualpha - FOC_SQRT3_2 * Ubeta;

        if (s_modulation_centered) {
            center = v_limit / 2.0f;

            if (s_modulation == BLDC_MODULATION_SVPWM) {
                /* Midpoint Clamp SVPWM：零序分量注入，提高母线电压利用率 ~15% */
                float Umin = s_min3(Ua, Ub, Uc);
                float Umax = s_max3(Ua, Ub, Uc);
                center -= (Umax + Umin) / 2.0f;
            }

            Ua += center;
            Ub += center;
            Uc += center;
        } else {
            /* 非居中调制：最低相电压拉到 0，适合低侧电流采样 */
            float Umin = s_min3(Ua, Ub, Uc);
            Ua -= Umin;
            Ub -= Umin;
            Uc -= Umin;
        }
        break;
    }
    }

    /* 写入三相电压 → 驱动硬件 */
    bldc_motor_set_phase_voltage(Ua, Ub, Uc);
}

/* ==================== FOC 调制接口实现 ==================== */

esp_err_t bldc_foc_set_modulation(bldc_modulation_type_t modulation) {
    s_modulation = modulation;

    const char *name;
    switch (modulation) {
    case BLDC_MODULATION_SINE_PWM:
        name = "SinePWM";
        break;
    case BLDC_MODULATION_SVPWM:
        name = "SVPWM";
        break;
    case BLDC_MODULATION_TRAPEZOID_120:
        name = "Trapezoid_120";
        break;
    case BLDC_MODULATION_TRAPEZOID_150:
        name = "Trapezoid_150";
        break;
    default:
        name = "Unknown";
        break;
    }
    ESP_LOGI(TAG, "调制方式切换为: %s", name);
    return ESP_OK;
}

esp_err_t bldc_foc_set_modulation_centered(bool centered) {
    s_modulation_centered = centered;
    ESP_LOGI(TAG, "调制居中模式: %s", centered ? "启用" : "禁用");
    return ESP_OK;
}

esp_err_t bldc_foc_set_voltage(float Uq, float Ud, float angle_el) {
    if (!bldc_motor_is_initialized()) {
        ESP_LOGE(TAG, "驱动未初始化，无法执行 FOC 电压输出");
        return ESP_ERR_INVALID_STATE;
    }

    /* Uq/Ud 限幅 */
    float v_limit = bldc_motor_get_voltage_limit();
    Uq = foc_constrain(Uq, -v_limit, v_limit);
    Ud = foc_constrain(Ud, -v_limit, v_limit);

    /* 角度归一化 */
    angle_el = foc_normalize_angle(angle_el);

    /* 执行调制 → 写硬件 */
    s_foc_modulate(Uq, Ud, angle_el);

    return ESP_OK;
}

float bldc_foc_normalize_angle(float angle) {
    return foc_normalize_angle(angle);
}

float bldc_foc_electrical_angle(float mechanical_angle, int pole_pairs) {
    return foc_normalize_angle(mechanical_angle * (float)pole_pairs);
}
