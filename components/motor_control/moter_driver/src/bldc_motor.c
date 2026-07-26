#include "bldc_motor.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "bldc_motor";

/* ==================== 静态上下文 ==================== */

static bool s_initialized = false;
static bldc_motor_config_t s_config;
static uint32_t s_max_duty; /* (2^duty_resolution - 1), 用于电压→占空比换算 */

/* ==================== 内部辅助宏 ==================== */

/* 判断引脚是否有效（非哨兵值） */
#define s_pin_isset(pin) ((pin) != BLDC_MOTOR_PIN_NC)

/* 数值限幅 */
#define s_constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

/* 获取使能引脚的有效电平 */
static inline int s_en_level(void) {
    return s_config.enable_active_high ? 1 : 0;
}

static inline int s_dis_level(void) {
    return s_config.enable_active_high ? 0 : 1;
}

/* ==================== GPIO 初始化 ==================== */

/**
 * @brief   初始化单个 GPIO 为输出模式（如果引脚有效）
 */
static esp_err_t gpio_output_init(int pin) {
    if (!s_pin_isset(pin)) {
        return ESP_OK;
    }
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_conf);
}

/**
 * @brief   初始化全局使能引脚，拉至失能电平
 */
static esp_err_t enable_gpio_init(void) {
    esp_err_t ret;

    /* 初始化全局使能引脚 */
    ret = gpio_output_init(s_config.enable_pin);
    if (ret != ESP_OK) return ret;

    if (s_pin_isset(s_config.enable_pin)) {
        gpio_set_level(s_config.enable_pin, s_dis_level());
    }

    /* 预留：初始化三相独立使能引脚（当前硬件不使用） */
    ret = gpio_output_init(s_config.enable_pin_u);
    if (ret != ESP_OK) return ret;
    ret = gpio_output_init(s_config.enable_pin_v);
    if (ret != ESP_OK) return ret;
    ret = gpio_output_init(s_config.enable_pin_w);
    if (ret != ESP_OK) return ret;

    if (s_pin_isset(s_config.enable_pin_u)) {
        gpio_set_level(s_config.enable_pin_u, s_dis_level());
    }
    if (s_pin_isset(s_config.enable_pin_v)) {
        gpio_set_level(s_config.enable_pin_v, s_dis_level());
    }
    if (s_pin_isset(s_config.enable_pin_w)) {
        gpio_set_level(s_config.enable_pin_w, s_dis_level());
    }

    return ESP_OK;
}

/* ==================== LEDC PWM 初始化 ==================== */

static esp_err_t ledc_pwm_init(const bldc_motor_config_t *cfg) {
    /* 配置 LEDC 定时器 */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = cfg->speed_mode,
        .timer_num = cfg->timer_num,
        .duty_resolution = cfg->duty_resolution,
        .freq_hz = cfg->pwm_freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC 定时器配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 共用通道配置模板 */
    ledc_channel_config_t ch_cfg = {
        .speed_mode = cfg->speed_mode,
        .timer_sel = cfg->timer_num,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };

    /* 配置 U 相通道 */
    ch_cfg.channel = cfg->channel_u;
    ch_cfg.gpio_num = cfg->pwm_pin_u;
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "U 相 LEDC 通道配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 配置 V 相通道 */
    ch_cfg.channel = cfg->channel_v;
    ch_cfg.gpio_num = cfg->pwm_pin_v;
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "V 相 LEDC 通道配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 配置 W 相通道 */
    ch_cfg.channel = cfg->channel_w;
    ch_cfg.gpio_num = cfg->pwm_pin_w;
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "W 相 LEDC 通道配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/* ==================== 电压→占空比换算 ==================== */

/**
 * @brief   将相电压转换为 PWM 占空比
 * @note    对标 BLDCDriver3PWM::setPwm() 中的 dc = constrain(U / Vsupply, 0, 1)
 */
static inline uint32_t voltage_to_duty(float voltage) {
    float dc = s_constrain(voltage / s_config.voltage_power_supply, 0.0f, 1.0f);
    return (uint32_t)(dc * (float)s_max_duty + 0.5f);
}

/* ==================== 基础接口实现 ==================== */

esp_err_t bldc_motor_init(const bldc_motor_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "配置参数不可为空");
        return ESP_ERR_INVALID_ARG;
    }

    /* 保存配置 */
    s_config = *config;

    /* 电压上限校验：未设置或超过母线电压时取母线电压 */
    if (s_config.voltage_limit <= 0.0f || s_config.voltage_limit > s_config.voltage_power_supply) {
        s_config.voltage_limit = s_config.voltage_power_supply;
    }

    /* 计算最大占空比值 */
    s_max_duty = (1U << s_config.duty_resolution) - 1;

    /* 初始化全局使能 GPIO */
    esp_err_t ret = enable_gpio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "使能引脚 GPIO 初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 初始化 LEDC PWM */
    ret = ledc_pwm_init(&s_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC PWM 初始化失败");
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "BLDC 电机驱动初始化完成 "
             "(freq=%lu Hz, Vsup=%.1f V, Vlim=%.1f V, "
             "U=%d V=%d W=%d, EN=%d, active=%s)",
             s_config.pwm_freq_hz,
             (double)s_config.voltage_power_supply,
             (double)s_config.voltage_limit,
             s_config.pwm_pin_u,
             s_config.pwm_pin_v,
             s_config.pwm_pin_w,
             s_config.enable_pin,
             s_config.enable_active_high ? "high" : "low");
    return ESP_OK;
}

esp_err_t bldc_motor_deinit(void) {
    if (!s_initialized) {
        return ESP_OK;
    }

    /* 先禁用电机 */
    bldc_motor_disable();

    /* 释放 LEDC 资源：停止三路通道 */
    ledc_stop(s_config.speed_mode, s_config.channel_u, 0);
    ledc_stop(s_config.speed_mode, s_config.channel_v, 0);
    ledc_stop(s_config.speed_mode, s_config.channel_w, 0);

    /* 复位全局使能引脚 */
    if (s_pin_isset(s_config.enable_pin)) {
        gpio_reset_pin(s_config.enable_pin);
    }
    /* 复位预留的三相独立使能引脚 */
    if (s_pin_isset(s_config.enable_pin_u)) {
        gpio_reset_pin(s_config.enable_pin_u);
    }
    if (s_pin_isset(s_config.enable_pin_v)) {
        gpio_reset_pin(s_config.enable_pin_v);
    }
    if (s_pin_isset(s_config.enable_pin_w)) {
        gpio_reset_pin(s_config.enable_pin_w);
    }

    s_initialized = false;
    ESP_LOGI(TAG, "BLDC 电机驱动已反初始化");
    return ESP_OK;
}

esp_err_t bldc_motor_enable(void) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "驱动未初始化，无法使能");
        return ESP_ERR_INVALID_STATE;
    }

    /* 先拉高全局使能引脚（对标 Arduino-FOC: 先 digitalWrite(EN, high)） */
    if (s_pin_isset(s_config.enable_pin)) {
        gpio_set_level(s_config.enable_pin, s_en_level());
    }

    /* 再设 PWM=0，防止使能瞬间出现不确定输出（对标 Arduino-FOC: setPwm(0,0,0)） */
    bldc_motor_set_duty(0, 0, 0);

    ESP_LOGI(TAG, "电机已使能");
    return ESP_OK;
}

esp_err_t bldc_motor_disable(void) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "驱动未初始化，无法禁用");
        return ESP_ERR_INVALID_STATE;
    }

    /* 先 PWM 归零（对标 Arduino-FOC: setPwm(0,0,0)） */
    bldc_motor_set_duty(0, 0, 0);

    /* 再拉低全局使能引脚（对标 Arduino-FOC: digitalWrite(EN, !high)） */
    if (s_pin_isset(s_config.enable_pin)) {
        gpio_set_level(s_config.enable_pin, s_dis_level());
    }

    ESP_LOGI(TAG, "电机已禁用");
    return ESP_OK;
}

esp_err_t bldc_motor_set_duty(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "驱动未初始化，无法设置占空比");
        return ESP_ERR_INVALID_STATE;
    }

    ledc_set_duty(s_config.speed_mode, s_config.channel_u, duty_u);
    ledc_update_duty(s_config.speed_mode, s_config.channel_u);
    ledc_set_duty(s_config.speed_mode, s_config.channel_v, duty_v);
    ledc_update_duty(s_config.speed_mode, s_config.channel_v);
    ledc_set_duty(s_config.speed_mode, s_config.channel_w, duty_w);
    ledc_update_duty(s_config.speed_mode, s_config.channel_w);

    return ESP_OK;
}

esp_err_t bldc_motor_set_phase_voltage(float voltage_u, float voltage_v, float voltage_w) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "驱动未初始化，无法设置相电压");
        return ESP_ERR_INVALID_STATE;
    }

    /* 电压限幅（对标 Arduino-FOC: _constrain(U, 0, voltage_limit)） */
    voltage_u = s_constrain(voltage_u, 0.0f, s_config.voltage_limit);
    voltage_v = s_constrain(voltage_v, 0.0f, s_config.voltage_limit);
    voltage_w = s_constrain(voltage_w, 0.0f, s_config.voltage_limit);

    /* 电压→占空比换算 */
    uint32_t duty_u = voltage_to_duty(voltage_u);
    uint32_t duty_v = voltage_to_duty(voltage_v);
    uint32_t duty_w = voltage_to_duty(voltage_w);

    return bldc_motor_set_duty(duty_u, duty_v, duty_w);
}

/* ==================== 查询接口 ==================== */

float bldc_motor_get_voltage_limit(void) {
    return s_config.voltage_limit;
}

bool bldc_motor_is_initialized(void) {
    return s_initialized;
}
