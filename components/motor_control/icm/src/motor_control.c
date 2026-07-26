#include "motor_control.h"
#include "bldc_motor.h"
#include "bldc_foc_modulation.h"
#include "sensor.h"
#include "foc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "hal/gpio_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor_ctrl";

/* ==================== 电机控制上下文（对 main.c 隐藏） ==================== */

struct motor_control_t {
    bldc_motor_config_t motor_cfg;
    foc_context_t        foc;
    sensor_t            *sensor;    /* 传感器抽象句柄 */
};

/* ==================== 硬件配置（内部，main.c 不感知） ==================== */

static bldc_motor_config_t get_default_motor_config(void) {
    bldc_motor_config_t cfg = {
        .pwm_pin_u = 10,
        .pwm_pin_v = 11,
        .pwm_pin_w = 12,
        .enable_pin = 4,                     /* GPIO4: 驱动电路全局使能 */
        .enable_pin_u = BLDC_MOTOR_PIN_NC,   /* 预留：U 相独立使能 */
        .enable_pin_v = BLDC_MOTOR_PIN_NC,   /* 预留：V 相独立使能 */
        .enable_pin_w = BLDC_MOTOR_PIN_NC,   /* 预留：W 相独立使能 */
        .enable_active_high = true,
        .voltage_power_supply = 12.0f,
        .voltage_limit = 0.0f,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .channel_u = LEDC_CHANNEL_0,
        .channel_v = LEDC_CHANNEL_1,
        .channel_w = LEDC_CHANNEL_2,
        .pwm_freq_hz = 20000,
        .duty_resolution = LEDC_TIMER_8_BIT,
    };
    return cfg;
}

static foc_motor_params_t get_default_motor_params(void) {
    foc_motor_params_t params = {
        .pole_pairs = 7,
        .phase_resistance = FOC_NOT_SET,
        .kv_rating = FOC_NOT_SET,
        .phase_inductance = FOC_NOT_SET,
        .axis_inductance = {FOC_NOT_SET, FOC_NOT_SET},
    };
    return params;
}

/* ==================== 回调实现 ==================== */

static motor_control_t *s_active_mc = NULL;

static void s_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void s_delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

static int64_t s_get_time_us(void) {
    return esp_timer_get_time();
}

static void s_set_phase_voltage(float Uq, float Ud, float angle_el) {
    bldc_foc_set_voltage(Uq, Ud, angle_el);
}

static void s_sensor_update(void) {
    if (s_active_mc && s_active_mc->sensor) {
        sensor_update(s_active_mc->sensor);
    }
}

static float s_sensor_get_angle(void) {
    if (s_active_mc && s_active_mc->sensor) {
        return sensor_get_angle_rad(s_active_mc->sensor);
    }
    return 0.0f;
}

static float s_sensor_get_velocity(void) {
    if (s_active_mc && s_active_mc->sensor) {
        return sensor_get_velocity_rad(s_active_mc->sensor);
    }
    return 0.0f;
}

static float s_sensor_get_mechanical_angle(void) {
    if (s_active_mc && s_active_mc->sensor) {
        return sensor_get_angle_rad(s_active_mc->sensor);
    }
    return 0.0f;
}

static bool s_sensor_needs_search(void) {
    if (s_active_mc && s_active_mc->sensor) {
        return sensor_needs_search(s_active_mc->sensor);
    }
    return false;
}

static void s_driver_enable(void) {
    bldc_motor_enable();
}

static void s_driver_disable(void) {
    bldc_motor_disable();
}

static void foc_inject_callbacks(foc_context_t *ctx) {
    ctx->delay_ms = s_delay_ms;
    ctx->delay_us = s_delay_us;
    ctx->get_time_us = s_get_time_us;
    ctx->set_phase_voltage = s_set_phase_voltage;
    ctx->sensor_update = s_sensor_update;
    ctx->sensor_get_angle = s_sensor_get_angle;
    ctx->sensor_get_velocity = s_sensor_get_velocity;
    ctx->sensor_get_mechanical_angle = s_sensor_get_mechanical_angle;
    ctx->sensor_needs_search = s_sensor_needs_search;
    ctx->driver_enable = s_driver_enable;
    ctx->driver_disable = s_driver_disable;
}

/* ==================== 对外接口 ==================== */

esp_err_t motor_control_init(motor_control_t **out_mc) {
    if (out_mc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_control_t *mc = calloc(1, sizeof(motor_control_t));
    if (mc == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_active_mc = mc;

    /* 1. 初始化传感器（通过抽象层，不感知具体型号） */
    esp_err_t ret = sensor_init(&mc->sensor, SENSOR_TYPE_AS5600);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "传感器初始化失败");
        free(mc);
        return ret;
    }

    /* 2. 初始化电机驱动 */
    mc->motor_cfg = get_default_motor_config();
    ret = bldc_motor_init(&mc->motor_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机驱动初始化失败");
        sensor_deinit(mc->sensor);
        free(mc);
        return ret;
    }

    /* 3. 初始化 FOC 算法 */
    foc_motor_params_t motor_params = get_default_motor_params();
    ret = foc_init(&mc->foc, &motor_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FOC 算法初始化失败");
        bldc_motor_deinit();
        sensor_deinit(mc->sensor);
        free(mc);
        return ret;
    }

    /* 4. 注入回调 */
    foc_inject_callbacks(&mc->foc);

    *out_mc = mc;
    ESP_LOGI(TAG, "电机控制系统初始化完成");
    return ESP_OK;
}

esp_err_t motor_control_deinit(motor_control_t *mc) {
    if (mc == NULL) return ESP_OK;

    motor_control_disable(mc);
    sensor_deinit(mc->sensor);
    bldc_motor_deinit();
    s_active_mc = NULL;
    free(mc);

    ESP_LOGI(TAG, "电机控制系统已反初始化");
    return ESP_OK;
}

esp_err_t motor_control_calibrate(motor_control_t *mc) {
    if (mc == NULL) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "开始 FOC 校准...");
    motor_control_enable(mc);

    esp_err_t ret = foc_init_foc(&mc->foc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FOC 校准失败");
        motor_control_disable(mc);
        return ret;
    }

    ESP_LOGI(TAG, "FOC 校准完成");
    return ESP_OK;
}

void motor_control_step(motor_control_t *mc) {
    if (mc == NULL) return;

    foc_loop_foc(&mc->foc);

    /* 运动外环每 10 次电流环执行一次 (100Hz @ 1kHz) */
    static int cnt = 0;
    if (++cnt >= 10) {
        cnt = 0;
        foc_move(&mc->foc, NAN);
    }
}

esp_err_t motor_control_enable(motor_control_t *mc) {
    if (mc == NULL) return ESP_ERR_INVALID_ARG;
    bldc_motor_enable();
    mc->foc.enabled = true;
    ESP_LOGI(TAG, "电机输出已使能");
    return ESP_OK;
}

esp_err_t motor_control_disable(motor_control_t *mc) {
    if (mc == NULL) return ESP_ERR_INVALID_ARG;
    mc->foc.enabled = false;
    bldc_motor_disable();
    ESP_LOGI(TAG, "电机输出已禁用");
    return ESP_OK;
}

void motor_control_set_target(motor_control_t *mc, float target) {
    if (mc == NULL) return;
    foc_set_target(&mc->foc, target);
}

void motor_control_set_control_type(motor_control_t *mc, mc_control_type_t type) {
    if (mc == NULL) return;
    foc_control_type_t foc_type;
    switch (type) {
    case MC_CTRL_VELOCITY:
        foc_type = FOC_CTRL_VELOCITY;
        break;
    case MC_CTRL_ANGLE:
        foc_type = FOC_CTRL_ANGLE;
        break;
    default:
        foc_type = FOC_CTRL_TORQUE;
        break;
    }
    foc_set_control_type(&mc->foc, foc_type);
}

void motor_control_set_torque_type(motor_control_t *mc, mc_torque_type_t type) {
    if (mc == NULL) return;
    (void)type;
    foc_set_torque_type(&mc->foc, FOC_TORQUE_VOLTAGE);
}
