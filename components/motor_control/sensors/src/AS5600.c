#include "AS5600.h"

#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "AS5600";

/* ===================== 内部运行状态 ===================== */
typedef struct {
    i2c_master_bus_handle_t  bus;
    i2c_master_dev_handle_t  dev;
    bool                     initialized;

    float angle;        // 当前机械角度 (度)
    float speed;        // 角速度 (度/秒)
    float accel;        // 角加速度 (度/秒²)

    float last_angle;      // 上一帧角度 (度)
    float last_speed;      // 上一帧速度 (度/秒)
    int64_t last_time_us;  // 上一帧时间戳 (us)

    float speed_filter_alpha;
    float accel_filter_alpha;
} as5600_ctx_t;

static as5600_ctx_t s_ctx = {
    .initialized = false,
    .angle = 0.0f, .speed = 0.0f, .accel = 0.0f,
    .last_angle = 0.0f, .last_speed = 0.0f, .last_time_us = 0,
    .speed_filter_alpha = 0.2f, .accel_filter_alpha = 0.2f,
};

/* ===================== 默认配置 ===================== */
as5600_config_t as5600_default_config(void) {
    as5600_config_t c = {
        .i2c_port             = I2C_NUM_0,
        .sda_gpio             = 1,
        .scl_gpio             = 2,
        .dev_addr             = AS5600_DEFAULT_ADDR,
        .i2c_speed_hz         = 100000,
        .enable_internal_pullup = true,
        .glitch_ignore_cnt    = 7,
        .speed_filter_alpha   = 0.2f,
        .accel_filter_alpha   = 0.2f,
    };
    return c;
}

/* ===================== 初始化 / 反初始化 ===================== */
esp_err_t as5600_init(const as5600_config_t *config) {
    if (s_ctx.initialized) {
        return ESP_OK;  // 已初始化，重复调用安全返回
    }

    as5600_config_t cfg;
    if (config == NULL) {
        cfg = as5600_default_config();
    } else {
        cfg = *config;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .i2c_port             = cfg.i2c_port,
        .scl_io_num           = cfg.scl_gpio,
        .sda_io_num           = cfg.sda_gpio,
        .glitch_ignore_cnt    = cfg.glitch_ignore_cnt,
        .flags.enable_internal_pullup = cfg.enable_internal_pullup,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_ctx.bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建 I2C 总线失败: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg.dev_addr,
        .scl_speed_hz    = cfg.i2c_speed_hz,
    };

    ret = i2c_master_bus_add_device(s_ctx.bus, &dev_cfg, &s_ctx.dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "添加 AS5600 设备失败: %s", esp_err_to_name(ret));
        i2c_del_master_bus(s_ctx.bus);
        s_ctx.bus = NULL;
        return ret;
    }

    s_ctx.speed_filter_alpha = cfg.speed_filter_alpha;
    s_ctx.accel_filter_alpha = cfg.accel_filter_alpha;
    s_ctx.angle = s_ctx.speed = s_ctx.accel = 0.0f;
    s_ctx.last_angle = s_ctx.last_speed = 0.0f;
    s_ctx.last_time_us = 0;
    s_ctx.initialized = true;

    ESP_LOGI(TAG, "AS5600 初始化完成 (addr=0x%02X, sda=%d, scl=%d, %luHz)",
             cfg.dev_addr, cfg.sda_gpio, cfg.scl_gpio, cfg.i2c_speed_hz);
    return ESP_OK;
}

esp_err_t as5600_deinit(void) {
    if (!s_ctx.initialized) {
        return ESP_OK;
    }
    if (s_ctx.bus != NULL) {
        i2c_del_master_bus(s_ctx.bus);
        s_ctx.bus = NULL;
        s_ctx.dev = NULL;
    }
    s_ctx.initialized = false;
    return ESP_OK;
}

/* ===================== 原始角度读取 ===================== */
esp_err_t as5600_read_raw(uint16_t *raw) {
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = AS5600_REG_RAW_ANGLE_H;  // 0x0C
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(s_ctx.dev, &reg, 1, data, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读取 RAW ANGLE 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // data[0]=角度高8位, data[1]=角度低4位(高4位为0)，掩码取 12 位
    *raw = ((uint16_t)data[0] << 8 | data[1]) & AS5600_12BIT_MASK;
    return ESP_OK;
}

esp_err_t as5600_read_angle_deg(float *angle_deg) {
    uint16_t raw;
    esp_err_t ret = as5600_read_raw(&raw);
    if (ret != ESP_OK) {
        return ret;
    }
    *angle_deg = (float)raw * 360.0f / 4096.0f;
    return ESP_OK;
}

esp_err_t as5600_read_angle_rad(float *angle_rad) {
    float deg;
    esp_err_t ret = as5600_read_angle_deg(&deg);
    if (ret != ESP_OK) {
        return ret;
    }
    *angle_rad = deg * (PI / 180.0f);
    return ESP_OK;
}

/* ===================== 采样更新：计算速度/加速度 ===================== */
esp_err_t as5600_update(void) {
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw;
    esp_err_t ret = as5600_read_raw(&raw);
    if (ret != ESP_OK) {
        return ret;  // 读取失败则保持上一帧状态
    }

    float angle = (float)raw * 360.0f / 4096.0f;   // 度
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - s_ctx.last_time_us) / 1e6f;

    float inst_speed = 0.0f;
    float inst_accel = 0.0f;

    if (s_ctx.last_time_us > 0 && dt > 0.0f) {
        // 处理 0↔360 回绕
        float d = angle - s_ctx.last_angle;
        if (d > 180.0f)       d -= 360.0f;
        else if (d < -180.0f) d += 360.0f;

        inst_speed = d / dt;                       // 度/秒
        inst_accel = (inst_speed - s_ctx.last_speed) / dt;  // 度/秒²

        // 一阶低通滤波，抑制噪声
        s_ctx.speed = s_ctx.speed_filter_alpha * inst_speed
                    + (1.0f - s_ctx.speed_filter_alpha) * s_ctx.speed;
        s_ctx.accel = s_ctx.accel_filter_alpha * inst_accel
                    + (1.0f - s_ctx.accel_filter_alpha) * s_ctx.accel;
    }

    s_ctx.last_angle   = angle;
    s_ctx.last_speed   = inst_speed;
    s_ctx.last_time_us = now;
    s_ctx.angle        = angle;

    return ESP_OK;
}

/* ===================== 状态获取 ===================== */
float as5600_get_angle(void)     { return s_ctx.angle; }
float as5600_get_speed(void)     { return s_ctx.speed; }
float as5600_get_accel(void)     { return s_ctx.accel; }
float as5600_get_angle_rad(void) { return s_ctx.angle * (PI / 180.0f); }
float as5600_get_speed_rad(void) { return s_ctx.speed * (PI / 180.0f); }
float as5600_get_accel_rad(void) { return s_ctx.accel * (PI / 180.0f); }

/* ===================== 磁铁检测 ===================== */
bool as5600_magnet_detected(void) {
    if (!s_ctx.initialized) {
        return false;
    }
    uint8_t reg = AS5600_REG_STATUS;  // 0x0B
    uint8_t status = 0;
    if (i2c_master_transmit_receive(s_ctx.dev, &reg, 1, &status, 1, -1) != ESP_OK) {
        return false;
    }
    return (status & 0x20) != 0;  // MD 位 = 磁铁检测到
}
