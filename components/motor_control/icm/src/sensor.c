/**
 * @file    sensor.c
 * @brief   传感器硬件抽象层实现
 *
 * 通过函数指针实现硬件隔离：
 *   - sensor_t.ops 存储函数指针表，指向具体驱动的实现
 *   - 所有对外接口通过 sensor->ops->xxx() 间接调用
 *   - 本文件不引用任何具体传感器头文件
 *   - 替换/新增传感器：新增 sensor_init_as5048a() 函数 + 在 sensor_init() 中添加 case 分支
 *
 * 架构：
 *   motor_control.c  →  sensor.h (统一接口)
 *                           ↓
 *                       sensor.c (本文件，函数指针调度)
 *                           ↓
 *          ┌────────────────┼────────────────┐
 *          ↓                ↓                ↓
 *     AS5600.c          AS5048A.c        MT6701.c
 *    (I2C驱动)         (SPI驱动)        (SSI驱动)
 */

#include "sensor.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "sensor";

/* ==================== 传感器上下文（对上层隐藏） ==================== */

struct sensor_t {
    sensor_ops_t ops;         /* 函数指针表（指向具体驱动实现） */
    bool initialized;
};

/* ==================== 各传感器驱动的 ops 注入函数 ==================== */

/**
 * @brief 注入 AS5600 驱动的函数指针
 * @note  本函数引用 AS5600.h，是 sensor.c 中唯一与具体硬件耦合的地方。
 *        新增传感器时，添加对应的注入函数即可。
 */
#ifdef CONFIG_SENSOR_AS5600_ENABLE
#include "AS5600.h"

static esp_err_t s_as5600_init(void) {
    return as5600_init(NULL);
}

static esp_err_t s_as5600_deinit(void) {
    return as5600_deinit();
}

static esp_err_t s_as5600_update(void) {
    return as5600_update();
}

static float s_as5600_get_angle_rad(void) {
    return as5600_get_angle_rad();
}

static float s_as5600_get_velocity_rad(void) {
    return as5600_get_speed_rad();
}

static bool s_as5600_needs_search(void) {
    return false;  /* 磁编码器无索引信号 */
}

static void sensor_ops_init_as5600(sensor_ops_t *ops) {
    ops->init = s_as5600_init;
    ops->deinit = s_as5600_deinit;
    ops->update = s_as5600_update;
    ops->get_angle_rad = s_as5600_get_angle_rad;
    ops->get_velocity_rad = s_as5600_get_velocity_rad;
    ops->needs_search = s_as5600_needs_search;
}

#else
static void sensor_ops_init_as5600(sensor_ops_t *ops) {
    (void)ops;
    ESP_LOGW(TAG, "AS5600 未启用 (CONFIG_SENSOR_AS5600_ENABLE=n)");
}
#endif /* CONFIG_SENSOR_AS5600_ENABLE */

/* ==================== 对外接口（通过函数指针间接调用） ==================== */

esp_err_t sensor_init(sensor_t **out_sensor, sensor_type_t type) {
    if (out_sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sensor_t *s = calloc(1, sizeof(sensor_t));
    if (s == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* 根据类型注入对应驱动的函数指针 */
    switch (type) {
    case SENSOR_TYPE_AS5600:
        sensor_ops_init_as5600(&s->ops);
        break;
    case SENSOR_TYPE_AS5048A:
    case SENSOR_TYPE_MT6701:
    default:
        ESP_LOGE(TAG, "传感器类型 %d 未实现", type);
        free(s);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* 调用驱动的初始化函数 */
    if (s->ops.init) {
        esp_err_t ret = s->ops.init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "传感器硬件初始化失败");
            free(s);
            return ret;
        }
    }

    s->initialized = true;
    *out_sensor = s;

    ESP_LOGI(TAG, "传感器初始化完成 (type=%d)", type);
    return ESP_OK;
}

esp_err_t sensor_deinit(sensor_t *sensor) {
    if (sensor == NULL) {
        return ESP_OK;
    }

    if (sensor->ops.deinit) {
        sensor->ops.deinit();
    }
    free(sensor);

    ESP_LOGI(TAG, "传感器已反初始化");
    return ESP_OK;
}

esp_err_t sensor_update(sensor_t *sensor) {
    if (sensor == NULL || !sensor->initialized || sensor->ops.update == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return sensor->ops.update();
}

float sensor_get_angle_rad(sensor_t *sensor) {
    if (sensor == NULL || !sensor->initialized || sensor->ops.get_angle_rad == NULL) {
        return 0.0f;
    }
    return sensor->ops.get_angle_rad();
}

float sensor_get_velocity_rad(sensor_t *sensor) {
    if (sensor == NULL || !sensor->initialized || sensor->ops.get_velocity_rad == NULL) {
        return 0.0f;
    }
    return sensor->ops.get_velocity_rad();
}

bool sensor_needs_search(sensor_t *sensor) {
    if (sensor == NULL || !sensor->initialized || sensor->ops.needs_search == NULL) {
        return false;
    }
    return sensor->ops.needs_search();
}
