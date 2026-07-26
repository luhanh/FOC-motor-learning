#ifndef SENSOR_H_
#define SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"

/**
 * @file    sensor.h
 * @brief   传感器硬件抽象层（HAL）
 *
 * 通过函数指针实现硬件隔离与可替换：
 *   - sensor_t 内部存储函数指针，指向具体传感器驱动的实现
 *   - 替换传感器时，新增对应的 sensor_init_as5048a() 即可
 *   - sensor.c 不引用任何具体传感器头文件（AS5600.h 等）
 *   - 上层代码调用统一接口，完全不感知底层传感器型号
 */

/* ==================== 传感器操作接口（函数指针表） ==================== */

typedef struct {
    esp_err_t (*init)(void);             /* 初始化 */
    esp_err_t (*deinit)(void);           /* 反初始化 */
    esp_err_t (*update)(void);           /* 采样更新 */
    float (*get_angle_rad)(void);        /* 获取角度 (rad) */
    float (*get_velocity_rad)(void);     /* 获取角速度 (rad/s) */
    bool (*needs_search)(void);          /* 是否需要索引搜索 */
} sensor_ops_t;

/* ==================== 传感器抽象上下文（不透明句柄） ==================== */

typedef struct sensor_t sensor_t;

/* ==================== 传感器类型枚举 ==================== */

typedef enum {
    SENSOR_TYPE_AS5600 = 0,   /* AS5600 磁编码器 (I2C) */
    SENSOR_TYPE_AS5048A,      /* AS5048A 磁编码器 (SPI) — 预留 */
    SENSOR_TYPE_MT6701,       /* MT6701 磁编码器 (SSI/I2C) — 预留 */
} sensor_type_t;

/* ==================== API ==================== */

/**
 * @brief   创建并初始化传感器（根据类型注入对应驱动的函数指针）
 * @param   out_sensor  输出：传感器句柄
 * @param   type        传感器型号
 * @return  ESP_OK: 成功
 */
esp_err_t sensor_init(sensor_t **out_sensor, sensor_type_t type);

/**
 * @brief   反初始化传感器，释放硬件资源
 * @param   sensor  传感器句柄
 * @return  ESP_OK: 成功
 */
esp_err_t sensor_deinit(sensor_t *sensor);

/**
 * @brief   采样更新（在控制循环中调用）
 * @param   sensor  传感器句柄
 * @return  ESP_OK: 成功
 */
esp_err_t sensor_update(sensor_t *sensor);

/**
 * @brief   获取当前机械角度
 * @param   sensor  传感器句柄
 * @return  角度 (rad)，范围 [0, 2π)
 */
float sensor_get_angle_rad(sensor_t *sensor);

/**
 * @brief   获取当前角速度
 * @param   sensor  传感器句柄
 * @return  角速度 (rad/s)
 */
float sensor_get_velocity_rad(sensor_t *sensor);

/**
 * @brief   是否需要索引搜索（编码器 ABZ 信号）
 * @param   sensor  传感器句柄
 * @return  true: 需要搜索, false: 不需要
 */
bool sensor_needs_search(sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H_ */
