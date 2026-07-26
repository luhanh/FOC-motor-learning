#ifndef AS5600_H_
#define AS5600_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* ===================== 寄存器地址 ===================== */
#define AS5600_REG_RAW_ANGLE_H  0x0C    // 原始角度高 8 位
#define AS5600_REG_RAW_ANGLE_L  0x0D    // 原始角度低 4 位
#define AS5600_REG_ANGLE_H      0x0E    // 滤波后角度高 8 位
#define AS5600_REG_ANGLE_L      0x0F    // 滤波后角度低 4 位
#define AS5600_REG_STATUS       0x0B    // 状态寄存器 (MD/MH/ML)

#define AS5600_12BIT_MASK       0x0FFF  // 12 位角度掩码
#define AS5600_DEFAULT_ADDR     0x36

/**
 * @brief AS5600 驱动配置结构
 */
typedef struct {
    i2c_port_num_t i2c_port;        // I2C 端口号，如 I2C_NUM_0
    int            sda_gpio;        // SDA 引脚
    int            scl_gpio;        // SCL 引脚
    uint8_t        dev_addr;        // 设备 7 位地址 (默认 0x36)
    uint32_t       i2c_speed_hz;    // I2C 速率 (默认 100000)
    bool           enable_internal_pullup; // 是否使能内部上拉
    uint8_t        glitch_ignore_cnt;      // 毛刺滤除计数
    float          speed_filter_alpha;     // 速度低通滤波系数 (0~1，越小越平滑)
    float          accel_filter_alpha;     // 加速度低通滤波系数 (0~1，越小越平滑)
} as5600_config_t;

/**
 * @brief 获取默认配置 (与 motor_angle_control.c 中 AS5600 配置一致)
 */
as5600_config_t as5600_default_config(void);

/**
 * @brief 初始化 AS5600 (创建 I2C 总线与设备，并加入为组件)
 *
 * @param config 配置指针，传 NULL 时使用 as5600_default_config()
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t as5600_init(const as5600_config_t *config);

/**
 * @brief 反初始化，释放 I2C 总线
 */
esp_err_t as5600_deinit(void);

/**
 * @brief 读取原始 12 位角度值 (0~4095)
 */
esp_err_t as5600_read_raw(uint16_t *raw);

/**
 * @brief 读取机械角度 (单位: 度, 0~360)
 */
esp_err_t as5600_read_angle_deg(float *angle_deg);

/**
 * @brief 读取机械角度 (单位: 弧度, 0~2π)
 */
esp_err_t as5600_read_angle_rad(float *angle_rad);

/**
 * @brief 采样更新：在控制循环（定时）中调用。
 *        内部读取角度，并基于时间戳计算速度(度/秒)与加速度(度/秒²)。
 *        首帧仅记录基准，速度/加速度保持为 0。
 *
 * @return ESP_OK 成功，其他值表示失败 (如 I2C 读取错误)
 */
esp_err_t as5600_update(void);

/* ===================== 获取缓存状态 (无需 I2C 读取) ===================== */
float as5600_get_angle(void);     // 当前机械角度 (度)
float as5600_get_speed(void);     // 角速度 (度/秒)
float as5600_get_accel(void);     // 角加速度 (度/秒²)
float as5600_get_angle_rad(void); // 当前机械角度 (弧度)
float as5600_get_speed_rad(void); // 角速度 (弧度/秒)
float as5600_get_accel_rad(void); // 角加速度 (弧度/秒²)

/**
 * @brief 磁铁是否检测到 (STATUS 寄存器 MD 位)
 */
bool as5600_magnet_detected(void);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H_ */
