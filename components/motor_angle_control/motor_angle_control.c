#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "motor_angle_control.h"

static const char *TAG = "motor_angle";

// ==================== 引脚定义 ====================
#define IN1_GPIO        10
#define IN2_GPIO        11
#define IN3_GPIO        12
#define EN_GPIO         4

#define SDA_GPIO        1
#define SCL_GPIO        2
#define AS5600_ADDR     0x36

// ==================== LEDC PWM 配置 ====================
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_IN1     LEDC_CHANNEL_0
#define LEDC_CH_IN2     LEDC_CHANNEL_1
#define LEDC_CH_IN3     LEDC_CHANNEL_2
#define LEDC_FREQ_HZ    20000

// ==================== UART 配置 ====================
#define UART_PORT       UART_NUM_0
#define UART_BAUD       115200
#define UART_RX_BUF     256
#define UART_TX_BUF     0

// ==================== 电机参数 ====================
static const int   POLE_PAIRS    = 7;
static const float PHASE_OFFSET  = 0.0f;

// ==================== 母线电压（用于 SVPWM 电压→占空比换算） ====================
static const float VOLTAGE_POWER_SUPPLY = 12.0f;

// ==================== 控制参数 ====================
static const float MOVE_PWM      = 205.0f;    // 运行力矩
// static const float HOLD_PWM      = 0.0f;     // 锁定力矩（零电流）
static const float DEAD_ZONE     = 0.8f;     // 到位误差区间

// ==================== 调试参数 ====================
// 调试输出周期（控制周期数），0=关闭
static const int   DEBUG_PRINT_PERIOD = 50;   // 50周期=500ms输出一次

// ==================== PID 参数 ====================
static const float KP = 0.35f;
static const float KI = 0.2f;
static const float KD = 0.05f;

// ==================== 运行变量 ====================
static float target_angle  = 0.0f;
static float current_angle = 0.0f;

// I2C 设备句柄
static i2c_master_dev_handle_t as5600_handle = NULL;

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// ====================== 【UART 输出辅助】 ======================
static void uart_printf(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        uart_write_bytes(UART_PORT, buf, len);
    }
}

static void uart_println(const char *str) {
    uart_write_bytes(UART_PORT, str, strlen(str));
    uart_write_bytes(UART_PORT, "\r\n", 2);
}

// ==================== 角度归一化：-180 ~ 180 ====================
static float normalizeAngle(float angle) {
    angle = fmod(angle, 360.0f);
    if (angle > 180.0f)  angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

// ==================== AS5600 角度读取 (I2C) ====================
static float readAS5600Angle(void) {
    static float last_valid_angle = 0.0f;

    if (as5600_handle == NULL) {
        return last_valid_angle;
    }

    // AS5600 寄存器 0x0C = RAW ANGLE (高8位), 0x0D = RAW ANGLE (低4位)
    uint8_t reg = 0x0C;
    uint8_t data[2] = {0};

    esp_err_t ret = i2c_master_transmit_receive(as5600_handle, &reg, 1, data, 2, -1);
    if (ret != ESP_OK) {
        return last_valid_angle;
    }

    uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
    last_valid_angle = raw_angle * 360.0f / 4096.0f;
    return last_valid_angle;
}

// ==================== 三相电压驱动（Park⁻¹ → Clarke⁻¹ → SVPWM） ====================
static void driveMotor(float angle_deg, float torque) {
    // 1. 机械角度 → 电角度（弧度），并归一化到 [0, 2π)
    float angle_el = angle_deg * PI / 180.0f;
    angle_el = fmodf(angle_el, 2.0f * PI);
    if (angle_el < 0.0f) angle_el += 2.0f * PI;

    // 2. 力矩 → Uq 电压（Id=0 控制策略：所有电压用于产生转矩）
    // 取负号：保证正输出→正转，负输出→反转，与 PID 方向约定一致
    float Uq = -torque * (VOLTAGE_POWER_SUPPLY / 255.0f);

    // 3. Park 逆变换：Uq,Ud → Uα,Uβ
    // 注意：这里使用非标准符号约定（Ubeta 取负），匹配原始 SPWM 驱动中
    //       Vα∝sin(θe), Vβ∝-cos(θe) 的电压矢量方向，确保力矩方向一致
    float Ualpha = Uq * sinf(angle_el);
    float Ubeta  = -Uq * cosf(angle_el);

    // 4. Clarke 逆变换 + 中线偏置（SVPWM 等效）
    float Ua = Ualpha + VOLTAGE_POWER_SUPPLY / 2.0f;
    float Ub = (sqrtf(3.0f) * Ubeta - Ualpha) / 2.0f + VOLTAGE_POWER_SUPPLY / 2.0f;
    float Uc = (-Ualpha - sqrtf(3.0f) * Ubeta) / 2.0f + VOLTAGE_POWER_SUPPLY / 2.0f;

    // 5. 电压 → PWM 占空比（0-255，8-bit 分辨率）
    uint32_t pwm1 = (uint32_t)_constrain(Ua / VOLTAGE_POWER_SUPPLY * 255.0f, 0.0f, 255.0f);
    uint32_t pwm2 = (uint32_t)_constrain(Ub / VOLTAGE_POWER_SUPPLY * 255.0f, 0.0f, 255.0f);
    uint32_t pwm3 = (uint32_t)_constrain(Uc / VOLTAGE_POWER_SUPPLY * 255.0f, 0.0f, 255.0f);

    ledc_set_duty(LEDC_MODE, LEDC_CH_IN1, pwm1);
    ledc_update_duty(LEDC_MODE, LEDC_CH_IN1);
    ledc_set_duty(LEDC_MODE, LEDC_CH_IN2, pwm2);
    ledc_update_duty(LEDC_MODE, LEDC_CH_IN2);
    ledc_set_duty(LEDC_MODE, LEDC_CH_IN3, pwm3);
    ledc_update_duty(LEDC_MODE, LEDC_CH_IN3);
}

// ==================== 串口命令接收 ====================
#define CMD_BUF_SIZE 32

static float parseSerialAngle(void) {
    static char cmd_buf[CMD_BUF_SIZE];
    static int cmd_idx = 0;

    uint8_t ch;
    bool has_new = false;

    while (1) {
        int rx = uart_read_bytes(UART_PORT, &ch, 1, pdMS_TO_TICKS(1));
        if (rx != 1) break;

        if (ch == '\n' || ch == '\r') {
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';
                has_new = true;
            }
            cmd_idx = 0;
            if (has_new) break;
        } else {
            if (cmd_idx < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_idx++] = ch;
            }
        }
    }

    if (has_new && cmd_buf[0] != '\0') {
        float val = atof(cmd_buf);
        uart_printf("目标角度：%.1f\r\n", val);
        return val;
    }
    return target_angle;  // 无新值，返回原值
}

// ==================== FreeRTOS 控制任务 (~100Hz) ====================
static void angle_control_task(void *arg) {
    float integral = 0.0f;
    float last_error = 0.0f;
    int64_t last_print_us = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = 1;  // 1 tick = 10ms @ 100Hz

    int cycle_count = 0;  // 周期计数器，用于节流调试输出

    while (1) {
        // 读取并归一化当前角度
        current_angle = readAS5600Angle();
        current_angle = normalizeAngle(current_angle);

        // 串口接收目标角度
        float new_target = parseSerialAngle();
        target_angle = new_target;

        // 计算角度误差
        float error = target_angle - current_angle;
        if (error > 180.0f)  error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        // PID 控制 & 电机驱动
        if (fabs(error) < DEAD_ZONE) {
            // 死区内：积分缓慢衰减，施加小比例保持力矩（避免零电流导致的漂移振荡）
            integral *= 0.95f;  // 每周期衰减 5%，而非直接清零
            float hold_output = KP * error + KI * integral + KD * (error - last_error);
            hold_output = _constrain(hold_output, -15.0f, 15.0f);  // 限制保持力矩
            float electrical_angle = -current_angle * POLE_PAIRS + PHASE_OFFSET;
            driveMotor(electrical_angle, hold_output);
        } else {
            integral += error;
            integral = _constrain(integral, -200.0f, 200.0f);

            float derivative = error - last_error;
            last_error = error;

            float output = KP * error + KI * integral + KD * derivative;
            output = _constrain(output, -MOVE_PWM, MOVE_PWM);

            float electrical_angle = -current_angle * POLE_PAIRS + PHASE_OFFSET;
            driveMotor(electrical_angle, output);

            // 调试输出：节流输出（避免阻塞控制循环）
            if (DEBUG_PRINT_PERIOD > 0 && (cycle_count % DEBUG_PRINT_PERIOD == 0)) {
                uart_printf("电角度：%.1f, output: %.1f\r\n", electrical_angle, output);
            }
        }

        // 状态指示
        const char* direction;
        if (fabs(error) < DEAD_ZONE) {
            direction = "【已锁定·保持力矩】";
        } else if (error > 0) {
            direction = "正转";
        } else {
            direction = "反转";
        }

        // 串口输出（100ms 刷新）
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_print_us >= 100000) {
            uart_printf("当前：%.1f  目标：%.1f  误差：%.1f  状态：%s\r\n",
                        current_angle, target_angle, error, direction);
            last_print_us = now_us;
        }

        cycle_count++;
        vTaskDelayUntil(&last_wake, period);
    }
}

// ==================== GPIO 初始化 ====================
static esp_err_t gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(EN_GPIO, 1);  // 使能驱动
    return ESP_OK;
}

// ==================== LEDC PWM 初始化 (20kHz, 8-bit) ====================
static esp_err_t ledc_pwm_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };

    ch_cfg.channel = LEDC_CH_IN1;
    ch_cfg.gpio_num = IN1_GPIO;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    ch_cfg.channel = LEDC_CH_IN2;
    ch_cfg.gpio_num = IN2_GPIO;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    ch_cfg.channel = LEDC_CH_IN3;
    ch_cfg.gpio_num = IN3_GPIO;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    return ESP_OK;
}

// ==================== I2C 初始化 (AS5600) ====================
static esp_err_t i2c_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL_GPIO,
        .sda_io_num = SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = 100000,  // 100kHz
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &as5600_handle));

    ESP_LOGI(TAG, "AS5600 I2C 初始化完成 (addr=0x%02X)", AS5600_ADDR);
    return ESP_OK;
}

// ==================== UART 初始化 ====================
static esp_err_t uart_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(UART_PORT, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "UART driver install failed: %d", ret);
        return ret;
    }
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    return ESP_OK;
}

// ==================== 主初始化函数 ====================
esp_err_t motor_angle_control_init(void) {
    ESP_LOGI(TAG, "初始化 GPIO ...");
    gpio_init();

    ESP_LOGI(TAG, "初始化 LEDC PWM (20kHz, 8-bit) ...");
    ledc_pwm_init();

    ESP_LOGI(TAG, "初始化 I2C (AS5600) ...");
    i2c_init();

    ESP_LOGI(TAG, "初始化 UART ...");
    uart_init();

    uart_println("DRV8313 + AS5600 无刷电机控制系统已启动");

    BaseType_t ret = xTaskCreate(angle_control_task, "angle_ctrl", 4096, NULL, 10, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "角度控制任务创建失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "角度闭环控制初始化完成");
    return ESP_OK;
}
