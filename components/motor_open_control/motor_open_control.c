#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "motor_open_control.h"

static const char *TAG = "motor_open";

// ====================== 【引脚定义】 ======================
#define IN1_GPIO        10       // 三相PWM输出1
#define IN2_GPIO        11       // 三相PWM输出2
#define IN3_GPIO        12       // 三相PWM输出3
#define ENABLE_GPIO     4        // 驱动使能脚（HIGH=使能）

// ====================== 【LEDC PWM 配置】 ======================
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_IN1     LEDC_CHANNEL_0
#define LEDC_CH_IN2     LEDC_CHANNEL_1
#define LEDC_CH_IN3     LEDC_CHANNEL_2
#define LEDC_FREQ_HZ    30000

// ====================== 【UART 配置】 ======================
#define UART_PORT       UART_NUM_0
#define UART_BAUD       115200
#define UART_RX_BUF     256
#define UART_TX_BUF     0

// ====================== 【硬件参数】 ======================
static float voltage_power_supply = 12.0;  // 电源电压：12V
static int pole_pairs = 7;                 // 电机极对数：7

// ====================== 【速度与电压配置 - 核心参数】 ======================
static const float Uq_min_divider = 1.8;    // 最小电压分压（启动扭矩）
static const float Uq_max_divider = 16.0;   // 最大电压分压（高速限制）

static const float speed_low = 10.0;        // 最低启动速度，低于此无有效扭矩
static const float speed_high = 200.0;      // 最高速度上限

// ====================== 【运行控制参数】 ======================
static float current_speed = 0;             // 当前实际电速度（内部计算）
static float target_speed = 30;             // 默认目标速度（上电初始）
static float speed_accel = 80.0;            // 加减速速率

static bool motor_enabled = true;           // 电机使能状态
static float shaft_angle = 0;               // 转子角度
static int64_t open_loop_timestamp = 0;     // 时间戳（微秒）
static float zero_electric_angle = 0.2;     // 电机零电角度

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

// ====================== 【数学工具函数】 ======================
static float _normalizeAngle(float angle) {
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

static float _electricalAngle(float shaft_angle, int pole_pairs) {
    return shaft_angle * pole_pairs;
}

// ====================== 【PWM 输出】 ======================
static void setPwm(float Ua, float Ub, float Uc) {
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
    uart_printf("pwm1: %d, pwm2: %d, pwm3: %d\r\n", (uint32_t)(dc_a * 255), (uint32_t)(dc_b * 255), (uint32_t)(dc_c * 255));

    ledc_set_duty(LEDC_MODE, LEDC_CH_IN1, (uint32_t)(dc_a * 255));
    ledc_update_duty(LEDC_MODE, LEDC_CH_IN1);
    ledc_set_duty(LEDC_MODE, LEDC_CH_IN2, (uint32_t)(dc_b * 255));
    ledc_update_duty(LEDC_MODE, LEDC_CH_IN2);
    ledc_set_duty(LEDC_MODE, LEDC_CH_IN3, (uint32_t)(dc_c * 255));
    ledc_update_duty(LEDC_MODE, LEDC_CH_IN3);
}

// ====================== 【三相电压生成】 ======================
static void setPhaseVoltage(float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el + zero_electric_angle);

    float Ualpha = Uq * sin(angle_el);
    float Ubeta  = Uq * cos(angle_el);

    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
    setPwm(Ua, Ub, Uc);
}

// ====================== 【速度→扭矩映射】 ======================
static float computeUqBySpeed(float velocity) {
    float abs_speed = fabs(velocity);
    float t;
    if (abs_speed <= speed_low)
        t = 0;
    else if (abs_speed >= speed_high)
        t = 1;
    else
        t = (abs_speed - speed_low) / (speed_high - speed_low);

    float Uq_min = voltage_power_supply / Uq_max_divider;
    float Uq_max = voltage_power_supply / Uq_min_divider;
    return Uq_min + t * (Uq_max - Uq_min);
}

// ====================== 【开环运动】 ======================
static float velocityOpenloop(float velocity) {
    if (!motor_enabled) {
        setPhaseVoltage(0, 0, 0);
        return 0;
    }

    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - open_loop_timestamp) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    shaft_angle = _normalizeAngle(shaft_angle + velocity * Ts);
    float Uq = computeUqBySpeed(velocity);

    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));
    open_loop_timestamp = now_us;
    return Uq;
}

// ====================== 【软启动】 ======================
static void softStart(float final_Uq_divider, float duration_sec, float start_speed) {
    uart_println("=== 软启动中 ===");
    float start_Uq = 0;
    float target_Uq = voltage_power_supply / final_Uq_divider;
    int64_t start_us = esp_timer_get_time();
    int64_t duration_us = (int64_t)(duration_sec * 1000000);
    int64_t last_us = start_us;

    while (esp_timer_get_time() - start_us < duration_us) {
        int64_t now_us = esp_timer_get_time();
        float Ts = (now_us - last_us) * 1e-6f;
        if (Ts <= 0 || Ts > 0.1f) Ts = 1e-3f;

        shaft_angle = _normalizeAngle(shaft_angle + start_speed * Ts);
        float t = (float)(now_us - start_us) / (float)duration_us;
        float Uq = start_Uq + t * (target_Uq - start_Uq);

        setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));
        last_us = now_us;
        esp_rom_delay_us(100);
    }
    uart_println("=== 软启动完成 ===");
}

// ====================== 【平滑加减速】 ======================
static void updateSpeedRamp(void) {
    static int64_t last_time = 0;
    int64_t now = esp_timer_get_time();
    if (last_time == 0) {
        last_time = now;
        return;
    }

    float dt = (now - last_time) * 1e-6f;
    if (dt > 0.01f) dt = 0.01f;
    if (dt <= 0) return;

    if (current_speed < target_speed) {
        current_speed += speed_accel * dt;
        if (current_speed > target_speed) current_speed = target_speed;
    } else if (current_speed > target_speed) {
        current_speed -= speed_accel * dt;
        if (current_speed < target_speed) current_speed = target_speed;
    }
    last_time = now;
}

// ====================== 【串口命令解析】 ======================
// 支持命令：
// speed xxx    → 设置速度（正=正转，负=反转）
// stop         → 停止并断电
// start        → 重新启动
#define CMD_BUF_SIZE 64

static void handleSerialCommands(void) {
    static char cmd_buf[CMD_BUF_SIZE];
    static int cmd_idx = 0;

    uint8_t ch;
    while (1) {
        int rx = uart_read_bytes(UART_PORT, &ch, 1, pdMS_TO_TICKS(1));
        if (rx != 1) break;

        if (ch == '\n' || ch == '\r') {
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';
                uart_printf("[收到命令] %s\r\n", cmd_buf);

                if (strncmp(cmd_buf, "speed", 5) == 0) {
                    char *p = cmd_buf + 5;
                    while (*p == ' ') p++;
                    float sp = atof(p);
                    target_speed = sp;

                    uart_printf("[设置成功] 速度 = %.1f\r\n", sp);
                    if (sp > 0)
                        uart_println("方向：正转");
                    else if (sp < 0)
                        uart_println("方向：反转");
                    else
                        uart_println("已停机");
                } else if (strcmp(cmd_buf, "stop") == 0) {
                    motor_enabled = false;
                    setPhaseVoltage(0, 0, 0);
                    current_speed = 0;
                    target_speed = 0;
                    uart_println("[已停止]");
                } else if (strcmp(cmd_buf, "start") == 0) {
                    motor_enabled = true;
                    uart_println("[已启动]");
                } else {
                    uart_println("[未知命令]");
                }
                cmd_idx = 0;
            }
        } else {
            if (cmd_idx < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_idx++] = ch;
            }
        }
    }
}

// ====================== 【FreeRTOS 任务】 ======================

// 电机控制任务 (~100Hz，1 tick = 10ms @ 100Hz FreeRTOS tick)
static void motor_control_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = 1;  // 1 tick = 10ms
    while (1) {
        updateSpeedRamp();
        velocityOpenloop(current_speed);
        vTaskDelayUntil(&last_wake, period);
    }
}

// 串口命令任务 (~50Hz)
static void uart_cmd_task(void *arg) {
    while (1) {
        handleSerialCommands();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ====================== 【GPIO 初始化】 ======================
static esp_err_t gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(ENABLE_GPIO, 1);  // 使能驱动
    return ESP_OK;
}

// ====================== 【LEDC PWM 初始化】 ======================
static esp_err_t ledc_pwm_init(void) {
    // 配置定时器：30kHz, 8-bit 分辨率
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 配置三个输出通道
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

// ====================== 【UART 初始化】 ======================
static esp_err_t uart_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 尝试安装驱动（可能已被系统控制台占用）
    esp_err_t ret = uart_driver_install(UART_PORT, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "UART driver install failed: %d", ret);
        return ret;
    }
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    return ESP_OK;
}

// ====================== 【主初始化函数 - 对外接口】 ======================
esp_err_t motor_open_control_init(void) {
    ESP_LOGI(TAG, "初始化 GPIO ...");
    gpio_init();

    ESP_LOGI(TAG, "初始化 LEDC PWM (30kHz, 8-bit) ...");
    ledc_pwm_init();

    ESP_LOGI(TAG, "初始化 UART ...");
    uart_init();

    // 软启动
    float ss_speed = target_speed >= 0 ? 10.0f : -10.0f;
    softStart(Uq_min_divider, 2, ss_speed);
    current_speed = ss_speed;

    // 启动信息
    uart_println("==========================");
    uart_println(" 正反转正常：正=正转，负=反转");
    uart_println(" 最高电速度：±200  →  ≈1900 转/分钟");
    uart_println(" 最低有效速度：±10 → 无力矩");
    uart_println(" speed 200 / speed -200 / stop / start");
    uart_println("==========================");

    // 创建 FreeRTOS 任务
    BaseType_t ret;
    ret = xTaskCreate(motor_control_task, "motor_ctrl", 4096, NULL, 10, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "电机控制任务创建失败");
        return ESP_FAIL;
    }

    ret = xTaskCreate(uart_cmd_task, "uart_cmd", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "串口命令任务创建失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "开环控制初始化完成");
    return ESP_OK;
}
