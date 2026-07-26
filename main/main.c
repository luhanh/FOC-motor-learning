/**
 * @file    main.c
 * @brief   ESP32 BLDC 电机 FOC 控制 — 纯入口
 *
 * main.c 不涉及任何电机控制耦合代码：
 *   - 不引用 bldc_motor.h / AS5600.h / foc.h 等底层头文件
 *   - 不构造任何硬件配置结构体
 *   - 不进行任何传感器回调组装
 *   - 仅调用 motor_control 模块提供的接口
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_control.h"

static const char *TAG = "main";

/* ==================== 控制任务 ==================== */

static void foc_control_task(void *arg) {
    motor_control_t *mc = (motor_control_t *)arg;

    ESP_LOGI(TAG, "FOC 控制任务启动");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);  /* 1kHz (需 FreeRTOS tick=1000Hz) */

    while (1) {
        motor_control_step(mc);
        vTaskDelayUntil(&last_wake_time, period);
    }
}

/* ==================== 主函数 ==================== */

void app_main(void) {
    ESP_LOGI(TAG, "ESP32 BLDC FOC 控制系统启动");

    /* 1. 一站式初始化（硬件 + 算法 + 传感器全部内部完成） */
    motor_control_t *mc = NULL;
    ESP_ERROR_CHECK(motor_control_init(&mc));

    /* 2. 配置控制模式 */
    motor_control_set_control_type(mc, MC_CTRL_TORQUE);
    motor_control_set_torque_type(mc, MC_TORQUE_VOLTAGE);
    motor_control_set_target(mc, 1.0f);

    /* 3. 校准（失败时重试，避免电机未上电直接 crash） */
    esp_err_t ret = motor_control_calibrate(mc);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "校准失败 (0x%x)，请检查电机供电与接线，3s 后重试...", ret);
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }

    /* 4. 启动控制任务 */
    xTaskCreate(foc_control_task, "foc_ctrl", 4096, mc, 5, NULL);

    ESP_LOGI(TAG, "系统就绪");
}
