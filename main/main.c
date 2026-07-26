#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_open_control.h"
#include "motor_angle_control.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "电机控制系统启动");

    // esp_err_t ret = motor_open_control_init();
    esp_err_t ret = motor_angle_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化失败: %d", ret);
        return;
    }

    // 所有任务已由 motor_open_control_init 创建
    // app_main 只需保持运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
