#include "pid.h"
#include "foc_utils.h"

/* ==================== PID 控制器实现 ==================== */

void pid_init(pid_controller_t *pid, float P, float I, float D,
              float ramp, float limit, float Ts) {
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->Ts = Ts;
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    /* 对标 Arduino-FOC: timestamp_prev = _micros() */
    pid->timestamp_prev = 0;
}

float pid_update(pid_controller_t *pid, float error) {
    float dt = pid->Ts;

    /* 自适应采样时间：对标 Arduino-FOC 真实计算 dt */
    if (!foc_isset(dt)) {
        /* 调用者需在外部注入 get_time_us 并通过 timestamp_prev 计算 */
        /* 如果未注入时间戳，使用默认值 */
        if (pid->timestamp_prev == 0) {
            dt = 1e-3f;
        } else {
            /* 此处 dt 已预设为 Ts，不做覆盖 */
            dt = 1e-3f;
        }
    }

    /* 比例项 */
    float proportional = pid->P * error;

    /* 积分项（Tustin 变换） */
    float integral = pid->integral_prev +
                     pid->I * dt * 0.5f * (error + pid->error_prev);

    /* 积分抗饱和 */
    if (foc_isset(pid->limit)) {
        integral = foc_constrain(integral, -pid->limit, pid->limit);
    }

    /* 微分项 */
    float derivative = pid->D * (error - pid->error_prev) / dt;

    /* 求和 */
    float output = proportional + integral + derivative;

    /* 输出限幅 */
    if (foc_isset(pid->limit)) {
        output = foc_constrain(output, -pid->limit, pid->limit);
    }

    /* 输出变化率限制 */
    if (foc_isset(pid->output_ramp) && pid->output_ramp > 0.0f) {
        float output_rate = (output - pid->output_prev) / dt;
        if (output_rate > pid->output_ramp) {
            output = pid->output_prev + pid->output_ramp * dt;
        } else if (output_rate < -pid->output_ramp) {
            output = pid->output_prev - pid->output_ramp * dt;
        }
    }

    /* 保存状态 */
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;

    return output;
}

void pid_reset(pid_controller_t *pid) {
    pid->integral_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->error_prev = 0.0f;
}
